import math
import os
import time
from typing import List, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Header, String
from gb_visual_detection_3d_msgs.msg import BoundingBoxes3d, BoundingBox3d
from ament_index_python.packages import get_package_share_directory
import yaml
import tf2_ros
from tf2_ros import TransformException


def quaternion_from_yaw(yaw: float) -> Quaternion:
    """Create a geometry_msgs/Quaternion from a yaw angle (Z-axis rotation)."""
    half_yaw = yaw * 0.5
    cy = math.cos(half_yaw)
    sy = math.sin(half_yaw)
    # roll = pitch = 0, so x = y = 0
    return Quaternion(x=0.0, y=0.0, z=sy, w=cy)


def yaw_from_quaternion(q: Quaternion) -> float:
    """Extract yaw angle from a geometry_msgs/Quaternion."""
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw


class MissionState:
    IDLE = "IDLE"
    NAV_TO_TRUCK_STANDOFF = "NAV_TO_TRUCK_STANDOFF"
    WAIT_TRUCK_BOX = "WAIT_TRUCK_BOX"
    TURN_IN_PLACE_TRUCK = "TURN_IN_PLACE_TRUCK"
    APPROACH_TRUCK = "APPROACH_TRUCK"
    WAIT_WHEEL_BOX = "WAIT_WHEEL_BOX"
    TURN_IN_PLACE_WHEEL = "TURN_IN_PLACE_WHEEL"
    INSPECT_WHEEL = "INSPECT_WHEEL"
    NEXT_TRUCK = "NEXT_TRUCK"
    DONE = "DONE"


class TruckInspectionManager(Node):
    """Simple finite-state mission manager for truck inspection."""

    def __init__(self):
        super().__init__("inspection_manager")

        self.declare_parameter("trucks_file", "")
        self.declare_parameter("standoff_distance", 2.0)
        self.declare_parameter("approach_offset", 0.5)
        self.declare_parameter("wheel_offset", 0.4)
        self.declare_parameter("truck_label", "truck")
        self.declare_parameter("wheel_label", "wheel")
        self.declare_parameter("detection_topic", "/detections_3d")
        self.declare_parameter("world_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("detection_timeout", 5.0)  # seconds to wait before recovery
        self.declare_parameter("rotation_angle", 0.785)  # 45 degrees in radians
        self.declare_parameter("max_rotation_attempts", 8)  # 8 * 45 = 360 degrees
        self.declare_parameter("rotation_position_offset", 0.1)  # Small forward offset to force Nav2 execution
        self.declare_parameter("segmentation_mode_topic", "/segmentation_mode")  # Topic to control segmentation model
        self.declare_parameter("wheel_position_tolerance", 0.5)  # meters - wheels closer than this are considered the same
        self.declare_parameter("max_wheel_distance_from_truck", 5.0)  # meters - wheels beyond this are from other trucks

        self.trucks: List[dict] = self._load_trucks()
        self.current_truck_idx = 0
        self.current_state = MissionState.IDLE
        self.pending_goal_handle = None
        self.current_truck_box: Optional[BoundingBox3d] = None
        self.current_wheel_idx = 0
        
        # Wheel tracking: store positions of inspected wheels to avoid duplicates
        self.inspected_wheel_positions: List[tuple] = []  # List of (x, y, z) tuples
        
        # Recovery state tracking
        self.wait_start_time: Optional[float] = None
        self.rotation_attempts = 0
        self.initial_wait_yaw: Optional[float] = None  # Store yaw when starting to wait

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        detection_topic = self.get_parameter("detection_topic").value
        self.detection_sub = self.create_subscription(
            BoundingBoxes3d, detection_topic, self._detection_cb, qos_profile=qos
        )

        # Publish current FSM state for debugging.
        self.state_pub = self.create_publisher(String, "inspection_state", 10)
        
        # Publish segmentation mode to control which model the segmentation node uses
        segmentation_mode_topic = self.get_parameter("segmentation_mode_topic").value
        self.segmentation_mode_pub = self.create_publisher(String, segmentation_mode_topic, 10)
        self.get_logger().info(f"Publishing segmentation mode to: {segmentation_mode_topic}")

        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        
        # TF buffer and listener for getting robot pose
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1.0, self._tick)
        self.get_logger().info(f"Loaded {len(self.trucks)} trucks.")

        # Publish initial state.
        self._set_state(MissionState.IDLE)

    # ----------------------- State Helpers ----------------------- #
    def _set_state(self, new_state: str):
        """Update internal state, log transition, and publish for debugging."""
        if new_state == self.current_state:
            return
        self.get_logger().info(f"State transition: {self.current_state} -> {new_state}")
        self.current_state = new_state
        msg = String()
        msg.data = self.current_state
        self.state_pub.publish(msg)
        
        # Publish segmentation mode based on state
        # Use navigation model (Model 1) when waiting for truck detection
        if new_state == MissionState.WAIT_TRUCK_BOX:
            mode_msg = String()
            mode_msg.data = "navigation"
            self.segmentation_mode_pub.publish(mode_msg)
            self.get_logger().info("Published segmentation mode: navigation (Model 1)")
        # Use inspection model (Model 2) when waiting for wheel detection
        elif new_state == MissionState.WAIT_WHEEL_BOX:
            mode_msg = String()
            mode_msg.data = "inspection"
            self.segmentation_mode_pub.publish(mode_msg)
            self.get_logger().info("Published segmentation mode: inspection (Model 2)")
        
        # Reset wait timer and rotation tracking when entering wait states
        if new_state in [MissionState.WAIT_TRUCK_BOX, MissionState.WAIT_WHEEL_BOX]:
            self.wait_start_time = time.time()
            self.rotation_attempts = 0
            current_yaw = self._get_current_yaw()
            self.initial_wait_yaw = current_yaw
            if current_yaw is not None:
                self.get_logger().info(f"Starting wait state with initial yaw: {math.degrees(current_yaw):.2f}°")
        elif new_state not in [MissionState.TURN_IN_PLACE_TRUCK, MissionState.TURN_IN_PLACE_WHEEL]:
            self.wait_start_time = None
            # Don't reset rotation_attempts here - keep it for the next wait cycle

    def _load_trucks(self) -> List[dict]:
        trucks_file = self.get_parameter("trucks_file").value
        if not trucks_file:
            # Default to package share config path (works in install and from source)
            try:
                share_dir = get_package_share_directory("inspection_manager")
                trucks_file = os.path.join(share_dir, "config", "trucks.yaml")
            except Exception as exc:  # pragma: no cover - defensive
                self.get_logger().warn(
                    f"Failed to resolve package share directory: {exc}"
                )
                return []

        if not os.path.exists(trucks_file):
            self.get_logger().warn(f"No trucks file at {trucks_file}, starting empty.")
            return []

        with open(trucks_file, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        trucks = data.get("trucks", [])
        return trucks

    # ----------------------- State Machine ----------------------- #
    def _tick(self):
        if self.current_state == MissionState.IDLE:
            if not self.trucks:
                self.get_logger().warn("No trucks configured; mission done.")
                self._set_state(MissionState.DONE)
                return
            self.get_logger().info("Starting mission.")
            self._dispatch_standoff_goal()
            return

        if self.current_state == MissionState.NAV_TO_TRUCK_STANDOFF:
            return  # waiting on action feedback

        if self.current_state == MissionState.WAIT_TRUCK_BOX:
            # Check for timeout and trigger recovery if needed
            if self.wait_start_time is not None:
                elapsed = time.time() - self.wait_start_time
                timeout = self.get_parameter("detection_timeout").value
                if elapsed > timeout:
                    if self.rotation_attempts < self.get_parameter("max_rotation_attempts").value:
                        self.get_logger().warn(
                            f"No truck detection after {elapsed:.1f}s. Attempting recovery rotation {self.rotation_attempts + 1}."
                        )
                        self._dispatch_rotation_goal(is_truck=True)
                    else:
                        self.get_logger().error(
                            f"Max rotation attempts reached. Skipping truck {self.current_truck_idx + 1}."
                        )
                        self._set_state(MissionState.NEXT_TRUCK)
            return  # waiting on detections

        if self.current_state == MissionState.TURN_IN_PLACE_TRUCK:
            return  # waiting on rotation to complete

        if self.current_state == MissionState.APPROACH_TRUCK:
            return

        if self.current_state == MissionState.WAIT_WHEEL_BOX:
            # Check for timeout and trigger recovery if needed
            if self.wait_start_time is not None:
                elapsed = time.time() - self.wait_start_time
                timeout = self.get_parameter("detection_timeout").value
                if elapsed > timeout:
                    if self.rotation_attempts < self.get_parameter("max_rotation_attempts").value:
                        self.get_logger().warn(
                            f"No un-inspected wheel detection after {elapsed:.1f}s. "
                            f"Attempting recovery rotation {self.rotation_attempts + 1}. "
                            f"Already inspected {len(self.inspected_wheel_positions)} wheels."
                        )
                        self._dispatch_rotation_goal(is_truck=False)
                    else:
                        # Check if we have enough wheels inspected
                        inspected_count = len(self.inspected_wheel_positions)
                        self.get_logger().warn(
                            f"Max rotation attempts reached. Inspected {inspected_count} wheels. "
                            f"Moving to next truck."
                        )
                        self._set_state(MissionState.NEXT_TRUCK)
            return

        if self.current_state == MissionState.TURN_IN_PLACE_WHEEL:
            return  # waiting on rotation to complete

        if self.current_state == MissionState.INSPECT_WHEEL:
            return

        if self.current_state == MissionState.NEXT_TRUCK:
            self.current_truck_idx += 1
            if self.current_truck_idx >= len(self.trucks):
                self.get_logger().info("All trucks inspected.")
                self._set_state(MissionState.DONE)
                return
            self.current_truck_box = None
            self.current_wheel_idx = 0
            self.inspected_wheel_positions = []  # Reset for next truck
            self.rotation_attempts = 0  # Reset rotation attempts for new truck
            self._dispatch_standoff_goal()
            return

    # ----------------------- Detections ----------------------- #
    def _detection_cb(self, msg: BoundingBoxes3d):
        if self.current_state == MissionState.WAIT_TRUCK_BOX:
            truck_box = self._find_box(msg.bounding_boxes, self.get_parameter("truck_label").value)
            if truck_box:
                # Log bounding box details
                self._log_bounding_box(truck_box, "TRUCK")
                self.current_truck_box = truck_box
                self.get_logger().info(
                    f"Truck box received ({truck_box.object_name}); approaching."
                )
                self._dispatch_box_goal(truck_box, offset=self.get_parameter("approach_offset").value)
                self._set_state(MissionState.APPROACH_TRUCK)
            return

        if self.current_state == MissionState.WAIT_WHEEL_BOX:
            wheel_box = self._find_wheel_for_inspection(msg.bounding_boxes)
            if wheel_box:
                # Log bounding box details
                wheel_num = len(self.inspected_wheel_positions) + 1
                self._log_bounding_box(wheel_box, f"WHEEL_{wheel_num}")
                wheel_center = (
                    (wheel_box.xmin + wheel_box.xmax) / 2.0,
                    (wheel_box.ymin + wheel_box.ymax) / 2.0,
                    (wheel_box.zmin + wheel_box.zmax) / 2.0
                )
                self.get_logger().info(
                    f"Wheel {wheel_num} detected at ({wheel_center[0]:.2f}, {wheel_center[1]:.2f}, {wheel_center[2]:.2f}); "
                    f"moving to inspect. Already inspected: {len(self.inspected_wheel_positions)}"
                )
                # Store wheel position BEFORE navigating to avoid detecting it again while navigating
                self.inspected_wheel_positions.append(wheel_center)
                self._dispatch_box_goal(wheel_box, offset=self.get_parameter("wheel_offset").value)
                self._set_state(MissionState.INSPECT_WHEEL)
            else:
                # Log why no wheel was selected
                wheel_label = self.get_parameter("wheel_label").value
                all_wheels = [b for b in msg.bounding_boxes if b.object_name.lower() == wheel_label.lower()]
                if all_wheels:
                    self.get_logger().debug(
                        f"Found {len(all_wheels)} wheel(s) but none selected. "
                        f"Already inspected: {len(self.inspected_wheel_positions)}"
                    )
            return

    def _find_box(self, boxes: List[BoundingBox3d], label: str, index: Optional[int] = None) -> Optional[BoundingBox3d]:
        filtered = [b for b in boxes if b.object_name.lower() == label.lower()]
        if not filtered:
            return None
        if index is not None and index < len(filtered):
            return filtered[index]
        # otherwise pick the highest probability
        return sorted(filtered, key=lambda b: b.probability, reverse=True)[0]

    def _find_wheel_for_inspection(self, boxes: List[BoundingBox3d]) -> Optional[BoundingBox3d]:
        """Find a wheel that:
        1. Matches the wheel label
        2. Has not been inspected yet (not in inspected_wheel_positions)
        3. Belongs to the current truck (within reasonable distance)
        4. Returns the closest un-inspected wheel to the ROBOT's current position
        """
        wheel_label = self.get_parameter("wheel_label").value
        filtered = [b for b in boxes if b.object_name.lower() == wheel_label.lower()]
        
        if not filtered:
            return None
        
        # Get robot's current position (prefer this for selecting nearest wheel)
        robot_pose = self._get_current_pose()
        if robot_pose is None:
            self.get_logger().warn("Cannot get robot pose for wheel selection")
            # Fallback: just return first un-inspected wheel
            robot_pos = None
        else:
            robot_pos = (
                robot_pose.pose.position.x,
                robot_pose.pose.position.y,
                robot_pose.pose.position.z
            )
        
        # Get current truck position for filtering (only wheels near this truck)
        truck_pos = None
        max_wheel_distance_from_truck = self.get_parameter("max_wheel_distance_from_truck").value
        
        if self.current_truck_idx < len(self.trucks):
            if self.current_truck_box:
                # Use detected truck box center
                truck_pos = (
                    (self.current_truck_box.xmin + self.current_truck_box.xmax) / 2.0,
                    (self.current_truck_box.ymin + self.current_truck_box.ymax) / 2.0,
                    (self.current_truck_box.zmin + self.current_truck_box.zmax) / 2.0
                )
            else:
                # Fallback: use truck config position
                truck = self.trucks[self.current_truck_idx]
                truck_pos = (truck["x"], truck["y"], truck.get("z", 0.0))
        
        # Filter out already inspected wheels and wheels from other trucks
        un_inspected = []
        tolerance = self.get_parameter("wheel_position_tolerance").value
        
        for box in filtered:
            wheel_center = (
                (box.xmin + box.xmax) / 2.0,
                (box.ymin + box.ymax) / 2.0,
                (box.zmin + box.zmax) / 2.0
            )
            
            # Check if this wheel position was already inspected
            already_inspected = False
            for inspected_pos in self.inspected_wheel_positions:
                dist = math.sqrt(
                    (wheel_center[0] - inspected_pos[0])**2 +
                    (wheel_center[1] - inspected_pos[1])**2 +
                    (wheel_center[2] - inspected_pos[2])**2
                )
                if dist < tolerance:
                    already_inspected = True
                    break
            
            if already_inspected:
                continue
            
            # Filter by truck proximity if we have truck position
            if truck_pos:
                dist_to_truck = math.sqrt(
                    (wheel_center[0] - truck_pos[0])**2 +
                    (wheel_center[1] - truck_pos[1])**2 +
                    (wheel_center[2] - truck_pos[2])**2
                )
                if dist_to_truck > max_wheel_distance_from_truck:
                    self.get_logger().debug(
                        f"Wheel at ({wheel_center[0]:.2f}, {wheel_center[1]:.2f}) is "
                        f"{dist_to_truck:.2f}m from truck (max: {max_wheel_distance_from_truck}m) - skipping"
                    )
                    continue
            
            un_inspected.append(box)
        
        if not un_inspected:
            self.get_logger().debug(
                f"No un-inspected wheels found. Total wheels: {len(filtered)}, "
                f"Already inspected: {len(self.inspected_wheel_positions)}"
            )
            return None
        
        # Find wheel closest to ROBOT's current position (not truck position)
        # This ensures robot goes to nearest un-inspected wheel
        if robot_pos:
            def distance_to_robot(box: BoundingBox3d) -> float:
                wheel_center = (
                    (box.xmin + box.xmax) / 2.0,
                    (box.ymin + box.ymax) / 2.0,
                    (box.zmin + box.zmax) / 2.0
                )
                return math.sqrt(
                    (wheel_center[0] - robot_pos[0])**2 +
                    (wheel_center[1] - robot_pos[1])**2 +
                    (wheel_center[2] - robot_pos[2])**2
                )
            # Sort by distance to robot, then by probability
            closest_wheel = min(un_inspected, key=lambda b: (distance_to_robot(b), -b.probability))
        else:
            # Fallback: use highest probability
            closest_wheel = max(un_inspected, key=lambda b: b.probability)
        
        return closest_wheel

    def _log_bounding_box(self, box: BoundingBox3d, label: str):
        """Log detailed bounding box information."""
        center_x = (box.xmin + box.xmax) / 2.0
        center_y = (box.ymin + box.ymax) / 2.0
        center_z = (box.zmin + box.zmax) / 2.0
        width = box.xmax - box.xmin
        height = box.ymax - box.ymin
        depth = box.zmax - box.zmin
        
        self.get_logger().info(
            f"[{label}] Bounding Box Details:\n"
            f"  Object: {box.object_name}\n"
            f"  Probability: {box.probability:.3f}\n"
            f"  Center: ({center_x:.3f}, {center_y:.3f}, {center_z:.3f})\n"
            f"  Dimensions: W={width:.3f}, H={height:.3f}, D={depth:.3f}\n"
            f"  Bounds: X=[{box.xmin:.3f}, {box.xmax:.3f}], "
            f"Y=[{box.ymin:.3f}, {box.ymax:.3f}], "
            f"Z=[{box.zmin:.3f}, {box.zmax:.3f}]"
        )

    # ----------------------- Goals ----------------------- #
    def _dispatch_standoff_goal(self):
        if self.current_truck_idx >= len(self.trucks):
            return
        truck = self.trucks[self.current_truck_idx]
        standoff = self.get_parameter("standoff_distance").value
        yaw = truck.get("yaw", 0.0)
        goal_pose = self._pose_from_truck(truck, standoff, yaw)
        self.get_logger().info(
            f"Truck {self.current_truck_idx+1}/{len(self.trucks)}: navigating to standoff."
        )
        self._send_nav_goal(goal_pose, self._on_standoff_done)
        self._set_state(MissionState.NAV_TO_TRUCK_STANDOFF)

    def _dispatch_box_goal(self, box: BoundingBox3d, offset: float):
        center_x = (box.xmin + box.xmax) / 2.0
        center_y = (box.ymin + box.ymax) / 2.0
        center_z = (box.zmin + box.zmax) / 2.0

        # Approach from behind along x-axis of detection frame, offset backwards.
        heading = math.atan2(center_y, center_x)
        goal = PoseStamped()
        goal.header = Header()
        goal.header.frame_id = self.get_parameter("world_frame").value
        goal.pose.position.x = center_x - offset * math.cos(heading)
        goal.pose.position.y = center_y - offset * math.sin(heading)
        goal.pose.position.z = center_z
        goal.pose.orientation = quaternion_from_yaw(heading)

        self._send_nav_goal(goal, self._on_box_goal_done)

    def _send_nav_goal(self, pose: PoseStamped, done_cb):
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Nav2 action server not available.")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        self.pending_goal_handle = self.nav_client.send_goal_async(goal_msg)
        self.pending_goal_handle.add_done_callback(done_cb)

    def _get_current_yaw(self) -> Optional[float]:
        """Get current robot yaw from TF."""
        try:
            world_frame = self.get_parameter("world_frame").value
            base_frame = self.get_parameter("base_frame").value
            transform = self.tf_buffer.lookup_transform(
                world_frame, base_frame, rclpy.time.Time()
            )
            q = transform.transform.rotation
            return yaw_from_quaternion(q)
        except TransformException as ex:
            self.get_logger().warn(f"Could not get current pose: {ex}")
            return None

    def _get_current_pose(self) -> Optional[PoseStamped]:
        """Get current robot pose from TF."""
        try:
            world_frame = self.get_parameter("world_frame").value
            base_frame = self.get_parameter("base_frame").value
            transform = self.tf_buffer.lookup_transform(
                world_frame, base_frame, rclpy.time.Time()
            )
            pose = PoseStamped()
            pose.header.frame_id = world_frame
            pose.header.stamp = transform.header.stamp
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
            return pose
        except TransformException as ex:
            self.get_logger().warn(f"Could not get current pose: {ex}")
            return None

    def _dispatch_rotation_goal(self, is_truck: bool):
        """Dispatch a rotation goal to turn in place by rotation_angle.
        
        To ensure Nav2 actually executes the rotation, we add a small position offset
        in the direction of the new heading. This forces Nav2 to rotate to align
        with the goal orientation.
        """
        current_pose = self._get_current_pose()
        if current_pose is None:
            self.get_logger().error("Cannot get current pose for rotation. Aborting recovery.")
            if is_truck:
                self._set_state(MissionState.NEXT_TRUCK)
            else:
                self.current_wheel_idx += 1
                if self.current_wheel_idx >= 4:
                    self._set_state(MissionState.NEXT_TRUCK)
                else:
                    self._set_state(MissionState.WAIT_WHEEL_BOX)
            return

        current_yaw = yaw_from_quaternion(current_pose.pose.orientation)
        rotation_angle = self.get_parameter("rotation_angle").value
        new_yaw = current_yaw + rotation_angle
        
        # Normalize yaw to [-pi, pi]
        new_yaw = math.atan2(math.sin(new_yaw), math.cos(new_yaw))
        
        # Calculate actual yaw difference (accounting for wrap-around)
        yaw_diff = new_yaw - current_yaw
        # Normalize to [-pi, pi]
        if yaw_diff > math.pi:
            yaw_diff -= 2 * math.pi
        elif yaw_diff < -math.pi:
            yaw_diff += 2 * math.pi
        
        # Add a small position offset in the direction of the new heading
        # This forces Nav2 to actually execute the rotation instead of
        # thinking it's already at the goal (due to orientation tolerance)
        offset = self.get_parameter("rotation_position_offset").value
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = current_pose.header.frame_id
        goal_pose.header.stamp = self.get_clock().now().to_msg()  # Use current time
        # Add small forward offset along new heading to force rotation execution
        goal_pose.pose.position.x = current_pose.pose.position.x + offset * math.cos(new_yaw)
        goal_pose.pose.position.y = current_pose.pose.position.y + offset * math.sin(new_yaw)
        goal_pose.pose.position.z = current_pose.pose.position.z
        goal_pose.pose.orientation = quaternion_from_yaw(new_yaw)
        
        self.rotation_attempts += 1
        current_yaw_deg = math.degrees(current_yaw)
        new_yaw_deg = math.degrees(new_yaw)
        yaw_diff_deg = math.degrees(yaw_diff)
        self.get_logger().info(
            f"Rotation goal (attempt {self.rotation_attempts}):\n"
            f"  Current yaw: {current_yaw_deg:.2f}° ({current_yaw:.3f} rad)\n"
            f"  Target yaw: {new_yaw_deg:.2f}° ({new_yaw:.3f} rad)\n"
            f"  Yaw difference: {yaw_diff_deg:.2f}° ({yaw_diff:.3f} rad)\n"
            f"  Position offset: {offset:.3f}m\n"
            f"  Goal position: ({goal_pose.pose.position.x:.3f}, {goal_pose.pose.position.y:.3f})"
        )
        
        if is_truck:
            self._send_nav_goal(goal_pose, self._on_rotation_done_truck)
            self._set_state(MissionState.TURN_IN_PLACE_TRUCK)
        else:
            self._send_nav_goal(goal_pose, self._on_rotation_done_wheel)
            self._set_state(MissionState.TURN_IN_PLACE_WHEEL)

    # ----------------------- Callbacks ----------------------- #
    def _on_standoff_done(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Standoff goal rejected.")
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_standoff_result)

    def _on_standoff_result(self, future):
        status = future.result().status
        self.get_logger().info(f"Standoff result status: {status}")
        # Regardless of status we move to waiting for truck box; you may gate this on success.
        self._set_state(MissionState.WAIT_TRUCK_BOX)

    def _on_box_goal_done(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Box goal rejected.")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_box_result)

    def _on_box_result(self, future):
        status = future.result().status
        self.get_logger().info(f"Box goal result status: {status}")
        if self.current_state == MissionState.APPROACH_TRUCK:
            # Reset wheel tracking for this truck
            self.current_wheel_idx = 0
            self.inspected_wheel_positions = []
            self.rotation_attempts = 0  # Reset rotation attempts for wheel detection
            self._set_state(MissionState.WAIT_WHEEL_BOX)
            return

        if self.current_state == MissionState.INSPECT_WHEEL:
            # Check if we've inspected 4 distinct wheels
            if len(self.inspected_wheel_positions) >= 4:
                self.get_logger().info(f"Completed inspection of {len(self.inspected_wheel_positions)} wheels. Moving to next truck.")
                self._set_state(MissionState.NEXT_TRUCK)
            else:
                self._set_state(MissionState.WAIT_WHEEL_BOX)

    def _on_rotation_done_truck(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Rotation goal rejected.")
            self._set_state(MissionState.NEXT_TRUCK)
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_rotation_result_truck)

    def _on_rotation_result_truck(self, future):
        status = future.result().status
        self.get_logger().info(f"Rotation result status: {status}")
        # Reset wait timer to give more time after rotation
        self.wait_start_time = time.time()
        # Return to waiting for truck box
        self._set_state(MissionState.WAIT_TRUCK_BOX)

    def _on_rotation_done_wheel(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Rotation goal rejected.")
            # Check if we have enough wheels
            if len(self.inspected_wheel_positions) >= 4:
                self._set_state(MissionState.NEXT_TRUCK)
            else:
                self._set_state(MissionState.WAIT_WHEEL_BOX)
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_rotation_result_wheel)

    def _on_rotation_result_wheel(self, future):
        status = future.result().status
        self.get_logger().info(f"Rotation result status: {status}")
        # Reset wait timer to give more time after rotation
        self.wait_start_time = time.time()
        # Return to waiting for wheel box
        self._set_state(MissionState.WAIT_WHEEL_BOX)

    # ----------------------- Helpers ----------------------- #
    def _pose_from_truck(self, truck: dict, standoff: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header = Header()
        pose.header.frame_id = self.get_parameter("world_frame").value

        # Stand off along -x of truck heading.
        pose.pose.position.x = truck["x"] - standoff * math.cos(yaw)
        pose.pose.position.y = truck["y"] - standoff * math.sin(yaw)
        pose.pose.position.z = truck.get("z", 0.0)

        pose.pose.orientation = quaternion_from_yaw(yaw)
        return pose


def main(args=None):
    rclpy.init(args=args)
    node = TruckInspectionManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


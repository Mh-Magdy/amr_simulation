#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import ros2_numpy as rnp
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from segmentation_msgs.msg import ObjectsSegment, ObjectSegment
from ultralytics import YOLO
import cv2
import time

class UltralyticsSegmentationNode(Node):
    def __init__(self):
        super().__init__('ultralytics_segmentation')
        
        # Declare parameters for model paths
        self.declare_parameter("navigation_model", "yolov8m-seg.pt")  # Model 1 for navigation
        self.declare_parameter("inspection_model", "best.pt")  # Model 2 for inspection
        self.declare_parameter("mode_topic", "/segmentation_mode")
        self.declare_parameter("default_mode", "navigation")
        
        # Load both YOLO segmentation models
        nav_model_path = self.get_parameter("navigation_model").value
        insp_model_path = self.get_parameter("inspection_model").value
        
        self.get_logger().info(f"Loading navigation model: {nav_model_path}")
        self.navigation_model = YOLO(nav_model_path)  # Model 1 for navigation to trucks
        
        self.get_logger().info(f"Loading inspection model: {insp_model_path}")
        self.inspection_model = YOLO(insp_model_path)  # Model 2 for inspection under truck
        
        # Current active model (default to navigation)
        default_mode = self.get_parameter("default_mode").value
        if default_mode == "inspection":
            self.segmentation_model = self.inspection_model
            self.current_mode = "inspection"
        else:
            self.segmentation_model = self.navigation_model
            self.current_mode = "navigation"
        
        self.get_logger().info(f"Initial segmentation mode: {self.current_mode}")

        # Publishers
        self.objects_segment_pub = self.create_publisher(
            ObjectsSegment, 
            "/ultralytics/segmentation/objects_segment", 
            10
        )
        self.seg_image_pub = self.create_publisher(
            Image, 
            "/ultralytics/segmentation/image", 
            10
        )

        # Subscribers
        self.rgb_sub = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.callback,
            10
        )
        
        # Subscribe to mode topic to switch between models
        mode_topic = self.get_parameter("mode_topic").value
        self.mode_sub = self.create_subscription(
            String,
            mode_topic,
            self.mode_callback,
            10
        )
        self.get_logger().info(f"Subscribed to mode topic: {mode_topic}")

    def mode_callback(self, msg: String):
        """Callback to switch between navigation and inspection models."""
        mode = msg.data.lower().strip()
        
        if mode == "navigation":
            if self.current_mode != "navigation":
                self.get_logger().info("Switching to navigation mode (Model 1)")
                self.segmentation_model = self.navigation_model
                self.current_mode = "navigation"
        elif mode == "inspection":
            if self.current_mode != "inspection":
                self.get_logger().info("Switching to inspection mode (Model 2)")
                self.segmentation_model = self.inspection_model
                self.current_mode = "inspection"
        else:
            self.get_logger().warn(f"Unknown mode received: '{mode}'. Expected 'navigation' or 'inspection'")

    def callback(self, rgb_data):
        # Convert the RGB image to a NumPy array
        try:
            image = rnp.numpify(rgb_data)
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        height, width, _ = image.shape
        self.get_logger().info(f"Received image with shape: {image.shape}")

        # Apply segmentation model to the RGB image
        seg_result = self.segmentation_model(image)

        # Prepare the ObjectsSegment message
        objects_msg = ObjectsSegment()
        objects_msg.header = rgb_data.header  # Copy the header from the RGB image
        objects_msg.header.stamp = self.get_clock().now().to_msg()

        for index, cls in enumerate(seg_result[0].boxes.cls):
            class_index = int(cls.cpu().numpy())
            name = seg_result[0].names[class_index]

            # Ensure result.masks is not None
            if seg_result[0].masks is not None:
                mask = seg_result[0].masks.data.cpu().numpy()[index, :, :]
                mask_resized = cv2.resize(mask, (width, height))
                binary_mask = (mask_resized > 0.5).astype(np.uint8)

                # Get pixel indices for the mask
                y_indices, x_indices = np.where(binary_mask > 0)

                if len(x_indices) == 0 or len(y_indices) == 0:
                    self.get_logger().warn(f"No valid indices found for object: {name}")
                    continue

                # Create ObjectSegment message
                obj_msg = ObjectSegment()
                obj_msg.header = objects_msg.header
                obj_msg.class_name = name
                obj_msg.probability = float(seg_result[0].boxes.conf[index].item())  # Accessing the probability score
                obj_msg.x_indices = x_indices.tolist()
                obj_msg.y_indices = y_indices.tolist()

                # Append the object segment to the array
                objects_msg.objects.append(obj_msg)
            else:
                self.get_logger().warn("Segmentation result has no masks")

        # Publish the ObjectsSegment message
        if objects_msg.objects:
            self.get_logger().info(f"Publishing {len(objects_msg.objects)} segmented objects")
            self.objects_segment_pub.publish(objects_msg)

        # Segmentation Visualization
        if self.seg_image_pub.get_subscription_count() > 0:
            try:
                # Generate and publish the annotated segmentation image
                seg_annotated = seg_result[0].plot(show=False)
                self.seg_image_pub.publish(rnp.msgify(Image, seg_annotated, encoding="rgb8"))
                self.get_logger().info("Segmentation image published")
            except Exception as e:
                self.get_logger().error(f"Error while publishing segmentation image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = UltralyticsSegmentationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


## inspection_manager

Mission manager for automated truck inspection using Nav2 and 3D detections
(`gb_visual_detection_3d_msgs/BoundingBoxes3d`).

The node runs a **finite state machine (FSM)** that:

- **Reads** a list of truck poses from a YAML config.
- **Sends Nav2 goals** to a standoff pose near each truck.
- **Waits for 3D truck detections**, then sends an approach goal.
- **Loops over 4 wheels**, waiting for wheel detections and sending local
  inspection goals.
- **Repeats** for all trucks, then finishes.

---

## Node overview

- **Node name**: `inspection_manager`
- **Executable**: `inspection_manager_node`
- **Package**: `inspection_manager`

### Parameters

- **`trucks_file`** (string, default: *package share config*):
  - Path to YAML file with truck definitions.
  - If empty, the node automatically uses:
    - `$(ros2 pkg prefix inspection_manager)/share/inspection_manager/config/trucks.yaml`

- **`standoff_distance`** (double, default: `2.0`):
  - Distance [m] to stop in front of the truck before starting truck detection.

- **`approach_offset`** (double, default: `0.5`):
  - Offset [m] when sending the "approach truck" goal from the 3D box center.

- **`wheel_offset`** (double, default: `0.4`):
  - Offset [m] when sending wheel inspection goals from wheel 3D boxes.

- **`truck_label`** (string, default: `"truck"`):
  - `object_name` label expected in `BoundingBox3d` for truck detection.

- **`wheel_label`** (string, default: `"wheel"`):
  - `object_name` label expected in `BoundingBox3d` for wheel detection.

- **`detection_topic`** (string, default: `"/detections_3d"`):
  - Topic to subscribe for `gb_visual_detection_3d_msgs/BoundingBoxes3d`.

- **`world_frame`** (string, default: `"map"`):
  - Frame used for Nav2 goals.

### Truck config file

Default file: `config/trucks.yaml`

```yaml
trucks:
  - id: truck_1
    x: 5.0
    y: 2.0
    z: 0.0
    yaw: 0.0
  - id: truck_2
    x: 10.0
    y: -1.0
    z: 0.0
    yaw: 1.57
```

- `x, y, z` are truck positions in `world_frame` (default `map`).
- `yaw` is the truck heading (rad). The standoff pose is generated along
  **-x of the truck heading** at distance `standoff_distance`.

You can override this file at launch:

```bash
ros2 launch inspection_manager inspection_manager.launch.py \
  trucks_file:=/path/to/your_trucks.yaml
```

---

## Topics and actions

### Subscribed topics

- **`detection_topic`** (`gb_visual_detection_3d_msgs/BoundingBoxes3d`)
  - Default: `/detections_3d`
  - Used in states:
    - `WAIT_TRUCK_BOX` – waits for a truck box (`object_name == truck_label`).
    - `WAIT_WHEEL_BOX` – waits for wheel boxes (`object_name == wheel_label`).

### Action clients

- **`navigate_to_pose`** (`nav2_msgs/action/NavigateToPose`)
  - Used in:
    - `NAV_TO_TRUCK_STANDOFF`
    - `APPROACH_TRUCK`
    - `INSPECT_WHEEL`

You must have Nav2 running and accepting goals in the same `world_frame`.

---

## State machine

Implementation: `inspection_manager/inspection_manager_node.py`

### State list and transitions

- **`IDLE`**
  - **Entry condition**: Node just started.
  - **Behavior**:
    - Loads trucks from `trucks_file`.
    - If zero trucks: logs *"No trucks configured; mission done."* and goes to `DONE`.
    - Otherwise, dispatches first standoff Nav2 goal and goes to `NAV_TO_TRUCK_STANDOFF`.

- **`NAV_TO_TRUCK_STANDOFF`**
  - **What it does**:
    - Sends Nav2 goal to a standoff pose computed from the current truck:
      - Position: `x = truck.x - standoff_distance * cos(yaw)`
      - `y = truck.y - standoff_distance * sin(yaw)`
      - Orientation: facing truck (`yaw`).
  - **Transition**:
    - When Nav2 result is received (via action callback):
      - Logs result status, then goes to **`WAIT_TRUCK_BOX`**.
    - If rejected: logs error and stays in this state (you must debug Nav2).

- **`WAIT_TRUCK_BOX`**
  - **What it waits for**:
    - A `BoundingBoxes3d` message on `detection_topic` **containing a truck box**:
      - `object_name.lower() == truck_label.lower()`
  - **Transition**:
    - On first valid truck box:
      - Stores it as `current_truck_box`.
      - Computes approach pose from the box center with `approach_offset`.
      - Sends Nav2 goal to that pose.
      - Goes to **`APPROACH_TRUCK`**.
    - If no boxes: node stays in this state (keep debugging detections).

- **`APPROACH_TRUCK`**
  - **What it does**:
    - Waits for Nav2 result of the "approach truck" goal.
  - **Transition**:
    - On result:
      - Sets `current_wheel_idx = 0`.
      - Goes to **`WAIT_WHEEL_BOX`**.

- **`WAIT_WHEEL_BOX`**
  - **What it waits for**:
    - A `BoundingBoxes3d` message on `detection_topic` containing a wheel box:
      - `object_name.lower() == wheel_label.lower()`
      - If multiple wheels exist, it currently selects by:
        - Index `current_wheel_idx` if available, otherwise highest probability.
  - **Transition**:
    - When a wheel box is found:
      - Logs which wheel: `"Wheel {current_wheel_idx+1} detected; moving to inspect."`
      - Sends Nav2 goal with `wheel_offset` from that box.
      - Goes to **`INSPECT_WHEEL`**.

- **`INSPECT_WHEEL`**
  - **What it does**:
    - Waits for Nav2 result of the wheel inspection goal.
  - **Transition**:
    - On result:
      - `current_wheel_idx += 1`
      - If `current_wheel_idx >= 4`:
        - Goes to **`NEXT_TRUCK`**.
      - Else:
        - Goes back to **`WAIT_WHEEL_BOX`** for the next wheel.

- **`NEXT_TRUCK`**
  - **What it does**:
    - Increments `current_truck_idx`.
    - If there are more trucks:
      - Clears `current_truck_box` and `current_wheel_idx`.
      - Sends a new standoff goal for the next truck.
      - Goes to **`NAV_TO_TRUCK_STANDOFF`**.
    - If no more trucks:
      - Logs `"All trucks inspected."`
      - Goes to **`DONE`**.

- **`DONE`**
  - **Terminal state**: Mission complete. Node does not send any more goals.

---

## How to run

### 1. Build

```bash
cd ~/nav2_ws
colcon build --packages-select inspection_manager
source install/setup.bash
```

### 2. Launch with defaults

```bash
ros2 launch inspection_manager inspection_manager.launch.py
```

This uses:

- The default `config/trucks.yaml` from the package.
- The default detection topic `/detections_3d`.

### 3. Customize trucks file or detection topic

```bash
ros2 launch inspection_manager inspection_manager.launch.py \
  trucks_file:=/path/to/your_trucks.yaml \
  detection_topic:=/your_detections_topic
```

---

## Debugging tips

### 1. Check parameters and trucks file

```bash
ros2 param list /inspection_manager
ros2 param get /inspection_manager trucks_file
```

- If you see `Loaded 0 trucks.` in logs:
  - Make sure your YAML has a `trucks:` list.
  - Ensure the path in `trucks_file` is correct.

### 2. Verify Nav2 is running and accepting goals

- Check that `navigate_to_pose` action exists:

```bash
ros2 action list | grep navigate_to_pose
```

- Inspect feedback / status:

```bash
ros2 action info /navigate_to_pose
```

- If you see `Standoff result status: 4` (or other non-success code):
  - Status codes:
    - `0` – UNKNOWN
    - `1` – ACCEPTED
    - `2` – EXECUTING
    - `3` – CANCELING
    - `4` – SUCCEEDED
    - `5` – CANCELED
    - `6` – ABORTED
  - Use RViz2 to visualize the goal pose and the map.
  - Verify costmaps and robot footprint allow reaching the standoff / approach
    poses.

### 3. Check 3D detections

- Echo the detection topic:

```bash
ros2 topic echo /detections_3d
```

- Validate:
  - Messages of type `gb_visual_detection_3d_msgs/BoundingBoxes3d` are coming.
  - `object_name` for truck matches `truck_label` (e.g. `"truck"`).
  - `object_name` for wheels matches `wheel_label` (e.g. `"wheel"`).

- If the node is stuck in `WAIT_TRUCK_BOX`:
  - Probably no truck boxes are being published or labels do not match.

- If stuck in `WAIT_WHEEL_BOX`:
  - Wheel detections are missing or their labels differ from `wheel_label`.

### 4. Inspect current state via logs

- The FSM uses `get_logger().info` / `warn` messages for each major event:
  - Starting mission.
  - Sending standoff / box goals.
  - Receiving Nav2 result status.
  - When each truck / wheel is detected and inspected.

- To see logs with timestamps:

```bash
ros2 launch inspection_manager inspection_manager.launch.py \
  | sed -u 's/\x1b\[[0-9;]*m//g'
```

### 5. Visualize goals and poses in RViz2

- Ensure `world_frame` matches the Nav2 global frame (usually `map`).
- Add:
  - TF display to verify transforms.
  - Pose display subscribed to `/goal_pose` or use Nav2 tools to see goals.
  - PointCloud / markers for detections (you can add them later to this node).

---

## Extending the behavior

- Replace the simple "4 wheels by index" logic with:
  - Explicit wheel IDs (front-left, front-right, etc.) based on geometry.
  - Additional states for license plate alignment and capture.
- Add timeouts per state:
  - E.g. if `WAIT_TRUCK_BOX` exceeds N seconds, re-plan or abort.
- Publish diagnostics:
  - Current state as a topic (e.g. `std_msgs/String`).
  - Marker arrays for truck and wheel target poses for RViz debugging.





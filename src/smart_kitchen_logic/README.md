# 🍳 Smart Kitchen Logic

**ROS 2 logic layer for a the smart kitchen robot** — pick cups, drop them at the sink, and react to humans in the scene. Supports two operation modes: **fixed** (taught waypoints) and **dynamic** (ArUco-based detection + inverse kinematics at runtime).

---

## 🎯 What Is This?

`smart_kitchen_logic` implements the high-level behavior for a mobile manipulator in a kitchen: it runs **command sequences** (e.g. pick cup → move to sink → drop), drives the **arm** and **gripper**, and can either follow **pre-taught positions** or **dynamically** find cups via ArUco tags and solve Inverse Kinematics to grasp them.


| Mode        | Description                                                                                                          |
| ----------- | -------------------------------------------------------------------------------------------------------------------- |
| **Fixed**   | Uses a sequence of **fixed, taught positions** (waypoints). Ideal when cup locations are known and repeatable.       |
| **Dynamic** | **Detects cups** with ArUco tags on them and **solves inverse kinematics** at runtime to capture the cup on the fly. |


---

## ✨ Features

- **Two execution modes**: fixed waypoint sequences vs. dynamic ArUco-based picking  
- **High-level commands**: `pick_cup_1`, `drop_cup`, `safe_to_table`, `dynamic_scan`, etc., composed from waypoints  
- **Human-aware decisions**: decision points that check `/human_detection/left` and `/human_detection/right` and run alternative sequences when a person is present  
- **Emergency stop**: `/emergency_stop` (Bool) cancels in-flight trajectories  
- **Config-driven**: waypoints and commands are defined in TOML; no code change needed to add new positions or command sequences

---

## 🚀 How to Run

### Prerequisites


| Approach    | Prerequisites                                                                                                                  |
| ----------- | ------------------------------------------------------------------------------------------------------------------------------ |
| **Fixed**   | **Azure Kinect K4A** global camera mounted on the wall. Used by `human_detection_node` for RGB input.                          |
| **Dynamic** | **Vision module / RGB camera** on the robot gripper, running and publishing image + camera info, using the ArUco tags on cups. |


Ensure the robot (arm + joint trajectory controller, carriage, lift, gripper) and any cameras are running before starting the logic nodes.

---

### 🔷 Fixed (taught waypoints)

Uses pre-taught positions from `config/waypoints.toml` and a fixed command sequence (including human decision points).

**Run these in separate terminals:**

```bash
ros2 run smart_kitchen_logic human_detection_node
ros2 run smart_kitchen_logic robot_controller
ros2 run smart_kitchen_logic command_executor
ros2 run smart_kitchen_logic command_sequence_client
```

---

### 🔶 Dynamic (ArUco + IK at runtime)

Detects cups via ArUco tags, computes IK to reach the tag pose.

**Run these in separate terminals:**

```bash
ros2 run aruco_tracker aruco_all_distance
ros2 run smart_kitchen_logic robot_controller
ros2 run smart_kitchen_logic command_executor
ros2 run smart_kitchen_logic moving_node
ros2 run smart_kitchen_logic planning_node
```

- **Prerequisite**: Vision / RGB camera running (e.g. publishing `/camera/color/image_raw` and `/camera/color/camera_info`, or topics configured for `aruco_all_distance`).  
- **Flow**: `aruco_all_distance` publishes tag poses → `planning_node` consumes them, runs init + `dynamic_scan`, then for each detected tag computes IK and uses `moving_node` + `command_executor` to move to the cup and drop it.

---

## 📦 Nodes


| Node                                                 | Role                                                                                                                                                                                                                                                                                                                                                                   |
| ---------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `**robot_controller_node`** / `**robot_controller**` | Exposes the `**/move_to_joints**` action. Accepts joint-angle goals and forwards them to the joint trajectory controller. Handles **emergency stop** by cancelling the current trajectory. Used by both fixed and dynamic stacks.                                                                                                                                      |
| `**command_executor`**                               | **Action server** for `**/execute_command`**. Takes high-level command names (e.g. `pick_cup_1`, `drop_cup`), resolves them to waypoint sequences from config, and runs them (joint moves, carriage/lift, gripper). Used by both modes.                                                                                                                                |
| `**command_sequence_client**`                        | **Runs the fixed sequence**: sends a predefined list of commands (and decision points) to `/execute_command`. Subscribes to `**/human_detection/left`** and `**/human_detection/right**`; at decision points it chooses an alternative path if a human is detected. Only needed for the **fixed** flow.                                                                |
| `**human_detection_node`**                           | Consumes **RGB images** (e.g. from Azure Kinect), runs **YOLO** person detection, and publishes **Bool** on `**/human_detection/left`** and `**/human_detection/right**` depending on where the person is in the image. Used by the fixed flow for human-aware decisions.                                                                                              |
| `**planning_node**`                                  | **Dynamic mode brain**: subscribes to `**/aruco_distances`** (from `aruco_all_distance`), keeps a map of tag poses in `base_link`, runs **inverse kinematics** (ikpy) to get joint angles for a target pose, and orchestrates **move → grip → drop**. Calls `**/execute_command`** for init/dynamic_scan/drop_cup and sends joint/carriage goals to `**moving_node**`. |
| `**moving_node**`                                    | Receives **joint goals** (`/move_robot_goal`), **carriage** and **lift** goals, and executes them via the **FollowJointTrajectory** action and ELMO topics. Used only in **dynamic** mode.                                                                                                                                                                             |
| `**aruco_all_distance`** (from `aruco_tracker`)      | **Vision**: subscribes to RGB image + camera info, detects ArUco markers, and publishes `**/aruco_distances`** (e.g. marker id, distance, x, y, z in camera frame). Required for **dynamic** mode.                                                                                                                                                                     |


---

## ⚙️ Config

- `**config/waypoints.toml`**  
Defines named **waypoints**: joint angles, `time_from_start`, carriage/lift, optional `wait_after_sec`, and gripper open/close values. Referenced by commands.
- `**config/commands.toml`**  
Defines **commands** as lists of waypoint names (e.g. `pick_cup_1` → `pre_pick_1`, `pick_1`, `close_gripper`, `post_pick_1`). The command executor resolves these and runs the corresponding motions and gripper actions.

You can add new waypoints and commands without changing code; just extend the TOML files and use the same node binaries.

---

## 📌 Example Commands (from config)

Examples of what the **command executor** can run (names from `commands.toml`):

- `**init`** — home + open gripper  
- `**start_detecting**` — move to detection poses (table, sink, swipe)  
- `**pick_cup_1**`, `**pick_cup_2**`, `**pick_cup_3**` — pre-pick → pick → close gripper → post-pick  
- `**drop_cup**` — go to sink, open gripper, return  
- `**safe_to_table**` / `**safe_to_counter**` — safe paths via safety waypoints  
- `**dynamic_scan**` — scan path for dynamic cup detection

The **fixed** flow runs a sequence that includes these plus **decision points** (e.g. if human on right → pick cup 1; else → unsafe_to_table + pick_cup_2). The **dynamic** flow uses `**init`**, `**dynamic_scan**`, then per-tag **move → grip → drop_cup** via IK.

---

## 🛠 Build & Run

From your ROS 2 workspace that contains `smart_kitchen_logic`:

```bash
colcon build --packages-select smart_kitchen_logic
source install/setup.bash
```

Then start the nodes for either **fixed** or **dynamic** as above.
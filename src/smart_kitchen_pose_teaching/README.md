# 🎓 Smart Kitchen Pose Teaching

**Utilities for recording, replaying, and testing robot positions** — teach-in tooling kindly shared with us by **Group 3**.

---

## 🎯 What Is This?

`smart_kitchen_pose_teaching` provides a small set of CLI tools and an action server for capturing the robot's current joint + carriage + lift state into `positions.toml` and replaying named positions. This is how the waypoints used by `smart_kitchen_logic` are initially taught.

## ✨ Features

- **Record** the current arm + carriage + lift pose under a human-readable name with a single command
- **Replay** any named position by sending it directly to the joint trajectory controller
- **Drive carriage and lift** to a named position and wait for arrival confirmation
- **Test the gripper** at arbitrary open/close values to calibrate `gripper_open` / `gripper_close` constants

## 🚀 How to Run

```bash
colcon build --packages-select smart_kitchen_pose_teaching
source install/setup.bash
# Start main node
ros2 run smart_kitchen_pose_teaching pick_place_node

# Record the current robot pose as "pre_pick"
ros2 run smart_kitchen_pose_teaching record_position pre_pick

# Move the arm to a saved position
ros2 run smart_kitchen_pose_teaching goto_position home

# Drive carriage + lift to the values saved for a position
ros2 run smart_kitchen_pose_teaching goto_carriage_lift pre_pick
```

## 📦 Executables

| Executable | Description |
| ---------- | ----------- |
| `record_position <name>` | Reads `/joint_states`, `/elmo/id1/carriage/position/get`, and `/elmo/id1/lift/position/get`, then writes all values under `[name]` in `positions.toml`. |
| `goto_position <name>` | Sends the saved joint angles for a named position as a `JointTrajectory` to the arm controller and exits. |
| `goto_carriage_lift <name>` | Publishes the saved carriage/lift values to the ELMO `/set` topics and waits until positions are reached within tolerance (0.05). |
| `pick_place_node` | Action server (`/pick_place_node/pick_place`) that executes a full pick-and-place sequence using positions from `positions.toml`. |

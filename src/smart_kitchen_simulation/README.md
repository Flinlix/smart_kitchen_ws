# 🍽️ Smart Kitchen Simulation

**ROS 2 Gazebo simulation environment for the Smart Kitchen robot** — mock and helper nodes that replace real hardware drivers so the full logic stack can be developed and tested without physical hardware.

---

## 🎯 What Is This?

`smart_kitchen_simulation` provides five nodes that bridge the **Gazebo simulation** to the rest of the system. They expose the same ROS2 topics that real hardware would publish, so `smart_kitchen_logic` and all other packages run unchanged in simulation.

> **Coordinate system:** Gazebo and the robot use different conventions. The smart_kitchen origin sits at Gazebo world offset `(x=0.12, y=1.56, z=2.395)` and the **Z-axis is inverted**. All nodes that translate between frames apply this offset and inversion automatically

---

## ✨ Features

- **Simulated ELMO drives**: translates carriage and lift `/set` commands into `JointTrajectory` messages for the `rail_trajectory_controller` and echoes them back on `/get` topics
- **Visual debug marker**: teleports a Gazebo sphere between two configurable positions for quick visual sanity checks
- **TF2 position helpers**: `PositionTracker` class and standalone functions for computing the full end-effector world position (TF2 arm transform + carriage/lift offsets)
- Reads all positions of the environment from the model files defined in `smart_kitchen_models`

---

## 🚀 How to Run

In practice all simulation nodes are launched together with the rest of the system via `smart_kitchen.launch.py` in the `smart_kitchen_model` package. 
To build and run the simulation within gazebo, execute: 

```bash
cd smart_kitchen_ws
run_sim
```

This will open gazebo and run all necessary nodes in the background.

---

## 📦 Nodes

| Node | Role |
| ---- | ---- |
| `rail_simulator_node` | Replaces the real ELMO drive interface. Subscribes to `/elmo/id1/carriage/position/set` and `/elmo/id1/lift/position/set`, forwards commands to `rail_trajectory_controller` as `JointTrajectory` messages, and publishes position feedback on the corresponding `/get` topics at 10 Hz. The carriage sign is inverted internally to match the controller convention. (the movement of the carriage in simulation is not happening in the same speed as in the real kitchen, but reaches the same positions) |
| `cup_position_node` | Subscribes to `/model/<cup_name>/pose` for `cup_blue`, `cup_red`, `cup_blue_filled`, and `cup_red_filled`. Publishes all poses as `PoseArray` on `/cup_positions` (Gazebo world frame) and `/cup_positions_real` (robot coordinate frame, Z inverted + origin offset applied) at 10 Hz. |
| `visual_debug_node` | Debugging utility that cycles a purely-visual Gazebo sphere (`visual_marker`) between two hardcoded positions (`POSITION_A`, `POSITION_B`) every 2 seconds via the `gz service` CLI. Also listens to carriage, lift, and joint state topics so sensor state is available when extending the node. `POSITION_A`/`POSITION_B` default to `(0, 0, 0)` and must be set in source before use. |
| `tf_transformation` | Library module and standalone node providing helpers for computing the full gripper world position. Key utilities: `lookup_gripper_tf`, `lookup_base_link_tf`, `compute_full_position`, `lookup_aruco_in_base_link`. The `PositionTracker` class wires up TF2 + carriage/lift subscriptions on any existing node for convenient use: `x, y, z = tracker.get_full_position()`. When run directly, prints the full gripper position at 1 Hz. |
| `fake_human_node` | Publishes a synthetic human position on `/human_position` at 10 Hz for the `safety_monitor_node`. Defaults to the fixed URDF pose (`x=0.8`). If the Gazebo `static_human` model is present, the node overrides the fixed position with the live simulation pose from `/model/static_human/pose` (Unused at current stage of the project)|

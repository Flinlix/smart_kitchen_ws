# 🏗️ Smart Kitchen Model

**URDF, Gazebo world, meshes, models, and the main launch file for the Smart Kitchen robot.**

---

## 🎯 What Is This?

`smart_kitchen_model` is a pure asset and configuration package — no runtime nodes. It holds everything needed to spawn and describe the robot in simulation or on real hardware.

## ✨ Contents

| Path | Description |
| ---- | ----------- |
| `urdf/kitchen_robot.xacro` | Kinova Gen3 6-DOF arm + Robotiq 2F-140 gripper mounted on a linear rail (carriage + lift). Supports both simulation and real hardware via xacro arguments. |
| `urdf/static_human.urdf` | Static placeholder human model used by the safety monitor in simulation. |
| `worlds/smart_kitchen.world` | Gazebo world containing the kitchen scene. |
| `models/` | Gazebo SDF models: `cup_blue`, `cup_red`, `cup_blue_filled`, `cup_red_filled`, `kitchen`, `visual_marker`. |
| `config/sim_ros2_controllers.yaml` | ros2_control controller configuration for simulation (arm trajectory controller, rail trajectory controller, gripper). |
| `launch/smart_kitchen.launch.py` | Main launch file. Starts `robot_state_publisher`, Gazebo, ros2_control, all controllers, and optionally RViz. Key arguments: `sim_gazebo`, `robot_type`, `dof`, `vision`, `gripper`, `launch_rviz`. |

## 🚀 How to Run

```bash
colcon build --packages-select smart_kitchen_model
source install/setup.bash
ros2 launch smart_kitchen_model smart_kitchen.launch.py sim_gazebo:=true
```

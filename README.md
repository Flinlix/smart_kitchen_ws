# 🍳🦾 Smart Kitchen Workspace

**ROS 2 Jazzy workspace** for a smart kitchen robot: pick cups, drop them at the sink, and react to humans—in **simulation** (Gazebo) or on **real hardware**. 

*Developed by Group 4, "Mind the Bro, Bot!" for the Human-Robot-Interaction practical at Ludwig-Maximilian-University Munich, February 2026.*

---

## 📖 Short Description

This workspace contains everything needed to run a Kinova arm in a kitchen scenario: **fixed waypoint sequences** or **dynamic ArUco-based picking**, with optional human-aware decision points. You can develop and test in **Gazebo** using `smart_kitchen_simulation` and `smart_kitchen_model`, then run the same high-level logic on the **real robot** with `smart_kitchen_logic`.

---

## 🎬 Media (Videos & Pictures)

*Placeholder: add links or embeds to demos, setup photos, and screenshots here.*

<!--
### Demo video
[![Demo](optional-thumbnail-url)](optional-video-url)

### Setup / hardware
![Setup](docs/images/setup.jpg)

### Simulation (Gazebo)
![Gazebo](docs/images/gazebo.png)
-->

---

## 🏗️ Repository Structure Overview

```
smart_kitchen_ws/
├── src/
│   ├── smart_kitchen_interfaces/  # Message, service & action definitions
│   ├── smart_kitchen_model/       # Gazebo world, robot & object models
│   ├── smart_kitchen_simulation/  # Simulation-only mock & bridge nodes (Gazebo)
│   ├── smart_kitchen_logic/       # Main robot logic — real-world executor
│   └── aruco_tracker/             # ArUco marker detection & 3D pose (vision)
└── run_sim                        # Script to run the simulation
```

---

## 📦 What Each Package Is For

| Package | Role |
|--------|------|
| **`smart_kitchen_interfaces`** | **Message types & interfaces** — Custom actions (e.g. `/move_to_joints`, `/execute_command`), services, and messages shared across the project. |
| **`smart_kitchen_model`** | **Simulation assets** — Gazebo world, robot URDF/Xacro, SDF models (cups, kitchen, static human, visual marker). Used only when running in **Gazebo**. |
| **`smart_kitchen_simulation`** | **Simulation support** — Mock nodes for Gazebo: ELMO rail/lift simulator, fake human position, cup position export, TF helpers, visual debug. Use this **only in simulation**. |
| **`smart_kitchen_logic`** | **Main robot real-world executor** — High-level behavior: command sequences, arm/gripper control, human detection, fixed waypoints or dynamic ArUco + IK. Runs on the **real robot** (and can be tested in sim with the simulation packages). |
| **`aruco_tracker`** | **Vision / perception** — ArUco marker detection from a camera stream, 3D pose in camera frame, publishes distances and IDs. Used by **dynamic** mode in `smart_kitchen_logic` on real hardware. |

---

## 📚 Sub-package READMEs

- [**smart_kitchen_logic**](src/smart_kitchen_logic/README.md) — Main robot logic, fixed/dynamic modes, nodes, config, how to run on real hardware.
- [**smart_kitchen_simulation**](src/smart_kitchen_simulation/README.md) — Simulation nodes (rail simulator, fake human, cup positions, visual debug, TF helpers).
- [**aruco_tracker**](src/aruco_tracker/README.md) — ArUco detection, 3D pose, parameters and topic layout.
- [**smart_kitchen_model**](src/smart_kitchen_model/README.md) — Gazebo world, robot URDF/Xacro, SDF models (cups, kitchen, static human, visual marker). Used only when running in **Gazebo**.

---

## 🔀 Simulation vs Real World

| | **Simulation (Gazebo)** | **Real World** |
|---|-------------------------|----------------|
| **Packages** | `smart_kitchen_model` + `smart_kitchen_simulation` | `smart_kitchen_logic` (and optionally `aruco_tracker`) |
| **Purpose** | `smart_kitchen_model` provides the **Gazebo world, robot and object models**. `smart_kitchen_simulation` provides **mock and bridge nodes** (rail simulator, fake human, cup positions, TF helpers) so the rest of the stack can run inside Gazebo without real hardware. | `smart_kitchen_logic` is the **main executor** that talks to real arm, gripper, carriage, lift, and cameras. No Gazebo; real ELMO drives, real sensors. |
| **Launch** | Typically `smart_kitchen.launch.py` from `smart_kitchen_model` brings up Gazebo + simulation nodes. | Start `smart_kitchen_logic` nodes (and `aruco_tracker` if using dynamic mode) with real drivers already running. |

So: **simulation = `smart_kitchen_simulation` + `smart_kitchen_model`**; **real execution = `smart_kitchen_logic`** (and vision via `aruco_tracker` when needed).

---

## 🚀 Quick Start

**Simulation (Gazebo):**

```bash
cd smart_kitchen_ws
# Building will be done automatically when running the simulation
run_sim
```

**Real robot:**

```bash
cd smart_kitchen_ws
colcon build 
source install/setup.bash
# Start your robot drivers (arm, carriage, lift, gripper, camera), then e.g.:
ros2 run smart_kitchen_logic robot_controller
ros2 run smart_kitchen_logic command_executor
ros2 run smart_kitchen_logic command_sequence_client   # fixed mode
# Or for dynamic: aruco_tracker + planning_node + moving_node (see smart_kitchen_logic README)
```

For detailed run instructions, see the [smart_kitchen_logic README](src/smart_kitchen_logic/README.md).

---

## 📄 Credits

Alexander Feix, Felix Lindenmeier, Tamara Muras
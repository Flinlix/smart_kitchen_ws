# 🎯 aruco_tracker

> **Real-time ArUco marker detection & 3D pose estimation for ROS 2**

Detect ArUco markers from a camera stream, estimate their 3D position in the camera frame, and publish distances and a debug overlay—all in one lightweight ROS 2 node.

---

## 📖 What is it?

`aruco_tracker` is a ROS 2 (Jazzy) package that:

- **Subscribes** to an image topic and camera info
- **Detects** ArUco markers using OpenCV
- **Estimates** 3D pose (translation) per marker via `solvePnP`
- **Publishes** a flat array of `[id, euclidean_distance_m, x_m, y_m, z_m, ...]` and an optional annotated debug image

Positions are in **meters** in the **camera optical frame** (X right, Y down, Z forward). No TF or `base_link` conversion is done inside this package—you get raw camera-frame coordinates ready for your own transforms or logic.

---

## 🚀 How to run

### Build

From your workspace root:

```bash
cd ~/workspace/smart_kitchen_ws
colcon build --packages-select aruco_tracker
source install/setup.bash
```

### Run the node

**Default topics** (e.g. RealSense color camera):

```bash
ros2 run aruco_tracker aruco_all_distance
```

**With custom parameters:**

```bash
ros2 run aruco_tracker aruco_all_distance --ros-args \
  -p image_topic:=/camera/color/image_raw \
  -p camera_info_topic:=/camera/color/camera_info \
  -p marker_size_m:=0.05 \
  -p aruco_dictionary:=DICT_4X4_50 \
  -p distances_topic:=/aruco_distances \
  -p debug_image_topic:=/aruco_debug_image
```

Ensure your camera node is publishing `sensor_msgs/Image` and `sensor_msgs/CameraInfo` on the configured topics; the node will wait for the first `CameraInfo` before processing images.

---

## ⚙️ Parameters

| Parameter              | Type   | Default                      | Description                                      |
|------------------------|--------|------------------------------|--------------------------------------------------|
| `image_topic`          | string | `/camera/color/image_raw`    | Incoming camera images                           |
| `camera_info_topic`    | string | `/camera/color/camera_info`  | Camera intrinsics (K, D)                         |
| `marker_size_m`        | double | `0.05`                       | Marker side length in **meters** (critical for scale) |
| `aruco_dictionary`     | string | `DICT_4X4_50`                | ArUco dictionary (see below)                     |
| `distances_topic`      | string | `/aruco_distances`           | Output: `Float32MultiArray`                      |
| `debug_image_topic`    | string | `/aruco_debug_image`         | Output: annotated image                          |

### Supported ArUco dictionaries

- `DICT_4X4_50`, `DICT_4X4_100`
- `DICT_5X5_50`, `DICT_5X5_100`
- `DICT_6X6_50`, `DICT_6X6_100`
- `DICT_ARUCO_ORIGINAL`

---

## 📡 Topics

| Type                   | Topic (default)   | Description |
|------------------------|-------------------|-------------|
| **Subscribed**         |                   |             |
| `sensor_msgs/Image`    | `image_topic`     | Camera image |
| `sensor_msgs/CameraInfo` | `camera_info_topic` | Intrinsics |
| **Published**          |                   |             |
| `std_msgs/Float32MultiArray` | `distances_topic` | `[id0, dist0_m, x0, y0, z0, id1, ...]` |
| `sensor_msgs/Image`    | `debug_image_topic` | Markers + axes + labels drawn |

---

## ✨ Features

- **Multiple dictionaries** — Choose the ArUco family that matches your printed markers.
- **Configurable topics** — Plug into any image + camera_info source.
- **Metric 3D pose** — Uses `marker_size_m` and camera intrinsics for real-world scale.
- **Euclidean distance** — Per-marker distance (meters) included in the array.
- **Debug image** — Draws marker borders, IDs, coordinate axes, and distance/position labels.
- **Compatible OpenCV APIs** — Uses `ArucoDetector` when available, falls back to legacy `detectMarkers`.
- **Planar marker model** — `SOLVEPNP_IPPE_SQUARE` for stable pose on square markers.

---

## 📋 Message format: `distances_topic`

Each detected marker adds **5 floats**:

```
[id, euclidean_distance_m, x_m, y_m, z_m]
```

- **id** — ArUco marker ID (integer, as float).
- **euclidean_distance_m** — √(x² + y² + z²) in meters.
- **x_m, y_m, z_m** — Translation (tvec) in camera frame, meters.

Example for 2 markers (IDs 0 and 4):

```
[0.0, 1.2, 0.5, 0.3, 1.0,  4.0, 0.8, 0.2, -0.1, 0.75]
```

---

## 🖼️ Create your own markers

The package includes a small helper script to generate printable ArUco images. Run it from the package source (or adjust paths):

```bash
cd ~/workspace/smart_kitchen_ws/src/aruco_tracker/aruco_tracker
python3 aruco_creater.py
```

By default it creates `aruco_4.png` (ID 4, 600×600 px, `DICT_4X4_50`). Edit `aruco_creater.py` to change `marker_id`, `px_size`, or the dictionary, then print at the physical size you use for `marker_size_m`.

---

## 📦 Dependencies

- **ROS 2** (tested with Jazzy)
- **rclpy**, **tf2_ros**, **geometry_msgs**
- **aruco_opencv_msgs** (package dependency; node uses OpenCV + standard msgs)
- **cv_bridge**, **OpenCV** (with ArUco support)
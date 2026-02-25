#!/usr/bin/env python3
import math
from typing import Optional, List

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge

import cv2
import numpy as np


ARUCO_DICTS = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
}


class ArucoAllDistance(Node):
    """
    Subscribes:
      image_topic (sensor_msgs/Image)
      camera_info_topic (sensor_msgs/CameraInfo)

    Publishes:
      distances_topic (std_msgs/Float32MultiArray):
         [id0, euclid_m0, x_m0, y_m0, z_m0, id1, euclid_m1, x_m1, y_m1, z_m1, ...]
         where (x,y,z) is the tag position (tvec) in the camera frame, meters.
      debug_image_topic (sensor_msgs/Image): annotated image
    """

    def __init__(self):
        super().__init__("aruco_all_distance")

        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("marker_size_m", 0.05)  # marker side length in meters
        self.declare_parameter("aruco_dictionary", "DICT_4X4_50")
        self.declare_parameter("debug_image_topic", "/aruco_debug_image")
        self.declare_parameter("distances_topic", "/aruco_distances")

        self.image_topic = self.get_parameter("image_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.marker_size_m = float(self.get_parameter("marker_size_m").value)
        self.dict_name = self.get_parameter("aruco_dictionary").value
        self.debug_topic = self.get_parameter("debug_image_topic").value
        self.dist_topic = self.get_parameter("distances_topic").value

        if self.dict_name not in ARUCO_DICTS:
            raise RuntimeError(
                f"Unknown aruco_dictionary '{self.dict_name}'. Options: {list(ARUCO_DICTS.keys())}"
            )

        # OpenCV ArUco setup (support old and new detector APIs)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICTS[self.dict_name])
        self.detector_params = cv2.aruco.DetectorParameters()
        self.use_new_detector_api = hasattr(cv2.aruco, "ArucoDetector")
        self.aruco_detector = (
            cv2.aruco.ArucoDetector(self.aruco_dict, self.detector_params)
            if self.use_new_detector_api else None
        )

        # Camera intrinsics from CameraInfo
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        self.got_camera_info = False

        self.bridge = CvBridge()

        self.pub_debug = self.create_publisher(Image, self.debug_topic, 10)
        self.pub_dist = self.create_publisher(Float32MultiArray, self.dist_topic, 10)

        self.sub_info = self.create_subscription(CameraInfo, self.camera_info_topic, self.on_camera_info, 10)
        self.sub_img = self.create_subscription(Image, self.image_topic, self.on_image, 10)

        self.get_logger().info(
            f"image={self.image_topic}, camera_info={self.camera_info_topic}, "
            f"marker_size_m={self.marker_size_m}, dict={self.dict_name}"
        )
        self.get_logger().info(
            f"Publishing: {self.dist_topic} ([id, euclid_m, x_m, y_m, z_m, ...]) and {self.debug_topic} (debug image)"
        )

    def on_camera_info(self, msg: CameraInfo):
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        # D can be empty; default to zeros
        if len(msg.d) > 0:
            D = np.array(msg.d, dtype=np.float64).reshape(-1, 1)
        else:
            D = np.zeros((5, 1), dtype=np.float64)

        self.camera_matrix = K
        self.dist_coeffs = D
        self.got_camera_info = True

    def detect_markers(self, gray: np.ndarray):
        if self.use_new_detector_api:
            corners, ids, rejected = self.aruco_detector.detectMarkers(gray)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.detector_params
            )
        return corners, ids, rejected

    def marker_object_points(self, marker_size_m: float) -> np.ndarray:
        """
        3D points of the marker corners in marker coordinate frame.
        Marker centered at (0,0,0) on the Z=0 plane.
        Corner order matches OpenCV's detected corner order.
        """
        s = marker_size_m / 2.0
        # Order: top-left, top-right, bottom-right, bottom-left (OpenCV convention)
        return np.array([
            [-s,  s, 0.0],
            [ s,  s, 0.0],
            [ s, -s, 0.0],
            [-s, -s, 0.0],
        ], dtype=np.float64)

    def estimate_pose_solvepnp(
        self,
        corners: list,
        marker_size_m: float,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray
    ):
        """
        corners: list of (1,4,2) arrays from detectMarkers
        returns: rvecs, tvecs shaped like estimatePoseSingleMarkers outputs
        """
        objp = self.marker_object_points(marker_size_m)
        rvecs = []
        tvecs = []

        for c in corners:
            imgp = c.reshape(4, 2).astype(np.float64)

            ok, rvec, tvec = cv2.solvePnP(
                objp,
                imgp,
                camera_matrix,
                dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE  # good for planar square markers
            )
            if not ok:
                ok, rvec, tvec = cv2.solvePnP(
                    objp, imgp, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE
                )
            rvecs.append(rvec)
            tvecs.append(tvec)

        # Shapes: (N,1,3)
        rvecs = np.array(rvecs, dtype=np.float64).reshape(-1, 1, 3)
        tvecs = np.array(tvecs, dtype=np.float64).reshape(-1, 1, 3)
        return rvecs, tvecs

    def on_image(self, msg: Image):
        if not self.got_camera_info or self.camera_matrix is None or self.dist_coeffs is None:
            self.get_logger().warn_throttle(2.0, "Waiting for CameraInfo...")
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = self.detect_markers(gray)

        # If nothing detected: publish empty + raw image
        if ids is None or len(ids) == 0:
            out = Float32MultiArray()
            out.data = []
            self.pub_dist.publish(out)
            self.pub_debug.publish(self.bridge.cv2_to_imgmsg(frame, encoding="bgr8"))
            return

        ids = ids.flatten().astype(int)

        rvecs, tvecs = self.estimate_pose_solvepnp(
            corners,
            self.marker_size_m,
            self.camera_matrix,
            self.dist_coeffs
        )

        # Build output: [id, euclid, x, y, z, ...]
        data: List[float] = []

        # Draw marker borders + IDs
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        for i, marker_id in enumerate(ids):
            t = tvecs[i].reshape(3)  # meters, camera frame
            x, y, z = float(t[0]), float(t[1]), float(t[2])
            euclid = math.sqrt(x * x + y * y + z * z)

            data.extend([float(marker_id), euclid, x, y, z])

            # Draw axis
            cv2.drawFrameAxes(
                frame, self.camera_matrix, self.dist_coeffs,
                rvecs[i], tvecs[i],
                self.marker_size_m * 0.5
            )

            # Put label near first corner
            c = corners[i][0][0]
            label = f"id={marker_id} d={euclid:.2f}m x={x:.2f} y={y:.2f} z={z:.2f}"
            cv2.putText(
                frame, label, (int(c[0]), int(c[1]) - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2
            )

        out = Float32MultiArray()
        out.data = data
        self.pub_dist.publish(out)
        self.pub_debug.publish(self.bridge.cv2_to_imgmsg(frame, encoding="bgr8"))


def main():
    rclpy.init()
    node = ArucoAllDistance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
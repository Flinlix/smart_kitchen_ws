#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from aruco_opencv_msgs.msg import ArucoDetection, MarkerPose

from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException

# tf2_geometry_msgs in ROS2 wird über tf2_geometry_msgs importiert
from tf2_geometry_msgs import do_transform_pose


class HumanPoseFromAruco(Node):
    """
    Subscribes:
      /aruco_detections (aruco_opencv_msgs/ArucoDetection)

    Publishes:
      /aruco_pose (aruco_opencv_msgs/ArucoDetection) in target_frame (default: base_link)
                  Contains a single MarkerPose entry with the matched marker_id and transformed pose.
    NOTE: Publishes the marker_id and the pose of the aruco marker transformed to the base_link frame.
    Picks the marker with marker_id (default: 0). If not present -> publishes nothing.
    """

    def __init__(self):
        super().__init__("human_pose_from_aruco")

        # Parameters
        self.declare_parameter("detections_topic", "/aruco_detections")
        self.declare_parameter("marker_id", 0)
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("output_topic", "/aruco_pose")

        self.detections_topic = self.get_parameter("detections_topic").value
        self.marker_id = int(self.get_parameter("marker_id").value)
        self.target_frame = self.get_parameter("target_frame").value
        self.output_topic = self.get_parameter("output_topic").value

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Pub/Sub
        self.pub = self.create_publisher(ArucoDetection, self.output_topic, 10)
        self.sub = self.create_subscription(ArucoDetection, self.detections_topic, self.on_detections, 10)

        self.get_logger().info(
            f"Listening on {self.detections_topic}, marker_id={self.marker_id}, publishing {self.output_topic} in {self.target_frame}"
        )

    def on_detections(self, msg: ArucoDetection):
        # Find the requested marker
        chosen = None
        for m in msg.markers:
            # MarkerPose typically has fields: id, pose
            # If your field names differ, run:
            #   ros2 interface show aruco_opencv_msgs/msg/MarkerPose
            if int(getattr(m, "id")) == self.marker_id:
                chosen = m
                break

        if chosen is None:
            return

        # Build PoseStamped in camera frame (msg.header.frame_id)
        in_pose = PoseStamped()
        in_pose.header = msg.header  # has stamp + frame_id of the camera images
        in_pose.pose = getattr(chosen, "pose")

        # Transform to target_frame
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame,
                in_pose.header.frame_id,
                rclpy.time.Time()  # latest available
            )
            transformed_pose = do_transform_pose(in_pose.pose, tf).pose

            marker = MarkerPose()
            marker.id = self.marker_id
            marker.pose = transformed_pose

            out_detection = ArucoDetection()
            out_detection.header.stamp = in_pose.header.stamp
            out_detection.header.frame_id = self.target_frame
            out_detection.markers = [marker]

            self.pub.publish(out_detection)

        except TransformException as ex:
            self.get_logger().warn(f"TF transform failed ({in_pose.header.frame_id} -> {self.target_frame}): {ex}")


def main():
    rclpy.init()
    node = HumanPoseFromAruco()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
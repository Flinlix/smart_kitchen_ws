#!/usr/bin/env python3
"""Visual Marker Node – move a purely-visual Gazebo sphere via ROS2.

Subscribes:
  /visual_marker/goal_pose   (geometry_msgs/Pose)  – desired pose for the marker

Publishes:
  /visual_marker/pose        (geometry_msgs/Pose)  – current pose echoed back
                                                      (also published by Gazebo
                                                       PosePublisher on the same topic)

The node forwards every goal_pose to Gazebo's UserCommands set_pose service:
  /world/smart_kitchen/set_pose  (gz.msgs.Pose → gz.msgs.Boolean)

Usage example (move marker to x=0.5, y=0.3, z=1.2):
    ros2 topic pub --once /visual_marker/goal_pose geometry_msgs/msg/Pose \\
      "{position: {x: 0.5, y: 0.3, z: 1.2}, orientation: {w: 1.0}}"
"""

import subprocess
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

# Name of the Gazebo world (must match the <world name="..."> in the .world file)
WORLD_NAME = 'smart_kitchen'
# Name of the model inside Gazebo (must match <name> in the world <include>)
MODEL_NAME = 'visual_marker'


class VisualMarkerNode(Node):
    """ROS2 node that repositions a purely-visual Gazebo marker on demand."""

    def __init__(self) -> None:
        super().__init__('visual_marker_node')

        self._lock = threading.Lock()
        self._pending: Pose | None = None
        self._current: Pose = Pose()
        self._current.orientation.w = 1.0  # identity quaternion

        self.create_subscription(
            Pose,
            '/visual_marker/goal_pose',
            self._on_goal_pose,
            10,
        )

        self._pose_pub = self.create_publisher(Pose, '/visual_marker/pose', 10)

        # Process goals at ~10 Hz so rapid commands are coalesced
        self.create_timer(0.1, self._process_pending)

        self.get_logger().info(
            f'VisualMarkerNode started. '
            f'Publish to /visual_marker/goal_pose to move the marker.'
        )

    def _on_goal_pose(self, msg: Pose) -> None:
        with self._lock:
            self._pending = msg

    def _process_pending(self) -> None:
        with self._lock:
            if self._pending is None:
                return
            pose = self._pending
            self._pending = None

        self._set_gz_pose(pose)

    def _set_gz_pose(self, pose: Pose) -> None:
        """Call the Gazebo UserCommands set_pose service via gz CLI."""
        p = pose.position
        o = pose.orientation

        # Ensure quaternion is normalised
        if o.x == 0.0 and o.y == 0.0 and o.z == 0.0 and o.w == 0.0:
            o.w = 1.0

        # Protobuf text format expected by gz service
        req = (
            f'name: "{MODEL_NAME}" '
            f'position {{ x: {p.x} y: {p.y} z: {p.z} }} '
            f'orientation {{ x: {o.x} y: {o.y} z: {o.z} w: {o.w} }}'
        )

        cmd = [
            'gz', 'service',
            '-s', f'/world/{WORLD_NAME}/set_pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '500',
            '--req', req,
        ]

        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=1.0)
            if result.returncode == 0:
                self._current = pose
                self._pose_pub.publish(self._current)
                self.get_logger().debug(
                    f'Marker moved to ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})'
                )
            else:
                self.get_logger().warning(
                    f'set_pose failed (rc={result.returncode}): {result.stderr.strip()}'
                )
        except subprocess.TimeoutExpired:
            self.get_logger().warning('set_pose call timed out.')
        except FileNotFoundError:
            self.get_logger().error(
                "'gz' CLI not found. Make sure Gazebo is installed and sourced."
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisualMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

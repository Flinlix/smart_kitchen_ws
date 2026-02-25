#!/usr/bin/env python3
"""Visual Marker Node – cycles a purely-visual Gazebo sphere between two positions.

The marker jumps to POSITION_A, waits 2 seconds, then jumps to POSITION_B,
waits 2 seconds, and repeats indefinitely.

The node forwards each position to Gazebo's UserCommands set_pose service:
  /world/smart_kitchen/set_pose  (gz.msgs.Pose → gz.msgs.Boolean)
"""

import subprocess

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState

# Name of the Gazebo world (must match the <world name="..."> in the .world file)
WORLD_NAME = 'smart_kitchen'
# Name of the model inside Gazebo (must match <name> in the world <include>)
MODEL_NAME = 'visual_marker'

# TODO: set the desired positions (x, y, z in metres)

REAL_ORIGIN_IN_GZ = (0.12, 1.695, 2.395)  # smart_kitchen origin in Gazebo world coordinates

# (distance, carriage, lift)
POSITION_A = (0.5, 1.7, 0.96)
POSITION_B = (0.0, 0.0, 0.0)

# Seconds between each jump
JUMP_INTERVAL_SEC = 2.0


class VisualMarkerNode(Node):
    """ROS2 node that cycles a purely-visual Gazebo marker between two positions."""

    def __init__(self) -> None:
        super().__init__('visual_marker_node')

        self._at_a: bool = False  # toggles which position to jump to next

        # Sensor state
        self._carriage_position: float = 0.0
        self._lift_position: float = 0.0
        self._joint_states: JointState | None = None

        self.create_subscription(
            Float32, '/elmo/id1/carriage/position/get', self._on_carriage, 10)
        self.create_subscription(
            Float32, '/elmo/id1/lift/position/get', self._on_lift, 10)
        self.create_subscription(
            JointState, '/joint_states', self._on_joint_states, 10)

        self.create_timer(JUMP_INTERVAL_SEC, self._on_timer)
        

    def _on_carriage(self, msg: Float32) -> None:
        self._carriage_position = msg.data

    def _on_lift(self, msg: Float32) -> None:
        self._lift_position = msg.data

    def _on_joint_states(self, msg: JointState) -> None:
        self._joint_states = msg

    def _on_timer(self) -> None:
        self._at_a = not self._at_a
        x, y, z = POSITION_A if self._at_a else POSITION_B
        pose = Pose()
        # Convert from smart_kitchen coordinates to Gazebo world coordinates.
        pose.position.x = float(x) + REAL_ORIGIN_IN_GZ[0]
        pose.position.y = float(y) + REAL_ORIGIN_IN_GZ[1]
        pose.position.z = -float(z) + REAL_ORIGIN_IN_GZ[2] # Gazebo's z-axis is inverted compared to smart_kitchen's
        pose.orientation.w = 1.0
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
            pass
        except (subprocess.TimeoutExpired, FileNotFoundError):
            pass


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

#!/usr/bin/env python3
"""
Rail movement translator node:

Subscribes to   "/elmo/id1/carriage/position/set std_msgs/msg/Float32"
and             "/elmo/id1/lift/position/set std_msgs/msg/Float32"

then translates this input and publishes to

"/rail/x/position/set",
"/rail/y/position/set",
"/rail/z/position/set", and
"/rail/yaw/position/set" (std_msgs/msg/Float32)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


JOINT_NAMES = ['rail_x_joint', 'rail_y_joint', 'rail_z_joint', 'rail_yaw_joint']
CONTROLLER_TOPIC = '/rail_trajectory_controller/joint_trajectory'

# How long the controller should take to reach the commanded position (s)
MOVE_DURATION_SEC = 2


class FakeRailNode(Node):
    def __init__(self):
        super().__init__('fake_rail_movement_translator')

        # Publisher
        self._pub_x = self.create_publisher(Float32, '/rail/x/position/set', 10)

        # Subscribers for the input topics
        self.create_subscription(Float32, '/elmo/id1/carriage/position/set',
                                 lambda msg: self._on_set(0, msg), 10)

    def _on_set(self, axis: int, msg: Float32):
        """Receive a position command for one axis and publish a trajectory."""
        self._positions[axis] = msg.data

        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES

        pt = JointTrajectoryPoint()
        pt.positions = list(self._positions)
        pt.velocities = [0.0, 0.0, 0.0, 0.0]
        pt.time_from_start = Duration(sec=MOVE_DURATION_SEC, nanosec=0)

        traj.points.append(pt)
        self._pub.publish(traj)

        axis_name = ['x', 'y', 'z', 'yaw'][axis]
        unit = 'rad' if axis == 3 else 'm'
        self.get_logger().info(
            f'Rail command → {axis_name}={msg.data:.4f} {unit}  '
            f'(full: x={self._positions[0]:.4f}m, '
            f'y={self._positions[1]:.4f}m, '
            f'z={self._positions[2]:.4f}m, '
            f'yaw={self._positions[3]:.4f}rad)'
        )


def main(args=None):
    rclpy.init(args=args)
    node = FakeRailNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

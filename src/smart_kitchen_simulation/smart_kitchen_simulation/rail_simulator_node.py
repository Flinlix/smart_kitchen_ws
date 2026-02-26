#!/usr/bin/env python3
"""
Rail simulator node:

Subscribes to four Float32 topics (one per axis and yaw) and forwards the
commanded positions to the rail_trajectory_controller.

Topics (in):
  /elmo/id1/carriage/position/set  (std_msgs/Float32; Values are inverted)
  /elmo/id1/lift/position/set  (std_msgs/Float32)

  
  (optional) /rail/y/position/set  (std_msgs/Float32)
  (optional) /rail/yaw/position/set  (std_msgs/Float32)

Topics (out):
  /elmo/id1/carriage/position/get  (std_msgs/Float32; Published at 10 Hz, reflects last commanded value)
  /elmo/id1/lift/position/get      (std_msgs/Float32; Published at 10 Hz, reflects last commanded value)

The Float32 value is used directly as the joint position in metres.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


JOINT_NAMES = ['rail_x_joint', 'rail_y_joint', 'rail_z_joint', 'rail_yaw_joint']
CONTROLLER_TOPIC = '/rail_trajectory_controller/joint_trajectory'

# How long the controller should take to reach the commanded position (in seconds, for simplicity)
MOVE_DURATION_SEC = 2


class RailSimulatorNode(Node):
    def __init__(self):
        super().__init__('rail_simulator_node')

        # Current commanded positions (start at 0): x, y, z, yaw
        self._positions = [0.0, 0.0, 0.0, 0.0]

        # Publisher: rail_trajectory_controller
        self._pub = self.create_publisher(JointTrajectory, CONTROLLER_TOPIC, 10)

        # Publishers: current position feedback
        self._pub_carriage_get = self.create_publisher(Float32, '/elmo/id1/carriage/position/get', 10)
        self._pub_lift_get = self.create_publisher(Float32, '/elmo/id1/lift/position/get', 10)

        # One subscriber per axis
        self.create_subscription(Float32, '/elmo/id1/carriage/position/set',
                                 lambda msg: self._on_set(0, msg, scale=-1.0), 10)
        self.create_subscription(Float32, '/rail/y/position/set',
                                 lambda msg: self._on_set(1, msg), 10)
        self.create_subscription(Float32, '/elmo/id1/lift/position/set',
                                 lambda msg: self._on_set(2, msg), 10)
        self.create_subscription(Float32, '/rail/yaw/position/set',
                                 lambda msg: self._on_set(3, msg), 10)

        # Timer: publish /get topics at 10 Hz
        self.create_timer(0.1, self._publish_get_topics)

    def _on_set(self, axis: int, msg: Float32, scale: float = 1.0):
        """Receive a position command for one axis and publish a trajectory."""
        self._positions[axis] = msg.data * scale

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

    def _publish_get_topics(self):
        """Publish current carriage and lift positions at a fixed rate."""
        carriage_msg = Float32()
        carriage_msg.data = -self._positions[0]  # invert back to /set frame
        self._pub_carriage_get.publish(carriage_msg)

        lift_msg = Float32()
        lift_msg.data = self._positions[2]
        self._pub_lift_get.publish(lift_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RailSimulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

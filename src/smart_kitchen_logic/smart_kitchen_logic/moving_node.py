#!/usr/bin/env python3
"""Mock moving node: simulates robot movement to a target position.

Subscribes to /move_robot_goal (Pose), simulates travel, and publishes
True on /move_robot_result (Bool) when the movement is complete.
"""

import time
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool

SIMULATED_TRAVEL_SEC = 2.0


class MovingNode(Node):
    def __init__(self):
        super().__init__('moving_node')

        self.create_subscription(
            Pose, '/move_robot_goal', self._on_goal, 10)
        self._result_pub = self.create_publisher(
            Bool, '/move_robot_result', 10)

        self.get_logger().info(
            'Moving node started. Listening on /move_robot_goal ...')

    def _on_goal(self, msg: Pose) -> None:
        x, y, z = msg.position.x, msg.position.y, msg.position.z
        self.get_logger().info(
            f'Goal received: ({x:.3f}, {y:.3f}, {z:.3f})')
        threading.Thread(
            target=self._simulate_move, args=(x, y, z), daemon=True).start()

    def _simulate_move(self, x: float, y: float, z: float) -> None:
        self.get_logger().info(
            f'Simulating movement ({SIMULATED_TRAVEL_SEC}s) ...')
        time.sleep(SIMULATED_TRAVEL_SEC)
        self.get_logger().info(
            f'Arrived at ({x:.3f}, {y:.3f}, {z:.3f})')
        self._result_pub.publish(Bool(data=True))


def main(args=None):
    rclpy.init(args=args)
    node = MovingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

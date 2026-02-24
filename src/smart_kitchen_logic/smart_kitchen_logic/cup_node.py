#!/usr/bin/env python3
"""Listen to Gazebo pose topics for all cups and publish their positions as PoseArray."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray

CUP_NAMES = ['cup_blue', 'cup_red', 'cup_blue_filled', 'cup_red_filled']


class CupNode(Node):
    """Subscribes to Gazebo pose topics for each cup and publishes positions as PoseArray."""
    
    def __init__(self):
        super().__init__('cup_node')

        self._poses: dict[str, Pose] = {}

        for name in CUP_NAMES:
            self.create_subscription(
                Pose,
                f'/model/{name}/pose',
                lambda msg, n=name: self._on_pose(n, msg),
                10,
            )

        self._publisher = self.create_publisher(PoseArray, '/cup_positions', 10)
        self.create_timer(0.1, self._publish_positions)
        self.get_logger().info('Cup node started.')

    def _on_pose(self, name: str, msg: Pose) -> None:
        self._poses[name] = msg

    def _publish_positions(self) -> None:
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.poses = [self._poses[name] for name in CUP_NAMES if name in self._poses]
        self._publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CupNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
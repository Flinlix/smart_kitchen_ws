#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose


class FakeHumanNode(Node):
    """Publishes the human position for the safety monitor."""

    # The fixed position matches the URDF: world_to_human joint at xyz="0.8 0 0"
    FIXED_HUMAN_X = 0.8
    FIXED_HUMAN_Y = 0.0
    FIXED_HUMAN_Z = 0.0

    def __init__(self):
        super().__init__('fake_human_node')

        # Override position when a dynamic Gazebo model is present
        self.gazebo_sub = self.create_subscription(
            Pose,
            '/model/static_human/pose',
            self.gazebo_callback,
            10,
        )

        self.publisher_ = self.create_publisher(Point, '/human_position', 10)

        # Current position (defaults to the fixed URDF pose)
        self.human_point = Point(
            x=self.FIXED_HUMAN_X,
            y=self.FIXED_HUMAN_Y,
            z=self.FIXED_HUMAN_Z,
        )

        # Publish at 10 Hz so the safety monitor always has data
        self.create_timer(0.1, self.publish_position)

        self.get_logger().info('Fake Human Node started! Publishing human position at 10 Hz.')

    def gazebo_callback(self, msg):
        """Override the fixed position with live Gazebo data."""
        self.human_point.x = msg.position.x
        self.human_point.y = msg.position.y
        self.human_point.z = msg.position.z

    def publish_position(self):
        self.publisher_.publish(self.human_point)

def main(args=None):
    rclpy.init(args=args)
    node = FakeHumanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
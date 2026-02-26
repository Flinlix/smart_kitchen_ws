#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
import math

# TF2 for tracking the robot's end effector
from tf2_ros import Buffer, TransformListener

class SafetyMonitorNode(Node):
    def __init__(self):
        super().__init__('safety_monitor')

        self.human_subscription = self.create_subscription(
            Point, '/human_position', self.human_callback, 10)
        self.human_position = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self) # to get gripper position

        self.stop_publisher = self.create_publisher(Bool, '/emergency_stop', 10)
        
        self.monitor_timer = self.create_timer(0.1, self.check_distance)

        self.get_logger().info('Safety Monitor initialized. Watching for humans...')

    def human_callback(self, msg):
        self.human_position = msg

    def check_distance(self):
        if self.human_position is None:
            self.get_logger().warn('No position received')
            return

        try:
            trans = self.tf_buffer.lookup_transform(
                'world', 'end_effector_link', rclpy.time.Time())
            
            # robot position
            rx, ry, rz = trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z

            # human position (Basis)
            hx, hy, hz = self.human_position.x, self.human_position.y, self.human_position.z

            human_radius = 0.3 
            human_height = 1.8  # Meter
            safety_threshold = 0.5

            # distance to center (horizontally)
            dist_center_xy = math.sqrt((rx - hx)**2 + (ry - hy)**2)

            # substract radius from humans
            human_radius = 0.25
            surface_distance = dist_center_xy - human_radius

            safety_threshold = 0.5
            stop_msg = Bool()

            # If human is closer than 0.5 meters
            if surface_distance < safety_threshold:
                self.get_logger().warn(f'DANGER! Human is {surface_distance:.2f}m close!')
                stop_msg.data = True
                self.stop_publisher.publish(stop_msg)

            # Reset danger state once human is far away again
            elif surface_distance >= safety_threshold:
                self.get_logger().info('Area clear.')
                stop_msg.data = False
                self.stop_publisher.publish(stop_msg)

        except Exception as e:
            self.get_logger().debug(f'Exception while checking distance {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
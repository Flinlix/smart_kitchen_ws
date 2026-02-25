#!/usr/bin/env python3
"""Simple client node that executes a sequence of commands.

This node demonstrates how to call the command executor action server
with multiple commands in sequence.

Usage:
  ros2 run smart_kitchen_logic command_sequence_client
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from smart_kitchen_interfaces.action import ExecuteCommand


class CommandSequenceClient(Node):
    def __init__(self):
        super().__init__('command_sequence_client')
        
        self._action_client = ActionClient(
            self,
            ExecuteCommand,
            '/execute_command')
        
        # Define the sequence of commands to execute
        self._command_sequence = [
            {'command_name': 'home', 'cup_id': ''},
            {'command_name': 'start_detecting', 'cup_id': ''},
            {'command_name': 'full_pick_and_drop_1', 'cup_id': '1'},
            {'command_name': 'full_pick_and_drop_2', 'cup_id': '2'},
            {'command_name': 'home', 'cup_id': ''},
        ]
        
        self._current_index = 0
        
        self.get_logger().info('Command Sequence Client started')
        self.get_logger().info(f'Will execute {len(self._command_sequence)} commands')
        
        # Start the sequence
        self.create_timer(2.0, self._check_and_start)
    
    def _check_and_start(self):
        """Wait for action server, then start sequence."""
        if self._action_client.wait_for_server(timeout_sec=0.0):
            self.destroy_timer(self._check_and_start)
            self.get_logger().info('Action server available. Starting command sequence...')
            self._send_next_command()
        else:
            self.get_logger().info('Waiting for action server...')
    
    def _send_next_command(self):
        """Send the next command in the sequence."""
        if self._current_index >= len(self._command_sequence):
            self.get_logger().info('✓ All commands completed successfully!')
            rclpy.shutdown()
            return
        
        command_data = self._command_sequence[self._current_index]
        command_name = command_data['command_name']
        cup_id = command_data.get('cup_id', '')
        
        self.get_logger().info(
            f'\n=== Executing command {self._current_index + 1}/{len(self._command_sequence)}: '
            f'"{command_name}" (cup_id: {cup_id or "none"}) ===')
        
        # Create goal
        goal = ExecuteCommand.Goal()
        goal.command_name = command_name
        goal.cup_id = cup_id
        
        # Send goal
        send_future = self._action_client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback)
        send_future.add_done_callback(self._goal_response_callback)
    
    def _goal_response_callback(self, future):
        """Handle goal acceptance/rejection."""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('✗ Goal rejected!')
            rclpy.shutdown()
            return
        
        self.get_logger().info('Goal accepted, waiting for result...')
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)
    
    def _feedback_callback(self, feedback_msg):
        """Handle progress feedback."""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'  Progress: {feedback.progress} - Waypoint: {feedback.current_waypoint}')
    
    def _result_callback(self, future):
        """Handle command result and move to next command."""
        result = future.result().result
        
        if result.success:
            self.get_logger().info(f'✓ Command completed: {result.message}')
            self._current_index += 1
            # Wait a bit before next command
            self.create_timer(1.0, self._send_next_command, oneshot=True)
        else:
            self.get_logger().error(f'✗ Command failed: {result.message}')
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    
    node = CommandSequenceClient()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

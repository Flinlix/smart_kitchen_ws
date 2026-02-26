#!/usr/bin/env python3
"""Simple client node that executes a sequence of commands.

Supports decision points: subscribes to /human_pose/left and /human_pose/right.
Tracks where a human is (left/right) if present. At a decision point you can check:
- check_side 'left'  → only care about human on left; 5x no human left → continue
- check_side 'right' → only care about human on right; 5x no human right → continue
- check_side 'either' (default) → human on either side runs alternative; both clear 5x → continue
if_human_sequence is chosen if at least one True was received in the last 5 messages on the checked topic(s).
Decision points can be nested: if_human_sequence may contain further decision points.
Optional else_sequence: run when no human is detected (5x clear) before continuing.

Usage:
  ros2 run smart_kitchen_logic command_sequence_client
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Bool
from smart_kitchen_interfaces.action import ExecuteCommand
from collections import deque

CONSECUTIVE_NO_HUMAN_THRESHOLD = 5


class CommandSequenceClient(Node):
    def __init__(self):
        super().__init__('command_sequence_client')

        self._action_client = ActionClient(
            self,
            ExecuteCommand,
            '/execute_command')

        # Human pose state: where is a human (left/right) if present
        self._human_left = False
        self._human_right = False
        # Last 5 messages per topic: if_human_sequence when at least one True in last 5
        self._human_left_history = deque(maxlen=5)
        self._human_right_history = deque(maxlen=5)
        self._consecutive_no_human = 0       # both false (for check_side 'either')
        self._consecutive_no_human_left = 0  # left false (for check_side 'left')
        self._consecutive_no_human_right = 0 # right false (for check_side 'right')
        self.create_subscription(Bool, '/human_detection/left', self._human_left_cb, 10)
        self.create_subscription(Bool, '/human_detection/right', self._human_right_cb, 10)
        # Publish False on both human_pose topics every 2 seconds (keeps topics active)

        # Decision point handling
        self._decision_point_timer = None
        # Current execution context: we may be in main sequence or a nested if_human_sequence / else_sequence
        self._current_sequence = None  # set below after _command_sequence
        # Stack: list of (sequence, index, from_else). from_else=True means we ran else_sequence and should advance past DP when done.
        self._sequence_stack = []
        # Ensure only one command runs at a time: do not send next until current has finished
        self._command_in_flight = False

        # Sequence: normal entries or decision_point with optional check_side and if_human_sequence
        self._command_sequence = [
            {'command_name': 'init', 'cup_id': ''},
            {'command_name': 'start_detecting', 'cup_id': ''},
            {'decision_point': True, 'check_side': 'right', 'if_human_sequence': [
                {'command_name': 'pick_cup_1', 'cup_id': ''},
            ], 'else_sequence': [
                {'command_name': 'unsafe_to_table', 'cup_id': ''},
                {'command_name': 'pick_cup_2', 'cup_id': ''},
            ]},
            {'command_name': 'drop_cup', 'cup_id': ''},
            {'decision_point': True, 'check_side': 'right', 'if_human_sequence': [
                {'command_name': 'safe_to_table', 'cup_id': ''},
            ], 'else_sequence': [
                {'command_name': 'unsafe_to_table', 'cup_id': ''},
            ]},
            {'command_name': 'pick_cup_3', 'cup_id': ''},
            {'decision_point': True, 'check_side': 'right', 'if_human_sequence': [
                {'command_name': 'safe_to_counter', 'cup_id': ''},
            ], 'else_sequence': [
            ]},
            {'command_name': 'drop_cup', 'cup_id': ''},
        ]

        self._current_sequence = self._command_sequence
        self._current_index = 0

        self.get_logger().info('Command Sequence Client started')
        self.get_logger().info(f'Sequence has {len(self._command_sequence)} entries')
        self.get_logger().info('Subscribed to /human_pose/left and /human_pose/right')

        self.create_timer(2.0, self._check_and_start)

    def _human_left_cb(self, msg: Bool) -> None:
        self._human_left = msg.data
        self._human_left_history.append(msg.data)

    def _human_right_cb(self, msg: Bool) -> None:
        self._human_right = msg.data
        self._human_right_history.append(msg.data)

    def _update_consecutive_no_human(self) -> None:
        """Update all consecutive-no-human counters (called each decision-point tick)."""
        if self._human_left or self._human_right:
            self._consecutive_no_human = 0
        else:
            self._consecutive_no_human = min(
                self._consecutive_no_human + 1,
                CONSECUTIVE_NO_HUMAN_THRESHOLD)
        if self._human_left:
            self._consecutive_no_human_left = 0
        else:
            self._consecutive_no_human_left = min(
                self._consecutive_no_human_left + 1,
                CONSECUTIVE_NO_HUMAN_THRESHOLD)
        if self._human_right:
            self._consecutive_no_human_right = 0
        else:
            self._consecutive_no_human_right = min(
                self._consecutive_no_human_right + 1,
                CONSECUTIVE_NO_HUMAN_THRESHOLD)

    def _oneshot_timer(self, period_sec: float, callback) -> None:
        """Run callback once after period_sec (timer cancels itself after one shot)."""
        t = self.create_timer(period_sec, lambda: (t.cancel(), t.destroy(), callback()))

    def _check_and_start(self):
        """Wait for action server, then start sequence."""
        if self._action_client.wait_for_server(timeout_sec=0.0):
            self.destroy_timer(self._check_and_start)
            self.get_logger().info('Action server available. Starting command sequence...')
            self._send_next_command()
        else:
            self.get_logger().info('Waiting for action server...')

    def _send_next_command(self):
        """Send the next command in the current sequence (or handle decision point / end of nested)."""
        if self._command_in_flight:
            self.get_logger().warn('Ignoring _send_next_command: still waiting for current command to finish.')
            return
        if self._current_index >= len(self._current_sequence):
            # End of current sequence
            if not self._sequence_stack:
                self.get_logger().info('✓ All commands completed successfully!')
                rclpy.shutdown()
                return
            # Pop: return to caller (either if_human or else branch)
            seq, idx, from_else = self._sequence_stack.pop()
            self._current_sequence, self._current_index = seq, idx
            if from_else:
                self.get_logger().info('Else sequence done. Continuing past decision point.')
            else:
                self.get_logger().info('Human sequence done. Continuing past decision point.')
            self._current_index += 1
            self._oneshot_timer(0.1, self._send_next_command)
            return

        entry = self._current_sequence[self._current_index]
        if entry.get('decision_point'):
            check_side = entry.get('check_side', 'either')
            depth = len(self._sequence_stack)
            prefix = '  ' * depth + '[nested] ' if depth else ''
            self.get_logger().info(
                f'=== {prefix}Decision point (check_side={check_side}, need '
                f'{CONSECUTIVE_NO_HUMAN_THRESHOLD}x no human on that side) ===')
            self._start_decision_point_check()
            return

        command_data = entry
        command_name = command_data['command_name']
        cup_id = command_data.get('cup_id', '')
        depth = len(self._sequence_stack)
        prefix = '  ' * depth if depth else ''

        self.get_logger().info(
            f'\n=== {prefix}Executing command {self._current_index + 1}/{len(self._current_sequence)}: '
            f'"{command_name}" (cup_id: {cup_id or "none"}) ===')

        goal = ExecuteCommand.Goal()
        goal.command_name = command_name
        goal.cup_id = cup_id

        self._command_in_flight = True
        send_future = self._action_client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback)
        send_future.add_done_callback(self._goal_response_callback)

    def _start_decision_point_check(self) -> None:
        """Start periodic check at current decision point."""
        if self._decision_point_timer is not None:
            return
        self._decision_point_timer = self.create_timer(0.5, self._check_decision_point)

    def _stop_decision_point_check(self) -> None:
        """Stop the decision-point timer."""
        if self._decision_point_timer is not None:
            self._decision_point_timer.cancel()
            self._decision_point_timer.destroy()
            self._decision_point_timer = None

    def _check_decision_point(self) -> None:
        """At a decision point: continue if 5x no human on checked side, else run if_human_sequence.
        if_human_sequence is chosen when at least one True appears in the last 5 messages on the checked topic(s)."""
        self._update_consecutive_no_human()
        entry = self._current_sequence[self._current_index]
        check_side = entry.get('check_side', 'either')

        if check_side == 'left':
            clear = self._consecutive_no_human_left >= CONSECUTIVE_NO_HUMAN_THRESHOLD
            human_present = any(self._human_left_history)
        elif check_side == 'right':
            clear = self._consecutive_no_human_right >= CONSECUTIVE_NO_HUMAN_THRESHOLD
            human_present = any(self._human_right_history)
        else:
            clear = self._consecutive_no_human >= CONSECUTIVE_NO_HUMAN_THRESHOLD
            human_present = any(self._human_left_history) or any(self._human_right_history)

        if clear:
            self.get_logger().info(f'No human on {check_side} (5x clear).')
            self._stop_decision_point_check()
            else_seq = entry.get('else_sequence', [])
            if else_seq:
                self.get_logger().info('Running else_sequence (no human).')
                self._sequence_stack.append((self._current_sequence, self._current_index, True))
                self._current_sequence = else_seq
                self._current_index = 0
                self._oneshot_timer(1.0, self._send_next_command)
            else:
                self._current_index += 1
                self._oneshot_timer(1.0, self._send_next_command)
            return
        if human_present:
            self.get_logger().info(f'Human on {check_side}. Running alternative sequence.')
            self._stop_decision_point_check()
            self._run_human_sequence()

    def _run_human_sequence(self) -> None:
        """Run the if_human_sequence for the current decision point (can contain nested decision points)."""
        entry = self._current_sequence[self._current_index]
        seq = entry.get('if_human_sequence', [])
        if not seq:
            self.get_logger().warn('Decision point has empty if_human_sequence; continuing.')
            self._current_index += 1
            self._oneshot_timer(1.0, self._send_next_command)
            return
        # Push current context and step into the nested sequence; when done we continue past this decision point
        self._sequence_stack.append((self._current_sequence, self._current_index, False))
        self._current_sequence = seq
        self._current_index = 0
        self._send_next_command()

    def _goal_response_callback(self, future):
        """Handle goal acceptance/rejection."""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self._command_in_flight = False
            self.get_logger().error('✗ Goal rejected!')
            rclpy.shutdown()
            return
        
        self.get_logger().info('Goal accepted. Waiting for command to complete before next...')
        
        # Wait for result — next command is sent only from _result_callback when this finishes
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)
    
    def _feedback_callback(self, feedback_msg):
        """Handle progress feedback."""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'  Progress: {feedback.progress} - Waypoint: {feedback.current_waypoint}')
    
    def _result_callback(self, future):
        """Handle command result and move to next command only after this command is fully done."""
        self._command_in_flight = False
        try:
            result = future.result().result
        except Exception as e:
            self.get_logger().error(f'✗ Command result error: {e}')
            rclpy.shutdown()
            return
        
        if result.success:
            self.get_logger().info(f'✓ Command completed: {result.message}. Proceeding to next.')
            self._current_index += 1
            self._oneshot_timer(0.1, self._send_next_command)
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

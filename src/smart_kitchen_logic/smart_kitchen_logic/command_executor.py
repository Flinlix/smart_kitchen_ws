#!/usr/bin/env python3
"""Command executor node: executes named commands as waypoint sequences.

This node provides an action server that accepts high-level command names
(e.g., 'pick_cup', 'drop_cup') and executes them as sequences of waypoints.

Commands are defined in commands.toml, which references waypoints defined
in waypoints.toml. The node orchestrates:
  - Sequential waypoint execution via waypoint_sequencer
  - Gripper actions (open/close) via topics
  - Carriage/lift positioning
  - Progress feedback to action clients

Parameters:
  commands_file   (string)  Path to commands.toml
  waypoints_file  (string)  Path to waypoints.toml

Action servers:
  /execute_command  (smart_kitchen_interfaces/action/ExecuteCommand)

Action clients:
  /move_to_joints   (smart_kitchen_interfaces/action/MoveToJoints)

Topics published:
  /elmo/id1/carriage/position/set  (std_msgs/Float32)
  /elmo/id1/lift/position/set      (std_msgs/Float32)
  /robotiq_gripper_controller/gripper_cmd  (std_msgs/String)  # "open" or "close"
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from control_msgs.action import GripperCommand
from rclpy.callback_groups import ReentrantCallbackGroup
from action_msgs.msg import GoalStatus
from std_msgs.msg import Float32, String
from smart_kitchen_interfaces.action import ExecuteCommand, MoveToJoints

import tomllib
import time
from pathlib import Path

WAYPOINT_PATH = (
    Path.home() / 'workspace' / 'smart_kitchen_ws' / 'src' / 'smart_kitchen_logic' / 'config' /'waypoints.toml'
)

COMMANDS_PATH = (
    Path.home() / 'workspace' / 'smart_kitchen_ws' / 'src' / 'smart_kitchen_logic' / 'config' /'commands.toml'
)

DEFAULT_DURATION_SEC = 3.0

GRIPPER_WAYPOINTS = ['open_gripper', 'close_gripper']
CUP_COMMAND = ['pick_cup']


class CommandExecutorNode(Node):
    def __init__(self):
        super().__init__('command_executor_node')

        # Allow concurrent action execution
        self._callback_group = ReentrantCallbackGroup()

        # Publishers
        self._carriage_pub = self.create_publisher(
            Float32, '/elmo/id1/carriage/position/set', 10)
        self._lift_pub = self.create_publisher(
            Float32, '/elmo/id1/lift/position/set', 10)

        # Action client for joint movements
        self._move_client = ActionClient(
            self, 
            MoveToJoints, 
            '/move_to_joints',
            callback_group=self._callback_group)
        
        self._gripper_client = ActionClient(
            self,
            GripperCommand,
            '/robotiq_gripper_controller/gripper_cmd',
            callback_group=self._callback_group)

        self._action_server = ActionServer(
            self,
            ExecuteCommand,
            '/execute_command',
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group)

        # Load configuration
        self._commands = {}
        self._waypoints = {}
        self._load_configuration()
        self._gripper_open = float(self._waypoints.get('gripper_open', 0.0))
        self._gripper_close = float(self._waypoints.get('gripper_close', 0.5))

        # State tracking
        self._current_goal_handle = None
        self._cancel_requested = False

        self.get_logger().info('Command Executor Node initialized')

    def _load_configuration(self) -> None:
        """Load commands and waypoints from TOML files."""
        
        with open(WAYPOINT_PATH, 'rb') as f:
            self._waypoints = tomllib.load(f)
        
        with open(COMMANDS_PATH, 'rb') as f:
            self._commands = tomllib.load(f)

    ######### Action server callbacks #########
    def _goal_callback(self, goal_request) -> GoalResponse:
        """Accept or reject incoming goal requests."""
        command_name = goal_request.command_name
        
        if command_name not in self._commands:
            self.get_logger().warn(
                f'Rejecting unknown command: "{command_name}". '
                f'Available: {list(self._commands.keys())}')
            return GoalResponse.REJECT

        params = goal_request.cup_id if goal_request.cup_id else None
        
        if command_name in CUP_COMMAND and not params:
            self.get_logger().warn(
                f'Rejecting command "{command_name}" due to missing cup_id parameter.')
            return GoalResponse.REJECT
        
        self.get_logger().info(
            f'Accepting command: "{command_name}" with params: {params}')
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        """Handle cancellation requests."""
        self.get_logger().info('Cancellation requested')
        self._cancel_requested = True
        return CancelResponse.ACCEPT

    async def _execute_callback(self, goal_handle):
        """Execute the requested command as a waypoint sequence."""
        self._current_goal_handle = goal_handle
        self._cancel_requested = False

        command_name = goal_handle.request.command_name
        command = self._commands[command_name]
        
        cup_id = goal_handle.request.cup_id if goal_handle.request.cup_id else None        
        waypoint_names = command.get('waypoints', [])
        
        feedback = ExecuteCommand.Feedback()
        result = ExecuteCommand.Result()
        
        self.get_logger().info(
            f'Executing command "{command_name}" with cup_id: {cup_id} '
            f'→ waypoints: {waypoint_names}')

        total_waypoints = len(waypoint_names)

        for idx, waypoint_name in enumerate(waypoint_names):
            # Check for cancellation
            if self._cancel_requested or not goal_handle.is_active:
                self.get_logger().warn(f'Command "{command_name}" cancelled')
                goal_handle.canceled()
                result.success = False
                result.message = 'Cancelled by user'
                return result

            # Publish feedback
            feedback.current_waypoint = waypoint_name
            feedback.progress = f"{idx+1}/{total_waypoints}"
            goal_handle.publish_feedback(feedback)

            # Handle gripper
            if waypoint_name in GRIPPER_WAYPOINTS:
                success = await self._execute_gripper(waypoint_name)
                if not success:
                    result.success = False
                    result.message = f'Failed to execute gripper command: {waypoint_name}'
                    goal_handle.abort()
                    return result
                continue

            # Execute waypoint
            if waypoint_name not in self._waypoints:
                self.get_logger().error(
                    f'Waypoint "{waypoint_name}" not found in configuration')
                result.success = False
                result.message = f'Unknown waypoint: {waypoint_name}'
                goal_handle.abort()
                return result

            success = await self._execute_waypoint(waypoint_name)
            if not success:
                self.get_logger().error(f'Failed to execute waypoint: {waypoint_name}')
                result.success = False
                result.message = f'Failed at waypoint: {waypoint_name}'
                goal_handle.abort()
                return result

            self.get_logger().info(
                f'Completed waypoint [{idx+1}/{total_waypoints}]: {waypoint_name}')

        # Success
        self.get_logger().info(f'Command "{command_name}" completed successfully')
        goal_handle.succeed()
        result.success = True
        result.message = f'Command "{command_name}" completed'
        return result


    ######### Action execution helper #########
    async def _call_action(self, client: ActionClient, goal, timeout_sec: float = 10.0) -> tuple[bool, str]:
        """Generic action call handler with future management.
        
        Args:
            client: Action client to use
            goal: Goal message to send
            timeout_sec: Timeout for the entire action
        
        Returns:
            Tuple of (success: bool, message: str)
        """
        # Wait for action server
        if not client.wait_for_server(timeout_sec=2.0):
            return False, 'Action server not available'

        # Send goal (non-blocking await)
        goal_handle = await client.send_goal_async(goal)
        
        if not goal_handle.accepted:
            return False, 'Goal rejected by action server'

        # Wait for result (non-blocking await)
        result = await goal_handle.get_result_async()
        
        success = (result.status == GoalStatus.STATUS_SUCCEEDED)
        
        if success:
            return True, 'Success'
        else:
            return False, f'Action failed with status {result.status}'

    ########## Command execution helpers #########
    async def _execute_gripper(self, command: str) -> bool:
        """Execute gripper open/close command via action."""
        if command == 'open_gripper':
            gripper_position = self._gripper_open
            self.get_logger().info('Gripper: open')
            
        elif command == 'close_gripper':
            gripper_position = self._gripper_close
            self.get_logger().info('Gripper: close')
        else:
            return False

        # Create goal
        goal = GripperCommand.Goal()
        goal.command.position = gripper_position
        goal.command.max_effort = 50.0

        # Execute action
        self._sleep(DEFAULT_DURATION_SEC/2)
        success, message = await self._call_action(self._gripper_client, goal, timeout_sec=5.0)
        
        if not success:
            self.get_logger().error(f'Gripper action failed: {message}')
            
        self._sleep(DEFAULT_DURATION_SEC/2)
        
        return success

    async def _execute_waypoint(self, waypoint_name: str, cup_id: str = None) -> bool:
        """Execute a single waypoint."""
        waypoint = self._waypoints[waypoint_name]

        # Extract waypoint data
        joints = waypoint.get('joints', [])
        carriage = waypoint.get('carriage', None)
        lift = waypoint.get('lift', None)
        duration = DEFAULT_DURATION_SEC # duration = waypoint.get('time_from_start', DEFAULT_DURATION_SEC)
        wait_after = waypoint.get('wait_after_sec', 0.0)

        # Create and send goal - TODO: adjust angles here according to cup_id
        goal = MoveToJoints.Goal()
        goal.joint_angles = [float(j) for j in joints]
        goal.duration_sec = float(duration)

        # Execute joint movement first and wait until complete
        success, message = await self._call_action(self._move_client, goal, timeout_sec=duration + 5.0)
        
        if not success:
            self.get_logger().error(f'Waypoint execution failed: {message}')
            return False
        
        self.get_logger().info(f'Joints reached target position for waypoint: {waypoint_name}')
        
        # Wait a bit to ensure joints are fully settled
        self._sleep(duration/2)
        
        # Now move carriage/lift after joints are in position
        if carriage is not None:
            self.get_logger().info(f'Setting carriage to: {carriage}')
            self._carriage_pub.publish(Float32(data=float(carriage)))
            self._sleep(DEFAULT_DURATION_SEC/2)
        if lift is not None:
            self.get_logger().info(f'Setting lift to: {lift}')
            self._lift_pub.publish(Float32(data=float(lift)))
            self._sleep(DEFAULT_DURATION_SEC/2)

        # Wait after movement if specified
        if wait_after > 0.0:
            self._sleep(wait_after)
        
        return True
    
    def _sleep(self, seconds: float) -> None:
        """Blocking sleep - safe to use with ReentrantCallbackGroup."""
        time.sleep(seconds)


def main(args=None):
    rclpy.init(args=args)
    node = CommandExecutorNode()
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
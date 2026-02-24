#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Bool, Float64MultiArray

# Latched QoS — late subscribers still receive the last message
_LATCHED_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)


class RobotMoverNode(Node):
    def __init__(self):
        super().__init__('robot_mover_node')

        self.declare_parameter('target_joints', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.action_client = ActionClient(
            self, FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory')
        self.goal_handle = None

        self.stop_sub = self.create_subscription(
            Bool, '/emergency_stop', self.emergency_stop_callback, 10)

        # Accept joint goals from other nodes (e.g. init_node)
        self.create_subscription(
            Float64MultiArray, '/move_joints_goal',
            self._on_joints_goal, 10)

        # Publish movement result so other nodes can react
        self._result_pub = self.create_publisher(
            Bool, '/move_joints_result', 10)

        # Latched: late subscribers (e.g. init_node) still get the ready signal
        self._ready_pub = self.create_publisher(
            Bool, '/move_joints_ready', _LATCHED_QOS)

        self.current_target = None
        self.is_moving = False
        self.obstacle_detected = False
        self._server_ready = False
        self._waiting_to_resume = False  # True after stop, cleared on resume

        self.joint_names = [
            'joint_1', 'joint_2', 'joint_3',
            'joint_4', 'joint_5', 'joint_6',
        ]
        self.get_logger().info('Robot Mover Node initialized. Ready for commands.')

    # ── emergency stop / resume ─────────────────────────────────────────
    def emergency_stop_callback(self, msg):
        """Listens for triggers from the Safety Monitor."""
        prev = self.obstacle_detected
        self.obstacle_detected = msg.data

        if self.obstacle_detected and self.is_moving:
            self.get_logger().warn('EMERGENCY STOP — cancelling movement!')
            self.cancel_movement()
            self._waiting_to_resume = True

        elif not self.obstacle_detected and prev and self._waiting_to_resume:
            # Resume only once on the falling edge (True → False)
            self._waiting_to_resume = False
            self.get_logger().info('Clearance received — resuming movement.')
            if self.current_target:
                self.send_goal(self.current_target, duration_sec=4.0)

    # ── joints-goal topic ────────────────────────────────────────────────
    def _on_joints_goal(self, msg: Float64MultiArray) -> None:
        """Receive a joint goal from another node and execute it."""
        self.get_logger().info(f'Received /move_joints_goal: {list(msg.data)}')
        self.send_goal(list(msg.data))

    # ── movement ────────────────────────────────────────────────────────
    def send_goal(self, target_positions, duration_sec=5.0):
        """Send a trajectory goal without blocking the executor."""
        self.current_target = target_positions

        if not self._server_ready:
            self.get_logger().warn('Action server not ready yet — skipping goal.')
            return

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = list(target_positions)
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=0)
        goal_msg.trajectory.points.append(point)

        self.get_logger().info(f'Sending trajectory goal: {target_positions}')
        future = self.action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.is_moving = False
            self.get_logger().error('Goal was rejected by the controller.')
            return

        self.goal_handle = goal_handle
        self.is_moving = True
        self.get_logger().info('Goal accepted — robot is moving.')

        # Monitor the result so is_moving is cleared when the goal finishes
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result()
        status = result.status
        self.is_moving = False
        success = status == GoalStatus.STATUS_SUCCEEDED
        if success:
            self.get_logger().info('Trajectory completed successfully.')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Trajectory was cancelled.')
        else:
            self.get_logger().warn(f'Trajectory finished with status {status}.')
        result_msg = Bool()
        result_msg.data = success
        self._result_pub.publish(result_msg)
        self.get_logger().info(f'Published /move_joints_result = {success}')

    # ── cancel ──────────────────────────────────────────────────────────
    def cancel_movement(self):
        self.is_moving = False
        if self.goal_handle is not None:
            self.get_logger().info('Cancelling current goal...')
            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(
                lambda f: self.get_logger().info('Cancel request processed.'))


def main(args=None):
    rclpy.init(args=args)
    node = RobotMoverNode()

    def _on_server_ready():
        """Called once the action server is available."""
        node._server_ready = True
        target = node.get_parameter('target_joints').value
        node.get_logger().info(f'Action server ready — sending initial goal: {target}')
        # Announce readiness so other nodes can start sending goals
        ready_msg = Bool()
        ready_msg.data = True
        node._ready_pub.publish(ready_msg)
        node.send_goal(target, duration_sec=8.0)

    def _check_server():
        """Non-blocking poll for action server availability."""
        if node.action_client.server_is_ready():
            node.destroy_timer(node._poll_timer)
            _on_server_ready()

    # Poll every 2 s — never blocks the executor
    node._poll_timer = node.create_timer(2.0, _check_server)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
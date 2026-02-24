#!/usr/bin/env python3
"""Init node: moves the Kinova Gen3 arm to a fixed home position on startup.

Sends a single MoveToJoints action goal to robot_mover_node and waits for
a proper success/failure result before publishing /robot_initialized.

Flow
----
  1. Wait for the /move_to_joints action server (robot_mover_node).
  2. Send one goal: configured joint angles + duration.
  3. Log feedback (elapsed seconds) while the arm moves.
  4. Publish True/False on /robot_initialized based on the result.

Parameters
----------
  init_joints              list[float]  Six joint angles [j1..j6] in radians
                                        (range –π … π, default: all zeros).
  trajectory_duration_sec  float        Time allowed to reach the position (s).
                                        Default: 8.0

Topics published
----------------
  /robot_initialized  (std_msgs/Bool)  True on success, False on failure.

Action client
-------------
  /move_to_joints  (smart_kitchen_interfaces/action/MoveToJoints)
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from std_msgs.msg import Bool, Float32
from smart_kitchen_interfaces.action import MoveToJoints


HOME = [
-0.221,
-0.029,
-0.676,
0.510,
-1.117,
-1.886,
]
HOME_LIFT = 0.25
HOME_CARRIAGE = 1.0

DEFAULT_DURATION_SEC = 8.0


class InitNode(Node):
    """Sends a single MoveToJoints goal on startup and reports the result."""

    def __init__(self):
        super().__init__('init_node')

        self.declare_parameter('init_joints', HOME)
        self.declare_parameter('trajectory_duration_sec', DEFAULT_DURATION_SEC)
        self.declare_parameter('init_carriage', HOME_CARRIAGE)
        self.declare_parameter('init_lift', HOME_LIFT)

        self._goal_sent = False

        # ── result publisher ─────────────────────────────────────────────
        self._init_pub = self.create_publisher(Bool, '/robot_initialized', 1)

        # ── rail / lift publishers ───────────────────────────────────────
        self._carriage_pub = self.create_publisher(
            Float32, '/elmo/id1/carriage/position/set', 1)
        self._lift_pub = self.create_publisher(
            Float32, '/elmo/id1/lift/position/set', 1)

        # ── action client to robot_mover_node ────────────────────────────
        self._action_client = ActionClient(self, MoveToJoints, '/move_to_joints')

        # Poll every 2 s until the action server is available
        self._poll_timer = self.create_timer(2.0, self._check_server)
        self.get_logger().info(
            'Init node started. Waiting for /move_to_joints action server ...'
        )

    # ── server polling ───────────────────────────────────────────────────

    def _check_server(self) -> None:
        if self._action_client.server_is_ready():
            self.destroy_timer(self._poll_timer)
            self.get_logger().info('Action server ready — sending init goal.')
            self._send_init_goal()

    # ── goal ─────────────────────────────────────────────────────────────

    def _send_init_goal(self) -> None:
        if self._goal_sent:
            return
        self._goal_sent = True

        joint_angles: list[float] = self.get_parameter('init_joints').value
        duration_sec: float = self.get_parameter('trajectory_duration_sec').value

        goal = MoveToJoints.Goal()
        goal.joint_angles = joint_angles
        goal.duration_sec = duration_sec

        self.get_logger().info(
            f'Sending MoveToJoints goal: joints={joint_angles}, duration={duration_sec} s'
        )
        send_future = self._action_client.send_goal_async(
            goal, feedback_callback=self._feedback_callback)
        send_future.add_done_callback(self._on_goal_response)

    # ── action client callbacks ───────────────────────────────────────────

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Init goal rejected by robot_mover_node.')
            self._publish_result(success=False)
            return
        self.get_logger().info('Init goal accepted — arm is moving to home position.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _feedback_callback(self, feedback_msg) -> None:
        elapsed = feedback_msg.feedback.elapsed_sec
        # self.get_logger().info(f'Moving ... elapsed: {elapsed:.1f} s')

    def _on_goal_result(self, future) -> None:
        wrapped = future.result()
        # Use the action goal STATUS as the source of truth.
        # result.success reflects what the mover returned, but the Kortex
        # controller can leave it False even on a successful move.
        # STATUS_SUCCEEDED is set only when the server explicitly calls
        # goal_handle.succeed(), which IS the reliable success indicator.
        success = (wrapped.status == GoalStatus.STATUS_SUCCEEDED)
        message = wrapped.result.message
        self.get_logger().info(
            f'Init move finished — status={wrapped.status}, '
            f'result.success={wrapped.result.success}, using success={success}, '
            f'message="{message}"'
        )
        if success:
            self._move_rail_and_lift()
        self._publish_result(success=success)

    # ── helpers ───────────────────────────────────────────────────────────

    def _move_rail_and_lift(self) -> None:
        """Publish carriage and lift target positions after the arm is home."""
        carriage_val: float = self.get_parameter('init_carriage').value
        lift_val: float = self.get_parameter('init_lift').value

        carriage_msg = Float32()
        carriage_msg.data = carriage_val
        self._carriage_pub.publish(carriage_msg)
        self.get_logger().info(
            f'Published carriage position: {carriage_val} → /elmo/id1/carriage/position/set'
        )

        lift_msg = Float32()
        lift_msg.data = lift_val
        self._lift_pub.publish(lift_msg)
        self.get_logger().info(
            f'Published lift position: {lift_val} → /elmo/id1/lift/position/set'
        )

    def _publish_result(self, success: bool) -> None:
        msg = Bool()
        msg.data = success
        self._init_pub.publish(msg)
        self.get_logger().info(f'Published /robot_initialized = {success}')


# ── entry point ───────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = InitNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


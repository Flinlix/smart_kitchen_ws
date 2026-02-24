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

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool
from smart_kitchen_interfaces.action import MoveToJoints


HOME = [-0.2206833763616345,
-0.029291043565344843,
-0.6762437124305887,
- 0.5103309070475242,
- -1.1174171196587173,
- -1.8868943734274533,
- 0.0035526392698287967
]
DEFAULT_DURATION_SEC = 8.0


class InitNode(Node):
    """Sends a single MoveToJoints goal on startup and reports the result."""

    def __init__(self):
        super().__init__('init_node')

        self.declare_parameter('init_joints', HOME)
        self.declare_parameter('trajectory_duration_sec', DEFAULT_DURATION_SEC)

        self._goal_sent = False

        # ── result publisher ─────────────────────────────────────────────
        self._init_pub = self.create_publisher(Bool, '/robot_initialized', 1)

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

        clamped = [max(-math.pi, min(math.pi, a)) for a in joint_angles]
        if clamped != joint_angles:
            self.get_logger().warn(
                f'Some init_joints were outside [–π, π] and were clamped: '
                f'{joint_angles} → {clamped}'
            )

        goal = MoveToJoints.Goal()
        goal.joint_angles = clamped
        goal.duration_sec = duration_sec

        self.get_logger().info(
            f'Sending MoveToJoints goal: joints={clamped}, duration={duration_sec} s'
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
        self.get_logger().info(f'Moving ... elapsed: {elapsed:.1f} s')

    def _on_goal_result(self, future) -> None:
        result = future.result().result
        self.get_logger().info(
            f'Init move finished — success={result.success}, '
            f'message="{result.message}"'
        )
        self._publish_result(success=result.success)

    # ── helpers ───────────────────────────────────────────────────────────

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


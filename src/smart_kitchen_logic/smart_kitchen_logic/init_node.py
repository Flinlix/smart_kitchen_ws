#!/usr/bin/env python3
"""Init node: moves the Kinova Gen3 arm to a fixed home position on startup.

Coordinates with robot_mover_node via topics — no duplicated action-client
or trajectory code here.

Flow
----
  1. Waits for /move_joints_ready (True) from robot_mover_node.
  2. Publishes the init joint angles on /move_joints_goal.
  3. Waits for /move_joints_result (Bool) from robot_mover_node.
  4. Re-publishes the result on /robot_initialized and stops.

Parameters
----------
  init_joints              list[float]  Six joint angles [j1..j6] in radians
                                        (range –π … π, default: all zeros).
  trajectory_duration_sec  float        Forwarded to robot_mover_node via the
                                        goal message layout is not needed here;
                                        robot_mover_node uses its own default.

Topics subscribed
-----------------
  /move_joints_ready   (std_msgs/Bool)              Latched — mover is online.
  /move_joints_result  (std_msgs/Bool)              True=success, False=failure.

Topics published
----------------
  /move_joints_goal    (std_msgs/Float64MultiArray) Init joint angles.
  /robot_initialized   (std_msgs/Bool)              True on success.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool, Float64MultiArray


DEFAULT_INIT_JOINTS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# Match the latched QoS used by robot_mover_node for /move_joints_ready
_LATCHED_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)


class InitNode(Node):
    """Sends a single joint goal to robot_mover_node and reports the outcome."""

    def __init__(self):
        super().__init__('init_node')

        self.declare_parameter('init_joints', DEFAULT_INIT_JOINTS)

        self._goal_sent = False  # send only once

        # ── publishers ───────────────────────────────────────────────────
        self._goal_pub = self.create_publisher(
            Float64MultiArray, '/move_joints_goal', 10)
        self._init_pub = self.create_publisher(
            Bool, '/robot_initialized', 1)

        # ── subscribers ──────────────────────────────────────────────────
        # Latched: receives the signal even if mover_node started first
        self.create_subscription(
            Bool, '/move_joints_ready', self._on_mover_ready, _LATCHED_QOS)

        self.create_subscription(
            Bool, '/move_joints_result', self._on_move_result, 10)

        self.get_logger().info(
            'Init node started. Waiting for robot_mover_node ...'
        )

    # ── callbacks ────────────────────────────────────────────────────────

    def _on_mover_ready(self, msg: Bool) -> None:
        """Called once robot_mover_node signals the action server is up."""
        if not msg.data or self._goal_sent:
            return

        joint_angles: list[float] = self.get_parameter('init_joints').value
        clamped = [max(-math.pi, min(math.pi, a)) for a in joint_angles]

        if clamped != joint_angles:
            self.get_logger().warn(
                f'Some init_joints values were outside [–π, π] and were '
                f'clamped: {joint_angles} → {clamped}'
            )

        goal_msg = Float64MultiArray()
        goal_msg.data = clamped

        self._goal_sent = True
        self._goal_pub.publish(goal_msg)
        self.get_logger().info(f'Published init goal on /move_joints_goal: {clamped}')

    def _on_move_result(self, msg: Bool) -> None:
        """Forwards the mover result as /robot_initialized."""
        result_msg = Bool()
        result_msg.data = msg.data
        self._init_pub.publish(result_msg)
        self.get_logger().info(
            f'Robot init {"succeeded" if msg.data else "FAILED"}. '
            f'Published /robot_initialized = {msg.data}'
        )


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


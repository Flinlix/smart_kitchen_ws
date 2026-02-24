#!/usr/bin/env python3
"""Moving node: executes joint-angle goals and controls carriage/lift.

Receives pre-computed joint angles from planning_node and executes them
via the joint_trajectory_controller FollowJointTrajectory action.
Carriage and lift goals are forwarded to the ELMO drives.

Topics subscribed:
  /move_robot_goal     (std_msgs/Float64MultiArray) – 6 joint angles
  /move_carriage_goal  (std_msgs/Float32)
  /move_lift_goal      (std_msgs/Float32)

Topics published:
  /move_robot_result                (std_msgs/Bool)
  /move_carriage_result             (std_msgs/Bool)
  /move_lift_result                 (std_msgs/Bool)
  /elmo/id1/carriage/position/set   (std_msgs/Float32)
  /elmo/id1/lift/position/set       (std_msgs/Float32)

Action client:
  /joint_trajectory_controller/follow_joint_trajectory
"""

import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from std_msgs.msg import Bool, Float32, Float64MultiArray
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


ARM_JOINT_NAMES = [
    'joint_1', 'joint_2', 'joint_3',
    'joint_4', 'joint_5', 'joint_6',
]

TRAJECTORY_DURATION_SEC = 5
ACTION_TIMEOUT_SEC = 30.0


class MovingNode(Node):
    def __init__(self):
        super().__init__('moving_node')

        self._action_done = threading.Event()
        self._action_success = False

        # Joint trajectory action client
        self._traj_client = ActionClient(
            self, FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory')

        # Joint goal / result
        self.create_subscription(
            Float64MultiArray, '/move_robot_goal', self._on_goal, 10)
        self._result_pub = self.create_publisher(
            Bool, '/move_robot_result', 10)

        # Carriage: goal → ELMO drive → result
        self._carriage_pub = self.create_publisher(
            Float32, '/elmo/id1/carriage/position/set', 10)
        self._carriage_result_pub = self.create_publisher(
            Bool, '/move_carriage_result', 10)
        self.create_subscription(
            Float32, '/move_carriage_goal', self._on_carriage_goal, 10)

        # Lift: goal → ELMO drive → result
        self._lift_pub = self.create_publisher(
            Float32, '/elmo/id1/lift/position/set', 10)
        self._lift_result_pub = self.create_publisher(
            Bool, '/move_lift_result', 10)
        self.create_subscription(
            Float32, '/move_lift_goal', self._on_lift_goal, 10)

        self.get_logger().info('Moving node started.')

    # ── Joint goal handling ───────────────────────────────────────────────

    def _on_goal(self, msg: Float64MultiArray) -> None:
        joints = list(msg.data)
        if len(joints) != len(ARM_JOINT_NAMES):
            self.get_logger().error(
                f'Expected {len(ARM_JOINT_NAMES)} joints, got {len(joints)}')
            self._result_pub.publish(Bool(data=False))
            return

        self.get_logger().info(
            f'Joint goal received: {[f"{j:.3f}" for j in joints]}')
        threading.Thread(
            target=self._execute, args=(joints,), daemon=True).start()

    # ── Trajectory execution ──────────────────────────────────────────────

    def _execute(self, positions: list[float]) -> None:
        if not self._traj_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Trajectory action server not available!')
            self._result_pub.publish(Bool(data=False))
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM_JOINT_NAMES)

        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.time_from_start = Duration(sec=TRAJECTORY_DURATION_SEC, nanosec=0)
        goal.trajectory.points.append(pt)

        self._action_done.clear()
        self._action_success = False

        self.get_logger().info('Sending trajectory ...')
        future = self._traj_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

        if not self._action_done.wait(timeout=ACTION_TIMEOUT_SEC):
            self.get_logger().error(
                f'Trajectory timed out after {ACTION_TIMEOUT_SEC}s!')
            self._result_pub.publish(Bool(data=False))
            return

        self.get_logger().info(
            'Move ' + ('succeeded.' if self._action_success else 'FAILED.'))
        self._result_pub.publish(Bool(data=self._action_success))

    def _on_goal_response(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Trajectory goal rejected.')
            self._action_done.set()
            return
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        status = future.result().status
        self._action_success = (status == GoalStatus.STATUS_SUCCEEDED)
        self._action_done.set()

    # ── Carriage / lift control ───────────────────────────────────────────

    def _on_carriage_goal(self, msg: Float32) -> None:
        self._carriage_pub.publish(Float32(data=msg.data))
        self.get_logger().info(f'Carriage setpoint → {msg.data}')
        self._carriage_result_pub.publish(Bool(data=True))

    def _on_lift_goal(self, msg: Float32) -> None:
        self._lift_pub.publish(Float32(data=msg.data))
        self.get_logger().info(f'Lift setpoint → {msg.data}')
        self._lift_result_pub.publish(Bool(data=True))


def main(args=None):
    rclpy.init(args=args)
    node = MovingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

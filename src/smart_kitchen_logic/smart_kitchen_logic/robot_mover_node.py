#!/usr/bin/env python3
"""Robot Mover Node — exposes /move_to_joints as a ROS 2 action server.

Action  : smart_kitchen_interfaces/action/MoveToJoints
  Goal     joint_angles (float64[6])  — target positions in radians
           duration_sec (float64)     — time to reach the position
  Feedback elapsed_sec  (float64)     — seconds since goal was accepted
  Result   success      (bool)
           message      (string)

Internally forwards every goal to the joint_trajectory_controller via
FollowJointTrajectory and maps the outcome back to the action result.

Emergency stop: subscribing to /emergency_stop (std_msgs/Bool) cancels
the in-flight trajectory and aborts the active action goal.
"""

import threading

import rclpy
import rclpy.executors
from rclpy.action import ActionClient, ActionServer
from rclpy.action.server import GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Bool
from smart_kitchen_interfaces.action import MoveToJoints


JOINT_NAMES = [
    'joint_1', 'joint_2', 'joint_3',
    'joint_4', 'joint_5', 'joint_6',
]


class RobotMoverNode(Node):
    def __init__(self):
        super().__init__('robot_mover_node')

        # ReentrantCallbackGroup lets the execute_callback block on an Event
        # while other callbacks (trajectory result, emergency stop) still run.
        self._cb_group = ReentrantCallbackGroup()

        # ── /move_to_joints action server ────────────────────────────────
        self._move_server = ActionServer(
            self,
            MoveToJoints,
            '/move_to_joints',
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._cb_group,
        )

        # ── FollowJointTrajectory action client ──────────────────────────
        self._traj_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory',
            callback_group=self._cb_group,
        )

        # ── Emergency stop ───────────────────────────────────────────────
        self.create_subscription(
            Bool, '/emergency_stop',
            self._emergency_stop_callback,
            10, callback_group=self._cb_group,
        )

        # Shared state (protected by the fact that only one goal runs at once)
        self._traj_goal_handle = None
        self._traj_done = threading.Event()
        self._traj_success = False
        self._traj_message = ''

        self.get_logger().info(
            'Robot Mover Node started. Action server /move_to_joints is ready.')

    # ── /move_to_joints server: goal / cancel ─────────────────────────────

    def _goal_callback(self, goal_request):
        """Accept or reject an incoming MoveToJoints goal."""
        if self._traj_goal_handle is not None:
            self.get_logger().warn('Goal rejected — a movement is already in progress.')
            return GoalResponse.REJECT
        self.get_logger().info(
            f'Goal received: joints={list(goal_request.joint_angles)}, '
            f'duration={goal_request.duration_sec} s'
        )
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        """Accept cancel requests from the client."""
        self.get_logger().info('Cancel requested by client.')
        self._cancel_trajectory()
        return CancelResponse.ACCEPT

    # ── /move_to_joints server: execution ─────────────────────────────────

    def _execute_callback(self, goal_handle):
        """Run in its own thread (MultiThreadedExecutor).

        Sends the trajectory to FollowJointTrajectory, publishes elapsed-time
        feedback once per second, then returns the MoveToJoints result once
        the trajectory finishes (or is cancelled / aborted).
        """
        req = goal_handle.request
        self.get_logger().info(
            f'Executing goal: joints={list(req.joint_angles)}, '
            f'duration={req.duration_sec} s'
        )

        # ── wait for trajectory controller ───────────────────────────────
        if not self._traj_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Trajectory controller unavailable.')
            result = MoveToJoints.Result()
            result.success = False
            result.message = 'Trajectory controller unavailable.'
            goal_handle.abort()
            return result

        # ── build and send FollowJointTrajectory goal ─────────────────────
        traj_goal = FollowJointTrajectory.Goal()
        traj_goal.trajectory.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = list(req.joint_angles)
        point.time_from_start = Duration(sec=int(req.duration_sec), nanosec=0)
        traj_goal.trajectory.points.append(point)

        self._traj_done.clear()
        send_future = self._traj_client.send_goal_async(traj_goal)
        send_future.add_done_callback(self._on_traj_accepted)

        # ── publish feedback (elapsed seconds) until done ─────────────────
        start_ns = self.get_clock().now().nanoseconds
        feedback = MoveToJoints.Feedback()
        while not self._traj_done.wait(timeout=1.0):
            if goal_handle.is_cancel_requested:
                self._cancel_trajectory()
                # Give the cancel time to propagate
                self._traj_done.wait(timeout=5.0)
                break
            feedback.elapsed_sec = (
                self.get_clock().now().nanoseconds - start_ns) * 1e-9
            goal_handle.publish_feedback(feedback)

        # ── resolve the action goal ───────────────────────────────────────
        result = MoveToJoints.Result()
        result.success = self._traj_success
        result.message = self._traj_message

        if result.success:
            goal_handle.succeed()
        elif goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.abort()

        self._traj_goal_handle = None
        self.get_logger().info(
            f'Goal finished — success={result.success}, msg="{result.message}"'
        )
        return result

    # ── FollowJointTrajectory callbacks ───────────────────────────────────

    def _on_traj_accepted(self, future) -> None:
        traj_goal_handle = future.result()
        if not traj_goal_handle.accepted:
            self.get_logger().error('Trajectory goal rejected by controller.')
            self._traj_success = False
            self._traj_message = 'Trajectory goal rejected by controller.'
            self._traj_done.set()
            return

        self._traj_goal_handle = traj_goal_handle
        self.get_logger().info('Trajectory accepted — arm is moving.')
        result_future = traj_goal_handle.get_result_async()
        result_future.add_done_callback(self._on_traj_result)

    def _on_traj_result(self, future) -> None:
        traj_result = future.result()
        status = traj_result.status
        error_code = traj_result.result.error_code

        # error_code == 0 means SUCCESSFUL in FollowJointTrajectory.
        # The Kinova Kortex controller can return STATUS_ABORTED at the goal
        # level even when the motion completed correctly, so we treat
        # error_code == 0 as success regardless of the goal status.
        success = (status == GoalStatus.STATUS_SUCCEEDED) or (error_code == 0)

        if success:
            msg = 'Arm reached the target position.'
            self.get_logger().info(msg)
        elif status == GoalStatus.STATUS_CANCELED:
            msg = 'Trajectory was cancelled.'
            self.get_logger().warn(msg)
        else:
            msg = f'Trajectory ended with status={status}, error_code={error_code}.'
            self.get_logger().error(msg)

        self._traj_success = success
        self._traj_message = msg
        self._traj_done.set()   # unblocks _execute_callback

    # ── emergency stop ────────────────────────────────────────────────────

    def _emergency_stop_callback(self, msg: Bool) -> None:
        if msg.data and self._traj_goal_handle is not None:
            self.get_logger().warn('EMERGENCY STOP received — cancelling trajectory!')
            self._cancel_trajectory()

    def _cancel_trajectory(self) -> None:
        if self._traj_goal_handle is not None:
            cancel_future = self._traj_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(
                lambda f: self.get_logger().info('Trajectory cancel request sent.'))


def main(args=None):
    rclpy.init(args=args)
    node = RobotMoverNode()

    # MultiThreadedExecutor is required so that _execute_callback can block
    # on threading.Event while trajectory / emergency-stop callbacks still run.
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
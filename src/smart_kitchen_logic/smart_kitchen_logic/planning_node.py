#!/usr/bin/env python3
"""Main planning node: orchestrates the cup pickup sequence.

Subscribes to /cup_positions (PoseArray) and for each cup:
  1. Calls safety check  – is the position safely reachable?
  2. Moves the robot      – go to the cup position
  3. Grips                – close the gripper
  4. Moves back           – return to the home position
"""

import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Bool
from control_msgs.action import GripperCommand as GripperCommandAction

HOME_POSITION = (0.0, 0.0, 0.0)

MOVE_TIMEOUT_SEC = 30.0
GRIP_TIMEOUT_SEC = 15.0

GRIPPER_CLOSED = 0.7
GRIPPER_OPEN = 0.0
GRIPPER_MAX_EFFORT = 50.0


class PlanningNode(Node):
    def __init__(self):
        super().__init__('planning_node')

        self._lock = threading.Lock()
        self._processing = False
        self._done = False

        self.create_subscription(
            PoseArray, '/cup_positions', self._on_cup_positions, 10)

        # /move_robot communication
        self._move_goal_pub = self.create_publisher(
            Pose, '/move_robot_goal', 10)
        self._move_done = threading.Event()
        self.create_subscription(
            Bool, '/move_robot_result', self._on_move_result, 10)

        # gripper action client
        self._gripper_client = ActionClient(
            self, GripperCommandAction,
            '/robotiq_gripper_controller/gripper_cmd')
        self._grip_done = threading.Event()
        self._grip_success = False

        self.get_logger().info('Planning node started. Waiting for cup positions...')

    # ── subscriber callback ──────────────────────────────────────────────

    def _on_cup_positions(self, msg: PoseArray) -> None:
        with self._lock:
            if self._processing or self._done:
                return
            if not msg.poses:
                return
            self._processing = True

        positions = list(msg.poses)
        thread = threading.Thread(
            target=self._process_cups, args=(positions,), daemon=True)
        thread.start()

    # ── main processing loop ─────────────────────────────────────────────

    def _process_cups(self, poses: list[Pose]) -> None:
        total = len(poses)
        self.get_logger().info(
            f'Starting pickup sequence for {total} cup(s).')

        for i, pose in enumerate(poses, start=1):
            x = pose.position.x
            y = pose.position.y
            z = pose.position.z
            self.get_logger().info(
                f'--- Cup {i}/{total} at ({x:.3f}, {y:.3f}, {z:.3f}) ---')

            # Step 1: safety check
            safe = self._check_safety(x, y, z)
            if not safe:
                self.get_logger().warn(
                    f'Cup {i} is NOT safe to reach. Skipping.')
                continue

            # Step 2: move robot to cup position
            self._move_robot(x, y, z)

            # Step 3: grip the cup
            self._grip()

            # Step 4: move robot back to home
            self._move_robot(*HOME_POSITION)

            # Step 5: release the cup
            self._release()

            self.get_logger().info(f'Cup {i} pickup complete.')

        self.get_logger().info('All cups processed.')
        with self._lock:
            self._processing = False
            self._done = True

    # ── /move_robot_result callback ─────────────────────────────────────

    def _on_move_result(self, msg: Bool) -> None:
        if msg.data:
            self._move_done.set()

    # ── movement (talks to moving_node via topics) ───────────────────────

    def _move_robot(self, x: float, y: float, z: float) -> bool:
        """Publish goal to /move_robot_goal and block until /move_robot_result."""
        goal = Pose()
        goal.position.x = x
        goal.position.y = y
        goal.position.z = z

        self._move_done.clear()
        self.get_logger().info(
            f'Sending move goal ({x:.3f}, {y:.3f}, {z:.3f}) ...')
        self._move_goal_pub.publish(goal)

        if not self._move_done.wait(timeout=MOVE_TIMEOUT_SEC):
            self.get_logger().error('Move timed out!')
            return False

        self.get_logger().info(
            f'Move to ({x:.3f}, {y:.3f}, {z:.3f}) complete.')
        return True

    # ── gripper (via robotiq_gripper_controller action) ─────────────────

    def _send_gripper_goal(self, position: float, label: str) -> bool:
        """Send a GripperCommand goal and block until it finishes."""
        if not self._gripper_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Gripper action server not available!')
            return False

        goal = GripperCommandAction.Goal()
        goal.command.position = position
        goal.command.max_effort = GRIPPER_MAX_EFFORT

        self._grip_done.clear()
        self._grip_success = False

        self.get_logger().info(f'{label} (position={position:.2f}) ...')
        future = self._gripper_client.send_goal_async(goal)
        future.add_done_callback(self._gripper_goal_response)

        if not self._grip_done.wait(timeout=GRIP_TIMEOUT_SEC):
            self.get_logger().error(f'{label} timed out!')
            return False

        return self._grip_success

    def _gripper_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Gripper goal rejected.')
            self._grip_done.set()
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._gripper_result)

    def _gripper_result(self, future) -> None:
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._grip_success = True
        else:
            self.get_logger().warn(f'Gripper finished with status {status}.')
        self._grip_done.set()

    def _grip(self) -> bool:
        """Close the gripper to grasp a cup."""
        return self._send_gripper_goal(GRIPPER_CLOSED, 'Closing gripper')

    def _release(self) -> bool:
        """Open the gripper to release a cup."""
        return self._send_gripper_goal(GRIPPER_OPEN, 'Opening gripper')

    # ── mock helpers (replace with real service calls later) ──────────────

    def _check_safety(self, x: float, y: float, z: float) -> bool:
        """Mock /safety_estimation – always returns True."""
        self.get_logger().info(
            f'[MOCK safety] Checking ({x:.3f}, {y:.3f}, {z:.3f}) -> safe')
        time.sleep(0.5)
        return True


def main(args=None):
    rclpy.init(args=args)
    node = PlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

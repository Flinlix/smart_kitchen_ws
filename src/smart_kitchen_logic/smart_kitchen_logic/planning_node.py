#!/usr/bin/env python3
"""Main planning node: orchestrates the cup pickup sequence.

Subscribes to /cup_positions (PoseArray) and for each cup:
  1. Calls safety check  – is the position safely reachable?
  2. Moves the robot      – go to the cup position
  3. Grips                – close the gripper
  4. Moves back           – return to the home position

Topics published:
  /move_robot_goal                   (geometry_msgs/Pose)    – arm IK goal
  /move_carriage_goal                (std_msgs/Float32)      – carriage position goal (consumed by moving_node)
  /move_lift_goal                    (std_msgs/Float32)      – lift position goal (consumed by moving_node)
"""



# ALEX NODES
# Carriage is spawned initially at 0.0

import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Bool, Float32
from control_msgs.action import GripperCommand as GripperCommandAction
from smart_kitchen_logic.tf_transformation import PositionTracker

HOME_POSITION = (0.0, 0.0, 1.18)

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
        self._move_success = False
        self.create_subscription(
            Bool, '/move_robot_result', self._on_move_result, 10)

        # carriage goal / result (forwarded to ELMO by moving_node)
        self._move_carriage_pub = self.create_publisher(
            Float32, '/move_carriage_goal', 10)
        self._carriage_done = threading.Event()
        self._carriage_success = False
        self.create_subscription(
            Bool, '/move_carriage_result', self._on_carriage_result, 10)

        # lift goal / result (forwarded to ELMO by moving_node)
        self._move_lift_pub = self.create_publisher(
            Float32, '/move_lift_goal', 10)
        self._lift_done = threading.Event()
        self._lift_success = False
        self.create_subscription(
            Bool, '/move_lift_result', self._on_lift_result, 10)

        # gripper action client
        self._gripper_client = ActionClient(
            self, GripperCommandAction,
            '/robotiq_gripper_controller/gripper_cmd')
        self._grip_done = threading.Event()
        self._grip_success = False

        self._tf_tracker = PositionTracker(self)
        self.get_logger().info('Planning node started. Waiting for cup positions...')

    # ── subscriber callback ──────────────────────────────────────────────
    
    def _on_cup_positions(self, msg: PoseArray) -> None:
        with self._lock:
            if self._processing:
                self.get_logger().debug('Cup positions received but already processing. Ignoring.')
                return
            if self._done:
                self.get_logger().debug('Cup positions received but already done. Ignoring.')
                return
            if not msg.poses:
                self.get_logger().debug('Received empty PoseArray. Waiting for cups ...')
                return
            self._processing = True

        positions = list(msg.poses)
        self.get_logger().debug(
            f'Received {len(positions)} cup pose(s). Spawning processing thread.')
        thread = threading.Thread(
            target=self._process_cups, args=(positions,), daemon=True)
        thread.start()

    # ── main processing loop ─────────────────────────────────────────────

    def _process_cups(self, poses: list[Pose]) -> None:
        total = len(poses)
        self.get_logger().info(
            f'Starting pickup sequence for {total} cup(s).')

        try:
            bx, by, bz = self._tf_tracker.get_base_link_position()
            self.get_logger().info(
                f'Robot base_link in world: ({bx:.4f}, {by:.4f}, {bz:.4f})')
            gx, gy, gz = self._tf_tracker.get_full_position()
            self.get_logger().info(
                f'Gripper world position (incl. carriage/lift): ({gx:.4f}, {gy:.4f}, {gz:.4f})')
        except Exception as e:
            self.get_logger().warn(f'Could not read initial positions: {e}')

        for i, pose in enumerate(poses, start=1):
            # x = pose.position.x
            # y = pose.position.y
            # z = pose.position.z
            # Mocking
            x = 0.4
            y = 1.0
            z = 1.60
            self.get_logger().info(
                f'--- Cup {i}/{total} at ({x:.3f}, {y:.3f}, {z:.3f}) ---')

            # Step 1: safety check
            safe = self._check_safety(x, y, z)
            if not safe:
                self.get_logger().warn(
                    f'Cup {i} is NOT safe to reach. Skipping.')
                continue

            # Step 2.1: Move carriage to the cup x position
            self._move_carriage(x)

            # Step 2.2: Move lift to the cup z position
            self._move_lift(0.3)
            
            # Step 2.3 Get current carriage and lift positions
            carriage_pos = self._tf_tracker.carriage_x
            lift_pos = self._tf_tracker.lift_z

            # Step 2.4 Remove the carriage and lift offsets from the cup position
            x_robot_move = x - carriage_pos
            z_robot_move = z - lift_pos
            
            # Step 2.5: move robot to cup position
            if not self._move_robot(x_robot_move, y, z_robot_move):
                self.get_logger().error(f'Cup {i}: move to cup failed. Skipping.')
                continue

            # Step 3: grip the cup
            if not self._grip():
                self.get_logger().error(f'Cup {i}: grip failed. Skipping.')
                continue

            # Step 5: release the cup
            # self._release()

            self.get_logger().info(f'Cup {i} pickup complete.')

        self.get_logger().info('All cups processed.')
        with self._lock:
            self._processing = False
            self._done = True

    # ── /move_robot_result callback ─────────────────────────────────────

    def _on_move_result(self, msg: Bool) -> None:
        self.get_logger().debug(f'Received /move_robot_result: {msg.data}')
        self._move_success = msg.data
        self._move_done.set()

    def _on_carriage_result(self, msg: Bool) -> None:
        self.get_logger().debug(f'Received /move_carriage_result: {msg.data}')
        self._carriage_success = msg.data
        self._carriage_done.set()

    def _on_lift_result(self, msg: Bool) -> None:
        self.get_logger().debug(f'Received /move_lift_result: {msg.data}')
        self._lift_success = msg.data
        self._lift_done.set()

    # ── movement (talks to moving_node via topics) ───────────────────────

    def _move_carriage(self, x: float) -> bool:
        """Send a carriage position goal to moving_node and wait for confirmation."""
        self._carriage_done.clear()
        self._carriage_success = False
        self.get_logger().info(f'Carriage goal → {x}')
        self._move_carriage_pub.publish(Float32(data=float(x)))

        self.get_logger().debug(
            f'Waiting for /move_carriage_result (timeout={MOVE_TIMEOUT_SEC}s) ...')
        if not self._carriage_done.wait(timeout=MOVE_TIMEOUT_SEC):
            self.get_logger().error(
                f'Carriage move timed out after {MOVE_TIMEOUT_SEC}s!')
            return False

        if not self._carriage_success:
            self.get_logger().error(f'Carriage move to {x:.3f} failed!')
            return False

        self.get_logger().info(f'Carriage move to {x:.3f} complete.')
        return True

    def _move_lift(self, z: float) -> bool:
        """Send a lift position goal to moving_node and wait for confirmation."""
        self._lift_done.clear()
        self._lift_success = False
        self.get_logger().info(f'Lift goal → {z}')
        self._move_lift_pub.publish(Float32(data=float(z)))

        self.get_logger().debug(
            f'Waiting for /move_lift_result (timeout={MOVE_TIMEOUT_SEC}s) ...')
        if not self._lift_done.wait(timeout=MOVE_TIMEOUT_SEC):
            self.get_logger().error(
                f'Lift move timed out after {MOVE_TIMEOUT_SEC}s!')
            return False

        if not self._lift_success:
            self.get_logger().error(f'Lift move to {z:.3f} failed!')
            return False

        self.get_logger().info(f'Lift move to {z:.3f} complete.')
        return True

    def _move_robot(self, x: float, y: float, z: float) -> bool:
        """Publish goal to /move_robot_goal and block until /move_robot_result."""
        goal = Pose()
        goal.position.x = x
        goal.position.y = y
        goal.position.z = z

        self._move_done.clear()
        self._move_success = False
        self.get_logger().info(
            f'Sending move goal ({x:.3f}, {y:.3f}, {z:.3f}) ...')
        self._move_goal_pub.publish(goal)

        self.get_logger().debug(
            f'Waiting for /move_robot_result (timeout={MOVE_TIMEOUT_SEC}s) ...')
        if not self._move_done.wait(timeout=MOVE_TIMEOUT_SEC):
            self.get_logger().error(
                f'Move timed out after {MOVE_TIMEOUT_SEC}s!')
            return False

        if not self._move_success:
            self.get_logger().error(
                f'Move to ({x:.3f}, {y:.3f}, {z:.3f}) failed!')
            return False

        self.get_logger().info(
            f'Move to ({x:.3f}, {y:.3f}, {z:.3f}) complete.')
        return True

    # ── gripper (via robotiq_gripper_controller action) ─────────────────

    def _send_gripper_goal(self, position: float, label: str) -> bool:
        """Send a GripperCommand goal and block until it finishes."""
        self.get_logger().debug(
            f'{label}: waiting for gripper action server ...')
        if not self._gripper_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Gripper action server not available!')
            return False
        self.get_logger().debug(f'{label}: gripper action server is ready.')

        goal = GripperCommandAction.Goal()
        goal.command.position = position
        goal.command.max_effort = GRIPPER_MAX_EFFORT

        self._grip_done.clear()
        self._grip_success = False

        self.get_logger().info(f'{label} (position={position:.2f}) ...')
        future = self._gripper_client.send_goal_async(goal)
        future.add_done_callback(self._gripper_goal_response)

        self.get_logger().debug(
            f'{label}: waiting for result (timeout={GRIP_TIMEOUT_SEC}s) ...')
        if not self._grip_done.wait(timeout=GRIP_TIMEOUT_SEC):
            self.get_logger().error(f'{label} timed out after {GRIP_TIMEOUT_SEC}s!')
            return False

        self.get_logger().debug(f'{label}: result received, success={self._grip_success}')
        return self._grip_success

    def _gripper_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Gripper goal rejected.')
            self._grip_done.set()
            return
        self.get_logger().debug('Gripper goal accepted. Waiting for execution ...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._gripper_result)

    def _gripper_result(self, future) -> None:
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._grip_success = True
            self.get_logger().debug('Gripper action succeeded.')
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

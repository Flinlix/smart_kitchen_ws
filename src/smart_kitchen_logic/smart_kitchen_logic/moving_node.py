#!/usr/bin/env python3
"""Moving node: receives Cartesian goals and moves the Kinova Gen3 arm.

Uses ikpy for inverse kinematics (Cartesian → joint angles) and sends
the result to the joint_trajectory_controller via FollowJointTrajectory.

Topics subscribed:
  /move_robot_goal                   (geometry_msgs/Pose)    – target end-effector position
  /move_carriage_goal                (std_msgs/Float32)      – carriage position goal
  /move_lift_goal                    (std_msgs/Float32)      – lift position goal
  /robot_description                 (std_msgs/String)       – URDF for IK chain
  /joint_states                      (sensor_msgs/JointState)

Topics published:
  /move_robot_result                 (std_msgs/Bool)         – True when arm move is done
  /move_carriage_result              (std_msgs/Bool)         – True after carriage setpoint forwarded
  /move_lift_result                  (std_msgs/Bool)         – True after lift setpoint forwarded
  /elmo/id1/carriage/position/set    (std_msgs/Float32)      – carriage position setpoint
  /elmo/id1/lift/position/set        (std_msgs/Float32)      – lift position setpoint

Action client:
  /joint_trajectory_controller/follow_joint_trajectory
"""

import math
import tempfile
import threading
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool, Float32, String
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from ikpy.chain import Chain

ARM_JOINT_NAMES = [
    'joint_1', 'joint_2', 'joint_3',
    'joint_4', 'joint_5', 'joint_6',
]

# Number of links ikpy creates for the arm (base + 6 joints + end_effector)
ARM_CHAIN_LEN = 8

ACTIVE_MASK = [False, True, True, True, True, True, True, False]

TRAJECTORY_DURATION_SEC = 5
ACTION_TIMEOUT_SEC = 30.0


class MovingNode(Node):
    def __init__(self):
        super().__init__('moving_node')

        self._chain: Chain | None = None
        self._current_joints: dict[str, float] = {n: 0.0 for n in ARM_JOINT_NAMES}

        self._action_done = threading.Event()
        self._action_success = False

        # URDF → IK chain
        desc_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            String, '/robot_description', self._on_robot_description, desc_qos)

        # Current joint positions (IK seed)
        self.create_subscription(
            JointState, '/joint_states', self._on_joint_states, 10)

        # Goal / result topics
        self.create_subscription(Pose, '/move_robot_goal', self._on_goal, 10)
        self._result_pub = self.create_publisher(Bool, '/move_robot_result', 10)

        # Carriage and lift: receive goals, forward to ELMO drives, publish result
        self._carriage_pub = self.create_publisher(
            Float32, '/elmo/id1/carriage/position/set', 10)
        self._lift_pub = self.create_publisher(
            Float32, '/elmo/id1/lift/position/set', 10)
        self._carriage_result_pub = self.create_publisher(
            Bool, '/move_carriage_result', 10)
        self._lift_result_pub = self.create_publisher(
            Bool, '/move_lift_result', 10)
        self.create_subscription(
            Float32, '/move_carriage_goal', self._on_carriage_goal, 10)
        self.create_subscription(
            Float32, '/move_lift_goal', self._on_lift_goal, 10)

        # Joint trajectory action client
        self._traj_client = ActionClient(
            self, FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory')

        self.get_logger().info(
            'Moving node started.  Waiting for /robot_description ...')

    # ── URDF → ikpy chain ────────────────────────────────────────────────

    def _on_robot_description(self, msg: String) -> None:
        if self._chain is not None:
            self.get_logger().debug('Robot description received again – chain already built, skipping.')
            return

        self.get_logger().debug(
            f'Received /robot_description ({len(msg.data)} chars). Building IK chain ...')
        try:
            self._chain = self._build_chain(msg.data)
            chain_links = [(i, l.name) for i, l in enumerate(self._chain.links)]
            self.get_logger().info(
                f'IK chain ready  ({len(self._chain.links)} links, '
                f'active joints: {[l.name for l, a in zip(self._chain.links, ACTIVE_MASK) if a]})')
            self.get_logger().debug(f'Full chain link list: {chain_links}')

            fk_zero = self._chain.forward_kinematics([0.0] * ARM_CHAIN_LEN)[:3, 3]
            self.get_logger().debug(
                f'FK at zero config: ({fk_zero[0]:.4f}, {fk_zero[1]:.4f}, {fk_zero[2]:.4f})')
        except Exception as e:
            self.get_logger().error(f'Failed to build IK chain: {e}')

    @staticmethod
    def _build_chain(urdf_string: str) -> Chain:
        """Parse URDF, patch continuous joints, and return a truncated ikpy chain."""
        tree = ET.ElementTree(ET.fromstring(urdf_string))
        for joint in tree.iter('joint'):
            if joint.get('type') == 'continuous':
                joint.set('type', 'revolute')
                if joint.find('limit') is None:
                    ET.SubElement(joint, 'limit', {
                        'lower': str(-2 * math.pi),
                        'upper': str(2 * math.pi),
                        'effort': '100',
                        'velocity': '1.0',
                    })

        with tempfile.NamedTemporaryFile(
                suffix='.urdf', mode='wb', delete=False) as f:
            tree.write(f)
            urdf_path = f.name

        full_chain = Chain.from_urdf_file(
            urdf_path, base_elements=['base_link'], name='kinova_arm')

        return Chain(
            name='kinova_arm',
            links=full_chain.links[:ARM_CHAIN_LEN],
            active_links_mask=ACTIVE_MASK,
        )

    # ── joint-state tracking ─────────────────────────────────────────────

    def _on_joint_states(self, msg: JointState) -> None:
        updated = {}
        for name, pos in zip(msg.name, msg.position):
            if name in self._current_joints:
                self._current_joints[name] = pos
                updated[name] = pos
        if updated:
            self.get_logger().debug(
                f'Joint states updated: {{{", ".join(f"{k}: {v:.3f}" for k, v in updated.items())}}}')

    # ── goal handling ────────────────────────────────────────────────────

    def _on_goal(self, msg: Pose) -> None:
        x, y, z = msg.position.x, msg.position.y, msg.position.z

        if self._chain is None:
            self.get_logger().error('IK chain not loaded yet – ignoring goal.')
            self._result_pub.publish(Bool(data=False))
            return

        self.get_logger().info(f'Goal received: ({x:.3f}, {y:.3f}, {z:.3f})')

        seed = self._build_seed()
        self.get_logger().debug(
            f'IK seed: {[f"{v:.3f}" for v in seed]}')
        target = np.array([x, y, z])

        self.get_logger().debug('Running inverse kinematics ...')
        try:
            ik = self._chain.inverse_kinematics(
                target_position=target, initial_position=seed)
        except Exception as e:
            self.get_logger().error(f'IK failed: {e}')
            self._result_pub.publish(Bool(data=False))
            return

        joint_positions = [ik[i] for i in range(ARM_CHAIN_LEN) if ACTIVE_MASK[i]]

        fk = self._chain.forward_kinematics(ik)[:3, 3]
        err = np.linalg.norm(fk - target)
        self.get_logger().info(
            f'IK solution (error {err:.4f} m): '
            f'{[f"{v:.3f}" for v in joint_positions]}')
        self.get_logger().debug(
            f'FK verification: ({fk[0]:.4f}, {fk[1]:.4f}, {fk[2]:.4f})')

        if err > 0.05:
            self.get_logger().error(
                f'IK error too large ({err:.3f} m) – aborting move.')
            self._result_pub.publish(Bool(data=False))
            return

        self.get_logger().debug('IK accepted. Spawning trajectory execution thread.')
        threading.Thread(
            target=self._execute, args=(joint_positions,), daemon=True).start()

    def _build_seed(self) -> list[float]:
        """Map current joint states into the ikpy solution vector."""
        seed = [0.0] * ARM_CHAIN_LEN
        joint_names_in_chain = [
            link.name for link in self._chain.links
        ]
        for i, name in enumerate(joint_names_in_chain):
            if name in self._current_joints:
                seed[i] = self._current_joints[name]
        return seed

    # ── trajectory execution ─────────────────────────────────────────────

    def _execute(self, positions: list[float]) -> None:
        self.get_logger().debug('Waiting for trajectory action server ...')
        if not self._traj_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Trajectory action server not available!')
            self._result_pub.publish(Bool(data=False))
            return
        self.get_logger().debug('Trajectory action server is ready.')

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM_JOINT_NAMES)

        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.time_from_start = Duration(sec=TRAJECTORY_DURATION_SEC, nanosec=0)
        goal.trajectory.points.append(pt)

        self._action_done.clear()
        self._action_success = False

        self.get_logger().info('Sending trajectory ...')
        self.get_logger().debug(
            f'Trajectory joints: {list(ARM_JOINT_NAMES)}, '
            f'positions: {[f"{p:.3f}" for p in positions]}, '
            f'duration: {TRAJECTORY_DURATION_SEC}s')
        future = self._traj_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

        self.get_logger().debug(
            f'Waiting for trajectory result (timeout={ACTION_TIMEOUT_SEC}s) ...')
        if not self._action_done.wait(timeout=ACTION_TIMEOUT_SEC):
            self.get_logger().error(
                f'Trajectory timed out after {ACTION_TIMEOUT_SEC}s!')
            self._result_pub.publish(Bool(data=False))
            return

        self.get_logger().info(
            'Move ' + ('succeeded.' if self._action_success else 'FAILED.'))
        self.get_logger().debug(f'Publishing /move_robot_result: {self._action_success}')
        self._result_pub.publish(Bool(data=self._action_success))

    def _on_goal_response(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Trajectory goal rejected by action server.')
            self._action_done.set()
            return
        self.get_logger().debug('Trajectory goal accepted. Executing ...')
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        status = future.result().status
        self._action_success = (status == GoalStatus.STATUS_SUCCEEDED)
        if self._action_success:
            self.get_logger().debug('Trajectory action succeeded.')
        else:
            self.get_logger().warn(f'Trajectory ended with status {status}.')
        self._action_done.set()

    # ── carriage / lift control ──────────────────────────────────────────

    def _on_carriage_goal(self, msg: Float32) -> None:
        """Forward a carriage position goal to the ELMO drive and confirm."""
        self._carriage_pub.publish(Float32(data=msg.data))
        self.get_logger().info(f'Carriage setpoint → {msg.data}')
        self._carriage_result_pub.publish(Bool(data=True))

    def _on_lift_goal(self, msg: Float32) -> None:
        """Forward a lift position goal to the ELMO drive and confirm."""
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

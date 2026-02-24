#!/usr/bin/env python3
"""Tests for the MovingNode (IK chain, inverse kinematics, node behaviour)."""

import subprocess
import threading

import numpy as np
import pytest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState

from smart_kitchen_logic.moving_node import (
    MovingNode,
    ARM_JOINT_NAMES,
    ARM_CHAIN_LEN,
    ACTIVE_MASK,
)


# ── fixtures ─────────────────────────────────────────────────────────────

@pytest.fixture(scope='session')
def urdf_string():
    """Generate the full robot URDF via xacro (once per test session)."""
    src = (
        'src/smart_kitchen_description/urdf/kitchen_robot.xacro'
    )
    result = subprocess.run(
        ['xacro', src, 'sim_gazebo:=true',
         'simulation_controllers:=config/sim_ros2_controllers.yaml'],
        capture_output=True, text=True,
        cwd='/home/ros2-jazzy/workspace/smart_kitchen_ws',
    )
    assert result.returncode == 0, f'xacro failed: {result.stderr}'
    return result.stdout


@pytest.fixture(scope='session')
def ik_chain(urdf_string):
    """Build the truncated IK chain used by MovingNode."""
    return MovingNode._build_chain(urdf_string)


@pytest.fixture(scope='module')
def ros_context():
    """Initialise / shutdown rclpy once per test module."""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture()
def moving_node(ros_context):
    """Create a MovingNode instance (chain NOT loaded yet)."""
    node = MovingNode()
    yield node
    node.destroy_node()


# ── 1. chain construction ───────────────────────────────────────────────

class TestBuildChain:
    def test_chain_has_correct_link_count(self, ik_chain):
        assert len(ik_chain.links) == ARM_CHAIN_LEN

    def test_active_mask_length_matches(self, ik_chain):
        assert len(ACTIVE_MASK) == ARM_CHAIN_LEN

    def test_six_active_joints(self, ik_chain):
        assert sum(ACTIVE_MASK) == 6

    def test_active_joint_names(self, ik_chain):
        active_names = [
            link.name for link, active in zip(ik_chain.links, ACTIVE_MASK)
            if active
        ]
        assert active_names == ARM_JOINT_NAMES

    def test_first_link_is_fixed_base(self, ik_chain):
        assert ACTIVE_MASK[0] is False

    def test_last_link_is_fixed_end_effector(self, ik_chain):
        assert ACTIVE_MASK[-1] is False


# ── 2. forward kinematics sanity ────────────────────────────────────────

class TestForwardKinematics:
    def test_zero_config_position(self, ik_chain):
        """At zero joint angles the end effector should be roughly above the base."""
        fk = ik_chain.forward_kinematics([0.0] * ARM_CHAIN_LEN)
        pos = fk[:3, 3]
        assert abs(pos[0]) < 0.1, f'x should be near 0, got {pos[0]:.3f}'
        assert abs(pos[1]) < 0.1, f'y should be near 0, got {pos[1]:.3f}'
        assert pos[2] > 0.8, f'z should be > 0.8 m, got {pos[2]:.3f}'

    def test_fk_returns_4x4_matrix(self, ik_chain):
        fk = ik_chain.forward_kinematics([0.0] * ARM_CHAIN_LEN)
        assert fk.shape == (4, 4)


# ── 3. inverse kinematics accuracy ──────────────────────────────────────

REACHABLE_TARGETS = [
    (0.3, 0.0, 0.3),
    (0.3, 0.2, 0.5),
    (-0.2, 0.1, 0.6),
    (0.2, -0.2, 0.8),
    (0.4, 0.0, 0.6),
]


class TestInverseKinematics:
    @pytest.mark.parametrize('target', REACHABLE_TARGETS,
                             ids=[f'({x},{y},{z})' for x, y, z in REACHABLE_TARGETS])
    def test_ik_reaches_target(self, ik_chain, target):
        target_arr = np.array(target)
        ik = ik_chain.inverse_kinematics(target_position=target_arr)
        fk = ik_chain.forward_kinematics(ik)[:3, 3]
        err = np.linalg.norm(fk - target_arr)
        assert err < 0.01, (
            f'IK error {err:.4f} m exceeds 1 cm for target {target}')

    def test_ik_with_seed_improves_or_matches(self, ik_chain):
        """Using the current config as seed should not degrade accuracy."""
        target = np.array([0.3, 0.0, 0.5])
        ik_no_seed = ik_chain.inverse_kinematics(target_position=target)
        ik_seeded = ik_chain.inverse_kinematics(
            target_position=target, initial_position=ik_no_seed)

        fk_seeded = ik_chain.forward_kinematics(ik_seeded)[:3, 3]
        err = np.linalg.norm(fk_seeded - target)
        assert err < 0.01

    def test_ik_returns_correct_length(self, ik_chain):
        target = np.array([0.3, 0.0, 0.5])
        ik = ik_chain.inverse_kinematics(target_position=target)
        assert len(ik) == ARM_CHAIN_LEN

    def test_ik_inactive_joints_stay_zero(self, ik_chain):
        """Base link and end-effector entries should remain 0."""
        target = np.array([0.3, 0.0, 0.5])
        ik = ik_chain.inverse_kinematics(target_position=target)
        assert ik[0] == pytest.approx(0.0)
        assert ik[ARM_CHAIN_LEN - 1] == pytest.approx(0.0)


# ── 4. node-level: joint-state tracking ─────────────────────────────────

class TestJointStateTracking:
    def test_updates_known_joints(self, moving_node):
        msg = JointState()
        msg.name = list(ARM_JOINT_NAMES)
        msg.position = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        moving_node._on_joint_states(msg)

        for name, expected in zip(ARM_JOINT_NAMES, msg.position):
            assert moving_node._current_joints[name] == pytest.approx(expected)

    def test_ignores_unknown_joints(self, moving_node):
        msg = JointState()
        msg.name = ['unknown_joint_99']
        msg.position = [9.9]
        moving_node._on_joint_states(msg)

        assert 'unknown_joint_99' not in moving_node._current_joints

    def test_partial_update(self, moving_node):
        moving_node._current_joints['joint_1'] = 0.0

        msg = JointState()
        msg.name = ['joint_1']
        msg.position = [1.23]
        moving_node._on_joint_states(msg)

        assert moving_node._current_joints['joint_1'] == pytest.approx(1.23)
        assert moving_node._current_joints['joint_2'] == pytest.approx(0.0)


# ── 5. node-level: build_seed ────────────────────────────────────────────

class TestBuildSeed:
    def test_seed_maps_joints_correctly(self, moving_node, ik_chain):
        moving_node._chain = ik_chain
        moving_node._current_joints = {n: float(i) for i, n in enumerate(ARM_JOINT_NAMES)}

        seed = moving_node._build_seed()
        assert len(seed) == ARM_CHAIN_LEN

        for i, link in enumerate(ik_chain.links):
            if link.name in ARM_JOINT_NAMES:
                expected = float(ARM_JOINT_NAMES.index(link.name))
                assert seed[i] == pytest.approx(expected)

    def test_seed_base_and_ee_are_zero(self, moving_node, ik_chain):
        moving_node._chain = ik_chain
        seed = moving_node._build_seed()
        assert seed[0] == 0.0
        assert seed[-1] == 0.0


# ── 6. node-level: goal without chain loaded ─────────────────────────────

class TestGoalWithoutChain:
    def test_publishes_false(self, moving_node):
        """Goal received before IK chain is loaded must publish False."""
        assert moving_node._chain is None

        received = threading.Event()
        result_value = []

        sub_node = Node('test_listener')
        sub_node.create_subscription(
            Bool, '/move_robot_result',
            lambda msg: (result_value.append(msg.data), received.set()),
            10,
        )

        goal = Pose()
        goal.position.x = 0.3
        goal.position.y = 0.0
        goal.position.z = 0.5
        moving_node._on_goal(goal)

        for _ in range(50):
            rclpy.spin_once(moving_node, timeout_sec=0.01)
            rclpy.spin_once(sub_node, timeout_sec=0.01)
            if received.is_set():
                break

        sub_node.destroy_node()

        assert received.is_set(), 'No result published within timeout'
        assert result_value[0] is False

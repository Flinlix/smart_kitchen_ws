#!/usr/bin/env python3
"""Tests for the MovingNode (IK chain, inverse kinematics, node behaviour)."""

import subprocess
import threading

import numpy as np
import pytest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool, Float64MultiArray
from sensor_msgs.msg import JointState

from smart_kitchen_logic.moving_node import MovingNode, ARM_JOINT_NAMES
from smart_kitchen_logic.planning_node import (
    PlanningNode,
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
    """Build the truncated IK chain used by PlanningNode (same chain logic)."""
    return PlanningNode._build_chain(urdf_string)


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

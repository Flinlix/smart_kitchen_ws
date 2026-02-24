"""Helpers for querying the gripper world position.

Combines the tf2 arm transform (base_link → robotiq_85_base_link) with
carriage (X) and lift (Z) offsets from the linear actuators.

Usage inside any spinning Node::

    tracker = PositionTracker(node)
    # after data starts flowing …
    x, y, z = tracker.get_gripper_tf()
    x, y, z = tracker.get_full_position()

The two core functions (``lookup_gripper_tf`` and ``compute_full_position``)
can also be used standalone if you manage the tf buffer / subscriptions
yourself.
"""

from __future__ import annotations

from typing import Tuple

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
import tf2_ros
from std_msgs.msg import Float32


# ---------------------------------------------------------------------------
# Pure helper functions
# ---------------------------------------------------------------------------

def lookup_gripper_tf(
    tf_buffer: tf2_ros.Buffer,
    timeout_sec: float = 0.0,
    source_frame: str = "base_link",
    target_frame: str = "robotiq_85_base_link",
) -> Tuple[float, float, float]:
    """Return (x, y, z) translation of *target_frame* in *source_frame*."""
    timeout = Duration(seconds=timeout_sec) if timeout_sec > 0 else Duration()
    ts = tf_buffer.lookup_transform(source_frame, target_frame, Time(), timeout=timeout)
    t = ts.transform.translation
    return (t.x, t.y, t.z)


def compute_full_position(
    gripper_xyz: Tuple[float, float, float],
    carriage_x: float,
    lift_z: float,
) -> Tuple[float, float, float]:
    """Add carriage X offset and lift Z offset to a gripper position."""
    return (gripper_xyz[0] + carriage_x, gripper_xyz[1], gripper_xyz[2] + lift_z)


# ---------------------------------------------------------------------------
# Convenience class – wires up tf2 + topic subscriptions on a Node
# ---------------------------------------------------------------------------

class PositionTracker:
    """Tracks gripper world position including carriage / lift offsets.

    Subscribes to:
      /elmo/id1/carriage/position/get  (std_msgs/Float32) – carriage X
      /elmo/id1/lift/position/get      (std_msgs/Float32) – lift Z
    """

    def __init__(self, node: Node) -> None:
        self._node = node
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, node)

        self._carriage_x: float = 0.0
        self._lift_z: float = 0.0

        node.create_subscription(
            Float32, "/elmo/id1/carriage/position/get", self._on_carriage, 10,
        )
        node.create_subscription(
            Float32, "/elmo/id1/lift/position/get", self._on_lift, 10,
        )

    # -- callbacks ----------------------------------------------------------

    def _on_carriage(self, msg: Float32) -> None:
        self._carriage_x = msg.data

    def _on_lift(self, msg: Float32) -> None:
        self._lift_z = msg.data

    # -- public API ---------------------------------------------------------

    @property
    def tf_buffer(self) -> tf2_ros.Buffer:
        return self._tf_buffer

    @property
    def carriage_x(self) -> float:
        return self._carriage_x

    @property
    def lift_z(self) -> float:
        return self._lift_z

    def get_gripper_tf(self, timeout_sec: float = 0.0) -> Tuple[float, float, float]:
        """(x, y, z) of the gripper w.r.t. base_link from tf2 alone."""
        return lookup_gripper_tf(self._tf_buffer, timeout_sec)

    def get_full_position(self, timeout_sec: float = 0.0) -> Tuple[float, float, float]:
        """Gripper position with carriage (X) and lift (Z) offsets applied."""
        return compute_full_position(
            self.get_gripper_tf(timeout_sec), self._carriage_x, self._lift_z,
        )


# ---------------------------------------------------------------------------
# main – spin a node that periodically prints the full gripper position
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = Node("tf_transformation")
    tracker = PositionTracker(node)

    def _timer_cb():
        try:
            x, y, z = tracker.get_full_position()
            node.get_logger().info(f"Full gripper position: x={x:.4f}  y={y:.4f}  z={z:.4f}")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            node.get_logger().warn("TF not available yet ...")

    node.create_timer(1.0, _timer_cb)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

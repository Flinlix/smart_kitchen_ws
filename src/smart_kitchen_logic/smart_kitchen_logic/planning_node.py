#!/usr/bin/env python3
"""Planning node: aruco tracking, IK computation, and motion orchestration.

Subscribes:
  /aruco_distances                  (std_msgs/Float32MultiArray)       – [marker_id, distance, x, y, z, ...] in camera frame
  /robot_description                (std_msgs/String)                  – URDF for IK chain
  /joint_states                     (sensor_msgs/JointState)           – current joint positions
  /elmo/id1/carriage/position/get   (std_msgs/Float32)                 – current carriage y
  /move_robot_result                (std_msgs/Bool)
  /move_carriage_result             (std_msgs/Bool)
  /move_lift_result                 (std_msgs/Bool)

Publishes:
  /move_robot_goal      (std_msgs/Float64MultiArray)       – joint angle goals
  /move_carriage_goal   (std_msgs/Float32)
  /move_lift_goal       (std_msgs/Float32)
"""

import math
import tempfile
import threading
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool, Float32, Float32MultiArray, Float64MultiArray, String
from sensor_msgs.msg import JointState
from control_msgs.action import GripperCommand as GripperCommandAction
from smart_kitchen_interfaces.action import ExecuteCommand
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_pose
from ikpy.chain import Chain


MOVE_TIMEOUT_SEC = 30.0
GRIP_TIMEOUT_SEC = 15.0
EXECUTE_COMMAND_TIMEOUT_SEC = 120.0

GRIPPER_CLOSED = 0.5
GRIPPER_OPEN = 0.0
GRIPPER_MAX_EFFORT = 50.0

ARM_JOINT_NAMES = [
    'joint_1', 'joint_2', 'joint_3',
    'joint_4', 'joint_5', 'joint_6',
]

# We have 6 joints, but the IK chain needs 8 links to work.
# The first link is the base_link, which is fixed.
# The last link is the end_effector_link, which is fixed.
ARM_CHAIN_LEN = 8
ACTIVE_MASK = [False, True, True, True, True, True, True, False]


class PlanningNode(Node):
    def __init__(self):
        super().__init__('planning_node')

        self._lock = threading.Lock()

        # ── TF2 for pose transformations ──────────────────────────────────
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ── IK chain from URDF ────────────────────────────────────────────
        self._chain: Chain | None = None
        self._current_joints: dict[str, float] = {n: 0.0 for n in ARM_JOINT_NAMES}

        desc_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            String, '/robot_description', self._on_robot_description, desc_qos)
        self.create_subscription(
            JointState, '/joint_states', self._on_joint_states, 10)

        # ── Aruco tag tracking ────────────────────────────────────────────
        # Source: /aruco_distances Float32MultiArray [marker_id, distance, x, y, z, ...] in camera frame
        # {marker_id: {'pose': Pose (base_link), 'distance': float, 'global_x': float}}
        self._aruco_tags: dict[int, dict] = {}

        self.declare_parameter("aruco_distances_topic", "/aruco_distances")
        self.declare_parameter("aruco_distances_frame", "robotiq_85_base_link") # TODO: Change here to camera
        aruco_topic = self.get_parameter("aruco_distances_topic").value
        self._aruco_distances_frame = self.get_parameter("aruco_distances_frame").value

        self.create_subscription(
            Float32MultiArray, aruco_topic, self._on_aruco_distances, 10)

        # ── Robot movement (joint goals → moving_node) ────────────────────
        self._move_goal_pub = self.create_publisher(
            Float64MultiArray, '/move_robot_goal', 10)
        self._move_done = threading.Event()
        self._move_success = False
        self.create_subscription(
            Bool, '/move_robot_result', self._on_move_result, 10)

        # ── Carriage (Y-axis) ─────────────────────────────────────────────
        self._move_carriage_pub = self.create_publisher(
            Float32, '/move_carriage_goal', 10)
        self._carriage_done = threading.Event()
        self._carriage_success = False
        self.create_subscription(
            Bool, '/move_carriage_result', self._on_carriage_result, 10)

        # ── Carriage position feedback (for global aruco coords) ──────────
        self._carriage_x = 0.0
        self._carriage_position_received = threading.Event()
        self.create_subscription(
            Float32, '/elmo/id1/carriage/position/get',
            self._on_carriage_position, 10)

        # Sync carriage_y to current position (wait for first message or timeout)
        for _ in range(200):
            rclpy.spin_once(self, timeout_sec=0.01)
            if self._carriage_position_received.is_set():
                self.get_logger().info(f'Carriage position synced: y={self._carriage_x:.3f}')
                break
        else:
            self.get_logger().warn(
                'No carriage position received within 2s; using 0.0 until /elmo/id1/carriage/position/get publishes')

        # ── Lift (Z-axis) ─────────────────────────────────────────────────
        self._move_lift_pub = self.create_publisher(
            Float32, '/move_lift_goal', 10)
        self._lift_done = threading.Event()
        self._lift_success = False
        self.create_subscription(
            Bool, '/move_lift_result', self._on_lift_result, 10)

        # ── Gripper action client ─────────────────────────────────────────
        self._gripper_client = ActionClient(
            self, GripperCommandAction,
            '/robotiq_gripper_controller/gripper_cmd')
        self._grip_done = threading.Event()
        self._grip_success = False

        # ── Execute command action client (command_executor) ──────────────
        self._execute_command_client = ActionClient(
            self, ExecuteCommand, '/execute_command')
        self._execute_command_done = threading.Event()
        self._execute_command_success = False

        # ── Main loop (own callback group so blocking doesn't starve subs) ─
        self._processed_tags: set[int] = set()
        self._timer_cb_group = MutuallyExclusiveCallbackGroup()
        self._main_timer = self.create_timer(
            2.0, self._main_loop, callback_group=self._timer_cb_group)

        self.get_logger().info('Planning node started.')

    # ═══════════════════════════════════════════════════════════════════════
    # URDF / IK chain
    # ═══════════════════════════════════════════════════════════════════════

    def _on_robot_description(self, msg: String) -> None:
        if self._chain is not None:
            return
        try:
            self._chain = self._build_chain(msg.data)
            active = [l.name for l, a in zip(self._chain.links, ACTIVE_MASK) if a]
            self.get_logger().info(
                f'IK chain ready ({len(self._chain.links)} links, '
                f'active: {active})')
        except Exception as e:
            self.get_logger().error(f'Failed to build IK chain: {e}')

    @staticmethod
    def _build_chain(urdf_string: str) -> Chain:
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

    def _on_joint_states(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            if name in self._current_joints:
                self._current_joints[name] = pos

    def _build_ik_seed(self) -> list[float]:
        seed = [0.0] * ARM_CHAIN_LEN
        for i, link in enumerate(self._chain.links):
            if link.name in self._current_joints:
                seed[i] = self._current_joints[link.name]
        return seed

    # ═══════════════════════════════════════════════════════════════════════
    # Aruco tag tracking (/aruco_distances: [marker_id, distance, x, y, z, ...])
    # ═══════════════════════════════════════════════════════════════════════

    def _on_aruco_distances(self, msg: Float32MultiArray):
        """Update tracked tags from /aruco_distances (camera frame). Prefer closer detections."""
        "Here we also do the transformation from camera/gripper frame to base_link frame"
        data = msg.data
        if len(data) % 5 != 0:
            self.get_logger().warn(
                f'Aruco distances length {len(data)} not divisible by 5; ignoring message.')
            return

        source_frame = self._aruco_distances_frame
        with self._lock:
            for i in range(0, len(data), 5):
                mid = int(data[i])
                dist = float(data[i + 1])
                x, y, z = float(data[i + 2]), float(data[i + 3]), float(data[i + 4])

                existing = self._aruco_tags.get(mid)
                if existing is not None and dist >= existing['distance']:
                    continue

                try:
                    tf = self._tf_buffer.lookup_transform(
                        'base_link', source_frame, rclpy.time.Time())
                    pose_camera = Pose()
                    pose_camera.position.x = x
                    pose_camera.position.y = y
                    pose_camera.position.z = z
                    pose_camera.orientation.w = 1.0
                    pose_base = do_transform_pose(pose_camera, tf)
                    self._aruco_tags[mid] = {
                        'pose': pose_base,
                        'distance': dist,
                        # Carriage moves in X; base_link moves with it.
                        # If carriage value decreases when moving left but base_link X increases left,
                        # use minus so the carriage goal has the correct sign.
                        'global_x': self._carriage_x - pose_base.position.x,
                    }
                    self.get_logger().info(f'Aruco {mid}: calculated pose_base = {pose_base}')
                    self.get_logger().info(
                        f'Aruco {mid}: updated (dist={dist:.3f}m)')
                except TransformException as e:
                    self.get_logger().warn(
                        f'TF failed for marker {mid}: {e}')

    def get_aruco_tags(self) -> dict[int, dict]:
        """Thread-safe snapshot of all tracked aruco tags."""
        with self._lock:
            return {k: dict(v) for k, v in self._aruco_tags.items()}

    # ═══════════════════════════════════════════════════════════════════════
    # Inverse kinematics
    # ═══════════════════════════════════════════════════════════════════════

    def compute_ik(self, pose: Pose) -> list[float] | None:
        """Compute joint angles for a base_link-frame pose. Returns None on failure."""
        if self._chain is None:
            self.get_logger().error('IK chain not ready')
            return None

        target = np.array([pose.position.x, pose.position.y, pose.position.z])
        # Using the current pose as the seed makes IK converge to a solution near the current configuration and avoids jumps to other joint-space solutions.
        seed = self._build_ik_seed()

        try:
            ik = self._chain.inverse_kinematics(
                target_position=target, initial_position=seed)
        except Exception as e:
            self.get_logger().error(f'IK failed: {e}')
            return None

        fk = self._chain.forward_kinematics(ik)[:3, 3]
        err = np.linalg.norm(fk - target)
        if err > 0.05:
            self.get_logger().error(f'IK error too large ({err:.3f}m)')
            return None

        joints = [ik[i] for i in range(ARM_CHAIN_LEN) if ACTIVE_MASK[i]]
        self.get_logger().info(
            f'IK solution (err={err:.4f}m): {[f"{j:.3f}" for j in joints]}')
        return joints

    def move_to_pose(self, pose: Pose) -> bool:
        """Compute IK for a base_link pose and send joint goals to moving_node."""
        joints = self.compute_ik(pose)
        if joints is None:
            return False
        return self._send_joint_goal(joints)

    def move_to_aruco(self, marker_id: int) -> bool:
        """
        MAIN MOVE TO ARUCO METHOD!!
        
        Align carriage to the tag's global x, 
        move the arm to the tag, 
        grip the tag, 
        then return home.
        """
        with self._lock:
            entry = self._aruco_tags.get(marker_id)

        if entry is None or entry['pose'] is None:
            self.get_logger().error(f'No valid pose for aruco {marker_id}')
            return False

        self._release() # Open gripper
        # Snapshot current arm pose and carriage position so we can return afterwards
        home_joints = [self._current_joints[n] for n in ARM_JOINT_NAMES]
        start_carriage_x = self._carriage_x

        self.get_logger().info(
            f'Moving to aruco {marker_id} '
            f'(global_x={entry["global_x"]:.3f}, dist={entry["distance"]:.3f}m)')

        if not self._move_carriage(entry['global_x']):
            self.get_logger().error(f'Carriage alignment failed for aruco {marker_id}')
            return False

        p = entry['pose'].position
        ik_pose = Pose()
        ik_pose.position.x = 0.0
        ik_pose.position.y = p.y
        ik_pose.position.z = p.z
        ik_pose.orientation = entry['pose'].orientation # maybe the orientation changes when we move the carriage, but i think orientation is never used by IK
        success = self.move_to_pose(ik_pose)

        if success:
            if not self._grip():
                self.get_logger().error(f'Grip failed for aruco {marker_id}')
                success = False

        self.get_logger().info(f'Returning arm to pre-approach pose')
        self._send_joint_goal(home_joints)

        self.get_logger().info(f'Returning carriage to start position ({start_carriage_x:.3f})')
        if not self._move_carriage(start_carriage_x):
            self.get_logger().error('Carriage return to start failed')

        return success

    def move_to_xy(self, x: float, y: float, z: float = 0.3) -> bool:
        """Move end-effector to (x, y, z) in base_link frame. Returns True on success."""
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation.w = 1.0
        return self.move_to_pose(pose)

    # ═══════════════════════════════════════════════════════════════════════
    # Robot movement (joint goals → moving_node)
    # ═══════════════════════════════════════════════════════════════════════

    def _send_joint_goal(self, joints: list[float]) -> bool:
        """Publish joint-angle goal and block until moving_node confirms."""
        msg = Float64MultiArray(data=joints)
        self._move_done.clear()
        self._move_success = False

        self.get_logger().info(
            f'Sending joint goal: {[f"{j:.3f}" for j in joints]}')
        self._move_goal_pub.publish(msg)

        if not self._move_done.wait(timeout=MOVE_TIMEOUT_SEC):
            self.get_logger().error(
                f'Move timed out after {MOVE_TIMEOUT_SEC}s!')
            return False

        if not self._move_success:
            self.get_logger().error('Move failed!')
            return False

        self.get_logger().info('Move complete.')
        return True

    def _on_move_result(self, msg: Bool) -> None:
        self._move_success = msg.data
        self._move_done.set()

    # ═══════════════════════════════════════════════════════════════════════
    # Carriage control (Y-direction)
    # ═══════════════════════════════════════════════════════════════════════

    def _move_carriage(self, x: float) -> bool:
        """Send carriage goal to moving_node and block until done."""
        self._carriage_done.clear()
        self._carriage_success = False
        self.get_logger().info(f'Carriage goal → {x:.3f}')
        self._move_carriage_pub.publish(Float32(data=float(x)))

        if not self._carriage_done.wait(timeout=MOVE_TIMEOUT_SEC):
            self.get_logger().error('Carriage move timed out!')
            return False
        if not self._carriage_success:
            self.get_logger().error(f'Carriage move to {x:.3f} failed!')
            return False

        self.get_logger().info(f'Carriage at {x:.3f}.')
        return True

    def _on_carriage_result(self, msg: Bool) -> None:
        self._carriage_success = msg.data
        self._carriage_done.set()

    def _on_carriage_position(self, msg: Float32) -> None:
        self._carriage_x = msg.data
        self._carriage_position_received.set()

    # ═══════════════════════════════════════════════════════════════════════
    # Lift control (Z-direction)
    # ═══════════════════════════════════════════════════════════════════════

    def _move_lift(self, z: float) -> bool:
        """Send lift goal to moving_node and block until done."""
        self._lift_done.clear()
        self._lift_success = False
        self.get_logger().info(f'Lift goal → {z:.3f}')
        self._move_lift_pub.publish(Float32(data=float(z)))

        if not self._lift_done.wait(timeout=MOVE_TIMEOUT_SEC):
            self.get_logger().error('Lift move timed out!')
            return False
        if not self._lift_success:
            self.get_logger().error(f'Lift move to {z:.3f} failed!')
            return False

        self.get_logger().info(f'Lift at {z:.3f}.')
        return True

    def _on_lift_result(self, msg: Bool) -> None:
        self._lift_success = msg.data
        self._lift_done.set()

    # ═══════════════════════════════════════════════════════════════════════
    # Gripper control
    # ═══════════════════════════════════════════════════════════════════════

    def _send_gripper_goal(self, position: float, label: str) -> bool:
        if not self._gripper_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Gripper action server not available!')
            return False

        goal = GripperCommandAction.Goal()
        goal.command.position = position
        goal.command.max_effort = GRIPPER_MAX_EFFORT

        self._grip_done.clear()
        self._grip_success = False

        self.get_logger().info(f'{label} (position={position:.2f})')
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
        goal_handle.get_result_async().add_done_callback(self._gripper_result)

    def _gripper_result(self, future) -> None:
        status = future.result().status
        self._grip_success = (status == GoalStatus.STATUS_SUCCEEDED)
        self._grip_done.set()

    def _grip(self) -> bool:
        return self._send_gripper_goal(GRIPPER_CLOSED, 'Closing gripper')

    def _release(self) -> bool:
        return self._send_gripper_goal(GRIPPER_OPEN, 'Opening gripper')

    # ═══════════════════════════════════════════════════════════════════════
    # Execute command (command_executor action)
    # ═══════════════════════════════════════════════════════════════════════

    def execute_command(self, command_name: str, cup_id: str = '') -> bool:
        """Send a string goal to the /execute_command action; block until done.
        Returns True if the command completed successfully."""
        if not self._execute_command_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Execute command action server not available!')
            return False

        goal = ExecuteCommand.Goal()
        goal.command_name = command_name
        goal.cup_id = cup_id

        self._execute_command_done.clear()
        self._execute_command_success = False

        self.get_logger().info(f'Executing command: "{command_name}" (cup_id={cup_id or "none"})')
        future = self._execute_command_client.send_goal_async(goal)
        future.add_done_callback(self._execute_command_goal_response)

        if not self._execute_command_done.wait(timeout=EXECUTE_COMMAND_TIMEOUT_SEC):
            self.get_logger().error(f'Execute command "{command_name}" timed out!')
            return False

        return self._execute_command_success

    def _execute_command_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Execute command goal rejected.')
            self._execute_command_done.set()
            return
        goal_handle.get_result_async().add_done_callback(self._execute_command_result)

    def _execute_command_result(self, future) -> None:
        result = future.result().result
        self._execute_command_success = result.success
        if not result.success:
            self.get_logger().error(f'Execute command failed: {result.message}')
        self._execute_command_done.set()

    # ═══════════════════════════════════════════════════════════════════════
    # Main loop
    # ═══════════════════════════════════════════════════════════════════════

    def _main_loop(self):
        """For each unprocessed tag: align carriage, then solve IK."""
        # BASE_LINK FRAME IS Z TO THE BOTTOM, Y POSITIVE INTO THE WALL, X CARRIAGE MOVE POSITIVE zur spühle
        # CAMERA FRAME IS INVERTED TO THE GRIPPER FRAME
        # CAMERA FRAME: Z OUT, X TO THE RIGHT, Y DOWN
        # GRIPPER FRAME: Z OUT, X TO THE LEFT, Y UP
        tags = self.get_aruco_tags()
        if not tags:
            return
    
        # TODO: WHAT I NEED TO CHANGE ON WEDNESDAY
        # FIRST: INVERSE KINEMATICS WORKS ONLY IF WE ACTUALLY CAN REACH THE TAG WITH THE ARM
        # BEFORE WE CAN ONLY MOVE THE CARRIAGE TO THE GLOBAL Y POSITION AND THEN WE NEED TO RESCAN, because robot has other joints as it might had when noticed the cup first
        # THEREFORE WE NEED TO MOVE TO THE GLOBAL Y POSITION, GO INTO A SCANNING POSITION AND THEN IF WE SEE THE TAG WITHIN REACH, WE CAN GRAP IT

        for mid, entry in tags.items():
            self.get_logger().info(f'Tag {mid}: {entry}')
            if mid in self._processed_tags:
                continue
            if entry['pose'] is None:
                continue

            p = entry['pose'].position
            self.get_logger().info(
                f'[Main] Tag {mid}: local=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}), '
                f'global_x={entry["global_x"]:.3f}, dist={entry["distance"]:.3f}m')
            
            if not self.move_to_aruco(mid):
                self.get_logger().error(f'[Main] Failed on tag {mid}')
                continue

            self._processed_tags.add(mid)


def main(args=None):
    rclpy.init(args=args)
    node = PlanningNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    # while not node.execute_command('home'):
    #     node.get_logger().error('Failed to execute home command')
    #     time.sleep(5)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

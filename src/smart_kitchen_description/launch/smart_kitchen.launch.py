# Copyright (c) 2021 PickNik, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Marq Rasmussen

import os
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    # Initialize Arguments
    sim_gazebo = LaunchConfiguration("sim_gazebo")
    robot_type = LaunchConfiguration("robot_type")
    dof = LaunchConfiguration("dof")
    vision = LaunchConfiguration("vision")
    # General arguments
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    robot_name = LaunchConfiguration("robot_name")
    prefix = LaunchConfiguration("prefix")
    robot_traj_controller = LaunchConfiguration("robot_controller")
    robot_pos_controller = LaunchConfiguration("robot_pos_controller")
    robot_hand_controller = LaunchConfiguration("robot_hand_controller")
    robot_lite_hand_controller = LaunchConfiguration("robot_lite_hand_controller")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    gripper = LaunchConfiguration("gripper")

    if sim_gazebo.perform(context).lower() == "true":
        robot_controllers = PathJoinSubstitution(
            [
                FindPackageShare("smart_kitchen_description"),
                "config",
                "sim_ros2_controllers.yaml",
            ]
        )
    else:
        robot_controllers = PathJoinSubstitution(
            [
                FindPackageShare(description_package),
                "arms/" + robot_type.perform(context) + "/" + dof.perform(context) + "dof/config",
                controllers_file,
            ]
        )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "view_robot.rviz"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("smart_kitchen_description"), "urdf", "kitchen_robot.xacro"]
            ),
            " ",
            "robot_ip:=xxx.yyy.zzz.www",
            " ",
            "name:=",
            robot_name,
            " ",
            "arm:=",
            robot_type,
            " ",
            "dof:=",
            dof,
            " ",
            "vision:=",
            vision,
            " ",
            "prefix:=",
            prefix,
            " ",
            "sim_gazebo:=",
            sim_gazebo,
            " ",
            "simulation_controllers:=",
            robot_controllers,
            " ",
            "gripper:=",
            gripper,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content.perform(context)}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    robot_traj_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[robot_traj_controller, "-c", "/controller_manager",
                   "--controller-manager-timeout", "30"],
    )

    robot_pos_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[robot_pos_controller, "--inactive", "-c", "/controller_manager"],
    )

    robot_model = robot_type.perform(context)
    is_gen3_lite = "false"
    if robot_model == "gen3_lite":
        is_gen3_lite = "true"

    rail_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rail_trajectory_controller", "-c", "/controller_manager",
                   "--controller-manager-timeout", "30"],
    )

    robot_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[robot_hand_controller, "-c", "/controller_manager",
                   "--controller-manager-timeout", "30"],
        condition=UnlessCondition(is_gen3_lite),
    )

    robot_hand_lite_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[robot_lite_hand_controller, "-c", "/controller_manager",
                   "--controller-manager-timeout", "30"],
        condition=IfCondition(is_gen3_lite),
    )

    # Bridge
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    robotiq_description_prefix = get_package_prefix("robotiq_description")
    gz_robotiq_env_var_resource_path = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", os.path.join(robotiq_description_prefix, "share")
    )

    smart_kitchen_share = get_package_share_directory("smart_kitchen_description")
    gz_smart_kitchen_models_env = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", os.path.join(smart_kitchen_share, "models")
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_content,
            "-name",
            robot_name,
            "-x",
            "0.12",
            "-y",
            "2.0",
            "-z",
            "2.295",
            "-R",
            "3.14159",
            "-P",
            "0.0",
            "-Y",
            "-1.5708",
        ],
        condition=IfCondition(sim_gazebo),
    )

    world_file = os.path.join(smart_kitchen_share, "worlds", "smart_kitchen.world")

    gz_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={
            "gz_args": f" -r -v 1 {world_file} --physics-engine gz-physics-bullet-featherstone-plugin --render-engine ogre2 --render-engine-gui-api-backend opengl"
        }.items(),
        condition=IfCondition(sim_gazebo),
    )

    # Bridge
    gazebo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "/wrist_mounted_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/wrist_mounted_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/wrist_mounted_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/wrist_mounted_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/model/cup_blue/pose@geometry_msgs/msg/Pose[gz.msgs.Pose",
            "/model/cup_red/pose@geometry_msgs/msg/Pose[gz.msgs.Pose",
            "/model/cup_blue_filled/pose@geometry_msgs/msg/Pose[gz.msgs.Pose",
            "/model/cup_red_filled/pose@geometry_msgs/msg/Pose[gz.msgs.Pose",
        ],
        output="screen",
    )

    kill_gz = ExecuteProcess(
        cmd=["bash", "-c",
             "pkill -TERM -f 'ruby.*gz' 2>/dev/null; "
             "pkill -TERM -f 'gz sim' 2>/dev/null; "
             "sleep 1; "
             "pkill -9 -f 'ruby.*gz' 2>/dev/null; "
             "pkill -9 -f 'gz sim' 2>/dev/null; "
             "sleep 1; true"],
        output="screen",
        condition=IfCondition(sim_gazebo),
    )

    nodes_to_start = [
        bridge,
        robot_state_publisher_node,
        gz_robotiq_env_var_resource_path,
        gz_smart_kitchen_models_env,
        kill_gz,
        RegisterEventHandler(
            OnProcessExit(
                target_action=kill_gz,
                on_exit=[
                    gz_launch_description,
                    # Wait for Gazebo to finish starting before spawning the robot.
                    TimerAction(period=8.0, actions=[gz_spawn_entity]),
                ],
            )
        ),
        # Start controllers after the robot is spawned
        RegisterEventHandler(
            OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[
                    TimerAction(
                        period=10.0,
                        actions=(
                            [
                                joint_state_broadcaster_spawner,
                                robot_traj_controller_spawner,
                                rail_controller_spawner,
                                robot_hand_controller_spawner,
                                robot_hand_lite_controller_spawner,
                                delay_rviz_after_joint_state_broadcaster_spawner,
                                Node(
                                    package="smart_kitchen_logic",
                                    executable="fake_rail_node",
                                    output="screen",
                                    emulate_tty=True,
                                    respawn=True,
                                    respawn_delay=2.0,
                                ),
                            ]
                            if sim_gazebo.perform(context).lower() == "true"
                            else [
                                joint_state_broadcaster_spawner,
                                robot_traj_controller_spawner,
                                rail_controller_spawner,
                                robot_pos_controller_spawner,
                                robot_hand_controller_spawner,
                                robot_hand_lite_controller_spawner,
                                delay_rviz_after_joint_state_broadcaster_spawner,
                                Node(
                                    package="smart_kitchen_logic",
                                    executable="fake_rail_node",
                                    output="screen",
                                    emulate_tty=True,
                                    respawn=True,
                                    respawn_delay=2.0,
                                ),
                            ]
                        ),
                    )
                ],
            )
        ),
        gazebo_bridge,
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=[ExecuteProcess(
                    cmd=["bash", "-c",
                         "pkill -9 -f 'ruby.*gz' 2>/dev/null; "
                         "pkill -9 -f 'gz sim' 2>/dev/null; true"],
                    output="screen",
                )]
            )
        ),
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    # Simulation specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_gazebo",
            default_value="true",
            description="Use Gazebo for simulation",
        )
    )
    # Robot specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_type",
            description="Type/series of robot.",
            choices=["gen3", "gen3_lite"],
            default_value="gen3",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "dof",
            description="DoF of robot.",
            choices=["6", "7"],
            default_value="6",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "vision",
            description="Use arm mounted realsense",
            choices=["true", "false"],
            default_value="false",
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="ros2_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="kortex_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="kinova.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_name",
            default_value="gen3",
            description="Robot name.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_controller",
            default_value="joint_trajectory_controller",
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_pos_controller",
            default_value="twist_controller",
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_hand_controller",
            default_value="robotiq_gripper_controller",
            description="Robot hand controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_lite_hand_controller",
            default_value="gen3_lite_2f_gripper_controller",
            description="Robot hand controller to start for Gen3_Lite.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulated clock",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper",
            default_value="robotiq_2f_85",
            choices=["robotiq_2f_85", "robotiq_2f_140", "gen3_lite_2f", ""],
            description="Gripper to use",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

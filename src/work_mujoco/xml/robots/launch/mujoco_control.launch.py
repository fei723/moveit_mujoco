# Copyright (c) 2024 Franka Robotics GmbH
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch import LaunchContext, LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import AppendEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare
# from webots_ros2_driver.webots_launcher import WebotsLauncher
# from webots_ros2_driver.webots_controller import WebotsController
# from webots_ros2_driver.urdf_spawner import URDFSpawner, get_webots_driver_node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_path

PACKAGE_NAME = 'human_description'


def prepare_launch_description():
    # Configure ROS nodes for launch
    robot_id_name = 'robot_id'
    runtime_config_package_name = 'runtime_config_package'
    controllers_file_name = 'controllers_file'
    use_sim_time_name = 'use_sim_time'
    world_name = 'world'

    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "log_level",
            default_value=["info"],
            description="Logging level",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            robot_id_name,
            default_value='GR1T2',
            description='Available values: GR1T2, M92UW and M92UW'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            use_sim_time_name,
            default_value='true',
            description='If true, use simulated clock'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            runtime_config_package_name,
            default_value=PACKAGE_NAME,
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            controllers_file_name,
            default_value='ros2_controllers.yaml',
            description='YAML file with the controllers configuration.'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "ros2_control_config_file",
            default_value="mujoco_ros2_control_params.yaml",
            description="YAML file with the mujoco ros2 control params configuration.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "command_interface",
            default_value=['position', 'velocity',
                           'effort', 'stiffness', 'damping'],
            description="The output control command interface provided by ros2_control \
            ['position', 'velocity', 'effort', 'stiffness', 'damping'].",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "load_groups",
            default_value=['waist', 'head',
                           'left_arm', 'right_arm', 'left_hand', 'right_hand', 'sensors'],
            description="The output load_groups provided by ros2_control \
            ['waist', 'head', 'left_arm', 'right_arm', 'left_hand', 'right_hand', 'sensors'].",
        )
    )

    runtime_config_package = LaunchConfiguration(runtime_config_package_name)
    controllers_file = LaunchConfiguration(controllers_file_name)
    use_sim_time = LaunchConfiguration(use_sim_time_name)
    robot_id = LaunchConfiguration(robot_id_name)
    ros2_control_config_file = LaunchConfiguration("ros2_control_config_file")
    log_level = LaunchConfiguration("log_level")
    command_interface = LaunchConfiguration("command_interface")
    load_groups = LaunchConfiguration("load_groups")

    # Get URDF via xacro
    urdf_path = os.path.join(
        get_package_share_directory(PACKAGE_NAME),
        "urdf",
        "M92UW",
        "M92UW.urdf"
    )
    with open(urdf_path, "rt") as f:
        robot_description_content = f.read()
    robot_description = {
        "robot_description": ParameterValue(robot_description_content)
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time}
        ]
    )

    robot_controllers = '/home/mi/colcon_ws/src/worker2025/work_mujoco/xml/robots/config/ros2_controllers.yaml'

    mujoco_model_file = PathJoinSubstitution(
        [
            FindPackageShare(runtime_config_package),
            "mjcf",
            "M92UW",
            'M92UW_mujoco_scene.xml',
        ]
    )

    ros2_control_config = '/home/mi/colcon_ws/src/worker2025/work_mujoco/xml/robots/config/mujoco_ros2_control_params.yaml'

    # Visualize in RViz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "rviz", "mujoco.rviz"]
    )
    rviz = Node(package='rviz2',
                executable='rviz2',
                name='rviz2',
                parameters=[{'use_sim_time': use_sim_time}],
                arguments=['--display-config', rviz_config_file],
                )

    mujoco_node = Node(
        package="mujoco_simulate",
        executable="mujoco_simulate_node",
        # namespace="",
        parameters=[robot_description, robot_controllers, ros2_control_config],
        output="both",
        # prefix=['xterm -e gdb -ex run -args'],
        # prefix=['gnome-terminal --wait -- gdb -ex run --args'],
        arguments=[
            mujoco_model_file,
            "--ros-args",
            "--log-level",
            log_level,
        ],
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_body_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'body_controller'],
        output='screen'
    )

    load_head_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'head_controller'],
        output='screen'
    )

    load_right_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'right_arm_controller'],
        output='screen'
    )

    load_left_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'left_arm_controller'],
        output='screen'
    )

    load_right_hand_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'right_hand_controller'],
        output='screen'
    )

    load_left_hand_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'left_hand_controller'],
        output='screen'
    )

    nodes = [
        robot_state_publisher,
        # rviz,
        mujoco_node,
        load_joint_state_broadcaster,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[
                    load_body_controller,
                    load_head_controller,
                    load_left_arm_controller,
                    load_right_arm_controller,
                    load_right_hand_controller,
                    load_left_hand_controller,
                ],
            )
        ),
    ]
    return LaunchDescription(declared_arguments + nodes)


def generate_launch_description():
    launch_description = prepare_launch_description()

    # launch_description.add_action(
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(
    #             os.path.join(get_package_share_directory("m92uw_moveit_config"), "launch/my_demo.launch.py")
    #         )
    #     )
    # )

    return launch_description

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

PACKAGE_NAME = 'human_description'

def generate_launch_description():

    robot_description = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(PACKAGE_NAME), 'launch', 'visualize_robot.launch.py'    
                    )]), 
                    launch_arguments={'use_sim_time': 'true'}.items()
            )
    #  Gazebo nodes, provided by gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
                )]),
                launch_arguments={'paused': 'true'}.items() # Start paused
            )
    # Spawn robot, provided by gazebo_ros package
    spawn_robot = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'bbot'],
        output='screen'
    )

    spawn_joint_state_controllers = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"]
    )

    spawn_diff_drive_controllers = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', "-c", "/controller_manager"]
    )

    return LaunchDescription([
        robot_description,
        gazebo,
        spawn_robot,
        spawn_joint_state_controllers,
        spawn_diff_drive_controllers
    ])

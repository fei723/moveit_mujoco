from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch, generate_move_group_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("mi_robot_arm", package_name="mirobot").to_moveit_configs()

    # 声明启动参数，用于控制是否启动demo相关部分（例如RViz等）
    declare_use_demo_arg = DeclareLaunchArgument(
        "use_demo",
        default_value="true",
        description="Whether to launch the demo components (like RViz)"
    )

    # 生成move_group的启动描述
    move_group_launch = generate_move_group_launch(moveit_config)

    # 生成demo的启动描述
    demo_launch = generate_demo_launch(moveit_config)

    ld = LaunchDescription()

    ld.add_action(declare_use_demo_arg)

    # 添加move_group启动
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(move_group_launch),
    ))

    # 根据参数决定是否启动demo
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(demo_launch),
        condition=IfCondition(LaunchConfiguration("use_demo"))
    ))

    return ld
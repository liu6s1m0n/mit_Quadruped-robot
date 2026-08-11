"""
通过 ros2_control 启动 GO1 MuJoCo 模型.

此入口用于标准 ros2_control 链路；仅查看原生模型时使用 display.launch.py。
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    """通过 mujoco_ros2_control 加载未改动的 Menagerie GO1 MJCF."""
    package_share = FindPackageShare("mymit_robot")
    xacro_file = PathJoinSubstitution([package_share, "urdf", "go1.urdf.xacro"])
    controller_config = PathJoinSubstitution([
        package_share, "config", "ros2_controllers.yaml"
    ])

    description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        xacro_file, " headless:=", LaunchConfiguration("headless"),
    ])
    robot_description = {
        "robot_description": ParameterValue(
            description_content.perform(context), value_type=str
        )
    }

    return [
        # 发布 URDF 中的 TF 树；实际物理与渲染仍由 MJCF 场景负责。
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description, {"use_sim_time": True}],
            output="both",
        ),
        Node(
            # ros2_control 管理器加载 MuJoCo 硬件插件并推进仿真。
            package="mujoco_ros2_control",
            executable="ros2_control_node",
            parameters=[
                {"use_sim_time": True},
                ParameterFile(controller_config),
            ],
            remappings=(
                [("~/robot_description", "/robot_description")]
                if os.environ.get("ROS_DISTRO") == "humble" else []
            ),
            emulate_tty=True,
            output="both",
            on_exit=Shutdown(),
        ),
        Node(
            # 广播关节反馈，供 RViz 和其他 ROS 2 节点订阅。
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster", "--param-file", controller_config,
            ],
            output="both",
        ),
        Node(
            # 位置控制器默认保持未激活，避免与工程自身 WBC 同时控制关节。
            package="controller_manager",
            executable="spawner",
            arguments=[
                "forward_position_controller",
                "--param-file", controller_config,
                "--inactive",
            ],
            output="both",
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Run MuJoCo without its native Simulate window.",
        ),
        OpaqueFunction(function=launch_setup),
    ])

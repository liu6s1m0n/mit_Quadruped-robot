"""Launch the ROS-native GO1 controller and its robot description."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("mymit_robot")
    parameter_file = PathJoinSubstitution([package_share, "config", "mymit_robot.yaml"])
    xacro_file = PathJoinSubstitution([package_share, "urdf", "go1.urdf.xacro"])
    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", xacro_file]), value_type=str
    )
    namespace = LaunchConfiguration("namespace")

    return LaunchDescription([
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Optional namespace for all robot topics and services.",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=namespace,
            parameters=[
                {"robot_description": robot_description, "use_sim_time": True}
            ],
            output="screen",
        ),
        Node(
            package="mymit_robot",
            executable="mymit_robot_node",
            name="controller",
            namespace=namespace,
            parameters=[parameter_file],
            output="screen",
            emulate_tty=True,
            on_exit=Shutdown(),
        ),
    ])

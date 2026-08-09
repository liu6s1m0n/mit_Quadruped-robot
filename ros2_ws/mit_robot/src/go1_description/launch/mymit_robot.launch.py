"""Launch the native MuJoCo controller and its ROS 2 command gateway."""

from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="mymit_robot",
            executable="mymit_robot_height_service",
            name="mymit_robot_standing_height_service",
            output="screen",
        ),
        Node(
            package="mymit_robot",
            executable="mymit_robot_user",
            name="mymit_robot_simulation",
            output="screen",
            on_exit=Shutdown(),
        ),
    ])

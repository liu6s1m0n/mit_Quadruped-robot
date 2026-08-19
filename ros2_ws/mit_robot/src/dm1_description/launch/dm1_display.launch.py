"""只在原生 MuJoCo Simulate 中显示 DM1，不启动控制器。"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    scene_file = PathJoinSubstitution([
        FindPackageShare("mymit_robot"), "dm1", "scene.xml"
    ])
    return LaunchDescription([
        ExecuteProcess(
            cmd=[FindExecutable(name="simulate"), scene_file],
            output="screen",
        ),
    ])

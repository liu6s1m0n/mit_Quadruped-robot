from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Open the project-owned Menagerie GO1 scene in native MuJoCo Simulate."""
    scene_file = PathJoinSubstitution([
        FindPackageShare("mymit_robot"), "unitree_go1", "scene.xml"
    ])

    return LaunchDescription([
        ExecuteProcess(
            cmd=[FindExecutable(name="simulate"), scene_file],
            output="screen",
        ),
    ])

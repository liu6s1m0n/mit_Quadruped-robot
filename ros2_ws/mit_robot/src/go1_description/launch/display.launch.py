from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# 仅打开原生 MuJoCo Simulate 窗口，用于查看模型；此启动文件不会运行机器人控制器。


def generate_launch_description():
    """在原生 MuJoCo Simulate 中打开工程自带的 GO1 场景."""
    scene_file = PathJoinSubstitution([
        FindPackageShare("mymit_robot"), "unitree_go1", "scene.xml"
    ])

    return LaunchDescription([
        ExecuteProcess(
            cmd=[FindExecutable(name="simulate"), scene_file],
            output="screen",
        ),
    ])

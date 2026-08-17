"""同时启动原生 MuJoCo 控制程序和用于接收 ROS 2 高度命令的服务节点."""

from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node

# 启动高度服务和工程自带仿真控制程序；控制程序退出时关闭服务节点。


def generate_launch_description():
    # 高度服务通过本地 IPC 把 ROS 2 请求转交给仿真进程；仿真退出时关闭整组节点。
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

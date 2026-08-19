"""同时启动原生 MuJoCo 控制程序和用于接收 ROS 2 高度命令的服务节点."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

# 启动高度服务和工程自带仿真控制程序；控制程序退出时关闭服务节点。


def generate_launch_description():
    robot = LaunchConfiguration("robot")
    render_gpu = LaunchConfiguration("render_gpu")
    use_nvidia = IfCondition(
        PythonExpression(["'", render_gpu, "' == 'nvidia'"])
    )
    # 高度服务通过本地 IPC 把 ROS 2 请求转交给仿真进程；仿真退出时关闭整组节点。
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot",
            default_value="go1",
            description="Robot model: go1 or dm1",
        ),
        DeclareLaunchArgument(
            "render_gpu",
            default_value="nvidia",
            description="OpenGL renderer selection: nvidia or auto",
        ),
        # PRIME 变量必须在可执行文件启动前进入进程环境，才能让 GLVND 选择独显。
        SetEnvironmentVariable(
            "__NV_PRIME_RENDER_OFFLOAD", "1", condition=use_nvidia
        ),
        SetEnvironmentVariable(
            "__GLX_VENDOR_LIBRARY_NAME", "nvidia", condition=use_nvidia
        ),
        SetEnvironmentVariable(
            "__VK_LAYER_NV_optimus", "NVIDIA_only", condition=use_nvidia
        ),
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
            arguments=["--robot", robot, "--render-gpu", render_gpu],
            output="screen",
            on_exit=Shutdown(),
        ),
    ])

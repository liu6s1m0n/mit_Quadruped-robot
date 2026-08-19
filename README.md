# mit_Quadruped-robot

mit_Quadruped-robot 是一个面向四足机器人运动控制与仿真研究的 ROS 2 工程。项目以 C++ 为主要开发语言，结合 MuJoCo 构建机器人仿真环境，并集成状态估计、步态规划、有限状态机、模型预测控制（MPC）和全身控制（WBC）等模块。

仓库目前支持 Unitree GO1 与 DM1 两套机器人模型，可用于控制算法开发、仿真验证、模块测试以及四足机器人运动控制流程的学习与研究。

## 主要功能

- 基于 MuJoCo 的四足机器人动力学仿真与可视化
- GO1、DM1 多机器人模型支持
- 腿部控制、足端轨迹规划与步态调度
- 姿态、位置、速度及接触状态估计
- 基于有限状态机的机器人行为管理与安全检查
- MPC 与 WBC 运动控制框架
- ROS 2 自定义消息、服务和启动文件
- 覆盖模型、传感器、估计器、控制器及系统集成的自动化测试

## 控制框架

工程中的主要控制流程如下：

```text
MuJoCo 仿真 / 传感器数据
          ↓
      状态估计
          ↓
      控制状态机
          ↓
      MPC / WBC
          ↓
      腿部控制器
          ↓
     四足机器人模型
```

## 工程结构

```text
MY_ROBOT/
├── ros2_ws/
│   └── mit_robot/
│       ├── include/       # 控制、模型、估计与工具类头文件
│       ├── src/           # 核心算法及机器人模型资源
│       ├── user/          # 仿真程序、运行器与 ROS 2 服务
│       ├── msg/           # ROS 2 自定义消息
│       ├── srv/           # ROS 2 自定义服务
│       └── test/          # 单元测试与集成测试
└── README.md
```

## 环境依赖

建议在 Linux 环境下使用，主要依赖包括：

- ROS 2（当前工程面向 Humble 环境）
- C++17 编译器
- CMake 与 colcon
- MuJoCo
- Eigen3
- OpenGL 与 GLFW

此外，机器人描述与可视化功能会使用 Xacro、RViz2、robot_state_publisher 及 ros2_control 等 ROS 2 组件。

## 构建工程

确认 ROS 2 和相关依赖已安装后，在工作空间中执行：

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select mymit_robot
source install/setup.bash
```

## 运行仿真

启动默认的 GO1 仿真：

```bash
ros2 launch mymit_robot mymit_robot.launch.py
```

选择 DM1 模型：

```bash
ros2 launch mymit_robot mymit_robot.launch.py robot:=dm1
```

在不使用 NVIDIA PRIME 独立显卡渲染的环境中，可以选择自动渲染模式：

```bash
ros2 launch mymit_robot mymit_robot.launch.py render_gpu:=auto
```

## 测试

工程提供了按功能模块划分的测试用例。完成构建后可运行：

```bash
cd ros2_ws
colcon test --packages-select mymit_robot
colcon test-result --verbose
```

## 项目说明

本项目主要用于四足机器人控制算法的开发、验证与学习。当前仍在持续完善中，后续将继续扩展控制策略、机器人模型、仿真场景以及软硬件适配能力。

## License

本项目包含采用 Apache-2.0 与 BSD-3-Clause 许可证发布的代码及资源，具体授权信息请参阅工程中的 [LICENSE](ros2_ws/mit_robot/LICENSE) 文件及相关模型目录中的许可说明。

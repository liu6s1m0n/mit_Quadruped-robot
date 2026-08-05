# My_robot 四足机器人开发要求

项目位置：`/home/simon/My_robot`

目标平台：Unitree GO1

开发环境：ROS 2 Humble、C++、Eigen、MuJoCo、ros2_control

算法参考：MIT Cheetah-Software

## 1. 当前开发进度

当前项目已经完成“ROS 2 骨架、GO1 MuJoCo 模型和 ros2_control 基础闭环”，下一步进入运动学、传感器广播和基础腿部控制阶段。

已经完成：

- 创建 ROS 2 工作空间 `ros2_ws`。
- 创建 `mymit_robot` 功能包。
- 编写 `FootContact.msg`、`StateEstimate.msg` 和 `SetControlMode.srv`。
- ROSIDL 接口曾经成功编译和安装。
- 已将消息和服务恢复到功能包根目录，路径与 `CMakeLists.txt` 一致。
- 已修正根目录 `.gitignore`，`MyMIT_Robot` 新源码不再被忽略。
- 已强制重新配置并成功构建 `mymit_robot`，三个接口均可由 `ros2 interface show` 查询。
- 添加 MIT 风格的 `cTypes.h` 和 `cppTypes.h`。
- 已将两个公共头文件移动到 `include/mymit_robot/`，并完成 CMake 安装、导出和 Eigen 依赖配置。
- `cTypes.h` 的 C11 语法检查、`cppTypes.h` 的 C++17 语法检查以及重新构建均已通过。
- 已将 `/home/simon/mujoco_menagerie/unitree_go1` 的官方 Menagerie MJCF、网格和 BSD-3-Clause 许可证复制到功能包中。
- MuJoCo 物理仿真以 `mujoco/go1/scene.xml` 和 `go1.xml` 为唯一模型来源；Xacro 仅保留 ROS TF、关节限制和 ros2_control 接口所需的轻量关节树。
- 已使用官方 `mujoco_ros2_control/MujocoSystemInterface` 连接 Menagerie MJCF，不再开发项目私有的 `mymit_robot/MujocoSystem`。
- 已完成 12 个位置命令接口、48 个关节状态接口和 10 个 IMU 状态接口的注册。
- 已验证 `/joint_states`、MuJoCo 心跳、`joint_state_broadcaster` 和位置控制器；站立姿态指令能够从 ROS 2 写入 MuJoCo 执行器。
- GO1 Xacro、MJCF、控制器 YAML、Python launch、资源安装规则和工作区构建均已通过检查。
- `ros2_control.launch.py` 默认打开 MuJoCo Simulate，传入 `headless:=true` 可用于无界面测试。

当前需要先处理的问题：

1. IMU 已进入 ros2_control 状态接口，但还要配置 IMU broadcaster 才会产生标准 `/imu/data` 话题。
2. Menagerie 足端接触还没有映射成 `FootContact.msg`，需要加入足底力/接触读取组件。
3. 官方 Humble 二进制包在用户中断退出时会在自身析构流程中报告段错误；正常运行和控制不受影响，但需跟踪上游修复或改用修复后的源码版本。
4. 当前没有状态估计节点、运动控制节点、腿部控制、FSM、步态、WBC 或 MPC 实现。

因此接下来从“补齐传感器输出并实现 GO1 数学模型”开始，不重复实现已经可用的 MuJoCo 硬件桥接层。

## 2. 时间顺序总表

以下工期按单人开发估算。上一阶段通过验收后，才能进入下一阶段。

| 时间 | 阶段 | 阶段输出 |
|---|---|---|
| 第 1～2 天 | 整理现有 ROS 2 工程 | 功能包可从干净环境稳定编译。 |
| 第 3～5 天 | GO1 模型与坐标系 | GO1 能在 RViz 和 MuJoCo 正确显示。 |
| 第 2 周 | 公共类型、运动学和动力学 | 足端位置、雅可比和模型测试通过。 |
| 第 3 周 | MuJoCo 与 ros2_control | 能读取传感器并发送安全的关节命令。 |
| 第 4 周 | LegController 与关节 PD | 四腿悬空轨迹稳定。 |
| 第 5 周 | 状态估计 | 姿态、速度和接触估计稳定。 |
| 第 6 周 | FSM、站起和站立 | 机器人能安全站起并保持站立。 |
| 第 7 周 | 步态和摆动腿轨迹 | 能原地踏步并执行低速 Trot。 |
| 第 8 周 | 平衡控制与 WBC | 能抵抗小扰动并保持机身目标。 |
| 第 9～10 周 | Convex MPC | 能跟踪前进、横移和转向速度。 |
| 第 11 周 | 仿真集成和安全测试 | 仿真长时间运行无异常。 |
| 第 12 周以后 | GO1 真机迁移 | 从只读、阻尼逐步过渡到站立和行走。 |

工期只是建议；验收条件比日期更重要。

## 3. 阶段 1：整理现有 ROS 2 工程（第 1～2 天）

### 开发要求

- 修正 `.gitignore`，确保 `ros2_ws/MyMIT_Robot` 中的源码被 Git 跟踪，只忽略 `build/`、`install/` 和 `log/`。
- 将接口恢复为 ROS 2 标准位置：
  - `MyMIT_Robot/msg/FootContact.msg`
  - `MyMIT_Robot/msg/StateEstimate.msg`
  - `MyMIT_Robot/srv/SetControlMode.srv`
- 将公共头文件移动到：
  - `include/mymit_robot/cTypes.h`
  - `include/mymit_robot/cppTypes.h`
- 修正 `CMakeLists.txt` 中的接口、头文件和安装规则。
- 补全 `package.xml` 描述和依赖。

### 本阶段涉及文件

```text
ros2_ws/MyMIT_Robot/
├── CMakeLists.txt
├── package.xml
├── msg/
├── srv/
├── include/mymit_robot/
└── src/
```

### 验收要求

```bash
cd /home/simon/My_robot/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 interface show mymit_robot/msg/StateEstimate
ros2 interface show mymit_robot/msg/FootContact
ros2 interface show mymit_robot/srv/SetControlMode
```

以上命令必须全部成功，并且从空的 `build/install/log` 重新编译也能通过。

## 4. 阶段 2：GO1 模型与坐标系（第 3～5 天）

### 开发要求

- 直接采用 MuJoCo Menagerie 的 GO1 MJCF、网格和物理参数作为仿真模型。
- 保留最小 ROS Xacro 关节树，用于 TF、关节限制和 ros2_control 硬件描述，不用它生成 MuJoCo 动力学模型。
- 固定四腿顺序为 `FR、FL、RR、RL`，与当前消息保持一致。
- 固定每条腿关节顺序为 `abad、hip、knee`。
- 定义 `base_link`、`imu_link` 和四个足端坐标系。
- `display.launch.py` 使用 launch 参数传入模型路径，不在代码中写死用户目录。
- 验证 ROS 关节树与 MuJoCo 模型的关节名称、顺序和方向一致。

### 新增文件

```text
src/go1_description/
├── mujoco/go1/scene.xml
├── mujoco/go1/go1.xml
├── mujoco/go1/assets/*.stl
├── mujoco/go1/LICENSE
├── urdf/go1.urdf.xacro
├── urdf/go1_ros2_control.xacro
├── config/joint_limits.yaml
├── config/ros2_controllers.yaml
├── config/mujoco_ros2_control_plugins.yaml
├── rviz/go1.rviz
├── launch/display.launch.py
└── launch/ros2_control.launch.py
```

`display.launch.py` 的核心函数：

```python
def generate_launch_description():
    pass
```

### 验收要求

- RViz 中模型无断开的 TF。
- `scene.xml` 可直接由 MuJoCo 载入，模型具有自由基座、12 个关节、12 个执行器和 IMU。
- MuJoCo 中四条腿方向和 ROS 2 关节树一致。
- 所有关节名称、零位、单位和限制形成一张记录表。

## 5. 阶段 3：公共数据、运动学和动力学（第 2 周）

### MIT 参考

- `common/include/cTypes.h`
- `common/include/cppTypes.h`
- `common/include/Dynamics/Quadruped.h`
- `common/include/Dynamics/FloatingBaseModel.h`
- `common/src/Controllers/LegController.cpp`

只参考类的职责和算法，不复制 Cheetah 3 的质量、惯量、腿长和关节方向；这些参数必须换成 GO1 数据。

### 新增文件与核心函数

`include/mymit_robot/model/robot_types.hpp`：

```cpp
struct ImuData;
struct JointState;
struct JointCommand;
struct FootContactState;
struct StateEstimate;
struct DesiredState;
```

`include/mymit_robot/model/quadruped.hpp`：

```cpp
class Quadruped {
public:
  bool loadGo1Parameters();
  Vec3<double> getHipLocation(int leg) const;
  double getSideSign(int leg) const;
};
```

`include/mymit_robot/model/leg_kinematics.hpp`：

```cpp
Vec3<double> forwardKinematics(int leg, const Vec3<double>& q);
bool inverseKinematics(int leg, const Vec3<double>& foot_position,
                       Vec3<double>& q_result);
void computeLegJacobianAndPosition(int leg, const Vec3<double>& q,
                                   Mat3<double>* J, Vec3<double>* p);
```

`include/mymit_robot/model/floating_base_model.hpp`：

```cpp
class FloatingBaseModel {
public:
  void setState(const StateEstimate& base, const JointState& joints);
  void forwardKinematics();
  DMat<double> massMatrix() const;
  DVec<double> generalizedGravityForce() const;
  Mat3<double> contactJacobian(int leg) const;
};
```

### 验收要求

- 正运动学和逆运动学互相验证。
- 解析雅可比与数值差分雅可比误差满足设定阈值。
- 左右腿镜像关系正确。
- 数学模型代码不包含 `rclcpp`。

## 6. 阶段 4：MuJoCo 与 ros2_control 数据闭环（第 3 周）

### 开发要求

使用官方 `mujoco_ros2_control/MujocoSystemInterface`，让上层控制器不直接调用 MuJoCo API。项目侧只维护以下适配内容：

- 在 `go1_ros2_control.xacro` 中声明 12 个关节命令/状态接口和 IMU 状态接口。
- 在 `go1.xml` 中保证执行器名与 ROS 关节名一致，并提供 `base_imu_quat`、`base_imu_gyro`、`base_imu_accel`。
- 在 `ros2_controllers.yaml` 中配置广播器和基础控制器。
- 在 `ros2_control.launch.py` 中启动官方桥接节点、状态发布器和控制器 spawner。
- 上层算法只访问 ros2_control 接口，不包含 MuJoCo API。

### 本阶段运行组件

- `controller_manager`
- `joint_state_broadcaster`
- `robot_state_publisher`
- IMU broadcaster（下一步补充）
- 足端接触 broadcaster（下一步补充）

### 验收要求

- `/joint_states` 持续发布 12 个关节状态。
- `/imu/data` 能发布四元数、角速度和线加速度。
- `/foot_contacts` 按 `FR、FL、RR、RL` 发布。
- 控制命令超时后自动写入零力矩或阻尼命令。

## 7. 阶段 5：LegController 和关节 PD（第 4 周）

### MIT 参考

- `common/include/Controllers/LegController.h`
- `common/src/Controllers/LegController.cpp`
- `user/JPos_Controller/`

### 新增文件与核心函数

`include/mymit_robot/control/leg_controller.hpp`：

```cpp
class LegController {
public:
  void zeroCommand();
  void dampingCommand(double kd);
  void updateData(const JointState& joint_state);
  void setJointCommand(int leg, const Vec3<double>& q_des,
                       const Vec3<double>& qd_des);
  void setCartesianCommand(int leg, const Vec3<double>& p_des,
                           const Vec3<double>& v_des);
  JointCommand computeCommand() const;
};
```

控制公式：

```text
tau = tau_ff + Kp(q_des - q) + Kd(qd_des - qd)
F = F_ff + Kp_cart(p_des - p) + Kd_cart(v_des - v)
tau_foot = J(q)^T F
```

### 验收顺序

1. 单关节小幅度 PD。
2. 单腿三关节悬空控制。
3. 四腿悬空位置轨迹。
4. 足端笛卡尔轨迹。

只有低增益、小幅度测试稳定后才能进入站立测试。

## 8. 阶段 6：状态估计（第 5 周）

### MIT 参考

- `StateEstimatorContainer.h`
- `OrientationEstimator.cpp`
- `PositionVelocityEstimator.cpp`
- `ContactEstimator.cpp`

### 新增文件与核心函数

`state_estimator_container.hpp`：

```cpp
void addEstimator(std::unique_ptr<GenericEstimator> estimator);
void removeAllEstimators();
void run();
const StateEstimate& getResult() const;
```

`orientation_estimator.hpp`：

```cpp
void reset(const ImuData& imu);
void run(const ImuData& imu, StateEstimate& result);
```

`position_velocity_estimator.hpp`：

```cpp
void reset(const StateEstimate& initial_state);
void predict(const ImuData& imu, double dt);
void correct(const JointState& joints,
             const FootContactState& contacts,
             const Quadruped& model);
```

### `state_estimator_node`

订阅：`/imu/data`、`/joint_states`、`/foot_contacts`

发布：`/state_estimate`、`/odom`、`odom -> base_link` TF

核心回调：

```cpp
void imuCallback(const sensor_msgs::msg::Imu& msg);
void jointStateCallback(const sensor_msgs::msg::JointState& msg);
void contactCallback(const mymit_robot::msg::FootContact& msg);
void updateAndPublish();
```

### 验收要求

- 先使用 MuJoCo 真值验证控制算法。
- 再切换到 IMU 姿态估计。
- 最后加入支撑足约束和位置速度卡尔曼滤波。
- 静止时速度接近零，接触切换时估计值不明显跳变。

## 9. 阶段 7：FSM、站起和站立（第 6 周）

### MIT 参考

- `user/MIT_Controller/FSM_States/ControlFSM.*`
- `FSM_State_Passive`
- `FSM_State_JointPD`
- `FSM_State_StandUp`
- `FSM_State_BalanceStand`
- `SafetyChecker`

### 状态顺序

```text
PASSIVE → DAMPING → JOINT_PD → STAND_UP → BALANCE_STAND
                                      ↘ ESTOP
```

`control_fsm.hpp` 核心函数：

```cpp
void initialize();
void requestMode(ControlMode mode);
void runFSM(const ControlInput& input, ControlOutput& output);
bool checkTransition(ControlMode from, ControlMode to) const;
void enterEmergencyStop(const std::string& reason);
```

`safety_checker.hpp` 核心函数：

```cpp
bool checkOrientation(const StateEstimate& state) const;
bool checkJointLimits(const JointState& joints) const;
bool checkCommandTimeout(const rclcpp::Time& now) const;
bool checkFinite(const JointCommand& command) const;
```

### 验收要求

- 机器人从趴卧姿态平滑站起。
- 能保持固定高度、Roll 和 Pitch。
- 任意阶段都能安全退回 Damping 或 ESTOP。

## 10. 阶段 8：步态和摆动腿轨迹（第 7 周）

### MIT 参考

- `common/Controllers/GaitScheduler.*`
- `common/Controllers/FootSwingTrajectory.*`
- `user/MIT_Controller/Controllers/convexMPC/Gait.*`

`gait_scheduler.hpp` 核心函数：

```cpp
void configureTrot(double period, double duty_factor);
void reset(double time);
void update(double time, double dt);
double phase(int leg) const;
bool desiredContact(int leg) const;
```

`foot_swing_trajectory.hpp` 核心函数：

```cpp
void setInitialPosition(const Vec3<double>& p0);
void setFinalPosition(const Vec3<double>& pf);
void setHeight(double height);
void computeBezier(double phase, double swing_time);
Vec3<double> position() const;
Vec3<double> velocity() const;
```

### 验收顺序

1. 离线画出 Trot 四腿相位。
2. 仿真中原地抬腿。
3. 原地踏步。
4. 低速前进，不加入 MPC。

## 11. 阶段 9：平衡控制和 WBC（第 8 周）

### MIT 参考

- `Controllers/BalanceController/`
- `Controllers/WBC/`
- `Controllers/WBC_Ctrl/`

核心函数：

```cpp
DesiredWrench computeDesiredBodyWrench(
    const StateEstimate& state, const DesiredState& desired);

ContactForceResult solveContactForces(
    const DesiredWrench& wrench, const FootContactState& contacts);

JointCommand runWholeBodyControl(
    const StateEstimate& state,
    const DesiredState& body_task,
    const FootTrajectory& foot_tasks);
```

### 验收要求

- 接触力满足法向力限制和摩擦锥限制。
- 站立时能抵抗小扰动。
- 摆动足任务不会破坏机身平衡。

## 12. 阶段 10：Convex MPC（第 9～10 周）

### MIT 参考阅读顺序

```text
Gait
  → ConvexMPCLocomotion
  → RobotState
  → convexMPC_interface
  → SolverMPC
```

核心函数：

```cpp
void buildContactTable(const GaitScheduler& gait);
void buildReferenceTrajectory(const DesiredState& desired);
void buildDynamics(const StateEstimate& state);
void buildCostAndConstraints();
bool solve(MpcSolution& solution);
bool solutionValid(const rclcpp::Time& now) const;
```

### 运行要求

- MPC 在控制器内部以低频线程运行，建议先使用 20～30 Hz。
- LegController、状态估计和 WBC 保持高频运行。
- 高频线程只读取最近一次有效 MPC 解，不等待求解器。
- 解超时或失败时退回 BalanceStand 或 Damping。

### 验收要求

- 稳定跟踪低速前进、横移和偏航命令。
- 地面反力和关节力矩没有明显跳变。
- 求解失败不会导致机器人失控。

## 13. 阶段 11：仿真集成和安全测试（第 11 周）

### 最终节点大纲

| 节点或组件 | 作用 |
|---|---|
| `controller_manager` | 管理 MuJoCo/真机硬件接口和控制器。 |
| `joint_state_broadcaster` | 发布 12 个关节状态。 |
| `robot_state_publisher` | 发布 GO1 TF。 |
| `state_estimator_node` | 输出机身状态估计。 |
| `locomotion_controller` | 内部运行 FSM、腿控、步态、WBC 和 MPC。 |
| `teleop_node` | 发布速度和模式请求。 |

### 启动顺序

```text
GO1 description
  → MuJoCo + controller_manager
  → state broadcasters
  → state_estimator_node
  → locomotion_controller（初始 PASSIVE）
  → teleop_node
```

### 验收要求

- 仿真连续运行至少 30 分钟无 NaN、崩溃和控制超时。
- 每个控制状态能够重复进入和退出。
- 急停、命令超时和估计失效测试全部通过。
- 保存状态估计、关节命令、接触力、FSM 状态和 MPC 求解时间。

## 14. 阶段 12：GO1 真机迁移（第 12 周以后）

### 迁移顺序

1. 实现 `Go1System`，只读取关节、IMU、电池和通信状态。
2. 核对关节编号、零位、正方向和单位。
3. 加入力矩、速度、位置、温度和通信超时保护。
4. 机器人悬空测试 Damping。
5. 悬空测试单关节和单腿。
6. 安全支架上测试 StandUp 和 BalanceStand。
7. 最后测试低速 Trot 和 MPC。

`go1_system.hpp` 核心函数：

```cpp
CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
hardware_interface::return_type read(
    const rclcpp::Time& time, const rclcpp::Duration& period) override;
hardware_interface::return_type write(
    const rclcpp::Time& time, const rclcpp::Duration& period) override;
void emergencyStop();
```

仿真和真机必须使用同一个 `locomotion_controller`；只能替换硬件插件和参数。

## 15. 当前下一步

当前 GO1 描述和 ros2_control 基础闭环已经验证完成，不要提前编写 MPC。按以下顺序继续：

1. `.gitignore` 已修复。
2. `msg/`、`srv/` 路径和 CMake 已同步，接口构建及查询验证通过。
3. `cTypes.h`、`cppTypes.h` 已移入标准 include 目录，安装、导出、语法和构建验证通过。
4. Menagerie GO1 MJCF 和网格已纳入功能包，许可证已保留，MuJoCo 载入验证通过。
5. 官方 `mujoco_ros2_control` 已接入；12 个关节、IMU、状态广播和位置命令闭环验证通过。
6. 配置 IMU broadcaster，输出标准 `/imu/data`，核对静止时重力方向和四元数顺序。
7. 增加四足接触/足底力读取，按 `FR、FL、RR、RL` 发布 `FootContact.msg`。
8. 实现并测试 GO1 正运动学、逆运动学和解析雅可比。
9. 实现 LegController 和安全的关节 PD，先做悬空单腿测试，再做四腿测试。
10. 处理或规避官方 Humble 桥接包退出时的析构段错误，再进行长时间自动化测试。

开发过程中保持三条原则：模型与算法不依赖 ROS 2；控制算法不直接调用 MuJoCo 或 GO1 SDK；每个阶段验收通过后才进入下一阶段。

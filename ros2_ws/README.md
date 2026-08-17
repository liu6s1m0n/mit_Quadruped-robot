# GO1 控制参数表

本文只记录当前正式控制链中可人为调整的控制、估计、步态、求解和调度参数。质量、惯量、连杆尺寸、关节限位、额定转矩等机器人固有参数不在此表中。

参数优先级按 `初始化默认值 → 状态内覆盖值 → user/main.cpp 最终配置值` 排列；同一参数出现多次时，以最右侧、最接近正式入口的值为准。`未覆盖` 表示默认值就是当前生效值，`未启用` 表示代码提供该参数，但当前正式仿真链没有使用它。

## 1. 正式入口、频率和用户命令

| 文件 | 参数 | 初始化/默认值 | 当前最终值 | 说明 |
|---|---|---:|---:|---|
| `mit_robot/src/go1_description/unitree_go1/go1.xml` | MuJoCo `timestep` | XML 未显式设置，MuJoCo默认 `0.002 s` | `0.002 s`（500 Hz） | 物理、状态估计、FSM、WBC和最终关节PD统一周期；测试确认WBC不应降频。 |
| `mit_robot/include/FSM/ControlFSMData.h` | `control_time_step` | `0.001 s` | 被 `RobotRunner` 覆盖为 `model->opt.timestep = 0.002 s` | 只有不从正式入口构造 FSM 时才会使用 1 ms 默认值。 |
| `mit_robot/user/main.cpp` | `kStandingHeight` | — | `0.27 m` | 正式入口最终站立高度。 |
| `mit_robot/user/main.cpp` | `kSlowWalkingForwardSpeed` / `kFastWalkingForwardSpeed` | — | `0.18 / 0.36 m/s` | MuJoCo 的 `Forward slow` 用于崎岖地形稳定行走，`Forward fast` 比原 `0.30 m/s` 适度提速。 |
| `mit_robot/user/main.cpp` | `kWalkingLateralSpeed` | — | `0.25 m/s` | MuJoCo Left/Right 两个方向开关使用的速度绝对值。 |
| `mit_robot/user/main.cpp` | `kTurningYawRate` | — | `0.35 rad/s` | MuJoCo Rotate CCW 开关使用的逆时针自转角速度。 |
| `mit_robot/user/SimulationBridge.hpp` | `standing_height_` / UI 初始值 | `0.27 m` | 被 `main.cpp` 设置为 `0.27 m` | 仿真 UI 与控制器的初始目标。 |
| `mit_robot/user/SimulationBridge.hpp` | `slow_walking_forward_speed_` / `fast_walking_forward_speed_` | `0.18 / 0.36 m/s` | 被 `main.cpp` 设置为相同值 | 慢档必须低于快档，两档均沿用 MPC 的平滑速度斜坡。 |
| `mit_robot/user/SimulationBridge.hpp` | `walking_backward_speed_` | `0.30 m/s` | 未覆盖 | 后退速度保持拆档前的当前值。 |
| `mit_robot/user/SimulationBridge.hpp` | `walking_lateral_speed_` | `0.25 m/s` | 被 `main.cpp` 设置为 `0.25 m/s` | 左移为正、右移为负。 |
| `mit_robot/user/SimulationBridge.hpp` | `turning_yaw_rate_` | `0.35 rad/s` | 被 `main.cpp` 设置为 `0.35 rad/s` | 当前第五个开关执行逆时针自转。 |
| `mit_robot/user/SimulationBridge.hpp` / `.cpp` | 六个运动开关 | 全部关闭 | `Forward slow / Forward fast / Backward / Left / Right / Rotate CCW` 互斥 | 新打开的运动取得控制权；关闭当前运动后切回 BalanceStand。 |
| `mit_robot/user/SimulationBridge.cpp` | `Jump forward` 按钮 | 未触发 | 单次向前跳 | 仅在稳定的 BalanceStand 中接受；动作结束自动回站立，行走、倾斜或速度过大时拒绝。 |
| `mit_robot/src/FSM/FSM_State_FrontJump.cpp` | 向前跳阶段时间 | — | `0.18 / 0.18 / 0.17 / 0.25 s` | 顺序为预蹲、水平化蹬伸、空中收腿、带俯仰阻尼的落地缓冲，总时长约 `0.78 s`。 |
| `mit_robot/src/FSM/FSM_State_FrontJump.cpp` | 蹬伸分配 | — | 前腿 `[0,1.20,-1.25]`，后腿 `[0,1.10,-1.20] rad` | 足端以后扫为主并保留小幅前后腿差异，增加水平冲量同时抵消俯仰力矩。 |
| `mit_robot/user/StandingHeightIpc.hpp` | 高度最小值 / 最大值 / 默认值 | `0.18 / 0.34 / 0.27 m` | 未覆盖 | 同时约束 ROS 2 服务、IPC 和 MuJoCo 滑块。 |
| `mit_robot/user/SimulationBridge.cpp` | UI 高度范围 | `0.18～0.34 m` | 未覆盖 | 应与 `StandingHeightIpc.hpp` 保持一致。 |
| `mit_robot/user/SimulationBridge.cpp` | 物理线程轮询休眠 | `1 ms` | 未覆盖 | 只影响主机调度和 CPU 占用，不改变 MuJoCo 的 2 ms 物理步长。 |
| `mit_robot/user/SimulationDiagnostics.cpp` | 正式运行诊断输出周期 | `500`帧 | 约`1 s` | 主程序持续统计估计RMS、最大俯仰、最低高度、小腿碰地和控制拒绝帧；只读诊断，不修改控制。 |
| `mit_robot/user/RobotRunner.hpp` | `defaultWalkingForwardSpeed()` | `0.32 m/s` | UI 慢/快档命令覆盖为入口配置值 | 不经过方向接口时使用。 |
| `mit_robot/user/RobotRunner.hpp` | `joint_initialization_duration_` | `0.4 s` | 未覆盖 | 上电关节回 home 的 smoothstep 时间；启动阶段不用于行走提速。 |
| `mit_robot/user/RobotRunner.hpp` | `standing_height_target_` / `standing_height_command_` | `0.27 / 0.27 m` | 目标由入口或 UI 设置 | 目标值和经过限速后的实际命令值需区分。 |
| `mit_robot/user/RobotRunner.hpp` | `standing_height_rate_limit_` | `0.08 m/s` | 未覆盖 | 第一层高度变化限速；500 Hz 时每周期最多 `0.00016 m`。 |
| `mit_robot/user/RobotRunner.cpp` | 初始化关节 `Kp / Kd` | — | `60 / 3` | 仅 0.4 s 关节初始化阶段生效。 |
| `mit_robot/user/RobotRunner.cpp` | `setUseWbc` | 默认可配置 | `true` | 正式链启用 WBC。 |
| `mit_robot/user/RobotRunner.cpp` | 姿态估计模式 | `IMU_FUSION` | `SIMULATION_TRUTH` | 当前仿真直接使用 MuJoCo 姿态真值，融合增益不参与最终姿态输出。 |
| `mit_robot/src/go1_description/unitree_go1/go1.xml` | MuJoCo 位置执行器 `kp` | `100` | 反馈作用为 0 | `SimulationBridge.cpp` 每帧令 `ctrl=qpos`，因此该位置伺服器不产生位置误差；最终关节 PD 由控制器命令决定。 |

## 2. MPC 与行走命令

| 文件 | 参数 | 初始化/默认值 | 当前最终值 | 说明 |
|---|---|---:|---:|---|
| `mit_robot/include/MPC/ConvexMPCLocomotion.h` | `iterations_between_mpc` | `15` 帧 | `20` 帧 | `FSM_State_Locomotion.cpp` 用 `round(0.04 / 0.002)` 覆盖，MPC 基础刷新频率为 25 Hz；接触组合变化时立即补算。 |
| `mit_robot/include/MPC/ConvexMPCLocomotion.h` | `iterations_per_gait_segment` | 默认跟随求解间隔 | `25` 帧 | 正式链单独使用 `round(0.05 / 0.002)`，保持 0.5 s TROT 周期，不随求解频率改变。 |
| `mit_robot/src/FSM/FSM_State_Locomotion.cpp` | MPC 刷新周期 | — | `0.04 s`（25 Hz） | 与 0.05 s 步态分段周期解耦，接触切换仍保持严格同步。 |
| `mit_robot/include/MPC/ConvexMPCLocomotion.h` | `maximum_linear_acceleration_` | `1.0 m/s²` | 未覆盖 | 前后/左右二维速度向量的总斜坡加速度；0.40 m/s命令约0.4秒到达。 |
| `mit_robot/include/MPC/ConvexMPCLocomotion.h` | `maximum_yaw_acceleration_` | `0.8 rad/s²` | 未覆盖 | 自转角速度的斜坡加速度，降低后启动和停止更柔和。 |
| `mit_robot/src/MPC/ConvexMPCLocomotion.cpp` | 平面速度范围 | — | 二维模长 `≤1 m/s` | 同时约束前后和左右速度。 |
| `mit_robot/src/MPC/ConvexMPCLocomotion.cpp` | 偏航角速度范围 | — | `[-2, 2] rad/s` | `SimulationBridge` 进一步限制入口绝对值不超过 `1.5 rad/s`。 |
| `mit_robot/src/MPC/ConvexMPCLocomotion.cpp` | MPC 站立接触时长 | `horizon / horizon` | `10 / 10` 段 | 四足全支撑。 |
| `mit_robot/src/MPC/ConvexMPCLocomotion.cpp` | MPC TROT 相位偏移 | `{0, H/2, H/2, 0}` | `{0, 5, 5, 0}` | 腿序为 FR、FL、RR、RL。 |
| `mit_robot/src/MPC/ConvexMPCLocomotion.cpp` | MPC TROT 支撑时长 | `3H/5` | `6/10` 段（60%） | 与 `GaitScheduler` 的 `TROT_WALK` 保持一致。 |
| `mit_robot/include/MPC/SolverMPC.h` | `horizon` | `10` 段 | 未覆盖 | 预测段数。 |
| `mit_robot/include/MPC/SolverMPC.h` | `time_step` | `0.03 s` | 未覆盖 | 单段预测时间；总预测时域为 `10 × 0.03 = 0.30 s`，它不同于 25 Hz 基础求解周期。 |
| `mit_robot/include/MPC/SolverMPC.h` | `friction_coefficient` | `0.4` | 未覆盖 | MPC 摩擦锥系数，属于控制器约束，不是模型质量参数。 |
| `mit_robot/include/MPC/SolverMPC.h` | `minimum_normal_force` | `0 N` | 未覆盖 | 支撑脚法向力下界。 |
| `mit_robot/include/MPC/SolverMPC.h` | `maximum_normal_force` | `120 N` | 未覆盖 | MPC 单脚法向力上界。 |
| `mit_robot/include/MPC/SolverMPC.h` | `force_regularization` | `1e-5` | 未覆盖 | 力正则权重/数值稳定项。 |
| `mit_robot/include/MPC/SolverMPC.h` | `maximum_iterations` | `75` | 未覆盖 | 从60温和提高，增加25 Hz MPC单次求解余量，不降低500 Hz WBC/电机闭环频率。 |
| `mit_robot/include/MPC/SolverMPC.h` | `convergence_tolerance` | `1e-5` | 未覆盖 | 无穷范数残差收敛阈值。 |
| `mit_robot/src/MPC/SolverMPC.cpp` | 状态跟踪权重 | — | `{20,20,10, 2,2,50, 0.2,0.2,0.2, 1,1,2}` | 顺序为 `roll,pitch,yaw,x,y,z,roll_rate,pitch_rate,yaw_rate,vx,vy,vz`。 |
| `mit_robot/src/MPC/SolverMPC.cpp` | 欧拉角奇异阈值 | — | `1e-4` | `abs(cos(pitch))` 小于该值时拒绝求解。 |
| `mit_robot/src/MPC/RobotState.cpp` | 欧拉角速率奇异阈值 | — | `1e-4` | 状态映射的同类数值保护。 |

## 3. 行走足端、FSM 和安全参数

| 文件 | 参数 | 初始化/默认值 | 当前最终值 | 说明 |
|---|---|---:|---:|---|
| `mit_robot/include/FSM/FSM_State_Locomotion.h` | `swing_height_` | `0.10 m` | 未覆盖 | 保持原抬脚高度，不再通过增加高度间接提速。 |
| `mit_robot/include/FSM/FSM_State_Locomotion.h` | `maximum_step_length_` | `0.20 m` | 未覆盖 | 单步水平位移限幅。 |
| `mit_robot/include/FSM/FSM_State_Locomotion.h` | `swing_joint_velocity_scale_` | `[1,1.5,1.5]` | 未覆盖 | 仅摆动腿生效；Hip/Thigh/Calf速度前馈倍率，支撑腿不缩放，最终按GO1速度上限裁剪。 |
| `mit_robot/src/FSM/FSM_State_Locomotion.cpp` | 落脚支撑时间比例 | — | `0.5 × stance_time` | 落点预估使用 `swing_time + 0.5×stance_time`。 |
| `mit_robot/src/FSM/FSM_State_Locomotion.cpp` | 落脚速度误差反馈增益 | — | `0.08 s` | `step = placement_time × measured_velocity + 0.08 × velocity_error`。 |
| `mit_robot/src/FSM/FSM_State_Locomotion.cpp` | 行走目标步态 | MPC 初始化为 `STAND` | MPC=`TROT`，调度器=`TROT_WALK` | 两者最终都使用 60% 支撑率。 |
| `mit_robot/src/FSM/FSM_State_Locomotion.cpp` | 行走安全 roll / pitch | — | `40° / 40°` | 超限后回到 BalanceStand。 |
| `mit_robot/src/FSM/FSM_State_Locomotion.cpp` | 足端横向偏移 / 速度上限 | — | `0.18 m / 9 m/s` | 任一腿超限则退出行走。 |
| `mit_robot/src/FSM/FSM_State_BalanceStand.cpp` | 浮动基加速度权重 | WBC 默认 `0.1` | `1000` | 站立状态显著提高机身任务优先级；行走状态仍为 `0.1`。 |
| `mit_robot/src/FSM/FSM_State_BalanceStand.cpp` | 低高度回退阈值 / 回退高度 | — | `0.20 / 0.30 m` | 进入站立时估计高度过低才使用。 |
| `mit_robot/src/FSM/FSM_State_BalanceStand.cpp` | 第二层高度单周期限幅 | — | `0.001 m/周期` | 500 Hz 下等价 `0.5 m/s`；当前最终实际仍受 `RobotRunner` 的 `0.08 m/s` 更严格限制。 |
| `mit_robot/src/FSM/FSM_State_StandUp.cpp` | 站起轨迹时间 | — | `0.5 s` | smoothstep 足端下降轨迹。 |
| `mit_robot/src/FSM/FSM_State_StandUp.cpp` | 笛卡尔足端 `Kp / Kd` | — | `500 / 8` | 只在 StandUp 状态生效。 |
| `mit_robot/src/FSM/ControlFSM.cpp` | JointPd 状态 `Kp / Kd` | — | `20 / 2` | JointPd 保持 home 关节角。 |
| `mit_robot/src/FSM/FSM_State_RecoveryStand.cpp` | Recovery 关节 `Kp / Kd` | — | `20 / 2` | 所有恢复动作共用。 |
| `mit_robot/src/FSM/FSM_State_RecoveryStand.cpp` | 翻身斜坡 / 稳定时间 | — | `0.30 / 0.30 s` | 按控制周期换算为迭代次数。 |
| `mit_robot/src/FSM/FSM_State_RecoveryStand.cpp` | 收腿斜坡 / 稳定时间 | — | `0.80 / 1.40 s` | 按控制周期换算为迭代次数。 |
| `mit_robot/src/FSM/FSM_State_RecoveryStand.cpp` | 站起斜坡时间 | — | `0.50 s` | 按控制周期换算为迭代次数。 |
| `mit_robot/src/FSM/FSM_State_RecoveryStand.cpp` | 收腿目标 `[hip,thigh,calf]` | — | `[0, 1.4, -2.7] rad` | 运动目标，会再按模型关节限位裁剪。 |
| `mit_robot/src/FSM/FSM_State_RecoveryStand.cpp` | 翻身目标（右腿/左腿） | — | `[0.8,1.6,-2.77] / [0.8,3.1,-2.77] rad` | 运动目标，会再按模型关节限位裁剪。 |
| `mit_robot/src/FSM/FSM_State_RecoveryStand.cpp` | 可直接站起的高度窗口 | — | `(0.20, 0.45) m` | 非倒置且位于窗口内时直接执行 StandUp。 |
| `mit_robot/src/FSM/FSM_State_RecoveryStand.cpp` | 站起失败高度 / 检查进度 | — | `0.10 m / 70%` | 失败后重新收腿。 |
| `mit_robot/src/FSM/SafetyChecker.cpp` | 通用姿态安全上限 | — | `1.4 rad`（约 80°） | 普通 FSM 的紧急停止阈值；行走另有更严格的 40° 阈值。 |
| `mit_robot/src/FSM/SafetyChecker.cpp` | 足端水平范围比例 | — | `0.866 × 最大腿长` | 安全裁剪比例，不列出由机体尺寸计算出的最终长度。 |
| `mit_robot/src/FSM/SafetyChecker.cpp` | 足端 z 范围比例 | — | `[-1, -1/4] × 最大腿长` | 安全裁剪比例。 |

## 4. WBC、任务权重和 PD

| 文件 | 参数 | 初始化/默认值 | 当前最终值 | 说明 |
|---|---|---:|---:|---|
| `mit_robot/include/WBC/WBC_Ctrl/WBC_Ctrl.hpp` | 输出关节 `Kp` | `[5,5,5]` | 行走 `[30,42,42]`；站立 `[20,20,20]`；初始化 `[60,60,60]` | 顺序为hip、thigh、calf；配合速度前馈，不用过高刚度强行提速。 |
| `mit_robot/include/WBC/WBC_Ctrl/WBC_Ctrl.hpp` | 输出关节 `Kd` | `[1.5,1.5,1.5]` | 行走 `[4,4.5,4.5]`；站立 `[2,2,2]`；初始化 `[3,3,3]` | 提供适量阻尼；更高阻尼已验证会使高速落脚过硬并增加小腿擦地。 |
| `mit_robot/include/WBC/WBC_Ctrl/WBC_Ctrl.hpp` | 浮动基权重 | `0.1` | 行走 `0.1`；站立 `1000` | WBIC 中机身加速度修正的代价权重。 |
| `mit_robot/include/WBC/WBC_Ctrl/WBC_Ctrl.hpp` | 反作用力权重 | `1` | 未覆盖 | WBIC 对接触力修正的代价权重。 |
| `mit_robot/include/WBC/WBC_Ctrl/BodyPosTask.hpp` / `BodyPosTask.cpp` | 机身位置 `Kp_kin / Kp / Kd` | `[1,1,1] / [50,50,50] / [1,1,1]` | 未覆盖 | 站立和行走当前相同。 |
| `mit_robot/include/WBC/WBC_Ctrl/BodyOriTask.hpp` | 机身姿态 `Kp_kin / Kp / Kd` | `[1,1,1] / [50,50,50] / [1,1,1]` | 行走：`Kp=[100,100,50]`、`Kd=[10,10,3]`；站立保持默认 | 顺序为 roll、pitch、yaw。 |
| `mit_robot/include/WBC/WBC_Ctrl/LinkPosTask.hpp` | 摆动足位置 `Kp_kin / Kp / Kd` | `[1,1,1] / [100,100,100] / [5,5,5]` | 未覆盖 | 只对摆动腿的位置任务生效。 |
| `mit_robot/src/WBC/ContactSet/SingleContact.cpp` | WBC 接触摩擦系数 | `0.4` | 未覆盖 | WBC 摩擦棱锥，宜与 MPC 的 `0.4` 同步调整。 |
| `mit_robot/src/WBC/ContactSet/SingleContact.cpp` | WBC 单脚最大法向力 | `1500 N` | 未覆盖 | 可通过 `LocomotionCtrl::setMaxNormalForce()` 改写；这是控制约束而非电机额定转矩。 |
| `mit_robot/src/WBC/KinWBC.cpp` | 伪逆奇异值阈值 | `0.001` | 未覆盖 | KinWBC 数值稳定参数。 |
| `mit_robot/include/WBC/WBC.hpp` | 加权伪逆阈值 | `0.0001` | 未覆盖 | WBC 通用数值稳定参数。 |
| `mit_robot/src/controller/leg_controller.cpp` | 紧急阻尼 `gain` | 由调用方传入 | 当前正式链未调用 | 调用 `edampCommand()` 时等价于统一关节 `Kd=gain`、`Kp=0`。 |

## 5. 步态调度参数

`GaitSchedulerParameters` 的默认覆盖模式为 `USE_REQUESTED_GAIT`，默认步态为 `STAND`，覆盖周期和支撑比分别初始化为 `0.5 s / 0.5`。只有把 `override_mode` 改成相应覆盖模式时，这两个覆盖值才替代下表的预定义时序。正式行走由 FSM 请求 `TROT_WALK`。

| 文件 | 步态 | 周期 `s` | 支撑比 | 相位偏移 FR/FL/RR/RL | 当前正式链 |
|---|---|---:|---:|---|---|
| `mit_robot/src/controller/GaitScheduler.cpp` | STAND | `10` | `1.0` | `0.5/0.5/0.5/0.5` | 站立状态使用 |
| 同上 | STAND_CYCLE | `1.0` | `1.0` | `0.5/0.5/0.5/0.5` | 未使用 |
| 同上 | STATIC_WALK | `1.25` | `0.8` | `0.25/0/0.75/0.5` | 未使用 |
| 同上 | AMBLE | `0.5` | `0.625` | `0/0.5/0.25/0.75` | 未使用 |
| 同上 | TROT_WALK | `0.5` | `0.6` | `0/0.5/0.5/0` | **行走最终值**；支撑 `0.30 s`、摆动 `0.20 s` |
| 同上 | TROT | `0.5` | `0.5` | `0/0.5/0.5/0` | 未使用 |
| 同上 | TROT_RUN | `0.4` | `0.4` | `0/0.5/0.5/0` | 未使用 |
| 同上 | PACE | `0.35` | `0.5` | `0/0.5/0/0.5` | 未使用；初始相位 `0.25` |
| 同上 | BOUND | `0.4` | `0.4` | `0/0/0.5/0.5` | 未使用 |
| 同上 | ROTARY_GALLOP | `0.4` | `0.2` | `0/0.8571/0.3571/0.5` | 未使用 |
| 同上 | TRAVERSE_GALLOP | `0.5` | `0.2` | `0/0.8571/0.3571/0.5` | 未使用 |
| 同上 | PRONK | `0.5` | `0.5` | `0/0/0/0` | 未使用 |
| 同上 | THREE_FOOT | `0.4` | `0.666` | `0/0.666/0/0.333` | 未使用；FR 禁用，其余腿相位缩放为 1 |
| `mit_robot/include/controller/GaitScheduler.hpp` | 估计器接触过渡比例 | — | `0.2` | — | 支撑相开始和结束各 20% 区间内平滑接触可信度 |
| `mit_robot/src/controller/GaitScheduler.cpp` | TRANSITION_TO_STAND 周期倍率 | 原步态周期 | `3 × 原周期` | 由当前相位重算 | 未使用 |

## 6. 状态估计参数

| 文件 | 参数 | 初始化/默认值 | 当前最终值 | 说明 |
|---|---|---:|---:|---|
| `mit_robot/include/controller/PositionVelocityEstimator.hpp` | `nominal_time_step` | `0.002 s` | `model->opt.timestep = 0.002 s` | 在 `RobotRunner.cpp` 中显式覆盖。 |
| 同上 | `maximum_time_step` | `0.1 s` | `max(0.1, nominal_time_step) = 0.1 s` | 超过后拒绝连续积分。 |
| 同上 | `process_noise_position` | `0.02` | 未覆盖 | 位置过程噪声缩放。 |
| 同上 | `process_noise_velocity` | `0.02` | 未覆盖 | 速度过程噪声缩放。 |
| 同上 | `process_noise_foot_position` | `0.002` | 未覆盖 | 足端世界位置过程噪声缩放。 |
| 同上 | `sensor_noise_relative_position` | `0.001` | 未覆盖 | 机身—足端相对位置观测噪声。 |
| 同上 | `sensor_noise_relative_velocity` | `0.1` | 未覆盖 | 支撑足零速度观测噪声。 |
| 同上 | `sensor_noise_foot_height` | `0.001` | 未覆盖 | 支撑足地面高度观测噪声。 |
| 同上 | `initial_covariance` | `100` | 未覆盖 | 滤波器初始协方差。 |
| 同上 | `suspect_noise_multiplier` | `100` | 未覆盖 | 接触可信度下降时的最大噪声放大倍数。 |
| `mit_robot/src/controller/PositionVelocityEstimator.cpp` | 水平协方差重整阈值 / 缩放 | `1e-6 / 10` | 未覆盖 | 水平位置协方差过大时使用的数值稳定处理。 |
| `mit_robot/include/controller/OrientationEstimator.hpp` | `maximum_sensor_skew` | `0.02 s` | `0.02 s` | IMU 与腿反馈最大时间差。 |
| 同上 | 加速度姿态修正增益 | `2 s⁻¹` | 当前 `SIMULATION_TRUTH` 下未启用 | 切换到 `IMU_FUSION` 后生效。 |
| 同上 | IMU 绝对姿态修正增益 | `10 s⁻¹` | 当前 `SIMULATION_TRUTH` 下未启用 | 切换到 `IMU_FUSION` 后生效。 |
| `mit_robot/src/controller/OrientationEstimator.cpp` | 最大融合积分周期 | `0.1 s` | 当前 `SIMULATION_TRUTH` 下未启用 | 超过则拒绝本次融合更新。 |
| 同上 | 加速度重力判定窗口 | `0.5g～1.5g` | 当前 `SIMULATION_TRUTH` 下未启用 | 只有落在窗口内才用加速度修正倾角。 |
| `mit_robot/include/controller/ContactEstimator.hpp` | 最大传感器时差 | `0.02 s` | 当前正式链未启用该估计器 | 正式链使用 GaitScheduler 的接触可信度。 |
| 同上 | `force_ratio_gain` | `8` | 未启用 | 接触概率 sigmoid 斜率。 |
| 同上 | `force_ratio_threshold` | `0.3` | 未启用 | 接触概率参考比例。 |
| 同上 | `contact_probability_threshold` | `0.5` | 未启用 | 二值接触阈值。 |
| 同上 | `minimum_support_ratio` | `0.2` | 未启用 | 期望支撑力下限比例。 |
| 同上 | `force_estimation_damping` | `1e-3` | 未启用 | 足端力阻尼最小二乘系数。 |

## 调参时的生效顺序

1. 入口目标（`user/main.cpp`、UI、ROS 2）先写入 `RobotRunner`。
2. `RobotRunner` 的 500 Hz 控制周期运行状态估计、FSM 和 WBC。
3. Locomotion 状态把 MPC 刷新周期设为 0.04 s，因此求解器以 25 Hz 基础频率刷新；步态仍按独立的 0.05 s 分段推进，接触切换时立即补算。
4. WBC 头文件里的默认 PD 会被具体 FSM 状态覆盖；调行走必须修改 Locomotion 的最终值，调站立必须修改 BalanceStand 的最终值。
5. 仿真执行端最终使用 `前馈力矩 + Kp(q_des-q) + Kd(qd_des-qd)`，MuJoCo XML 的位置伺服器在当前桥接方式下没有位置误差。

# DM1 description

本目录是工程内第二套四足模型。模型尺寸、惯量、关节限制和 STL 网格来自：

`~/桌面/legged_examples/legged_damiao/legged_damiao_description`

源模型名称映射到控制框架的统一顺序如下：

| 源模型 | 控制框架 | 关节映射 |
| --- | --- | --- |
| RF | FR | HAA / HFE / KFE → hip / thigh / calf |
| LF | FL | HAA / HFE / KFE → hip / thigh / calf |
| RH | RR | HAA / HFE / KFE → hip / thigh / calf |
| LH | RL | HAA / HFE / KFE → hip / thigh / calf |

运行控制程序：

```bash
ros2 run mymit_robot mymit_robot_user --robot dm1
```

仅查看模型：

```bash
ros2 launch mymit_robot dm1_display.launch.py
```

## 平衡站立校准

DM1 使用独立于 GO1 的站立参数和机械 Home 姿态。KFE（calf）站立零位为：

- 前腿 FR/FL：`-0.818 rad`
- 后腿 RR/RL：`-0.782 rad`

前后腿差动用于补偿 DM1 质心和连杆布置造成的静态俯仰；四腿平均姿态与更硬的
足底接触参数共同让实际机身高度接近 `0.39 m`，并避免小腿依靠地面支撑。控制参数位于
`include/model/robot_control_parameters.hpp`，DM1 的站立姿态和 MuJoCo `home`
关键帧必须同步修改。

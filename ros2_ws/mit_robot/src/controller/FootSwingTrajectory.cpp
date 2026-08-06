// ============================================================
// 单足摆动轨迹实现
// 使用三次 Bézier（smoothstep）曲线生成足端位置、速度和加速度。
// ============================================================

#include "controller/FootSwingTrajectory.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

// ---------- 内部三次 Bézier 插值函数 ----------
namespace
{

// 三次 Bézier 的归一化混合系数：b(s) = 3s^2 - 2s^3。
// b(0)=0、b(1)=1，且两端一阶导数均为零，因此足端起落时速度连续为零。
template<typename T>
T cubicBlend(T phase)
{
  return phase * phase * (T(3) - T(2) * phase);
}

// 混合系数对归一化相位 s 的一阶导数：db/ds = 6s(1-s)。
template<typename T>
T cubicBlendFirstDerivative(T phase)
{
  return T(6) * phase * (T(1) - phase);
}

// 混合系数对归一化相位 s 的二阶导数：d²b/ds² = 6-12s。
template<typename T>
T cubicBlendSecondDerivative(T phase)
{
  return T(6) - T(12) * phase;
}

// 在 start 和 finish 之间进行三次 Bézier 插值。
// Value 既可以是标量 T，也可以是当前工程定义的 Vec3<T>。
template<typename Value, typename T>
Value cubicBezier(const Value & start, const Value & finish, T phase)
{
  return start + (finish - start) * cubicBlend(phase);
}

// 对归一化相位求一阶导数；换算成真实时间导数时还需除以轨迹时长。
template<typename Value, typename T>
Value cubicBezierFirstDerivative(
  const Value & start, const Value & finish, T phase)
{
  return (finish - start) * cubicBlendFirstDerivative(phase);
}

// 对归一化相位求二阶导数；换算成真实时间二阶导数时需除以时长平方。
template<typename Value, typename T>
Value cubicBezierSecondDerivative(
  const Value & start, const Value & finish, T phase)
{
  return (finish - start) * cubicBlendSecondDerivative(phase);
}

}  // namespace

// ---------- 轨迹参数设置 ----------

template<typename T>
FootSwingTrajectory<T>::FootSwingTrajectory()
{
  reset();
}

// 将规划参数和输出同时恢复为安全零值。
template<typename T>
void FootSwingTrajectory<T>::reset()
{
  initial_position_.setZero();
  final_position_.setZero();
  position_.setZero();
  velocity_.setZero();
  acceleration_.setZero();
  height_ = T(0);
}

template<typename T>
void FootSwingTrajectory<T>::setInitialPosition(
  const Vec3<T> & initial_position)
{
  initial_position_ = initial_position;
}

template<typename T>
void FootSwingTrajectory<T>::setFinalPosition(
  const Vec3<T> & final_position)
{
  final_position_ = final_position;
}

template<typename T>
void FootSwingTrajectory<T>::setHeight(T height)
{
  height_ = height;
}

// ---------- 三次 Bézier 摆动轨迹计算 ----------

template<typename T>
void FootSwingTrajectory<T>::computeSwingTrajectoryBezier(
  T phase, T swing_time)
{
  // 在参与除法和矩阵运算前统一检查参数，防止 NaN/Inf 进入控制器。
  if (!std::isfinite(static_cast<double>(phase)) ||
    !std::isfinite(static_cast<double>(swing_time)) || swing_time <= T(0) ||
    !std::isfinite(static_cast<double>(height_)) || height_ < T(0) ||
    !initial_position_.allFinite() || !final_position_.allFinite())
  {
    throw std::invalid_argument("invalid foot swing trajectory parameters");
  }

  // 控制周期离散积分可能让相位略小于 0 或略大于 1，这里限制到合法范围。
  const T clamped_phase = std::clamp(phase, T(0), T(1));

  // x/y/z 先沿起点到终点计算一条完整曲线；随后单独覆盖 z 方向，形成抬脚弧线。
  position_ = cubicBezier(initial_position_, final_position_, clamped_phase);
  velocity_ = cubicBezierFirstDerivative(
    initial_position_, final_position_, clamped_phase) / swing_time;
  acceleration_ = cubicBezierSecondDerivative(
    initial_position_, final_position_, clamped_phase) /
    (swing_time * swing_time);

  const T highest_z = initial_position_.z() + height_;
  T vertical_position = T(0);
  T vertical_velocity = T(0);
  T vertical_acceleration = T(0);

  if (clamped_phase < T(0.5)) {
    // 前半程：从起点高度抬到最高点。局部相位是全局相位的 2 倍。
    const T local_phase = clamped_phase * T(2);
    vertical_position = cubicBezier(initial_position_.z(), highest_z, local_phase);
    // ds_local/ds_global=2，因此速度需要乘 2。
    vertical_velocity = cubicBezierFirstDerivative(
      initial_position_.z(), highest_z, local_phase) * T(2) / swing_time;
    // 二阶时间导数对应乘 2^2=4。
    vertical_acceleration = cubicBezierSecondDerivative(
      initial_position_.z(), highest_z, local_phase) * T(4) /
      (swing_time * swing_time);
  } else {
    // 后半程：从最高点落到目标高度，局部相位仍从 0 变化到 1。
    const T local_phase = clamped_phase * T(2) - T(1);
    vertical_position = cubicBezier(highest_z, final_position_.z(), local_phase);
    vertical_velocity = cubicBezierFirstDerivative(
      highest_z, final_position_.z(), local_phase) * T(2) / swing_time;
    vertical_acceleration = cubicBezierSecondDerivative(
      highest_z, final_position_.z(), local_phase) * T(4) /
      (swing_time * swing_time);
  }

  // 用两段抬脚曲线的结果覆盖完整曲线原本的 z 分量。
  position_.z() = vertical_position;
  velocity_.z() = vertical_velocity;
  acceleration_.z() = vertical_acceleration;
}

// 模板实现位于 .cpp，因此显式生成当前工程可能使用的两种精度。
template class FootSwingTrajectory<float>;
template class FootSwingTrajectory<double>;
    
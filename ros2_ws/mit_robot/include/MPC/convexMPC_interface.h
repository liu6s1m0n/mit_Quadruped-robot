/**
 * @file convexMPC_interface.h
 * @brief 旧版凸 MPC 求解器的 C ABI 兼容入口。
 *
 * 现代 C++ 控制链优先使用 MPC/SolverMPC.h 和
 * MPC/ConvexMPCLocomotion.h；本接口仅供历史调用方通过原始数组访问。
 */
#ifndef _convexmpc_interface
#define _convexmpc_interface
#define K_MAX_GAIT_SEGMENTS 36

//#include "common_types.h"

#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

struct problem_setup
{
  float dt;       ///< 预测离散时间间隔。
  float mu;       ///< 地面摩擦系数。
  float f_max;    ///< 单腿最大法向力。
  int horizon;    ///< 预测窗长度。
};

struct update_data_t
{
  float p[3];  ///< 当前机身位置。
  float v[3];  ///< 当前机身速度。
  float q[4];  ///< 当前姿态四元数。
  float w[3];  ///< 当前角速度。
  float r[12];  ///< 当前四条腿足端位置。
  float yaw;  ///< 当前偏航角。
  float weights[12];  ///< 状态/力优化权重。
  float traj[12*K_MAX_GAIT_SEGMENTS];  ///< 预测窗状态轨迹。
  float alpha;  ///< 旧版求解器的附加参数。
  unsigned char gait[K_MAX_GAIT_SEGMENTS];  ///< 预测窗接触表。
  unsigned char hack_pad[1000];  ///< 保留旧 ABI 布局的填充区。
  int max_iterations;  ///< 最大迭代次数。
  double rho, sigma, solver_alpha, terminate;  ///< 旧版求解器参数。
  int use_jcqp;  ///< 是否使用旧版 JCQP 求解路径。
  float x_drag;  ///< 旧版 x 方向阻力补偿参数。
};

/** @brief 初始化或更新旧版 MPC 的基本问题参数。 */
EXTERNC void setup_problem(double dt, int horizon, double mu, double f_max);
/** @brief 复制 double 输入数组并启动一次旧版 MPC 求解。 */
EXTERNC void update_problem_data(double* p, double* v, double* q, double* w, double* r, double yaw, double* weights, double* state_trajectory, double alpha, int* gait);
/** @brief 获取旧版求解器输出数组中的一个元素。 */
EXTERNC double get_solution(int index);
/** @brief 更新旧版求解器的迭代参数。 */
EXTERNC void update_solver_settings(int max_iter, double rho, double sigma, double solver_alpha, double terminate, double use_jcqp);
/** @brief 复制 float 输入数组并启动一次旧版 MPC 求解。 */
EXTERNC void update_problem_data_floats(float* p, float* v, float* q, float* w,
                                        float* r, float yaw, float* weights,
                                        float* state_trajectory, float alpha, int* gait);

/** @brief 更新旧版 x 方向阻力补偿参数。 */
void update_x_drag(float x_drag);
#endif

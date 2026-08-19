/**
 * @file WBIC.hpp
 * @brief 动力学层全身控制器，通过二次规划求广义加速度修正和接触力修正。
 *
 * 最终关节力矩满足浮动基动力学等式，同时满足足端摩擦与法向力不等式。
 */
#ifndef WHOLE_BODY_IMPULSE_CONTROL_H
#define WHOLE_BODY_IMPULSE_CONTROL_H

#include "Utilities/QuadProg++.hh"
#include "WBC/ContactSpec.hpp"
#include "WBC/Task.hpp"
#include "WBC/WBC.hpp"
#include <cstddef>

template<typename T>
/**
 * @brief WBIC 的输入权重和输出缓存。
 *
 * 优化变量通常为 @f$z=[\Delta\ddot q_{base},\Delta F]^T@f$，
 * 其中前 6 维是浮动基座加速度修正，后面是接触力修正。
 *
 * @tparam T 标量类型。
 */
class WBIC_ExtraData
{
public:
  /** @brief 二次规划最优解。 */
  DVec<T> _opt_result;
  /** @brief 最终广义加速度 @f$\ddot q@f$。 */
  DVec<T> _qddot;
  /** @brief 最终接触反力 @f$F@f$。 */
  DVec<T> _Fr;

  /** @brief 浮动基座加速度修正项的正定权重。 */
  DVec<T> _W_floating;
  /** @brief 接触反力修正项的正定权重。 */
  DVec<T> _W_rf;

  /** @brief 使用空权重初始化。 */
  WBIC_ExtraData() = default;
  /**
   * @brief 按优化变量维度初始化单位权重。
   * @param floating_dimension 浮动基座修正维数，通常为 6。
   * @param reaction_force_dimension 所有接触反力的总维数。3x4 =12
   */
  WBIC_ExtraData(std::size_t floating_dimension, std::size_t reaction_force_dimension)
  : _W_floating(DVec<T>::Ones(floating_dimension)),
    _W_rf(DVec<T>::Ones(reaction_force_dimension)) {}
  ~WBIC_ExtraData() = default;
};

template<typename T>
using WBICExtraData = WBIC_ExtraData<T>;

template<typename T>
/**
 * @brief 全身逆动力学控制器。
 *
 * WBIC 在 KinWBC/任务层得到的名义加速度附近建立二次规划，优化浮动基座
 * 加速度和接触反力，使其同时满足：
 *
 * @f$ A\ddot q+C+G=S^T\tau+J_c^TF @f$
 *
 * 的浮动基座动力学等式、接触加速度约束以及摩擦锥不等式。最终只输出
 * 可驱动关节的力矩。
 *
 * @tparam T 标量类型。
 */
class WBIC : public WBC<T>
{
public:
  /**
   * @brief 创建 WBIC。Sa_：驱动关节选择矩阵  Sv_：浮动基选择矩阵
   * @param num_qdot 广义速度维数。
   * @param contact_list 当前接触约束列表指针。
   * @param task_list 按优先级排列的任务列表指针。
   * @throws std::invalid_argument 当任一列表指针为空时抛出。
   */
  WBIC(
    size_t num_qdot, const std::vector<ContactSpec<T> *> * contact_list,
    const std::vector<Task<T> *> * task_list);
  /** @brief 析构函数。 */
  ~WBIC() override = default;

  /**
   * @brief 更新当前时刻的动力学矩阵和偏置项。
   * @param A 广义质量矩阵。
   * @param Ainv 质量矩阵的逆或加权逆。
   * @param cori 科里奥利/离心力向量。
   * @param grav 重力向量。
   * @param extra_setting 保留的扩展参数。
   */
  virtual void UpdateSetting(
    const DMat<T> & A, const DMat<T> & Ainv,
    const DVec<T> & cori, const DVec<T> & grav,
    void * extra_setting = nullptr) override;

  /** @brief 通过基类接口计算关节力矩；输入为空时输出零力矩。 */
  void MakeTorque(DVec<T> & cmd, void * extra_input = nullptr) override;
  /**
   * @brief 执行 WBIC 二次规划并输出关节力矩。
   * @param cmd 输出的可驱动关节力矩。
   * @param data 输入权重及输出缓存。
   * @return 求解成功且输出有限时返回 true。
   */
  bool makeTorque(DVec<T> & cmd, WBIC_ExtraData<T> & data);

private:
  const std::vector<ContactSpec<T> *> * _contact_list;//接触约束
  const std::vector<Task<T> *> * _task_list;//任务约束

  void _SetEqualityConstraint(const DVec<T> & qddot);//浮动基动力学约束
  void _SetInEqualityConstraint();//接触力不等式约束
  void _ContactBuilding();//接触约束构建

  void _GetSolution(const DVec<T> & qddot, DVec<T> & cmd);//根据优化结果恢复关节力矩
  void _SetCost();//构造 QP 代价函数
  void _SetOptimizationSize();//根据接触数量确定变量和矩阵维度
  bool _MakeTorqueInternal(DVec<T> & cmd);//执行一次完整求解
  bool _ValidateInputs(const WBIC_ExtraData<T> & data) const;//检查输入矩阵是否合法

  /** @brief 优化变量维数：浮动基座修正加所有接触反力修正。6+nf=18 */
  size_t _dim_opt;
  /** @brief 等式约束维数，浮动基座动力学为 6。 */
  size_t _dim_eq_cstr;

  /** @brief 所有接触反力的总维数。nf */
  size_t _dim_rf;
  /** @brief 所有接触力不等式的总行数。 */
  size_t _dim_Uf;

  /** @brief 浮动基座维数，固定为 6。 */
  size_t _dim_floating;

  /** @brief 当前一次求解所使用的输入输出缓存。 */
  WBIC_ExtraData<T> * _data;

  GolDIdnani::GVect<double> z;   ///< QP 解：[浮动基加速度修正；接触力修正]。
  GolDIdnani::GMatr<double> G;   ///< QP 二次代价矩阵，使 1/2 z^T G z 最小。
  GolDIdnani::GVect<double> g0;  ///< QP 线性代价向量；当前问题通常设为零。

  GolDIdnani::GMatr<double> CE;   ///< 求解器格式的等式矩阵，满足 CE^T z + ce0 = 0。
  GolDIdnani::GVect<double> ce0;  ///< 等式约束常数项，主要来自浮动基动力学残差。

  GolDIdnani::GMatr<double> CI;   ///< 求解器格式的不等式矩阵，满足 CI^T z + ci0 >= 0。
  GolDIdnani::GVect<double> ci0;  ///< 摩擦锥及法向力上下界的不等式常数项。

  DMat<T> _dyn_CE;   ///< Eigen 格式的动力学等式系数，转置后复制给 CE。
  DVec<T> _dyn_ce0;  ///< Eigen 格式的动力学等式残差。
  DMat<T> _dyn_CI;   ///< Eigen 格式的接触力不等式系数，转置后复制给 CI。
  DVec<T> _dyn_ci0;  ///< Eigen 格式的接触力不等式余量。

  DMat<T> _eye;           ///< num_qdot 维单位阵，用于构造任务零空间投影。
  DMat<T> _eye_floating;  ///< 6 维单位阵，将浮动基修正嵌入优化变量。

  DMat<T> _S_delta;     ///< 从 z 中选择浮动基加速度修正的选择矩阵。
  DMat<T> _Uf;          ///< 汇总所有接触点摩擦锥/法向力约束的块对角矩阵。
  DVec<T> _Uf_ieq_vec;  ///< 与 _Uf 配套的接触不等式边界向量。

  DMat<T> _Jc;          ///< 堆叠后的总接触雅可比，行数等于 _dim_rf。
  DVec<T> _JcDotQdot;   ///< 接触雅可比变化项 Jc_dot * qdot，即接触偏置加速度。
  DVec<T> _Fr_des;      ///< 各接触点期望反力拼接向量，单位 N。

  DMat<T> _B;       ///< 由动力学等式消元得到的接触力到基座修正映射。
  DVec<T> _c;       ///< 与 _B 配套的动力学常数项。
  DVec<T> task_cmd_;  ///< 临时保存当前优先级任务的操作空间期望加速度。
};

#endif

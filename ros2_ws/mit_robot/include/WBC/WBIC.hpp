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

  GolDIdnani::GVect<double> z;
  // Cost
  GolDIdnani::GMatr<double> G;
  GolDIdnani::GVect<double> g0;

  // Equality
  GolDIdnani::GMatr<double> CE;
  GolDIdnani::GVect<double> ce0;

  // Inequality
  GolDIdnani::GMatr<double> CI;
  GolDIdnani::GVect<double> ci0;

  DMat<T> _dyn_CE;
  DVec<T> _dyn_ce0;
  DMat<T> _dyn_CI;
  DVec<T> _dyn_ci0;

  DMat<T> _eye;
  DMat<T> _eye_floating;

  DMat<T> _S_delta;
  DMat<T> _Uf;
  DVec<T> _Uf_ieq_vec;

  DMat<T> _Jc;
  DVec<T> _JcDotQdot;
  DVec<T> _Fr_des;

  DMat<T> _B;
  DVec<T> _c;
  DVec<T> task_cmd_;
};

#endif

/**
 * @file SingleContact.hpp
 * @brief 单个三维点接触约束的定义。
 *
 * 该类将一个足端接触建模为三维平移接触：足端的线速度和线加速度
 * 在接触约束下应为零，同时使用线性化摩擦锥限制接触力。
 * 新代码优先包含 WBC/ContactSet/SingleContact.hpp；本文件保留为旧路径兼容接口。
 */
#ifndef Cheetah_SINGLE_CONTACT
#define Cheetah_SINGLE_CONTACT

#include <cstddef>

#include <WBC/FloatingBaseModel.h>
#include <WBC/ContactSpec.hpp>

/**
 * @brief 单个足端的三维点接触约束。
 *
 * 接触雅可比满足 @f$ v_{foot}=J_c\dot{q} @f$，固定接触时还满足
 * @f$ J_c\ddot{q}+\dot{J}_c\dot{q}=0 @f$。
 * 接触力约束采用 @f$ U_fF\geq i_{eq} @f$ 的形式，其中
 * @f$ F=[F_x,F_y,F_z]^T @f$。
 *
 * @tparam T 标量类型，通常为 float 或 double。
 */
template <typename T>
class SingleContact : public ContactSpec<T> {
 public:
  /**
   * @brief 根据机器人模型和接触点编号创建点接触约束。
   * @param robot 浮动基座机器人模型指针，不能为空。
   * @param contact_pt 接触点在机器人模型接触点列表中的索引。
   * @throws std::invalid_argument 当 robot 为空时抛出。
   * @throws std::out_of_range 当 contact_pt 超出模型接触点范围时抛出。
   */
  SingleContact(const FloatingBaseModel<T>* robot, int contact_pt);

  /**
   * @brief 根据机器人模型引用和接触点编号创建点接触约束。
   * @param robot 浮动基座机器人模型引用。
   * @param contact_pt 接触点在机器人模型接触点列表中的索引。
   */
  SingleContact(const FloatingBaseModel<T>& robot, std::size_t contact_pt);

  /** @brief 析构函数。 */
  ~SingleContact() override = default;

  /**
   * @brief 设置足端允许的最大法向力。
   * @param max_fz 最大法向力 @f$F_{z,max}@f$，单位通常为 N。
   * 新值会在下一次 _UpdateInequalityVector() 调用时写入右端项。
   */
  void setMaxFz(T max_fz) { _max_Fz = max_fz; }

 protected:
  /** @brief 足端允许的最大法向力 @f$F_{z,max}@f$。 */
  T _max_Fz;

  /** @brief 当前接触点在 FloatingBaseModel 接触点列表中的索引。 */
  std::size_t _contact_pt;

  /**
   * @brief 力不等式矩阵的行数。
   * 六行分别对应法向力、两个 x 方向摩擦约束、两个 y 方向摩擦约束和
   * 最大法向力约束。
   */
  std::size_t _dim_U;

  /** @brief 从机器人模型更新足端接触雅可比 @f$J_c@f$。 */
  bool _UpdateJc() override;
  /** @brief 从机器人模型更新偏置项 @f$\dot{J}_c\dot{q}@f$。 */
  bool _UpdateJcDotQdot() override;
  /** @brief 更新接触力不等式矩阵 @f$U_f@f$。 */
  bool _UpdateUf() override;
  /** @brief 更新接触力不等式右端向量 @f$i_{eq}@f$。 */
  bool _UpdateInequalityVector() override;

  /** @brief 提供机器人模型，用于读取接触雅可比及其导数项。 */
  const FloatingBaseModel<T>* robot_sys_;
};

#endif

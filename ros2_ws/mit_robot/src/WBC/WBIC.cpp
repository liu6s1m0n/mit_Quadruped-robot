/**
 * @file WBIC.cpp
 * @brief WBIC 二次规划、动力学约束和关节力矩恢复的实现。sv = [i6,0] sa = [0,in-6]。
 *        难度很大基本上看不懂
 */

#include "WBC/WBIC.hpp"
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/SVD>

#include <cmath>
#include <exception>
#include <stdexcept>

/**
 * @brief 构造 WBIC 并绑定任务、接触列表。
 * @param num_qdot 广义速度维数。
 * @param contact_list 接触约束列表。
 * @param task_list 任务列表。
 */
template<typename T>
WBIC<T>::WBIC(
  size_t num_qdot, const std::vector<ContactSpec<T> *> * contact_list,
  const std::vector<Task<T> *> * task_list)
: WBC<T>(num_qdot), _dim_floating(6)
{
  if (contact_list == nullptr || task_list == nullptr) {
    throw std::invalid_argument("WBIC task and contact lists must not be null");
  }
  _contact_list = contact_list;
  _task_list = task_list;
  _data = nullptr;
  /*_eye 主要用于构造任务零空间：*/
  _eye = DMat<T>::Identity(WB::num_qdot_, WB::num_qdot_);
  _eye_floating = DMat<T>::Identity(_dim_floating, _dim_floating);
}

template<typename T>
/**
 * @brief 基类兼容入口，调用 makeTorque() 完成一次 WBIC 求解。
 */
void WBIC<T>::MakeTorque(DVec<T> & cmd, void * extra_input)
{
  /*12个关节总共12维度*/
  cmd = DVec<T>::Zero(WB::num_act_joint_);
  if (extra_input == nullptr) {return;}
  (void)makeTorque(cmd, *static_cast<WBIC_ExtraData<T> *>(extra_input));
}

template<typename T>
/**
 * @brief 清空输出、校验输入并执行一次二次规划。
 * @return 求解成功且关节力矩有限时返回 true。
 */
bool WBIC<T>::makeTorque(DVec<T> & cmd, WBIC_ExtraData<T> & data)
{ 
  /*每次求解前清空上一次结果，防止失败时继续使用旧数据。*/
  cmd = DVec<T>::Zero(WB::num_act_joint_);
  data._opt_result.resize(0);
  data._qddot.resize(0);
  data._Fr.resize(0);
  /*将当前输入输出缓存保存到成员变量中，供其他内部函数使用。*/
  if (!_ValidateInputs(data)) {return false;}
  _data = &data;
  try {
    if (!_MakeTorqueInternal(cmd)) {
      cmd.setZero();
      return false;
    }
  } catch (const std::exception &) {
    cmd.setZero();
    data._opt_result.resize(0);
    data._qddot.resize(0);
    data._Fr.resize(0);
    return false;
  }
  return true;
}

template<typename T>
/**
 * @brief 完成一次 WBIC 内部流程：构造接触约束、任务加速度、QP 并恢复力矩。
 *
 * 优化变量为 @f$z=[\Delta\ddot q_b,\Delta F]^T@f$；QP 的目标惩罚修正量，
 * 等式约束保证浮动基座动力学， 不等式约束保证接触反力处于摩擦锥内。
 */
bool WBIC<T>::_MakeTorqueInternal(DVec<T> & cmd)
{

  // 优化变量 z = [6 维浮动基座加速度修正, 各接触点反力修正]。
  _SetOptimizationSize();
  _SetCost();  // 构造 QP 代价函数  z为主要惩罚

  DVec<T> qddot_pre; //任务层最终得到的名义加速度
  DMat<T> JcBar;// 接触雅可比的加权广义逆
  DMat<T> Npre; //当前任务之前的零空间投影矩阵

  if (_dim_rf > 0) {
    // 合并所有支撑脚的接触雅可比、期望反力和摩擦约束。
    _ContactBuilding();

    //一个是足端加速度，另一个是由速度形成的向心加速度
    // 先求满足 Jc*qddot + JcDot*qdot = 0 的接触一致加速度。
    _SetInEqualityConstraint();
    //计算接触雅可比伪逆
    WB::_WeightedInverse(_Jc, WB::Ainv_, JcBar);
    qddot_pre = JcBar * (-_JcDotQdot);
    /*Nc = I - JcBar * _Jc*/
    Npre = _eye - JcBar * _Jc;
    // pretty_print(JcBar, std::cout, "JcBar");
    // pretty_print(_JcDotQdot, std::cout, "JcDotQdot");
    // pretty_print(qddot_pre, std::cout, "qddot 1");
  }
  /*没有接触时清空 并且没有需要避让的接触约束，所以零空间为：I*/
  else {
    qddot_pre = DVec<T>::Zero(WB::num_qdot_);
    Npre = _eye;
  }

  // 任务列表按从高到低的优先级排列，并逐层投影到前级任务的零空间。
  Task<T> * task;
  DMat<T> Jt, JtBar, JtPre;
  DVec<T> JtDotQdot, xddot;

  for (size_t i(0); i < (*_task_list).size(); ++i) {
    task = (*_task_list)[i];

    task->getTaskJacobian(Jt);
    task->getTaskJacobianDotQdot(JtDotQdot);
    task->getCommand(xddot);
    //这样任务只能利用前面任务没有使用的自由度。
    JtPre = Jt * Npre;
    WB::_WeightedInverse(JtPre, WB::Ainv_, JtBar);
    //更新加速度 更新任务零空间
    qddot_pre += JtBar * (xddot - JtDotQdot - Jt * qddot_pre);
    Npre = Npre * (_eye - JtBar * JtPre);
  }

  // 浮动基座没有执行器，其 6 维动力学平衡必须作为等式约束严格满足。
  _SetEqualityConstraint(qddot_pre);

  // QuadProg 的约定为 CE^T z + ce0 = 0，CI^T z + ci0 >= 0。
  const double f = solve_quadprog(G, g0, CE, ce0, CI, ci0, z);
  // std::cout<<"\n wbic old time: "<<timer.getMs()<<std::endl;
  if (!std::isfinite(f)) {return false;}
  for (size_t i = 0; i < _dim_opt; ++i) {
    if (!std::isfinite(z[i])) {return false;}
  }

  // pretty_print(qddot_pre, std::cout, "qddot_cmd");
  for (size_t i(0); i < _dim_floating; ++i) {
    // 把优化得到的浮动基座修正量叠加到任务层级给出的名义加速度上。
    qddot_pre[i] += z[i];
  }
  _GetSolution(qddot_pre, cmd);

  _data->_opt_result = DVec<T>(_dim_opt);
  for (size_t i(0); i < _dim_opt; ++i) {
    _data->_opt_result[i] = z[i];
  }

  return cmd.allFinite();

  // std::cout << "f: " << f << std::endl;
  //std::cout << "x: " << z << std::endl;

  // DVec<T> check_eq = _dyn_CE * _data->_opt_result + _dyn_ce0;
  // pretty_print(check_eq, std::cout, "equality constr");
  // std::cout << "cmd: "<<cmd<<std::endl;
  // pretty_print(qddot_pre, std::cout, "qddot_pre");
  // pretty_print(JcN, std::cout, "JcN");
  // pretty_print(Nci_, std::cout, "Nci");
  // DVec<T> eq_check = dyn_CE * data_->opt_result_;
  // pretty_print(dyn_ce0, std::cout, "dyn ce0");
  // pretty_print(eq_check, std::cout, "eq_check");

  // pretty_print(Jt, std::cout, "Jt");
  // pretty_print(JtDotQdot, std::cout, "Jtdotqdot");
  // pretty_print(xddot, std::cout, "xddot");

  // printf("CE:\n");
  // std::cout<<CE<<std::endl;
  // printf("ce0:\n");
  // std::cout<<ce0<<std::endl;

  // printf("CI:\n");
  // std::cout<<CI<<std::endl;
  // printf("ci0:\n");
  // std::cout<<ci0<<std::endl;
}

template<typename T>
bool WBIC<T>::_ValidateInputs(const WBIC_ExtraData<T> & data) const
{
  // 求解器对维度和非有限数值很敏感，因此在进入矩阵运算前集中拒绝异常输入。
  const Eigen::Index n = static_cast<Eigen::Index>(WB::num_qdot_);
  if (!WB::b_updatesetting_ || WB::A_.rows() != n || WB::A_.cols() != n ||
    WB::Ainv_.rows() != n || WB::Ainv_.cols() != n ||
    WB::cori_.size() != n || WB::grav_.size() != n ||
    !WB::A_.allFinite() || !WB::Ainv_.allFinite() ||
    !WB::cori_.allFinite() || !WB::grav_.allFinite() ||
    data._W_floating.size() != static_cast<Eigen::Index>(_dim_floating) ||
    !data._W_floating.allFinite() ||
    (data._W_floating.array() <= T(0)).any())
  {
    return false;
  }

  std::size_t reaction_force_dimension = 0;
  for (const ContactSpec<T> * contact : *_contact_list) {
    if (contact == nullptr) {return false;}
    const Eigen::Index dim = static_cast<Eigen::Index>(contact->getDim());
    DMat<T> jacobian;
    DVec<T> jacobian_dot_qdot;
    DMat<T> constraint;
    DVec<T> constraint_vector;
    contact->getContactJacobian(jacobian);
    contact->getJcDotQdot(jacobian_dot_qdot);
    contact->getRFConstraintMtx(constraint);
    contact->getRFConstraintVec(constraint_vector);
    if (jacobian.rows() != dim || jacobian.cols() != n ||
      jacobian_dot_qdot.size() != dim || constraint.cols() != dim ||
      constraint.rows() != constraint_vector.size() ||
      constraint.rows() != static_cast<Eigen::Index>(contact->getDimRFConstraint()) ||
      contact->getRFDesired().size() != dim || !jacobian.allFinite() ||
      !jacobian_dot_qdot.allFinite() || !constraint.allFinite() ||
      !constraint_vector.allFinite() || !contact->getRFDesired().allFinite())
    {
      return false;
    }
    reaction_force_dimension += contact->getDim();
  }
  if (data._W_rf.size() != static_cast<Eigen::Index>(reaction_force_dimension) ||
    !data._W_rf.allFinite() || (data._W_rf.array() <= T(0)).any())
  {
    return false;
  }

  for (const Task<T> * task : *_task_list) {
    if (task == nullptr) {return false;}
    DMat<T> jacobian;
    DVec<T> jacobian_dot_qdot;
    DVec<T> command;
    task->getTaskJacobian(jacobian);
    task->getTaskJacobianDotQdot(jacobian_dot_qdot);
    task->getCommand(command);
    if (jacobian.cols() != n || jacobian.rows() != jacobian_dot_qdot.size() ||
      jacobian.rows() != command.size() || !jacobian.allFinite() ||
      !jacobian_dot_qdot.allFinite() || !command.allFinite())
    {
      return false;
    }
  }
  return true;
}
/*完全没看懂*/
template<typename T>
void WBIC<T>::_SetEqualityConstraint(const DVec<T> & qddot)
{
  // 取整机动力学的浮动基座 6 行：这部分不能由关节力矩直接补偿。
  if (_dim_rf > 0) {
    /*取质量矩阵前 6 行、前 6 列：SvAEb
    其中 \(E_b\) 表示将 6 维浮动基座修正量嵌入完整广义加速度。*/
    _dyn_CE.block(0, 0, _dim_eq_cstr, _dim_floating) =
      WB::A_.block(0, 0, _dim_floating, _dim_floating);
    /*接触力修正对浮动基座动力学的影响为*/
    _dyn_CE.block(0, _dim_floating, _dim_eq_cstr, _dim_rf) =
      -WB::Sv_ * _Jc.transpose();
    _dyn_ce0 = -WB::Sv_ * (WB::A_ * qddot + WB::cori_ + WB::grav_ -
      _Jc.transpose() * _Fr_des);
  } else {
    _dyn_CE.block(0, 0, _dim_eq_cstr, _dim_floating) =
      WB::A_.block(0, 0, _dim_floating, _dim_floating);
    _dyn_ce0 = -WB::Sv_ * (WB::A_ * qddot + WB::cori_ + WB::grav_);
  }

  for (size_t i(0); i < _dim_eq_cstr; ++i) {
    for (size_t j(0); j < _dim_opt; ++j) {
      CE[j][i] = _dyn_CE(i, j);
    }
    ce0[i] = -_dyn_ce0[i];
  }
  // pretty_print(_dyn_CE, std::cout, "WBIC: CE");
  // pretty_print(_dyn_ce0, std::cout, "WBIC: ce0");
}

/*完全没看懂*/
template<typename T>
void WBIC<T>::_SetInEqualityConstraint()
{
  // 将每只支撑脚的摩擦锥与法向力边界转换成 QuadProg 使用的不等式格式。
  _dyn_CI.block(0, _dim_floating, _dim_Uf, _dim_rf) = _Uf;
  _dyn_ci0 = _Uf_ieq_vec - _Uf * _Fr_des;

  for (size_t i(0); i < _dim_Uf; ++i) {
    for (size_t j(0); j < _dim_opt; ++j) {
      CI[j][i] = _dyn_CI(i, j);
    }
    ci0[i] = -_dyn_ci0[i];
  }
  // pretty_print(_dyn_CI, std::cout, "WBIC: CI");
  // pretty_print(_dyn_ci0, std::cout, "WBIC: ci0");
}

template<typename T>
void WBIC<T>::_ContactBuilding()
{
  // 不同接触对象分别保存自己的矩阵；这里按块拼成一个统一优化问题。
  DMat<T> Uf;
  DVec<T> Uf_ieq_vec;
  // Initial
  DMat<T> Jc;//接触雅可比
  DVec<T> JcDotQdot;//接触点在广义速度方向上的加速度分量。
  size_t dim_accumul_rf, dim_accumul_uf;//期望接触力,摩擦锥矩阵
  (*_contact_list)[0]->getContactJacobian(Jc);
  (*_contact_list)[0]->getJcDotQdot(JcDotQdot);
  (*_contact_list)[0]->getRFConstraintMtx(Uf);
  (*_contact_list)[0]->getRFConstraintVec(Uf_ieq_vec);

  dim_accumul_rf = (*_contact_list)[0]->getDim();
  dim_accumul_uf = (*_contact_list)[0]->getDimRFConstraint();

  _Jc.block(0, 0, dim_accumul_rf, WB::num_qdot_) = Jc;
  _JcDotQdot.head(dim_accumul_rf) = JcDotQdot;
  _Uf.block(0, 0, dim_accumul_uf, dim_accumul_rf) = Uf;
  _Uf_ieq_vec.head(dim_accumul_uf) = Uf_ieq_vec;
  _Fr_des.head(dim_accumul_rf) = (*_contact_list)[0]->getRFDesired();

  size_t dim_new_rf, dim_new_uf;

  for (size_t i(1); i < (*_contact_list).size(); ++i) {
    (*_contact_list)[i]->getContactJacobian(Jc);
    (*_contact_list)[i]->getJcDotQdot(JcDotQdot);

    dim_new_rf = (*_contact_list)[i]->getDim();
    dim_new_uf = (*_contact_list)[i]->getDimRFConstraint();

    // 追加接触雅可比。
    _Jc.block(dim_accumul_rf, 0, dim_new_rf, WB::num_qdot_) = Jc;

    // 追加雅可比变化率项。
    _JcDotQdot.segment(dim_accumul_rf, dim_new_rf) = JcDotQdot;

    // Uf 采用块对角形式，每只脚的摩擦约束互不串扰。
    (*_contact_list)[i]->getRFConstraintMtx(Uf);
    _Uf.block(dim_accumul_uf, dim_accumul_rf, dim_new_uf, dim_new_rf) = Uf;

    // 追加不等式右端项。
    (*_contact_list)[i]->getRFConstraintVec(Uf_ieq_vec);
    _Uf_ieq_vec.segment(dim_accumul_uf, dim_new_uf) = Uf_ieq_vec;

    // 追加来自 MPC 或站立控制器的期望地面反力。
    _Fr_des.segment(dim_accumul_rf, dim_new_rf) =
      (*_contact_list)[i]->getRFDesired();
    dim_accumul_rf += dim_new_rf;
    dim_accumul_uf += dim_new_uf;
  }
}

template<typename T>
void WBIC<T>::_GetSolution(const DVec<T> & qddot, DVec<T> & cmd)
{
  // 由 M(q)qddot + C + G = S^T tau + Jc^T Fr 反解广义力。
  DVec<T> tot_tau;
  if (_dim_rf > 0) {
    _data->_Fr = DVec<T>(_dim_rf);
    // 优化量保存的是反力修正值，需要加回期望反力 Fr_des。
    for (size_t i(0); i < _dim_rf; ++i) {
      _data->_Fr[i] = z[i + _dim_floating] + _Fr_des[i];
    }
    tot_tau =
      WB::A_ * qddot + WB::cori_ + WB::grav_ - _Jc.transpose() * _data->_Fr;

  } else {
    tot_tau = WB::A_ * qddot + WB::cori_ + WB::grav_;
  }
  _data->_qddot = qddot;
  // 广义力前 6 项属于不可驱动基座，只取后 12 项作为关节力矩。
  cmd = tot_tau.tail(WB::num_act_joint_);

}

template<typename T>
void WBIC<T>::_SetCost()
{
  // 对角权重分别惩罚基座加速度修正和接触力修正；权重越大越不希望该量变化。
  size_t idx_offset(0);
  for (size_t i(0); i < _dim_floating; ++i) {
    G[i + idx_offset][i + idx_offset] = _data->_W_floating[i];
  }
  idx_offset += _dim_floating;
  for (size_t i(0); i < _dim_rf; ++i) {
    G[i + idx_offset][i + idx_offset] = _data->_W_rf[i];
  }
  // pretty_print(_data->_W_floating, std::cout, "W floating");
  // pretty_print(_data->_W_rf, std::cout, "W rf");
}

template<typename T>
void WBIC<T>::UpdateSetting(
  const DMat<T> & A, const DMat<T> & Ainv,
  const DVec<T> & cori, const DVec<T> & grav,
  void * extra_setting)
{
  // A、C、G 必须来自同一时刻、同一个模型状态，否则逆动力学不再一致。
  const Eigen::Index n = static_cast<Eigen::Index>(WB::num_qdot_);
  if (A.rows() != n || A.cols() != n || Ainv.rows() != n ||
    Ainv.cols() != n || cori.size() != n || grav.size() != n ||
    !A.allFinite() || !Ainv.allFinite() || !cori.allFinite() ||
    !grav.allFinite())
  {
    WB::b_updatesetting_ = false;
    throw std::invalid_argument("WBIC dynamics setting has invalid size or non-finite values");
  }
  WB::A_ = A;
  WB::Ainv_ = Ainv;
  WB::cori_ = cori;
  WB::grav_ = grav;
  WB::b_updatesetting_ = true;

  (void)extra_setting;
}

template<typename T>
void WBIC<T>::_SetOptimizationSize()
{
  // 接触腿数量会随步态变化，所以每个控制周期都按当前接触列表重建矩阵尺寸。
  _dim_rf = 0;
  _dim_Uf = 0;  // 摩擦与法向力不等式的总维数。
  for (size_t i(0); i < (*_contact_list).size(); ++i) {
    _dim_rf += (*_contact_list)[i]->getDim();
    _dim_Uf += (*_contact_list)[i]->getDimRFConstraint();
  }

  _dim_opt = _dim_floating + _dim_rf;
  _dim_eq_cstr = _dim_floating;

  // QuadProg 使用自己的数组类型保存目标、等式和不等式矩阵。
  G.resize(0., _dim_opt, _dim_opt);
  g0.resize(0., _dim_opt);
  CE.resize(0., _dim_opt, _dim_eq_cstr);
  ce0.resize(0., _dim_eq_cstr);

  // 同时保留 Eigen 矩阵，便于用块操作构造约束。
  _dyn_CE = DMat<T>::Zero(_dim_eq_cstr, _dim_opt);
  _dyn_ce0 = DVec<T>(_dim_eq_cstr);
  if (_dim_rf > 0) {
    CI.resize(0., _dim_opt, _dim_Uf);
    ci0.resize(0., _dim_Uf);
    _dyn_CI = DMat<T>::Zero(_dim_Uf, _dim_opt);
    _dyn_ci0 = DVec<T>(_dim_Uf);

    _Jc = DMat<T>(_dim_rf, WB::num_qdot_);
    _JcDotQdot = DVec<T>(_dim_rf);
    _Fr_des = DVec<T>(_dim_rf);

    _Uf = DMat<T>(_dim_Uf, _dim_rf);
    _Uf.setZero();
    _Uf_ieq_vec = DVec<T>(_dim_Uf);
  } else {
    CI.resize(0., _dim_opt, 0);
    ci0.resize(0., 0);
  }
}

template class WBIC<double>;
template class WBIC<float>;

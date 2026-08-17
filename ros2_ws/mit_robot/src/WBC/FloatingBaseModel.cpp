/**
 * @file FloatingBaseModel.cpp
 * @brief 刚体浮动基座模型和 Featherstone 动力学算法的实现。
 *
 * 本类存储 Featherstone 在《Rigid Body Dynamics Algorithms》中描述的
 * 运动学树（可在 MIT 内网下载：
 * https://www.springer.com/us/book/9780387743141）
 *
 * 树中为每个刚体额外包含一个“转子”刚体。该转子固定在父刚体上，
 * 并受传动比约束。本实现采用与 Jain 《Robot and Multibody Dynamics》
 * 第 12 章所述方法类似的技术，以高效地将转子纳入模型。请注意，
 * 该实现专门针对每个刚体只有一个旋转转子的情况。转子与对应刚体
 * 具有相同的关节类型，但其运动子空间会额外乘以传动比。与浮动基座
 * 关联的转子不起作用。
 */

#include <stdio.h>
#include <stdexcept>
#include <string>
#include <vector>

#include "WBC/FloatingBaseModel.h"
#include "Utilities/orientation_tools.h"
#include "Utilities/Utilities_print.h"

using namespace ori;
using namespace spatial;
using namespace std;

/*!
 * 在接触点施加单位测试力，返回该方向的接触惯量逆，并计算由此产生的 qdd
 * @param gc_index 接触点索引
 * @param force_ics_at_contact 用惯性坐标表示的单位测试力
 * @params dstate 输出参数，用于保存所得加速度
 * @return 1x1 接触惯量逆 J H^{-1} J^T
 */
template<typename T>
T FloatingBaseModel<T>::applyTestForce(
  const int gc_index,
  const Vec3<T> & force_ics_at_contact,
  DVec<T> & dstate_out)
{
  forwardKinematics();
  updateArticulatedBodies();
  updateForcePropagators();
  udpateQddEffects();

  size_t i_opsp = _gcParent.at(gc_index);
  size_t i = i_opsp;

  dstate_out = DVec<T>::Zero(_nDof);

  // 旋转到绝对坐标系
  /*_Xa[i]：获取数组 _Xa 的第 i 个元素。
    .template block<3, 3>(0, 0)：调用 block 方法，
    切出一个从 (0,0) 开始的 3x3 子矩阵。
    .transpose()：计算这个子矩阵的转置。*/
  Mat3<T> Rai = _Xa[i].template block<3, 3>(0, 0).transpose();
  Mat6<T> Xc = createSXform(Rai, _gcLocation.at(gc_index));

  // D 是扩展力传播子矩阵的一列（参见 Wensing，ICRA 2012）
  SVec<T> F = Xc.transpose().template rightCols<3>() * force_ics_at_contact;

  T LambdaInv = 0;
  T tmp = 0;

  // 从末端向基座遍历
  while (i > 5) {
    tmp = F.dot(_S[i]);
    LambdaInv += tmp * tmp / _d[i];
    dstate_out.tail(_nDof - 6) += _qdd_from_subqdd.col(i - 6) * tmp / _d[i];

    // 应用力传播子（参见 Pat 的 ICRA 2012 论文）
    // 本质上，由于关节可运动，前驱刚体只会感受到部分力。
    // Xup^T 像关节被锁定时那样向后传递力，而 ChiUp^T 则像关节自由时那样
    // 向后传递力
    F = _ChiUp[i].transpose() * F;
    i = _parents[i];
  }

  dstate_out.head(6) = _invIA5.solve(F);
  LambdaInv += F.dot(dstate_out.head(6));
  dstate_out.tail(_nDof - 6) += _qdd_from_base_accel * dstate_out.head(6);

  return LambdaInv;
}

/*!
 * 接触惯量算法的辅助函数
 * 计算由“subqdd”分量产生的 qdd
 * 如果你熟悉 Featherstone 的稀疏操作空间方法，
 * 或 Jain 的新息分解：
 * H = L * D * L^T
 * 这些 subqdd 分量表示中间空间
 * 也就是说，若 H^{-1} = L^{-T} * D^{-1} * L^{1}
 * 则这里所称的 subqdd = L^{-1} * tau
 * 这个解释不够清晰，需要用 LaTeX 表达。
 */
template<typename T>
void FloatingBaseModel<T>::udpateQddEffects()
{
  if (_qddEffectsUpToDate) {return;}
  updateForcePropagators();
  _qdd_from_base_accel.setZero();
  _qdd_from_subqdd.setZero();

  // 力传播子的遍历
  // 该循环近似等价于对 H 进行 Cholesky 分解，
  // 类似于 Featherstone 的稀疏操作空间算法
  // 这些计算用于将关节速度视为任务空间
  // 为此，F 计算力矩对运动学树下游各刚体的动力学影响
  //
  for (size_t i = 6; i < _nDof; i++) {
    _qdd_from_subqdd(i - 6, i - 6) = 1;
    SVec<T> F = (_ChiUp[i].transpose() - _Xup[i].transpose()) * _S[i];
    size_t j = _parents[i];
    while (j > 5) {
      _qdd_from_subqdd(i - 6, j - 6) = _S[j].dot(F);
      F = _ChiUp[j].transpose() * F;
      j = _parents[j];
    }
    _qdd_from_base_accel.row(i - 6) = F.transpose();
  }
  _qddEffectsUpToDate = true;
}

/*!
 * 接触惯量算法的辅助函数
 * 计算跨越各关节的力传播子
 */
template<typename T>
void FloatingBaseModel<T>::updateForcePropagators()
{
  if (_forcePropagatorsUpToDate) {return;}
  updateArticulatedBodies();
  for (size_t i = 6; i < _nDof; i++) {
    _ChiUp[i] = _Xup[i] - _S[i] * _Utot[i].transpose() / _d[i];
  }
  _forcePropagatorsUpToDate = true;
}

/*!
 * ABA 的辅助函数
 */
template<typename T>
void FloatingBaseModel<T>::updateArticulatedBodies()
{
  if (_articulatedBodiesUpToDate) {return;}

  forwardKinematics();

  _IA[5] = _Ibody[5].getMatrix();

  // 第 1 轮遍历：沿树向下
  for (size_t i = 6; i < _nDof; i++) {
    _IA[i] = _Ibody[i].getMatrix();  // 初始化
    Mat6<T> XJrot = jointXform(
      _jointTypes[i], _jointAxes[i],
      _state.q[i - 6] * _gearRatios[i]);
    _Xuprot[i] = XJrot * _Xrot[i];
    _Srot[i] = _S[i] * _gearRatios[i];
  }

  // Pat 神奇的最小约束原理（Gauss 也是！）
  for (size_t i = _nDof - 1; i >= 6; i--) {
    _U[i] = _IA[i] * _S[i];
    _Urot[i] = _Irot[i].getMatrix() * _Srot[i];
    _Utot[i] = _Xup[i].transpose() * _U[i] + _Xuprot[i].transpose() * _Urot[i];

    _d[i] = _Srot[i].transpose() * _Urot[i];
    _d[i] += _S[i].transpose() * _U[i] + _jointArmatures[i];

    // 关节惯量递归
    Mat6<T> Ia = _Xup[i].transpose() * _IA[i] * _Xup[i] +
      _Xuprot[i].transpose() * _Irot[i].getMatrix() * _Xuprot[i] -
      _Utot[i] * _Utot[i].transpose() / _d[i];
    _IA[_parents[i]] += Ia;
  }

  _invIA5.compute(_IA[5]);
  _articulatedBodiesUpToDate = true;
}

// 父节点、传动比、关节类型、Xtree、I、Xrot、Irot

/*!
 * 添加刚体时填充成员变量
 * @param count 浮动基座为 6，关节为 1
 */
template<typename T>
void FloatingBaseModel<T>::addDynamicsVars(int count)
{
  if (count != 1 && count != 6) {
    throw std::runtime_error(
            "addDynamicsVars must be called with count=1 (joint) or count=6 "
            "(base).\n");
  }

  Mat6<T> eye6 = Mat6<T>::Identity();
  SVec<T> zero6 = SVec<T>::Zero();
  Mat6<T> zero66 = Mat6<T>::Zero();

  SpatialInertia<T> zeroInertia(zero66);
  for (int i = 0; i < count; i++) {
    _v.push_back(zero6);
    _vrot.push_back(zero6);
    _a.push_back(zero6);
    _arot.push_back(zero6);
    _avp.push_back(zero6);
    _avprot.push_back(zero6);
    _c.push_back(zero6);
    _crot.push_back(zero6);
    _S.push_back(zero6);
    _Srot.push_back(zero6);
    _f.push_back(zero6);
    _frot.push_back(zero6);
    _fvp.push_back(zero6);
    _fvprot.push_back(zero6);
    _ag.push_back(zero6);
    _agrot.push_back(zero6);
    _IC.push_back(zeroInertia);
    _Xup.push_back(eye6);
    _Xuprot.push_back(eye6);
    _Xa.push_back(eye6);

    _ChiUp.push_back(eye6);
    _d.push_back(0.);
    _u.push_back(0.);
    _IA.push_back(eye6);

    _U.push_back(zero6);
    _Urot.push_back(zero6);
    _Utot.push_back(zero6);
    _pA.push_back(zero6);
    _pArot.push_back(zero6);
    _externalForces.push_back(zero6);
  }

  _J.push_back(D6Mat<T>::Zero(6, _nDof));
  _Jdqd.push_back(SVec<T>::Zero());

  resizeSystemMatricies();
}

/*!
 * 添加刚体时更新 H、C、Cqd、G 和 Js 的尺寸
 */
template<typename T>
void FloatingBaseModel<T>::resizeSystemMatricies()
{
  _H.setZero(_nDof, _nDof);
  _C.setZero(_nDof, _nDof);
  _Cqd.setZero(_nDof);
  _G.setZero(_nDof);
  for (size_t i = 0; i < _J.size(); i++) {
    _J[i].setZero(6, _nDof);
    _Jdqd[i].setZero();
  }

  for (size_t i = 0; i < _Jc.size(); i++) {
    _Jc[i].setZero(3, _nDof);
    _Jcdqd[i].setZero();
  }
  _qdd_from_subqdd.resize(_nDof - 6, _nDof - 6);
  _qdd_from_base_accel.resize(_nDof - 6, 6);
  _state.q = DVec<T>::Zero(_nDof - 6);
  _state.qd = DVec<T>::Zero(_nDof - 6);
}

/*!
 * 创建浮动刚体
 * @param inertia 浮动刚体的空间惯量
 */
template<typename T>
void FloatingBaseModel<T>::addBase(const SpatialInertia<T> & inertia)
{
  if (_nDof) {
    throw std::runtime_error("Cannot add base multiple times!\n");
  }

  Mat6<T> eye6 = Mat6<T>::Identity();
  Mat6<T> zero6 = Mat6<T>::Zero();
  SpatialInertia<T> zeroInertia(zero6);
  // 浮动基座有 6 个自由度

  _nDof = 6;
  for (size_t i = 0; i < 6; i++) {
    _parents.push_back(0);
    _gearRatios.push_back(0);
    _jointArmatures.push_back(0);
    _jointTypes.push_back(JointType::Nothing);  // 实际上无关紧要
    _jointAxes.push_back(CoordinateAxis::X);    // 实际上无关紧要
    _Xtree.push_back(eye6);
    _Ibody.push_back(zeroInertia);
    _Xrot.push_back(eye6);
    _Irot.push_back(zeroInertia);
    _bodyNames.push_back("N/A");
  }

  _jointTypes[5] = JointType::FloatingBase;
  _Ibody[5] = inertia;
  _gearRatios[5] = 1;
  _bodyNames[5] = "Floating Base";

  _state.bodyOrientation << T(1), T(0), T(0), T(0);
  _state.bodyPosition.setZero();
  _state.bodyVelocity.setZero();
  _dState.dBodyPosition.setZero();
  _dState.dBodyVelocity.setZero();

  addDynamicsVars(6);
}

/*!
 * 创建浮动刚体
 * @param mass 浮动刚体的质量
 * @param com  浮动刚体的质心
 * @param I    浮动刚体的转动惯量
 */
template<typename T>
void FloatingBaseModel<T>::addBase(
  T mass, const Vec3<T> & com,
  const Mat3<T> & I)
{
  SpatialInertia<T> IS(mass, com, I);
  addBase(IS);
}

/*!
 * 向模型中添加地面接触点
 * @param bodyID 接触点所属刚体的 ID
 * @param location 接触点的位置（用刚体坐标表示）
 * @param isFoot 该接触点是否属于足部
 * @return 地面接触点的 ID
 */
template<typename T>
int FloatingBaseModel<T>::addGroundContactPoint(
  int bodyID,
  const Vec3<T> & location,
  bool isFoot)
{
  if ((size_t)bodyID >= _nDof) {
    throw std::runtime_error(
            "addGroundContactPoint got invalid bodyID: " + std::to_string(bodyID) +
            " nDofs: " + std::to_string(_nDof) + "\n");
  }

  // std::cout << "pt-add: " << location.transpose() << "\n";
  _gcParent.push_back(bodyID);
  _gcLocation.push_back(location);

  Vec3<T> zero3 = Vec3<T>::Zero();

  _pGC.push_back(zero3);
  _vGC.push_back(zero3);

  D3Mat<T> J(3, _nDof);
  J.setZero();

  _Jc.push_back(J);
  _Jcdqd.push_back(zero3);
  //_compute_contact_info.push_back(false);
  _compute_contact_info.push_back(true);

  // 将足部添加到足部列表
  if (isFoot) {
    _footIndicesGC.push_back(_nGroundContact);
    _compute_contact_info[_nGroundContact] = true;
  }

  resizeSystemMatricies();
  return _nGroundContact++;
}

/*!
 * 将长方体的边界点添加到接触模型。假定长方体以刚体坐标系原点为中心，
 * 且各边与坐标轴对齐。
 */
template<typename T>
void FloatingBaseModel<T>::addGroundContactBoxPoints(
  int bodyId,
  const Vec3<T> & dims)
{
  addGroundContactPoint(bodyId, Vec3<T>(dims(0), dims(1), dims(2)) / 2);
  addGroundContactPoint(bodyId, Vec3<T>(-dims(0), dims(1), dims(2)) / 2);
  addGroundContactPoint(bodyId, Vec3<T>(dims(0), -dims(1), dims(2)) / 2);
  addGroundContactPoint(bodyId, Vec3<T>(-dims(0), -dims(1), dims(2)) / 2);

  //addGroundContactPoint(bodyId, Vec3<T>(dims(0), dims(1), 0.) / 2);
  //addGroundContactPoint(bodyId, Vec3<T>(-dims(0), dims(1), 0.) / 2);
  //addGroundContactPoint(bodyId, Vec3<T>(dims(0), -dims(1), 0.) / 2);
  //addGroundContactPoint(bodyId, Vec3<T>(-dims(0), -dims(1), 0.) / 2);

  addGroundContactPoint(bodyId, Vec3<T>(dims(0), dims(1), -dims(2)) / 2);
  addGroundContactPoint(bodyId, Vec3<T>(-dims(0), dims(1), -dims(2)) / 2);
  addGroundContactPoint(bodyId, Vec3<T>(dims(0), -dims(1), -dims(2)) / 2);
  addGroundContactPoint(bodyId, Vec3<T>(-dims(0), -dims(1), -dims(2)) / 2);
}

/*!
 * 添加刚体
 * @param inertia 刚体的惯量
 * @param rotorInertia 与该刚体相连转子的惯量
 * @param gearRatio 刚体与转子之间的传动比
 * @param parent 父刚体，同时也假定为转子所连接的刚体
 * @param jointType 关节类型（移动关节或转动关节）
 * @param jointAxis 父刚体坐标系中的关节轴（X、Y、Z）
 * @param Xtree  从父刚体到该刚体的坐标变换
 * @param Xrot  从父刚体到该刚体转子的坐标变换
 * @return 刚体的 ID（可用作父刚体 ID）
 */
template<typename T>
int FloatingBaseModel<T>::addBody(
  const SpatialInertia<T> & inertia,
  const SpatialInertia<T> & rotorInertia,
  T gearRatio, int parent, JointType jointType,
  CoordinateAxis jointAxis,
  const Mat6<T> & Xtree, const Mat6<T> & Xrot)
{
  if ((size_t)parent >= _nDof) {
    throw std::runtime_error(
            "addBody got invalid parent: " + std::to_string(parent) +
            " nDofs: " + std::to_string(_nDof) + "\n");
  }

  _parents.push_back(parent);
  _gearRatios.push_back(gearRatio);
  _jointArmatures.push_back(T(0));
  _jointTypes.push_back(jointType);
  _jointAxes.push_back(jointAxis);
  _Xtree.push_back(Xtree);
  _Xrot.push_back(Xrot);
  _Ibody.push_back(inertia);
  _Irot.push_back(rotorInertia);
  _bodyNames.push_back("N/A");
  _nDof++;

  addDynamicsVars(1);

  return static_cast<int>(_nDof - 1);
}

template<typename T>
int FloatingBaseModel<T>::addBody(
  const SpatialInertia<T> & inertia,
  T jointArmature, int parent,
  JointType jointType,
  CoordinateAxis jointAxis,
  const Mat6<T> & Xtree,
  const std::string & bodyName)
{
  if (!std::isfinite(static_cast<double>(jointArmature)) || jointArmature < T(0)) {
    throw std::invalid_argument("joint armature must be finite and non-negative");
  }
  const Mat6<T> identity = Mat6<T>::Identity();
  const Mat6<T> zeroMatrix = Mat6<T>::Zero();
  const SpatialInertia<T> zeroInertia(zeroMatrix);
  const int bodyId = addBody(
    inertia, zeroInertia, T(0), parent, jointType,
    jointAxis, Xtree, identity);
  _jointArmatures.at(static_cast<size_t>(bodyId)) = jointArmature;
  _bodyNames.at(static_cast<size_t>(bodyId)) = bodyName.empty() ? "N/A" : bodyName;
  return bodyId;
}

/*!
 * 添加刚体
 * @param inertia 刚体的惯量
 * @param rotorInertia 与该刚体相连转子的惯量
 * @param gearRatio 刚体与转子之间的传动比
 * @param parent 父刚体，同时也假定为转子所连接的刚体
 * @param jointType 关节类型（移动关节或转动关节）
 * @param jointAxis 父刚体坐标系中的关节轴（X、Y、Z）
 * @param Xtree  从父刚体到该刚体的坐标变换
 * @param Xrot  从父刚体到该刚体转子的坐标变换
 * @return 刚体的 ID（可用作父刚体 ID）
 */
template<typename T>
int FloatingBaseModel<T>::addBody(
  const MassProperties<T> & inertia,
  const MassProperties<T> & rotorInertia,
  T gearRatio, int parent, JointType jointType,
  CoordinateAxis jointAxis,
  const Mat6<T> & Xtree, const Mat6<T> & Xrot)
{
  return addBody(
    SpatialInertia<T>(inertia), SpatialInertia<T>(rotorInertia),
    gearRatio, parent, jointType, jointAxis, Xtree, Xrot);
}

template<typename T>
void FloatingBaseModel<T>::check()
{
  if (_nDof < 6 || _nDof != _parents.size() || _nDof != _gearRatios.size() ||
    _nDof != _jointArmatures.size() || _nDof != _jointTypes.size() ||
    _nDof != _jointAxes.size() || _nDof != _Xtree.size() ||
    _nDof != _Xrot.size() || _nDof != _Ibody.size() ||
    _nDof != _Irot.size() || _nDof != _bodyNames.size())
  {
    throw std::runtime_error("floating-base model arrays have inconsistent sizes");
  }
  for (size_t i = 6; i < _nDof; ++i) {
    if (_parents[i] < 5 || static_cast<size_t>(_parents[i]) >= i) {
      throw std::runtime_error("floating-base model has an invalid parent ordering");
    }
  }
}

/*!
 * 计算非转子刚体的总质量。
 * @return
 */
template<typename T>
T FloatingBaseModel<T>::totalNonRotorMass() const
{
  T totalMass = 0;
  for (size_t i = 0; i < _nDof; i++) {
    totalMass += _Ibody[i].getMass();
  }
  return totalMass;
}

/*!
 * 计算所有转子的总质量
 * @return
 */
template<typename T>
T FloatingBaseModel<T>::totalRotorMass() const
{
  T totalMass = 0;
  for (size_t i = 0; i < _nDof; i++) {
    totalMass += _Irot[i].getMass();
  }
  return totalMass;
}

/*!
 * 对所有刚体进行正向运动学计算。计算 _Xup（从树的上游坐标系到当前坐标系）
 * 和 _Xa（从绝对坐标系到当前坐标系），同时计算 _S（运动子空间）、
 * _v（连杆坐标系中的空间速度）和 _c（连杆坐标系中的科里奥利加速度）
 */
template<typename T>
void FloatingBaseModel<T>::forwardKinematics()
{
  if (_kinematicsUpToDate) {return;}

  // 计算关节变换
  _Xup[5] = createSXform(
    quaternionToRotationMatrix(_state.bodyOrientation),
    _state.bodyPosition);
  _v[5] = _state.bodyVelocity;
  for (size_t i = 6; i < _nDof; i++) {
    // 父连杆坐标系下的位置 = 关节转动量 × 固定安装位置。
    //这就是你在前一个问题里看到的 R*p_A + r，只不过这里是 6 维齐次变换矩阵的乘法。
    Mat6<T> XJ = jointXform(_jointTypes[i], _jointAxes[i], _state.q[i - 6]);
    _Xup[i] = XJ * _Xtree[i];
    _S[i] = jointMotionSubspace<T>(_jointTypes[i], _jointAxes[i]);
    SVec<T> vJ = _S[i] * _state.qd[i - 6];
    // 刚体 i 的绝对速度 =（父连杆速度通过关节传递过来）+（关节自己转动产生的速度）。
    _v[i] = _Xup[i] * _v[_parents[i]] + vJ;

    // 对转子进行相同计算
    Mat6<T> XJrot = jointXform(
      _jointTypes[i], _jointAxes[i],
      _state.q[i - 6] * _gearRatios[i]);
    _Srot[i] = _S[i] * _gearRatios[i];
    SVec<T> vJrot = _Srot[i] * _state.qd[i - 6];
    _Xuprot[i] = XJrot * _Xrot[i];
    _vrot[i] = _Xuprot[i] * _v[_parents[i]] + vJrot;

    // 因旋转坐标系产生的附加加速度,科里奥利加速度
    _c[i] = motionCrossProduct(_v[i], vJ);
    _crot[i] = motionCrossProduct(_vrot[i], vJrot);
  }

  // 计算相对于绝对坐标系的变换
  /*世界坐标 = 当前连杆相对父连杆 × 父连杆相对祖父连杆 × ... × 基座相对世界
    数学上就是不断地套用 p_B = R * p_A + r，只不过这里用矩阵乘法把旋转和平
    移打包在了一起（齐次变换矩阵）。*/
  for (size_t i = 5; i < _nDof; i++) {
    if (_parents[i] == 0) {
      _Xa[i] = _Xup[i];  // 浮动基座
    } else {
      _Xa[i] = _Xup[i] * _Xa[_parents[i]];
    }
  }

  // 地面接触点
  // TODO：最终会对同一个 Xa 求逆多次（例如刚体上的 8 个点），
  // 这样的效率不太高。
  for (size_t j = 0; j < _nGroundContact; j++) {
    if (!_compute_contact_info[j]) {continue;}
    size_t i = _gcParent.at(j);
    Mat6<T> Xai = invertSXform(_Xa[i]);      // 求逆：从“连杆坐标”到“世界坐标”
    SVec<T> vSpatial = Xai * _v[i];          // 把速度转换到世界坐标系

    // 足部在世界坐标系中的位置
    _pGC.at(j) = sXFormPoint(Xai, _gcLocation.at(j));
    _vGC.at(j) = spatialToLinearVelocity(vSpatial, _pGC.at(j));
  }
  _kinematicsUpToDate = true;
}

// template <typename T>
// void FloatingBaseModel<T>::getPositionVelocity(
// const int & link_idx, const Vec3<T> & local_pos,
// Vec3<T> & link_pos, Vec3<T> & link_vel) const {

// Mat6<T> Xai = invertSXform(_Xa[link_idx]); // 从连杆坐标系到绝对坐标系
// link_pos = sXFormPoint(Xai, local_pos);
// link_vel = spatialToLinearVelocity(Xai*_v[link_idx], link_pos);
//}

/*!
 * 计算各接触点速度的接触雅可比矩阵（3xn 矩阵），
 * 速度用绝对坐标表示
 */
template<typename T>
void FloatingBaseModel<T>::contactJacobians()
{
  forwardKinematics(); // 算出所有连杆的位置和速度
  biasAccelerations(); // 算出重力、科里奥利等产生的偏置加速度

  for (size_t k = 0; k < _nGroundContact; k++) {
    _Jc[k].setZero();
    _Jcdqd[k].setZero();

    // 如果不关心该接触点，则跳过
    if (!_compute_contact_info[k]) {continue;}

    size_t i = _gcParent.at(k);

    // 旋转到绝对坐标系
    /*这里有三个关键点：
    _Xa[i]：脚掌在世界坐标系下的位姿（就是你之前理解的 R 和 r）
    Rai = R^T：取旋转矩阵的转置。因为我们要把世界坐标系下的速度
    投影到脚掌的局部坐标系中。为什么？Xc：从"连杆坐标系"到"接触点坐
    标系"的变换。把脚掌上那个具体接触点（比如脚后跟）的位置和朝向考虑进去*/
    Mat3<T> Rai = _Xa[i].template block<3, 3>(0, 0).transpose();
    Mat6<T> Xc = createSXform(Rai, _gcLocation.at(k));

    // 偏置加速度
    SVec<T> ac = Xc * _avp[i];  // 接触点的偏置加速度
    SVec<T> vc = Xc * _v[i];    // 接触点的速度

    // 校正为经典加速度,把 6 维空间加速度（角加速度 + 线加速度）转换成纯线加速度
    _Jcdqd[k] = spatialToLinearAcceleration(ac, vc);

    // 对应世界坐标系中线速度的行
    D3Mat<T> Xout = Xc.template bottomRows<3>();

    // 从末端向基座遍历
    while (i > 5) {
      _Jc[k].col(i) = Xout * _S[i];
      Xout = Xout * _Xup[i];
      i = _parents[i];
    }
    _Jc[k].template leftCols<6>() = Xout;
  }
}

/*!
 * （辅助函数）计算各连杆和转子的速度积加速度
 * _avp 和 _avprot
 */
template<typename T>
void FloatingBaseModel<T>::biasAccelerations()
{
  if (_biasAccelerationsUpToDate) {return;}
  forwardKinematics();
  // 基座的速度积加速度
  _avp[5] << 0, 0, 0, 0, 0, 0;

  // 从基座向末端遍历
  for (size_t i = 6; i < _nDof; i++) {
    // 运动学向外传播
    _avp[i] = _Xup[i] * _avp[_parents[i]] + _c[i];
    _avprot[i] = _Xuprot[i] * _avp[_parents[i]] + _crot[i];
  }
  _biasAccelerationsUpToDate = true;
}

/*!
 * 计算逆动力学中的广义重力（G）
 * @return G（_nDof x 1 向量）
 */
template<typename T>
DVec<T> FloatingBaseModel<T>::generalizedGravityForce()
{
  compositeInertias();

  SVec<T> aGravity;
  aGravity << 0, 0, 0, _gravity[0], _gravity[1], _gravity[2];
  _ag[5] = _Xup[5] * aGravity;

  // 重力补偿力等于产生与重力方向相反加速度所需的力
  _G.template topRows<6>() = -_IC[5].getMatrix() * _ag[5];
  for (size_t i = 6; i < _nDof; i++) {
    _ag[i] = _Xup[i] * _ag[_parents[i]];
    _agrot[i] = _Xuprot[i] * _ag[_parents[i]];

    // 刚体和转子
    _G[i] = -_S[i].dot(_IC[i].getMatrix() * _ag[i]) -
      _Srot[i].dot(_Irot[i].getMatrix() * _agrot[i]);
  }
  return _G;
}

/*!
 * 计算逆动力学中的广义科里奥利力（Cqd）
 * @return Cqd（_nDof x 1 向量）
 */
template<typename T>
DVec<T> FloatingBaseModel<T>::generalizedCoriolisForce()
{
  biasAccelerations();

  // 浮动基座上的力
  Mat6<T> Ifb = _Ibody[5].getMatrix();
  SVec<T> hfb = Ifb * _v[5];
  _fvp[5] = Ifb * _avp[5] + forceCrossProduct(_v[5], hfb);

  for (size_t i = 6; i < _nDof; i++) {
    // 刚体 i 上的力
    Mat6<T> Ii = _Ibody[i].getMatrix();
    SVec<T> hi = Ii * _v[i];
    _fvp[i] = Ii * _avp[i] + forceCrossProduct(_v[i], hi);

    // 转子 i 上的力
    Mat6<T> Ir = _Irot[i].getMatrix();
    SVec<T> hr = Ir * _vrot[i];
    _fvprot[i] = Ir * _avprot[i] + forceCrossProduct(_vrot[i], hr);
  }

  for (size_t i = _nDof - 1; i > 5; i--) {
    // 提取沿关节方向的力
    _Cqd[i] = _S[i].dot(_fvp[i]) + _Srot[i].dot(_fvprot[i]);

    // 沿树向下传播力
    _fvp[_parents[i]] += _Xup[i].transpose() * _fvp[i];
    _fvp[_parents[i]] += _Xuprot[i].transpose() * _fvprot[i];
  }

  // 浮动基座上的力
  _Cqd.template topRows<6>() = _fvp[5];
  return _Cqd;
}

template<typename T>
Mat3<T> FloatingBaseModel<T>::getOrientation(int link_idx)
{
  forwardKinematics();
  Mat3<T> Rai = _Xa[link_idx].template block<3, 3>(0, 0);
  Rai.transposeInPlace();
  return Rai;
}


template<typename T>
Vec3<T> FloatingBaseModel<T>::getPosition(const int link_idx)
{
  forwardKinematics();
  Mat6<T> Xai = invertSXform(_Xa[link_idx]); // 从连杆坐标系到绝对坐标系
  Vec3<T> link_pos = sXFormPoint(Xai, Vec3<T>::Zero());
  return link_pos;
}

template<typename T>
Vec3<T> FloatingBaseModel<T>::getPosition(const int link_idx, const Vec3<T> & local_pos)
{
  forwardKinematics();
  Mat6<T> Xai = invertSXform(_Xa[link_idx]); // 从连杆坐标系到绝对坐标系
  Vec3<T> link_pos = sXFormPoint(Xai, local_pos);
  return link_pos;
}

template<typename T>
Vec3<T> FloatingBaseModel<T>::getLinearAcceleration(
  const int link_idx,
  const Vec3<T> & point)
{
  forwardAccelerationKinematics();
  Mat3<T> R = getOrientation(link_idx);
  return R * spatialToLinearAcceleration(_a[link_idx], _v[link_idx], point);
}

template<typename T>
Vec3<T> FloatingBaseModel<T>::getLinearAcceleration(const int link_idx)
{
  forwardAccelerationKinematics();
  Mat3<T> R = getOrientation(link_idx);
  return R * spatialToLinearAcceleration(_a[link_idx], _v[link_idx], Vec3<T>::Zero());
}


template<typename T>
Vec3<T> FloatingBaseModel<T>::getLinearVelocity(
  const int link_idx,
  const Vec3<T> & point)
{
  forwardKinematics();
  Mat3<T> Rai = getOrientation(link_idx);
  return Rai * spatialToLinearVelocity(_v[link_idx], point);
}

template<typename T>
Vec3<T> FloatingBaseModel<T>::getLinearVelocity(const int link_idx)
{
  forwardKinematics();
  Mat3<T> Rai = getOrientation(link_idx);
  return Rai * spatialToLinearVelocity(_v[link_idx], Vec3<T>::Zero());
}


template<typename T>
Vec3<T> FloatingBaseModel<T>::getAngularVelocity(const int link_idx)
{
  forwardKinematics();
  Mat3<T> Rai = getOrientation(link_idx);
  // Vec3<T> v3 =
  return Rai * _v[link_idx].template head<3>();
}

template<typename T>
Vec3<T> FloatingBaseModel<T>::getAngularAcceleration(const int link_idx)
{
  forwardAccelerationKinematics();
  Mat3<T> Rai = getOrientation(link_idx);
  return Rai * _a[link_idx].template head<3>();
}

/*!
 * （辅助函数）计算各子树的复合刚体惯量。
 * _IC[i] 包含刚体 i，以及刚体 i 所有后继刚体和转子的惯量。
 * （重要：_IC[i] 不包含转子 i）
 */
template<typename T>
void FloatingBaseModel<T>::compositeInertias()
{
  if (_compositeInertiasUpToDate) {return;}

  forwardKinematics();
  // 初始化
  /*_Ibody[i]：连杆 i 本身 的惯量（6x6 空间惯性矩阵）
    先把每个连杆的复合惯量初始化为它自己的惯量（相当于"
    先不管子孙，只看自己"）*/
  for (size_t i = 5; i < _nDof; i++) {
    _IC[i].setMatrix(_Ibody[i].getMatrix());
  }

  // 反向循环
  for (size_t i = _nDof - 1; i > 5; i--) {
    // 沿树向下传播惯量
    _IC[_parents[i]].addMatrix(
      _Xup[i].transpose() * _IC[i].getMatrix() *
      _Xup[i]);
    _IC[_parents[i]].addMatrix(
      _Xuprot[i].transpose() * _Irot[i].getMatrix() *
      _Xuprot[i]);
  }
  _compositeInertiasUpToDate = true;
}

/*!
 * 计算逆动力学形式中的质量矩阵（H）
 * @return H（_nDof x _nDof 矩阵）
 */
template<typename T>
DMat<T> FloatingBaseModel<T>::massMatrix()
{
  compositeInertias();
  _H.setZero();

  // 左上角是整个系统的锁定惯量
  _H.template topLeftCorner<6, 6>() = _IC[5].getMatrix();

  for (size_t j = 6; j < _nDof; j++) {
    // f = 产生单位 qdd_j 所需的空间力
    SVec<T> f = _IC[j].getMatrix() * _S[j];
    SVec<T> frot = _Irot[j].getMatrix() * _Srot[j];

    _H(j, j) = _S[j].dot(f) + _Srot[j].dot(frot) + _jointArmatures[j];

    // 沿树向下传播
    f = _Xup[j].transpose() * f + _Xuprot[j].transpose() * frot;
    size_t i = _parents[j];
    while (i > 5) {
      // 此处 f 用坐标系 {i} 表示
      _H(i, j) = _S[i].dot(f);
      _H(j, i) = _H(i, j);

      // 沿树向下传播
      f = _Xup[i].transpose() * f;
      i = _parents[i];
    }

    // 浮动基座上的力
    _H.template block<6, 1>(0, j) = f;
    _H.template block<1, 6>(j, 0) = f.adjoint();
  }
  return _H;
}

template<typename T>
void FloatingBaseModel<T>::forwardAccelerationKinematics()
{
  if (_accelerationsUpToDate) {
    return;
  }

  forwardKinematics();
  biasAccelerations();

  // 使用模型信息初始化重力
  SVec<T> aGravity = SVec<T>::Zero();
  aGravity.template tail<3>() = _gravity;

  // 浮动基座的空间加速度
  _a[5] = -_Xup[5] * aGravity + _dState.dBodyVelocity;

  // 遍历各关节
  for (size_t i = 6; i < _nDof; i++) {
    // 空间加速度
    _a[i] = _Xup[i] * _a[_parents[i]] + _S[i] * _dState.qdd[i - 6] + _c[i];
    _arot[i] =
      _Xuprot[i] * _a[_parents[i]] + _Srot[i] * _dState.qdd[i - 6] + _crot[i];
  }
  _accelerationsUpToDate = true;
}

/*!
 * 计算系统的逆动力学
 * @return 一个 _nDof x 1 向量。前六个元素给出基座上的外部力旋量，
 *         其余元素给出关节力矩
 */
template<typename T>
DVec<T> FloatingBaseModel<T>::inverseDynamics(
  const FBModelStateDerivative<T> & dState)
{
  setDState(dState);
  forwardAccelerationKinematics();

  // 浮动基座的空间力
  SVec<T> hb = _Ibody[5].getMatrix() * _v[5];
  _f[5] = _Ibody[5].getMatrix() * _a[5] + forceCrossProduct(_v[5], hb);

  // 遍历各关节
  for (size_t i = 6; i < _nDof; i++) {
    // 空间动量
    SVec<T> hi = _Ibody[i].getMatrix() * _v[i];
    SVec<T> hr = _Irot[i].getMatrix() * _vrot[i];

    // 空间力
    _f[i] = _Ibody[i].getMatrix() * _a[i] + forceCrossProduct(_v[i], hi);
    _frot[i] =
      _Irot[i].getMatrix() * _arot[i] + forceCrossProduct(_vrot[i], hr);
  }

  DVec<T> genForce(_nDof);
  for (size_t i = _nDof - 1; i > 5; i--) {
    // 提取沿关节方向的力分量
    genForce[i] = _S[i].dot(_f[i]) + _Srot[i].dot(_frot[i]) +
      _jointArmatures[i] * dState.qdd[i - 6];

    // 沿树向下传播
    _f[_parents[i]] += _Xup[i].transpose() * _f[i];
    _f[_parents[i]] += _Xuprot[i].transpose() * _frot[i];
  }
  genForce.template head<6>() = _f[5];
  return genForce;
}

template<typename T>
void FloatingBaseModel<T>::runABA(
  const DVec<T> & tau,
  FBModelStateDerivative<T> & dstate)
{
  if (tau.size() != static_cast<Eigen::Index>(_nDof - 6) || !tau.allFinite()) {
    throw std::invalid_argument("ABA torque vector has invalid size or non-finite values");
  }
  forwardKinematics();
  updateArticulatedBodies();

  // 创建重力的空间向量
  SVec<T> aGravity;
  aGravity << 0, 0, 0, _gravity[0], _gravity[1], _gravity[2];

  // 浮动基座的关节惯量
  SVec<T> ivProduct = _Ibody[5].getMatrix() * _v[5];
  _pA[5] = forceCrossProduct(_v[5], ivProduct);

  // 第 1 轮遍历：沿树向下
  for (size_t i = 6; i < _nDof; i++) {
    ivProduct = _Ibody[i].getMatrix() * _v[i];
    _pA[i] = forceCrossProduct(_v[i], ivProduct);

    // 对转子进行相同计算
    SVec<T> vJrot = _Srot[i] * _state.qd[i - 6];
    _vrot[i] = _Xuprot[i] * _v[_parents[i]] + vJrot;
    _crot[i] = motionCrossProduct(_vrot[i], vJrot);
    ivProduct = _Irot[i].getMatrix() * _vrot[i];
    _pArot[i] = forceCrossProduct(_vrot[i], ivProduct);
  }

  // 根据外力调整 pA
  for (size_t i = 5; i < _nDof; i++) {
    // TODO：添加 if 语句（当力为零时避免这些计算）
    Mat3<T> R = rotationFromSXform(_Xa[i]);
    Vec3<T> p = translationFromSXform(_Xa[i]);
    Mat6<T> iX = createSXform(R.transpose(), -R * p);
    _pA[i] = _pA[i] - iX.transpose() * _externalForces.at(i);
  }

  // Pat 神奇的最小约束原理
  for (size_t i = _nDof - 1; i >= 6; i--) {
    _u[i] = tau[i - 6] - _S[i].transpose() * _pA[i] -
      _Srot[i].transpose() * _pArot[i] - _U[i].transpose() * _c[i] -
      _Urot[i].transpose() * _crot[i];

    // 关节惯量递归
    SVec<T> pa =
      _Xup[i].transpose() * (_pA[i] + _IA[i] * _c[i]) +
      _Xuprot[i].transpose() * (_pArot[i] + _Irot[i].getMatrix() * _crot[i]) +
      _Utot[i] * _u[i] / _d[i];
    _pA[_parents[i]] += pa;
  }

  // 计入重力并计算浮动基座的加速度
  SVec<T> a0 = -aGravity;
  SVec<T> ub = -_pA[5];
  _a[5] = _Xup[5] * a0;
  SVec<T> afb = _invIA5.solve(ub - _IA[5].transpose() * _a[5]);
  _a[5] += afb;

  // 关节加速度
  dstate.qdd = DVec<T>(_nDof - 6);
  for (size_t i = 6; i < _nDof; i++) {
    dstate.qdd[i - 6] =
      (_u[i] - _Utot[i].transpose() * _a[_parents[i]]) / _d[i];
    _a[i] = _Xup[i] * _a[_parents[i]] + _S[i] * dstate.qdd[i - 6] + _c[i];
  }

  // 输出
  RotMat<T> Rup = rotationFromSXform(_Xup[5]);
  dstate.dBodyPosition =
    Rup.transpose() * _state.bodyVelocity.template block<3, 1>(3, 0);
  dstate.dBodyVelocity = afb;
  // qdd 已在上面的 for 循环中设置
}

/*!
 * 在接触点施加单位测试力，返回该方向的接触惯量逆，并计算由此产生的 qdd
 * @param gc_index 接触点索引
 * @param force_ics_at_contact 单位测试力
 * @params dstate 输出参数，用于保存所得加速度
 * @return 1x1 接触惯量逆 J H^{-1} J^T
 */
template<typename T>
T FloatingBaseModel<T>::applyTestForce(
  const int gc_index,
  const Vec3<T> & force_ics_at_contact,
  FBModelStateDerivative<T> & dstate_out)
{
  forwardKinematics();
  updateArticulatedBodies();
  updateForcePropagators();
  udpateQddEffects();

  size_t i_opsp = _gcParent.at(gc_index);
  size_t i = i_opsp;

  dstate_out.qdd.setZero();

  // 旋转到绝对坐标系
  Mat3<T> Rai = _Xa[i].template block<3, 3>(0, 0).transpose();
  Mat6<T> Xc = createSXform(Rai, _gcLocation.at(gc_index));

  // D 是扩展力传播子矩阵的一列（参见 Wensing，ICRA 2012）
  SVec<T> F = Xc.transpose().template rightCols<3>() * force_ics_at_contact;

  double LambdaInv = 0;
  double tmp = 0;

  // 从末端向基座遍历
  while (i > 5) {
    tmp = F.dot(_S[i]);
    LambdaInv += tmp * tmp / _d[i];
    dstate_out.qdd += _qdd_from_subqdd.col(i - 6) * tmp / _d[i];

    // 应用力传播子（参见 Pat 的 ICRA 2012 论文）
    // 本质上，由于关节可运动，前驱刚体只会感受到部分力。
    // Xup^T 像关节被锁定时那样向后传递力，而 ChiUp^T 则像关节自由时那样
    // 向后传递力
    F = _ChiUp[i].transpose() * F;
    i = _parents[i];
  }

  // TODO：在 updateArticulatedBodies 中只执行一次 QR 分解
  dstate_out.dBodyVelocity = _invIA5.solve(F);
  LambdaInv += F.dot(dstate_out.dBodyVelocity);
  dstate_out.qdd += _qdd_from_base_accel * dstate_out.dBodyVelocity;

  return LambdaInv;
}

/*!
 * 计算接触惯量矩阵的逆（mxm）
 * @param force_ics_at_contact (3x1)
 *        例如，若需要 z_ics 方向上的笛卡尔接触惯量逆，
 *             force_ics_at_contact = [0 0 1]^T
 * @return 1x1 接触惯量逆 J H^{-1} J^T
 */
template<typename T>
T FloatingBaseModel<T>::invContactInertia(
  const int gc_index,
  const Vec3<T> & force_ics_at_contact)
{
  forwardKinematics();
  updateArticulatedBodies();
  updateForcePropagators();

  size_t i_opsp = _gcParent.at(gc_index);
  size_t i = i_opsp;

  // 旋转到绝对坐标系
  Mat3<T> Rai = _Xa[i].template block<3, 3>(0, 0).transpose();
  Mat6<T> Xc = createSXform(Rai, _gcLocation.at(gc_index));

  // D 是扩展力传播子矩阵的一列（参见 Wensing，ICRA 2012）
  SVec<T> F = Xc.transpose().template rightCols<3>() * force_ics_at_contact;

  double LambdaInv = 0;
  double tmp = 0;

  // 从末端向基座遍历
  while (i > 5) {
    tmp = F.dot(_S[i]);
    LambdaInv += tmp * tmp / _d[i];

    // 应用力传播子（参见 Pat 的 ICRA 2012 论文）
    // 本质上，由于关节可运动，前驱刚体只会感受到部分力。
    // Xup^T 像关节被锁定时那样向后传递力，而 ChiUp^T 则像关节自由时那样
    // 向后传递力
    F = _ChiUp[i].transpose() * F;
    i = _parents[i];
  }
  LambdaInv += F.dot(_invIA5.solve(F));
  return LambdaInv;
}

/*!
 * 计算接触惯量矩阵的逆（mxm）
 * @param force_directions (6xm)，每一列表示一个关心的方向
 *        col = [惯性坐标系中的力矩, 惯性坐标系中的力]
 *        例如，若需要笛卡尔接触惯量逆，
 *             force_directions = [ 0_{3x3} I_{3x3}]^T
 *        若只需要某一方向上的笛卡尔接触惯量逆，请使用重载版本。
 * @return mxm 接触惯量逆 J H^{-1} J^T
 */
template<typename T>
DMat<T> FloatingBaseModel<T>::invContactInertia(
  const int gc_index, const D6Mat<T> & force_directions)
{
  forwardKinematics();
  updateArticulatedBodies();
  updateForcePropagators();

  size_t i_opsp = _gcParent.at(gc_index);
  size_t i = i_opsp;

  // 旋转到绝对坐标系
  Mat3<T> Rai = _Xa[i].template block<3, 3>(0, 0).transpose();
  Mat6<T> Xc = createSXform(Rai, _gcLocation.at(gc_index));

  // D 是扩展力传播子矩阵的一个子块（参见 Wensing，ICRA 2012）
  D6Mat<T> D = Xc.transpose() * force_directions;

  size_t m = force_directions.cols();

  DMat<T> LambdaInv = DMat<T>::Zero(m, m);
  DVec<T> tmp = DVec<T>::Zero(m);

  // 从末端向基座遍历
  while (i > 5) {
    tmp = D.transpose() * _S[i];
    LambdaInv += tmp * tmp.transpose() / _d[i];

    // 应用力传播子（参见 Pat 的 ICRA 2012 论文）
    // 本质上，由于关节可运动，前驱刚体只会感受到部分力。
    // Xup^T 像关节被锁定时那样向后传递力，而 ChiUp^T 则像关节自由时那样
    // 向后传递力
    D = _ChiUp[i].transpose() * D;
    i = _parents[i];
  }

  // TODO：在 updateArticulatedBodies 中只执行一次 QR 分解
  LambdaInv += D.transpose() * _invIA5.solve(D);

  return LambdaInv;
}

template class FloatingBaseModel<double>;
template class FloatingBaseModel<float>;

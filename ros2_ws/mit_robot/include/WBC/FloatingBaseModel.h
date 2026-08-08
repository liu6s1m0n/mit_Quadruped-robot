/*! @file FloatingBaseModel.h
 *  @brief 刚体浮动基座模型的数据结构与接口
 *
 * 本类存储 Featherstone 在《Rigid Body Dynamics Algorithms》中描述的
 * 运动学树（可在 MIT 内网下载：
 * https://www.springer.com/us/book/9780387743141）。
 *
 * 树中为每个刚体额外包含一个“转子”刚体。该转子固定在父刚体上，
 * 并受传动比约束。本实现采用与 Jain 《Robot and Multibody Dynamics》
 * 第 12 章所述方法类似的技术，以高效地将转子纳入模型。该实现专门针对
 * 每个刚体只有一个旋转转子的情况。转子与对应刚体具有相同的关节类型，
 * 但其运动子空间会额外乘以传动比。与浮动基座关联的转子不起作用。
 */

#ifndef LIBBIOMIMETICS_FLOATINGBASEMODEL_H
#define LIBBIOMIMETICS_FLOATINGBASEMODEL_H

#include <string>
#include <vector>
#include <stdexcept>
#include <cmath>
#include <cstdint>

#include "orientation_tools.h"
#include "SpatialInertia.h"
#include "spatial.h"

#include <eigen3/Eigen/StdVector>

using std::vector;
using namespace ori;
using namespace spatial;

/*!
 * 浮动基座模型的状态，包含基座状态和关节状态。
 */
template < typename T >
struct FBModelState
{
  Quat < T > bodyOrientation;    // 身体姿态（四元数）
  Vec3 < T > bodyPosition;       // 身体位置（x,y,z）
  SVec < T > bodyVelocity;       // 身体速度（在身体坐标系中）
  DVec < T > q;                  // 所有关节角度（12维）
  DVec < T > qd;                 // 所有关节速度（12维）

  /*!
   * 打印基座位置。
   */
  void print() const
  {
    printf(
      "position: %.3f %.3f %.3f\n", bodyPosition[0], bodyPosition[1],
      bodyPosition[2]);
  }
};

/*!
 * 对刚体浮动基座模型运行关节体算法（ABA）得到的状态导数。
 */
template < typename T >
struct FBModelStateDerivative
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Vec3 < T > dBodyPosition; // 身体速度（位置导数）
  SVec < T > dBodyVelocity;     // 身体加速度（速度导数）
  DVec < T > qdd;               // 关节加速度（12维）
};

/*!
 * 表示带有转子和地面接触点的浮动基座刚体模型。
 * 模型结构与状态数据分离，状态通过 setState() 更新。
 */
template < typename T >
class FloatingBaseModel {
public:
  /*!
   * 使用默认重力加速度初始化浮动基座模型。
   */
  FloatingBaseModel() : _gravity(0, 0, -9.81) {
  }
  ~FloatingBaseModel() {
  }

  void addBase(const SpatialInertia < T > & inertia);
  void addBase(T mass, const Vec3 < T > & com, const Mat3 < T > & I);
  int addGroundContactPoint(
    int bodyID, const Vec3 < T > & location,
    bool isFoot = false);
  void addGroundContactBoxPoints(int bodyId, const Vec3 < T > & dims);
  int addBody(
    const SpatialInertia < T > & inertia,
    const SpatialInertia < T > & rotorInertia, T gearRatio, int parent,
    JointType jointType, CoordinateAxis jointAxis,
    const Mat6 < T > & Xtree, const Mat6 < T > & Xrot);
  int addBody(
    const MassProperties < T > & inertia,
    const MassProperties < T > & rotorInertia, T gearRatio, int parent,
    JointType jointType, CoordinateAxis jointAxis,
    const Mat6 < T > & Xtree, const Mat6 < T > & Xrot);
  /** Add a body when the model supplies joint-side armature directly. */
  int addBody(
    const SpatialInertia < T > & inertia, T jointArmature, int parent,
    JointType jointType, CoordinateAxis jointAxis,
    const Mat6 < T > & Xtree, const std::string & bodyName = {});
  void check();
  T totalRotorMass() const;
  T totalNonRotorMass() const;

  size_t getNumDof() const noexcept {return _nDof;}
  size_t getNumActuatedDof() const noexcept {return _nDof >= 6 ? _nDof - 6 : 0;}
  size_t getNumGroundContacts() const noexcept {return _nGroundContact;}
  const std::vector < uint64_t > & getFootIndices() const noexcept {return _footIndicesGC;}
  const std::vector < size_t > & getGroundContactParents() const noexcept {return _gcParent;}
  const std::vector < Vec3 < T >> & getGroundContactLocations() const noexcept {return _gcLocation;}
  const vectorAligned < D3Mat < T >> & getContactJacobians() const noexcept {return _Jc;}
  const vectorAligned < Vec3 < T >> & getContactJacobianDotQdot() const noexcept {return _Jcdqd;}

  /*!
   * 获取父刚体索引数组，其中 parents[i] 是刚体 i 的父刚体。
   * @return 父刚体索引数组
   */
  const std::vector < int > & getParentVector() const {return _parents;}

  /*!
   * 获取各刚体的空间惯量数组。
   * @return 刚体空间惯量数组
   */
  const std::vector < SpatialInertia < T >,
    Eigen::aligned_allocator < SpatialInertia < T >> > &
  getBodyInertiaVector() const {
    return _Ibody;
  }

  /*!
   * 获取各转子的空间惯量数组。
   * @return 转子空间惯量数组
   */
  const std::vector < SpatialInertia < T >,
    Eigen::aligned_allocator < SpatialInertia < T >> > &
  getRotorInertiaVector() const {
    return _Irot;
  }

  /*!
   * 设置重力加速度向量。
   */
  void setGravity(const Vec3 < T > & g)
  {
    if (!g.allFinite()) {throw std::invalid_argument("gravity must be finite");}
    _gravity = g;
    _biasAccelerationsUpToDate = false;
    _accelerationsUpToDate = false;
  }

  /*!
   * 设置是否计算指定接触点的接触信息。
   * @param gc_index 接触点索引
   * @param flag true 启用接触计算，false 禁用接触计算
   */
  void setContactComputeFlag(size_t gc_index, bool flag)
  {
    _compute_contact_info.at(gc_index) = flag;
  }

  DMat < T > invContactInertia(
    const int gc_index,
    const D6Mat < T > &force_directions);
  T invContactInertia(const int gc_index, const Vec3 < T > & force_ics_at_contact);

  T applyTestForce(
    const int gc_index, const Vec3 < T > & force_ics_at_contact,
    FBModelStateDerivative < T > & dstate_out);

  T applyTestForce(
    const int gc_index, const Vec3 < T > & force_ics_at_contact,
    DVec < T > & dstate_out);

  void addDynamicsVars(int count);

  void resizeSystemMatricies();

  /*!
   * 更新模型状态，并将依赖旧状态的已缓存计算结果标记为失效。
   * @param state 新的模型状态
   */
  void setState(const FBModelState < T > & state)
  {
    if (_nDof < 6 || state.q.size() != static_cast < Eigen::Index > (_nDof - 6) ||
      state.qd.size() != static_cast < Eigen::Index > (_nDof - 6) ||
      !state.bodyOrientation.allFinite() || !state.bodyPosition.allFinite() ||
      !state.bodyVelocity.allFinite() || !state.q.allFinite() || !state.qd.allFinite())
    {
      throw std::invalid_argument("floating-base state has invalid size or non-finite values");
    }
    const T quatNorm = state.bodyOrientation.norm();
    if (!std::isfinite(static_cast < double > (quatNorm)) || quatNorm <= T(1e-8)) {
      throw std::invalid_argument("body orientation quaternion has zero norm");
    }
    _state = state;
    _state.bodyOrientation /= quatNorm;

    _biasAccelerationsUpToDate = false;  //重力+科里奥利+离心力产生的加速度
    _compositeInertiasUpToDate = false;  //每个关节的等效转动惯量

    resetCalculationFlags();
  }

  /*!
   * 将之前缓存的计算结果全部标记为失效。
   */
  void resetCalculationFlags()
  {
    _articulatedBodiesUpToDate = false; //所有连杆的位置、速度、加速度是否已算好
    _kinematicsUpToDate = false;        //从末端到基座的力传递矩阵是否已算好
    _forcePropagatorsUpToDate = false;  //每个子树的等效惯性（把下游所有连杆合并成一个刚体）是否已算好
    _qddEffectsUpToDate = false;        //关节加速度对末端惯性力的影响是否已算好
    _accelerationsUpToDate = false;     //所有连杆的加速度是否已算好
  }

  /*!
   * 更新模型状态导数，并使之前的加速度计算结果失效。
   * @param dState 新的状态导数
   */
  void setDState(const FBModelStateDerivative < T > & dState)
  {
    if (dState.qdd.size() != static_cast < Eigen::Index > (_nDof - 6) ||
      !dState.dBodyPosition.allFinite() || !dState.dBodyVelocity.allFinite() ||
      !dState.qdd.allFinite())
    {
      throw std::invalid_argument(
        "floating-base state derivative has invalid size or non-finite values");
    }
    _dState = dState;
    _accelerationsUpToDate = false;
  }

  Vec3 < T > getPosition(const int link_idx, const Vec3 < T > &local_pos);
  Vec3 < T > getPosition(const int link_idx);


  Mat3 < T > getOrientation(const int link_idx);
  Vec3 < T > getLinearVelocity(const int link_idx, const Vec3 < T > &point);
  Vec3 < T > getLinearVelocity(const int link_idx);

  Vec3 < T > getLinearAcceleration(const int link_idx, const Vec3 < T > &point);
  Vec3 < T > getLinearAcceleration(const int link_idx);

  Vec3 < T > getAngularVelocity(const int link_idx);
  Vec3 < T > getAngularAcceleration(const int link_idx);
  // 正向运动学
  void forwardKinematics();

  void biasAccelerations();
  void compositeInertias();
  void forwardAccelerationKinematics();
  void contactJacobians();

  // 重力项
  DVec < T > generalizedGravityForce();

  // 科里奥利力/离心力
  DVec < T > generalizedCoriolisForce();

  // 质量矩阵
  DMat < T > massMatrix();

  // 逆动力学：由加速度求力矩
  DVec < T > inverseDynamics(const FBModelStateDerivative < T > &dState);

  // 正动力学：由力矩求加速度
  void runABA(const DVec < T > & tau, FBModelStateDerivative < T > & dstate);

  size_t _nDof = 0;
  Vec3 < T > _gravity;
  vector < int > _parents;
  vector < T > _gearRatios;
  vector < T > _jointArmatures;
  vector < T > _d, _u;

  vector < JointType > _jointTypes;
  vector < CoordinateAxis > _jointAxes;
  vector < Mat6 < T >, Eigen::aligned_allocator < Mat6 < T >> > _Xtree, _Xrot;
  vector < SpatialInertia < T >, Eigen::aligned_allocator < SpatialInertia < T >> > _Ibody,
  _Irot;
  vector < std::string > _bodyNames;

  size_t _nGroundContact = 0;
  vector < size_t > _gcParent;
  vector < Vec3 < T >> _gcLocation;
  vector < uint64_t > _footIndicesGC;

  vector < Vec3 < T >> _pGC;
  vector < Vec3 < T >> _vGC;

  vector < bool > _compute_contact_info;

  /*!
   * 获取系统质量矩阵。
   */
  const DMat < T > & getMassMatrix() const {return _H;}

  /*!
   * 获取重力项（广义力）。
   */
  const DVec < T > & getGravityForce() const {return _G;}

  /*!
   * 获取科里奥利/离心力项（广义力）。
   */
  const DVec < T > & getCoriolisForce() const {return _Cqd;}


  /// 算法辅助变量开始
  FBModelState < T > _state;
  FBModelStateDerivative < T > _dState;

  vectorAligned < SVec < T >> _v, _vrot, _a, _arot, _avp, _avprot, _c, _crot, _S,
  _Srot, _fvp, _fvprot, _ag, _agrot, _f, _frot;

  vectorAligned < SVec < T >> _U, _Urot, _Utot, _pA, _pArot;
  vectorAligned < SVec < T >> _externalForces;

  vectorAligned < SpatialInertia < T >> _IC;
  vectorAligned < Mat6 < T >> _Xup, _Xa, _Xuprot, _IA, _ChiUp;

  DMat < T > _H, _C;
  DVec < T > _Cqd, _G;

  vectorAligned < D6Mat < T >> _J;
  vectorAligned < SVec < T >> _Jdqd;

  vectorAligned < D3Mat < T >> _Jc;
  vectorAligned < Vec3 < T >> _Jcdqd;

  bool _kinematicsUpToDate = false;
  bool _biasAccelerationsUpToDate = false;
  bool _accelerationsUpToDate = false;

  bool _compositeInertiasUpToDate = false;

  void updateArticulatedBodies();
  void updateForcePropagators();
  void udpateQddEffects();

  /*!
   * 将所有外力清零。
   */
  void resetExternalForces()
  {
    for (size_t i = 0; i < _nDof; i++) {
      _externalForces[i] = SVec < T > ::Zero();
    }
  }

  bool _articulatedBodiesUpToDate = false;
  bool _forcePropagatorsUpToDate = false;
  bool _qddEffectsUpToDate = false;

  DMat < T > _qdd_from_base_accel;
  DMat < T > _qdd_from_subqdd;
  Eigen::ColPivHouseholderQR < Mat6 < T >> _invIA5;
};

#endif  // LIBBIOMIMETICS_FLOATINGBASEMODEL_H

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

#include "Utilities/orientation_tools.h"
#include "Utilities/SpatialInertia.h"
#include "Utilities/spatial.h"

#include <eigen3/Eigen/StdVector>

using std::vector;
using namespace ori;
using namespace spatial;

/**
 * @brief 浮动基座模型状态，包含基座和关节状态。
 *
 * 基座姿态使用四元数表示，基座速度使用 6 维空间速度表示；q 和 qd
 * 只保存各可动关节的角度与速度，不包含浮动基座的 6 个自由度。
 * @tparam T 标量类型。
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

/**
 * @brief 浮动基座模型状态导数。
 *
 * 该结构保存 ABA 或逆动力学计算产生的基座位置导数、基座空间加速度
 * 和关节加速度。
 * @tparam T 标量类型。
 */
template < typename T >
struct FBModelStateDerivative
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Vec3 < T > dBodyPosition; // 身体速度（位置导数）
  SVec < T > dBodyVelocity;     // 身体加速度（速度导数）
  DVec < T > qdd;               // 关节加速度（12维）
};

/**
 * @brief 带转子和地面接触点的浮动基座刚体模型。
 *
 * 模型采用 Featherstone 刚体动力学的树结构：前 6 个广义自由度属于
 * 浮动基座，其余每个刚体通常对应一个一自由度关节。模型结构与状态数据
 * 分离，状态通过 setState() 更新；质量矩阵、重力、科里奥利项和雅可比
 * 等结果使用缓存标志避免重复计算。
 *
 * @tparam T 标量类型，通常为 float 或 double。
 */
template < typename T >
class FloatingBaseModel {
public:
  /** @brief 使用默认重力加速度 (0, 0, -9.81) 初始化模型。 */
  FloatingBaseModel() : _gravity(0, 0, -9.81) {
  }
  /** @brief 析构函数。 */
  ~FloatingBaseModel() = default;

  /** @brief 添加带空间惯量的浮动基座。 */
  void addBase(const SpatialInertia < T > & inertia);
  /** @brief 根据质量、质心和转动惯量添加浮动基座。 */
  void addBase(T mass, const Vec3 < T > & com, const Mat3 < T > & I);
  /**
   * @brief 添加一个地面接触点。
   * @param bodyID 接触点所属刚体编号。
   * @param location 接触点在刚体坐标系中的位置。
   * @param isFoot 是否将该点加入足端索引列表。
   * @return 新接触点编号。
   */
  int addGroundContactPoint(
    int bodyID, const Vec3 < T > & location,
    bool isFoot = false);
  /** @brief 按长方体八个顶点添加地面接触点。 */
  void addGroundContactBoxPoints(int bodyId, const Vec3 < T > & dims);
  /**
   * @brief 添加带转子的刚体和一自由度关节。
   * @param inertia 刚体空间惯量。
   * @param rotorInertia 转子空间惯量。
   * @param gearRatio 刚体与转子的传动比。
   * @param parent 父刚体编号。
   * @param jointType 关节类型。
   * @param jointAxis 关节轴。
   * @param Xtree 父刚体到刚体的空间变换。
   * @param Xrot 父刚体到转子的空间变换。
   * @return 新刚体编号。
   */
  /** @brief 使用 MassProperties 版本的 addBody()。 */
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
  /**
   * @brief 添加直接给定关节侧转动惯量的刚体。
   * @param jointArmature 关节侧转动惯量，必须为有限非负值。
   * @param bodyName 刚体名称。
   */
  int addBody(
    const SpatialInertia < T > & inertia, T jointArmature, int parent,
    JointType jointType, CoordinateAxis jointAxis,
    const Mat6 < T > & Xtree, const std::string & bodyName = {});
  /** @brief 检查模型数组尺寸、父子顺序和浮动基座结构是否一致。 */
  void check();
  /** @brief 返回所有转子质量之和。 */
  T totalRotorMass() const;
  /** @brief 返回所有非转子刚体质量之和。 */
  T totalNonRotorMass() const;

  /** @brief 返回广义自由度数，包含浮动基座 6 维。 */
  size_t getNumDof() const noexcept {return _nDof;}
  /** @brief 返回可驱动关节自由度数。 */
  size_t getNumActuatedDof() const noexcept {return _nDof >= 6 ? _nDof - 6 : 0;}
  /** @brief 返回地面接触点数量。 */
  size_t getNumGroundContacts() const noexcept {return _nGroundContact;}
  /** @brief 返回当前模型状态。 */
  const FBModelState<T> & getState() const noexcept {return _state;}
  /** @brief 设置“电机读数 -> 机械关节角”的常量零位偏置。 */
  void setJointPositionOffsets(const DVec<T> & offsets)
  {
    if (offsets.size() != static_cast<Eigen::Index>(getNumActuatedDof()) ||
      !offsets.allFinite())
    {
      throw std::invalid_argument("joint position offsets have invalid size or values");
    }
    _jointPositionOffsets = offsets;
  }
  /** @brief 返回各驱动关节的机械零位偏置，顺序与模型 q 一致。 */
  const DVec<T> & getJointPositionOffsets() const noexcept
  {
    return _jointPositionOffsets;
  }
  /** @brief 返回足端接触点在全部接触点中的索引。 */
  const std::vector < uint64_t > & getFootIndices() const noexcept {return _footIndicesGC;}
  const std::vector < size_t > & getGroundContactParents() const noexcept {return _gcParent;}
  const std::vector < Vec3 < T >> & getGroundContactLocations() const noexcept {return _gcLocation;}
  const vectorAligned < D3Mat < T >> & getContactJacobians() const noexcept {return _Jc;}
  const vectorAligned < Vec3 < T >> & getContactJacobianDotQdot() const noexcept {return _Jcdqd;}
  const std::vector<Vec3<T>> & getGroundContactPositions() const noexcept {return _pGC;}
  const std::vector<Vec3<T>> & getGroundContactVelocities() const noexcept {return _vGC;}

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

  /**
   * @brief 计算多个接触方向的逆接触惯量。
   * @param gc_index 接触点编号。
   * @param force_directions 接触力方向矩阵。
   * @return 接触惯量逆矩阵，数学上为 @f$J_cH^{-1}J_c^T@f$ 的方向投影。
   */
  DMat < T > invContactInertia(
    const int gc_index,
    const D6Mat < T > &force_directions);
  T invContactInertia(const int gc_index, const Vec3 < T > & force_ics_at_contact);

  /**
   * @brief 在接触点施加单位测试力并计算逆接触惯量。
   * @param gc_index 接触点编号。
   * @param force_ics_at_contact 惯性坐标系中的测试力。
   * @param dstate_out 输出状态导数。
   * @return 标量逆接触惯量。
   */
  T applyTestForce(
    const int gc_index, const Vec3 < T > & force_ics_at_contact,
    FBModelStateDerivative < T > & dstate_out);

  T applyTestForce(
    const int gc_index, const Vec3 < T > & force_ics_at_contact,
    DVec < T > & dstate_out);

  /** @brief 为新增的基座或关节分配动力学递推变量。 */
  void addDynamicsVars(int count);

  /** @brief 根据当前自由度数重新调整系统矩阵和接触雅可比尺寸。 */
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

  /** @brief 获取刚体局部点在世界坐标系中的位置。 */
  Vec3 < T > getPosition(const int link_idx, const Vec3 < T > &local_pos);
  /** @brief 获取刚体原点在世界坐标系中的位置。 */
  Vec3 < T > getPosition(const int link_idx);


  /** @brief 获取刚体相对世界坐标系的旋转矩阵。 */
  Mat3 < T > getOrientation(const int link_idx);
  /** @brief 获取刚体指定点的线速度。 */
  Vec3 < T > getLinearVelocity(const int link_idx, const Vec3 < T > &point);
  Vec3 < T > getLinearVelocity(const int link_idx);

  /** @brief 获取刚体指定点的线加速度。 */
  Vec3 < T > getLinearAcceleration(const int link_idx, const Vec3 < T > &point);
  Vec3 < T > getLinearAcceleration(const int link_idx);

  /** @brief 获取刚体角速度。 */
  Vec3 < T > getAngularVelocity(const int link_idx);
  /** @brief 获取刚体角加速度。 */
  Vec3 < T > getAngularAcceleration(const int link_idx);
  /** @brief 执行正向运动学并更新各刚体位姿、速度缓存。 */
  void forwardKinematics();

  /** @brief 计算重力和科里奥利/离心项对应的偏置加速度。 */
  void biasAccelerations();
  /** @brief 递归合成各子树的等效空间惯量。 */
  void compositeInertias();
  /** @brief 执行正向加速度递推。 */
  void forwardAccelerationKinematics();
  /** @brief 计算所有启用接触点的三维接触雅可比。 */
  void contactJacobians();

  /** @brief 返回广义重力力向量 @f$G(q)@f$。 */
  DVec < T > generalizedGravityForce();

  /** @brief 返回广义科里奥利/离心力向量 @f$C(q,\dot q)@f$。 */
  DVec < T > generalizedCoriolisForce();

  /** @brief 返回广义质量矩阵 @f$H(q)@f$。 */
  DMat < T > massMatrix();

  /** @brief 根据状态导数执行逆动力学，计算广义力。 */
  DVec < T > inverseDynamics(const FBModelStateDerivative < T > &dState);

  /** @brief 根据广义力执行 ABA 正动力学，输出状态导数。 */
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
  DVec<T> _jointPositionOffsets;  ///< 机械角 = 电机读数 + 本偏置。

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

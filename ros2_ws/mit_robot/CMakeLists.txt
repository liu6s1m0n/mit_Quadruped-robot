cmake_minimum_required(VERSION 3.8)
project(mymit_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mujoco_vendor REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FootContact.msg"
  "msg/StateEstimate.msg"
  "srv/SetControlMode.srv"
  "srv/SetStandingHeight.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(Eigen3)
ament_export_dependencies(mujoco_vendor)
ament_export_include_directories(include)

# 安装供其他节点和功能包使用的公共头文件
install(DIRECTORY include/
  DESTINATION include
  PATTERN "MPC/common_types.h" EXCLUDE
  PATTERN "MPC/convexMPC_interface.h" EXCLUDE
  PATTERN "WBC/WBC_Ctrl/BodyOriTask.hpp" EXCLUDE
  PATTERN "WBC/WBC_Ctrl/BodyPosTask.hpp" EXCLUDE
  PATTERN "WBC/WBC_Ctrl/BodyPostureTask.hpp" EXCLUDE
  PATTERN "WBC/WBC_Ctrl/BodyRyRzTask.hpp" EXCLUDE
  PATTERN "WBC/WBC_Ctrl/FixedBodyContact.hpp" EXCLUDE
  PATTERN "WBC/WBC_Ctrl/SingleContact.hpp" EXCLUDE
)

# 安装 ROS 2 启动文件
install(DIRECTORY src/go1_description/launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 安装原始 Menagerie GO1 模型和轻量 ros2_control 适配层
install(DIRECTORY
  src/go1_description/unitree_go1
  src/go1_description/urdf
  src/go1_description/config
  DESTINATION share/${PROJECT_NAME}
)

# Use one MuJoCo ABI for the model library, sensors, tests, and GUI. The local
# 3.11 installation also provides the Simulate UI used by mymit_robot_node.
set(MYMIT_LOCAL_MUJOCO_INCLUDE "${CMAKE_CURRENT_BINARY_DIR}/local_mujoco_include")
file(MAKE_DIRECTORY "${MYMIT_LOCAL_MUJOCO_INCLUDE}")
if(NOT EXISTS "${MYMIT_LOCAL_MUJOCO_INCLUDE}/mujoco")
  file(CREATE_LINK
    /usr/local/include/mujoco
    "${MYMIT_LOCAL_MUJOCO_INCLUDE}/mujoco"
    SYMBOLIC
  )
endif()

# 与 ROS 2/MuJoCo 解耦的 GO1 数学模型参数库
add_library(mymit_robot_model
  src/Array.cc
  src/MPC/ConvexMPCLocomotion.cpp
  src/MPC/Gait.cpp
  src/MPC/RobotState.cpp
  src/MPC/SolverMPC.cpp
  src/QuadProg++.cc
  src/FSM/ControlFSM.cpp
  src/FSM/FSM_State_BalanceStand.cpp
  src/FSM/FSM_State_FrontJump.cpp
  src/FSM/FSM_State_Locomotion.cpp
  src/FSM/FSM_State_Passive.cpp
  src/FSM/FSM_State_RecoveryStand.cpp
  src/FSM/FSM_State_StandUp.cpp
  src/FSM/SafetyChecker.cpp
  src/WBC/FloatingBaseModel.cpp
  src/WBC/KinWBC.cpp
  src/WBC/LocomotionCtrl/LocomotionCtrl.cpp
  src/WBC/WBC_Ctrl/BodyOriTask.cpp
  src/WBC/WBC_Ctrl/BodyPosTask.cpp
  src/WBC/ContactSet/FixedBodyContact.cpp
  src/WBC/WBC_Ctrl/LinkPosTask.cpp
  src/WBC/ContactSet/SingleContact.cpp
  src/WBC/WBC_Ctrl/WBC_Ctrl.cpp
  src/WBC/WBIC.cpp
  src/controller/ContactEstimator.cpp
  src/controller/FootSwingTrajectory.cpp
  src/controller/GaitScheduler.cpp
  src/controller/OrientationEstimator.cpp
  src/controller/PositionVelocityEstimator.cpp
  src/controller/leg_controller.cpp
  src/model/quadruped.cpp
  src/sensor/imu.cpp
  src/sensor/leg.cpp
)
target_compile_features(mymit_robot_model PUBLIC cxx_std_17)
target_include_directories(mymit_robot_model BEFORE PRIVATE
  "${MYMIT_LOCAL_MUJOCO_INCLUDE}"
)
target_include_directories(mymit_robot_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mymit_robot_model PUBLIC
  Eigen3::Eigen
  /usr/local/lib/libmujoco.so
)

install(TARGETS mymit_robot_model
  EXPORT mymit_robot_model_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(mymit_robot_model_targets HAS_LIBRARY_TARGET)
ament_export_libraries(mymit_robot_model)

# MuJoCo backend is loaded locally at runtime to isolate its bundled third-party
# symbols from ROS 2 middleware. RobotRunner still owns the unchanged pipeline.
add_library(mymit_robot_simulation_backend SHARED
  user/SimulationBackend.cpp
  user/RobotRunner.cpp
  user/SimulationDiagnostics.cpp
  user/SimulationBridge.cpp
)
target_compile_features(mymit_robot_simulation_backend PRIVATE cxx_std_17)
target_include_directories(mymit_robot_simulation_backend PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/user
)
target_include_directories(mymit_robot_simulation_backend BEFORE PRIVATE
  "${MYMIT_LOCAL_MUJOCO_INCLUDE}"
)
target_include_directories(mymit_robot_simulation_backend SYSTEM PRIVATE
  /usr/local/include/simulate
)
target_link_libraries(mymit_robot_simulation_backend
  mymit_robot_model
  Eigen3::Eigen
  /usr/local/lib/libmujoco.so
  /usr/local/lib/libsimulate.a
  glfw
  dl
  pthread
)

# ROS executable contains no direct MuJoCo dependency; it talks to the loaded
# backend through RobotMiddlewareInterface.
add_executable(mymit_robot_node
  user/main.cpp
  user/RobotRosNode.cpp
)
target_compile_features(mymit_robot_node PRIVATE cxx_std_17)
target_include_directories(mymit_robot_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/user
)
target_link_libraries(mymit_robot_node Eigen3::Eigen dl pthread)
add_dependencies(mymit_robot_node mymit_robot_simulation_backend)
ament_target_dependencies(mymit_robot_node
  ament_index_cpp
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  rosgraph_msgs
  sensor_msgs
  std_msgs
  tf2_ros
)
rosidl_get_typesupport_target(mymit_robot_node_typesupport
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(mymit_robot_node "${mymit_robot_node_typesupport}")
# libsimulate.a is distributed with GCC LTO objects. Tell GCC to use the
# available parallelism when materializing those objects; otherwise lto-wrapper
# falls back to serial LTRANS and emits a warning on every clean link.
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  target_link_options(mymit_robot_simulation_backend PRIVATE -flto=auto)
endif()
install(TARGETS mymit_robot_node mymit_robot_simulation_backend
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(mymit_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mujoco_vendor REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
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
# 3.11 installation also provides the Simulate UI used by mymit_robot_user.
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

# MuJoCo GUI application. main only starts SimulationBridge; RobotRunner owns
# the complete estimator -> FSM -> MPC/WBC -> LegController control pipeline.
add_executable(mymit_robot_user
  user/main.cpp
  user/RobotRunner.cpp
  user/SimulationBridge.cpp
)
target_compile_definitions(mymit_robot_user PRIVATE
  MYMIT_ROBOT_SCENE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/src/go1_description/unitree_go1/scene.xml"
)
target_include_directories(mymit_robot_user PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/user
)
target_include_directories(mymit_robot_user BEFORE PRIVATE
  "${MYMIT_LOCAL_MUJOCO_INCLUDE}"
)
target_include_directories(mymit_robot_user SYSTEM PRIVATE
  /usr/local/include/simulate
)
target_link_libraries(mymit_robot_user
  mymit_robot_model
  Eigen3::Eigen
  /usr/local/lib/libmujoco.so
  /usr/local/lib/libsimulate.a
  glfw
  dl
  pthread
)
# libsimulate.a is distributed with GCC LTO objects. Tell GCC to use the
# available parallelism when materializing those objects; otherwise lto-wrapper
# falls back to serial LTRANS and emits a warning on every clean link.
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  target_link_options(mymit_robot_user PRIVATE -flto=auto)
endif()
add_executable(mymit_robot_height_service
  user/StandingHeightService.cpp
)
target_compile_features(mymit_robot_height_service PRIVATE cxx_std_17)
target_include_directories(mymit_robot_height_service PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/user
)
ament_target_dependencies(mymit_robot_height_service rclcpp)
rosidl_get_typesupport_target(mymit_robot_height_service_typesupport
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(mymit_robot_height_service
  "${mymit_robot_height_service_typesupport}"
)
install(TARGETS mymit_robot_user mymit_robot_height_service
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  add_subdirectory(test)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

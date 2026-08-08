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
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FootContact.msg"
  "msg/StateEstimate.msg"
  "srv/SetControlMode.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(Eigen3)
ament_export_dependencies(mujoco_vendor)
ament_export_include_directories(include)

# 安装供其他节点和功能包使用的公共头文件
install(DIRECTORY include/
  DESTINATION include
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

# 与 ROS 2/MuJoCo 解耦的 GO1 数学模型参数库
add_library(mymit_robot_model
  src/Array.cc
  src/QuadProg++.cc
  src/WBC/FloatingBaseModel.cpp
  src/WBC/KinWBC.cpp
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
target_include_directories(mymit_robot_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mymit_robot_model PUBLIC Eigen3::Eigen mujoco::mujoco)

install(TARGETS mymit_robot_model
  EXPORT mymit_robot_model_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(mymit_robot_model_targets HAS_LIBRARY_TARGET)
ament_export_libraries(mymit_robot_model)

# 用户示例可执行程序（GUI 版）：MuJoCo 内置窗口 + 实时打印 IMU 读取的机器人参数
# 使用 /usr/local 完整版 MuJoCo(3.11)，它才包含 Simulate 图形界面(libsimulate.a)。
# 直接编译传感器源文件，避免与 mymit_robot_model 链入的两套 libmujoco 冲突。
add_executable(mymit_robot_user
  user/main.cpp
  src/controller/ContactEstimator.cpp
  src/controller/leg_controller.cpp
  src/model/quadruped.cpp
  src/sensor/imu.cpp
  src/sensor/leg.cpp
)
target_compile_definitions(mymit_robot_user PRIVATE
  MYMIT_ROBOT_SCENE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/src/go1_description/unitree_go1/scene.xml"
)
target_include_directories(mymit_robot_user PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_include_directories(mymit_robot_user SYSTEM PRIVATE
  /usr/local/include
  /usr/local/include/simulate
)
target_link_libraries(mymit_robot_user
  Eigen3::Eigen
  /usr/local/lib/libmujoco.so
  /usr/local/lib/libsimulate.a
  glfw
  dl
  pthread
)
install(TARGETS mymit_robot_user
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_add_gtest(test_quadruped
    test/test_quadruped.cpp
  )
  target_link_libraries(test_quadruped
    mymit_robot_model
    Eigen3::Eigen
  )

  ament_add_gtest(test_imu
    test/test_imu.cpp
  )
  target_compile_definitions(test_imu PRIVATE
    MYMIT_ROBOT_TEST_SCENE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/src/go1_description/unitree_go1/scene.xml"
  )
  target_link_libraries(test_imu
    mymit_robot_model
    Eigen3::Eigen
    mujoco::mujoco
  )

  ament_add_gtest(test_leg
    test/test_leg.cpp
  )
  target_compile_definitions(test_leg PRIVATE
    MYMIT_ROBOT_TEST_SCENE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/src/go1_description/unitree_go1/scene.xml"
  )
  target_link_libraries(test_leg
    mymit_robot_model
    Eigen3::Eigen
    mujoco::mujoco
  )

  ament_add_gtest(test_leg_controller
    test/test_leg_controller.cpp
  )

  ament_add_gtest(test_contact_estimator
    test/test_contact_estimator.cpp
  )
  target_compile_definitions(test_contact_estimator PRIVATE
    MYMIT_ROBOT_TEST_SCENE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/src/go1_description/unitree_go1/scene.xml"
  )
  target_link_libraries(test_contact_estimator
    mymit_robot_model
    Eigen3::Eigen
    mujoco::mujoco
  )

  ament_add_gtest(test_foot_swing_trajectory
    test/test_foot_swing_trajectory.cpp
  )
  target_link_libraries(test_foot_swing_trajectory
    mymit_robot_model
    Eigen3::Eigen
  )

  ament_add_gtest(test_gait_scheduler
    test/test_gait_scheduler.cpp
  )
  target_link_libraries(test_gait_scheduler
    mymit_robot_model
    Eigen3::Eigen
  )

  ament_add_gtest(test_orientation_estimator
    test/test_orientation_estimator.cpp
  )
  target_link_libraries(test_orientation_estimator
    mymit_robot_model
    Eigen3::Eigen
  )

  ament_add_gtest(test_position_velocity_estimator
    test/test_position_velocity_estimator.cpp
  )
  target_link_libraries(test_position_velocity_estimator
    mymit_robot_model
    Eigen3::Eigen
  )
  target_link_libraries(test_leg_controller
    mymit_robot_model
    Eigen3::Eigen
  )

  ament_add_gtest(test_floating_base_model
    test/test_floating_base_model.cpp
  )
  target_link_libraries(test_floating_base_model
    mymit_robot_model
    Eigen3::Eigen
  )

  ament_add_gtest(test_wbc
    test/test_wbc.cpp
  )
  target_link_libraries(test_wbc
    mymit_robot_model
    Eigen3::Eigen
  )

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

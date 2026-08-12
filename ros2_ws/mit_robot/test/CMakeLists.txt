# Tests are grouped by subsystem so one layer can be built and debugged without
# pulling unrelated sources into the same executable.

function(add_robot_unit_test target source)
  ament_add_gtest(${target} ${source})
  target_link_libraries(${target} mymit_robot_model Eigen3::Eigen)
endfunction()

function(add_robot_mujoco_test target source)
  add_robot_unit_test(${target} ${source})
  target_compile_definitions(${target} PRIVATE
    MYMIT_ROBOT_TEST_SCENE_PATH="${PROJECT_SOURCE_DIR}/src/go1_description/unitree_go1/scene.xml"
  )
endfunction()

# Model and dynamics
add_robot_unit_test(test_model_quadruped model/quadruped_test.cpp)
add_robot_mujoco_test(test_model_go1_consistency model/go1_model_consistency_test.cpp)
add_robot_unit_test(test_model_floating_base model/floating_base_model_test.cpp)

# Sensor adapters
add_robot_mujoco_test(test_sensor_imu sensors/imu_test.cpp)
add_robot_mujoco_test(test_sensor_leg sensors/leg_test.cpp)

# State and contact estimation
add_robot_mujoco_test(test_estimator_contact estimation/contact_estimator_test.cpp)
add_robot_unit_test(test_estimator_orientation estimation/orientation_estimator_test.cpp)
add_robot_unit_test(
  test_estimator_position_velocity estimation/position_velocity_estimator_test.cpp)

# Low-level leg, swing and gait control
add_robot_unit_test(test_control_leg control/leg_controller_test.cpp)
add_robot_unit_test(test_control_swing control/foot_swing_trajectory_test.cpp)
add_robot_unit_test(test_control_gait control/gait_scheduler_test.cpp)

# MPC, WBC and FSM are split again by responsibility for focused debugging.
add_robot_unit_test(test_mpc_gait mpc/gait_test.cpp)
add_robot_unit_test(test_mpc_solver mpc/solver_test.cpp)
add_robot_unit_test(test_mpc_locomotion mpc/locomotion_test.cpp)
add_robot_unit_test(test_mpc_robot_state mpc/robot_state_test.cpp)
add_robot_unit_test(test_wbc_link_position wbc/link_position_task_test.cpp)
add_robot_unit_test(test_wbc_optimization wbc/whole_body_optimization_test.cpp)
add_robot_unit_test(test_wbc_controller wbc/wbc_controller_test.cpp)
add_robot_unit_test(test_fsm_control fsm/control_fsm_test.cpp)
add_robot_unit_test(test_fsm_locomotion fsm/locomotion_state_test.cpp)
add_robot_unit_test(test_fsm_recovery fsm/recovery_stand_test.cpp)
add_robot_unit_test(test_fsm_safety fsm/safety_checker_test.cpp)

# Slow end-to-end MuJoCo regression; intentionally separate from unit tests.
ament_add_gtest(test_integration_simulation
  integration/simulation_estimation_consistency_test.cpp
  ${PROJECT_SOURCE_DIR}/user/RobotRunner.cpp
  ${PROJECT_SOURCE_DIR}/user/SimulationDiagnostics.cpp
)
target_compile_definitions(test_integration_simulation PRIVATE
  MYMIT_ROBOT_TEST_SCENE_PATH="${PROJECT_SOURCE_DIR}/src/go1_description/unitree_go1/scene.xml"
)
target_include_directories(test_integration_simulation PRIVATE
  ${PROJECT_SOURCE_DIR}/user
)
target_link_libraries(test_integration_simulation mymit_robot_model Eigen3::Eigen)
set_tests_properties(test_integration_simulation PROPERTIES LABELS "integration;slow")

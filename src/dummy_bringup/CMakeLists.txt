cmake_minimum_required(VERSION 3.8)
project(dummy_bringup)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(manual_panel_deployment_config test/test_manual_panel_deployment_config.py)
  ament_add_pytest_test(manual_panel_runtime_config test/test_manual_panel_runtime_config.py)
  ament_add_pytest_test(manual_panel_launch test/test_manual_panel_launch.py)
  ament_add_pytest_test(manual_panel_install_deployment test/test_manual_panel_install_deployment.py)
  ament_add_pytest_test(sensor_optional test/test_sensor_optional.py)
  ament_add_pytest_test(real_6dof_description test/test_real_6dof_description.py)
  ament_add_pytest_test(real_six_axis_preflight ../../scripts/dummy/tests/test_real_six_axis_preflight.py)
  ament_add_pytest_test(real_6dof_launch test/test_real_6dof_launch.py)
  ament_add_pytest_test(manual_moveit_planner test/test_manual_moveit_planner.py)
  ament_add_pytest_test(manual_moveit_execution test/test_manual_moveit_execution.py)
  ament_add_pytest_test(manual_moveit_continuous test/test_manual_moveit_continuous.py)
  ament_add_pytest_test(manual_moveit_autotest test/test_manual_moveit_autotest.py)
  ament_add_pytest_test(manual_moveit_tab test/test_manual_moveit_tab.py)
  ament_add_pytest_test(manual_motor_parameters test/test_manual_motor_parameters.py)
  ament_add_pytest_test(manual_moveit_target test/test_manual_moveit_target.py)
  ament_add_pytest_test(manual_scene_obstacles test/test_manual_scene_obstacles.py)
  ament_add_pytest_test(manual_moveit_service_topology test/test_manual_moveit_service_topology.py)
  ament_add_pytest_test(manual_panel_messages test/test_manual_panel_messages.py)
  ament_add_pytest_test(ros_six_axis_movej ../../scripts/dummy/tests/test_ros_six_axis_movej.py)
  ament_add_pytest_test(application_e2e ../../scripts/dummy/tests/test_application_e2e.py)
  ament_add_pytest_test(activate_real_no_load_stack ../../scripts/dummy/tests/test_activate_real_no_load_stack.py)
  add_launch_test(test/test_gazebo_runtime.launch.py TIMEOUT 120)
  add_launch_test(test/test_plan_execute.launch.py TIMEOUT 150)
  add_launch_test(test/test_backend_contract.launch.py TIMEOUT 45)
endif()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES
  ../../scripts/dummy/real_six_axis_preflight.py
  ../../scripts/dummy/convert_application_ros_limits.py
  DESTINATION share/${PROJECT_NAME}/scripts)
install(FILES ../../contracts/dummy_robot_v2/model/model.yaml
  DESTINATION share/${PROJECT_NAME}/contracts/dummy_robot_v2/model)
install(FILES ../../scripts/dummy/join_real_6dof_evidence.py
  DESTINATION share/${PROJECT_NAME}/panel_runtime/scripts/dummy)
install(FILES ../../contracts/dummy_robot_v2/codec/wire_layout.json
  DESTINATION share/${PROJECT_NAME}/panel_runtime/contracts/dummy_robot_v2/codec)
install(FILES ../../contracts/dummy_robot_v2_legacy_2_4/codec/wire_layout.json
  DESTINATION share/${PROJECT_NAME}/panel_runtime/contracts/dummy_robot_v2_legacy_2_4/codec)
install(PROGRAMS
  scripts/manual_panel_config.py
  scripts/deploy_manual_panel.py
  scripts/render_manual_panel_config.py
  scripts/manual_joint_panel.py
  scripts/manual_panel_ui.py
  scripts/manual_panel_messages.py
  scripts/manual_panel_simulator.py
  scripts/manual_motor_parameters.py
  scripts/manual_target_presets.py
  scripts/manual_moveit_planner.py
  scripts/manual_moveit_execution.py
  scripts/manual_moveit_continuous.py
  scripts/manual_moveit_tab.py
  scripts/manual_moveit_autotest.py
  scripts/manual_moveit_target.py
  scripts/manual_scene_obstacles.py
  scripts/action_gate_probe.py
  scripts/gazebo_runtime_probe.py
  scripts/planning_probe.py
  scripts/scene_publisher.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()

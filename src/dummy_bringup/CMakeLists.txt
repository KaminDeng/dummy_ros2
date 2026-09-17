cmake_minimum_required(VERSION 3.8)
project(dummy_bringup)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(sensor_optional test/test_sensor_optional.py)
  add_launch_test(test/test_gazebo_runtime.launch.py TIMEOUT 120)
  add_launch_test(test/test_plan_execute.launch.py TIMEOUT 150)
  add_launch_test(test/test_backend_contract.launch.py TIMEOUT 45)
endif()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES ../../contracts/dummy_robot_v2/model/model.yaml
  DESTINATION share/${PROJECT_NAME}/contracts/dummy_robot_v2/model)
install(PROGRAMS
  scripts/action_gate_probe.py
  scripts/gazebo_runtime_probe.py
  scripts/planning_probe.py
  scripts/scene_publisher.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()

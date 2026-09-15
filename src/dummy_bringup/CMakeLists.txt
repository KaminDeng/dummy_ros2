cmake_minimum_required(VERSION 3.8)
project(dummy_bringup)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_gazebo_runtime.launch.py TIMEOUT 120)
  add_launch_test(test/test_plan_execute.launch.py TIMEOUT 150)
endif()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES ../../contracts/dummy_robot_v1/model/model.yaml
  DESTINATION share/${PROJECT_NAME}/contracts/dummy_robot_v1/model)
install(PROGRAMS
  scripts/action_gate_probe.py
  scripts/planning_probe.py
  scripts/scene_publisher.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()

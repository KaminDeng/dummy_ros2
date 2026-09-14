cmake_minimum_required(VERSION 3.16)
project(dummy_ros_control)

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
find_package(pluginlib REQUIRED)
find_package(trajectory_msgs REQUIRED)

set(DPSDK_ROOT "$ENV{DPSDK_ROOT}" CACHE PATH "DPSDK source root")
get_filename_component(ROS_WORKSPACE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(C03_PROTOTYPE_INCLUDE "${ROS_WORKSPACE_ROOT}/contracts/dummy_robot_v1/codec")
if(NOT EXISTS "${C03_PROTOTYPE_INCLUDE}/dp_dummy_robot_v1.hpp")
  message(FATAL_ERROR "ROS C03 prototype mirror does not contain the Dummy generated codec")
endif()

add_library(dummy_goal_registration_gate INTERFACE)
target_compile_features(dummy_goal_registration_gate INTERFACE cxx_std_17)
target_include_directories(dummy_goal_registration_gate INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(dummy_guarded_joint_trajectory_controller SHARED
  src/guarded_joint_trajectory_controller.cpp)
target_compile_features(dummy_guarded_joint_trajectory_controller PRIVATE cxx_std_17)
target_link_libraries(dummy_guarded_joint_trajectory_controller dummy_goal_registration_gate)
ament_target_dependencies(dummy_guarded_joint_trajectory_controller
  controller_interface joint_trajectory_controller pluginlib trajectory_msgs)
pluginlib_export_plugin_description_file(controller_interface dummy_ros_control_plugins.xml)

if(BUILD_TESTING)
  add_executable(wire_probe_tests test/wire_probe_tests.cpp)
  target_compile_features(wire_probe_tests PRIVATE cxx_std_17)
  target_include_directories(wire_probe_tests PRIVATE
    "${C03_PROTOTYPE_INCLUDE}"
    "${DPSDK_ROOT}/libraries/dp_comm/include")
  add_test(NAME wire_probe_tests COMMAND wire_probe_tests)

  add_executable(goal_gate_tests test/goal_gate_tests.cpp)
  target_link_libraries(goal_gate_tests PRIVATE dummy_goal_registration_gate)
  add_test(NAME goal_gate_tests COMMAND goal_gate_tests)

  add_executable(guarded_plugin_load_tests test/guarded_plugin_load_tests.cpp)
  ament_target_dependencies(guarded_plugin_load_tests controller_interface pluginlib)
  add_test(NAME guarded_plugin_load_tests COMMAND guarded_plugin_load_tests)

  add_executable(request_handle_destructor_tests test/request_handle_destructor_tests.cpp)
  target_include_directories(request_handle_destructor_tests PRIVATE
    "${DPSDK_ROOT}/libraries/dp_comm/include")
  target_compile_definitions(request_handle_destructor_tests PRIVATE
    DP_COMM_RUNTIME_MAX_MESSAGE_BYTES=618
    DP_COMM_RUNTIME_STORAGE_BYTES=18000
    DP_COMM_TESTING=1)
  target_link_libraries(request_handle_destructor_tests
    "${DPSDK_ROOT}/build/posix-zenohc/libraries/dp_comm/libdp_comm_ros_like_runtime.a"
    "${DPSDK_ROOT}/build/posix-zenohc/core/dp_sdk_core/dp_osal/libdp_osal.a"
    pthread)
  add_test(NAME request_handle_destructor_tests COMMAND request_handle_destructor_tests)
endif()

install(DIRECTORY include/ DESTINATION include)
install(TARGETS
  dummy_goal_registration_gate
  dummy_guarded_joint_trajectory_controller
  EXPORT export_dummy_goal_registration_gate
  LIBRARY DESTINATION lib)
ament_export_targets(export_dummy_goal_registration_gate)
ament_package()

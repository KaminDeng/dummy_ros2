cmake_minimum_required(VERSION 3.16)
project(dummy_ros_control)

find_package(ament_cmake REQUIRED)

set(DPSDK_ROOT "$ENV{DPSDK_ROOT}" CACHE PATH "DPSDK source root")
if(NOT EXISTS "${DPSDK_ROOT}/libraries/dp_protocol/generated/include/dp_dummy_robot_v1.hpp")
  message(FATAL_ERROR "DPSDK_ROOT does not contain the C03 Dummy generated codec")
endif()

add_library(dummy_goal_registration_gate INTERFACE)
target_compile_features(dummy_goal_registration_gate INTERFACE cxx_std_17)
target_include_directories(dummy_goal_registration_gate INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

if(BUILD_TESTING)
  add_executable(wire_probe_tests test/wire_probe_tests.cpp)
  target_compile_features(wire_probe_tests PRIVATE cxx_std_17)
  target_include_directories(wire_probe_tests PRIVATE
    "${DPSDK_ROOT}/libraries/dp_protocol/generated/include"
    "${DPSDK_ROOT}/libraries/dp_comm/include")
  add_test(NAME wire_probe_tests COMMAND wire_probe_tests)

  add_executable(goal_gate_tests test/goal_gate_tests.cpp)
  target_link_libraries(goal_gate_tests PRIVATE dummy_goal_registration_gate)
  add_test(NAME goal_gate_tests COMMAND goal_gate_tests)
endif()

install(DIRECTORY include/ DESTINATION include)
install(TARGETS dummy_goal_registration_gate EXPORT export_dummy_goal_registration_gate)
ament_export_targets(export_dummy_goal_registration_gate)
ament_package()

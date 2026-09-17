cmake_minimum_required(VERSION 3.16)
project(dummy_ros_control)

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(OpenSSL REQUIRED)

set(DPSDK_ROOT "$ENV{DPSDK_ROOT}" CACHE PATH "DPSDK source root")
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/verify_sdk_inputs.cmake")
get_filename_component(ROS_WORKSPACE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(C03_PROTOTYPE_INCLUDE "${ROS_WORKSPACE_ROOT}/contracts/dummy_robot_v1/codec")
set(DUMMY_WIRE_INCLUDE "${DPSDK_ROOT}/contracts/dummy_robot_v1/codec")
if(NOT EXISTS "${DUMMY_WIRE_INCLUDE}/dp_dummy_robot_v1_dp_comm.hpp")
  message(FATAL_ERROR "Production device transport requires the published DPSDK wire-v1 codec")
endif()
add_library(dummy_dp_comm INTERFACE)
set(DUMMY_DP_COMM_SDK_SOURCES
  "${DPSDK_ROOT}/libraries/dp_comm/src/dp_comm_ros_like.cpp"
  "${DPSDK_ROOT}/libraries/dp_comm/src/dp_comm_ros_like_transport.cpp"
  "${DPSDK_ROOT}/platform/posix_native/dp_comm_zenohc_port/dp_comm_port.cpp")
set(DUMMY_DP_COMM_SDK_HASHES
  5e9ac76a760546738173b7f227f0adfa6dbe5c543dcb58d0b036823d0e48d65c
  c2f6a2ebdc691486a6daa98eb00ec0cc0c385b79983423eb7e086e9529face04
  05a927e73df40d57ae8ed3fb889cc714f081b8907fd8f5fc23ebf1b9ad01037e)
set(source_index 0)
foreach(source IN LISTS DUMMY_DP_COMM_SDK_SOURCES)
  if(NOT EXISTS "${source}")
    message(FATAL_ERROR "Missing required read-only DPSDK source: ${source}")
  endif()
  file(SHA256 "${source}" source_sha256)
  list(GET DUMMY_DP_COMM_SDK_HASHES ${source_index} expected_source_sha256)
  if(NOT source_sha256 STREQUAL expected_source_sha256)
    message(FATAL_ERROR "DPSDK PIC source changed; revalidate the transport adapter before updating its source lock: ${source}")
  endif()
  math(EXPR source_index "${source_index}+1")
  message(STATUS "DPSDK PIC input ${source}: ${source_sha256}")
endforeach()
# The prebuilt runtime uses a local-exec TLS relocation and cannot be loaded by
# pluginlib. Compile only the unchanged transport sources as PIC in this build.
add_library(dummy_dp_comm_pic STATIC ${DUMMY_DP_COMM_SDK_SOURCES})
set_target_properties(dummy_dp_comm_pic PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(dummy_dp_comm_pic PUBLIC cxx_std_17)
target_compile_definitions(dummy_dp_comm_pic PUBLIC
  DP_COMM_RUNTIME_MAX_MESSAGE_BYTES=618 DP_COMM_RUNTIME_STORAGE_BYTES=18000)
target_compile_definitions(dummy_dp_comm_pic PRIVATE
  DP_COMM_ZENOHC_TRUSTED_TRANSPORT=1 DP_COMM_DEFAULT_LOCAL_ONLY=1)
target_include_directories(dummy_dp_comm_pic PRIVATE
  "${DPSDK_ROOT}/libraries/dp_comm/include"
  "${DPSDK_ROOT}/libraries/dp_comm/src"
  "${DPSDK_ROOT}/platform/posix_native/dp_comm_zenohc_port"
  "${DPSDK_ROOT}/platform/posix_native/kernel_port"
  "${DPSDK_ROOT}/platform/common/console_device"
  "${DPSDK_ROOT}/core/dp_sdk_core/dp_osal/src"
  "${DPSDK_ROOT}/core/dp_sdk_core/dp_osal/src/interface"
  "${DPSDK_ROOT}/core/dp_sdk_core/dp_osal/src/debug"
  "${DPSDK_ROOT}/core/dp_sdk_core/dp_osal/src/impl/posix/include"
  "${DPSDK_ROOT}/build/posix-zenohc/thirdparty/zenoh-c/debug/include")
target_include_directories(dummy_dp_comm INTERFACE
  "${DUMMY_WIRE_INCLUDE}" "${DPSDK_ROOT}/libraries/dp_comm/include")
target_compile_definitions(dummy_dp_comm INTERFACE
  DP_COMM_RUNTIME_MAX_MESSAGE_BYTES=618 DP_COMM_RUNTIME_STORAGE_BYTES=18000)
target_link_libraries(dummy_dp_comm INTERFACE
  dummy_dp_comm_pic
  "${DPSDK_ROOT}/build/posix-zenohc/core/dp_sdk_core/dp_osal/libdp_osal.a"
  "${DPSDK_ROOT}/build/posix-zenohc/thirdparty/zenoh-c/debug/target/debug/libzenohc.a"
  rt pthread m dl)
if(NOT EXISTS "${C03_PROTOTYPE_INCLUDE}/dp_dummy_robot_v1.hpp")
  message(FATAL_ERROR "ROS C03 prototype mirror does not contain the Dummy generated codec")
endif()

add_library(dummy_goal_registration_gate INTERFACE)
target_compile_features(dummy_goal_registration_gate INTERFACE cxx_std_17)
target_include_directories(dummy_goal_registration_gate INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(dummy_command_channel SHARED src/command_channel.cpp)
target_compile_features(dummy_command_channel PRIVATE cxx_std_17)
target_include_directories(dummy_command_channel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(dummy_guarded_joint_trajectory_controller SHARED
  src/guarded_joint_trajectory_controller.cpp)
target_compile_features(dummy_guarded_joint_trajectory_controller PRIVATE cxx_std_17)
target_link_libraries(dummy_guarded_joint_trajectory_controller
  dummy_goal_registration_gate dummy_command_channel OpenSSL::Crypto)
ament_target_dependencies(dummy_guarded_joint_trajectory_controller
  controller_interface joint_trajectory_controller pluginlib trajectory_msgs)
pluginlib_export_plugin_description_file(controller_interface dummy_ros_control_plugins.xml)

add_library(dummy_guarded_zenbedded_system SHARED
  src/guarded_zenbedded_system.cpp src/device_transport.cpp)
target_compile_features(dummy_guarded_zenbedded_system PRIVATE cxx_std_17)
target_include_directories(dummy_guarded_zenbedded_system PRIVATE "${DUMMY_WIRE_INCLUDE}")
target_link_libraries(dummy_guarded_zenbedded_system
  dummy_goal_registration_gate dummy_command_channel dummy_dp_comm)
ament_target_dependencies(dummy_guarded_zenbedded_system hardware_interface pluginlib)
pluginlib_export_plugin_description_file(hardware_interface dummy_ros_control_plugins.xml)

if(BUILD_TESTING)
  add_executable(state_contention_tests test/state_contention_tests.cpp)
  target_link_libraries(state_contention_tests dummy_guarded_zenbedded_system)
  add_test(NAME state_contention_tests COMMAND state_contention_tests)
  add_executable(transport_component_tests test/transport_component_tests.cpp)
  target_link_libraries(transport_component_tests dummy_guarded_zenbedded_system)
  add_test(NAME transport_component_tests COMMAND transport_component_tests)
  add_test(NAME transport_six_axis_component_tests COMMAND transport_component_tests six_axis)
  set_tests_properties(transport_component_tests PROPERTIES TIMEOUT 30)
  set_tests_properties(transport_six_axis_component_tests PROPERTIES TIMEOUT 30)
  set_tests_properties(transport_component_tests transport_six_axis_component_tests
    PROPERTIES LABELS "component_only;synthetic_peer")
  add_executable(real_peer_state_probe test/real_peer_state_probe.cpp)
  target_include_directories(real_peer_state_probe PRIVATE "${DUMMY_WIRE_INCLUDE}")
  target_link_libraries(real_peer_state_probe dummy_guarded_zenbedded_system)
  add_executable(production_transport_tests test/production_transport_tests.cpp)
  target_link_libraries(production_transport_tests dummy_guarded_zenbedded_system)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME sdk_input_lock_tests COMMAND "${CMAKE_COMMAND}"
    "-DDPSDK_ROOT=${DPSDK_ROOT}" "-DTEST_ROOT=${CMAKE_CURRENT_BINARY_DIR}/sdk-lock-test"
    "-DLOCK_FILE=${CMAKE_CURRENT_SOURCE_DIR}/cmake/verify_sdk_inputs.cmake"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/test/sdk_input_lock_tests.cmake")
  add_test(NAME production_transport_tests COMMAND "${Python3_EXECUTABLE}"
    "${CMAKE_CURRENT_SOURCE_DIR}/test/run_transport_peer_test.py"
    "${DPSDK_ROOT}/build/posix-zenohc/applications/dummy_ref_control/dummy_ref_posix_node"
    "$<TARGET_FILE:production_transport_tests>")
  set_tests_properties(production_transport_tests PROPERTIES TIMEOUT 40)
  set_tests_properties(production_transport_tests PROPERTIES LABELS "d06_real_peer;negative_path")
  foreach(r04_test IN ITEMS producer_tests stale_state_tests nonblocking_tests)
    add_executable(${r04_test} test/${r04_test}.cpp)
    target_link_libraries(${r04_test} dummy_goal_registration_gate)
    add_test(NAME ${r04_test} COMMAND ${r04_test})
  endforeach()
  target_link_libraries(nonblocking_tests dummy_guarded_zenbedded_system)

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
  ament_target_dependencies(guarded_plugin_load_tests controller_interface hardware_interface pluginlib)
  set(DUMMY_PLUGIN_TEST_PREFIX "${CMAKE_BINARY_DIR}/plugin-test-prefix")
  add_test(NAME guarded_plugin_load_tests COMMAND bash -c
    "cmake --install '${CMAKE_BINARY_DIR}' --prefix '${DUMMY_PLUGIN_TEST_PREFIX}' >/dev/null && AMENT_PREFIX_PATH='${DUMMY_PLUGIN_TEST_PREFIX}:/opt/ros/jazzy' LD_LIBRARY_PATH='${DUMMY_PLUGIN_TEST_PREFIX}/lib:/opt/ros/jazzy/lib' exec '${CMAKE_CURRENT_BINARY_DIR}/guarded_plugin_load_tests'")

  add_executable(guarded_lifecycle_probe test/guarded_lifecycle_probe.cpp)
  target_link_libraries(guarded_lifecycle_probe dummy_guarded_zenbedded_system)
  ament_target_dependencies(guarded_lifecycle_probe hardware_interface pluginlib)
  add_test(NAME guarded_lifecycle_probe COMMAND bash -c
    "cmake --install '${CMAKE_BINARY_DIR}' --prefix '${DUMMY_PLUGIN_TEST_PREFIX}' >/dev/null && AMENT_PREFIX_PATH='${DUMMY_PLUGIN_TEST_PREFIX}:/opt/ros/jazzy' LD_LIBRARY_PATH='${DUMMY_PLUGIN_TEST_PREFIX}/lib:/opt/ros/jazzy/lib' exec '${CMAKE_CURRENT_BINARY_DIR}/guarded_lifecycle_probe'")

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

  add_executable(command_channel_tests test/command_channel_tests.cpp)
  target_link_libraries(command_channel_tests dummy_command_channel)
  add_test(NAME command_channel_tests COMMAND command_channel_tests)

  add_executable(dp_comm_worker_tests test/dp_comm_worker_tests.cpp)
  target_compile_definitions(dp_comm_worker_tests PRIVATE
    DP_COMM_RUNTIME_MAX_MESSAGE_BYTES=618 DP_COMM_RUNTIME_STORAGE_BYTES=18000 DP_COMM_TESTING=1)
  target_include_directories(dp_comm_worker_tests PRIVATE
    "${C03_PROTOTYPE_INCLUDE}" "${DPSDK_ROOT}/libraries/dp_comm/include")
  target_link_libraries(dp_comm_worker_tests
    dummy_guarded_zenbedded_system
    "${DPSDK_ROOT}/build/posix-zenohc/libraries/dp_comm/libdp_comm_ros_like_runtime.a"
    "${DPSDK_ROOT}/build/posix-zenohc/core/dp_sdk_core/dp_osal/libdp_osal.a"
    pthread)
  add_test(NAME dp_comm_worker_tests COMMAND dp_comm_worker_tests)

  add_executable(wire_state_ingest_tests test/wire_state_ingest_tests.cpp)
  target_include_directories(wire_state_ingest_tests PRIVATE "${C03_PROTOTYPE_INCLUDE}")
  target_link_libraries(wire_state_ingest_tests dummy_guarded_zenbedded_system)
  add_test(NAME wire_state_ingest_tests COMMAND wire_state_ingest_tests)
endif()

install(DIRECTORY include/ DESTINATION include)
install(TARGETS
  dummy_goal_registration_gate
  dummy_dp_comm
  dummy_dp_comm_pic
  dummy_command_channel
  dummy_guarded_joint_trajectory_controller
  dummy_guarded_zenbedded_system
  EXPORT export_dummy_goal_registration_gate
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
ament_export_targets(export_dummy_goal_registration_gate HAS_LIBRARY_TARGET)
ament_export_dependencies(OpenSSL)
ament_package()

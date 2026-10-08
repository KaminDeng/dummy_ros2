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
set(C03_PROTOTYPE_INCLUDE "${ROS_WORKSPACE_ROOT}/contracts/dummy_robot_v2/codec")
set(DUMMY_WIRE_INCLUDE "${DPSDK_ROOT}/contracts/dummy_robot_v2/codec")
if(NOT EXISTS "${DUMMY_WIRE_INCLUDE}/dp_dummy_robot_v2_dp_comm.hpp")
  message(FATAL_ERROR "Production device transport requires the published DPSDK wire-v1 codec")
endif()
add_library(dummy_dp_comm INTERFACE)
set(DUMMY_DP_COMM_SDK_SOURCES
  "${DPSDK_ROOT}/libraries/dp_comm/src/dp_comm_ros_like.cpp"
  "${DPSDK_ROOT}/libraries/dp_comm/src/dp_comm_ros_like_transport.cpp"
  "${DPSDK_ROOT}/platform/common/dp_comm_pico_port/dp_comm_port.cpp")
set(DUMMY_DP_COMM_SDK_HASHES
  12743ffe67b1f2f63f14e981fea72715f6c604457b6e639817d9f8e1ffdc40b1
  ad45bd1428548136f3a0bfbcddf012a86c7a49add90f80adfb82529459fc5f88
  00127fe8b0e68f913f58ea7cd87bdd28c22c0e7bfd32605d2f876c14721f4e52)
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
add_library(dummy_dp_comm_pic STATIC ${DUMMY_DP_COMM_SDK_SOURCES} src/zenoh_pico_log_shim.cpp)
set_target_properties(dummy_dp_comm_pic PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(dummy_dp_comm_pic PUBLIC cxx_std_17)
target_compile_definitions(dummy_dp_comm_pic PUBLIC
  DP_COMM_RUNTIME_MAX_MESSAGE_BYTES=651 DP_COMM_RUNTIME_STORAGE_BYTES=18000)
target_compile_definitions(dummy_dp_comm_pic PRIVATE
  DP_COMM_DEFAULT_LOCAL_ONLY=1 ZENOH_LINUX ZENOH_C_STANDARD=11 ZENOH_COMPILER_GCC ZENOH_LOG_INFO
  DPSDK_ALLOW_UNSAFE_LIBC_HEAP_FALLBACK=0 DPSDK_ENABLE_TLSF_HEAP=1
  DP_OSAL_ENABLE_CHRONO=1 DP_OSAL_ENABLE_CONDITION_VAR=1 DP_OSAL_ENABLE_MEMORY_MANAGER=1
  DP_OSAL_ENABLE_RW_LOCK=1 DP_OSAL_ENABLE_SPIN_LOCK=1 DP_OSAL_ENABLE_STATIC_ALLOC=0
  DP_OSAL_ENABLE_THREAD_POOL=1 DP_OSAL_ENABLE_THREAD_SNAPSHOT=1 DP_OSAL_ENABLE_TIMER=1)
target_include_directories(dummy_dp_comm_pic PRIVATE
  "${DPSDK_ROOT}/libraries/dp_comm/include"
  "${DPSDK_ROOT}/libraries/dp_comm/src"
  "${DPSDK_ROOT}/platform/common/dp_comm_pico_port"
  "${DPSDK_ROOT}/platform/posix_native/kernel_port"
  "${DPSDK_ROOT}/platform/common/console_device"
  "${DPSDK_ROOT}/core/dp_sdk_core/dp_osal/src"
  "${DPSDK_ROOT}/core/dp_sdk_core/dp_osal/src/interface"
  "${DPSDK_ROOT}/core/dp_sdk_core/dp_osal/src/debug"
  "${DPSDK_ROOT}/core/dp_sdk_core/dp_osal/src/impl/posix/include"
  "${DPSDK_ROOT}/build_dummy/g6-host"
  "${DPSDK_ROOT}/platform/common/zenoh_port"
  "${DPSDK_ROOT}/thirdparty/zenoh-pico/include")
target_include_directories(dummy_dp_comm INTERFACE
  "${DUMMY_WIRE_INCLUDE}" "${DPSDK_ROOT}/libraries/dp_comm/include")
target_compile_definitions(dummy_dp_comm INTERFACE
  DP_COMM_RUNTIME_MAX_MESSAGE_BYTES=651 DP_COMM_RUNTIME_STORAGE_BYTES=18000)
target_link_libraries(dummy_dp_comm INTERFACE
  dummy_dp_comm_pic
  "${DPSDK_ROOT}/build_dummy/g6-host/core/dp_sdk_core/dp_osal/libdp_osal.a"
  "${DPSDK_ROOT}/build_dummy/g6-host/platform_common_console_device/libdp_console_device.a"
  "${DPSDK_ROOT}/build_dummy/g6-host/lib/libzenohpico.a"
  rt pthread m dl)
if(NOT EXISTS "${C03_PROTOTYPE_INCLUDE}/dp_dummy_robot_v2.hpp")
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
  src/guarded_zenbedded_system.cpp src/device_transport.cpp src/raw_evidence_collector.cpp)
target_compile_features(dummy_guarded_zenbedded_system PRIVATE cxx_std_17)
target_include_directories(dummy_guarded_zenbedded_system PRIVATE "${DUMMY_WIRE_INCLUDE}")
target_link_libraries(dummy_guarded_zenbedded_system
  dummy_goal_registration_gate dummy_command_channel dummy_dp_comm OpenSSL::Crypto)
ament_target_dependencies(dummy_guarded_zenbedded_system hardware_interface pluginlib)
pluginlib_export_plugin_description_file(hardware_interface dummy_ros_control_plugins.xml)

if(BUILD_TESTING)
  add_executable(trajectory_digest_tests test/trajectory_digest_tests.cpp)
  target_link_libraries(trajectory_digest_tests PRIVATE dummy_guarded_joint_trajectory_controller)
  add_test(NAME trajectory_digest_tests COMMAND trajectory_digest_tests)
  add_executable(raw_evidence_collector_tests test/raw_evidence_collector_tests.cpp)
  target_include_directories(raw_evidence_collector_tests PRIVATE "${DUMMY_WIRE_INCLUDE}")
  target_link_libraries(raw_evidence_collector_tests dummy_guarded_zenbedded_system OpenSSL::Crypto)
  add_test(NAME raw_evidence_collector_tests COMMAND raw_evidence_collector_tests)
  add_executable(raw_evidence_collector_ndebug_tests test/raw_evidence_collector_tests.cpp)
  target_compile_definitions(raw_evidence_collector_ndebug_tests PRIVATE NDEBUG)
  target_include_directories(raw_evidence_collector_ndebug_tests PRIVATE "${DUMMY_WIRE_INCLUDE}")
  target_link_libraries(raw_evidence_collector_ndebug_tests
    dummy_guarded_zenbedded_system OpenSSL::Crypto)
  add_test(NAME raw_evidence_collector_ndebug_tests COMMAND raw_evidence_collector_ndebug_tests)
  add_executable(state_contention_tests test/state_contention_tests.cpp)
  target_link_libraries(state_contention_tests dummy_guarded_zenbedded_system)
  add_test(NAME state_contention_tests COMMAND state_contention_tests)
  add_executable(transport_component_tests test/transport_component_tests.cpp)
  target_link_libraries(transport_component_tests dummy_guarded_zenbedded_system OpenSSL::Crypto)
  add_test(NAME transport_component_tests COMMAND transport_component_tests)
  add_test(NAME transport_stop_sequence_rebase_tests
    COMMAND transport_component_tests six_axis stop_sequence_rebase)
  add_test(NAME transport_stop_uncommitted_terminal_tests
    COMMAND transport_component_tests six_axis stop_uncommitted_terminal)
  add_test(NAME transport_stop_uncommitted_idle_status_tests
    COMMAND transport_component_tests six_axis stop_uncommitted_idle_status)
  add_test(NAME transport_historical_idle_baseline_tests
    COMMAND transport_component_tests historical_idle_baseline)
  set_tests_properties(transport_historical_idle_baseline_tests PROPERTIES LABELS "component_only;offline")
  add_test(NAME transport_six_axis_component_tests COMMAND transport_component_tests six_axis)
  add_test(NAME transport_ticket_refresh_tests COMMAND transport_component_tests six_axis ticket_refresh)
  add_test(NAME transport_ticket_stale_tests COMMAND transport_component_tests six_axis ticket_stale)
  add_test(NAME transport_ticket_authority_tests COMMAND transport_component_tests six_axis ticket_authority)
  add_test(NAME transport_released_idle_deactivate_tests COMMAND transport_component_tests six_axis released_idle_deactivate)
  add_test(NAME transport_released_idle_unconfirmed_tests COMMAND transport_component_tests six_axis released_idle_unconfirmed)
  add_test(NAME transport_released_idle_no_gate_reply_tests COMMAND transport_component_tests six_axis released_idle_no_gate_reply)
  add_test(NAME transport_released_state_before_status_tests COMMAND transport_component_tests six_axis released_state_before_status)
  add_test(NAME transport_released_idle_heartbeat_gap_tests COMMAND transport_component_tests six_axis released_idle_heartbeat_gap)
  add_test(NAME transport_released_idle_before_reply_tests COMMAND transport_component_tests six_axis released_idle_before_reply)
  add_test(NAME transport_released_no_final_idle_tests COMMAND transport_component_tests six_axis released_no_final_idle)
  add_test(NAME transport_released_idle_on_error_tests COMMAND transport_component_tests six_axis released_idle_on_error)
  add_test(NAME transport_running_deactivate_tests COMMAND transport_component_tests six_axis running_deactivate)
  add_test(NAME transport_raw_capture_component_tests COMMAND transport_component_tests six_axis raw_capture)
  add_test(NAME transport_raw_release_receipt_component_tests COMMAND transport_component_tests six_axis raw_release_receipt)
  add_test(NAME transport_raw_direct_receipt_component_tests COMMAND transport_component_tests six_axis raw_direct_receipt)
  add_test(NAME transport_raw_bad_release_component_tests COMMAND transport_component_tests six_axis raw_bad_release)
  add_test(NAME transport_raw_incomplete_component_tests COMMAND transport_component_tests six_axis raw_incomplete)
  add_test(NAME transport_raw_many_component_tests COMMAND transport_component_tests six_axis raw_many)
  add_test(NAME transport_raw_order_component_tests COMMAND transport_component_tests six_axis raw_order)
  add_test(NAME transport_raw_order_bad_boot_component_tests COMMAND transport_component_tests six_axis raw_order_bad_boot)
  add_test(NAME transport_raw_order_bad_session_component_tests COMMAND transport_component_tests six_axis raw_order_bad_session)
  add_test(NAME transport_raw_order_bad_kind_component_tests COMMAND transport_component_tests six_axis raw_order_bad_kind)
  add_test(NAME transport_raw_order_bad_chain_component_tests COMMAND transport_component_tests six_axis raw_order_bad_chain)
  add_test(NAME transport_raw_order_unverified_terminal_component_tests COMMAND transport_component_tests six_axis raw_order_unverified_terminal)
  add_test(NAME transport_raw_order_missing_status_component_tests COMMAND transport_component_tests six_axis raw_order_missing_status)
  add_test(NAME transport_raw_order_duplicate_seq_component_tests COMMAND transport_component_tests six_axis raw_order_duplicate_seq)
  add_test(NAME transport_raw_order_release_component_tests COMMAND transport_component_tests six_axis raw_order_release)
  add_test(NAME transport_raw_order_release_bad_boot_component_tests COMMAND transport_component_tests six_axis raw_order_release_bad_boot)
  add_test(NAME transport_raw_order_release_bad_kind_component_tests COMMAND transport_component_tests six_axis raw_order_release_bad_kind)
  add_test(NAME transport_raw_order_release_bad_chain_component_tests COMMAND transport_component_tests six_axis raw_order_release_bad_chain)
  add_test(NAME transport_raw_order_release_missing_final_component_tests COMMAND transport_component_tests six_axis raw_order_release_missing_final)
  add_test(NAME transport_raw_order_release_wrong_final_chain_component_tests COMMAND transport_component_tests six_axis raw_order_release_wrong_final_chain)
  set(_refresh_cases
    raw_order_refresh_page_first
    raw_order_refresh_page_first_bad_boot
    raw_order_refresh_page_first_bad_session
    raw_order_refresh_page_first_old_run
    raw_order_refresh_page_first_bad_category
    raw_order_refresh_page_first_bad_group_count
    raw_order_refresh_page_first_bad_mask
    raw_order_refresh_page_first_zero_motion_epoch
    raw_order_refresh_page_first_wrong_final_chain
    raw_order_refresh_final_status_before_terminal
    raw_order_refresh_hold_page_first
    raw_order_refresh_hold_bad_boot
    raw_order_refresh_hold_bad_session
    raw_order_refresh_hold_no_terminal
    raw_order_refresh_hold_bad_motion
    raw_order_refresh_hold_bad_run
    raw_order_refresh_hold_bad_mask
    raw_order_refresh_hold_no_motion_complete
    raw_order_refresh_hold_no_receipt
    raw_order_multi_successor
    raw_order_multi_successor_stale_terminal
    raw_order_multi_successor_bad_boot
    raw_order_multi_successor_bad_session
    raw_order_multi_successor_bad_run
    raw_order_multi_successor_no_terminal
    raw_order_multi_successor_late_wrong_chain
    raw_order_multi_successor_late_correct_chain)
  foreach(_case IN LISTS _refresh_cases)
    add_test(NAME transport_${_case}_component_tests
      COMMAND transport_component_tests six_axis ${_case})
    list(APPEND _refresh_tests transport_${_case}_component_tests)
  endforeach()
  add_test(NAME transport_complete_status_first_tests COMMAND transport_component_tests six_axis complete_status_first)
  add_test(NAME transport_complete_state_first_tests COMMAND transport_component_tests six_axis complete_state_first)
  add_test(NAME transport_complete_response_first_tests COMMAND transport_component_tests six_axis complete_response_first)
  add_test(NAME transport_complete_unsolicited_jump_tests COMMAND transport_component_tests six_axis complete_unsolicited_jump)
  add_test(NAME transport_complete_state_timeout_tests COMMAND transport_component_tests six_axis complete_state_timeout)
  add_test(NAME transport_complete_hold_ack_tests COMMAND transport_component_tests six_axis complete_hold_ack)
  add_test(NAME transport_complete_hold_ack_mismatch_tests COMMAND transport_component_tests six_axis complete_hold_ack_mismatch)
  add_test(NAME transport_complete_pending_hold_suppressed_tests COMMAND transport_component_tests six_axis complete_pending_hold_suppressed)
  add_test(NAME transport_complete_fault_status_tests COMMAND transport_component_tests six_axis complete_fault_status)
  add_test(NAME transport_complete_hold_before_response_tests COMMAND transport_component_tests six_axis complete_hold_before_response)
  add_test(NAME transport_complete_freeze_old_epoch_tests COMMAND transport_component_tests six_axis complete_freeze_old_epoch)
  add_test(NAME transport_complete_wait_terminal_ack_tests COMMAND transport_component_tests six_axis complete_wait_terminal_ack)
  add_test(NAME transport_complete_hold_small_delta_tests COMMAND transport_component_tests six_axis complete_hold_small_delta)
  add_test(NAME transport_complete_hold_expired_pending_tests COMMAND transport_component_tests six_axis complete_hold_expired_pending)
  add_test(NAME transport_complete_invalid_binding_tests COMMAND transport_component_tests six_axis complete_invalid_binding)
  add_test(NAME transport_complete_one_prime_tests COMMAND transport_component_tests six_axis complete_one_prime)
  add_test(NAME transport_complete_export_ready_refresh_tests COMMAND transport_component_tests six_axis complete_export_ready_refresh)
  add_test(NAME transport_complete_no_export_refresh_tests COMMAND transport_component_tests six_axis complete_no_export_refresh)
  add_test(NAME transport_complete_periodic_refresh_tests COMMAND transport_component_tests six_axis complete_periodic_refresh)
  add_test(NAME transport_complete_refresh_stale_authority_tests COMMAND transport_component_tests six_axis complete_refresh_stale_authority)
  add_test(NAME transport_complete_export_refresh_ack_unconfirmed_tests COMMAND transport_component_tests six_axis complete_export_refresh_ack_unconfirmed)
  add_test(NAME transport_complete_wrong_run_chain_tests COMMAND transport_component_tests six_axis complete_wrong_run_chain)
  add_test(NAME transport_complete_lost_chain_tests COMMAND transport_component_tests six_axis complete_lost_chain)
  add_test(NAME transport_complete_export_then_lost_tests COMMAND transport_component_tests six_axis complete_export_then_lost)
  add_test(NAME transport_complete_export_then_regressed_tests COMMAND transport_component_tests six_axis complete_export_then_regressed)
  add_test(NAME transport_complete_export_then_wrong_kind_tests COMMAND transport_component_tests six_axis complete_export_then_wrong_kind)
  set_tests_properties(transport_component_tests PROPERTIES TIMEOUT 30)
  set_tests_properties(transport_six_axis_component_tests PROPERTIES TIMEOUT 30)
  set_tests_properties(transport_component_tests transport_six_axis_component_tests
    transport_raw_capture_component_tests
    transport_raw_bad_release_component_tests
    transport_raw_incomplete_component_tests
    transport_raw_many_component_tests
    transport_raw_order_component_tests
    transport_raw_order_bad_boot_component_tests
    transport_raw_order_bad_session_component_tests
    transport_raw_order_bad_kind_component_tests
    transport_raw_order_bad_chain_component_tests
    transport_raw_order_unverified_terminal_component_tests
    transport_raw_order_missing_status_component_tests
    transport_raw_order_duplicate_seq_component_tests
    transport_raw_order_release_component_tests
    transport_raw_order_release_bad_boot_component_tests
    transport_raw_order_release_bad_kind_component_tests
    transport_raw_order_release_bad_chain_component_tests
    transport_raw_order_release_missing_final_component_tests
    transport_raw_order_release_wrong_final_chain_component_tests
    ${_refresh_tests}
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
  add_test(NAME production_transport_peer_discovery_tests COMMAND "${Python3_EXECUTABLE}"
    "${CMAKE_CURRENT_SOURCE_DIR}/test/run_transport_peer_test_tests.py")
  foreach(r04_test IN ITEMS producer_tests stale_state_tests nonblocking_tests)
    add_executable(${r04_test} test/${r04_test}.cpp)
    target_link_libraries(${r04_test} dummy_goal_registration_gate)
    add_test(NAME ${r04_test} COMMAND ${r04_test})
  endforeach()
  target_link_libraries(nonblocking_tests dummy_guarded_zenbedded_system)

  add_executable(wire_probe_tests test/wire_probe_tests.cpp)
  target_compile_features(wire_probe_tests PRIVATE cxx_std_17)
  target_include_directories(wire_probe_tests PRIVATE
    "${DUMMY_WIRE_INCLUDE}"
    "${DPSDK_ROOT}/libraries/dp_comm/include")
  add_test(NAME wire_probe_tests COMMAND wire_probe_tests)
  add_executable(wire_minor4_codec_tests test/wire_minor4_codec_tests.cpp)
  target_compile_features(wire_minor4_codec_tests PRIVATE cxx_std_17)
  target_include_directories(wire_minor4_codec_tests PRIVATE "${C03_PROTOTYPE_INCLUDE}")
  add_test(NAME wire_minor4_codec_tests COMMAND wire_minor4_codec_tests)

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
    "${DPSDK_ROOT}/build_dummy/g6-host/libraries/dp_comm/libdp_comm_ros_like_runtime.a"
    "${DPSDK_ROOT}/build_dummy/g6-host/core/dp_sdk_core/dp_osal/libdp_osal.a"
    "${DPSDK_ROOT}/build_dummy/g6-host/platform_common_console_device/libdp_console_device.a"
    pthread)
  add_test(NAME request_handle_destructor_tests COMMAND request_handle_destructor_tests)

  add_executable(management_admission_retry_tests test/management_admission_retry_tests.cpp)
  target_compile_features(management_admission_retry_tests PRIVATE cxx_std_17)
  target_include_directories(management_admission_retry_tests PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${DPSDK_ROOT}/libraries/dp_comm/include")
  add_test(NAME management_admission_retry_tests COMMAND management_admission_retry_tests)

  add_executable(command_channel_tests test/command_channel_tests.cpp)
  target_link_libraries(command_channel_tests dummy_command_channel)
  add_test(NAME command_channel_tests COMMAND command_channel_tests)

  foreach(position_only_test IN ITEMS capability_limit_tests latest_only_stream_tests)
    add_executable(${position_only_test} test/${position_only_test}.cpp)
    target_link_libraries(${position_only_test} dummy_guarded_zenbedded_system)
    add_test(NAME ${position_only_test} COMMAND ${position_only_test})
  endforeach()

  add_executable(dp_comm_worker_tests test/dp_comm_worker_tests.cpp)
  target_compile_definitions(dp_comm_worker_tests PRIVATE
    DP_COMM_TESTING=1)
  target_include_directories(dp_comm_worker_tests PRIVATE
    "${C03_PROTOTYPE_INCLUDE}" "${DPSDK_ROOT}/libraries/dp_comm/include")
  target_link_libraries(dp_comm_worker_tests
    dummy_guarded_zenbedded_system
    dummy_dp_comm)
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

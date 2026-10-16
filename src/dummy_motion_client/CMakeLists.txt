cmake_minimum_required(VERSION 3.16)
project(dummy_motion_client)
find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(dummy_ros_control REQUIRED)
find_package(controller_interface REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(controller_manager REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)
set(DPSDK_ROOT "$ENV{DPSDK_ROOT}" CACHE PATH "DPSDK source root with published wire codec")
if(NOT DPSDK_ROOT)
  get_filename_component(DPSDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../dpsdk" ABSOLUTE)
endif()
set(DUMMY_WIRE_INCLUDE "${DPSDK_ROOT}/contracts/dummy_robot_v2/codec")
if(NOT EXISTS "${DUMMY_WIRE_INCLUDE}/dp_dummy_robot_v2.hpp")
  message(FATAL_ERROR "The published DPSDK manage_v2 wire codec is required")
endif()
add_library(dummy_device_action STATIC
  src/action_gate.cpp src/bench_permit.cpp src/bootstrap_coordinator.cpp src/management_client.cpp)
target_link_libraries(dummy_device_action PUBLIC OpenSSL::Crypto
  PRIVATE dummy_ros_control::dummy_goal_registration_gate)
target_compile_features(dummy_device_action PUBLIC cxx_std_17)
target_include_directories(dummy_device_action PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${DUMMY_WIRE_INCLUDE}>
  $<INSTALL_INTERFACE:include>)
add_executable(native_action_gate src/native_action_gate.cpp)
add_library(dummy_dp_management STATIC src/dp_management_transport.cpp)
target_compile_definitions(dummy_dp_management PUBLIC DP_COMM_RUNTIME_MAX_MESSAGE_BYTES=651)
target_link_libraries(dummy_dp_management PUBLIC dummy_device_action dummy_ros_control::dummy_dp_comm
  dummy_ros_control::dummy_guarded_zenbedded_system)
add_library(dummy_device_profile STATIC src/device_profile.cpp)
target_link_libraries(dummy_device_profile PUBLIC dummy_device_action PRIVATE yaml-cpp OpenSSL::Crypto)
add_library(dummy_native_goal_validation STATIC src/native_goal_validation.cpp)
target_link_libraries(dummy_native_goal_validation dummy_device_action)
ament_target_dependencies(dummy_native_goal_validation control_msgs)
add_library(dummy_goal_timeline STATIC src/goal_timeline.cpp)
target_include_directories(dummy_goal_timeline PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(dummy_goal_timeline PUBLIC cxx_std_17)
target_link_libraries(native_action_gate dummy_native_goal_validation)
target_compile_features(native_action_gate PRIVATE cxx_std_17)
ament_target_dependencies(native_action_gate control_msgs rclcpp rclcpp_action)
add_executable(device_action_gate src/device_action_gate.cpp)
target_link_libraries(device_action_gate dummy_dp_management dummy_native_goal_validation dummy_device_profile
  dummy_ros_control::dummy_guarded_joint_trajectory_controller dummy_ros_control::dummy_command_channel
  dummy_goal_timeline)
ament_target_dependencies(device_action_gate controller_manager control_msgs rclcpp rclcpp_action
  controller_interface joint_trajectory_controller trajectory_msgs std_srvs)
install(TARGETS device_action_gate DESTINATION lib/${PROJECT_NAME})
install(TARGETS native_action_gate DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  add_executable(limited_goal_budget_tests test/limited_goal_budget_tests.cpp)
  target_include_directories(limited_goal_budget_tests PRIVATE include)
  target_compile_options(limited_goal_budget_tests PRIVATE -UNDEBUG)
  add_test(NAME limited_goal_budget_tests COMMAND limited_goal_budget_tests)
  set_tests_properties(limited_goal_budget_tests PROPERTIES LABELS "component_only;offline")
  find_package(Threads REQUIRED)
  # 历史ROS05真机原件保持wire2.4字节，独立target只用对应legacy codec；
  # 不把旧payload改写成新协议，也不将此target混入生产wire2.5链接。
  # 私有软件故障证据夹具复用原codec/完整collector，不是目标硬件资格。
  add_executable(fault_cleanup_receipt_tests test/fault_cleanup_receipt_tests.cpp
    "${CMAKE_CURRENT_SOURCE_DIR}/../dummy_ros_control/src/raw_evidence_collector.cpp")
  target_include_directories(fault_cleanup_receipt_tests PRIVATE include
    "${CMAKE_CURRENT_SOURCE_DIR}/../dummy_ros_control/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../contracts/dummy_robot_v2_legacy_2_4/codec")
  target_compile_options(fault_cleanup_receipt_tests PRIVATE -UNDEBUG)
  target_compile_features(fault_cleanup_receipt_tests PRIVATE cxx_std_17)
  target_link_libraries(fault_cleanup_receipt_tests PRIVATE OpenSSL::Crypto Threads::Threads)
  add_test(NAME fault_cleanup_verified_receipt_and_current_observation COMMAND fault_cleanup_receipt_tests)
  set_tests_properties(fault_cleanup_verified_receipt_and_current_observation PROPERTIES LABELS "component_only;offline" TIMEOUT 15)
  add_executable(fault_cleanup_timeline_tests test/fault_cleanup_timeline_tests.cpp)
  target_include_directories(fault_cleanup_timeline_tests PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../dummy_ros_control/include" "${DUMMY_WIRE_INCLUDE}")
  target_compile_options(fault_cleanup_timeline_tests PRIVATE -UNDEBUG)
  target_link_libraries(fault_cleanup_timeline_tests PRIVATE dummy_goal_timeline Threads::Threads)
  add_test(NAME fault_cleanup_clock_and_timeline_boundaries COMMAND fault_cleanup_timeline_tests)
  set_tests_properties(fault_cleanup_clock_and_timeline_boundaries PROPERTIES LABELS "component_only;offline" TIMEOUT 15)
  add_executable(goal_timeline_tests test/goal_timeline_tests.cpp)
  target_link_libraries(goal_timeline_tests PRIVATE dummy_goal_timeline Threads::Threads)
  add_test(NAME goal_timeline_tests COMMAND goal_timeline_tests)
  set_tests_properties(goal_timeline_tests PROPERTIES LABELS "component_only;offline")
  add_executable(serial_pty_bridge test/serial_pty_bridge.cpp)
  target_link_libraries(serial_pty_bridge PRIVATE Threads::Threads)
  add_executable(bench_permit_tests test/bench_permit_tests.cpp)
  target_link_libraries(bench_permit_tests dummy_device_action)
  add_test(NAME bench_permit_tests COMMAND bench_permit_tests)
  add_executable(c05_action_peer test/c05_action_peer.cpp)
  target_link_libraries(c05_action_peer dummy_dp_management dummy_device_profile)
  add_executable(device_profile_tests test/device_profile_tests.cpp)
  target_link_libraries(device_profile_tests dummy_device_profile)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  # 仅检查正式镜像身份；不得据此授予DeviceProfile运行/运动权限。
  add_executable(formal_bundle_identity_probe test/formal_bundle_identity_probe.cpp)
  target_link_libraries(formal_bundle_identity_probe PRIVATE dummy_device_profile)
  add_test(NAME formal_bundle_identity_integrity COMMAND formal_bundle_identity_probe
    "${CMAKE_CURRENT_SOURCE_DIR}/../../contracts/dummy_robot_v2")
  add_test(NAME formal_contract_canonical_mirror_integrity COMMAND ${Python3_EXECUTABLE} -m pytest -q
    "${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/dummy/tests/test_formal_contract_mirror.py")
  set_tests_properties(formal_bundle_identity_integrity formal_contract_canonical_mirror_integrity
    PROPERTIES TIMEOUT 15 LABELS "component_only;offline;source_integrity")
  add_test(NAME device_profile_tests COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/run_device_profile_fixture.py"
    "$<TARGET_FILE:device_profile_tests>"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../contracts/dummy_robot_v2")
  add_executable(application_hold_candidate_profile_tests test/application_hold_candidate_profile_tests.cpp)
  target_compile_options(application_hold_candidate_profile_tests PRIVATE -UNDEBUG)
  target_link_libraries(application_hold_candidate_profile_tests dummy_device_profile)
  add_test(NAME application_hold_candidate_profile_tests COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/run_application_hold_candidate_profile.py"
    "$<TARGET_FILE:application_hold_candidate_profile_tests>" "${DPSDK_ROOT}"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../contracts/dummy_robot_v2_application_hold_no_load_candidate")
  add_test(NAME device_action_e2e_fixture
    COMMAND ${Python3_EXECUTABLE} -m pytest -q
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_device_action_e2e_fixture.py)
  set_tests_properties(device_action_e2e_fixture PROPERTIES
    ENVIRONMENT "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1" LABELS "component_only;offline")
  add_test(NAME device_action_activation_wiring
    COMMAND ${Python3_EXECUTABLE} -m pytest -q
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_device_action_activation_wiring.py)
  set_tests_properties(device_action_activation_wiring PROPERTIES
    ENVIRONMENT "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1" LABELS "component_only;offline")
  add_test(NAME device_action_public_probe COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/device_action_public_probe.py $<TARGET_FILE:device_action_gate>)
  set_tests_properties(device_action_public_probe PROPERTIES TIMEOUT 30 LABELS "component_only;ros_action")
  set(DUMMY_D08_PEER_EXECUTABLE "$ENV{DUMMY_D08_PEER_EXECUTABLE}" CACHE FILEPATH
    "DPSDK D08 peer executable for device_action_e2e_probe")
  if(NOT DUMMY_D08_PEER_EXECUTABLE)
    set(DUMMY_D08_PEER_EXECUTABLE
      "${DPSDK_ROOT}/build_dummy/v2-tests/applications/dummy_ref_control/dummy_ref_d08_device_peer")
  endif()
  # 两种 SIL 测试时钟角色必须由调用方显式绑定；旧参数仅兼容 fallback。
  set(DUMMY_D08_J1_PEER_EXECUTABLE "$ENV{DUMMY_D08_J1_PEER_EXECUTABLE}" CACHE FILEPATH
    "DPSDK D08 10/10ms synthetic peer for device_action_e2e_probe")
  set(DUMMY_D08_SIX_AXIS_PEER_EXECUTABLE "$ENV{DUMMY_D08_SIX_AXIS_PEER_EXECUTABLE}" CACHE FILEPATH
    "DPSDK D08 10/20ms synthetic peer for smoke probes")
  if(NOT DUMMY_D08_J1_PEER_EXECUTABLE)
    set(DUMMY_D08_J1_PEER_EXECUTABLE "${DUMMY_D08_PEER_EXECUTABLE}")
  endif()
  if(NOT DUMMY_D08_SIX_AXIS_PEER_EXECUTABLE)
    set(DUMMY_D08_SIX_AXIS_PEER_EXECUTABLE "${DUMMY_D08_PEER_EXECUTABLE}")
  endif()
  add_test(NAME device_action_e2e_probe COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/device_action_e2e_probe.py $<TARGET_FILE:device_action_gate>
    "${DUMMY_D08_J1_PEER_EXECUTABLE}"
    ${CMAKE_CURRENT_SOURCE_DIR}/../../contracts/dummy_robot_v2 real)
  add_test(NAME device_action_smoke_probe COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/device_action_e2e_probe.py $<TARGET_FILE:device_action_gate>
    "${DUMMY_D08_SIX_AXIS_PEER_EXECUTABLE}"
    ${CMAKE_CURRENT_SOURCE_DIR}/../../contracts/dummy_robot_v2 real_smoke)
  add_test(NAME device_action_six_axis_smoke_probe COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/device_action_e2e_probe.py $<TARGET_FILE:device_action_gate>
    "${DUMMY_D08_SIX_AXIS_PEER_EXECUTABLE}"
    ${CMAKE_CURRENT_SOURCE_DIR}/../../contracts/dummy_robot_v2 real_six_axis_smoke)
  add_test(NAME device_motion_unsupported_probe COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/device_action_e2e_probe.py $<TARGET_FILE:device_action_gate>
    $<TARGET_FILE:c05_action_peer> ${CMAKE_CURRENT_SOURCE_DIR}/../../contracts/dummy_robot_v2 unsupported)
  add_test(NAME device_motion_supported_probe COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/device_action_e2e_probe.py $<TARGET_FILE:device_action_gate>
    $<TARGET_FILE:c05_action_peer> ${CMAKE_CURRENT_SOURCE_DIR}/../../contracts/dummy_robot_v2)
  add_test(NAME device_motion_six_axis_smoke_probe COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/device_action_e2e_probe.py $<TARGET_FILE:device_action_gate>
    $<TARGET_FILE:c05_action_peer> ${CMAKE_CURRENT_SOURCE_DIR}/../../contracts/dummy_robot_v2 six_axis_smoke)
  set_tests_properties(device_action_e2e_probe PROPERTIES
    TIMEOUT 60 ENVIRONMENT "DUMMY_E2E_TRANSPORT=serial"
    LABELS "real_dpsdk_peer;ros_action;bench_release")
  set_tests_properties(device_action_smoke_probe PROPERTIES
    TIMEOUT 45 ENVIRONMENT "DUMMY_E2E_TRANSPORT=serial;DUMMY_E2E_STATE_PERIOD_US=20000"
    LABELS "real_dpsdk_peer;ros_action;offline_smoke")
  set_tests_properties(device_action_six_axis_smoke_probe PROPERTIES
    TIMEOUT 45 ENVIRONMENT "DUMMY_E2E_TRANSPORT=serial;DUMMY_E2E_STATE_PERIOD_US=20000"
    LABELS "real_dpsdk_peer;ros_action;offline_smoke;six_axis")
  set_tests_properties(device_motion_unsupported_probe PROPERTIES
    TIMEOUT 45 ENVIRONMENT "DUMMY_E2E_TRANSPORT=serial" LABELS "component_only;synthetic_peer;ros_action")
  set_tests_properties(device_motion_supported_probe PROPERTIES
    TIMEOUT 60 ENVIRONMENT "DUMMY_E2E_TRANSPORT=serial;DUMMY_E2E_STATE_PERIOD_US=20000"
    LABELS "component_only;synthetic_peer;ros_action")
  set_tests_properties(device_motion_six_axis_smoke_probe PROPERTIES
    TIMEOUT 45 ENVIRONMENT "DUMMY_E2E_TRANSPORT=serial"
    LABELS "component_only;synthetic_peer;ros_action;offline_smoke;six_axis")
  set_property(TEST device_action_e2e_fixture device_action_e2e_probe device_action_smoke_probe
    device_action_six_axis_smoke_probe device_motion_unsupported_probe
    device_motion_supported_probe device_motion_six_axis_smoke_probe
    APPEND PROPERTY ENVIRONMENT "DUMMY_SERIAL_BRIDGE_EXECUTABLE=$<TARGET_FILE:serial_pty_bridge>")
  add_executable(device_goal_validation_tests test/device_goal_validation_tests.cpp)
  target_link_libraries(device_goal_validation_tests dummy_native_goal_validation)
  ament_target_dependencies(device_goal_validation_tests control_msgs)
  add_test(NAME device_goal_validation_tests COMMAND device_goal_validation_tests)
  add_executable(dp_management_transport_tests test/dp_management_transport_tests.cpp)
  target_link_libraries(dp_management_transport_tests dummy_dp_management)
  ament_target_dependencies(dp_management_transport_tests hardware_interface)
  add_test(NAME dp_management_transport_tests COMMAND dp_management_transport_tests)
  add_test(NAME observation_continuity_tests COMMAND dp_management_transport_tests continuity)
  set_tests_properties(observation_continuity_tests PROPERTIES LABELS "component_only;offline")
  add_test(NAME dp_management_raw_first_page_tests
    COMMAND dp_management_transport_tests raw_first_page)
  set_tests_properties(dp_management_raw_first_page_tests PROPERTIES LABELS "component_only;offline")
  add_test(NAME release_transition_tests COMMAND dp_management_transport_tests release_transition)
  set_tests_properties(release_transition_tests PROPERTIES LABELS "component_only;offline")
  add_test(NAME dp_management_transport_tcp_untrusted_tests COMMAND dp_management_transport_tests tcp)
  add_executable(management_request_minor_tests test/management_request_minor_tests.cpp)
  target_link_libraries(management_request_minor_tests dummy_device_action)
  add_test(NAME management_request_minor_tests COMMAND management_request_minor_tests)
  set_tests_properties(dp_management_transport_tests dp_management_transport_tcp_untrusted_tests PROPERTIES
    TIMEOUT 25 LABELS "component_only;synthetic_peer")
  add_executable(shared_admission_retry_tests test/shared_admission_retry_tests.cpp)
  target_link_libraries(shared_admission_retry_tests dummy_dp_management)
  target_compile_options(shared_admission_retry_tests PRIVATE -UNDEBUG)
  foreach(mode IN ITEMS select grant capture queued_other wrong_size invalid_codec not_ready cancel destroy stop stop_restart unready expired poll_queued_other)
    add_test(NAME shared_admission_retry_${mode} COMMAND shared_admission_retry_tests ${mode})
    set_tests_properties(shared_admission_retry_${mode} PROPERTIES LABELS "component_only;offline" TIMEOUT 5)
  endforeach()

  add_executable(control_chain_logging_tests test/control_chain_logging_tests.cpp)
  target_include_directories(control_chain_logging_tests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../dummy_ros_control/include")
  target_compile_options(control_chain_logging_tests PRIVATE -UNDEBUG)
  target_link_libraries(control_chain_logging_tests PRIVATE dummy_device_action)
  foreach(mode IN ITEMS action bootstrap)
    add_test(NAME control_chain_slow_stderr_${mode}
      COMMAND ${Python3_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/../dummy_ros_control/test/run_slow_stderr_fixture.py"
        "$<TARGET_FILE:control_chain_logging_tests>" ${mode})
    set_tests_properties(control_chain_slow_stderr_${mode} PROPERTIES TIMEOUT 45 LABELS "component_only;offline")
  endforeach()
  add_executable(management_logging_scope_tests test/management_logging_scope_tests.cpp)
  target_compile_options(management_logging_scope_tests PRIVATE -UNDEBUG)
  target_link_libraries(management_logging_scope_tests PRIVATE dummy_dp_management)
  add_test(NAME management_scope_slow_stderr
    COMMAND ${Python3_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/../dummy_ros_control/test/run_slow_stderr_fixture.py"
      "$<TARGET_FILE:management_logging_scope_tests>")
  set_tests_properties(management_scope_slow_stderr PROPERTIES TIMEOUT 45 LABELS "component_only;offline"
    ENVIRONMENT "LD_LIBRARY_PATH=$<TARGET_FILE_DIR:dummy_ros_control::dummy_command_channel>:$ENV{LD_LIBRARY_PATH}")
  add_executable(shared_management_channel_tests test/shared_management_channel_tests.cpp)
  target_link_libraries(shared_management_channel_tests dummy_dp_management)
  ament_target_dependencies(shared_management_channel_tests hardware_interface)
  add_test(NAME shared_management_channel_tests COMMAND shared_management_channel_tests)
  set_tests_properties(dp_management_transport_tests dp_management_transport_tcp_untrusted_tests
    dp_management_raw_first_page_tests release_transition_tests observation_continuity_tests
    shared_management_channel_tests PROPERTIES
    ENVIRONMENT "LD_LIBRARY_PATH=$<TARGET_FILE_DIR:dummy_ros_control::dummy_command_channel>:$<TARGET_FILE_DIR:dummy_ros_control::dummy_guarded_zenbedded_system>:$ENV{LD_LIBRARY_PATH}"
    TIMEOUT 25)
  set_tests_properties(dp_management_raw_first_page_tests PROPERTIES
    ENVIRONMENT "LD_LIBRARY_PATH=${CMAKE_BINARY_DIR}/../dummy_ros_control:$<TARGET_FILE_DIR:dummy_ros_control::dummy_command_channel>:$<TARGET_FILE_DIR:dummy_ros_control::dummy_guarded_zenbedded_system>:$ENV{LD_LIBRARY_PATH}")
  set_tests_properties(shared_management_channel_tests PROPERTIES TIMEOUT 15)
  find_package(dummy_ros_control REQUIRED)
  find_package(controller_interface REQUIRED)
  find_package(joint_trajectory_controller REQUIRED)
  find_package(trajectory_msgs REQUIRED)
  foreach(test_name IN ITEMS session_action_tests cancel_barrier_tests management_client_tests
      complete_motion_tests first_sample_handshake_tests device_action_engine_tests wire_management_tests
      native_action_gate_tests bootstrap_coordinator_tests)
    add_executable(${test_name} test/${test_name}.cpp)
    target_include_directories(${test_name} PRIVATE include)
    target_link_libraries(${test_name} dummy_device_action)
    add_test(NAME ${test_name} COMMAND ${test_name})
  endforeach()
  # 该测试用assert执行必要的状态推进；Release配置也必须执行这些断言。
  target_compile_options(bootstrap_coordinator_tests PRIVATE -UNDEBUG)
  target_link_libraries(native_action_gate_tests dummy_native_goal_validation)
  add_executable(native_wait_policy_tests test/native_wait_policy_tests.cpp)
  target_link_libraries(native_wait_policy_tests dummy_device_action)
  add_test(NAME native_wait_policy_tests COMMAND native_wait_policy_tests)
  add_executable(guarded_entry_tests test/guarded_entry_tests.cpp)
  target_link_libraries(guarded_entry_tests dummy_ros_control::dummy_guarded_joint_trajectory_controller)
  ament_target_dependencies(guarded_entry_tests controller_interface joint_trajectory_controller trajectory_msgs rclcpp_action)
  add_test(NAME guarded_topic_entry_tests COMMAND guarded_entry_tests MOV13)
  add_test(NAME guarded_action_entry_tests COMMAND guarded_entry_tests MOV14)
  add_test(NAME guarded_digest_binding_tests COMMAND guarded_entry_tests MOV16)
  add_executable(guarded_execution_tests test/guarded_execution_tests.cpp)
  target_link_libraries(guarded_execution_tests dummy_ros_control::dummy_guarded_joint_trajectory_controller)
  ament_target_dependencies(guarded_execution_tests controller_interface joint_trajectory_controller trajectory_msgs)
  add_test(NAME guarded_execution_tests COMMAND guarded_execution_tests)
  set(DUMMY_TEST_AMENT_PATH "$ENV{AMENT_PREFIX_PATH}")
  set_tests_properties(guarded_topic_entry_tests guarded_action_entry_tests guarded_digest_binding_tests guarded_execution_tests PROPERTIES
    ENVIRONMENT "LD_LIBRARY_PATH=$<TARGET_FILE_DIR:dummy_ros_control::dummy_command_channel>:$ENV{LD_LIBRARY_PATH};AMENT_PREFIX_PATH=${DUMMY_TEST_AMENT_PATH}")
  add_executable(update_generation_tests test/update_generation_tests.cpp)
  target_link_libraries(update_generation_tests dummy_ros_control::dummy_command_channel)
  add_test(NAME update_generation_tests COMMAND update_generation_tests)
  add_executable(direct_hold_handoff_tests test/direct_hold_handoff_tests.cpp)
  target_link_libraries(direct_hold_handoff_tests dummy_dp_management dummy_native_goal_validation dummy_device_profile
    dummy_ros_control::dummy_guarded_joint_trajectory_controller dummy_ros_control::dummy_command_channel
    dummy_goal_timeline)
  ament_target_dependencies(direct_hold_handoff_tests controller_manager control_msgs rclcpp rclcpp_action
    controller_interface joint_trajectory_controller trajectory_msgs std_srvs)
  add_test(NAME direct_hold_handoff_tests COMMAND direct_hold_handoff_tests)
  add_test(NAME gate_control_slow_stderr
    COMMAND ${Python3_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/../dummy_ros_control/test/run_slow_stderr_fixture.py"
      "$<TARGET_FILE:direct_hold_handoff_tests>")
  set_tests_properties(gate_control_slow_stderr PROPERTIES TIMEOUT 45 LABELS "component_only;offline")
  add_executable(device_lease_transition_tests test/device_lease_transition_tests.cpp)
  target_link_libraries(device_lease_transition_tests dummy_dp_management dummy_native_goal_validation dummy_device_profile
    dummy_ros_control::dummy_guarded_joint_trajectory_controller dummy_ros_control::dummy_command_channel
    dummy_goal_timeline)
  ament_target_dependencies(device_lease_transition_tests controller_manager control_msgs rclcpp rclcpp_action
    controller_interface joint_trajectory_controller trajectory_msgs std_srvs)
  add_test(NAME device_lease_transition_tests COMMAND device_lease_transition_tests)
  add_executable(queued_renew_completion_tests test/queued_renew_completion_tests.cpp)
  target_link_libraries(queued_renew_completion_tests dummy_dp_management dummy_native_goal_validation dummy_device_profile
    dummy_ros_control::dummy_guarded_joint_trajectory_controller dummy_ros_control::dummy_command_channel dummy_goal_timeline)
  ament_target_dependencies(queued_renew_completion_tests controller_manager control_msgs rclcpp rclcpp_action
    controller_interface joint_trajectory_controller trajectory_msgs std_srvs)
  target_compile_options(queued_renew_completion_tests PRIVATE -UNDEBUG)
  foreach(mode IN ITEMS positive sent_complete not_frozen wrong_boot wrong_session wrong_reference wrong_motion wrong_management wrong_profile wrong_mask wrong_device wrong_client missing_ack old_ack expired withdraw_positive withdraw_explicit withdraw_started withdraw_owns_busy withdraw_nonrenew withdraw_not_reserved withdraw_not_running withdraw_actual_inflight withdraw_stale_state withdraw_wrong_run withdraw_status_lag)
    add_test(NAME queued_renew_completion_${mode} COMMAND queued_renew_completion_tests ${mode})
  endforeach()

  add_executable(queued_renew_cancel_tests test/queued_renew_cancel_tests.cpp)
  target_link_libraries(queued_renew_cancel_tests dummy_dp_management dummy_native_goal_validation dummy_device_profile
    dummy_ros_control::dummy_guarded_joint_trajectory_controller dummy_ros_control::dummy_command_channel dummy_goal_timeline)
  ament_target_dependencies(queued_renew_cancel_tests controller_manager control_msgs rclcpp rclcpp_action
    controller_interface joint_trajectory_controller trajectory_msgs std_srvs)
  target_compile_options(queued_renew_cancel_tests PRIVATE -UNDEBUG)
  foreach(mode IN ITEMS positive wrong_boot wrong_session wrong_ref wrong_motion wrong_mgmt wrong_profile wrong_mask wrong_device wrong_client not_cancel abort request_missing no_cancel lost_authorization expired started actual_inflight owns_busy nonrenew not_reserved not_running)
    add_test(NAME queued_renew_cancel_${mode} COMMAND queued_renew_cancel_tests ${mode})
    set_tests_properties(queued_renew_cancel_${mode} PROPERTIES LABELS "component_only;offline")
  endforeach()

  add_executable(inflight_renew_cancel_tests test/inflight_renew_cancel_tests.cpp)
  target_link_libraries(inflight_renew_cancel_tests dummy_device_action dummy_dp_management dummy_native_goal_validation dummy_device_profile
    dummy_ros_control::dummy_guarded_joint_trajectory_controller dummy_ros_control::dummy_command_channel dummy_goal_timeline)
  ament_target_dependencies(inflight_renew_cancel_tests controller_manager control_msgs rclcpp rclcpp_action
    controller_interface joint_trajectory_controller trajectory_msgs std_srvs)
  target_compile_options(inflight_renew_cancel_tests PRIVATE -UNDEBUG)
  foreach(mode IN ITEMS positive wrong_boot wrong_session wrong_ref wrong_motion wrong_management wrong_profile wrong_mask wrong_device wrong_client expired failed_renew invalidate no_cancel abort guard_boot guard_session guard_ref guard_motion guard_management guard_profile guard_mask guard_operation waiting_abort)
    add_test(NAME inflight_renew_cancel_${mode} COMMAND inflight_renew_cancel_tests ${mode})
    set_tests_properties(inflight_renew_cancel_${mode} PROPERTIES TIMEOUT 15 LABELS "component_only;offline")
  endforeach()

  add_executable(review_boundary_tests test/review_boundary_tests.cpp)
  target_link_libraries(review_boundary_tests dummy_device_action)
  add_executable(cancel_query_cadence_tests test/cancel_query_cadence_tests.cpp)
  target_link_libraries(cancel_query_cadence_tests dummy_device_action)
  foreach(_cadence_mode cancel actual_late renew begin no_terminal wrong_identity)
    add_test(NAME wire_cancel_query_cadence_${_cadence_mode} COMMAND cancel_query_cadence_tests ${_cadence_mode})
    set_tests_properties(wire_cancel_query_cadence_${_cadence_mode} PROPERTIES LABELS "component_only;offline;source_model")
  endforeach()
  add_test(NAME wire_release_query_supervision_budget COMMAND wire_management_tests release_query_supervision_budget)
  add_test(NAME wire_release_query_backoff_guards COMMAND wire_management_tests release_query_backoff_guards)
  add_test(NAME wire_release_committed_status_gap_counterexample COMMAND wire_management_tests release_committed_status_gap)
  add_test(NAME wire_release_pending_observation_grace COMMAND wire_management_tests release_pending_observation_grace)
  add_test(NAME wire_release_pending_observation_grace_guards COMMAND wire_management_tests release_pending_observation_grace_guards)
  add_test(NAME wire_running_renewal_complete COMMAND wire_management_tests running_renewal_complete)
  add_test(NAME wire_running_renewal_complete_boundaries COMMAND wire_management_tests running_renewal_complete_boundaries)
  add_test(NAME wire_running_renewal_strict_identity COMMAND wire_management_tests running_renewal_strict_identity)
  foreach(scenario IN ITEMS enum deadline tolerance unused_nan hold_mask)
    add_test(NAME review_${scenario} COMMAND review_boundary_tests ${scenario})
  endforeach()
  foreach(scenario IN ITEMS device profile mask hold_mask)
    add_test(NAME wire_${scenario}_boundary COMMAND wire_management_tests ${scenario})
  endforeach()
endif()
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../../contracts/dummy_robot_v2" DESTINATION share/${PROJECT_NAME}/contracts)
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../../contracts/dummy_robot_v2_application_hold_no_load_candidate"
  DESTINATION share/${PROJECT_NAME}/contracts)
install(FILES "${DUMMY_WIRE_INCLUDE}/dp_dummy_robot_v2.hpp" DESTINATION include)
install(TARGETS dummy_device_action dummy_native_goal_validation dummy_dp_management dummy_device_profile
  dummy_goal_timeline EXPORT export_dummy_device_action ARCHIVE DESTINATION lib)
ament_export_targets(export_dummy_device_action)
ament_export_dependencies(dummy_ros_control control_msgs yaml-cpp OpenSSL)
ament_package()

# @file src/dummy_motion_client/CMakeLists.txt
# @brief 装配公共动作守卫、设备管理传输、profile和分层回归。
# @details 先解析同版DPSDK wire与ROS依赖，再构建纯逻辑静态库和两个节点，
# BUILD_TESTING时注册组件/软件peer用例，最后安装并导出依赖。
# 配置/构建不会自动运行测试；real_dpsdk_peer标签仍需查看其实际SIL executable和端点。

cmake_minimum_required(VERSION 3.16)
project(dummy_motion_client)
# @par 依赖发现
# ament负责ROS包导出；消息/Action/controller依赖提供类型与控制接口，OpenSSL用于摘要，yaml-cpp读profile。
# 这些find_package要求依赖已在当前underlay/前缀可见，不会在这里自动下载或编译SDK。
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
find_package(dpsdk_interfaces REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
# 协议/Policy/通信 ABI 由 control 导出的同一安装 SDK 递归提供。
set(DPSDK_ROOT "$ENV{DPSDK_ROOT}" CACHE PATH "离线源码模型回归所用DPSDK根")
find_package(DPSDKHostSDK CONFIG REQUIRED)
get_target_property(DUMMY_WIRE_INCLUDE dp_protocol::selected INTERFACE_INCLUDE_DIRECTORIES)
# @par 公共逻辑库
# action_gate/permit/bootstrap/management_client组成不含ROS节点main的逻辑静态库。
# PUBLIC C++17/公开头/协议及加密依赖传给消费者；跨包公开头的control接口亦必须PUBLIC。
# BUILD_INTERFACE用于源码构建，INSTALL_INTERFACE用于安装后的消费者。
add_library(dummy_device_action STATIC
  src/action_gate.cpp src/bench_permit.cpp src/bootstrap_coordinator.cpp src/management_client.cpp
  src/manual_reference_coordinator.cpp)
target_link_libraries(dummy_device_action PUBLIC OpenSSL::Crypto DPSDKHostSDK::dummy_protocol DPSDKHostSDK::dummy_manual_reference
  dummy_ros_control::dummy_goal_registration_gate)
target_compile_features(dummy_device_action PUBLIC cxx_std_17)
target_include_directories(dummy_device_action PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
# Native守卫转发仿真Action；device_action_gate在下方走设备管理/受控controller链路。
# 两个入口是不同后端角色，不能因都能接Action就将仿真视为真实设备许可。
add_executable(native_action_gate src/native_action_gate.cpp)
# 设备管理适配依赖control导出的651字节PIC通信布局及guarded system，
# PUBLIC宏必须与依赖库ABI一致；profile库借yaml-cpp/摘要核输入，不能单凭解析成功授权。
add_library(dummy_dp_management STATIC src/dp_management_transport.cpp)
target_link_libraries(dummy_dp_management PUBLIC dummy_device_action dummy_ros_control::dummy_dp_comm
  dummy_ros_control::dummy_guarded_zenbedded_system)
add_library(dummy_device_profile STATIC src/device_profile.cpp src/manual_panel_profile.cpp)
target_link_libraries(dummy_device_profile PUBLIC dummy_device_action PRIVATE yaml-cpp OpenSSL::Crypto)
target_compile_definitions(dummy_device_profile PRIVATE
  DUMMY_INSTALLED_ROBOT_MANIFEST_SHA256="${DPSDKHostSDK_ROBOT_MANIFEST_SHA256}")
add_library(dummy_native_goal_validation STATIC src/native_goal_validation.cpp)
target_link_libraries(dummy_native_goal_validation dummy_device_action)
ament_target_dependencies(dummy_native_goal_validation control_msgs)
# 独立timeline库维护时序/时钟一致性，单独导出头与C++17；不含传输或设备main。
add_library(dummy_goal_timeline STATIC src/goal_timeline.cpp)
target_include_directories(dummy_goal_timeline PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(dummy_goal_timeline PUBLIC cxx_std_17)
target_link_libraries(native_action_gate dummy_native_goal_validation)
target_compile_features(native_action_gate PRIVATE cxx_std_17)
ament_target_dependencies(native_action_gate control_msgs rclcpp rclcpp_action)
# 设备Action节点链接管理传输、目标校验、profile、controller/command channel与timeline。
# ament_target_dependencies提供ROS编译/链接依赖；安装到lib/包名以供ros2 run查找。
add_library(dummy_panel_feedback STATIC src/panel_feedback.cpp)
target_link_libraries(dummy_panel_feedback PUBLIC dummy_dp_management)
ament_target_dependencies(dummy_panel_feedback PUBLIC dpsdk_interfaces)
add_executable(device_action_gate src/device_action_gate.cpp)
target_link_libraries(device_action_gate dummy_panel_feedback dummy_dp_management dummy_native_goal_validation dummy_device_profile
  dummy_ros_control::dummy_guarded_joint_trajectory_controller dummy_ros_control::dummy_command_channel
  dummy_goal_timeline)
ament_target_dependencies(device_action_gate controller_manager control_msgs rclcpp rclcpp_action
  controller_interface joint_trajectory_controller trajectory_msgs std_srvs dpsdk_interfaces)
install(TARGETS device_action_gate DESTINATION lib/${PROJECT_NAME})
install(TARGETS native_action_gate DESTINATION lib/${PROJECT_NAME})
add_library(dummy_manual_ros_baseline STATIC src/manual_ros_baseline.cpp)
target_compile_features(dummy_manual_ros_baseline PUBLIC cxx_std_17)
target_include_directories(dummy_manual_ros_baseline PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
add_executable(manual_ros_baseline_node src/manual_ros_baseline_node.cpp)
target_link_libraries(manual_ros_baseline_node dummy_manual_ros_baseline OpenSSL::Crypto DPSDKHostSDK::dummy_manual_reference
  dummy_ros_control::dummy_guarded_joint_trajectory_controller)
ament_target_dependencies(manual_ros_baseline_node rclcpp rclcpp_action std_srvs control_msgs
  trajectory_msgs controller_manager_msgs moveit_msgs shape_msgs dpsdk_interfaces
  joint_trajectory_controller controller_interface dummy_ros_control)
install(TARGETS manual_ros_baseline_node DESTINATION lib/${PROJECT_NAME})
add_executable(manual_ros_recovery_node src/manual_ros_recovery_node.cpp)
target_link_libraries(manual_ros_recovery_node dummy_dp_management dummy_device_profile)
ament_target_dependencies(manual_ros_recovery_node rclcpp)
install(TARGETS manual_ros_recovery_node DESTINATION lib/${PROJECT_NAME})
# @par 测试装配
# 仅BUILD_TESTING时创建下列target/CTest。component_only/offline是测试分类，
# 不自动证明所有分支已执行；-UNDEBUG只对显式列出的target恢复assert，不能泛化到全部target。
if(BUILD_TESTING)
  add_executable(manual_ros_recovery_tests test/manual_ros_recovery_tests.cpp)
  target_link_libraries(manual_ros_recovery_tests dummy_device_action)
  target_compile_options(manual_ros_recovery_tests PRIVATE -UNDEBUG)
  add_test(NAME manual_ros_recovery_tests COMMAND manual_ros_recovery_tests)
  add_executable(panel_feedback_tests test/panel_feedback_tests.cpp)
  target_link_libraries(panel_feedback_tests dummy_panel_feedback)
  ament_target_dependencies(panel_feedback_tests dpsdk_interfaces)
  target_compile_options(panel_feedback_tests PRIVATE -UNDEBUG)
  add_test(NAME panel_feedback_tests COMMAND panel_feedback_tests)
  add_executable(manual_panel_profile_tests test/manual_panel_profile_tests.cpp)
  target_link_libraries(manual_panel_profile_tests dummy_device_profile)
  target_compile_options(manual_panel_profile_tests PRIVATE -UNDEBUG)
  add_test(NAME manual_panel_profile_requires_external_pin COMMAND manual_panel_profile_tests)
  # 真实部署由执行者显式提供，离线默认只注册由环境指定的固定部署。
  if(DEFINED ENV{DUMMY_MANUAL_DEPLOYMENT_ROOT} AND DEFINED ENV{DUMMY_MANUAL_DEPLOYMENT_SHA256})
    add_test(NAME manual_panel_profile_tests COMMAND manual_panel_profile_tests
      "$ENV{DUMMY_MANUAL_DEPLOYMENT_ROOT}" "$ENV{DUMMY_MANUAL_DEPLOYMENT_SHA256}"
      "${DPSDKHostSDK_ROBOT_BUNDLE_DIR}")
  endif()
  add_executable(manual_reference_coordinator_tests test/manual_reference_coordinator_tests.cpp)
  target_link_libraries(manual_reference_coordinator_tests dummy_device_action)
  target_compile_options(manual_reference_coordinator_tests PRIVATE -UNDEBUG)
  add_test(NAME manual_reference_coordinator_tests COMMAND manual_reference_coordinator_tests)
  add_executable(manual_ros_baseline_tests test/manual_ros_baseline_tests.cpp)
  target_link_libraries(manual_ros_baseline_tests dummy_manual_ros_baseline)
  target_compile_options(manual_ros_baseline_tests PRIVATE -UNDEBUG)
  add_test(NAME manual_ros_baseline_tests COMMAND manual_ros_baseline_tests)
# 预算与故障收尾组件均使用当前安装SDK。
  add_executable(limited_goal_budget_tests test/limited_goal_budget_tests.cpp)
  target_include_directories(limited_goal_budget_tests PRIVATE include)
  target_compile_options(limited_goal_budget_tests PRIVATE -UNDEBUG)
  add_test(NAME limited_goal_budget_tests COMMAND limited_goal_budget_tests)
  set_tests_properties(limited_goal_budget_tests PROPERTIES LABELS "component_only;offline")
  find_package(Threads REQUIRED)
  # 当前合成故障页验证收尾业务，不重放废弃wire2.4抓取。
  add_executable(fault_cleanup_receipt_tests test/fault_cleanup_receipt_tests.cpp)
  target_include_directories(fault_cleanup_receipt_tests PRIVATE include)
  target_compile_options(fault_cleanup_receipt_tests PRIVATE -UNDEBUG)
  target_link_libraries(fault_cleanup_receipt_tests PRIVATE dummy_dp_management)
  add_test(NAME fault_cleanup_verified_receipt_and_current_observation COMMAND fault_cleanup_receipt_tests)
  set_tests_properties(fault_cleanup_verified_receipt_and_current_observation PROPERTIES LABELS "component_only;offline" TIMEOUT 15)
# 清理/目标timeline用当前wire与Host线程；每个add_test以真实目标或生成器表达式定位可执行程序。
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
# PTY桥和c05_action_peer是供Python驱动的辅助进程，创建target本身不把它注册为独立CTest。
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
  foreach(mode IN ITEMS denied stale shutdown unconfirmed retained wrongref status_cadence status_qualification_cadence state_stale_status_live can_stale status_stale calibration_motion release_retry release_unavailable planning_duplicate_nodes planning_duplicate_servers planning_private_scene planning_unrelated_scene planning_bad_private_suffix planning_wrong_scene_namespace renewal_retry renewal_wrong_uuid renewal_no_receipt renewal_owner_drift renewal_pending consumed_permit consumed_permit_denied consumed_owner_drift scene_normalized scene_equivalent_local scene_wrong_pose scene_wrong_dimensions scene_wrong_frame scene_wrong_orientation scene_full_positive_origin scene_full_negative_origin scene_unrelated_plan_error scene_restore_plan_failure scene_clear_plan_failure scene_blocked_plan_success)
    add_test(NAME manual_ros_baseline_e2e_${mode} COMMAND ${Python3_EXECUTABLE}
      "${CMAKE_CURRENT_SOURCE_DIR}/test/manual_ros_baseline_e2e_tests.py"
      "$<TARGET_FILE:manual_ros_baseline_node>" ${mode})
    set_tests_properties(manual_ros_baseline_e2e_${mode} PROPERTIES TIMEOUT 35
      LABELS "component_only;synthetic_peer;ros_baseline")
  endforeach()
  foreach(mode IN ITEMS external stale shutdown timeout)
    add_test(NAME manual_ros_control_e2e_${mode} COMMAND ${Python3_EXECUTABLE}
      "${CMAKE_CURRENT_SOURCE_DIR}/test/manual_ros_control_e2e_tests.py"
      "$<TARGET_FILE:manual_ros_baseline_node>" ${mode})
    set_tests_properties(manual_ros_control_e2e_${mode} PROPERTIES TIMEOUT 35
      LABELS "component_only;synthetic_peer;ros_manual_control")
  endforeach()
  # 仅检查正式镜像身份；不得据此授予DeviceProfile运行/运动权限。
# 正式bundle的身份完整性正例不授予DeviceProfile运动权；镜像测试核源码合同的规范副本。
  add_executable(protocol_release_identity_tests test/protocol_release_identity_tests.cpp)
  target_link_libraries(protocol_release_identity_tests PRIVATE dummy_device_profile)
  add_test(NAME current_protocol_release_identity COMMAND protocol_release_identity_tests
    "${DPSDKHostSDK_ROBOT_BUNDLE_DIR}")
  set_tests_properties(current_protocol_release_identity PROPERTIES LABELS "component_only;offline;installed_sdk")
  get_filename_component(_dummy_ros_install_root "${CMAKE_INSTALL_PREFIX}/.." ABSOLUTE)
  add_test(NAME installed_sdk_source_hidden_consumer COMMAND "${Python3_EXECUTABLE}"
    "${CMAKE_CURRENT_SOURCE_DIR}/test/installed_sdk_consumer.py"
    --sdk-prefix "${DPSDK_HOST_SDK_PREFIX}" --ros-prefix "${_dummy_ros_install_root}"
    --sdk-source "${DPSDK_ROOT}" --ros-source "${CMAKE_CURRENT_SOURCE_DIR}/../..")
  set_tests_properties(installed_sdk_source_hidden_consumer PROPERTIES
    TIMEOUT 90 LABELS "component_only;offline;installed_sdk")
  add_executable(formal_bundle_identity_probe test/formal_bundle_identity_probe.cpp)
  target_link_libraries(formal_bundle_identity_probe PRIVATE dummy_device_profile)
  add_test(NAME formal_bundle_identity_integrity COMMAND formal_bundle_identity_probe
    "${DPSDKHostSDK_ROBOT_BUNDLE_DIR}")
  add_test(NAME formal_contract_canonical_mirror_integrity COMMAND ${Python3_EXECUTABLE} -m pytest -q
    "${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/dummy/tests/test_formal_contract_mirror.py")
  set_tests_properties(formal_bundle_identity_integrity formal_contract_canonical_mirror_integrity
    PROPERTIES TIMEOUT 15 LABELS "component_only;offline;source_integrity"
    ENVIRONMENT "DPSDK_ROOT=${DPSDK_ROOT};DPSDK_HOST_SDK_PREFIX=${DPSDK_HOST_SDK_PREFIX};PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/../..;PYTEST_DISABLE_PLUGIN_AUTOLOAD=1")
# 固定可信SIL夹具路径用于软件测试；APPLICATION_HOLD用专用wire source生成相应输入。
# 这些fixture不会被安装成真实设备已核准配置。
  set(DUMMY_SIL_FIXTURE_BUNDLE "${DPSDKHostSDK_ROBOT_BUNDLE_DIR}")
  add_test(NAME device_profile_tests COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/run_device_profile_fixture.py"
    "$<TARGET_FILE:device_profile_tests>"
    "${DUMMY_SIL_FIXTURE_BUNDLE}")
  add_executable(application_hold_candidate_profile_tests test/application_hold_candidate_profile_tests.cpp)
  target_compile_options(application_hold_candidate_profile_tests PRIVATE -UNDEBUG)
  target_link_libraries(application_hold_candidate_profile_tests dummy_device_profile)
  add_test(NAME application_hold_candidate_profile_tests COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/run_application_hold_candidate_profile.py"
    "$<TARGET_FILE:application_hold_candidate_profile_tests>" "${DPSDK_ROOT}"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../contracts/dummy_robot_v2_application_hold_no_load_candidate"
    "${DPSDKHostSDK_ROBOT_BUNDLE_DIR}")
# Python fixture/activation wiring是离线组件层；public probe会启动本包节点做ROS Action验证，
# 需区分离线结构断言、合成peer、真实SDK软件peer三个层次。
  add_test(NAME device_action_e2e_fixture
    COMMAND ${Python3_EXECUTABLE} -m pytest -q
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_device_action_e2e_fixture.py)
  set_tests_properties(device_action_e2e_fixture PROPERTIES
    ENVIRONMENT "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1;DPSDK_HOST_SDK_PREFIX=${DPSDK_HOST_SDK_PREFIX}" LABELS "component_only;offline")
  add_test(NAME device_action_activation_wiring
    COMMAND ${Python3_EXECUTABLE} -m pytest -q
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_device_action_activation_wiring.py)
  set_tests_properties(device_action_activation_wiring PROPERTIES
    ENVIRONMENT "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1;DPSDK_HOST_SDK_PREFIX=${DPSDK_HOST_SDK_PREFIX}" LABELS "component_only;offline")
  add_test(NAME device_action_public_probe COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/device_action_public_probe.py $<TARGET_FILE:device_action_gate>
    "${DUMMY_SIL_FIXTURE_BUNDLE}")
  set_tests_properties(device_action_public_probe PROPERTIES TIMEOUT 30 LABELS "component_only;ros_action"
    ENVIRONMENT "DPSDK_HOST_SDK_PREFIX=${DPSDK_HOST_SDK_PREFIX}")
  # 两种SIL测试时钟角色由调用方显式绑定，不探测旧build或复用面板角色。
# 两个peer路径必须由调用方显式提供；空值不会自动探测某个旧build。
# 下面传给e2e脚本的real/real_smoke指实际SDK SIL peer角色，不等于真实机械臂测试。
  set(DUMMY_D08_J1_PEER_EXECUTABLE "$ENV{DUMMY_D08_J1_PEER_EXECUTABLE}" CACHE FILEPATH
    "候选J1 10/10ms SIL peer，必须显式提供")
  set(DUMMY_D08_SIX_AXIS_PEER_EXECUTABLE "$ENV{DUMMY_D08_SIX_AXIS_PEER_EXECUTABLE}" CACHE FILEPATH
    "候选six 10/20ms SIL peer，必须显式提供")
  add_test(NAME device_action_e2e_probe COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/device_action_e2e_probe.py $<TARGET_FILE:device_action_gate>
    "${DUMMY_D08_J1_PEER_EXECUTABLE}"
    ${DUMMY_SIL_FIXTURE_BUNDLE} real)
  add_test(NAME device_action_smoke_probe COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/device_action_e2e_probe.py $<TARGET_FILE:device_action_gate>
    "${DUMMY_D08_SIX_AXIS_PEER_EXECUTABLE}"
    ${DUMMY_SIL_FIXTURE_BUNDLE} real_smoke)
  add_test(NAME device_action_six_axis_smoke_probe COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/device_action_e2e_probe.py $<TARGET_FILE:device_action_gate>
    "${DUMMY_D08_SIX_AXIS_PEER_EXECUTABLE}"
    ${DUMMY_SIL_FIXTURE_BUNDLE} real_six_axis_smoke)
  add_test(NAME device_motion_unsupported_probe COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/device_action_e2e_probe.py $<TARGET_FILE:device_action_gate>
    $<TARGET_FILE:c05_action_peer> ${DUMMY_SIL_FIXTURE_BUNDLE} unsupported)
  add_test(NAME device_motion_supported_probe COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/device_action_e2e_probe.py $<TARGET_FILE:device_action_gate>
    $<TARGET_FILE:c05_action_peer> ${DUMMY_SIL_FIXTURE_BUNDLE})
  add_test(NAME device_motion_six_axis_smoke_probe COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/device_action_e2e_probe.py $<TARGET_FILE:device_action_gate>
    $<TARGET_FILE:c05_action_peer> ${DUMMY_SIL_FIXTURE_BUNDLE} six_axis_smoke)
# 各场景分别限制总运行时、串行PTY传输和State周期，设置标签供有选择的CTest运行。
# LABELS只是分类，不代替目标源码或实际执行端点审计。
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
# 共享PTY桥路径由TARGET_FILE在生成阶段解析；fixture额外接收合成peer、control库目录与DPSDK根。
  set_property(TEST device_action_e2e_fixture device_action_e2e_probe device_action_smoke_probe
    device_action_six_axis_smoke_probe device_motion_unsupported_probe
    device_motion_supported_probe device_motion_six_axis_smoke_probe
    APPEND PROPERTY ENVIRONMENT "DUMMY_SERIAL_BRIDGE_EXECUTABLE=$<TARGET_FILE:serial_pty_bridge>"
      "DPSDK_HOST_SDK_PREFIX=${DPSDK_HOST_SDK_PREFIX}")
  set_property(TEST device_action_e2e_fixture APPEND PROPERTY ENVIRONMENT
    "DUMMY_C05_PEER_EXECUTABLE=$<TARGET_FILE:c05_action_peer>"
    "DUMMY_ROS_CONTROL_LIBRARY_DIR=$<TARGET_FILE_DIR:dummy_ros_control::dummy_guarded_zenbedded_system>"
    "DPSDK_ROOT=${DPSDK_ROOT}")
# 目标校验、管理传输和wire minor为独立组件目标；同一个二进制可按mode注册多条CTest。
  add_executable(device_goal_validation_tests test/device_goal_validation_tests.cpp)
  target_link_libraries(device_goal_validation_tests dummy_native_goal_validation)
  ament_target_dependencies(device_goal_validation_tests control_msgs)
  add_test(NAME device_goal_validation_tests COMMAND device_goal_validation_tests)
  add_executable(dp_management_transport_tests test/dp_management_transport_tests.cpp)
  target_link_libraries(dp_management_transport_tests dummy_dp_management)
  ament_target_dependencies(dp_management_transport_tests hardware_interface)
  add_test(NAME dp_management_transport_tests COMMAND dp_management_transport_tests)
  add_test(NAME dp_management_attempt_evidence_tests COMMAND dp_management_transport_tests attempt_evidence)
  add_test(NAME dp_management_attempt_evidence_overflow_tests COMMAND dp_management_transport_tests attempt_evidence_overflow)
  set_tests_properties(dp_management_attempt_evidence_tests dp_management_attempt_evidence_overflow_tests
    PROPERTIES TIMEOUT 25 LABELS "component_only;synthetic_peer")
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
# 准入重试覆盖队列、取消、停止/重启、过期等模式，各5s上限；不是给生产请求增加重试时间。
  add_executable(shared_admission_retry_tests test/shared_admission_retry_tests.cpp)
  target_link_libraries(shared_admission_retry_tests dummy_dp_management)
  target_compile_options(shared_admission_retry_tests PRIVATE -UNDEBUG)
  foreach(mode IN ITEMS select grant capture queued_other wrong_size invalid_codec not_ready cancel destroy stop stop_restart unready expired poll_queued_other)
    add_test(NAME shared_admission_retry_${mode} COMMAND shared_admission_retry_tests ${mode})
    set_tests_properties(shared_admission_retry_${mode} PROPERTIES LABELS "component_only;offline" TIMEOUT 5)
  endforeach()

# 慢stderr测试由公共Python替身运行对应组件，检查日志阻塞的作用域；不会连接物理串口。
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
# 共享管理通道与传输测试显式设置运行库搜索目录；后续set_tests_properties会覆盖同名属性，
# 因此raw_first_page另补构建目录、shared_channel再收紧到15s。
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
# 这组逻辑测试统一链接dummy_device_action，每个源码各自注册；native测试下方再追加目标校验依赖。
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
# guarded controller的topic/action/digest入口与执行回归使用control导出的插件实现。
# 环境显式带command库和AMENT前缀，确保插件发现；并非绕过guard调用内部JTC。
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
# generation隔离后，direct handoff/lease转移/排队续租目标链接同一真实Host逻辑链；
# 这些是受控Host组件夹具，不构成设备CAN/制动资格。
  add_executable(update_generation_tests test/update_generation_tests.cpp)
  target_link_libraries(update_generation_tests dummy_ros_control::dummy_command_channel)
  add_test(NAME update_generation_tests COMMAND update_generation_tests)
  add_executable(direct_hold_handoff_tests test/direct_hold_handoff_tests.cpp)
  target_link_libraries(direct_hold_handoff_tests dummy_panel_feedback dummy_dp_management dummy_native_goal_validation dummy_device_profile
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
  target_link_libraries(device_lease_transition_tests dummy_panel_feedback dummy_dp_management dummy_native_goal_validation dummy_device_profile
    dummy_ros_control::dummy_guarded_joint_trajectory_controller dummy_ros_control::dummy_command_channel
    dummy_goal_timeline)
  ament_target_dependencies(device_lease_transition_tests controller_manager control_msgs rclcpp rclcpp_action
    controller_interface joint_trajectory_controller trajectory_msgs std_srvs)
  add_test(NAME device_lease_transition_tests COMMAND device_lease_transition_tests)
# 完成阶段的排队续租：正例、完整身份错配、ACK/过期、撤销状态等分别注册mode；-UNDEBUG保留必要断言推进。
  add_executable(queued_renew_completion_tests test/queued_renew_completion_tests.cpp)
  target_link_libraries(queued_renew_completion_tests dummy_panel_feedback dummy_dp_management dummy_native_goal_validation dummy_device_profile
    dummy_ros_control::dummy_guarded_joint_trajectory_controller dummy_ros_control::dummy_command_channel dummy_goal_timeline)
  ament_target_dependencies(queued_renew_completion_tests controller_manager control_msgs rclcpp rclcpp_action
    controller_interface joint_trajectory_controller trajectory_msgs std_srvs)
  target_compile_options(queued_renew_completion_tests PRIVATE -UNDEBUG)
  foreach(mode IN ITEMS positive sent_complete not_frozen wrong_boot wrong_session wrong_reference wrong_motion wrong_management wrong_profile wrong_mask wrong_device wrong_client missing_ack old_ack expired withdraw_positive withdraw_explicit withdraw_started withdraw_owns_busy withdraw_nonrenew withdraw_not_reserved withdraw_not_running withdraw_actual_inflight withdraw_stale_state withdraw_wrong_run withdraw_status_lag)
    add_test(NAME queued_renew_completion_${mode} COMMAND queued_renew_completion_tests ${mode})
  endforeach()

# 取消阶段仅对仍可撤回的排队续租操作检查身份/授权/期限；不能把这些模式泛称已撤回在途请求。
  add_executable(queued_renew_cancel_tests test/queued_renew_cancel_tests.cpp)
  target_link_libraries(queued_renew_cancel_tests dummy_panel_feedback dummy_dp_management dummy_native_goal_validation dummy_device_profile
    dummy_ros_control::dummy_guarded_joint_trajectory_controller dummy_ros_control::dummy_command_channel dummy_goal_timeline)
  ament_target_dependencies(queued_renew_cancel_tests controller_manager control_msgs rclcpp rclcpp_action
    controller_interface joint_trajectory_controller trajectory_msgs std_srvs)
  target_compile_options(queued_renew_cancel_tests PRIVATE -UNDEBUG)
  foreach(mode IN ITEMS positive wrong_boot wrong_session wrong_ref wrong_motion wrong_mgmt wrong_profile wrong_mask wrong_device wrong_client not_cancel abort request_missing no_cancel lost_authorization expired started actual_inflight owns_busy nonrenew not_reserved not_running)
    add_test(NAME queued_renew_cancel_${mode} COMMAND queued_renew_cancel_tests ${mode})
    set_tests_properties(queued_renew_cancel_${mode} PROPERTIES LABELS "component_only;offline")
  endforeach()

# 真正inflight续租取消另设目标及15s上限，覆盖错误身份、失败续租、失效、取消/中止和guard变化。
  add_executable(inflight_renew_cancel_tests test/inflight_renew_cancel_tests.cpp)
  target_link_libraries(inflight_renew_cancel_tests dummy_panel_feedback dummy_device_action dummy_dp_management dummy_native_goal_validation dummy_device_profile
    dummy_ros_control::dummy_guarded_joint_trajectory_controller dummy_ros_control::dummy_command_channel dummy_goal_timeline)
  ament_target_dependencies(inflight_renew_cancel_tests controller_manager control_msgs rclcpp rclcpp_action
    controller_interface joint_trajectory_controller trajectory_msgs std_srvs)
  target_compile_options(inflight_renew_cancel_tests PRIVATE -UNDEBUG)
  foreach(mode IN ITEMS positive wrong_boot wrong_session wrong_ref wrong_motion wrong_management wrong_profile wrong_mask wrong_device wrong_client expired failed_renew invalidate no_cancel abort guard_boot guard_session guard_ref guard_motion guard_management guard_profile guard_mask guard_operation waiting_abort)
    add_test(NAME inflight_renew_cancel_${mode} COMMAND inflight_renew_cancel_tests ${mode})
    set_tests_properties(inflight_renew_cancel_${mode} PROPERTIES TIMEOUT 15 LABELS "component_only;offline")
  endforeach()

# review边界、Query节拍和wire释放/续租回执各自注册；不同mode共享二进制仍是不同刺激。
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
# @par 安装与导出
# 安装公开头与静态库；当前wire/Policy和机器人业务包由同一SDK前缀导出。
# export_dummy_device_action导出target及接口依赖，ament_package生成包索引；候选目录存在不表示运动资格。
install(DIRECTORY include/ DESTINATION include)
# 现行机器人业务包由显式SDK前缀提供；ROS不保留独立wire镜像。
install(TARGETS dummy_device_action dummy_native_goal_validation dummy_dp_management dummy_device_profile dummy_panel_feedback
  dummy_goal_timeline EXPORT export_dummy_device_action ARCHIVE DESTINATION lib)
ament_export_targets(export_dummy_device_action)
ament_export_dependencies(dummy_ros_control control_msgs dpsdk_interfaces yaml-cpp OpenSSL DPSDKHostSDK DpProtocol)
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})

ament_package()

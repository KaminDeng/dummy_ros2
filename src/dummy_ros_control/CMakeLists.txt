# @file CMakeLists.txt
# @brief ROS控制器/硬件插件、位置控制传输与组件测试的构建入口。
# @details 配置顺序：发现ROS/OpenSSL→核验指定SDK及codec→核验三个PIC源→
# 装配可加载库→注册测试/独立探针→安装导出。651/18000是runtime容量角色，
# 不是电机数或USB MTU。配置、编译、测试与物理设备验证是不同层次。
# BUILD_TESTING只控制测试目标注册；没有在配置/构建时自动连接USB或使能机械臂。
cmake_minimum_required(VERSION 3.16)
project(dummy_ros_control)

# ROS依赖来自已source的underlay；OpenSSL用于摘要/轨迹身份计算，不在这里建立设备许可。
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(OpenSSL REQUIRED)

# 调用者可通过环境或-D指定源码根；verify脚本还要求明确的SDK安装前缀、同源PIC角色和codec身份。
set(DPSDK_ROOT "$ENV{DPSDK_ROOT}" CACHE PATH "DPSDK source root")
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/verify_sdk_inputs.cmake")
# ROS合同镜像用于部分历史/兼容测试；生产传输直接包含SDK发布的当前wire codec，不能混链旧镜像。
get_filename_component(ROS_WORKSPACE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(C03_PROTOTYPE_INCLUDE "${ROS_WORKSPACE_ROOT}/contracts/dummy_robot_v2/codec")
set(DUMMY_WIRE_INCLUDE "${DPSDK_ROOT}/applications/dummy_ref_control/contracts/dummy_robot_v2/codec")
if(NOT EXISTS "${DUMMY_WIRE_INCLUDE}/dp_dummy_robot_v2_dp_comm.hpp")
  message(FATAL_ERROR "Production device transport requires the published DPSDK wire-v1 codec")
endif()
# 这个INTERFACE目标只传播包含路径/宏/链接依赖；实际通信实现由下方dummy_dp_comm_pic编译。
add_library(dummy_dp_comm INTERFACE)
# 三个源路径及下一列表的SHA按下标配对。即便只加注释也会改变原始SHA；升级须先验证适配与正负例。
set(DUMMY_DP_COMM_SDK_SOURCES
  "${DPSDK_ROOT}/libraries/dp_comm/src/dp_comm_ros_like.cpp"
  "${DPSDK_ROOT}/libraries/dp_comm/src/dp_comm_ros_like_transport.cpp"
  "${DPSDK_ROOT}/platform/common/dp_comm_pico_port/dp_comm_port.cpp")
set(DUMMY_DP_COMM_SDK_HASHES
  3a171d250a45cf774a1b0bff90263c5d7feddd57e1c9c0bd0d471d557c61e856
  ec5dfd24763098b5ca79e3d32618afa69b88b62aaca7e574eaff303974bcd7f5
  6b0e6d369ae8b1d2fefd1cd877ded9d6bd7d4f73c1b035b06c363de6cf86732e)
# 配置阶段逐源检查存在性和原始摘要，任何一项漂移均FATAL_ERROR；不自动接受当前文件的新SHA。
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
# pluginlib. Compile the source-locked transport as PIC in this build.
# 当前runtime锁对应Node reset释放服务及最近deadline修复，均有实际负例/回归。
# 存储借用还包含SDK已提交的std::launder生命周期处理；只接受下列精确源码，
# 配套组件与独立篡改负例用于核对升级，不能以重新计算SHA代替回归。
# 动态插件不能装载带local-exec TLS重定位的普通预编译runtime；将锁定源重编为PIC静态库供共享库链接。
add_library(dummy_dp_comm_pic STATIC ${DUMMY_DP_COMM_SDK_SOURCES})
# Zenoh日志桥也是PIC；这里只编译本项目桥接实现，不修改第三方Zenoh源码。
add_library(dummy_dp_comm_log_shim STATIC src/zenoh_pico_log_shim.cpp)
set_target_properties(dummy_dp_comm_log_shim PROPERTIES POSITION_INDEPENDENT_CODE ON)
set_target_properties(dummy_dp_comm_pic PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(dummy_dp_comm_pic PUBLIC cxx_std_17)
# PUBLIC容量定义必须传给使用runtime对象的调用方；否则静态存储/消息大小约定会不一致。
target_compile_definitions(dummy_dp_comm_pic PUBLIC
  DP_COMM_RUNTIME_MAX_MESSAGE_BYTES=651 DP_COMM_RUNTIME_STORAGE_BYTES=18000)
# PRIVATE平台/OSAL开关复现该SDK角色的编译环境，不向所有ROS应用传播这些实现选择。
target_compile_definitions(dummy_dp_comm_pic PRIVATE
  DP_COMM_DEFAULT_LOCAL_ONLY=1 ZENOH_LINUX ZENOH_C_STANDARD=11 ZENOH_COMPILER_GCC ZENOH_LOG_INFO
  DPSDK_ALLOW_UNSAFE_LIBC_HEAP_FALLBACK=0 DPSDK_ENABLE_TLSF_HEAP=1
  DP_OSAL_ENABLE_CHRONO=1 DP_OSAL_ENABLE_CONDITION_VAR=1 DP_OSAL_ENABLE_MEMORY_MANAGER=1
  DP_OSAL_ENABLE_RW_LOCK=1 DP_OSAL_ENABLE_SPIN_LOCK=1 DP_OSAL_ENABLE_STATIC_ALLOC=0
  DP_OSAL_ENABLE_THREAD_POOL=1 DP_OSAL_ENABLE_THREAD_SNAPSHOT=1 DP_OSAL_ENABLE_TIMER=1)
# 混合使用选定SDK源码接口及同一SDK安装区的版本/Zenoh头；严禁从任意旧build借头凑齐。
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
  "${DPSDKHostSDK_VERSION_INCLUDE_DIR}"
  "${DPSDK_ROOT}/platform/common/zenoh_port"
  "${DPSDKHostSDK_ZENOH_INCLUDE_DIR}")
target_include_directories(dummy_dp_comm INTERFACE
  "${DUMMY_WIRE_INCLUDE}" "${DPSDK_ROOT}/libraries/dp_comm/include")
target_compile_definitions(dummy_dp_comm INTERFACE
  DP_COMM_RUNTIME_MAX_MESSAGE_BYTES=651 DP_COMM_RUNTIME_STORAGE_BYTES=18000)
# 导入SDK的OSAL、console、Zenoh归档并链接Host系统库；归档身份已由导出的SDK Config校验。
target_link_libraries(dummy_dp_comm INTERFACE
  dummy_dp_comm_pic
  DPSDKHostSDK::osal
  DPSDKHostSDK::console
  DPSDKHostSDK::zenoh
  dummy_dp_comm_log_shim
  rt pthread m dl)
if(NOT EXISTS "${C03_PROTOTYPE_INCLUDE}/dp_dummy_robot_v2.hpp")
  message(FATAL_ERROR "ROS C03 prototype mirror does not contain the Dummy generated codec")
endif()

# 纯头文件守卫接口只导出include与C++17要求，不生成独立机器码库。
add_library(dummy_goal_registration_gate INTERFACE)
target_compile_features(dummy_goal_registration_gate INTERFACE cxx_std_17)
target_include_directories(dummy_goal_registration_gate INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# 同进程命令交接由独立共享库持有，避免不同插件各自产生一份不相通的静态命令通道状态；管理通道注册表在硬件库的device_transport中。
add_library(dummy_command_channel SHARED src/command_channel.cpp)
target_compile_features(dummy_command_channel PRIVATE cxx_std_17)
target_include_directories(dummy_command_channel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# 控制器插件：在JTC前核对注册凭据并向共享command_channel转交更新；加载类名见插件XML。
add_library(dummy_guarded_joint_trajectory_controller SHARED
  src/guarded_joint_trajectory_controller.cpp)
target_compile_features(dummy_guarded_joint_trajectory_controller PRIVATE cxx_std_17)
target_link_libraries(dummy_guarded_joint_trajectory_controller
  dummy_goal_registration_gate dummy_command_channel OpenSSL::Crypto)
ament_target_dependencies(dummy_guarded_joint_trajectory_controller
  controller_interface joint_trajectory_controller pluginlib trajectory_msgs)
pluginlib_export_plugin_description_file(controller_interface dummy_ros_control_plugins.xml)

# 硬件插件：非阻塞System入口、唯一device worker及raw证据writer；网络轮询不放在实时write回调。
add_library(dummy_guarded_zenbedded_system SHARED
  src/guarded_zenbedded_system.cpp src/device_transport.cpp src/raw_evidence_collector.cpp)
target_compile_features(dummy_guarded_zenbedded_system PRIVATE cxx_std_17)
target_include_directories(dummy_guarded_zenbedded_system PRIVATE "${DUMMY_WIRE_INCLUDE}")
target_link_libraries(dummy_guarded_zenbedded_system
  dummy_goal_registration_gate dummy_command_channel dummy_dp_comm OpenSSL::Crypto)
ament_target_dependencies(dummy_guarded_zenbedded_system hardware_interface pluginlib)
pluginlib_export_plugin_description_file(hardware_interface dummy_ros_control_plugins.xml)

# 以下首个测试区装配组件/替身和Host探针。CTest的TIMEOUT是测试进程墙钟上限，不能当作设备watchdog参数。
if(BUILD_TESTING)
  find_package(Python3 COMPONENTS Interpreter REQUIRED)
  # 慢stderr/日志保留、轨迹摘要和raw写入组件；分别验证其责任边界，不据日志成功推断物理动作成功。
  add_executable(diagnostic_journal_tests test/diagnostic_journal_tests.cpp)
  target_compile_features(diagnostic_journal_tests PRIVATE cxx_std_17)
  target_include_directories(diagnostic_journal_tests PRIVATE include)
  target_link_libraries(diagnostic_journal_tests PRIVATE pthread)
  add_test(NAME diagnostic_journal_slow_stderr_and_retention COMMAND diagnostic_journal_tests)
  set_tests_properties(diagnostic_journal_slow_stderr_and_retention PROPERTIES TIMEOUT 5 LABELS "component_only;offline")
  add_executable(trajectory_digest_tests test/trajectory_digest_tests.cpp)
  target_link_libraries(trajectory_digest_tests PRIVATE dummy_guarded_joint_trajectory_controller)
  add_test(NAME trajectory_digest_tests COMMAND trajectory_digest_tests)
  add_executable(raw_sequence_logging_tests test/raw_sequence_logging_tests.cpp)
  target_include_directories(raw_sequence_logging_tests PRIVATE "${DUMMY_WIRE_INCLUDE}")
  target_link_libraries(raw_sequence_logging_tests PRIVATE dummy_guarded_zenbedded_system OpenSSL::Crypto)
  add_test(NAME raw_sequence_slow_stderr
    COMMAND ${Python3_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/test/run_slow_stderr_fixture.py"
      "$<TARGET_FILE:raw_sequence_logging_tests>")
  set_tests_properties(raw_sequence_slow_stderr PROPERTIES TIMEOUT 45 LABELS "component_only;offline")
  add_executable(raw_evidence_collector_tests test/raw_evidence_collector_tests.cpp)
  target_include_directories(raw_evidence_collector_tests PRIVATE "${DUMMY_WIRE_INCLUDE}")
  target_link_libraries(raw_evidence_collector_tests dummy_guarded_zenbedded_system OpenSSL::Crypto)
  add_test(NAME raw_evidence_collector_tests COMMAND raw_evidence_collector_tests)
  # 同一测试源额外编译NDEBUG版本，核对证据检查不依赖普通assert是否被编译掉。
  add_executable(raw_evidence_collector_ndebug_tests test/raw_evidence_collector_tests.cpp)
  target_compile_definitions(raw_evidence_collector_ndebug_tests PRIVATE NDEBUG)
  target_include_directories(raw_evidence_collector_ndebug_tests PRIVATE "${DUMMY_WIRE_INCLUDE}")
  target_link_libraries(raw_evidence_collector_ndebug_tests
    dummy_guarded_zenbedded_system OpenSSL::Crypto)
  add_test(NAME raw_evidence_collector_ndebug_tests COMMAND raw_evidence_collector_ndebug_tests)
  # 真实System入口/真实command channel/原SPSC队列；link wrap仅控制线程交错。
  # 只对指定C++符号做链接wrap以安排线程交错，仍用生产System/channel/SPSC；不是另造简化队列。
  add_executable(transfer_owner_tests test/transfer_owner_tests.cpp)
  target_compile_features(transfer_owner_tests PRIVATE cxx_std_17)
  target_include_directories(transfer_owner_tests PRIVATE "${DUMMY_WIRE_INCLUDE}")
  target_link_libraries(transfer_owner_tests PRIVATE dummy_guarded_zenbedded_system)
  target_link_options(transfer_owner_tests PRIVATE
    "-Wl,--wrap=_ZN17dummy_ros_control22consume_control_updateEm")
  add_test(NAME transfer_owner_overlap_and_queue_contracts COMMAND transfer_owner_tests)
  set_tests_properties(transfer_owner_overlap_and_queue_contracts PROPERTIES
    TIMEOUT 10 LABELS "component_only;offline;thread_owner")
  add_executable(state_contention_tests test/state_contention_tests.cpp)
  target_link_libraries(state_contention_tests dummy_guarded_zenbedded_system)
  add_test(NAME state_contention_tests COMMAND state_contention_tests)
  # 共享组件二进制由命令行scenario选择故障/乱序分支；后续多个CTest名可以映射到同一可执行文件。
  add_executable(transport_component_tests test/transport_component_tests.cpp)
  # 台架组件的wire状态直接由D第一方正式映射产生，防止两侧手写同错。
  target_include_directories(transport_component_tests PRIVATE
    "${DPSDK_ROOT}/applications/dummy_ref_control/comm"
    "${DPSDK_ROOT}/applications/dummy_ref_control/include")
  target_link_libraries(transport_component_tests dummy_guarded_zenbedded_system OpenSSL::Crypto)
  add_test(NAME transport_component_tests COMMAND transport_component_tests)
  add_test(NAME transport_worker_slow_stderr
    COMMAND ${Python3_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/test/run_slow_stderr_fixture.py"
      "$<TARGET_FILE:transport_component_tests>" six_axis)
  set_tests_properties(transport_worker_slow_stderr PROPERTIES TIMEOUT 45 LABELS "component_only;offline;synthetic_pty")
  # 仅Host测试：真实Core停止握手+现SDK/Worker回调，非设备/完整Node资格。
  # 安全停车首样本测试直接编译当前SDK Core，并从当前model/profile生成模型头；Host模型不能冒充实体时序资格。
  set(_safe_prime_core_root "${DPSDK_ROOT}")
  set(_safe_prime_generated "${CMAKE_CURRENT_BINARY_DIR}/safe_prime_generated")
  # 按DEPENDS重建生成头，使用同一Python模块路径；VERBATIM保留包含空格的参数边界。
  add_custom_command(OUTPUT "${_safe_prime_generated}/dummy_model.hpp"
    COMMAND ${CMAKE_COMMAND} -E env "PYTHONPATH=${DPSDK_ROOT}/applications/dummy_ref_control/tools"
      ${Python3_EXECUTABLE} -m dummy_ref_tools.generate_model
      --model "${_safe_prime_core_root}/applications/dummy_ref_control/model/model.yaml"
      --profile "${_safe_prime_core_root}/applications/dummy_ref_control/profiles/f405_6dof_bench.yaml"
      --output "${_safe_prime_generated}/dummy_model.hpp"
    DEPENDS "${_safe_prime_core_root}/applications/dummy_ref_control/tools/dummy_ref_tools/generate_model.py"
      "${_safe_prime_core_root}/applications/dummy_ref_control/tools/dummy_ref_tools/rest_target.py"
      "${_safe_prime_core_root}/applications/dummy_ref_control/model/model.yaml"
      "${_safe_prime_core_root}/applications/dummy_ref_control/profiles/f405_6dof_bench.yaml"
    VERBATIM)
  add_custom_target(transport_safe_prime_model DEPENDS "${_safe_prime_generated}/dummy_model.hpp")
  # 固定列出真实Core翻译单元，避免glob意外加入主入口/平台实现；下方循环只拼接路径，不运行Core。
  set(_safe_prime_core_sources authorization joint_mapper motor_can_codec operation_cache reference
    command_guard bench_permit deployment_policy state_estimator stop_controller can_scheduler
    can_commit_tracker profile_loader motion_transaction run_ledger attempt_slots robot_core)
  set(_safe_prime_core_files "")
  foreach(_source IN LISTS _safe_prime_core_sources)
    list(APPEND _safe_prime_core_files "${_safe_prime_core_root}/applications/dummy_ref_control/core/${_source}.cpp")
  endforeach()
  add_executable(transport_safe_cancel_prime_tests test/transport_safe_cancel_prime_tests.cpp ${_safe_prime_core_files})
  add_dependencies(transport_safe_cancel_prime_tests transport_safe_prime_model)
  target_include_directories(transport_safe_cancel_prime_tests PRIVATE
    "${_safe_prime_core_root}/applications/dummy_ref_control/include"
    "${_safe_prime_core_root}/applications/dummy_ref_control/test"
    "${_safe_prime_core_root}/applications/dummy_ref_control/comm"
    "${_safe_prime_generated}")
  target_link_libraries(transport_safe_cancel_prime_tests dummy_guarded_zenbedded_system OpenSSL::Crypto)
  # 夹具完整性校验与正/负场景分开；只有可执行target被构建不表示这些场景都已被CTest运行。
  add_test(NAME transport_safe_prime_fixture_integrity
    COMMAND ${Python3_EXECUTABLE}
      "${CMAKE_CURRENT_SOURCE_DIR}/test/safe_prime_fixture_integrity_tests.py"
      "$<TARGET_FILE:transport_safe_cancel_prime_tests>")
  set_tests_properties(transport_safe_prime_fixture_integrity PROPERTIES TIMEOUT 60)
  add_test(NAME transport_safe_cancel_prime_positive COMMAND transport_safe_cancel_prime_tests six_axis cancel_coordination_positive)
  # 逐个注册错误身份、漂移、过期/重复Stop等安全首样本场景，传给同一测试二进制的scenario键保持稳定。
  foreach(_mode drift wrong_boot wrong_session wrong_ref old_motion arbitrary_target foreign_run wrong_status_phase wrong_query_op expired no_owned second_stop)
    add_test(NAME transport_safe_cancel_prime_${_mode} COMMAND transport_safe_cancel_prime_tests six_axis safe_prime_${_mode})
    set_tests_properties(transport_safe_cancel_prime_${_mode} PROPERTIES TIMEOUT 20 LABELS "component_only;offline;source_model")
  endforeach()
  set_tests_properties(transport_safe_cancel_prime_positive PROPERTIES TIMEOUT 20 LABELS "component_only;offline;source_model")
  # Received/Pending先到走当前严格D10；回执缺失独立验证safeprime守卫，不冒释放确认。
  # 此组刻意走初次取消RPC尚不可用的分支，区分守卫有效与释放终态已确认。
  foreach(_guard_mode positive drift wrong_boot wrong_session wrong_ref old_motion arbitrary_target foreign_run wrong_status_phase no_owned expired second_stop)
    add_test(NAME transport_safe_cancel_prime_guard_${_guard_mode}
      COMMAND transport_safe_cancel_prime_tests six_axis safe_prime_guard_${_guard_mode})
    set_tests_properties(transport_safe_cancel_prime_guard_${_guard_mode} PROPERTIES
      TIMEOUT 20 LABELS "component_only;offline;source_model;initial_cancel_rpc_unavailable")
  endforeach()
  # 现有严格D10在CancelPending期间释放：真实Core200ms stationary模型，非实体资格。
  add_executable(transport_current_cancel_release_tests test/transport_current_cancel_release_tests.cpp ${_safe_prime_core_files})
  add_dependencies(transport_current_cancel_release_tests transport_safe_prime_model)
  target_include_directories(transport_current_cancel_release_tests PRIVATE
    "${_safe_prime_core_root}/applications/dummy_ref_control/include"
    "${_safe_prime_core_root}/applications/dummy_ref_control/test"
    "${_safe_prime_core_root}/applications/dummy_ref_control/comm"
    "${_safe_prime_generated}")
  target_link_libraries(transport_current_cancel_release_tests dummy_guarded_zenbedded_system OpenSSL::Crypto)
  # 复用现Core/Worker模型：早Idle在强Query后重验；SOURCE范围不冒实体时间资格。
  # 早到Idle先隔离、在强Query之后重验；这些Host正负例不把单独Idle状态当成释放提交。
  foreach(_early_mode positive no_commit wrong_boot wrong_op wrong_run stale permission second_stop commit_first direct_retired)
    add_test(NAME transport_stop_early_idle_replay_${_early_mode}
      COMMAND transport_current_cancel_release_tests six_axis early_idle_replay_${_early_mode})
    set_tests_properties(transport_stop_early_idle_replay_${_early_mode} PROPERTIES
      TIMEOUT 20 LABELS "component_only;offline;source_model")
  endforeach()
  add_test(NAME transport_current_cancel_release_positive COMMAND transport_current_cancel_release_tests six_axis cancel_coordination_positive)
  foreach(_mode wrong_boot wrong_session wrong_ref old_motion incoherent foreign_run stale no_owned second_stop fault failed_final)
    add_test(NAME transport_current_cancel_release_${_mode} COMMAND transport_current_cancel_release_tests six_axis safe_prime_${_mode})
    set_tests_properties(transport_current_cancel_release_${_mode} PROPERTIES TIMEOUT 20 LABELS "component_only;offline;source_model")
  endforeach()
  set_tests_properties(transport_current_cancel_release_positive PROPERTIES TIMEOUT 20 LABELS "component_only;offline;source_model")
  # 独立Stop执行器边界与共享peer背压测试；实际职责由测试参数区分，不复用物理设备owner。
  add_executable(transport_stop_exec_tests test/transport_stop_exec_tests.cpp)
  target_include_directories(transport_stop_exec_tests PRIVATE
    "${DPSDK_ROOT}/applications/dummy_ref_control/comm"
    "${DPSDK_ROOT}/applications/dummy_ref_control/include")
  target_link_libraries(transport_stop_exec_tests dummy_guarded_zenbedded_system OpenSSL::Crypto)
  add_test(NAME transport_stop_exec_tests COMMAND transport_stop_exec_tests)
  set_tests_properties(transport_stop_exec_tests PROPERTIES TIMEOUT 60 LABELS "component_only;offline")
  add_test(NAME transport_peer_publish_backpressure_tests
    COMMAND transport_component_tests peer_publish_backpressure)
  add_test(NAME transport_peer_state_queue_full_once_tests
    COMMAND transport_component_tests peer_state_queue_full_once)
  add_test(NAME transport_stop_sequence_rebase_tests
    COMMAND transport_component_tests six_axis stop_sequence_rebase)
  add_test(NAME transport_stop_identity_restart_tests
    COMMAND transport_component_tests six_axis stop_identity_restart)
  add_test(NAME transport_stop_uncommitted_terminal_tests
    COMMAND transport_component_tests six_axis stop_uncommitted_terminal)
  add_test(NAME transport_stop_uncommitted_idle_status_tests
    COMMAND transport_component_tests six_axis stop_uncommitted_idle_status)
  add_test(NAME transport_historical_idle_baseline_tests
    COMMAND transport_component_tests historical_idle_baseline)
  set_tests_properties(transport_historical_idle_baseline_tests PROPERTIES LABELS "component_only;offline")
  add_test(NAME transport_six_axis_component_tests COMMAND transport_component_tests six_axis)
  # 票据更新/陈旧/身份变化与释放停用路径分开注册，防止仅凭收到新包就重新授予控制。
  add_test(NAME transport_ticket_refresh_tests COMMAND transport_component_tests six_axis ticket_refresh)
  add_test(NAME transport_ticket_stale_tests COMMAND transport_component_tests six_axis ticket_stale)
  add_test(NAME transport_ticket_authority_tests COMMAND transport_component_tests six_axis ticket_authority)
  add_test(NAME transport_released_idle_deactivate_tests COMMAND transport_component_tests six_axis released_idle_deactivate)
  add_test(NAME transport_released_idle_unconfirmed_tests COMMAND transport_component_tests six_axis released_idle_unconfirmed)
  add_test(NAME transport_released_idle_no_gate_reply_tests COMMAND transport_component_tests six_axis released_idle_no_gate_reply)
  add_test(NAME transport_released_state_before_status_tests COMMAND transport_component_tests six_axis released_state_before_status)
  add_test(NAME transport_released_idle_heartbeat_gap_tests COMMAND transport_component_tests six_axis released_idle_heartbeat_gap)
  # 释放回执的operation、代次、管理水位及Prepared/Committed层级逐一损坏；不得删负例来接受旧回执。
  foreach(_receipt_case wrong_receipt_op wrong_receipt_epoch wrong_receipt_management pending_receipt uncommitted_receipt)
    add_test(NAME transport_release_stopping_${_receipt_case}_tests
      COMMAND transport_component_tests six_axis release_stopping_${_receipt_case})
  endforeach()
  add_test(NAME transport_release_committed_idle_status_gap_tests COMMAND transport_component_tests six_axis release_committed_idle_status_gap)
  add_test(NAME transport_release_stopping_combined_stale_candidate_tests COMMAND transport_component_tests six_axis release_stopping_combined_stale_candidate)
  add_test(NAME transport_release_stopping_committed_idle_proof_tests COMMAND transport_component_tests six_axis release_stopping_committed_idle_proof)
  add_test(NAME transport_released_idle_before_reply_tests COMMAND transport_component_tests six_axis released_idle_before_reply)
  add_test(NAME transport_released_no_final_idle_tests COMMAND transport_component_tests six_axis released_no_final_idle)
  add_test(NAME transport_released_idle_on_error_tests COMMAND transport_component_tests six_axis released_idle_on_error)
  add_test(NAME transport_running_deactivate_tests COMMAND transport_component_tests six_axis running_deactivate)
  # 原250k下的exactCancel协调SOURCE回调，不代表ROS/硬件资格。
  foreach(_cancel_stop_case IN ITEMS positive boot session ref run owner failed pending nonhold foreign_query second_stop)
    add_test(NAME transport_cancel_stop_${_cancel_stop_case}
      COMMAND transport_component_tests six_axis cancel_coordination_${_cancel_stop_case})
  endforeach()

  # Raw证据记录、释放回执和页序/摘要/身份错配使用独立scenario；组件成功范围仅限相应输入。
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
  # 证据刷新/保持/取消终页的一组scenario键；循环同时收集CTest名字用于统一label，label本身不会启停硬件。
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
    raw_order_multi_successor_late_correct_chain
    raw_order_refresh_hold_stale_status_fresh_state
    raw_order_refresh_hold_stale_status_bad_boot
    raw_order_refresh_hold_stale_status_bad_session
    raw_order_refresh_hold_stale_status_bad_run
    raw_order_refresh_hold_stale_status_bad_chain
    raw_order_refresh_hold_stale_state
    raw_order_refresh_hold_stale_status_non_hold_state
    raw_order_refresh_hold_stale_status_bad_receipt_chain
    raw_order_cancel_stale_status_fresh_state
    raw_order_cancel_stale_status_bad_receipt_boot
    raw_order_cancel_stale_status_bad_receipt_session
    raw_order_cancel_stale_status_bad_receipt_run
    raw_order_cancel_stale_status_bad_receipt_chain
    raw_order_cancel_stale_status_stale_state
    raw_order_cancel_stale_status_non_hold_state
    raw_order_cancel_stale_status_not_ready_state
    raw_order_cancel_stale_status_management_mismatch
    raw_order_cancel_stale_status_no_safety_hold
    raw_order_cancel_stale_status_bad_safety_motion
    raw_order_cancel_stale_status_bad_successor_motion
    raw_order_cancel_hold_status_fresh_state
    raw_order_cancel_hold_status_bad_receipt_boot
    raw_order_cancel_hold_status_bad_receipt_session
    raw_order_cancel_hold_status_bad_receipt_run
    raw_order_cancel_hold_status_bad_receipt_chain
    raw_order_cancel_hold_status_stale_state
    raw_order_cancel_hold_status_non_hold_state
    raw_order_cancel_hold_status_not_ready_state
    raw_order_cancel_hold_status_management_mismatch
    raw_order_cancel_hold_status_no_safety_hold
    raw_order_cancel_hold_status_bad_safety_motion
    raw_order_cancel_hold_status_bad_successor_motion
    raw_order_cancel_hold_terminal7_fresh_state
    raw_order_cancel_hold_terminal7_old_final0
    raw_order_cancel_hold_terminal7_stopping_state_not_ready
    raw_order_cancel_hold_terminal7_final4
    raw_order_cancel_hold_terminal7_final5
    raw_order_cancel_hold_terminal7_final6
    raw_order_cancel_hold_terminal7_final9
    raw_order_cancel_hold_terminal7_wrong_status_chain
    raw_order_cancel_hold_terminal7_zero_status_chain
    raw_order_cancel_hold_terminal7_failure_source
    raw_order_cancel_hold_terminal7_failure_axis
    raw_order_cancel_hold_terminal7_failure_reason
    raw_order_cancel_hold_terminal7_bad_receipt_boot
    raw_order_cancel_hold_terminal7_bad_receipt_session
    raw_order_cancel_hold_terminal7_bad_receipt_run
    raw_order_cancel_hold_terminal7_bad_receipt_chain
    raw_order_cancel_hold_terminal7_stale_state
    raw_order_cancel_hold_terminal7_non_hold_state
    raw_order_cancel_hold_terminal7_not_ready_state
    raw_order_cancel_hold_terminal7_management_mismatch
    raw_order_cancel_hold_terminal7_no_safety_hold
    raw_order_cancel_hold_terminal7_bad_safety_motion
    raw_order_cancel_hold_terminal7_bad_successor_motion)
  # 每个键对应一项独立CTest，即使共享同一binary；新增键必须在测试源实际实现，否则不是有效覆盖。
  foreach(_case IN LISTS _refresh_cases)
    add_test(NAME transport_${_case}_component_tests
      COMMAND transport_component_tests six_axis ${_case})
    list(APPEND _refresh_tests transport_${_case}_component_tests)
  endforeach()
  # Complete的终态、同一/新保持首样本、SENT ACK、State/Status先后及证据后继分别验证。
  add_test(NAME transport_complete_same_prime_fresh_state_pending_cleanup_tests COMMAND transport_component_tests six_axis complete_same_prime_fresh_state_pending_cleanup)
  add_test(NAME transport_complete_retired_pending_wrong_hold_boot_tests COMMAND transport_component_tests six_axis complete_retired_pending_wrong_hold_boot)
  add_test(NAME transport_complete_retired_pending_wrong_hold_epoch_tests COMMAND transport_component_tests six_axis complete_retired_pending_wrong_hold_epoch)
  add_test(NAME transport_complete_retired_pending_divergent_prime_tests COMMAND transport_component_tests six_axis complete_retired_pending_divergent_prime)
  add_test(NAME transport_complete_retired_pending_stale_hold_payload_tests COMMAND transport_component_tests six_axis complete_retired_pending_stale_hold_payload)
  add_test(NAME transport_complete_retry_status_first_tests COMMAND transport_component_tests six_axis complete_retry_status_first)
  add_test(NAME transport_complete_retry_wrong_status_epoch_tests COMMAND transport_component_tests six_axis complete_retry_wrong_status_epoch)
  # 保持Status先于State到达时，区分同目标、分歧、未确认SENT、错代次和过期等输入。
  foreach(_pending_case same_prime divergent_prime unconfirmed_sent wrong_epoch expired_pending wrong_ack_seq partial_sent)
    add_test(NAME transport_complete_hold_status_before_state_${_pending_case}_tests
      COMMAND transport_component_tests six_axis complete_hold_status_before_state_${_pending_case})
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
  # 给本组设置component_only/synthetic_peer用于筛选；不是把每项测试自动转换成同一种夹具。
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
  # 两个生产peer探针只生成可执行文件，并未在这里注册真实设备CTest；使用前需另行明确所选端点和owner。
  add_executable(real_peer_state_probe test/real_peer_state_probe.cpp)
  target_include_directories(real_peer_state_probe PRIVATE "${DUMMY_WIRE_INCLUDE}")
  target_link_libraries(real_peer_state_probe dummy_guarded_zenbedded_system)
  add_executable(production_transport_tests test/production_transport_tests.cpp)
  target_link_libraries(production_transport_tests dummy_guarded_zenbedded_system)
  # SDK锁测试在私有工程损坏两codec与三安装归档并检验拒绝；三PIC源锁的篡改属于上层独立配置负例。
  add_test(NAME sdk_input_lock_tests COMMAND "${CMAKE_COMMAND}"
    "-DDPSDK_ROOT=${DPSDK_ROOT}" "-DTEST_ROOT=${CMAKE_CURRENT_BINARY_DIR}/sdk-lock-test"
    "-DLOCK_FILE=${CMAKE_CURRENT_SOURCE_DIR}/cmake/verify_sdk_inputs.cmake"
    "-DDPSDK_HOST_SDK_PREFIX=${DPSDK_HOST_SDK_PREFIX}"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/test/sdk_input_lock_tests.cmake")
  add_test(NAME production_transport_peer_discovery_tests COMMAND "${Python3_EXECUTABLE}"
    "${CMAKE_CURRENT_SOURCE_DIR}/test/run_transport_peer_test_tests.py")
  # 轻量生产者/陈旧状态/非阻塞接口回归；按目标补链接真实硬件插件，不连接物理总线。
  foreach(r04_test IN ITEMS producer_tests stale_state_tests nonblocking_tests)
    add_executable(${r04_test} test/${r04_test}.cpp)
    target_link_libraries(${r04_test} dummy_goal_registration_gate)
    add_test(NAME ${r04_test} COMMAND ${r04_test})
  endforeach()
  target_link_libraries(nonblocking_tests dummy_guarded_zenbedded_system)

  # 当前wire观察边界用生产dp_comm；minor4兼容codec另用历史镜像，不能替代当前消息身份检查。
  add_executable(wire_probe_tests test/wire_probe_tests.cpp)
  # 白盒thunk的typed-decode观察边界来自实际当前runtime，不提供测试空stub。
  target_link_libraries(wire_probe_tests PRIVATE dummy_dp_comm)
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

  # pluginlib装载测试使用私有resource index与库软链接；不会运行安装器或改用户正在使用的安装树。
  add_executable(guarded_plugin_load_tests test/guarded_plugin_load_tests.cpp)
  ament_target_dependencies(guarded_plugin_load_tests controller_interface hardware_interface pluginlib)
  # pluginlib组件测试只装配私有索引/链接，不运行installer或改live安装树。
  set(DUMMY_PLUGIN_TEST_PREFIX "${CMAKE_CURRENT_BINARY_DIR}/plugin_test_prefix")
  set(_plugin_stage_script "${CMAKE_CURRENT_BINARY_DIR}/prepare_plugin_test_prefix.cmake")
  # 括号字符串在配置时用@ONLY替换路径，再由file(GENERATE)展开TARGET_FILE；内容是测试运行时执行的CMake脚本。
  set(_plugin_stage_content [=[
file(MAKE_DIRECTORY "@DUMMY_PLUGIN_TEST_PREFIX@/share/ament_index/resource_index/packages"
  "@DUMMY_PLUGIN_TEST_PREFIX@/share/ament_index/resource_index/controller_interface__pluginlib__plugin"
  "@DUMMY_PLUGIN_TEST_PREFIX@/share/ament_index/resource_index/hardware_interface__pluginlib__plugin"
  "@DUMMY_PLUGIN_TEST_PREFIX@/share/dummy_ros_control" "@DUMMY_PLUGIN_TEST_PREFIX@/lib")
file(WRITE "@DUMMY_PLUGIN_TEST_PREFIX@/share/ament_index/resource_index/packages/dummy_ros_control" "")
file(WRITE "@DUMMY_PLUGIN_TEST_PREFIX@/share/ament_index/resource_index/controller_interface__pluginlib__plugin/dummy_ros_control" "share/dummy_ros_control/dummy_ros_control_plugins.xml\n")
file(WRITE "@DUMMY_PLUGIN_TEST_PREFIX@/share/ament_index/resource_index/hardware_interface__pluginlib__plugin/dummy_ros_control" "share/dummy_ros_control/dummy_ros_control_plugins.xml\n")
file(REMOVE "@DUMMY_PLUGIN_TEST_PREFIX@/share/dummy_ros_control/package.xml")
file(CREATE_LINK "@CMAKE_CURRENT_SOURCE_DIR@/package.xml" "@DUMMY_PLUGIN_TEST_PREFIX@/share/dummy_ros_control/package.xml" SYMBOLIC)
file(REMOVE "@DUMMY_PLUGIN_TEST_PREFIX@/share/dummy_ros_control/dummy_ros_control_plugins.xml")
file(CREATE_LINK "@CMAKE_CURRENT_SOURCE_DIR@/dummy_ros_control_plugins.xml" "@DUMMY_PLUGIN_TEST_PREFIX@/share/dummy_ros_control/dummy_ros_control_plugins.xml" SYMBOLIC)
file(REMOVE "@DUMMY_PLUGIN_TEST_PREFIX@/lib/$<TARGET_FILE_NAME:dummy_command_channel>" "@DUMMY_PLUGIN_TEST_PREFIX@/lib/$<TARGET_FILE_NAME:dummy_guarded_joint_trajectory_controller>" "@DUMMY_PLUGIN_TEST_PREFIX@/lib/$<TARGET_FILE_NAME:dummy_guarded_zenbedded_system>")
file(CREATE_LINK "$<TARGET_FILE:dummy_command_channel>" "@DUMMY_PLUGIN_TEST_PREFIX@/lib/$<TARGET_FILE_NAME:dummy_command_channel>" SYMBOLIC)
file(CREATE_LINK "$<TARGET_FILE:dummy_guarded_joint_trajectory_controller>" "@DUMMY_PLUGIN_TEST_PREFIX@/lib/$<TARGET_FILE_NAME:dummy_guarded_joint_trajectory_controller>" SYMBOLIC)
file(CREATE_LINK "$<TARGET_FILE:dummy_guarded_zenbedded_system>" "@DUMMY_PLUGIN_TEST_PREFIX@/lib/$<TARGET_FILE_NAME:dummy_guarded_zenbedded_system>" SYMBOLIC)
]=])
  string(CONFIGURE "${_plugin_stage_content}" _plugin_stage_content @ONLY)
  file(GENERATE OUTPUT "${_plugin_stage_script}" CONTENT "${_plugin_stage_content}")
  add_test(NAME prepare_plugin_test_prefix COMMAND ${CMAKE_COMMAND} -P "${_plugin_stage_script}")
  # FIXTURES_SETUP保证依赖该fixture的测试先生成私有索引；RUN环境只为这些装载测试设置。
  set_tests_properties(prepare_plugin_test_prefix PROPERTIES FIXTURES_SETUP dummy_plugin_test_prefix)
  add_test(NAME guarded_plugin_load_tests COMMAND guarded_plugin_load_tests)
  add_test(NAME independent_guarded_library_load
    COMMAND ${Python3_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/test/independent_library_load.py"
      "$<TARGET_FILE:dummy_guarded_zenbedded_system>")
  set_tests_properties(independent_guarded_library_load PROPERTIES
    TIMEOUT 15 LABELS "component_only;offline;runtime_link")
  add_executable(guarded_lifecycle_probe test/guarded_lifecycle_probe.cpp)
  target_link_libraries(guarded_lifecycle_probe dummy_guarded_zenbedded_system)
  ament_target_dependencies(guarded_lifecycle_probe hardware_interface pluginlib)
  add_test(NAME guarded_lifecycle_probe COMMAND guarded_lifecycle_probe)
  set_tests_properties(guarded_plugin_load_tests guarded_lifecycle_probe PROPERTIES
    FIXTURES_REQUIRED dummy_plugin_test_prefix
    ENVIRONMENT "AMENT_PREFIX_PATH=${DUMMY_PLUGIN_TEST_PREFIX}:/opt/ros/jazzy;LD_LIBRARY_PATH=${DUMMY_PLUGIN_TEST_PREFIX}/lib:${CMAKE_CURRENT_BINARY_DIR}:/opt/ros/jazzy/lib")

  # 当前公共头只能配当前runtime，原析构测试独立保618/18000容量与测试hook。
  # 请求句柄析构白盒测试独立编译DP_COMM_TESTING且保留618/18000历史容量；不把它链接进651/18000生产插件。
  add_library(dummy_dp_comm_current_test_runtime STATIC
    "${DPSDK_ROOT}/libraries/dp_comm/src/dp_comm_ros_like.cpp")
  target_compile_features(dummy_dp_comm_current_test_runtime PUBLIC cxx_std_17)
  get_target_property(_dummy_test_runtime_includes dummy_dp_comm_pic INCLUDE_DIRECTORIES)
  get_target_property(_dummy_test_runtime_defines dummy_dp_comm_pic COMPILE_DEFINITIONS)
  # 先去除从生产PIC目标继承的容量宏，再设测试值，避免同一编译命令重复冲突定义。
  list(FILTER _dummy_test_runtime_defines EXCLUDE REGEX "^DP_COMM_RUNTIME_(MAX_MESSAGE_BYTES|STORAGE_BYTES)=")
  target_include_directories(dummy_dp_comm_current_test_runtime PRIVATE ${_dummy_test_runtime_includes})
  target_compile_definitions(dummy_dp_comm_current_test_runtime PRIVATE ${_dummy_test_runtime_defines}
    DP_COMM_RUNTIME_MAX_MESSAGE_BYTES=618 DP_COMM_RUNTIME_STORAGE_BYTES=18000 DP_COMM_TESTING=1)
  add_executable(request_handle_destructor_tests test/request_handle_destructor_tests.cpp)
  target_include_directories(request_handle_destructor_tests PRIVATE
    "${DPSDK_ROOT}/libraries/dp_comm/include")
  target_compile_definitions(request_handle_destructor_tests PRIVATE
    DP_COMM_RUNTIME_MAX_MESSAGE_BYTES=618
    DP_COMM_RUNTIME_STORAGE_BYTES=18000
    DP_COMM_TESTING=1)
  target_link_libraries(request_handle_destructor_tests
    dummy_dp_comm_current_test_runtime
    DPSDKHostSDK::osal
    DPSDKHostSDK::console
    pthread)
  add_test(NAME request_handle_destructor_tests COMMAND request_handle_destructor_tests)

  # 有限准入重试的头文件级边界测试；不模拟或证明USB/CAN最终送达。
  add_executable(management_admission_retry_tests test/management_admission_retry_tests.cpp)
  target_compile_features(management_admission_retry_tests PRIVATE cxx_std_17)
  target_include_directories(management_admission_retry_tests PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${DPSDK_ROOT}/libraries/dp_comm/include")
  add_test(NAME management_admission_retry_tests COMMAND management_admission_retry_tests)

  add_executable(command_channel_tests test/command_channel_tests.cpp)
  target_link_libraries(command_channel_tests dummy_command_channel)
  add_test(NAME command_channel_tests COMMAND command_channel_tests)

  # 位置能力匹配和latest-only流使用生产硬件库的测试入口，分别核验准入和积压语义。
  foreach(position_only_test IN ITEMS capability_limit_tests latest_only_stream_tests)
    add_executable(${position_only_test} test/${position_only_test}.cpp)
    target_link_libraries(${position_only_test} dummy_guarded_zenbedded_system)
    add_test(NAME ${position_only_test} COMMAND ${position_only_test})
  endforeach()

  # worker/State注入白盒测试需相同codec与dp_comm接口；编译测试hook不改变已构建生产库的宏。
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

# 安装公共头及库并导出命名空间目标；下游还需能find_package同一DPSDKHostSDK，见README中SDK_DIR配置。
install(DIRECTORY include/ DESTINATION include)
install(TARGETS
  dummy_goal_registration_gate
  dummy_dp_comm
  dummy_dp_comm_pic
  dummy_dp_comm_log_shim
  dummy_command_channel
  dummy_guarded_joint_trajectory_controller
  dummy_guarded_zenbedded_system
  EXPORT export_dummy_goal_registration_gate
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
ament_export_targets(export_dummy_goal_registration_gate HAS_LIBRARY_TARGET)
ament_export_dependencies(OpenSSL DPSDKHostSDK)

# 第二个测试区核对故障清理的raw生命周期代次；仍是离线组件，不把旧run证据补成当前释放。
if(BUILD_TESTING)
  add_executable(raw_cleanup_lifetime_generation_tests test/raw_cleanup_lifetime_generation_tests.cpp)
  target_include_directories(raw_cleanup_lifetime_generation_tests PRIVATE include "${DUMMY_WIRE_INCLUDE}")
  target_link_libraries(raw_cleanup_lifetime_generation_tests PRIVATE dummy_guarded_zenbedded_system)
  target_compile_options(raw_cleanup_lifetime_generation_tests PRIVATE -UNDEBUG)
  add_test(NAME fault_raw_cleanup_lifetime_generation COMMAND raw_cleanup_lifetime_generation_tests)
  add_test(NAME fault_raw_observation_after_permission_invalidation COMMAND transport_component_tests six_axes raw_fault_after_public_invalidate)
  add_test(NAME fault_raw_observation_no_RUN_anchor COMMAND transport_component_tests six_axes raw_fault_no_RUN_anchor)
  add_test(NAME fault_raw_observation_wrong_boot COMMAND transport_component_tests six_axes raw_fault_wrong_boot)
  add_test(NAME fault_raw_observation_stale_producer COMMAND transport_component_tests six_axes raw_fault_stale_produced)
  set_tests_properties(fault_raw_cleanup_lifetime_generation fault_raw_observation_after_permission_invalidation
    fault_raw_observation_no_RUN_anchor fault_raw_observation_wrong_boot fault_raw_observation_stale_producer
    PROPERTIES TIMEOUT 10 LABELS "component_only;offline")
endif()


# 第三个测试区单独登记初始保持首样本ACK：含缺ACK、partial SENT及各种身份/代次/陈旧负例。
if(BUILD_TESTING)
  set(_initial_hold_ack_tests)
  add_test(NAME transport_initial_hold_state_ack_before_status_tests COMMAND transport_component_tests six_axes hold_state_ack_before_status)
  list(APPEND _initial_hold_ack_tests transport_initial_hold_state_ack_before_status_tests)
  add_test(NAME transport_initial_hold_no_ACK_tests COMMAND transport_component_tests six_axes hold_no_ACK)
  list(APPEND _initial_hold_ack_tests transport_initial_hold_no_ACK_tests)
  add_test(NAME transport_initial_hold_partial_SENT_tests COMMAND transport_component_tests six_axes hold_partial_SENT)
  list(APPEND _initial_hold_ack_tests transport_initial_hold_partial_SENT_tests)
  add_test(NAME transport_initial_hold_wrong_epoch_tests COMMAND transport_component_tests six_axes hold_wrong_epoch)
  list(APPEND _initial_hold_ack_tests transport_initial_hold_wrong_epoch_tests)
  add_test(NAME transport_initial_hold_Run_ACK_tests COMMAND transport_component_tests six_axes hold_Run_ACK)
  list(APPEND _initial_hold_ack_tests transport_initial_hold_Run_ACK_tests)
  add_test(NAME transport_initial_hold_wrong_session_tests COMMAND transport_component_tests six_axes hold_wrong_session)
  list(APPEND _initial_hold_ack_tests transport_initial_hold_wrong_session_tests)
  add_test(NAME transport_initial_hold_stale_ACK_tests COMMAND transport_component_tests six_axes hold_stale_ACK)
  list(APPEND _initial_hold_ack_tests transport_initial_hold_stale_ACK_tests)
  add_test(NAME transport_initial_hold_wrong_reference_tests COMMAND transport_component_tests six_axes hold_wrong_reference)
  list(APPEND _initial_hold_ack_tests transport_initial_hold_wrong_reference_tests)
  add_test(NAME transport_initial_hold_wrong_boot_tests COMMAND transport_component_tests six_axes hold_wrong_boot)
  list(APPEND _initial_hold_ack_tests transport_initial_hold_wrong_boot_tests)
  add_test(NAME transport_initial_hold_Stop_before_status_tests COMMAND transport_component_tests six_axes hold_Stop_before_status)
  list(APPEND _initial_hold_ack_tests transport_initial_hold_Stop_before_status_tests)
  set_tests_properties(${_initial_hold_ack_tests} PROPERTIES TIMEOUT 10 LABELS "component_only;offline")
endif()

# 生成ament包配置/resource索引并使install/export对下游可见；必须位于本包目标与导出声明之后。
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})

ament_package()

# @file root_component_peer.cmake
# @brief 测试peer独立进程的3发布者/2服务角色；不把不同StaticStorage ABI装入PIC插件进程。
add_library(root_component_peer_runtime STATIC
  "${DPSDK_ROOT}/libraries/dp_comm/src/dp_comm_ros_like.cpp"
  "${DPSDK_ROOT}/libraries/dp_comm/src/dp_comm_ros_like_transport.cpp"
  "${DPSDK_ROOT}/libraries/dp_comm/src/dp_comm_identity_binding.cpp"
  "${DPSDK_ROOT}/platform/common/dp_comm_pico_port/dp_comm_port.cpp")
target_compile_features(root_component_peer_runtime PUBLIC cxx_std_17)
target_compile_definitions(root_component_peer_runtime PUBLIC
  DP_COMM_RUNTIME_MAX_NODES=1 DP_COMM_RUNTIME_MAX_PUBLISHERS=3
  DP_COMM_RUNTIME_MAX_SUBSCRIPTIONS=2 DP_COMM_RUNTIME_MAX_TIMERS=0
  DP_COMM_RUNTIME_MAX_EVENTS=8 DP_COMM_RUNTIME_MAX_MESSAGE_BYTES=1536
  DP_COMM_RUNTIME_MAX_EVENT_MESSAGE_BYTES=1536 DP_COMM_RUNTIME_MAX_ROOT_EVENT_OWNED_BYTES=768 DP_COMM_RUNTIME_MAX_COMPLETED_RESPONSE_BYTES=1024
  DP_COMM_RUNTIME_MAX_TOPIC_BYTES=173 DP_COMM_RUNTIME_MAX_SERVICES=3
  DP_COMM_RUNTIME_MAX_CLIENTS=1 DP_COMM_RUNTIME_MAX_PENDING_REQUESTS=1
  DP_COMM_RUNTIME_MAX_COMPLETED_RESPONSES=3 DP_COMM_RUNTIME_STORAGE_BYTES=32768
  DP_HOST_RAW_OBSERVER=0 DP_COMM_PICO_TRUSTED_TRANSPORT=1
  DP_COMM_DEFAULT_LOCAL_ONLY=1 DP_COMM_PORT_ROOT_LIFECYCLE=1)
target_compile_definitions(root_component_peer_runtime PRIVATE
  DP_COMM_ENABLE_ENCODED_PUBLISH_OBSERVER=0 ZENOH_LINUX ZENOH_C_STANDARD=11
  ZENOH_COMPILER_GCC ZENOH_LOG_INFO DPSDK_ALLOW_UNSAFE_LIBC_HEAP_FALLBACK=0 DPSDK_ENABLE_TLSF_HEAP=1)
target_include_directories(root_component_peer_runtime PUBLIC "${DPSDK_ROOT}/libraries/dp_comm/include")
target_include_directories(root_component_peer_runtime PRIVATE
  "${DPSDK_ROOT}/libraries/dp_comm/src" "${DPSDK_ROOT}/platform/common/kernel_port" "${DPSDK_ROOT}/platform/posix_native/kernel_port"
  "${DPSDK_ROOT}/core/dp_sdk_core/dp_osal/src" "${DPSDK_ROOT}/core/dp_sdk_core/dp_osal/src/interface"
  "${DPSDK_ROOT}/core/dp_sdk_core/dp_osal/src/debug" "${DPSDK_ROOT}/core/dp_sdk_core/dp_osal/src/impl/posix/include"
  "${DPSDK_ROOT}/core/dp_sdk_core/dp_device/include" "${DPSDK_ROOT}/core/dp_sdk_core/dp_hal/include" "${DPSDK_ROOT}/platform/common/dp_comm_pico_port"
  "${DPSDK_ROOT}/platform/common/dp_comm_zenoh_port_common"
  "${DPSDK_ROOT}/platform/common/console_device" "${DPSDK_ROOT}/platform/common/zenoh_port")
target_link_libraries(root_component_peer_runtime PUBLIC DPSDKHostSDK::dummy_protocol
  DPSDKHostSDK::osal DPSDKHostSDK::console DPSDKHostSDK::zenoh DPSDKHostSDK::log_shim)
add_executable(root_component_peer "${CMAKE_CURRENT_LIST_DIR}/root_component_peer.cpp" "${CMAKE_CURRENT_LIST_DIR}/root_peer_serial_leaf.cpp")
target_compile_features(root_component_peer PRIVATE cxx_std_17)
target_compile_definitions(root_component_peer PRIVATE ZENOH_LINUX ZENOH_COMPILER_GCC ZENOH_C_STANDARD=11)
target_compile_options(root_component_peer PRIVATE -ffunction-sections -fdata-sections -fvisibility=hidden -fvisibility-inlines-hidden)
target_link_options(root_component_peer PRIVATE -Wl,--gc-sections -Wl,--wrap=_z_listen_serial_from_dev)
target_include_directories(root_component_peer PRIVATE "${CMAKE_CURRENT_LIST_DIR}/../include"
  "${DPSDK_ROOT}/applications/dummy_ref_control/comm" "${DPSDK_ROOT}/applications/dummy_ref_control/include")
target_link_libraries(root_component_peer root_component_peer_runtime DPSDKHostSDK::host_identity
  DPSDKHostSDK::hal OpenSSL::Crypto pthread rt dl)
ament_target_dependencies(root_component_peer hardware_interface pluginlib)

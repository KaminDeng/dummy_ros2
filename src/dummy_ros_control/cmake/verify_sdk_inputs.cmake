# 安装 SDK 是生产协议、公共头和 PIC runtime 的唯一来源；不访问 DPSDK 源码。
set(DPSDK_HOST_SDK_PREFIX "${DPSDK_HOST_SDK_PREFIX}" CACHE PATH "同版ROS PIC SDK安装前缀")
if(NOT DPSDK_HOST_SDK_PREFIX)
  message(FATAL_ERROR "必须显式指定DPSDK_HOST_SDK_PREFIX")
endif()
find_package(DPSDKHostSDK CONFIG REQUIRED
  PATHS "${DPSDK_HOST_SDK_PREFIX}/lib/cmake/DPSDKHostSDK" NO_DEFAULT_PATH)
file(REAL_PATH "${DPSDK_HOST_SDK_PREFIX}/lib/cmake/DPSDKHostSDK" _expected_sdk_config)
file(REAL_PATH "${DPSDKHostSDK_DIR}" _actual_sdk_config)
if(NOT _actual_sdk_config STREQUAL _expected_sdk_config)
  message(FATAL_ERROR "SDK配置必须来自显式安装前缀")
endif()
# R11只接纳同一正式Dummy profile及带方向/ABI身份的新安装包；数值由清单和归档配对核验。
if(NOT DPSDKHostSDK_PIC OR
   NOT DPSDKHostSDK_PROTOCOL_SET STREQUAL "dummy_managed" OR
   NOT DPSDKHostSDK_PROTOCOL_CODEC STREQUAL "flatbuffers" OR
   NOT DPSDKHostSDK_PROTOCOL_PROFILE STREQUAL "dummy" OR
   NOT DPSDKHostSDK_RUNTIME_ROLE STREQUAL "ros_pic" OR
   NOT DPSDKHostSDK_RUNTIME_ABI_REVISION STREQUAL "11" OR
   NOT DPSDKHostSDK_MESSAGE_BYTES GREATER_EQUAL 1536 OR
   NOT DPSDKHostSDK_RUNTIME_EVENT_MESSAGE_BYTES EQUAL DPSDKHostSDK_MESSAGE_BYTES OR
   NOT DPSDKHostSDK_RUNTIME_LEGACY_TX_BYTES EQUAL 0)
  message(FATAL_ERROR "SDK协议/Codec/角色必须为R11 dummy_managed/dummy flatbuffers PIC及同版方向资源ABI")
endif()
foreach(_target IN ITEMS DPSDKHostSDK::dp_comm DPSDKHostSDK::dummy_protocol dp_protocol::selected)
  if(NOT TARGET ${_target})
    message(FATAL_ERROR "SDK缺少自包含安装目标：${_target}")
  endif()
endforeach()

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
if(NOT DPSDKHostSDK_PIC OR NOT DPSDKHostSDK_MESSAGE_BYTES EQUAL 651 OR
   NOT DPSDKHostSDK_STORAGE_BYTES EQUAL 18000 OR
   NOT DPSDKHostSDK_PROTOCOL_SET STREQUAL "dummy" OR
   NOT DPSDKHostSDK_PROTOCOL_CODEC STREQUAL "raw-fixed" OR
   NOT DPSDKHostSDK_RUNTIME_ROLE STREQUAL "ros_pic")
  message(FATAL_ERROR "SDK协议/Codec/角色必须为dummy raw-fixed PIC 651/18000")
endif()
foreach(_target IN ITEMS DPSDKHostSDK::dp_comm DPSDKHostSDK::dummy_protocol dp_protocol::selected)
  if(NOT TARGET ${_target})
    message(FATAL_ERROR "SDK缺少自包含安装目标：${_target}")
  endif()
endforeach()

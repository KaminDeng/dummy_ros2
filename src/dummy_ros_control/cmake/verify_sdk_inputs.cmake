# 精确合同保持原锁；下层归档由同版ROS PIC SDK安装manifest验证。
if(NOT DPSDK_ROOT OR NOT IS_DIRECTORY "${DPSDK_ROOT}")
  message(FATAL_ERROR "必须显式指定候选DPSDK_ROOT")
endif()
set(DPSDK_HOST_SDK_PREFIX "${DPSDK_HOST_SDK_PREFIX}" CACHE PATH "同版ROS PIC SDK安装前缀")
if(NOT DPSDK_HOST_SDK_PREFIX)
  message(FATAL_ERROR "必须显式指定DPSDK_HOST_SDK_PREFIX，不能复用旧g6-host")
endif()
find_package(DPSDKHostSDK CONFIG REQUIRED
  PATHS "${DPSDK_HOST_SDK_PREFIX}/lib/cmake/DPSDKHostSDK" NO_DEFAULT_PATH)
file(REAL_PATH "${DPSDK_ROOT}" _sdk_source)
file(REAL_PATH "${DPSDKHostSDK_SOURCE_ROOT}" _exported_source)
if(NOT _sdk_source STREQUAL _exported_source OR NOT DPSDKHostSDK_PIC OR
   NOT DPSDKHostSDK_MESSAGE_BYTES EQUAL 651 OR NOT DPSDKHostSDK_STORAGE_BYTES EQUAL 18000)
  message(FATAL_ERROR "SDK源/角色与ROS PIC 651/18000不一致")
endif()
set(sdk_locked_paths
  applications/dummy_ref_control/contracts/dummy_robot_v2/codec/dp_dummy_robot_v2.hpp
  applications/dummy_ref_control/contracts/dummy_robot_v2/codec/dp_dummy_robot_v2_dp_comm.hpp)
set(sdk_locked_hashes
  649ebc7fc37ec7b85d9eaa116c8174438fac236e592da68350a9d3ae599a9143
  78d2fc3f6cc9cd3eef1e32f989434cb5fc284231593307f63e4499f48c219c40)
set(sdk_lock_index 0)
foreach(path IN LISTS sdk_locked_paths)
  if(NOT EXISTS "${DPSDK_ROOT}/${path}")
    message(FATAL_ERROR "Missing locked DPSDK input: ${DPSDK_ROOT}/${path}")
  endif()
  file(SHA256 "${DPSDK_ROOT}/${path}" actual_hash)
  list(GET sdk_locked_hashes ${sdk_lock_index} expected_hash)
  if(NOT actual_hash STREQUAL expected_hash)
    message(FATAL_ERROR "DPSDK input SHA256 mismatch: ${path}")
  endif()
  math(EXPR sdk_lock_index "${sdk_lock_index}+1")
endforeach()

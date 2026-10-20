# @file verify_sdk_inputs.cmake
# @brief 在配置阶段绑定同一源码根、ROS PIC角色及已锁定的两个生成codec。
# @details 必须由调用方提供DPSDK_ROOT和DPSDK_HOST_SDK_PREFIX；导入的Config还会
# 校验其安装归档身份。本文件不生成SDK，也不从任意旧构建目录自动选取依赖。
# 三个PIC实现源的原始SHA检查在上层CMakeLists.txt，不能把此处两个codec锁当作
# 对所有源码或实际设备固件的校验。验证失败应核对候选身份与回归，不删除门禁。
#
# 精确合同保持原锁；下层归档由同版ROS PIC SDK安装manifest验证。
if(NOT DPSDK_ROOT OR NOT IS_DIRECTORY "${DPSDK_ROOT}")
  message(FATAL_ERROR "必须显式指定候选DPSDK_ROOT")
endif()
# 源码根有效后还需明确安装前缀；两者不可混同，源码目录不是SDK安装目录。
set(DPSDK_HOST_SDK_PREFIX "${DPSDK_HOST_SDK_PREFIX}" CACHE PATH "同版ROS PIC SDK安装前缀")
if(NOT DPSDK_HOST_SDK_PREFIX)
  message(FATAL_ERROR "必须显式指定DPSDK_HOST_SDK_PREFIX，不能复用旧g6-host")
endif()
# NO_DEFAULT_PATH阻止从全局旧前缀意外找到另一个SDK，固定本包的导入来源。
find_package(DPSDKHostSDK CONFIG REQUIRED
  PATHS "${DPSDK_HOST_SDK_PREFIX}/lib/cmake/DPSDKHostSDK" NO_DEFAULT_PATH)
# 解析软链接后比较真实源码根；还必须是PIC和651/18000容量角色。
# 这些是构建配置/内存契约，不是网络MTU或电机数量。
file(REAL_PATH "${DPSDK_ROOT}" _sdk_source)
file(REAL_PATH "${DPSDKHostSDK_SOURCE_ROOT}" _exported_source)
if(NOT _sdk_source STREQUAL _exported_source OR NOT DPSDKHostSDK_PIC OR
   NOT DPSDKHostSDK_MESSAGE_BYTES EQUAL 651 OR NOT DPSDKHostSDK_STORAGE_BYTES EQUAL 18000)
  message(FATAL_ERROR "SDK源/角色与ROS PIC 651/18000不一致")
endif()
# 两个有序列表一一对应：消息codec与dp_comm类型适配器；改顺序须保持对应。
set(sdk_locked_paths
  applications/dummy_ref_control/contracts/dummy_robot_v2/codec/dp_dummy_robot_v2.hpp
  applications/dummy_ref_control/contracts/dummy_robot_v2/codec/dp_dummy_robot_v2_dp_comm.hpp)
# 哈希针对原始文件字节，添加注释也会改变身份；不能只比较解析后的结构。
set(sdk_locked_hashes
  649ebc7fc37ec7b85d9eaa116c8174438fac236e592da68350a9d3ae599a9143
  78d2fc3f6cc9cd3eef1e32f989434cb5fc284231593307f63e4499f48c219c40)
# 逐个确认存在并重算SHA；缺文件和内容漂移都在生成构建系统前失败。
set(sdk_lock_index 0)
foreach(path IN LISTS sdk_locked_paths)
  if(NOT EXISTS "${DPSDK_ROOT}/${path}")
    message(FATAL_ERROR "Missing locked DPSDK input: ${DPSDK_ROOT}/${path}")
  endif()
  file(SHA256 "${DPSDK_ROOT}/${path}" actual_hash)
  # 当前下标同时选取预期摘要，避免因文件名相近把两份codec互换。
  list(GET sdk_locked_hashes ${sdk_lock_index} expected_hash)
  if(NOT actual_hash STREQUAL expected_hash)
    message(FATAL_ERROR "DPSDK input SHA256 mismatch: ${path}")
  endif()
  # 每次严格推进一次；列表容量由本文件维护，越界也会由CMake报错。
  math(EXPR sdk_lock_index "${sdk_lock_index}+1")
endforeach()

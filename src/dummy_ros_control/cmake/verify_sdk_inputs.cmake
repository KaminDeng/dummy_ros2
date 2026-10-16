# Locks the ABI and prebuilt binary inputs used by the production PIC adapter.
# Revalidate transport/component tests before deliberately updating these hashes.
set(sdk_locked_paths
  contracts/dummy_robot_v2/codec/dp_dummy_robot_v2.hpp
  contracts/dummy_robot_v2/codec/dp_dummy_robot_v2_dp_comm.hpp
  build_dummy/g6-host/core/dp_sdk_core/dp_osal/libdp_osal.a
  build_dummy/g6-host/platform_common_console_device/libdp_console_device.a
  build_dummy/g6-host/lib/libzenohpico.a)
set(sdk_locked_hashes
  649ebc7fc37ec7b85d9eaa116c8174438fac236e592da68350a9d3ae599a9143
  78d2fc3f6cc9cd3eef1e32f989434cb5fc284231593307f63e4499f48c219c40
  d399b0d7ae4477247977bc80796e03a9c968c57cfe5ac24ca47dfe2ce4c0f84b
  b1b1aed93e6c79eb38a8fee66894583f64bd8c55ad1b2797b956e6fca125fc78
  6b75d230874ff354ba9da31cc19b28b6ddefae2ffc0f81e77c1733c12f4956cf)
set(sdk_lock_index 0)
foreach(path IN LISTS sdk_locked_paths)
  if(NOT EXISTS "${DPSDK_ROOT}/${path}")
    message(FATAL_ERROR "Missing locked DPSDK input: ${DPSDK_ROOT}/${path}")
  endif()
  file(SHA256 "${DPSDK_ROOT}/${path}" actual_hash)
  list(GET sdk_locked_hashes ${sdk_lock_index} expected_hash)
  if(NOT actual_hash STREQUAL expected_hash)
    message(FATAL_ERROR "DPSDK input SHA256 mismatch: ${path}; revalidate before updating the transport lock")
  endif()
  math(EXPR sdk_lock_index "${sdk_lock_index}+1")
endforeach()

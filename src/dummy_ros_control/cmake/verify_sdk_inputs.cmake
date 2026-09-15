# Locks the ABI and prebuilt binary inputs used by the production PIC adapter.
# Revalidate transport/component tests before deliberately updating these hashes.
set(sdk_locked_paths
  contracts/dummy_robot_v1/codec/dp_dummy_robot_v1.hpp
  contracts/dummy_robot_v1/codec/dp_dummy_robot_v1_dp_comm.hpp
  build/posix-zenohc/core/dp_sdk_core/dp_osal/libdp_osal.a
  build/posix-zenohc/thirdparty/zenoh-c/debug/target/debug/libzenohc.a)
set(sdk_locked_hashes
  e9c04f74bd73edcf5721401c3f2c35217ed9eed528e808fea0870fa56dcc8951
  b7959cff2853c32a4405aeff13b0ee1b4fdf1e9e5eb05f1b797a8598bb4e6743
  277d7411e0dd488244c4462e708c36a0c84f61fc99b5dddd61d2b77a4368b261
  35893ad6c4814de71685b35a2316e04fba6b8d6d17497e0981968d8acf6806b3)
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

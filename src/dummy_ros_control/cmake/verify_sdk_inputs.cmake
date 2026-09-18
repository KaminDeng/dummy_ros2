# Locks the ABI and prebuilt binary inputs used by the production PIC adapter.
# Revalidate transport/component tests before deliberately updating these hashes.
set(sdk_locked_paths
  contracts/dummy_robot_v2/codec/dp_dummy_robot_v2.hpp
  contracts/dummy_robot_v2/codec/dp_dummy_robot_v2_dp_comm.hpp
  build_dummy/g6-host/core/dp_sdk_core/dp_osal/libdp_osal.a
  build_dummy/g6-host/lib/libzenohpico.a)
set(sdk_locked_hashes
  6820ff1ae0d480f0f380a71d52cf8f08dcb50bba8b4c7486d3ed8c4f9148c6fa
  a069575a7eda2747036e8bf5b53bf0aa792b2ae9cb07ce7dfa6b309e3363ac0b
  a3077be71373c41ddde0a3b50acba5d933e14b3f303cdfb3096b374b10db0d50
  57748f3b30661cc1e90904f08d178a1b60663883bb5a4ddcfb26ef8d0f7173ab)
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

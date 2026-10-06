if(NOT DEFINED LOCK_FILE OR NOT DEFINED DPSDK_ROOT OR NOT DEFINED TEST_ROOT)
  message(FATAL_ERROR "LOCK_FILE, DPSDK_ROOT and TEST_ROOT are required")
endif()
# Symlink unchanged inputs so the 864 MB archive need not be copied. Replace
# exactly one input with altered bytes, then execute the configure-time checker.
set(paths
  contracts/dummy_robot_v2/codec/dp_dummy_robot_v2.hpp
  contracts/dummy_robot_v2/codec/dp_dummy_robot_v2_dp_comm.hpp
  build_dummy/g6-host/core/dp_sdk_core/dp_osal/libdp_osal.a
  build_dummy/g6-host/platform_common_console_device/libdp_console_device.a
  build_dummy/g6-host/lib/libzenohpico.a)
file(MAKE_DIRECTORY "${TEST_ROOT}")
foreach(changed IN LISTS paths)
  string(SHA256 case_id "${changed}")
  set(case_root "${TEST_ROOT}/${case_id}")
  foreach(path IN LISTS paths)
    get_filename_component(parent "${case_root}/${path}" DIRECTORY)
    file(MAKE_DIRECTORY "${parent}")
    if(path STREQUAL changed)
      file(WRITE "${case_root}/${path}" "intentional altered SDK input\n")
    else()
      file(CREATE_LINK "${DPSDK_ROOT}/${path}" "${case_root}/${path}" SYMBOLIC)
    endif()
  endforeach()
  execute_process(COMMAND "${CMAKE_COMMAND}" "-DDPSDK_ROOT=${case_root}" -P "${LOCK_FILE}"
                  RESULT_VARIABLE result OUTPUT_VARIABLE stdout ERROR_VARIABLE stderr)
  if(result EQUAL 0 OR NOT stderr MATCHES "DPSDK input SHA256 mismatch")
    message(FATAL_ERROR "Changed input was not rejected: ${changed}\n${stdout}\n${stderr}")
  endif()
endforeach()
message(STATUS "Published codec and both linked archive changes rejected")

# @file sdk_input_lock_tests.cmake
# @brief SDK输入锁正例与五项独立篡改负例：在隔离目录配置小工程，真实SDK原件只读。
# @details 必需LOCK_FILE校验脚本、DPSDK_ROOT源码根、DPSDK_HOST_SDK_PREFIX安装根和可写TEST_ROOT。
# 脚本运行会创建文件/符号链接并启动子CMake配置，不是无副作用文本检查；不执行固件/设备命令。
# 原件codec/归档全文身份由被include的LOCK_FILE检查，编译语义相同不能替代完整SHA一致。
if(NOT DEFINED LOCK_FILE OR NOT DEFINED DPSDK_ROOT OR NOT DEFINED TEST_ROOT OR
   NOT DEFINED DPSDK_HOST_SDK_PREFIX)
  message(FATAL_ERROR "LOCK_FILE, DPSDK_ROOT, TEST_ROOT与DPSDK_HOST_SDK_PREFIX必需")
endif()
# 在隔离配置工程分别损坏两codec与三安装归档，不修改真实源码/prefix。
# 两个codec分别验证消息实现头和dp_comm adapter头；集合不是所有合同文件的穷尽清单。
set(codecs
  applications/dummy_ref_control/contracts/dummy_robot_v2/codec/dp_dummy_robot_v2.hpp
  applications/dummy_ref_control/contracts/dummy_robot_v2/codec/dp_dummy_robot_v2_dp_comm.hpp)
# 三个已安装静态归档逐个验证，不能只核一个SDK配置文件就跳过真实库字节。
set(archives libdp_osal.a libdp_console_device.a libzenohpico.a)
file(MAKE_DIRECTORY "${TEST_ROOT}")
# 合成project使用NONE语言，只include锁检查；project_text中的转义换行会写成真实工程文本。
set(project_text "cmake_minimum_required(VERSION 3.16)\nproject(sdk_lock_probe NONE)\ninclude(\"${LOCK_FILE}\")\n")
file(MAKE_DIRECTORY "${TEST_ROOT}/positive")
file(WRITE "${TEST_ROOT}/positive/CMakeLists.txt" "${project_text}")
execute_process(COMMAND "${CMAKE_COMMAND}" -S "${TEST_ROOT}/positive" -B "${TEST_ROOT}/positive-build"
  "-DDPSDK_ROOT=${DPSDK_ROOT}" "-DDPSDK_HOST_SDK_PREFIX=${DPSDK_HOST_SDK_PREFIX}"
  RESULT_VARIABLE result OUTPUT_VARIABLE stdout ERROR_VARIABLE stderr)
if(NOT result EQUAL 0)
  message(FATAL_ERROR "原真实SDK正例失败:\n${stdout}\n${stderr}")
endif()
# 每次只破坏一件：其余项用符号链接指回原件，避免同时多项错误掩盖当前分支。
foreach(changed IN LISTS codecs archives)
  # case_id是路径字符串的摘要，用来隔离用例目录，不是产物内容SHA。
  string(SHA256 case_id "${changed}")
  set(case_root "${TEST_ROOT}/${case_id}")
  set(prefix "${case_root}/prefix")
  set(source "${case_root}/source")
  file(MAKE_DIRECTORY "${prefix}/lib/cmake/DPSDKHostSDK" "${source}" "${case_root}/project")
  file(CREATE_LINK "${DPSDK_HOST_SDK_PREFIX}/include" "${prefix}/include" SYMBOLIC)
  file(READ "${DPSDK_HOST_SDK_PREFIX}/lib/cmake/DPSDKHostSDK/DPSDKHostSDKConfig.cmake" config)
  # 复制SDK Config并把源码根重定向到假source；此替换会作用于配置文本中的全部匹配。
  string(REPLACE "${DPSDK_ROOT}" "${source}" config "${config}")
  file(WRITE "${prefix}/lib/cmake/DPSDKHostSDK/DPSDKHostSDKConfig.cmake" "${config}")
  file(COPY "${DPSDK_HOST_SDK_PREFIX}/lib/cmake/DPSDKHostSDK/DPSDKHostSDKInputs.cmake"
       DESTINATION "${prefix}/lib/cmake/DPSDKHostSDK")
  foreach(path IN LISTS codecs)
    get_filename_component(parent "${source}/${path}" DIRECTORY)
    file(MAKE_DIRECTORY "${parent}")
    if(path STREQUAL changed)
      file(WRITE "${source}/${path}" "intentional altered codec\n")
    else()
      file(CREATE_LINK "${DPSDK_ROOT}/${path}" "${source}/${path}" SYMBOLIC)
    endif()
  endforeach()
  foreach(archive IN LISTS archives)
    if(archive STREQUAL changed)
      file(WRITE "${prefix}/lib/${archive}" "intentional altered archive\n")
    else()
      file(CREATE_LINK "${DPSDK_HOST_SDK_PREFIX}/lib/${archive}" "${prefix}/lib/${archive}" SYMBOLIC)
    endif()
  endforeach()
  file(WRITE "${case_root}/project/CMakeLists.txt" "${project_text}")
  execute_process(COMMAND "${CMAKE_COMMAND}" -S "${case_root}/project" -B "${case_root}/build"
    "-DDPSDK_ROOT=${source}" "-DDPSDK_HOST_SDK_PREFIX=${prefix}"
    RESULT_VARIABLE result OUTPUT_VARIABLE stdout ERROR_VARIABLE stderr)
  # 负例必须非零且stderr命中身份错误；普通缺依赖/路径错误不能冒充篡改被识别。
  if(result EQUAL 0 OR NOT stderr MATCHES "SHA256 mismatch|安装归档身份不一致")
    message(FATAL_ERROR "变更未被正确拒绝: ${changed}\n${stdout}\n${stderr}")
  endif()
endforeach()
message(STATUS "两codec与三ROS PIC安装归档篡改均被拒绝，真实SDK正例通过")

/**
 * @file formal_bundle_identity_probe.cpp
 * @brief 只读探查指定包的SIL静态身份，明确没有runtime授权。
 *
 * 调用inspect_device_bundle_identity而不是load_device_profile运行资格入口；只要firmware_bound为true还会拒绝。打印FORMAL字样描述本probe静态边界，不等于正式机械运动许可。
 */
#include "dummy_motion_client/device_profile.hpp"
#include <iostream>
#include <exception>
/**
 * @brief 读取argv[1]包并检查未绑定固件的静态身份。
 * @param argc 参数个数（含程序名）。
 * @param argv 参数数组，模式/路径位置见下述说明。
 * @return 0表示所选检查通过；非零或断言/require异常表示失败。
 * 固定profile名sil_6dof；参数错2、firmware_bound为true3、异常1、通过0。无RPC、设备或ROS节点。
 */
int main(int argc,char** argv) {
  if(argc!=2)return 2;
  try {
    const auto identity=dummy_motion_client::inspect_device_bundle_identity(argv[1],"sil_6dof");
    if(identity.firmware_bound)return 3;
    std::cout << "FORMAL_BUNDLE_IDENTITY_VERIFIED_NO_RUNTIME_AUTHORITY\n";
    return 0;
  } catch(const std::exception& error) {
    std::cerr << error.what() << '\n';return 1;
  }
}

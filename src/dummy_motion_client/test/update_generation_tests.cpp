/**
 * @file update_generation_tests.cpp
 * @brief 核全局目标快照只有新逻辑更新产生generation。
 *
 * 进程内command channel；positions以rad表示目标而非测量反馈，固定身份只为本地准入。
 */
#include "dummy_ros_control/command_channel.hpp"
#include <array>
#include <iostream>
/**
 * @brief 发布一次后100次同水位consume不得出现重复代次。
 * @return 0表示所选检查通过；非零或断言/require异常表示失败。
 * consume_after0只见一次已有更新，随后用已消费generation查询应全空；clear身份收尾，不证明网络恰好发送一次。
 */
int main() {
  const std::array<double,6> positions{0,0,1,0,0,0};
  dummy_ros_control::set_command_identity({1,2,3,4,5});
  const auto generation=dummy_ros_control::publish_control_update(positions.data(),6,100,nullptr);
  const int fresh=dummy_ros_control::consume_control_update(0).has_value()?1:0;
  int duplicates=0;
  for(int write=0;write<100;++write)
    duplicates+=dummy_ros_control::consume_control_update(generation).has_value()?1:0;
  dummy_ros_control::clear_command_identity();
  if (!generation || fresh!=1 || duplicates) return 1;
  std::cout << "{\"fresh_generations\":" << fresh << ",\"duplicate_generations\":" << duplicates << "}\n";
}

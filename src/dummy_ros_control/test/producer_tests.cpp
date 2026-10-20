/**
 * @file producer_tests.cpp
 * @brief 验证CommandProducer快照代次与32位线序号回绕。
 *
 * 位置以rad保存，身份来自本地fixture；消费未产生新目标时不能增加代次。这里不执行串行发布/设备ACK。
 */
#include <array>
#include "dummy_ros_control/command_producer.hpp"
#include "dummy_ros_control/state_validator.hpp"

/**
 * @brief 检查重复读取无新值、相同位置重新提交仍产生新代次。
 *
 * 低32位线序号允许UINT32_MAX到0到1；辅助转换保留0，是否保留/跳过0由实际发送worker另行负责。半环有序比较只在规定窗口内有意义。
 * @return 0 表示本文件全部检查通过；非零定位失败检查。
 */
int main() {
  dummy_ros_control::CommandProducer producer;
  const dummy_ros_control::CommandIdentity hold{1U, 2U, 3U, 4U, 5U};
  const std::array<double, 6> position{0.1, 0.2, 0.7, 0.4, 0.5, 9.42477796076938};
  /**
   * @brief record_update是目标代次的唯一生产动作；consume_after只读取比水位新的快照。
   * 相同位置也可对应新的逻辑更新，不能用数值相等消除generation。
   */
  const auto first_generation = producer.record_update(hold, position);
  const auto first = producer.consume_after(0U);
  if (!first || first->generation != first_generation || first->position != position) return 1;
  if (producer.consume_after(first_generation).has_value()) return 2;
  for (unsigned write = 0; write < 100; ++write) {
    if (producer.consume_after(first_generation).has_value()) return 3;
  }
  const auto second_generation = producer.record_update(hold, position);
  const auto second = producer.consume_after(first_generation);
  if (!second || second->generation != second_generation || second_generation != first_generation + 1U)
    return 4;
  if (second->identity.ticket != hold.ticket || second->position != position) return 5;
  /**
   * @brief 验证uint64代次投影为低32位，包括回绕为0。
   * 后面的sequence_newer比较只检查有限串号距离的方向，不等同普通整数大小比较。
   */
  if (dummy_ros_control::wire_producer_sequence(0xFFFFFFFFULL) != 0xFFFFFFFFU ||
      dummy_ros_control::wire_producer_sequence(0x100000000ULL) != 0U ||
      dummy_ros_control::wire_producer_sequence(0x100000001ULL) != 1U) return 6;
  if (!dummy_ros_control::sequence_newer(1U, 0xFFFFFFFFU) ||
      dummy_ros_control::sequence_newer(0xFFFFFFFFU, 1U)) return 7;
  return 0;
}

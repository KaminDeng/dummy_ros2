/**
 * @file capability_limit_tests.cpp
 * @brief 检查位置模式能力匹配、六轴位置边界和心跳动作。
 *
 * 全部输入为本进程构造的固定值；不启动通信worker，也不代表设备已通过能力握手。
 * 角度使用rad，速度rad/s，加速度rad/s²；心跳时间是同一人工时钟域的µs。
 */
#include <array>
#include <cstdint>

#include "dummy_ros_control/device_transport.hpp"

/**
 * @brief 按成功和拒绝路径验证能力辅助函数。
 *
 * 先建立六轴能力，再单独检查J1掩码的能力匹配。六轴位置校验另有完整掩码要求，不能把二者支持范围混为一谈。
 * @return 0 表示本文件全部检查通过；非零定位失败检查。
 */
int main() {
  using namespace dummy_ros_control;
  /**
   * @brief 固定预期画像：0x3f表示J1至J6，10,000µs表示10ms。
   * 五种hash首字节故意不同，用于区分各身份字段；其余字节为零，并非部署身份。
   */
  PositionOnlyCapability expected{};
  expected.configured_mask = 0x3fU;
  expected.capabilities = kPositionOnlyCapability;
  expected.control_timebase_us = 10000U;
  expected.state_publish_period_us = 10000U;
  expected.policy_hash[0] = 1U;
  expected.contract_hash[0] = 2U;
  expected.model_hash[0] = 3U;
  expected.profile_hash[0] = 4U;
  expected.firmware_hash[0] = 5U;
  for (auto &limit : expected.limits) limit = {-1.0, 1.0};
  expected.velocity_max.fill(1.0);
  expected.acceleration_max.fill(2.0);

  /**
   * @brief 每次从预期复制后只改一个条件，避免前一负例污染后一负例。
   * 覆盖固件hash、策略hash存在性、控制周期和非正速度上界的拒绝。
   */
  auto observed = expected;
  if (!position_only_capability_matches(expected, observed)) return 1;
  expected.configured_mask = 0x01U;
  observed = expected;
  if (!position_only_capability_matches(expected, observed)) return 12;
  expected.configured_mask = 0x3fU;
  observed = expected;
  observed.firmware_hash[0] = 6U;
  if (position_only_capability_matches(expected, observed)) return 2;
  observed = expected;
  observed.has_policy_hash = false;
  if (position_only_capability_matches(expected, observed)) return 3;
  observed = expected;
  observed.control_timebase_us = 20000U;
  if (position_only_capability_matches(expected, observed)) return 4;
  observed = expected;
  observed.velocity_max[0] = 0.0;
  if (position_only_capability_matches(expected, observed)) return 5;

  /**
   * @brief ±1rad闭区间端点可接受；1.01rad越界与不完整六轴掩码应拒绝。
   */
  const std::array<double, 6> valid{0.0, 0.5, -0.5, 1.0, -1.0, 0.25};
  if (!position_only_positions_valid(valid, expected)) return 6;
  auto out_of_limit = valid;
  out_of_limit[3] = 1.01;
  if (position_only_positions_valid(out_of_limit, expected)) return 7;
  auto missing_axis = expected;
  missing_axis.configured_mask = 0x03U;
  if (position_only_positions_valid(valid, missing_axis)) return 8;

  /**
   * @brief 在人工接收时刻100µs后推进20ms/50ms，分别命中告警/Stop。
   * observe_state刷新心跳基点；这里只验证决策枚举，不发送Stop请求。
   */
  TransportHeartbeat heartbeat(10000U);
  heartbeat.observe_state(100U);
  if (heartbeat.poll(20100U) != HeartbeatAction::kWarn) return 9;
  if (heartbeat.poll(50100U) != HeartbeatAction::kStop) return 10;
  heartbeat.observe_state(50200U);
  if (heartbeat.poll(50200U) != HeartbeatAction::kNone) return 11;
  return 0;
}

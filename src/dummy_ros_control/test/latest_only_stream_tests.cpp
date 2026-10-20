/**
 * @file latest_only_stream_tests.cpp
 * @brief 检查单in-flight加最新pending的命令代次状态机。
 *
 * ACK仅由测试直接调用ack方法模拟；不包含网络可靠性、设备接收或物理完成证据。
 */
#include <array>

#include "dummy_ros_control/device_transport.hpp"

namespace {
/**
 * @brief 构造仅generation有效的最小夹具。
 * @param generation 命令发布代次，作为待发送/ACK关联键。
 * @return 其余字段零初始化的记录；不能直接视为已授权完整命令。
 */
dummy_ros_control::CommandGenerationRecord record(std::uint64_t generation) {
  dummy_ros_control::CommandGenerationRecord value{};
  value.generation = generation;
  return value;
}
}  // namespace

/**
 * @brief 验证pending被新值覆盖且只接受当前in-flight ACK。
 *
 * gen1在途时offer2再offer3，只保留3；ACK2和ACK0不能释放当前在途，ACK1后才可发送3；close_session丢弃在途及待发记录。
 * @return 0 表示本文件全部检查通过；非零定位失败检查。
 */
int main() {
  using namespace dummy_ros_control;
  LatestOnlyCommandStream stream;
  stream.offer(record(1U));
  const auto first = stream.begin_send();
  if (!first || first->generation != 1U || !stream.has_in_flight()) return 1;
  stream.offer(record(2U));
  stream.offer(record(3U));
  if (!stream.has_pending() || stream.pending_generation() != 3U) return 2;
  if (stream.acknowledge_committed(2U)) return 3;
  if (stream.begin_send().has_value()) return 4;
  if (!stream.acknowledge_committed(1U) || stream.has_in_flight()) return 5;
  const auto latest = stream.begin_send();
  if (!latest || latest->generation != 3U) return 6;
  if (stream.acknowledge_committed(0U)) return 7;
  if (!stream.acknowledge_committed(3U)) return 8;
  if (stream.has_in_flight() || stream.has_pending() || stream.begin_send().has_value()) return 9;
  stream.offer(record(4U));
  if (!stream.begin_send().has_value()) return 10;
  stream.close_session();
  if (stream.has_in_flight() || stream.has_pending() || stream.begin_send().has_value()) return 11;
  return 0;
}

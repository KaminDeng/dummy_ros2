/**
 * @file nonblocking_tests.cpp
 * @brief 检查有界FIFO、管理访问值对象及发布队列满后的恢复。
 *
 * 全部在未配置硬件对象中操作；不连接peer。名称nonblocking不代表测得了调度延迟或实时WCET，只断言有限容器的非等待返回语义。
 */
#include "dummy_ros_control/bounded_publish_queue.hpp"
#include "dummy_ros_control/management_access.hpp"
#include "dummy_ros_control/guarded_zenbedded_system.hpp"

/**
 * @brief 验证FIFO容量边界、丢弃计数和排空后同位置目标可再转移一次。
 *
 * 容量2的通用队列遵守FIFO；硬件发布队列容量4，1000次提交/transfer后应有996次drop，排空后可再转移一次相同位置目标；此处未比较重试generation。
 * @return 0 表示本文件全部检查通过；非零定位失败检查。
 */
int main() {
  dummy_ros_control::BoundedPublishQueue<2> queue;
  dummy_ros_control::CommandGenerationRecord first{};
  first.generation = 1U;
  dummy_ros_control::CommandGenerationRecord second{};
  second.generation = 2U;
  dummy_ros_control::CommandGenerationRecord third{};
  third.generation = 3U;
  if (!queue.try_push(first) || !queue.try_push(second)) return 1;
  if (queue.try_push(third) || queue.drops() != 1U) return 2;
  dummy_ros_control::CommandGenerationRecord output{};
  if (!queue.try_pop(&output) || output.generation != 1U) return 3;
  if (!queue.try_push(third)) return 4;
  if (!queue.try_pop(&output) || output.generation != 2U) return 5;
  if (!queue.try_pop(&output) || output.generation != 3U) return 6;
  if (queue.try_pop(&output) || queue.drops() != 1U) return 7;
  /**
   * @brief hardware_active=false且raw反馈条件满足时允许query/capture_reference。
   * with_hardware_active生成新访问快照；此处只检查活动标志和query仍允许，不断言活动态也允许capture。
   */
  const dummy_ros_control::ManagementAccess inactive(false, true);
  if (!inactive.can_query() || !inactive.can_capture_reference()) return 8;
  const auto active = inactive.with_hardware_active(true);
  if (!active.can_query() || !active.hardware_active()) return 9;
  dummy_ros_control::GuardedZenbeddedSystem system;
  const dummy_ros_control::CommandIdentity identity{1U, 2U, 3U, 4U, 5U};
  const std::array<double, 6> hold{0.1, 0.2, 0.7, 0.4, 0.5, 0.6};
  /**
   * @brief 同一rad目标提交1000次，每次submit返回非零；本用例未比较这些generation之间的关系。
   * 队列满后应有996次drop，排空后再transfer成功且位置仍为hold，随后没有更多待转移项；没有以generation断言确认它是哪次提交。
   */
  for (unsigned update = 0; update < 1000U; ++update) {
    if (system.submit_control_update(identity, hold) == 0U) return 10;
    system.transfer_control_update();
  }
  if (system.publish_drops() != 996U) return 11;
  while (system.take_publish_record(&output)) {}
  if (!system.transfer_control_update()) return 12;
  if (!system.take_publish_record(&output) || output.position != hold) return 13;
  if (system.transfer_control_update()) return 14;
  return 0;
}

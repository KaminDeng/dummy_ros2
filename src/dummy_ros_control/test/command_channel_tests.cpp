/**
 * @file command_channel_tests.cpp
 * @brief 检查进程内目标快照的generation、授权身份及替换边界。
 *
 * 目标hold以rad保存；这不是实测关节反馈。channel登记身份仅影响本地准入，不等于远端接受或执行了目标。
 */
#include <array>
#include "dummy_ros_control/command_channel.hpp"

/**
 * @brief 验证目标发布、最新值消费和退休身份隔离。
 *
 * generation是单调发布代次，consume的参数是已消费水位；ticket更新与motion_epoch变化具有不同的失效语义。
 * @return 0 表示本文件全部检查通过；非零定位失败检查。
 */
int main() {
  const std::array<double, 6> hold{0.1, 0.2, 0.7, 0.4, 0.5, 0.6};
  /**
   * @brief 清空授权后发布必须返回0；无有效身份时不能创建目标代次。
   */
  dummy_ros_control::clear_command_identity();
  if (dummy_ros_control::publish_control_update(hold.data(), hold.size(), 1, nullptr) != 0U) return 1;
  const dummy_ros_control::CommandIdentity identity{1U, 2U, 3U, 4U, 5U};
  dummy_ros_control::set_command_identity(identity);
  const auto first = dummy_ros_control::publish_control_update(hold.data(), hold.size(), 2, nullptr);
  /**
   * @brief 仅续票据而不换boot/session/reference/motion身份。
   * 已缓冲记录保留原ticket=5且仍属当前授权；下一次发布才携带ticket=6。
   */
  const dummy_ros_control::CommandIdentity refreshed{1U, 2U, 3U, 4U, 6U};
  dummy_ros_control::set_command_identity(refreshed);
  auto first_record = dummy_ros_control::consume_control_update(first - 1U);
  if (!first_record || !dummy_ros_control::command_record_current(*first_record) ||
      first_record->identity.ticket != 5U) return 15;
  const auto second = dummy_ros_control::publish_control_update(hold.data(), hold.size(), 3, nullptr);
  if (first == 0U || second != first + 1U) return 2;
  const auto latest = dummy_ros_control::consume_control_update(first);
  if (!latest || latest->generation != second || latest->identity.ticket != 6U) return 3;
  if (dummy_ros_control::consume_control_update(second).has_value()) return 4;
  /**
   * @brief 允许一轴和六轴目标；两轴长度是本协议实现明确拒绝的形状。
   */
  const auto joint1 = dummy_ros_control::publish_control_update(hold.data(), 1U, 4, nullptr);
  if (joint1 == 0U) return 6;
  const auto single = dummy_ros_control::consume_control_update(second);
  if (!single || single->position[0] != hold[0]) return 7;
  if (dummy_ros_control::publish_control_update(hold.data(), 2U, 4, nullptr) != 0U) return 8;
  /**
   * @brief 清空同时退休已有缓冲记录；重设同值身份也不能复活之前目标。
   */
  dummy_ros_control::clear_command_identity();
  if (dummy_ros_control::publish_control_update(hold.data(), hold.size(), 4, nullptr) != 0U) return 5;
  if (dummy_ros_control::consume_control_update(0U).has_value()) return 9;
  dummy_ros_control::set_command_identity(identity);
  if (dummy_ros_control::consume_control_update(0U).has_value()) return 10;
  if (dummy_ros_control::command_record_current(*single)) return 11;
  /**
   * @brief 用motion_epoch区分旧/新owner。
   * 按旧身份条件清理必须失败，以免旧回调撤销已替换的新owner；后续消费也不能吐出旧motion记录。
   */
  const dummy_ros_control::CommandIdentity old_identity{1U, 2U, 3U, 1U, 5U};
  const dummy_ros_control::CommandIdentity new_identity{1U, 2U, 3U, 2U, 5U};
  dummy_ros_control::set_command_identity(old_identity);
  const auto old_record_generation =
      dummy_ros_control::publish_control_update(hold.data(), hold.size(), 5, nullptr);
  dummy_ros_control::set_command_identity(new_identity);
  if (dummy_ros_control::clear_command_identity_if(old_identity)) return 12;
  if (dummy_ros_control::publish_control_update(hold.data(), hold.size(), 6, nullptr) == 0U) return 13;
  const auto old_record = dummy_ros_control::consume_control_update(old_record_generation - 1U);
  if (old_record && old_record->identity.motion_epoch == old_identity.motion_epoch) return 14;
  return 0;
}

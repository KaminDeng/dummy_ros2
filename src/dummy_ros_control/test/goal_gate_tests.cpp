/**
 * @file goal_gate_tests.cpp
 * @brief 验证不可变GoalGate值对象的忙闲和UUID/digest绑定。
 *
 * UUID和digest都是预填字节夹具；这里不计算真实轨迹摘要，也不建立ROS Action或远端授权。
 */
#include "dummy_ros_control/goal_registration_gate.hpp"

/**
 * @brief 检查登记/清理返回新gate以及错误凭据拒绝。
 *
 * register_goal和clear返回新值，原gate仍保持原状态；同一时刻只能由已登记UUID与digest组合获准。
 * @return 0 表示本文件全部检查通过；非零定位失败检查。
 */
int main() {
  using dummy_ros_control::GoalCredential;
  using dummy_ros_control::GoalRegistrationGate;

  /**
   * @brief 凭据同时绑定UUID和轨迹digest；两个错误夹具各只改其中一个维度。
   * 不允许只靠UUID相同或digest相同就取得另一目标的登记许可。
   */
  const GoalCredential registered{{1U}, {2U}};
  const GoalCredential wrong_uuid{{9U}, {2U}};
  const GoalCredential wrong_digest{{1U}, {8U}};
  const GoalRegistrationGate empty;
  if (empty.permits(registered) || empty.busy()) return 1;

  const auto active_result = empty.register_goal(registered);
  if (!active_result.has_value()) return 2;
  const auto active = active_result.value();
  if (!active.busy() || !active.permits(registered)) return 3;
  if (active.permits(wrong_uuid) || active.permits(wrong_digest)) return 4;
  if (active.register_goal(registered).has_value()) return 5;

  const auto cleared = active.clear();
  if (cleared.busy() || cleared.permits(registered)) return 6;
  return 0;
}

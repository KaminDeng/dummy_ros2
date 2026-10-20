/**
 * @file stale_state_tests.cpp
 * @brief 检查反馈包络的新鲜性、身份/掩码与票据窗口边界。
 *
 * 全部时刻均为同一人工µs域；未经过实际设备Source到主机Host映射。单独检查序号回绕关系和票据半开区间，不构成完整通信集成测试。
 */
#include "dummy_ros_control/state_validator.hpp"
#include "dummy_ros_control/freshness.hpp"

/**
 * @brief 逐项破坏StateExpectation并检查拒绝。
 *
 * 拒绝旧序号、错boot、缺轴、锚点前时间及超龄；UINT32_MAX→0按串号规则可新于旧值，半环距离不作为更新。ticket的截止时刻不含在有效窗内，溢出也必须拒绝。
 * @return 0 表示本文件全部检查通过；非零定位失败检查。
 */
int main() {
  using dummy_ros_control::StateEnvelope;
  using dummy_ros_control::StateExpectation;
  using dummy_ros_control::StateValidity;
  /**
   * @brief 预期boot9/profile7、已见seq100、配置63；请求锚1000µs，当前1100µs，最大年龄150µs。
   * 有效State seq101/produced1001µs；分别破坏一个维度以确认具体StateValidity枚举。
   */
  const StateExpectation expected{9U, 7U, 100U, true, 0x3FU, 1000U, 1100U, 150U};
  const StateEnvelope valid{9U, 7U, 101U, 1001U, 0x3FU, 0x3FU};
  if (dummy_ros_control::validate_state(valid, expected) != StateValidity::kValid) return 1;
  auto state = valid;
  state.state_seq = 99U;
  if (dummy_ros_control::validate_state(state, expected) != StateValidity::kOldSequence) return 2;
  state = valid;
  state.boot = 8U;
  if (dummy_ros_control::validate_state(state, expected) != StateValidity::kWrongIdentity) return 3;
  state = valid;
  state.position_valid_mask = 0x01U;
  if (dummy_ros_control::validate_state(state, expected) != StateValidity::kInvalidAxis) return 4;
  state = valid;
  state.produced_at_us = 949U;
  if (dummy_ros_control::validate_state(state, expected) != StateValidity::kBeforeRequestAnchor) return 5;
  state = valid;
  state.produced_at_us = 900U;
  /**
   * @brief 清零请求锚后再测试produced900µs，避免先被锚点前检查截获。
   * 这将超龄与请求锚点拒绝分开验证，不能只根据最终false猜测失败原因。
   */
  StateExpectation old_request = expected;
  old_request.request_started_at_us = 0U;
  if (dummy_ros_control::validate_state(state, old_request) != StateValidity::kStale) return 6;
  if (!dummy_ros_control::sequence_newer(0U, 0xFFFFFFFFU)) return 7;
  if (dummy_ros_control::sequence_newer(0x80000000U, 0U)) return 8;
  /**
   * @brief 票据有效窗为[0,50)µs；49接受、50及51拒绝。
   * 最后的UINT64_MAX样本检查issue+duration溢出必须失败关闭。
   */
  if (!dummy_ros_control::ticket_valid(0U, 49U, 50U)) return 9;
  if (dummy_ros_control::ticket_valid(0U, 50U, 50U)) return 10;
  if (dummy_ros_control::ticket_valid(0U, 51U, 50U)) return 11;
  if (dummy_ros_control::ticket_valid(UINT64_MAX - 10U, UINT64_MAX, 50U)) return 12;
  return 0;
}

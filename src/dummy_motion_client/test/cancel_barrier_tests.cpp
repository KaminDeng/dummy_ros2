/**
 * @file cancel_barrier_tests.cpp
 * @brief 核值状态机取消期间的生产屏障和终态分流。
 *
 * 只操作ActionCoordinator返回的新值，不发RPC或ROS Action；epoch/seq都是人工标识。
 */
#include "dummy_motion_client/action_coordinator.hpp"

/**
 * @brief 从Hold/Run进入Stopping，禁止旧成功与新Begin越过取消。
 * @return 0表示所选检查通过；非零或断言/require异常表示失败。
 * stop_ready仅到PrimeStopHold，须同新epoch保持样本accepted后才Canceled；非取消Stop归Aborted，stop_failed锁Fault。
 */
int main() {
  using dummy_motion_client::ActionCoordinator;
  using dummy_motion_client::Phase;
  using dummy_motion_client::Terminal;
  auto hold = ActionCoordinator{}.enable_accepted(1U)->hold_sample_accepted(1U, true).value();
  auto running = hold.begin_accepted(20U, 2U)->motion_sample_accepted(2U, true).value();
  auto stopping = running.request_stop(true);
  if (!stopping || stopping->producer_enabled || stopping->phase != Phase::kStopping) return 1;
  if (stopping->inner_succeeded(91U).has_value()) return 2;
  if (stopping->begin_accepted(21U, 3U).has_value()) return 3;
  auto stop_hold = stopping->stop_ready(3U);
  if (!stop_hold || stop_hold->phase != Phase::kPrimeStopHold) return 4;
  auto canceled = stop_hold->hold_sample_accepted(3U, true);
  if (!canceled || canceled->terminal != Terminal::kCanceled || canceled->goal != 0U) return 5;
  auto aborted_stop = running.request_stop(false)->stop_ready(3U)->hold_sample_accepted(3U, true);
  if (!aborted_stop || aborted_stop->terminal != Terminal::kAborted) return 6;
  const auto failed = running.request_stop(true)->stop_failed();
  return failed.phase == Phase::kFault && failed.terminal == Terminal::kAborted ? 0 : 7;
}

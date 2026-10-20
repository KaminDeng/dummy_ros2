/**
 * @file session_action_tests.cpp
 * @brief 核Enable→首采样→Begin→Complete→保持的值状态机生命周期。
 *
 * ActionCoordinator是返回新状态的内存对象，不发管理/ROS命令。terminal_producer_seq为末更新线序号，不是operation_id或物理到位计数。
 */
#include "dummy_motion_client/action_coordinator.hpp"

/**
 * @brief 每次新epoch先prime，完整完成Hold屏障后可开始第二目标。
 * @return 0表示所选检查通过；非零或断言/require异常表示失败。
 * 错误epoch或未accepted采样拒绝；内层成功仅进入completing并停producer，complete_stable后还须新Hold样本，第二Begin重置旧terminal_seq及终态。
 */
int main() {
  using dummy_motion_client::ActionCoordinator;
  using dummy_motion_client::Phase;
  using dummy_motion_client::Terminal;
  ActionCoordinator state;
  auto prime_hold = state.enable_accepted(1U);
  if (!prime_hold || prime_hold->phase != Phase::kPrimeHold || !prime_hold->producer_enabled) return 1;
  if (prime_hold->hold_sample_accepted(1U, false).has_value()) return 2;
  auto hold = prime_hold->hold_sample_accepted(1U, true);
  if (!hold || hold->phase != Phase::kHold) return 3;
  auto prime = hold->begin_accepted(10U, 2U);
  if (!prime || prime->phase != Phase::kPrimeMotion) return 4;
  if (prime->begin_accepted(11U, 3U).has_value()) return 5;
  if (prime->motion_sample_accepted(1U, true).has_value()) return 6;
  auto running = prime->motion_sample_accepted(2U, true);
  if (!running || running->phase != Phase::kRunning) return 7;
  auto completing = running->inner_succeeded(88U);
  if (!completing || completing->producer_enabled || completing->terminal_producer_seq != 88U) return 8;
  auto completion_hold = completing->complete_stable(3U);
  if (!completion_hold || completion_hold->phase != Phase::kPrimeCompletionHold) return 9;
  auto succeeded = completion_hold->hold_sample_accepted(3U, true);
  if (!succeeded || succeeded->phase != Phase::kHold || succeeded->terminal != Terminal::kSucceeded)
    return 10;
  auto second = succeeded->begin_accepted(11U, 4U);
  if (second && second->terminal_producer_seq != 0U) return 12;
  return second && second->terminal == Terminal::kNone && second->goal == 11U ? 0 : 11;
}

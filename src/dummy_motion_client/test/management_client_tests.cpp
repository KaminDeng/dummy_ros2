/**
 * @file management_client_tests.cpp
 * @brief 核管理操作值追踪器中Queued、Accepted和结果关联ID。
 *
 * 不实例化wire传输或设备；所有ID用大于2^53的uint64夹具验证精度/逻辑关联，不能以Queued产生operation_id。
 */
#include "dummy_motion_client/action_coordinator.hpp"

/**
 * @brief 只允许原request接受正确operation，再只接受该operation结果。
 * @return 0表示所选检查通过；非零或断言/require异常表示失败。
 * Queued的receipt仍kQueued/statusWaiting/op0；错误request接受或错误operation结果返回空；精确operation后结果才Succeeded。
 */
int main() {
  using dummy_motion_client::ManagementOperationTracker;
  using dummy_motion_client::OperationStatus;
  using dummy_motion_client::Receipt;
  ManagementOperationTracker tracker;
  auto queued = tracker.queued(9007199254740993ULL);
  if (!queued || queued->receipt != Receipt::kQueued || queued->status != OperationStatus::kWaiting ||
      queued->operation_id != 0U) return 1;
  if (queued->result(7U, true).has_value()) return 2;
  if (queued->accepted(8U, 7U).has_value()) return 3;
  auto accepted = queued->accepted(9007199254740993ULL, 9007199254740995ULL);
  if (!accepted || accepted->status != OperationStatus::kAccepted) return 4;
  if (accepted->result(2U, true).has_value()) return 5;
  auto completed = accepted->result(9007199254740995ULL, true);
  return completed && completed->status == OperationStatus::kSucceeded ? 0 : 6;
}

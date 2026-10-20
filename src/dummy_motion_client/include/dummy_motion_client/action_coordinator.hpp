/**
 * @file action_coordinator.hpp
 * @brief ROS Action 的纯值状态转换及管理事务接纳/终态跟踪。
 * @details 所有转换返回副本；调用方须检查 optional 并保存新状态。本文件不发 RPC、不运行电机，也不自行验证反馈，continuous 等输入须由上层建立事实。
 */
#pragma once

#include <cstdint>
#include <optional>

namespace dummy_motion_client {

/**
 * @brief Action 生命周期；Prime 表示等待新代次首个连续样本，Hold 表示已建立保持。
 * @details Running 后先 Completing，再进入新的 PrimeCompletionHold；停车也要 Stopping→PrimeStopHold。不能将内层成功或停止请求直接视为对外终态。
 */
enum class Phase { kIdle, kPrimeHold, kHold, kPrimeMotion, kRunning, kCompleting,
                   kPrimeCompletionHold, kStopping, kPrimeStopHold, kFault };
/**
 * @brief 对外动作结果；None 表示本协调器尚未建立成功、取消或中止终态。
 */
enum class Terminal { kNone, kSucceeded, kCanceled, kAborted };

/**
 * @brief 不含传输对象的协调器快照；const 转换失败返回 nullopt，不修改原值。
 * @details goal 为上层目标编号，motion_epoch 为当前运动/保持代次；terminal_producer_seq 保存完成时冻结的末序号。producer_enabled 只是上层是否继续生产样本的意图。
 */
struct ActionCoordinator {
  Phase phase = Phase::kIdle;
  Terminal terminal = Terminal::kNone;
  std::uint64_t goal = 0;
  std::uint32_t motion_epoch = 0;
  std::uint32_t terminal_producer_seq = 0;
  bool producer_enabled = false;
  bool cancel_terminal = false;

  /**
   * @brief Idle 收到非零保持代次后进入 PrimeHold，开启首样本生产。
   * @param hold_epoch 已由外层接纳的保持代次；本方法不判断设备是否已执行。
   * @return 新快照；非 Idle 或代次为零则无转换。
   */
  std::optional<ActionCoordinator> enable_accepted(std::uint32_t hold_epoch) const {
    if (phase != Phase::kIdle || hold_epoch == 0U) return std::nullopt;
    auto next = *this;
    next.phase = Phase::kPrimeHold;
    next.motion_epoch = hold_epoch;
    next.producer_enabled = true;
    return next;
  }

  /**
   * @brief 用同代次的连续保持样本完成三类 PrimeHold 握手。
   * @param epoch 观察到的保持代次，必须等于当前 motion_epoch。
   * @param continuous 上层已确认的连续样本条件；false 不推进。
   * @return 进入 Hold 并清 goal 的快照。完成握手标记 Succeeded；停车握手按 cancel_terminal 标记 Canceled/Aborted；初次使能不另造终态。
   */
  std::optional<ActionCoordinator> hold_sample_accepted(std::uint32_t epoch,
                                                         bool continuous) const {
    if ((phase != Phase::kPrimeHold && phase != Phase::kPrimeCompletionHold &&
         phase != Phase::kPrimeStopHold) || epoch != motion_epoch || !continuous)
      return std::nullopt;
    auto next = *this;
    next.phase = Phase::kHold;
    next.producer_enabled = true;
    if (phase == Phase::kPrimeCompletionHold) next.terminal = Terminal::kSucceeded;
    if (phase == Phase::kPrimeStopHold)
      next.terminal = cancel_terminal ? Terminal::kCanceled : Terminal::kAborted;
    next.goal = 0;
    return next;
  }

  /**
   * @brief 从 Hold 接纳非零目标及严格递增的运动代次，进入 PrimeMotion。
   * @details 清上次终态和末序号，保留生产开启；这一步只描述 Begin 被接纳，尚未达到 Running。
   * @return 不满足阶段、目标或代次约束时返回 nullopt。
   */
  std::optional<ActionCoordinator> begin_accepted(std::uint64_t goal_id,
                                                   std::uint32_t epoch) const {
    if (phase != Phase::kHold || goal_id == 0U || epoch <= motion_epoch) return std::nullopt;
    auto next = *this;
    next.phase = Phase::kPrimeMotion;
    next.terminal = Terminal::kNone;
    next.terminal_producer_seq = 0;
    next.goal = goal_id;
    next.motion_epoch = epoch;
    next.producer_enabled = true;
    return next;
  }

  /**
   * @brief PrimeMotion 的同代次连续首样本到达后进入 Running。
   * @return 阶段/代次/连续性任一不符则拒绝转换。
   */
  std::optional<ActionCoordinator> motion_sample_accepted(std::uint32_t epoch,
                                                           bool continuous) const {
    if (phase != Phase::kPrimeMotion || epoch != motion_epoch || !continuous) return std::nullopt;
    auto next = *this;
    next.phase = Phase::kRunning;
    return next;
  }

  /**
   * @brief 内层轨迹成功后冻结非零末序号，停生产并进入 Completing。
   * @param accepted_seq 上层确认的末个接纳序号；这里只检查非零。
   * @details 对外成功仍须等待 Complete 稳定及新保持握手。
   */
  std::optional<ActionCoordinator> inner_succeeded(std::uint32_t accepted_seq) const {
    if (phase != Phase::kRunning || accepted_seq == 0U) return std::nullopt;
    auto next = *this;
    next.phase = Phase::kCompleting;
    next.terminal_producer_seq = accepted_seq;
    next.producer_enabled = false;
    return next;
  }

  /**
   * @brief Completing 收到严格更新的保持代次后恢复生产，等待完成保持首样本。
   * @return PrimeCompletionHold 快照；尚不在这里设置 Succeeded。
   */
  std::optional<ActionCoordinator> complete_stable(std::uint32_t hold_epoch) const {
    if (phase != Phase::kCompleting || hold_epoch <= motion_epoch) return std::nullopt;
    auto next = *this;
    next.phase = Phase::kPrimeCompletionHold;
    next.motion_epoch = hold_epoch;
    next.producer_enabled = true;
    return next;
  }

  /**
   * @brief 仅在待运动首样本、运行或完成阶段接纳停车意图并停生产。
   * @param canceled true 最终拟报取消，false 最终拟报中止；实际终态仍等待停车和新保持。
   * @details 已在 Stopping 时返回 nullopt，不重复重置停车流程。
   */
  std::optional<ActionCoordinator> request_stop(bool canceled) const {
    if (phase != Phase::kPrimeMotion && phase != Phase::kRunning && phase != Phase::kCompleting)
      return std::nullopt;
    auto next = *this;
    next.phase = Phase::kStopping;
    next.producer_enabled = false;
    next.cancel_terminal = canceled;
    return next;
  }

  /**
   * @brief Stopping 收到更大保持代次后进入 PrimeStopHold，恢复保持样本生产。
   * @details 必须等 hold_sample_accepted 后才能设置 Canceled/Aborted。
   */
  std::optional<ActionCoordinator> stop_ready(std::uint32_t hold_epoch) const {
    if (phase != Phase::kStopping || hold_epoch <= motion_epoch) return std::nullopt;
    auto next = *this;
    next.phase = Phase::kPrimeStopHold;
    next.motion_epoch = hold_epoch;
    next.producer_enabled = true;
    return next;
  }

  /**
   * @brief 停车失败时无条件返回 Fault/Aborted 快照并禁止继续生产。
   * @details 本方法不发送失能，也不清 goal/代次；这些动作由持有快照的上层负责。
   */
  ActionCoordinator stop_failed() const {
    auto next = *this;
    next.phase = Phase::kFault;
    next.terminal = Terminal::kAborted;
    next.producer_enabled = false;
    return next;
  }
};

/**
 * @brief 本地管理提交收据；与设备 operation 终态分开。
 */
enum class Receipt { kNone, kQueued, kRejected };
/**
 * @brief 单笔管理跟踪：Idle→Waiting→Accepted→Succeeded/Failed。
 */
enum class OperationStatus { kIdle, kWaiting, kAccepted, kSucceeded, kFailed };

/**
 * @brief 用请求编号和设备操作编号分别关联接纳与最终结果。
 * @details 字段公开，默认转换只覆盖 queued/accepted/result 路径，不隐含重试或重置。
 */
struct ManagementOperationTracker {
  std::uint64_t request_id = 0;
  std::uint64_t operation_id = 0;
  Receipt receipt = Receipt::kNone;
  OperationStatus status = OperationStatus::kIdle;

  /**
   * @brief Idle 接纳非零本地 request，等待设备返回 operation。
   */
  std::optional<ManagementOperationTracker> queued(std::uint64_t request) const {
    if (status != OperationStatus::kIdle || request == 0U) return std::nullopt;
    return ManagementOperationTracker{request, 0U, Receipt::kQueued, OperationStatus::kWaiting};
  }
  /**
   * @brief 只将同一 Waiting 请求关联到非零设备 operation；收据仍保留 Queued。
   */
  std::optional<ManagementOperationTracker> accepted(std::uint64_t request,
                                                      std::uint64_t operation) const {
    if (status != OperationStatus::kWaiting || request != request_id || operation == 0U)
      return std::nullopt;
    return ManagementOperationTracker{request, operation, receipt, OperationStatus::kAccepted};
  }
  /**
   * @brief 同一 Accepted operation 的终态才可结束跟踪；不以 request_id 代替 operation_id。
   */
  std::optional<ManagementOperationTracker> result(std::uint64_t operation, bool success) const {
    if (status != OperationStatus::kAccepted || operation != operation_id) return std::nullopt;
    auto next = *this;
    next.status = success ? OperationStatus::kSucceeded : OperationStatus::kFailed;
    return next;
  }
};
}  // namespace dummy_motion_client

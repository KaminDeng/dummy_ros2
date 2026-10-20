/**
 * @file bootstrap_tick_policy.hpp
 * @brief 缺少普通观测时仍有界推进既有 Release 请求的 tick 判据。
 */
#pragma once
#include "dummy_motion_client/bootstrap_coordinator.hpp"
namespace dummy_motion_client {
/**
 * @brief 决定当前控制回调是否应继续调用 bootstrap.tick。
 * @param observation_ready 正常观测已满足上层前提。
 * @param observation_lost 上层已确认观测丢失，需要状态机处理失败。
 * @param release_transition_pending 已识别的释放过渡窗口。
 * @param release_stage 上层释放阶段；只有值2允许末项回退路径。
 * @param phase 当前 bootstrap 阶段。
 * @param operation 当前管理操作。
 * @param request_inflight 已存在且尚未结束的本次请求。
 * @return 任一前三项成立，或阶段2中已经发出的 DisableAndRelease 尚在 ReleasePending。后者只收尾原事务，不创建权限或补造反馈。
 */
inline bool bootstrap_tick_required(bool observation_ready, bool observation_lost,
    bool release_transition_pending, int release_stage,
    BootstrapPhase phase, ManageOperation operation, bool request_inflight) noexcept {
  // 只继续已发送的本次Release管理事务；不建立观测或授予运动。
  return observation_ready || observation_lost || release_transition_pending ||
      (release_stage==2 && phase==BootstrapPhase::kReleasePending &&
       operation==ManageOperation::kDisableAndRelease && request_inflight);
}
}

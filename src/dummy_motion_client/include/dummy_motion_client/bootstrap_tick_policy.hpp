#pragma once
#include "dummy_motion_client/bootstrap_coordinator.hpp"
namespace dummy_motion_client {
inline bool bootstrap_tick_required(bool observation_ready, bool observation_lost,
    bool release_transition_pending, int release_stage,
    BootstrapPhase phase, ManageOperation operation, bool request_inflight) noexcept {
  // 只继续已发送的本次Release管理事务；不建立观测或授予运动。
  return observation_ready || observation_lost || release_transition_pending ||
      (release_stage==2 && phase==BootstrapPhase::kReleasePending &&
       operation==ManageOperation::kDisableAndRelease && request_inflight);
}
}

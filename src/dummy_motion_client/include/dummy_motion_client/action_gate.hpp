#pragma once
#include <string>
#include <vector>
#include "dummy_motion_client/action_coordinator.hpp"
#include "dummy_motion_client/management_client.hpp"
namespace dummy_motion_client {
struct TrajectoryPoint { std::vector<double> positions, velocities, accelerations; double time=0; };
struct Trajectory {
  std::vector<std::string> joints;
  std::vector<TrajectoryPoint> points;
  std::vector<double> path_tolerance, goal_tolerance;
  double goal_time_tolerance=0;
};
struct JointProfile {
  std::vector<std::string> joints;
  std::vector<double> lower, upper, velocity, acceleration, path_tolerance, goal_tolerance;
};
std::optional<Trajectory> validate_trajectory(const Trajectory &, const JointProfile &,
                                              const Position *measured_start=nullptr);
struct Authorization {
  ManageRequest identity{};
  bool session=false, reference=false, permit=false;
};
struct CancelReceipt {
  std::uint64_t request_id=0, operation_id=0;
};
struct HoldDrainEvidence {
  std::uint64_t boot_epoch=0, session_id=0, processed_generation=0;
  std::uint32_t reference_epoch=0, motion_epoch=0, committed_seq=0;
  // 1: legitimate local no-op; 2: a device-acknowledged CAN group. Never a queued write.
  std::uint8_t disposition=0;
  bool quiescent=false;
  std::uint64_t observed_us=0, max_state_age_us=0;
  Position position{}, tolerance{};
};
class ActionPort {
 public:
  virtual ~ActionPort() = default;
  // Called only after core acceptance: the actual update callback must publish the prime.
  virtual void prime(const Position &, std::uint32_t epoch, std::uint64_t generation)=0;
  virtual void start(const Trajectory &, std::uint64_t generation)=0;
  virtual void disable()=0;
  virtual void cancel()=0;
  virtual bool freeze_hold() { return false; }
};
// Serialize these nonblocking event handlers on the caller's control executor.
// The port owns the actual JTC registration and update sink; no RobotCore is copied here.
class ActionGate {
 public:
  ActionGate(ManagementClient &, ActionPort &, JointProfile, std::uint64_t timeout_us=1000000);
  bool enable(const Authorization &, std::uint64_t now);
  bool renew_permit(const Authorization &);
  bool submit(const Trajectory &, std::uint64_t now);
  void tick(std::uint64_t now);
  void observe(const ManageReply &, std::uint64_t now);
  bool observe_state(const std::uint8_t *bytes, std::size_t size, std::uint64_t now);
  bool record_update(std::uint64_t generation, std::uint32_t epoch,
                     std::uint32_t seq, const Position &, std::uint64_t producer_generation=0);
  void observe_hold_drain(const HoldDrainEvidence &, std::uint64_t now);
  void inner_result(std::uint64_t generation, Terminal, const std::string &, std::uint64_t now);
  void cancel(std::uint64_t now);
  void abort(const std::string &, std::uint64_t now);
  void invalidate_authorization(std::uint64_t now);
  bool ready() const { return phase_==Phase::kHold && request_==0 &&
      !hold_drain_pending_ && authorization_.permit; }
  bool permit_refresh_allowed() const {
    return phase_==Phase::kHold && request_==0 && authorization_.session &&
        authorization_.reference && !authorization_.permit;
  }
  Terminal terminal() const { return terminal_; }
  std::optional<CancelReceipt> confirmed_cancel_receipt() const noexcept;
  Phase phase() const { return phase_; }
  std::uint64_t generation() const { return generation_; }
  const std::string &error() const { return error_; }
 private:
  bool issue(ManageOperation, std::uint64_t now, std::uint64_t existing_deadline=0);
  bool prime(const ManageReply &, Phase, std::uint64_t now);
  void stop(bool canceled, const std::string &, std::uint64_t now);
  void fail(const std::string &);
  bool handshake(const ManageReply &) const;
  void advance(const ManageReply &, std::uint64_t now);
  void try_completion_hold(std::uint64_t now);
  ManagementClient &client_; ActionPort &port_; const JointProfile profile_;
  const std::uint64_t timeout_;
  Authorization authorization_{}; Trajectory trajectory_{};
  Phase phase_=Phase::kIdle; Terminal terminal_=Terminal::kNone;
  ManageOperation operation_=ManageOperation::kEnable;
  std::uint64_t request_=0, deadline_=0, generation_=0;
  std::uint64_t cancel_request_id_=0;
  std::uint64_t last_producer_generation_=0, frozen_generation_=0, drained_generation_=0;
  std::uint64_t prime_producer_generation_=0, hold_state_observed_us_=0;
  HoldDrainEvidence completion_worker_{};
  std::uint32_t epoch_=0, last_seq_=0, frozen_seq_=0, prime_seq_=0;
  std::uint32_t hold_committed_seq_=0;
  std::uint32_t drained_committed_seq_=0;
  std::uint8_t hold_submitted_mask_=0, hold_sent_mask_=0;
  Position last_q_{}, prime_q_{}, frozen_q_{};
  bool sampled_=false, frozen_=false, cancel_terminal_=false, hold_drain_pending_=false;
  bool drain_state_fresh_=false, drain_worker_ready_=false;
  std::string error_;
  ManageReply management_evidence_{}, state_evidence_{};
  std::uint32_t state_seq_=0;
  bool state_seen_=false;
};
}  // namespace dummy_motion_client

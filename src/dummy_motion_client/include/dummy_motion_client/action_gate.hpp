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
std::optional<Trajectory> validate_trajectory(const Trajectory &, const JointProfile &);
struct Authorization {
  ManageRequest identity{};
  bool session=false, reference=false, permit=false;
};
class ActionPort {
 public:
  virtual ~ActionPort() = default;
  // Called only after core acceptance: the actual update callback must publish the prime.
  virtual void prime(const Position &, std::uint32_t epoch, std::uint64_t generation)=0;
  virtual void start(const Trajectory &, std::uint64_t generation)=0;
  virtual void disable()=0;
  virtual void cancel()=0;
};
// Serialize these nonblocking event handlers on the caller's control executor.
// The port owns the actual JTC registration and update sink; no RobotCore is copied here.
class ActionGate {
 public:
  ActionGate(ManagementClient &, ActionPort &, JointProfile, std::uint64_t timeout_us=1000000);
  bool enable(const Authorization &, std::uint64_t now);
  bool submit(const Trajectory &, std::uint64_t now);
  void tick(std::uint64_t now);
  void observe(const ManageReply &, std::uint64_t now);
  bool observe_state(const std::uint8_t *bytes, std::size_t size, std::uint64_t now);
  bool record_update(std::uint64_t generation, std::uint32_t epoch,
                     std::uint32_t seq, const Position &);
  void inner_result(std::uint64_t generation, Terminal, const std::string &, std::uint64_t now);
  void cancel(std::uint64_t now);
  void abort(const std::string &, std::uint64_t now);
  void invalidate_authorization(std::uint64_t now);
  bool ready() const { return phase_==Phase::kHold && request_==0; }
  Terminal terminal() const { return terminal_; }
  std::uint64_t generation() const { return generation_; }
  const std::string &error() const { return error_; }
 private:
  bool issue(ManageOperation, std::uint64_t now);
  void prime(const ManageReply &, Phase);
  void stop(bool canceled, const std::string &, std::uint64_t now);
  void fail(const std::string &);
  bool handshake(const ManageReply &) const;
  void advance(const ManageReply &, std::uint64_t now);
  ManagementClient &client_; ActionPort &port_; const JointProfile profile_;
  const std::uint64_t timeout_;
  Authorization authorization_{}; Trajectory trajectory_{};
  Phase phase_=Phase::kIdle; Terminal terminal_=Terminal::kNone;
  ManageOperation operation_=ManageOperation::kEnable;
  std::uint64_t request_=0, deadline_=0, generation_=0;
  std::uint32_t epoch_=0, last_seq_=0, frozen_seq_=0, prime_seq_=0;
  Position last_q_{}, prime_q_{}, frozen_q_{};
  bool sampled_=false, frozen_=false, cancel_terminal_=false;
  std::string error_;
  ManageReply management_evidence_{}, state_evidence_{};
  std::uint32_t state_seq_=0;
};
}  // namespace dummy_motion_client

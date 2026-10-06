#pragma once
#include <array>
#include <string>
#include "dummy_motion_client/action_gate.hpp"
#include "dummy_motion_client/management_client.hpp"
namespace dummy_motion_client {
struct DpManagementParameters {
  std::string endpoint;
  std::array<std::uint8_t,16> device{}, client{};
  std::uint32_t profile=0;
  std::uint8_t mask=0;
};
struct DeviceObservation {
  std::vector<std::uint8_t> state, status;
  std::uint64_t state_received_us=0, status_received_us=0;
  std::uint64_t rejected_ack_boot_epoch=0, rejected_ack_session_id=0, rejected_ack_received_us=0;
  std::uint32_t rejected_ack_reference_epoch=0, rejected_ack_motion_epoch=0,
                rejected_ack_sequence=0;
};
struct RawTerminalIdentity {
  std::uint64_t boot_epoch=0, session_id=0, run_id=0;
  std::uint64_t parent_run_id=0;
  std::uint8_t attempt_kind=0, final_state=0;
  std::uint8_t first_failure_source=0, first_failure_axis_mask=0;
  bool evidence_lost=false, motion_complete=false, release_committed=false;
  bool chain_valid=false;
  std::uint32_t safety_hold_motion_epoch=0;
  bool safety_hold_valid=false;
};
struct RawFirstPageWritten {
  std::uint64_t boot_epoch=0U, session_id=0U, run_id=0U;
  std::uint32_t motion_epoch=0U, evidence_seq=0U, group_count=0U;
  std::uint8_t attempt_kind=0U, category=0U;
  std::uint8_t expected_mask=0U, submitted_mask=0U, sent_mask=0U, failed_mask=0U;
  bool final=false, evidence_lost=false, chain_valid=false, full_six=false;
  bool strict_refresh_successor=false;
};
inline bool raw_first_page_matches(const RawFirstPageWritten &receipt,
                                   std::uint64_t boot, std::uint64_t session,
                                   std::uint64_t run, std::uint32_t motion_epoch) noexcept {
  return boot!=0U && session!=0U && run!=0U && motion_epoch!=0U &&
      receipt.boot_epoch==boot && receipt.session_id==session &&
      receipt.run_id==run && receipt.motion_epoch==motion_epoch;
}
inline bool raw_terminal_matches(const RawTerminalIdentity &terminal,
                                 std::uint64_t boot,std::uint64_t session,
                                 std::uint64_t run) noexcept {
  return run!=0U && terminal.boot_epoch==boot &&
    terminal.session_id==session && terminal.run_id==run;
}
inline bool observation_is_fresh(const DeviceObservation &observation,std::uint64_t now,
                                 std::uint64_t state_max_age_us=250000U) noexcept {
  if(!observation.state_received_us || !observation.status_received_us) return false;
  // Receipt stamps are local. A receive after the caller sampled `now` has zero age.
  const auto state_age=now>observation.state_received_us ? now-observation.state_received_us : 0;
  const auto status_age=now>observation.status_received_us ? now-observation.status_received_us : 0;
  return state_age<=state_max_age_us && status_age<=1000000U;
}
class ObservationContinuity {
 public:
  enum class Result { kReady, kWait, kLost };
  ObservationContinuity(std::uint64_t state_max_age_us,
                        std::uint64_t bootstrap_limit_us,
                        std::uint64_t running_limit_us) noexcept
      : state_max_age_us_(state_max_age_us), bootstrap_limit_us_(bootstrap_limit_us),
        running_limit_us_(running_limit_us) {}
  Result evaluate(const DeviceObservation &observation, std::uint64_t now,
                  bool enabled, bool coherent) noexcept {
    const bool state_live=observation.state_received_us!=0U &&
      (now<=observation.state_received_us || now-observation.state_received_us<=state_max_age_us_);
    const bool status_live=observation.status_received_us!=0U &&
      (now<=observation.status_received_us || now-observation.status_received_us<=1000000U);
    if(state_live && (enabled || status_live) && coherent) {
      interruption_started_us_=0U;
      return Result::kReady;
    }
    if(interruption_started_us_==0U) interruption_started_us_=now;
    const auto limit=enabled ? running_limit_us_ : bootstrap_limit_us_;
    return now-interruption_started_us_>limit ? Result::kLost : Result::kWait;
  }
 private:
  std::uint64_t state_max_age_us_;
  std::uint64_t bootstrap_limit_us_, running_limit_us_;
  std::uint64_t interruption_started_us_=0U;
};
struct StateAuthority {
  std::uint64_t boot_epoch=0U, session_id=0U;
  std::uint32_t reference_epoch=0U, motion_epoch=0U;
};
struct ReleaseTransitionObservation {
  StateAuthority original{}, status{}, state{};
  std::uint32_t status_management_epoch=0U, state_management_epoch=0U,
                original_management_epoch=0U;
  std::uint16_t status_core_state=0U, state_core_state=0U;
  int release_stage=0;
};
class ReleaseTransitionOrigin {
 public:
  void capture(StateAuthority authority, std::uint32_t management_epoch) noexcept {
    authority_=authority;
    management_epoch_=management_epoch;
  }
  ReleaseTransitionObservation observation(StateAuthority status, StateAuthority state,
      std::uint32_t status_management_epoch, std::uint32_t state_management_epoch,
      std::uint16_t status_core_state, std::uint16_t state_core_state,
      int release_stage) const noexcept {
    return {authority_,status,state,status_management_epoch,state_management_epoch,
            management_epoch_,status_core_state,state_core_state,release_stage};
  }
 private:
  StateAuthority authority_{};
  std::uint32_t management_epoch_=0U;
};
template <typename Status, typename State>
bool released_idle_status_confirmed(const Status &status, const State &state) noexcept {
  return status.boot_epoch()!=0U && status.boot_epoch()==state.boot_epoch() &&
    status.motion_epoch()==state.motion_epoch() &&
    status.management_epoch()==state.management_epoch() &&
    status.core_state()==1U && state.core_state()==1U &&
    status.session_id()==0U && state.session_id()==0U &&
    status.reference_epoch()==0U && state.reference_epoch()==0U &&
    status.release_committed() && !status.real_motion_permitted() &&
    status.permit_remaining_us()==0U;
}
inline bool release_idle_transition_pending(const ReleaseTransitionObservation &value,
                                            const DeviceObservation &observation,
                                            std::uint64_t now,
                                            std::uint64_t state_max_age_us) noexcept {
  return value.release_stage==2 && value.original.boot_epoch!=0U &&
    value.original.session_id!=0U && value.original.reference_epoch!=0U &&
    value.status.boot_epoch==value.original.boot_epoch &&
    value.status.session_id==value.original.session_id &&
    value.status.reference_epoch==value.original.reference_epoch &&
    value.status_core_state==7U && value.state_core_state==1U &&
    value.state.boot_epoch==value.original.boot_epoch &&
    value.state.session_id==0U && value.state.reference_epoch==0U &&
    value.original.motion_epoch!=UINT32_MAX &&
    value.status.motion_epoch==value.original.motion_epoch+1U &&
    value.state.motion_epoch==value.status.motion_epoch &&
    value.original_management_epoch!=UINT32_MAX &&
    value.status_management_epoch==value.original_management_epoch+1U &&
    value.status_management_epoch==value.state_management_epoch &&
    observation_is_fresh(observation,now,state_max_age_us);
}
inline bool state_authority_is_valid(const StateAuthority &state,
                                     const StateAuthority &expected,
                                     bool fault, bool motion_stable) noexcept {
  if(fault || state.boot_epoch!=expected.boot_epoch || state.session_id!=expected.session_id ||
     state.reference_epoch!=expected.reference_epoch || state.motion_epoch<expected.motion_epoch)
    return false;
  const auto motion_advance=state.motion_epoch-expected.motion_epoch;
  return motion_advance==0U || (!motion_stable && motion_advance==1U);
}
class DpManagementTransport final : public ManagementTransport {
 public:
  enum class Connection { kStandaloneWorker, kDeviceWorker };
  explicit DpManagementTransport(DpManagementParameters, Connection=Connection::kStandaloneWorker);
  ~DpManagementTransport() override;
  bool start();
  void stop();
  bool ready() const noexcept;
  std::uint8_t raw_evidence_failure() const noexcept;
  bool raw_evidence_capture_enabled() const noexcept;
  bool post_complete_begin_ready() const noexcept;
  std::optional<HoldDrainEvidence> hold_drain_snapshot() const noexcept;
  RawTerminalIdentity raw_terminal_flushed() const noexcept;
  RawFirstPageWritten raw_first_page_written() const noexcept;
  std::unique_ptr<RequestHandle> send(const std::vector<std::uint8_t> &) override;
  std::optional<DeviceObservation> observe();
  bool allow_inactive_prime(bool allowed) noexcept;
  bool set_post_complete_hold_tolerance(const std::vector<double> &values) noexcept;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
std::uint64_t management_now_us() noexcept;
}  // namespace dummy_motion_client

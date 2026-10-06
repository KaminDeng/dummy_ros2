#pragma once
#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <type_traits>
#include <vector>
#include "dp_dummy_robot_v2.hpp"
#include "dummy_motion_client/bench_permit.hpp"

namespace dummy_motion_client {
using ManageRequest = dp::protocol::dummy_robot_v2::DummyManageV2Request;
using Position = std::array<double, 6>;
// Dummy wire-major-v2 stable values.
enum class ManageOperation : std::uint16_t {
  kGetStatus=0, kSelectMode=1, kGrantSession=2, kCaptureReference=3,
  kClearReference=4, kSetPermit=5, kEnable=6, kBegin=7, kComplete=8,
  kCancel=9, kDisableAndRelease=10, kClearFault=11, kQuery=12,
  kRenewLease=20
};
enum class WireCoreState : std::uint16_t {
  kUnreferencedIdle=1, kIdle=2, kEnablingHold=3, kHold=4, kRunning=5, kFault=9
};
enum class WireReceipt : std::uint8_t { kQueued=1, kReplayed=2, kBusy=4 };
enum class WireResult : std::uint16_t { kOk=0, kPending=23 };
enum class WireTransactionPhase : std::uint8_t { kIdle=0, kReceived=1, kPrepared=2, kCommitted=3, kFailed=4 };
template <typename Enum> constexpr auto wire_value(Enum value) { return static_cast<std::underlying_type_t<Enum>>(value); }
struct ManageReply {
  std::uint64_t request_id=0, operation_id=0;
  bool queued=false, accepted=false, rejected=false, result_valid=false, succeeded=false;
  bool snapshot_valid=false, hold_available=false, hold_ready=false;
  bool release_committed=false;
  std::uint16_t result_code=0, first_failure_reason=0;
  std::uint8_t transaction_phase=0;
  std::uint32_t motion_epoch=0, accepted_seq=0;
  std::uint64_t session_id=0;
  std::uint32_t reference_epoch=0, management_epoch=0;
  Position position{}, hold_position{};
};
class ManagementClient {
 public:
  virtual ~ManagementClient() = default;
  virtual std::uint64_t submit(const ManageRequest &, std::uint64_t sent_at_us)=0;
  virtual std::uint64_t submit_permit(const ManageRequest &, const PermitBinding &,
                                      std::uint64_t sent_at_us) { return 0U; }
  virtual std::optional<ManageReply> poll(std::uint64_t request_id)=0;
};
// Bind this port to the selected device's manage_v2 endpoint. A handle owns the
// native request until its reply or explicit destruction. Poll never blocks.
class ManagementTransport {
 public:
  class RequestHandle {
   public:
    virtual ~RequestHandle() = default;
    virtual std::optional<std::vector<std::uint8_t>> poll()=0;
  };
  virtual ~ManagementTransport() = default;
  virtual std::unique_ptr<RequestHandle> send(const std::vector<std::uint8_t> &)=0;
};
class WireManagementClient final : public ManagementClient {
 public:
  explicit WireManagementClient(ManagementTransport &transport) : transport_(transport) {}
  WireManagementClient(ManagementTransport &transport, std::uint64_t session_id)
      : WireManagementClient(transport, session_id, 0U) {}
  WireManagementClient(ManagementTransport &transport, std::uint64_t session_id,
                       std::uint64_t process_nonce)
      : transport_(transport),
        next_id_(session_request_namespace(session_id ^ process_nonce)),
        next_seq_(static_cast<std::uint32_t>(session_id) + 1U) {
    if (next_id_ == 0U) next_id_ = 1U;
    if (next_seq_ == 0U) next_seq_ = 1U;
  }
  std::uint64_t submit(const ManageRequest &, std::uint64_t sent_at_us) override;
  std::uint64_t submit_permit(const ManageRequest &, const PermitBinding &,
                              std::uint64_t sent_at_us) override;
  std::optional<ManageReply> poll(std::uint64_t request_id) override;
  std::uint64_t original_sent_at_us() const { return sent_at_us_; }
 private:
  static constexpr std::uint64_t session_request_namespace(std::uint64_t session_id) noexcept {
    std::uint64_t mixed = session_id + 0x9e3779b97f4a7c15ULL;
    mixed = (mixed ^ (mixed >> 30U)) * 0xbf58476d1ce4e5b9ULL;
    mixed = (mixed ^ (mixed >> 27U)) * 0x94d049bb133111ebULL;
    return mixed ^ (mixed >> 31U);
  }
  std::uint64_t take_request_id() noexcept {
    if (next_id_ == 0U) next_id_ = 1U;
    return next_id_++;
  }
  std::uint32_t take_request_sequence() noexcept {
    if (next_seq_ == 0U) next_seq_ = 1U;
    return next_seq_++;
  }
  bool send(const ManageRequest &);
  ManagementTransport &transport_;
  std::unique_ptr<ManagementTransport::RequestHandle> handle_;
  ManageRequest original_{};
  ManageRequest current_{};
  bool pending_send_=false;
  unsigned query_delay_polls_=0;
  unsigned query_retries_=0;
  std::uint64_t next_id_=1, wire_id_=0, operation_id_=0, sent_at_us_=0;
  std::uint32_t next_seq_=1;
  bool accepted_=false;
};
}  // namespace dummy_motion_client

#pragma once
#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <type_traits>
#include <vector>
#include "dp_dummy_robot_v1.hpp"

namespace dummy_motion_client {
using ManageRequest = dp::protocol::dummy_robot_v1::DummyManageV1Request;
using Position = std::array<double, 6>;
// C05 contracts/dummy_robot_v1/enums/dummy_enum_values_v1.json wire values.
enum class ManageOperation : std::uint16_t { kEnable=5, kBegin=6, kCancel=7, kStop=8, kQuery=11, kComplete=13 };
enum class WireCoreState : std::uint16_t { kEnablingHold=3, kHold=4, kRunning=5, kFault=9 };
enum class WireReceipt : std::uint8_t { kQueued=1, kReplayed=2, kBusy=4 };
enum class WireResult : std::uint16_t { kOk=0, kPending=23 };
enum class WireOperationStatus : std::uint16_t { kAccepted=1, kRunning=2, kSucceeded=3, kFailed=4, kRejected=5, kExpired=6 };
template <typename Enum> constexpr auto wire_value(Enum value) { return static_cast<std::underlying_type_t<Enum>>(value); }
struct ManageReply {
  std::uint64_t request_id=0, operation_id=0;
  bool queued=false, accepted=false, rejected=false, result_valid=false, succeeded=false;
  bool snapshot_valid=false, hold_available=false, hold_ready=false;
  std::uint32_t motion_epoch=0, accepted_seq=0;
  Position position{}, hold_position{};
};
class ManagementClient {
 public:
  virtual ~ManagementClient() = default;
  virtual std::uint64_t submit(const ManageRequest &, std::uint64_t sent_at_us)=0;
  virtual std::optional<ManageReply> poll(std::uint64_t request_id)=0;
};
// Bind this port to the selected device's manage_v1 endpoint. A handle owns the
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
  std::uint64_t submit(const ManageRequest &, std::uint64_t sent_at_us) override;
  std::optional<ManageReply> poll(std::uint64_t request_id) override;
  std::uint64_t original_sent_at_us() const { return sent_at_us_; }
 private:
  bool send(const ManageRequest &);
  ManagementTransport &transport_;
  std::unique_ptr<ManagementTransport::RequestHandle> handle_;
  ManageRequest original_{};
  ManageRequest current_{};
  bool pending_send_=false;
  unsigned query_retries_=0;
  std::uint64_t next_id_=1, wire_id_=0, operation_id_=0, sent_at_us_=0;
  std::uint32_t next_seq_=1;
  bool accepted_=false;
};
}  // namespace dummy_motion_client

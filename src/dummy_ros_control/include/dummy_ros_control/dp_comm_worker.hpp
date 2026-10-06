#pragma once

#include <cstdint>

#include "dp_comm/dp_comm.hpp"
#include "dp_dummy_robot_v2_dp_comm.hpp"
#include "dummy_ros_control/guarded_zenbedded_system.hpp"

namespace dummy_ros_control {

class DpCommCommandWorker {
 public:
  using WireCommand = dp::protocol::dummy_robot_v2::DummyJointCommandV2;
  using Publisher = dp::comm::Publisher<WireCommand>;

  DpCommCommandWorker(GuardedZenbeddedSystem *system, Publisher *publisher,
                      std::uint32_t profile, std::uint8_t configured_mask)
      : system_(system), publisher_(publisher), profile_(profile), configured_mask_(configured_mask) {}

  dp::comm::Result publish_one() noexcept {
    if (!system_ || !publisher_) return dp::comm::Result::kInvalidArgument;
    CommandGenerationRecord record{};
    if (!system_->take_publish_record(&record)) return dp::comm::Result::kNotReady;
    WireCommand message;
    message.set_schema_major(2U);
    message.set_schema_minor(4U);
    message.set_boot_epoch(record.identity.boot);
    message.set_session_id(record.identity.session);
    message.set_reference_epoch(record.identity.reference_epoch);
    message.set_motion_epoch(record.identity.motion_epoch);
    message.set_profile_id(profile_);
    message.set_producer_seq(static_cast<std::uint32_t>(record.generation));
    message.set_freshness_ticket(record.identity.ticket);
    message.set_configured_mask(configured_mask_);
    const double positions[6]{record.position[0], record.position[1], record.position[2],
                              record.position[3], record.position[4], record.position[5]};
    message.set_position_rad(positions);
    return publisher_->publish(message);
  }

 private:
  GuardedZenbeddedSystem *system_;
  Publisher *publisher_;
  std::uint32_t profile_;
  std::uint8_t configured_mask_;
};

class DpCommStateReceiver {
 public:
  using WireState = dp::protocol::dummy_robot_v2::DummyJointStateV2;
  DpCommStateReceiver(GuardedZenbeddedSystem *system, StateExpectation expectation)
      : system_(system), expectation_(expectation) {}
  static void receive(dp::comm::BorrowedMessage<WireState> borrowed, void *context) noexcept {
    auto *self = static_cast<DpCommStateReceiver *>(context);
    if (!self || !self->system_) return;
    WireState state;
    if (borrowed.copy_to(&state) != dp::comm::Result::kOk) return;
    std::uint8_t bytes[dp::protocol::dummy_robot_v2::DummyJointStateV2Codec::kEncodedSize]{};
    if (dp::protocol::dummy_robot_v2::DummyJointStateV2Codec::encode(
            state, bytes, sizeof(bytes)) != dp::protocol::dummy_robot_v2::CodecResult::kOk) return;
    self->accepted_ = self->system_->ingest_wire_state(bytes, sizeof(bytes), self->expectation_);
  }
  bool accepted() const noexcept { return accepted_; }
 private:
  GuardedZenbeddedSystem *system_;
  StateExpectation expectation_;
  bool accepted_ = false;
};
}  // namespace dummy_ros_control

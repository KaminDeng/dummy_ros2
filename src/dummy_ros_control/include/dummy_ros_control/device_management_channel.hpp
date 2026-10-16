#pragma once
#include <array>
#include <algorithm>
#include <atomic>
#include <memory>
#include <mutex>
#include "dp_dummy_robot_v2.hpp"
#include "dummy_ros_control/device_transport.hpp"
#include "dummy_ros_control/raw_evidence_collector.hpp"

namespace dummy_ros_control {
// One pending request and one latest observation: capacity is independent of traffic rate.
struct DeviceManagementRequest {
  std::array<std::uint8_t, dp::protocol::dummy_robot_v2::DummyManageV2RequestCodec::kEncodedSize> request{};
  std::array<std::uint8_t, dp::protocol::dummy_robot_v2::DummyManageV2ResponseCodec::kEncodedSize> response{};
  std::atomic<bool> canceled{false}, failed{false}, complete{false};
};
struct DeviceManagementObservation {
  std::array<std::uint8_t, dp::protocol::dummy_robot_v2::DummyJointStateV2Codec::kEncodedSize> state{};
  std::array<std::uint8_t, dp::protocol::dummy_robot_v2::DummyDeviceStatusV2Codec::kEncodedSize> status{};
  std::uint64_t state_received_us=0, status_received_us=0;
  std::uint64_t rejected_ack_boot_epoch=0, rejected_ack_session_id=0, rejected_ack_received_us=0;
  std::uint32_t rejected_ack_reference_epoch=0, rejected_ack_motion_epoch=0,
                rejected_ack_sequence=0;
};
struct RawEvidenceTerminalFlushed {
  std::uint64_t boot_epoch = 0U;
  std::uint64_t session_id = 0U;
  std::uint64_t run_id = 0U;
  std::uint64_t parent_run_id = 0U;
  std::uint8_t attempt_kind = 0U, final_state = 0U;
  std::uint16_t first_failure_source = 0U;
  std::uint8_t first_failure_axis_mask = 0U;
  bool evidence_lost = false, motion_complete = false;
  bool release_committed = false, chain_valid = false;
  std::uint32_t safety_hold_motion_epoch = 0U;
  bool safety_hold_valid = false;
  std::uint32_t attempt_motion_epoch=0U, fault_stop_motion_epoch=0U;
  std::uint64_t terminal_received_us=0U;
  std::array<std::uint8_t,32> terminal_chain{};
  bool full_fault_groups_valid=false;
  std::array<std::array<std::uint8_t,32>,4> identity_hashes{};
  std::uint32_t terminal_group_count=0U;
};
struct HoldDrainSnapshot {
  std::uint64_t boot_epoch=0U, session_id=0U, processed_generation=0U;
  std::uint32_t reference_epoch=0U, motion_epoch=0U, committed_seq=0U;
  std::uint8_t disposition=0U;
  bool quiescent=false;
  std::uint64_t observed_us=0U, max_state_age_us=0U;
  std::array<double,6> position{}, tolerance{};
};
struct DeviceManagementChannel {
  // 每个DeviceTransport只创建一个channel；保留共享只读句柄可在worker退出后取原记录。
  DiagnosticJournal diagnostics;
  explicit DeviceManagementChannel(TransportParameters value)
      : parameters(std::move(value)),
        raw_evidence_capture_enabled(!parameters.raw_evidence_path.empty()) {}
  void observe_request_sequence(std::uint32_t sequence) noexcept {
    if (sequence == 0U) return;
    std::lock_guard<std::mutex> lock(request_sequence_mutex_);
    if (last_request_sequence_ == 0U ||
        (sequence != last_request_sequence_ &&
         static_cast<std::uint32_t>(sequence - last_request_sequence_) < 0x80000000U))
      last_request_sequence_ = sequence;
  }
  std::uint32_t reserve_request_sequence(std::uint32_t initial = 1U) noexcept {
    std::lock_guard<std::mutex> lock(request_sequence_mutex_);
    if (last_request_sequence_ == 0U) last_request_sequence_ = initial == 0U ? 1U : initial;
    else if (++last_request_sequence_ == 0U) last_request_sequence_ = 1U;
    return last_request_sequence_;
  }
  RawEvidenceTerminalFlushed raw_evidence_terminal_flushed() const noexcept {
    std::lock_guard<std::mutex> lock(raw_evidence_terminal_mutex_);
    return raw_evidence_terminal_;
  }
  void publish_raw_evidence_terminal_flushed(
      const RawEvidenceTerminalFlushed &receipt) noexcept {
    std::lock_guard<std::mutex> lock(raw_evidence_terminal_mutex_);
    raw_evidence_terminal_ = receipt;
  }
  void publish_raw_evidence_terminal_flushed(std::uint64_t boot_epoch,
                                             std::uint64_t session_id,
                                             std::uint64_t run_id) noexcept {
    publish_raw_evidence_terminal_flushed({boot_epoch, session_id, run_id});
  }
  RawEvidenceFirstPageWritten raw_evidence_first_page_written() const noexcept {
    std::lock_guard<std::mutex> lock(raw_evidence_first_page_mutex_);
    return raw_evidence_first_page_;
  }
  void publish_raw_evidence_first_page_written(
      const RawEvidenceFirstPageWritten &receipt) noexcept {
    std::lock_guard<std::mutex> lock(raw_evidence_first_page_mutex_);
    if (receipt.run_id > raw_evidence_first_page_.run_id)
      raw_evidence_first_page_ = receipt;
  }
  DeviceManagementObservation raw_cleanup_observation() const noexcept {
    std::lock_guard<std::mutex> lock(raw_cleanup_mutex_);return ready.load(std::memory_order_acquire) && raw_cleanup_generation_.load(std::memory_order_acquire)!=0U?raw_cleanup_:DeviceManagementObservation{};
  }
  std::uint64_t raw_cleanup_generation() const noexcept {return raw_cleanup_generation_.load(std::memory_order_acquire);}
  void reset_raw_cleanup_lifetime() noexcept {
    std::lock_guard<std::mutex> lock(raw_cleanup_mutex_);const auto generation=raw_cleanup_generation_.load(std::memory_order_relaxed);
    raw_cleanup_generation_=generation==0U || generation==UINT64_MAX?0U:generation+1U;raw_cleanup_={};
  }
  void publish_raw_cleanup_observation(bool state,const std::uint8_t* raw,std::size_t size,std::uint64_t received,bool producer_valid,std::uint64_t generation) noexcept {
    std::lock_guard<std::mutex> lock(raw_cleanup_mutex_);
    if(!ready.load(std::memory_order_acquire) || generation==0U || generation!=raw_cleanup_generation_.load(std::memory_order_acquire))return;
    if(state && size==raw_cleanup_.state.size()) {std::copy_n(raw,size,raw_cleanup_.state.begin());raw_cleanup_.state_received_us=producer_valid?received:0U;}
    if(!state && size==raw_cleanup_.status.size()) {std::copy_n(raw,size,raw_cleanup_.status.begin());raw_cleanup_.status_received_us=received;}
  }
  const TransportParameters parameters;
  std::mutex mutex;
  std::atomic<bool> ready{false};
  std::atomic<bool> inactive_prime_allowed{false};
  std::atomic<bool> post_complete_begin_ready{true};
  std::array<std::atomic<double>,6> post_complete_hold_tolerance_rad{};
  std::atomic<bool> raw_evidence_capture_enabled{false};
  // Zero means capture is disabled or has not failed; nonzero is sticky.
  std::atomic<std::uint8_t> raw_evidence_failure{0U};
  std::shared_ptr<DeviceManagementRequest> queued;
  DeviceManagementObservation observation;
  HoldDrainSnapshot hold_drain;
 private:
  std::atomic<std::uint64_t> raw_cleanup_generation_{1U};
  mutable std::mutex raw_cleanup_mutex_;
  DeviceManagementObservation raw_cleanup_{};
  std::mutex request_sequence_mutex_;
  std::uint32_t last_request_sequence_ = 0U;
  mutable std::mutex raw_evidence_first_page_mutex_;
  RawEvidenceFirstPageWritten raw_evidence_first_page_{};
  mutable std::mutex raw_evidence_terminal_mutex_;
  RawEvidenceTerminalFlushed raw_evidence_terminal_{};
};
}  // namespace dummy_ros_control

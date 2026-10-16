#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string>
#include <thread>

namespace dummy_ros_control {
struct DeviceManagementChannel;

struct RawFaultProducerClock {
  std::uint32_t last_sequence=0;
  std::uint64_t last_produced=0,last_received=0;
};
// 只用于已建立正常流源时钟锚后的故障清理原页，不产生正常控制/admission资格。
inline bool raw_fault_producer_sample_valid(std::uint64_t received,std::uint64_t produced,std::uint32_t seq,
    bool has_normal,std::uint32_t normal_seq,std::uint64_t normal_produced,std::int64_t normal_offset,
    std::int64_t anchor_offset,std::uint64_t anchor_received,RawFaultProducerClock &fault,std::uint64_t budget) noexcept {
  if(!has_normal || !produced || !seq || !received || !anchor_received || received<anchor_received ||
     received>static_cast<std::uint64_t>(INT64_MAX) || produced>static_cast<std::uint64_t>(INT64_MAX) ||
     produced<=normal_produced || received<fault.last_received || produced<=fault.last_produced)return false;
  const auto last=fault.last_produced?fault.last_sequence:normal_seq;
  const auto distance=static_cast<std::uint32_t>(seq-last);
  if(distance==0U || distance>=0x80000000U)return false;
  const auto offset=static_cast<std::int64_t>(received)-static_cast<std::int64_t>(produced);
  if(offset<anchor_offset && static_cast<std::uint64_t>(anchor_offset)-static_cast<std::uint64_t>(offset)>budget+(received-anchor_received)/1000U)return false;
  if(offset<normal_offset) {
    if(static_cast<std::uint64_t>(normal_offset)-static_cast<std::uint64_t>(offset)>budget)return false;
  } else if(static_cast<std::uint64_t>(offset)-static_cast<std::uint64_t>(normal_offset)>budget)return false;
  fault={seq,produced,received};return true;
}

struct RawEvidenceIdentity {
  std::array<std::uint8_t, 16U> client_id{};
  std::array<std::uint8_t, 32U> contract_hash{};
  std::array<std::uint8_t, 32U> model_hash{};
  std::array<std::uint8_t, 32U> profile_hash{};
  std::array<std::uint8_t, 32U> firmware_hash{};
};

struct RawEvidenceBinding {
  std::uint64_t run_id = 0U;
  std::uint64_t boot_epoch = 0U;
  std::uint64_t session_id = 0U;
  std::uint64_t parent_run_id = 0U;
  std::uint8_t attempt_kind = 0U;
};

class RawEvidenceBindings final {
 public:
  bool observe(const RawEvidenceBinding &binding,
               std::uint64_t terminal_written_run_id) noexcept;
  const RawEvidenceBinding *find(std::uint64_t run_id) const noexcept;

 private:
  std::array<RawEvidenceBinding, 8U> slots_{};
  std::size_t count_ = 0U;
  std::uint64_t last_run_id_ = 0U;
};

struct RawEvidencePage {
  static constexpr std::size_t kRawSize = 372U;
  std::array<std::uint8_t, kRawSize> raw{};
  std::array<std::uint8_t, 32U> previous_chain_sha256{};
  std::array<std::uint8_t, 32U> chain_sha256{};
  std::uint64_t boot_epoch = 0U;
  std::uint64_t session_id = 0U;
  std::uint64_t run_id = 0U;
  std::uint64_t parent_run_id = 0U;
  std::uint32_t evidence_seq = 0U;
  std::uint32_t group_count = 0U;
  std::uint16_t terminal_reason = 0U;
  std::uint8_t attempt_kind = 0U;
  std::uint8_t category = 0U;
  std::uint8_t final_state = 0U;
  bool final = false;
  bool evidence_lost = false;
};

struct RawEvidenceFirstPageWritten {
  std::uint64_t boot_epoch = 0U;
  std::uint64_t session_id = 0U;
  std::uint64_t run_id = 0U;
  std::uint32_t motion_epoch = 0U;
  std::uint32_t evidence_seq = 0U;
  std::uint32_t group_count = 0U;
  std::uint8_t attempt_kind = 0U;
  std::uint8_t category = 0U;
  std::uint8_t expected_mask = 0U;
  std::uint8_t submitted_mask = 0U;
  std::uint8_t sent_mask = 0U;
  std::uint8_t failed_mask = 0U;
  bool final = false;
  bool evidence_lost = false;
  bool chain_valid = false;
  bool full_six = false;
  bool strict_refresh_successor = false;
};

// One dp_comm callback thread produces records; one private writer thread drains
// them. Capture is diagnostic only and never grants motion or qualification.
class RawEvidenceCollector final {
 public:
  using BeforeTerminalFlush = bool (*)(void *) noexcept;
  using FlushFile = int (*)(std::FILE *, void *) noexcept;
  static constexpr std::size_t kCapacity = 256U;
  static constexpr std::size_t kMaxRawSize = 651U;
  enum class Kind : std::uint8_t {
    kPage = 1U, kState = 2U, kStatus = 3U,
    kManagementRequest = 4U, kManagementResponse = 5U
  };
  enum class Failure : std::uint8_t {
    kNone, kSequence, kChain, kIdentity, kOverflow, kIo, kInvalid, kIncomplete
  };

  explicit RawEvidenceCollector(RawEvidenceIdentity identity,
                                DeviceManagementChannel *channel = nullptr,
                                BeforeTerminalFlush before_terminal_flush = nullptr,
                                void *flush_context = nullptr,
                                FlushFile flush_file = nullptr);
  ~RawEvidenceCollector();
  RawEvidenceCollector(const RawEvidenceCollector &) = delete;
  RawEvidenceCollector &operator=(const RawEvidenceCollector &) = delete;
  bool start(const std::string &path);
  void stop() noexcept;
  bool offer_page(const RawEvidencePage &page, std::uint64_t received_at_us) noexcept;
  bool expect_run(std::uint64_t run_id) noexcept;
  bool offer_observation(Kind kind, const std::uint8_t *raw, std::size_t size,
                         std::uint64_t received_at_us, std::uint64_t boot_epoch,
                         std::uint64_t session_id, bool cleanup_producer_valid=false) noexcept;
  bool offer_management_request(const std::uint8_t *raw, std::size_t size,
                                std::uint64_t sent_at_us) noexcept;
  bool offer_management_response(const std::uint8_t *raw, std::size_t size,
                                 std::uint64_t received_at_us) noexcept;
  void reject_invalid() noexcept { fail(Failure::kInvalid); }
  Failure failure() const noexcept { return failure_.load(std::memory_order_acquire); }
  std::uint64_t written() const noexcept { return written_.load(std::memory_order_acquire); }
  std::uint64_t pages_written() const noexcept { return pages_written_.load(std::memory_order_acquire); }
  std::uint64_t terminal_written_run_id() const noexcept {
    return terminal_written_run_id_.load(std::memory_order_acquire);
  }

 private:
  struct Record {
    std::array<std::uint8_t, kMaxRawSize> raw{};
    std::uint64_t received_at_us = 0U;
    std::uint64_t boot_epoch = 0U;
    std::uint64_t session_id = 0U;
    std::uint64_t run_id = 0U;
    std::uint32_t evidence_seq = 0U;
    std::uint16_t size = 0U;
    Kind kind = Kind::kStatus;
    bool terminal = false;
    bool cleanup_producer_valid=false;
    std::uint64_t cleanup_generation=0U;
    std::uint64_t parent_run_id = 0U;
    std::uint8_t attempt_kind = 0U, final_state = 0U;
    std::uint16_t first_failure_source = 0U;
    std::uint8_t first_failure_axis_mask = 0U;
    bool evidence_lost = false, motion_complete = false, release_committed = false;
    std::uint32_t safety_hold_motion_epoch = 0U;
    bool safety_hold_valid = false;
  std::uint32_t attempt_motion_epoch=0U, fault_stop_motion_epoch=0U;
  std::uint64_t terminal_received_us=0U;
  std::array<std::uint8_t,32> terminal_chain{};
  bool full_fault_groups_valid=false;
  std::array<std::array<std::uint8_t,32>,4> identity_hashes{};
  std::uint32_t terminal_group_count=0U;

    RawEvidenceFirstPageWritten first_page{};
  };
  bool enqueue(const Record &record) noexcept;
  void fail(Failure value) noexcept;
  void write_loop() noexcept;
  bool write_record(const Record &record) noexcept;
  bool verify_chain(const RawEvidencePage &page) noexcept;

  std::array<Record, kCapacity> queue_{};
  std::atomic<std::uint64_t> produced_{0U}, consumed_{0U}, written_{0U}, pages_written_{0U};
  std::atomic<std::uint64_t> terminal_written_run_id_{0U};
  std::atomic<Failure> failure_{Failure::kNone};
  std::atomic<bool> running_{false};
  std::thread writer_;
  std::FILE *file_ = nullptr;
  const RawEvidenceIdentity identity_;
  DeviceManagementChannel *const channel_;
  const BeforeTerminalFlush before_terminal_flush_;
  void *const flush_context_;
  const FlushFile flush_file_;
  struct ExpectedRun {
    std::uint64_t run_id = 0U;
    std::uint64_t terminal_record_end = 0U;
    bool terminal = false;
  };
  std::array<ExpectedRun, 8U> expected_runs_{};
  std::size_t expected_count_ = 0U;
  std::uint64_t last_expected_run_id_ = 0U;
  std::uint64_t run_id_ = 0U, boot_epoch_ = 0U, session_id_ = 0U;
  std::uint64_t parent_run_id_ = 0U;
  std::uint32_t next_seq_ = 0U;
  std::uint32_t can_page_count_ = 0U;
  std::uint32_t last_reported_group_count_ = 0U;
  std::uint32_t first_motion_epoch_ = 0U, safety_hold_motion_epoch_ = 0U;
  bool safety_hold_valid_ = false;
  bool safety_hold_regressed_ = false;
  std::uint8_t attempt_kind_ = 0U;
  std::array<std::uint8_t, 32U> last_chain_{};
  bool terminal_seen_ = false;
  bool fault_groups_valid_=true;
  std::uint8_t fault_group_phase_=0U;
  std::uint32_t fault_motion_epoch_=0U, fault_stop_epoch_=0U, fault_disable_count_=0U;
  std::uint64_t fault_last_group_id_=0U;
  std::uint16_t fault_group_source_=0U;

};

}  // namespace dummy_ros_control

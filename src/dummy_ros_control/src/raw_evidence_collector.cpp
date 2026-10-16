#include "dummy_ros_control/raw_evidence_collector.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <openssl/sha.h>
#include <unistd.h>

#include "dp_dummy_robot_v2.hpp"
#include "dummy_ros_control/device_management_channel.hpp"

namespace dummy_ros_control {
namespace {
void put_le(std::uint8_t *out, std::uint64_t value, std::size_t width) noexcept {
  for (std::size_t i = 0U; i < width; ++i) out[i] = static_cast<std::uint8_t>(value >> (8U * i));
}

int open_new_capture(const std::string &path) noexcept {
  if (path.size() < 2U || path.front() != '/' || path.find('\0') != std::string::npos) return -1;
  int parent = ::open("/", O_RDONLY | O_DIRECTORY | O_CLOEXEC);
  if (parent < 0) return -1;
  std::size_t begin = 1U;
  while (begin < path.size()) {
    const auto slash = path.find('/', begin);
    const auto component = path.substr(begin, slash == std::string::npos ?
        std::string::npos : slash - begin);
    if (component.empty() || component == "." || component == "..") break;
    const int next = slash == std::string::npos
        ? ::openat(parent, component.c_str(),
                   O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW | O_CLOEXEC, 0600)
        : ::openat(parent, component.c_str(), O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC);
    ::close(parent);
    if (next < 0) return -1;
    if (slash == std::string::npos) return next;
    parent = next;
    begin = slash + 1U;
  }
  ::close(parent);
  return -1;
}
}  // namespace

const RawEvidenceBinding *RawEvidenceBindings::find(std::uint64_t run_id) const noexcept {
  for (std::size_t i = 0U; i < count_; ++i)
    if (slots_[i].run_id == run_id) return &slots_[i];
  return nullptr;
}

bool RawEvidenceBindings::observe(const RawEvidenceBinding &binding,
                                  std::uint64_t terminal_written_run_id) noexcept {
  if (binding.run_id == 0U || binding.boot_epoch == 0U ||
      binding.attempt_kind < 1U || binding.attempt_kind > 3U ||
      (binding.attempt_kind == 1U ? binding.parent_run_id != 0U :
       binding.parent_run_id == 0U || binding.parent_run_id >= binding.run_id)) return false;
  if (const auto *existing = find(binding.run_id))
    return existing->boot_epoch == binding.boot_epoch &&
        (binding.session_id == 0U || existing->session_id == binding.session_id) &&
        existing->parent_run_id == binding.parent_run_id &&
        existing->attempt_kind == binding.attempt_kind;
  if (binding.session_id == 0U || binding.run_id <= last_run_id_) return false;
  std::size_t slot = count_;
  if (slot == slots_.size()) {
    for (std::size_t i = 0U; i < count_; ++i)
      if (slots_[i].run_id <= terminal_written_run_id &&
          (slot == slots_.size() || slots_[i].run_id < slots_[slot].run_id)) slot = i;
    if (slot == slots_.size()) return false;
  } else ++count_;
  slots_[slot] = binding;
  last_run_id_ = binding.run_id;
  return true;
}

RawEvidenceCollector::RawEvidenceCollector(RawEvidenceIdentity identity,
    DeviceManagementChannel *channel, BeforeTerminalFlush before_terminal_flush,
    void *flush_context, FlushFile flush_file)
    : identity_(std::move(identity)), channel_(channel),
      before_terminal_flush_(before_terminal_flush), flush_context_(flush_context),
      flush_file_(flush_file) {}
RawEvidenceCollector::~RawEvidenceCollector() { stop(); }

bool RawEvidenceCollector::start(const std::string &path) {
  if (path.empty() || running_ || file_ || writer_.joinable()) return false;
  const int fd = open_new_capture(path);
  if (fd < 0) { fail(Failure::kIo); return false; }
  file_ = ::fdopen(fd, "wb");
  if (!file_) { ::close(fd); fail(Failure::kIo); return false; }
  if (std::setvbuf(file_, nullptr, _IONBF, 0) != 0) {
    fail(Failure::kIo);
    stop();
    return false;
  }
  const std::uint8_t header[8]{'D', 'P', 'R', 'A', 'W', '0', '1', 0};
  if (std::fwrite(header, 1U, sizeof(header), file_) != sizeof(header)) {
    fail(Failure::kIo);
    stop();
    return false;
  }
  running_.store(true, std::memory_order_release);
  try { writer_ = std::thread([this] { write_loop(); }); }
  catch (...) { running_ = false; fail(Failure::kIo); stop(); return false; }
  return true;
}

void RawEvidenceCollector::stop() noexcept {
  for (std::size_t i = 0U; i < expected_count_; ++i)
    if (!expected_runs_[i].terminal) fail(Failure::kIncomplete);
  if (run_id_ != 0U && !terminal_seen_) fail(Failure::kIncomplete);
  running_.store(false, std::memory_order_release);
  if (writer_.joinable()) writer_.join();
  if (file_) {
    if (std::fflush(file_) != 0) fail(Failure::kIo);
    if (std::fclose(file_) != 0) fail(Failure::kIo);
    file_ = nullptr;
  }
}

bool RawEvidenceCollector::expect_run(std::uint64_t run_id) noexcept {
  if (failure() != Failure::kNone) return false;
  if (run_id == 0U) { fail(Failure::kInvalid); return false; }
  for (std::size_t i = 0U; i < expected_count_; ++i)
    if (expected_runs_[i].run_id == run_id) return true;
  if (run_id <= last_expected_run_id_) {
    fail(Failure::kIdentity);
    return false;
  }
  std::size_t slot = expected_count_;
  if (slot == expected_runs_.size()) {
    const auto consumed = consumed_.load(std::memory_order_acquire);
    for (std::size_t i = 0U; i < expected_count_; ++i) {
      const auto &candidate = expected_runs_[i];
      if (candidate.terminal && candidate.terminal_record_end <= consumed &&
          (slot == expected_runs_.size() || candidate.run_id < expected_runs_[slot].run_id))
        slot = i;
    }
    if (slot == expected_runs_.size()) { fail(Failure::kOverflow); return false; }
  } else ++expected_count_;
  expected_runs_[slot] = {run_id, 0U, false};
  last_expected_run_id_ = run_id;
  return true;
}

bool RawEvidenceCollector::verify_chain(const RawEvidencePage &page) noexcept {
  std::array<std::uint8_t, 32U> expected_previous{};
  if (page.evidence_seq == 0U) {
    std::array<std::uint8_t, 192U> seed{};
    put_le(seed.data(), page.run_id, 8U);
    put_le(seed.data() + 8U, page.boot_epoch, 8U);
    std::copy(identity_.client_id.begin(), identity_.client_id.end(), seed.begin() + 16U);
    std::copy(identity_.contract_hash.begin(), identity_.contract_hash.end(), seed.begin() + 32U);
    std::copy(identity_.model_hash.begin(), identity_.model_hash.end(), seed.begin() + 64U);
    std::copy(identity_.profile_hash.begin(), identity_.profile_hash.end(), seed.begin() + 96U);
    std::copy(identity_.firmware_hash.begin(), identity_.firmware_hash.end(), seed.begin() + 128U);
    if (!::SHA256(seed.data(), seed.size(), expected_previous.data())) return false;
  } else expected_previous = last_chain_;
  if (page.previous_chain_sha256 != expected_previous) return false;
  constexpr std::size_t kPrefix =
      dp::protocol::dummy_robot_v2::DummyAttemptEvidenceV2Codec::kCanonicalPrefixSize;
  constexpr std::size_t kLostOffset =
      dp::protocol::dummy_robot_v2::DummyAttemptEvidenceV2Codec::kEvidenceLostOffset;
  static_assert(RawEvidencePage::kRawSize == kLostOffset + 1U);
  std::array<std::uint8_t, 32U + kPrefix + 1U> input{};
  std::copy(expected_previous.begin(), expected_previous.end(), input.begin());
  std::copy_n(page.raw.begin(), kPrefix, input.begin() + 32U);
  input.back() = page.raw[kLostOffset];
  std::array<std::uint8_t, 32U> computed{};
  return ::SHA256(input.data(), input.size(), computed.data()) && computed == page.chain_sha256;
}

void RawEvidenceCollector::fail(Failure value) noexcept {
  Failure expected = Failure::kNone;
  if (failure_.compare_exchange_strong(expected, value, std::memory_order_acq_rel) && channel_)
    channel_->raw_evidence_failure.store(static_cast<std::uint8_t>(value),
                                         std::memory_order_release);
}

bool RawEvidenceCollector::enqueue(const Record &record) noexcept {
  if (failure() != Failure::kNone) return false;
  const auto produced = produced_.load(std::memory_order_relaxed);
  if (produced - consumed_.load(std::memory_order_acquire) >= kCapacity) {
    fail(Failure::kOverflow);
    return false;
  }
  queue_[produced % kCapacity] = record;
  produced_.store(produced + 1U, std::memory_order_release);
  return true;
}

bool RawEvidenceCollector::offer_observation(Kind kind, const std::uint8_t *raw,
    std::size_t size, std::uint64_t received_at_us, std::uint64_t boot_epoch,
    std::uint64_t session_id,bool cleanup_producer_valid) noexcept {
  if ((kind != Kind::kState && kind != Kind::kStatus) || !raw || size == 0U ||
      size > kMaxRawSize) { fail(Failure::kInvalid); return false; }
  Record record{};
  std::copy_n(raw, size, record.raw.begin());
  record.size = static_cast<std::uint16_t>(size);
  record.kind = kind;record.cleanup_producer_valid=cleanup_producer_valid;
  record.cleanup_generation=channel_?channel_->raw_cleanup_generation():0U;
  record.received_at_us = received_at_us;
  record.boot_epoch = boot_epoch;
  record.session_id = session_id;
  return enqueue(record);
}

bool RawEvidenceCollector::offer_management_request(const std::uint8_t *raw,
    std::size_t size, std::uint64_t sent_at_us) noexcept {
  using namespace dp::protocol::dummy_robot_v2;
  DummyManageV2Request decoded{};
  if (!raw || size != DummyManageV2RequestCodec::kEncodedSize ||
      DummyManageV2RequestCodec::decode(raw, size, &decoded) != CodecResult::kOk) {
    fail(Failure::kInvalid);
    return false;
  }
  if (!running_.load(std::memory_order_acquire)) {
    fail(Failure::kIo);
    return false;
  }
  Record record{};
  std::copy_n(raw, size, record.raw.begin());
  record.size = static_cast<std::uint16_t>(size);
  record.kind = Kind::kManagementRequest;
  record.received_at_us = sent_at_us;
  record.boot_epoch = decoded.expected_boot_epoch();
  record.session_id = decoded.expected_session_id();
  record.evidence_seq = decoded.request_seq();
  return enqueue(record);
}

bool RawEvidenceCollector::offer_management_response(const std::uint8_t *raw,
    std::size_t size, std::uint64_t received_at_us) noexcept {
  using namespace dp::protocol::dummy_robot_v2;
  DummyManageV2Response decoded{};
  if (!raw || size != DummyManageV2ResponseCodec::kEncodedSize ||
      DummyManageV2ResponseCodec::decode(raw, size, &decoded) != CodecResult::kOk) {
    fail(Failure::kInvalid);
    return false;
  }
  if (!running_.load(std::memory_order_acquire)) {
    fail(Failure::kIo);
    return false;
  }
  Record record{};
  std::copy_n(raw, size, record.raw.begin());
  record.size = static_cast<std::uint16_t>(size);
  record.kind = Kind::kManagementResponse;
  record.received_at_us = received_at_us;
  record.boot_epoch = decoded.boot_epoch();
  record.session_id = decoded.session_id();
  record.run_id = decoded.run_id();
  return enqueue(record);
}

bool RawEvidenceCollector::offer_page(const RawEvidencePage &page,
                                      std::uint64_t received_at_us) noexcept {
  if (failure() != Failure::kNone) return false;
  if (page.boot_epoch == 0U || page.session_id == 0U || page.run_id == 0U ||
      (page.attempt_kind != 1U && page.attempt_kind != 2U && page.attempt_kind != 3U) ||
      (page.category < 1U || page.category > 4U) ||
      !page.final || (page.category == 4U && (page.final_state == 0U || page.final_state > 9U)) ||
      page.evidence_lost ||
      page.evidence_seq == UINT32_MAX) { fail(Failure::kInvalid); return false; }
  if (run_id_ != page.run_id) {
    if (run_id_ == 0U && (page.attempt_kind != 1U || page.parent_run_id != 0U)) {
      fail(Failure::kIdentity);
      return false;
    }
    if (run_id_ != 0U && (!terminal_seen_ || page.run_id <= run_id_ ||
                         page.boot_epoch != boot_epoch_ || page.session_id != session_id_ ||
                         (page.attempt_kind == 1U ? page.parent_run_id != 0U :
                          page.parent_run_id != run_id_))) {
      fail(Failure::kIdentity);
      return false;
    }
    if (page.evidence_seq != 0U) { fail(Failure::kSequence); return false; }
    run_id_ = page.run_id;
    boot_epoch_ = page.boot_epoch;
    session_id_ = page.session_id;
    parent_run_id_ = page.parent_run_id;
    attempt_kind_ = page.attempt_kind;
    next_seq_ = 0U;
    can_page_count_ = 0U;
    last_reported_group_count_ = 0U;
    safety_hold_motion_epoch_ = 0U;
    first_motion_epoch_ = 0U;
    safety_hold_valid_ = false;
    safety_hold_regressed_ = false;
    terminal_seen_ = false;
    fault_groups_valid_=true;fault_group_phase_=0U;fault_motion_epoch_=0U;
    fault_stop_epoch_=0U;fault_disable_count_=0U;fault_last_group_id_=0U;fault_group_source_=0U;
  }
  if (page.boot_epoch != boot_epoch_ || page.session_id != session_id_ ||
      page.parent_run_id != parent_run_id_ || page.attempt_kind != attempt_kind_ || terminal_seen_) {
    fail(Failure::kIdentity);
    return false;
  }
  if (page.evidence_seq != next_seq_) {
    if(channel_) {
      const auto expected=channel_->diagnostics.context().second;
      const DiagnosticIdentity observed{page.boot_epoch,page.session_id,0U,0U,page.evidence_seq,0U,false};
      channel_->diagnostics.reject(DiagnosticReason::kRawSequence,observed,expected,received_at_us,"RAW_SEQUENCE_DIAG",0U,{page.run_id,page.evidence_seq,next_seq_,page.group_count,can_page_count_,page.category});
      channel_->diagnostics.detail(
        "RAW_SEQUENCE_DIAG run=%llu seq=%u expected_seq=%u group_count=%u accepted_pages=%u category=%u reason=seq\n",
        static_cast<unsigned long long>(page.run_id),page.evidence_seq,next_seq_,
        page.group_count,can_page_count_,page.category);
    } else std::fprintf(stderr,
        "RAW_SEQUENCE_DIAG run=%llu seq=%u expected_seq=%u group_count=%u accepted_pages=%u category=%u reason=seq\n",
        static_cast<unsigned long long>(page.run_id),page.evidence_seq,next_seq_,
        page.group_count,can_page_count_,page.category);
    fail(Failure::kSequence); return false;
  }
  if ((page.category == 4U && page.group_count != can_page_count_) ||
      (page.category != 4U && (page.group_count == 0U ||
                               page.group_count <= can_page_count_ ||
                               page.group_count < last_reported_group_count_))) {
    if(channel_) {
      const auto expected=channel_->diagnostics.context().second;
      const DiagnosticIdentity observed{page.boot_epoch,page.session_id,0U,0U,page.evidence_seq,0U,false};
      channel_->diagnostics.reject(DiagnosticReason::kRawGroupCount,observed,expected,received_at_us,"RAW_SEQUENCE_DIAG",0U,{page.run_id,page.evidence_seq,next_seq_,page.group_count,can_page_count_,last_reported_group_count_,page.category});
      channel_->diagnostics.detail(
        "RAW_SEQUENCE_DIAG run=%llu seq=%u expected_seq=%u group_count=%u accepted_pages=%u last_reported=%u category=%u reason=group_count\n",
        static_cast<unsigned long long>(page.run_id),page.evidence_seq,next_seq_,
        page.group_count,can_page_count_,last_reported_group_count_,page.category);
    } else std::fprintf(stderr,
        "RAW_SEQUENCE_DIAG run=%llu seq=%u expected_seq=%u group_count=%u accepted_pages=%u last_reported=%u category=%u reason=group_count\n",
        static_cast<unsigned long long>(page.run_id),page.evidence_seq,next_seq_,
        page.group_count,can_page_count_,last_reported_group_count_,page.category);
    fail(Failure::kSequence);
    return false;
  }
  if (page.category == 4U && page.group_count == 0U &&
      (page.final_state != 3U || page.terminal_reason != 14U ||
       page.attempt_kind != 1U || page.parent_run_id != 0U)) {
    fail(Failure::kInvalid);
    return false;
  }
  if (!verify_chain(page)) {
    fail(Failure::kChain);
    return false;
  }
  dp::protocol::dummy_robot_v2::DummyAttemptEvidenceV2 decoded{};
  if (dp::protocol::dummy_robot_v2::DummyAttemptEvidenceV2Codec::decode(
          page.raw.data(), page.raw.size(), &decoded) !=
          dp::protocol::dummy_robot_v2::CodecResult::kOk ||
      decoded.run_id() != page.run_id || decoded.attempt_id() != page.run_id ||
      decoded.parent_run_id() != page.parent_run_id ||
      decoded.evidence_seq() != page.evidence_seq ||
      decoded.group_count() != page.group_count ||
      decoded.final_state() != page.final_state ||
      decoded.terminal_reason() != page.terminal_reason ||
      decoded.attempt_kind() != page.attempt_kind || decoded.category() != page.category ||
      (decoded.final() != 0U) != page.final || decoded.evidence_lost() != page.evidence_lost ||
      !std::equal(page.previous_chain_sha256.begin(), page.previous_chain_sha256.end(),
                  std::begin(decoded.previous_chain_sha256())) ||
      !std::equal(page.chain_sha256.begin(), page.chain_sha256.end(),
                  std::begin(decoded.chain_sha256()))) {
    fail(Failure::kInvalid);
    return false;
  }
  if (page.evidence_seq == 0U) first_motion_epoch_ = decoded.motion_epoch();
  if (page.category != 4U && safety_hold_valid_ &&
      decoded.motion_epoch() < safety_hold_motion_epoch_) {
    safety_hold_valid_ = false;
    safety_hold_regressed_ = true;
  }
  if (!safety_hold_regressed_ && page.category == 2U &&
      decoded.motion_epoch() > first_motion_epoch_ &&
      decoded.expected_mask() == 63U && decoded.submitted_mask() == 63U &&
      decoded.sent_mask() == 63U && decoded.failed_mask() == 0U &&
      decoded.first_failure_source() == 0U &&
      decoded.first_failure_axis_mask() == 0U) {
    safety_hold_motion_epoch_ = decoded.motion_epoch();
    safety_hold_valid_ = true;
  }
  // 完整fault证明另lane：不改变旧collector接纳/取消Hold语义，只导出已验证原页的证明。
  if(page.category!=4U) {
    bool valid=decoded.group_id()>fault_last_group_id_ && decoded.expected_mask()==63U &&
      decoded.submitted_mask()==63U && decoded.sent_mask()==63U && decoded.failed_mask()==0U &&
      !decoded.motion_complete() && !decoded.release_committed() && decoded.final_state()==0U;
    for(std::size_t axis=0;axis<6;++axis) {
      const auto q=decoded.queued_at_us()[axis],take=decoded.taken_at_us()[axis],sub=decoded.submitted_at_us()[axis],sent=decoded.sent_at_us()[axis];
      valid=valid && q!=0U && q<=take && take<=sub && sub<=sent && sent-q<=100000U && decoded.abort_at_us()[axis]==0U;
    }
    if(decoded.category()==1U) {
      valid=valid && fault_group_phase_==0U && decoded.first_failure_source()==0U && decoded.first_failure_axis_mask()==0U &&
        decoded.motion_epoch()>=first_motion_epoch_ && decoded.motion_epoch()-first_motion_epoch_<=1U &&
        decoded.motion_epoch()>=fault_motion_epoch_;
      fault_motion_epoch_=decoded.motion_epoch();
    } else if(decoded.category()==2U) {
      valid=valid && fault_group_phase_<=1U && fault_motion_epoch_!=0U && fault_motion_epoch_!=UINT32_MAX &&
        decoded.motion_epoch()==fault_motion_epoch_+1U && decoded.first_failure_source()!=0U && decoded.first_failure_axis_mask()==63U;
      fault_group_phase_=1U;fault_stop_epoch_=decoded.motion_epoch();
      if(fault_group_source_!=0U && fault_group_source_!=decoded.first_failure_source())valid=false;
      fault_group_source_=decoded.first_failure_source();
    } else if(decoded.category()==3U) {
      valid=valid && fault_group_phase_==1U && decoded.motion_epoch()==fault_stop_epoch_ &&
        decoded.first_failure_source()==fault_group_source_ && decoded.first_failure_axis_mask()==63U;
      fault_group_phase_=2U;++fault_disable_count_;
    } else valid=false;
    fault_groups_valid_=fault_groups_valid_ && valid;fault_last_group_id_=decoded.group_id();
  }
  Record record{};
  std::copy(page.raw.begin(), page.raw.end(), record.raw.begin());
  record.size = static_cast<std::uint16_t>(page.raw.size());
  record.kind = Kind::kPage;
  record.received_at_us = received_at_us;
  record.boot_epoch = page.boot_epoch;
  record.session_id = page.session_id;
  record.run_id = page.run_id;
  record.evidence_seq = page.evidence_seq;
  record.terminal = page.category == 4U;
  if (record.terminal) {
    record.parent_run_id = page.parent_run_id;
    record.attempt_kind = page.attempt_kind;
    record.final_state = page.final_state;
    record.first_failure_source = decoded.first_failure_source();
    record.first_failure_axis_mask = decoded.first_failure_axis_mask();
    record.evidence_lost = page.evidence_lost;
    record.motion_complete = decoded.motion_complete();
    record.release_committed = decoded.release_committed();
    record.identity_hashes={identity_.contract_hash,identity_.model_hash,identity_.profile_hash,identity_.firmware_hash};record.terminal_group_count=decoded.group_count();
    record.attempt_motion_epoch=first_motion_epoch_;record.fault_stop_motion_epoch=fault_stop_epoch_;
    record.terminal_received_us=received_at_us;record.terminal_chain=page.chain_sha256;
    const bool zero_terminal_times=std::all_of(std::begin(decoded.queued_at_us()),std::end(decoded.queued_at_us()),[](auto x){return x==0U;}) &&
      std::all_of(std::begin(decoded.taken_at_us()),std::end(decoded.taken_at_us()),[](auto x){return x==0U;}) &&
      std::all_of(std::begin(decoded.submitted_at_us()),std::end(decoded.submitted_at_us()),[](auto x){return x==0U;}) &&
      std::all_of(std::begin(decoded.sent_at_us()),std::end(decoded.sent_at_us()),[](auto x){return x==0U;}) &&
      std::all_of(std::begin(decoded.abort_at_us()),std::end(decoded.abort_at_us()),[](auto x){return x==0U;});
    record.full_fault_groups_valid=fault_groups_valid_ && fault_group_phase_==2U && fault_disable_count_==1U &&
      page.attempt_kind==1U && page.parent_run_id==0U && decoded.motion_epoch()==first_motion_epoch_ &&
      decoded.group_id()==0U && decoded.group_count()==can_page_count_ && decoded.expected_mask()==0U &&
      decoded.submitted_mask()==0U && decoded.sent_mask()==0U && decoded.failed_mask()==0U && zero_terminal_times &&
      decoded.final_state()==4U && decoded.release_committed() && !decoded.motion_complete() && !decoded.evidence_lost() &&
      decoded.first_failure_source()!=0U && decoded.first_failure_source()==fault_group_source_ && decoded.first_failure_axis_mask()==63U;
    record.safety_hold_motion_epoch = safety_hold_motion_epoch_;
    record.safety_hold_valid = safety_hold_valid_ &&
        decoded.motion_epoch() == first_motion_epoch_;
  }
  if (page.evidence_seq == 0U && !record.terminal) {
    const bool full_six = decoded.expected_mask() == 0x3fU &&
        decoded.submitted_mask() == 0x3fU && decoded.sent_mask() == 0x3fU &&
        decoded.failed_mask() == 0U;
    record.first_page = {page.boot_epoch, page.session_id, page.run_id,
        decoded.motion_epoch(), page.evidence_seq, page.group_count,
        page.attempt_kind, page.category, decoded.expected_mask(),
        decoded.submitted_mask(), decoded.sent_mask(), decoded.failed_mask(),
        page.final, page.evidence_lost, true, full_six,
        page.attempt_kind == 1U && page.parent_run_id == 0U &&
        page.category == 1U && page.group_count >= 1U && decoded.group_id() != 0U &&
        decoded.motion_epoch() != 0U && decoded.final_state() == 0U &&
        !decoded.motion_complete() && !decoded.release_committed() &&
        page.final && !page.evidence_lost && full_six};
  }
  if (!enqueue(record)) return false;
  last_chain_ = page.chain_sha256;
  next_seq_ = page.evidence_seq + 1U;
  terminal_seen_ = page.category == 4U;
  if (!terminal_seen_) {
    ++can_page_count_;
    last_reported_group_count_ = page.group_count;
  }
  if (terminal_seen_) {
    for (std::size_t i = 0U; i < expected_count_; ++i)
      if (expected_runs_[i].run_id == page.run_id) {
        expected_runs_[i].terminal = true;
        expected_runs_[i].terminal_record_end = produced_.load(std::memory_order_acquire);
      }
  }
  return true;
}

bool RawEvidenceCollector::write_record(const Record &record) noexcept {
  // Version 1: fixed 39-byte little-endian record header, followed by exact
  // wire codec bytes. Length permits future wire variants without ambiguity.
  std::uint8_t header[39]{};
  header[0] = static_cast<std::uint8_t>(record.kind);
  put_le(header + 1U, record.size, 2U);
  put_le(header + 3U, record.received_at_us, 8U);
  put_le(header + 11U, record.boot_epoch, 8U);
  put_le(header + 19U, record.session_id, 8U);
  put_le(header + 27U, record.run_id, 8U);
  put_le(header + 35U, record.evidence_seq, 4U);
  return std::fwrite(header, 1U, sizeof(header), file_) == sizeof(header) &&
      std::fwrite(record.raw.data(), 1U, record.size, file_) == record.size;
}

void RawEvidenceCollector::write_loop() noexcept {
  while (running_.load(std::memory_order_acquire) ||
         consumed_.load(std::memory_order_relaxed) != produced_.load(std::memory_order_acquire)) {
    const auto consumed = consumed_.load(std::memory_order_relaxed);
    if (consumed == produced_.load(std::memory_order_acquire)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    const Record record = queue_[consumed % kCapacity];
    if (!write_record(record)) {
      fail(Failure::kIo);
      return;
    }
    const bool first_page = record.first_page.run_id != 0U;
    if ((record.terminal || first_page) &&
        ((record.terminal && before_terminal_flush_ &&
          !before_terminal_flush_(flush_context_)) ||
         (flush_file_ ? flush_file_(file_, flush_context_) : std::fflush(file_)) != 0)) {
      fail(Failure::kIo);
      return;
    }
    consumed_.store(consumed + 1U, std::memory_order_release);
    if(channel_ && (record.kind==Kind::kState || record.kind==Kind::kStatus) && failure()==Failure::kNone)
      channel_->publish_raw_cleanup_observation(record.kind==Kind::kState,record.raw.data(),record.size,record.received_at_us,record.cleanup_producer_valid,record.cleanup_generation);
    if (record.kind == Kind::kPage) ++pages_written_;
    if (record.first_page.run_id != 0U && channel_ && failure() == Failure::kNone)
      channel_->publish_raw_evidence_first_page_written(record.first_page);
    if (record.terminal) {
      if (channel_) channel_->publish_raw_evidence_terminal_flushed(
          {record.boot_epoch, record.session_id, record.run_id,
           record.parent_run_id, record.attempt_kind, record.final_state,
           record.first_failure_source, record.first_failure_axis_mask,
           record.evidence_lost, record.motion_complete,
           record.release_committed, true, record.safety_hold_motion_epoch,
           record.safety_hold_valid,record.attempt_motion_epoch,record.fault_stop_motion_epoch,
           record.terminal_received_us,record.terminal_chain,record.full_fault_groups_valid,record.identity_hashes,record.terminal_group_count});
      terminal_written_run_id_.store(record.run_id, std::memory_order_release);
    }
    ++written_;
  }
}

}  // namespace dummy_ros_control

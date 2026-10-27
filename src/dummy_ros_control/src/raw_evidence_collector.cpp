/**
 * @file raw_evidence_collector.cpp
 * @brief 实现有界原始证据校验与私有writer归档。
 * 生产回调复制/校验/入256槽队列，捕获文件由writer独占fwrite和flush；channel为空时序号/组数错误仍有同步stderr诊断，带channel的生产路径使用固定journal。first/terminal回执在相应写出刷新成功后发布；这里未调用fsync，不声称断电持久性或动作资格。任何首错粘滞并阻止继续入队。
 */
#include "dummy_ros_control/raw_evidence_collector.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <openssl/sha.h>
#include <unistd.h>

#include "dummy_ref/wire_validation.hpp"
namespace collector_wire = dp::iface;
#include "dummy_ros_control/device_management_channel.hpp"

namespace dummy_ros_control {
namespace {
/**
 * @brief 把一个整数低width字节写为小端。
 * @param out 至少width字节可写缓冲。
 * @param value 整数源位型。
 * @param width 当前调用为2/4/8，不超过8。
 */
void put_le(std::uint8_t *out, std::uint64_t value, std::size_t width) noexcept {
  for (std::size_t i = 0U; i < width; ++i) out[i] = static_cast<std::uint8_t>(value >> (8U * i));
}

/**
 * @brief 从根目录逐段打开无符号链接路径并独占创建证据文件。
 * 目录openat使用O_DIRECTORY/O_NOFOLLOW，末项O_EXCL权限0600；不自动建目录或覆盖已有文件。
 * @param path 规范绝对路径，不含NUL、空目录段、点或父目录段。
 * @return 文件描述符；任一目录/创建错误返回-1。
 */
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

/**
 * @brief 查找run绑定的只读借用地址。
 * @param run_id 尝试ID。
 * @return 内部槽指针或nullptr；后续observe可能回收该槽，调用方不可无限保存指针。
 */
const RawEvidenceBinding *RawEvidenceBindings::find(std::uint64_t run_id) const noexcept {
  for (std::size_t i = 0U; i < count_; ++i)
    if (slots_[i].run_id == run_id) return &slots_[i];
  return nullptr;
}

/**
 * @brief 观察同run一致身份或登记单调新增run。
 * 已有run允许session_id=0只作其他字段核对；真正新run不能用0会话。表满无可回收槽即拒绝，不动态扩容。
 * @param binding 待核对尝试绑定。
 * @param terminal_written_run_id writer已刷新终页水位，允许回收不晚于此的旧槽。
 * @return bool；新绑定必须session非零、run递增且parent关系合法。
 */
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

/**
 * @brief 冻结身份并保存可选channel及flush钩子的借用。
 * @param identity 构造首链seed所用固定身份。
 * @param channel 可空共享管理对象裸指针，外部保证寿命覆盖collector。
 * @param before_terminal_flush 可选终页flush前判定函数。
 * @param flush_context 两个钩子的借用上下文。
 * @param flush_file 可选刷新函数，空则std::fflush。
 */
RawEvidenceCollector::RawEvidenceCollector(RawEvidenceIdentity identity,
    DeviceManagementChannel *channel, BeforeTerminalFlush before_terminal_flush,
    void *flush_context, FlushFile flush_file)
    : identity_(std::move(identity)), channel_(channel),
      before_terminal_flush_(before_terminal_flush), flush_context_(flush_context),
      flush_file_(flush_file) {}
/**
 * @brief 调用stop排空/关闭并检查未封口run。
 */
RawEvidenceCollector::~RawEvidenceCollector() { stop(); }

/**
 * @brief 独占创建新捕获文件、写DPRaw头并启动writer。
 * 文件权限0600，逐段openat+O_NOFOLLOW，最终O_EXCL；不覆盖旧证据。启动不清除此前首错/验证水位，本对象应按一个采集寿命使用。
 * @param path 绝对路径，各目录段存在且无符号链接，目标必须不存在。
 * @return bool；重复启动/无路径/打开或线程创建失败返回false。
 */
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

/**
 * @brief 标记未终结run为Incomplete并结束/等待writer和关闭文件。
 * 正常writer排空已排队记录，I/O错误可能提前结束；join/flush/close均非RT。无返回值，应读取failure而非假设证据完整。
 */
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

/**
 * @brief 登记未来应收到终页的run并限制期望槽容量。
 * @param run_id 非零且新增时单调递增的尝试ID。
 * @return bool；重复期望允许，满槽且无已消费终页可回收时粘滞Overflow。
 */
bool RawEvidenceCollector::expect_run(std::uint64_t run_id,
    std::uint32_t verified_pre_motion_epoch) noexcept {
  if (failure() != Failure::kNone) return false;
  if (run_id == 0U) { fail(Failure::kInvalid); return false; }
  for (std::size_t i = 0U; i < expected_count_; ++i)
    if (expected_runs_[i].run_id == run_id) {
      // 同生产者已接纳首CAN后，任何迟到Status都不能重写attempt来源。
      if (verified_pre_motion_epoch != 0U && !expected_runs_[i].terminal &&
          (run_id_ != run_id || next_seq_ == 0U) &&
          verified_pre_motion_epoch >= expected_runs_[i].pre_motion_epoch)
        expected_runs_[i].pre_motion_epoch = verified_pre_motion_epoch;
      return true;
    }
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
  expected_runs_[slot] = {run_id, 0U, false, verified_pre_motion_epoch};
  last_expected_run_id_ = run_id;
  return true;
}

/**
 * @brief 核对首seed或前页链，以及原页规范前缀的SHA-256。
 * 首seed为192字节零初始化，身份字段占前160字节；页摘要由前链+codec规范前缀+evidence_lost尾字节计算，不包含自引用摘要。
 * @param page 本次原页与前/当前摘要。
 * @return bool；首seed绑定run/boot/client及四类hash，后页接last_chain。
 */
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
  constexpr std::size_t kPrefix = collector_wire::DummyAttemptEvidenceV2Codec::kOffset_previous_chain_sha256;
  constexpr std::size_t kLostOffset = collector_wire::DummyAttemptEvidenceV2Codec::kOffset_evidence_lost;
  static_assert(RawEvidencePage::kRawSize == kLostOffset + 1U);
  std::array<std::uint8_t, 32U + kPrefix + 1U> input{};
  std::copy(expected_previous.begin(), expected_previous.end(), input.begin());
  std::copy_n(page.raw.begin(), kPrefix, input.begin() + 32U);
  input.back() = page.raw[kLostOffset];
  std::array<std::uint8_t, 32U> computed{};
  return ::SHA256(input.data(), input.size(), computed.data()) && computed == page.chain_sha256;
}

/**
 * @brief 只保留第一个失败并同步到可选channel。
 * 后续失败不覆盖原因为便于定位首因；不自动请求设备运动或停止。
 * @param value 待锁存失败原因。
 */
void RawEvidenceCollector::fail(Failure value) noexcept {
  Failure expected = Failure::kNone;
  if (failure_.compare_exchange_strong(expected, value, std::memory_order_acq_rel) && channel_)
    channel_->raw_evidence_failure.store(static_cast<std::uint8_t>(value),
                                         std::memory_order_release);
}

/**
 * @brief 将一条已复制记录交给唯一writer。
 * @param record 单生产者拥有的完整值。
 * @return bool；已有首错或满256槽拒绝。
 */
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

/**
 * @brief 复制State/Status原始字节并携带清理生命周期代入队。
 * @param kind 仅kState或kStatus。
 * @param raw 借用字节，调用时有效。
 * @param size 非零且≤651字节。
 * @param received_at_us 宿主接收µs。
 * @param boot_epoch 提取的设备boot。
 * @param session_id 提取的设备session。
 * @param cleanup_producer_valid 故障State源时钟校验结果，只供清理lane。
 * @return bool：入RAM队列成功，不表示此帧通过正常控制校验。
 */
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

/**
 * @brief 核对请求codec并复制到采集队列。
 * @param raw 管理请求原载荷。
 * @param size 必须等于请求codec编码长度。
 * @param sent_at_us 宿主发送/提交观察µs，不是设备接纳时刻。
 * @return bool；未running或codec错误粘滞失败。
 */
bool RawEvidenceCollector::offer_management_request(const std::uint8_t *raw,
    std::size_t size, std::uint64_t sent_at_us) noexcept {
  using namespace collector_wire;
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

/**
 * @brief 核对管理响应codec并保留原字节。
 * @param raw 借用响应载荷。
 * @param size 精确codec长度。
 * @param received_at_us 宿主收到响应µs。
 * @return bool：合法并入队，不代表管理操作结果一定成功。
 */
bool RawEvidenceCollector::offer_management_response(const std::uint8_t *raw,
    std::size_t size, std::uint64_t received_at_us) noexcept {
  using namespace collector_wire;
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

/**
 * @brief 校验run链、页序、group计数和原字节一致性后排队。
 * 根run首seq=0，后续run须在前run终页后单调且保持boot/session；拒绝丢证据、异常final/kind/parent/链。category4核终页组数，首非终页及终页各导出额外证明；单生产者调用。
 * @param page 固定372字节原页及提取元数据。
 * @param received_at_us 宿主steady接收µs。
 * @return bool仅表示通过验证且RAM入队；真正写出/flush稍后发生。
 */
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
    cancel_origin_epoch_ = 0U;
    for (std::size_t i = 0U; i < expected_count_; ++i)
      if (expected_runs_[i].run_id == page.run_id)
        cancel_origin_epoch_ = expected_runs_[i].pre_motion_epoch;
    safety_hold_valid_ = false;
    safety_hold_regressed_ = false;
    terminal_seen_ = false;
    accepted_clean_cancel_hold_ = false;
    owner_last_disable_valid_=false;owner_group_counts_valid_=true;
    owner_disable_motion_=0U;owner_disable_source_=0U;owner_disable_axis_mask_=0U;
    owner_connection_generation_=channel_?channel_->worker_connection_generation_.load():0U;
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
  /**
   * @brief 非终页报告组数须严格前进；终页组数必须等于实际已接纳CAN页数，不因设备声明更大group_count就补出缺失原页。
   */
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
  /**
   * @brief 零CAN组终页只接受特定根尝试无动作拒绝终态；不能把任意空原页集合当作完整运行。
   */
  if (page.category == 4U && page.group_count == 0U &&
      (page.final_state != 3U || page.terminal_reason != 14U ||
       page.attempt_kind != 1U || page.parent_run_id != 0U)) {
    fail(Failure::kInvalid);
    return false;
  }
  /**
   * @brief 先校验链再解码核对全部提取元数据，确保外部包装字段与实际372字节载荷相符。
   */
  if (!verify_chain(page)) {
    fail(Failure::kChain);
    return false;
  }
  collector_wire::DummyAttemptEvidenceV2 decoded{};
  if (collector_wire::DummyAttemptEvidenceV2Codec::decode(
          page.raw.data(), page.raw.size(), &decoded) !=
          collector_wire::CodecResult::kOk ||
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
  /**
   * @brief 故障证明独立lane要求六轴SENT且设备queue≤take≤submit≤sent、每轴不超过100ms，顺序为motion→stop→disable；不改变普通collector接纳规则。
   */
  if(page.category!=4U) {
    bool valid=decoded.group_id()>fault_last_group_id_ && decoded.expected_mask()==63U &&
      decoded.submitted_mask()==63U && decoded.sent_mask()==63U && decoded.failed_mask()==0U &&
      !decoded.motion_complete() && !decoded.release_committed() && decoded.final_state()==0U;
    for(std::size_t axis=0;axis<6;++axis) {
      const auto q=decoded.queued_at_us()[axis],take=decoded.taken_at_us()[axis],sub=decoded.submitted_at_us()[axis],sent=decoded.sent_at_us()[axis];
      valid=valid && q!=0U && q<=take && take<=sub && sub<=sent && sent-q<=100000U && decoded.abort_at_us()[axis]==0U;
    }
    // 独立退休证明只消费完整链中最后一组的真实SENT及设备时间，不要求旧fault相位。
    owner_group_counts_valid_=owner_group_counts_valid_ && page.group_count==can_page_count_+1U;
    owner_last_disable_valid_=valid && decoded.category()==3U;
    owner_disable_motion_=decoded.motion_epoch();owner_disable_source_=decoded.first_failure_source();
    owner_disable_axis_mask_=decoded.first_failure_axis_mask();
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
    /**
     * @brief 只有完整fault组相位、唯一disable、终页掩码/时间清零、failure与release等条件都满足才导出全故障证明；终页flags单独不足。
     */
    record.full_fault_groups_valid=fault_groups_valid_ && fault_group_phase_==2U && fault_disable_count_==1U &&
      page.attempt_kind==1U && page.parent_run_id==0U && decoded.motion_epoch()==first_motion_epoch_ &&
      decoded.group_id()==0U && decoded.group_count()==can_page_count_ && decoded.expected_mask()==0U &&
      decoded.submitted_mask()==0U && decoded.sent_mask()==0U && decoded.failed_mask()==0U && zero_terminal_times &&
      decoded.final_state()==4U && decoded.release_committed() && !decoded.motion_complete() && !decoded.evidence_lost() &&
      decoded.first_failure_source()!=0U && decoded.first_failure_source()==fault_group_source_ && decoded.first_failure_axis_mask()==63U;
    record.owner_retirement_disable_valid=owner_last_disable_valid_ && owner_group_counts_valid_ &&
      page.attempt_kind==1U && page.parent_run_id==0U && decoded.motion_epoch()==first_motion_epoch_ &&
      decoded.group_id()==0U && decoded.expected_mask()==0U && decoded.submitted_mask()==0U &&
      decoded.sent_mask()==0U && decoded.failed_mask()==0U && zero_terminal_times &&
      decoded.final_state()==4U && decoded.release_committed() && !decoded.motion_complete() && !decoded.evidence_lost() &&
      decoded.first_failure_source()!=0U && decoded.first_failure_source()==owner_disable_source_ &&
      decoded.first_failure_axis_mask()!=0U && (decoded.first_failure_axis_mask() & ~63U)==0U &&
      decoded.first_failure_axis_mask()==owner_disable_axis_mask_;
    record.owner_retirement_motion_epoch=owner_disable_motion_;
    record.connection_generation=owner_connection_generation_;
    record.safety_hold_motion_epoch = safety_hold_motion_epoch_;
    record.safety_hold_valid = safety_hold_valid_ &&
        (cancel_origin_epoch_ == 0U || cancel_origin_epoch_ <= first_motion_epoch_) &&
        decoded.motion_epoch() == (cancel_origin_epoch_ != 0U ?
          cancel_origin_epoch_ : first_motion_epoch_);
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
  /**
   * @brief RAM入队成功才推进链与next_seq；文件写出尚在writer，后续I/O失败会锁存并阻断资格消费者。
   */
  if (!enqueue(record)) return false;
  last_chain_ = page.chain_sha256;
  next_seq_ = page.evidence_seq + 1U;
  terminal_seen_ = page.category == 4U;
  accepted_clean_cancel_hold_ = record.terminal && record.attempt_kind == 1U &&
      record.parent_run_id == 0U && record.final_state == 7U &&
      record.first_failure_source == 0U && record.first_failure_axis_mask == 0U &&
      !record.evidence_lost && !record.motion_complete && !record.release_committed &&
      record.safety_hold_valid;
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

/**
 * @brief 写39字节小端记录头及精确wire载荷。
 * @param record 待输出记录，size指明有效载荷长度。
 * @return bool：两次fwrite均达到请求长度；本方法本身不fflush/fsync。
 */
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

/**
 * @brief 私有writer按序落盘并在首/终页刷新后发布回执。
 * 空队列休眠1ms；消费水位在成功写/必要flush后推进。State/Status写出后可进入独立cleanup槽，终页flush成功后才发布terminal_written；I/O失败立即锁存并退出。
 */
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
    /**
     * @brief 首个非终页与终页需要flush后才发布回执；before_terminal_flush可注入测试失败。fflush不是fsync。
     */
    const bool first_page = record.first_page.run_id != 0U;
    if ((record.terminal || first_page) &&
        ((record.terminal && before_terminal_flush_ &&
          !before_terminal_flush_(flush_context_)) ||
         (flush_file_ ? flush_file_(file_, flush_context_) : std::fflush(file_)) != 0)) {
      fail(Failure::kIo);
      return;
    }
    /**
     * @brief 成功写/必要flush后才归还队列槽；cleanup观测及终页回执由写出路径发布，不能拿尚在RAM的页作完成凭证。
     */
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
           record.terminal_received_us,record.terminal_chain,record.full_fault_groups_valid,record.identity_hashes,record.terminal_group_count,
           record.owner_retirement_disable_valid,record.owner_retirement_motion_epoch,record.connection_generation});
      terminal_written_run_id_.store(record.run_id, std::memory_order_release);
    }
    ++written_;
  }
}

}  // namespace dummy_ros_control

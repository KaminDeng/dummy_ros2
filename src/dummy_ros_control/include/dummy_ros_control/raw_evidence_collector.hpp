/**
 * @file raw_evidence_collector.hpp
 * @brief 声明有界原始证据采集、链校验及writer刷新回执。
 * 单dp_comm回调线程产生定长record，私有writer消费落文件。诊断采集不授予运动或资格；offer成功仅入RAM队列，first/terminal写出并flush后才发布共享回执，flush不等于fsync或掉电原子性。
 */
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

/**
 * @brief 故障清理lane最近源序号/源生产µs和宿主接收µs；不同域分别存放，不能直接混作同epoch。
 */
struct RawFaultProducerClock {
  std::uint32_t last_sequence=0;
  std::uint64_t last_produced=0,last_received=0;
};
// 只用于已建立正常流源时钟锚后的故障清理原页，不产生正常控制/admission资格。
/**
 * @brief 在已有正常流时钟锚下校验故障清理原页源时序。
 * @param received 宿主steady接收µs。
 * @param produced 设备生产µs。
 * @param seq 设备32位序号，0拒绝。
 * @param has_normal 已建立正常流源锚。
 * @param normal_seq 正常流序号水位。
 * @param normal_produced 正常流生产时间水位µs。
 * @param normal_offset 正常流最小接收减生产偏移µs。
 * @param anchor_offset 最初锚偏移µs，可带符号。
 * @param anchor_received 初锚宿主接收µs。
 * @param fault 故障lane水位，只有成功时更新。
 * @param budget 允许偏移变化/排队预算µs。
 * @return bool；拒绝零/倒退/序号过旧/超偏移，不产生正常控制或admission资格。
 */
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

/**
 * @brief 本collector固定客户端ID及契约/模型/profile/固件hash，用于首链seed；不含编译策略hash，不自行推导设备资格。
 */
struct RawEvidenceIdentity {
  std::array<std::uint8_t, 16U> client_id{};
  std::array<std::uint8_t, 32U> contract_hash{};
  std::array<std::uint8_t, 32U> model_hash{};
  std::array<std::uint8_t, 32U> profile_hash{};
  std::array<std::uint8_t, 32U> firmware_hash{};
};

/**
 * @brief 一次尝试run与boot/session/parent/kind绑定；kind标根/子尝试类型，不是运动目标UUID。
 */
struct RawEvidenceBinding {
  std::uint64_t run_id = 0U;
  std::uint64_t boot_epoch = 0U;
  std::uint64_t session_id = 0U;
  std::uint64_t parent_run_id = 0U;
  std::uint8_t attempt_kind = 0U;
};

/**
 * @brief 最多8槽的run身份表，可复用已写终页的最旧槽。
 */
class RawEvidenceBindings final {
 public:
  /**
   * @brief 观察同run一致身份或登记单调新增run。
   * 已有run允许session_id=0只作其他字段核对；真正新run不能用0会话。表满无可回收槽即拒绝，不动态扩容。
   * @param binding 待核对尝试绑定。
   * @param terminal_written_run_id writer已刷新终页水位，允许回收不晚于此的旧槽。
   * @return bool；新绑定必须session非零、run递增且parent关系合法。
   */
  bool observe(const RawEvidenceBinding &binding,
               std::uint64_t terminal_written_run_id) noexcept;
  /**
   * @brief 查找run绑定的只读借用地址。
   * @param run_id 尝试ID。
   * @return 内部槽指针或nullptr；后续observe可能回收该槽，调用方不可无限保存指针。
   */
  const RawEvidenceBinding *find(std::uint64_t run_id) const noexcept;

 private:
  /**
   * @brief 定长绑定表及已用数量/最大新run；生命周期与collector worker一致。
   */
  std::array<RawEvidenceBinding, 8U> slots_{};
  std::size_t count_ = 0U;
  std::uint64_t last_run_id_ = 0U;
};

/**
 * @brief 372字节原页及已提取元数据/链；final表示此页记录已封口，只有category=4才是整个attempt终页。group_count和序号必须另核对。
 */
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

/**
 * @brief 首个非终原页已写/刷新后的证明字段；full_six核六轴mask，strict_refresh_successor要求根运动页等附加条件，均非动作到位凭证。
 */
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
/**
 * @brief 固定256槽原页/观测采集器及单writer文件所有者。
 * start/stop属于非RT生命周期，offer侧不写捕获文件；生产路径带channel时写固定诊断，channel为空的序号/组数错误分支会同步fprintf(stderr)，不能把任意用法都称为无I/O。失败是首错粘滞，拒绝后续enqueue。调用方须先停止生产者再stop/析构，避免并发修改单生产者验证状态。
 */
class RawEvidenceCollector final {
 public:
  /**
   * @brief 测试/集成注入：终页flush前钩子，False令采集失败；FlushFile替代fflush，返回0表示成功，context借用须活到stop。
   */
  using BeforeTerminalFlush = bool (*)(void *) noexcept;
  using FlushFile = int (*)(std::FILE *, void *) noexcept;
  /**
   * @brief 256个定长RAM槽，每条最大651字节载荷；容量满即Failure::kOverflow，不覆盖尚未写出的证据。
   */
  static constexpr std::size_t kCapacity = 256U;
  static constexpr std::size_t kMaxRawSize = 651U;
  /**
   * @brief DPRaw记录类型：证据页/State/Status/管理请求/响应；每类载荷保持实际wire codec字节。
   */
  enum class Kind : std::uint8_t {
    kPage = 1U, kState = 2U, kStatus = 3U,
    kManagementRequest = 4U, kManagementResponse = 5U
  };
  /**
   * @brief 首个采集失败原因；Sequence/Chain/Identity/Overflow/Io/Invalid/Incomplete互不混同，非零状态不会自动恢复。
   */
  enum class Failure : std::uint8_t {
    kNone, kSequence, kChain, kIdentity, kOverflow, kIo, kInvalid, kIncomplete
  };

  /**
   * @brief 冻结身份并保存可选channel及flush钩子的借用。
   * @param identity 构造首链seed所用固定身份。
   * @param channel 可空共享管理对象裸指针，外部保证寿命覆盖collector。
   * @param before_terminal_flush 可选终页flush前判定函数。
   * @param flush_context 两个钩子的借用上下文。
   * @param flush_file 可选刷新函数，空则std::fflush。
   */
  explicit RawEvidenceCollector(RawEvidenceIdentity identity,
                                DeviceManagementChannel *channel = nullptr,
                                BeforeTerminalFlush before_terminal_flush = nullptr,
                                void *flush_context = nullptr,
                                FlushFile flush_file = nullptr);
  /**
   * @brief 调用stop排空/关闭并检查未封口run。
   */
  ~RawEvidenceCollector();
  /**
   * @brief 禁止复制writer和文件所有者。
   */
  RawEvidenceCollector(const RawEvidenceCollector &) = delete;
  /**
   * @brief 禁止复制赋值采集器。
   */
  RawEvidenceCollector &operator=(const RawEvidenceCollector &) = delete;
  /**
   * @brief 独占创建新捕获文件、写DPRaw头并启动writer。
   * 文件权限0600，逐段openat+O_NOFOLLOW，最终O_EXCL；不覆盖旧证据。启动不清除此前首错/验证水位，本对象应按一个采集寿命使用。
   * @param path 绝对路径，各目录段存在且无符号链接，目标必须不存在。
   * @return bool；重复启动/无路径/打开或线程创建失败返回false。
   */
  bool start(const std::string &path);
  /**
   * @brief 标记未终结run为Incomplete并结束/等待writer和关闭文件。
   * 正常writer排空已排队记录，I/O错误可能提前结束；join/flush/close均非RT。无返回值，应读取failure而非假设证据完整。
   */
  void stop() noexcept;
  /**
   * @brief 校验run链、页序、group计数和原字节一致性后排队。
   * 根run首seq=0，后续run须在前run终页后单调且保持boot/session；拒绝丢证据、异常final/kind/parent/链。category4核终页组数，首非终页及终页各导出额外证明；单生产者调用。
   * @param page 固定372字节原页及提取元数据。
   * @param received_at_us 宿主steady接收µs。
   * @return bool仅表示通过验证且RAM入队；真正写出/flush稍后发生。
   */
  bool offer_page(const RawEvidencePage &page, std::uint64_t received_at_us) noexcept;
  /**
   * @brief 登记未来应收到终页的run并限制期望槽容量。
   * @param run_id 非零且新增时单调递增的尝试ID。
   * @return bool；重复期望允许，满槽且无已消费终页可回收时粘滞Overflow。
   */
  bool expect_run(std::uint64_t run_id) noexcept;
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
  bool offer_observation(Kind kind, const std::uint8_t *raw, std::size_t size,
                         std::uint64_t received_at_us, std::uint64_t boot_epoch,
                         std::uint64_t session_id, bool cleanup_producer_valid=false) noexcept;
  /**
   * @brief 核对请求codec并复制到采集队列。
   * @param raw 管理请求原载荷。
   * @param size 必须等于请求codec编码长度。
   * @param sent_at_us 宿主发送/提交观察µs，不是设备接纳时刻。
   * @return bool；未running或codec错误粘滞失败。
   */
  bool offer_management_request(const std::uint8_t *raw, std::size_t size,
                                std::uint64_t sent_at_us) noexcept;
  /**
   * @brief 核对管理响应codec并保留原字节。
   * @param raw 借用响应载荷。
   * @param size 精确codec长度。
   * @param received_at_us 宿主收到响应µs。
   * @return bool：合法并入队，不代表管理操作结果一定成功。
   */
  bool offer_management_response(const std::uint8_t *raw, std::size_t size,
                                 std::uint64_t received_at_us) noexcept;
  /**
   * @brief 由外层发现证据矛盾时锁存Invalid首错。
   */
  void reject_invalid() noexcept { fail(Failure::kInvalid); }
  /**
   * @brief 以acquire读取采集首错。
   * @return Failure；None仅表示目前未记录失败，不证明run已完整。
   */
  Failure failure() const noexcept { return failure_.load(std::memory_order_acquire); }
  /**
   * @brief 读取writer处理成功的全部记录数。
   * @return 次数，包含页、状态、管理等记录。
   */
  std::uint64_t written() const noexcept { return written_.load(std::memory_order_acquire); }
  /**
   * @brief 读取已写证据页数。
   * @return 次数，不含State/Status。
   */
  std::uint64_t pages_written() const noexcept { return pages_written_.load(std::memory_order_acquire); }
  /**
   * @brief 读取已经完成终页写出及flush的最大当前run水位。
   * @return run ID；0表示无终页回执，非设备自身动作状态。
   */
  std::uint64_t terminal_written_run_id() const noexcept {
    return terminal_written_run_id_.load(std::memory_order_acquire);
  }

 private:
  /**
   * @brief RAM队列拥有的定长原始载荷副本及提取证明；received_at_us为宿主µs，cleanup_generation防writer旧记录跨连接发布。
   */
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
  bool owner_retirement_disable_valid=false;
  std::uint32_t owner_retirement_motion_epoch=0U;
  std::uint64_t connection_generation=0U;

    RawEvidenceFirstPageWritten first_page{};
  };
  /**
   * @brief 将一条已复制记录交给唯一writer。
   * @param record 单生产者拥有的完整值。
   * @return bool；已有首错或满256槽拒绝。
   */
  bool enqueue(const Record &record) noexcept;
  /**
   * @brief 只保留第一个失败并同步到可选channel。
   * 后续失败不覆盖原因为便于定位首因；不自动请求设备运动或停止。
   * @param value 待锁存失败原因。
   */
  void fail(Failure value) noexcept;
  /**
   * @brief 私有writer按序落盘并在首/终页刷新后发布回执。
   * 空队列休眠1ms；消费水位在成功写/必要flush后推进。State/Status写出后可进入独立cleanup槽，终页flush成功后才发布terminal_written；I/O失败立即锁存并退出。
   */
  void write_loop() noexcept;
  /**
   * @brief 写39字节小端记录头及精确wire载荷。
   * @param record 待输出记录，size指明有效载荷长度。
   * @return bool：两次fwrite均达到请求长度；本方法本身不fflush/fsync。
   */
  bool write_record(const Record &record) noexcept;
  /**
   * @brief 核对首seed或前页链，以及原页规范前缀的SHA-256。
   * 首seed为192字节零初始化，身份字段占前160字节；页摘要由前链+codec规范前缀+evidence_lost尾字节计算，不包含自引用摘要。
   * @param page 本次原页与前/当前摘要。
   * @return bool；首seed绑定run/boot/client及四类hash，后页接last_chain。
   */
  bool verify_chain(const RawEvidencePage &page) noexcept;

  /**
   * @brief SPSC环：produced/consumed以release/acquire交接，written/pages/terminal为writer发布的观察量。
   */
  std::array<Record, kCapacity> queue_{};
  std::atomic<std::uint64_t> produced_{0U}, consumed_{0U}, written_{0U}, pages_written_{0U};
  std::atomic<std::uint64_t> terminal_written_run_id_{0U};
  std::atomic<Failure> failure_{Failure::kNone};
  std::atomic<bool> running_{false};
  std::thread writer_;
  std::FILE *file_ = nullptr;
  /**
   * @brief 身份值归collector所有；channel与flush_context借用，不得早于stop销毁。file只由生命周期/唯一writer按顺序操作。
   */
  const RawEvidenceIdentity identity_;
  DeviceManagementChannel *const channel_;
  const BeforeTerminalFlush before_terminal_flush_;
  void *const flush_context_;
  const FlushFile flush_file_;
  /**
   * @brief 最多8个应收终页的run；terminal_record_end记录终页入队后水位，writer消费后才可回收。
   */
  struct ExpectedRun {
    std::uint64_t run_id = 0U;
    std::uint64_t terminal_record_end = 0U;
    bool terminal = false;
  };
  std::array<ExpectedRun, 8U> expected_runs_{};
  std::size_t expected_count_ = 0U;
  std::uint64_t last_expected_run_id_ = 0U;
  /**
   * @brief 单生产者验证状态：当前run身份、下一32位页序号、CAN页计数、前链和已见终页；不由writer直接修改。
   */
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
  /**
   * @brief 完整故障证明另lane：运动→停止→唯一disable组的身份/六轴SENT/设备µs时间顺序等，不能与普通取消Hold证明混同。
   */
  // 独立记录最后全轴Disable原页；允许先普通Stop后Fault，不放宽旧相位审计。
  bool owner_last_disable_valid_=false, owner_group_counts_valid_=true;
  std::uint32_t owner_disable_motion_=0U;
  std::uint16_t owner_disable_source_=0U;
  std::uint8_t owner_disable_axis_mask_=0U;
  std::uint64_t owner_connection_generation_=0U;
  bool fault_groups_valid_=true;
  std::uint8_t fault_group_phase_=0U;
  std::uint32_t fault_motion_epoch_=0U, fault_stop_epoch_=0U, fault_disable_count_=0U;
  std::uint64_t fault_last_group_id_=0U;
  std::uint16_t fault_group_source_=0U;

};

}  // namespace dummy_ros_control

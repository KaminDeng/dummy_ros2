/**
 * @file device_management_channel.hpp
 * @brief 定义gate与独占传输worker之间的单槽管理和只读证据交接。
 * 只有一个queued请求及最新State/Status；共享channel可活过worker供读取已发布诊断，但ready失效后不能沿用授权。普通observation与原始故障清理observation分开，后者经过writer写出及generation屏障仍不赋予正常控制资格。
 */
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
/**
 * @brief 一个管理事务的定长请求/响应codec字节及原子canceled/failed/complete；响应先写字节再发布complete，调用方负责按协议同步观察。
 */
struct DeviceManagementRequest {
  std::array<std::uint8_t, dp::protocol::dummy_robot_v2::DummyManageV2RequestCodec::kEncodedSize> request{};
  std::array<std::uint8_t, dp::protocol::dummy_robot_v2::DummyManageV2ResponseCodec::kEncodedSize> response{};
  std::atomic<bool> canceled{false}, failed{false}, complete{false};
};
/**
 * @brief 最新原始State/Status字节及HOST_TRANSPORT_STEADY_US接收时刻；rejected_ack_*记录同身份拒绝命令的独立水位，不能当作成功ACK。
 */
struct DeviceManagementObservation {
  std::array<std::uint8_t, dp::protocol::dummy_robot_v2::DummyJointStateV2Codec::kEncodedSize> state{};
  std::array<std::uint8_t, dp::protocol::dummy_robot_v2::DummyDeviceStatusV2Codec::kEncodedSize> status{};
  std::uint64_t state_received_us=0, status_received_us=0;
  std::uint64_t rejected_ack_boot_epoch=0, rejected_ack_session_id=0, rejected_ack_received_us=0;
  std::uint32_t rejected_ack_reference_epoch=0, rejected_ack_motion_epoch=0,
                rejected_ack_sequence=0;
};
/**
 * @brief 已校验终页且writer刷新完成的回执；boot/session/run与父尝试绑定，chain/组证据和release/motion标志原样导出。terminal_received_us为宿主µs，不是设备完成时间。
 */
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
/**
 * @brief 最新命令排空及Hold核对现场：processed_generation为宿主代数、committed_seq为线序号，position/tolerance为rad，observed/max_age为宿主µs。quiescent非单纯位置接近。
 */
struct HoldDrainSnapshot {
  std::uint64_t boot_epoch=0U, session_id=0U, processed_generation=0U;
  std::uint32_t reference_epoch=0U, motion_epoch=0U, committed_seq=0U;
  std::uint8_t disposition=0U;
  bool quiescent=false;
  std::uint64_t observed_us=0U, max_state_age_us=0U;
  std::array<double,6> position{}, tolerance{};
};
/**
 * @brief 持有定长管理单槽、诊断以及经过writer确认的证据回执。
 * mutex保护queued/observation/hold_drain等非原子组合；各专用回执有独立mutex。调用者不得把所有方法都当RT无阻塞接口，ready只表示worker管理可用。
 */
struct DeviceManagementChannel {
  // 每个DeviceTransport只创建一个channel；保留共享只读句柄可在worker退出后取原记录。
  DiagnosticJournal diagnostics;
  /**
   * @brief 复制并冻结传输参数，按raw路径决定采集开关。
   * @param value 明确端点、身份及限制的配置值。
   */
  explicit DeviceManagementChannel(TransportParameters value)
      : parameters(std::move(value)),
        raw_evidence_capture_enabled(!parameters.raw_evidence_path.empty()) {}
  /**
   * @brief 按32位串行序观察已用管理序号，忽略0和旧值。
   * 获取sequence mutex；同请求序号空间由gate与worker共享，避免各自从1重复分配。
   * @param sequence 实际观察的request_seq，0保留不用。
   */
  void observe_request_sequence(std::uint32_t sequence) noexcept {
    if (sequence == 0U) return;
    std::lock_guard<std::mutex> lock(request_sequence_mutex_);
    if (last_request_sequence_ == 0U ||
        (sequence != last_request_sequence_ &&
         static_cast<std::uint32_t>(sequence - last_request_sequence_) < 0x80000000U))
      last_request_sequence_ = sequence;
  }
  /**
   * @brief 串行分配下一个非零管理序号。
   * @param initial 尚无序号时的起点，0自动改1。
   * @return uint32序号；回绕跳过0。
   */
  std::uint32_t reserve_request_sequence(std::uint32_t initial = 1U) noexcept {
    std::lock_guard<std::mutex> lock(request_sequence_mutex_);
    if (last_request_sequence_ == 0U) last_request_sequence_ = initial == 0U ? 1U : initial;
    else if (++last_request_sequence_ == 0U) last_request_sequence_ = 1U;
    return last_request_sequence_;
  }
  /**
   * @brief 锁下复制最新终页刷新回执。
   * @return 值副本，默认全0表示尚无相应回执。
   */
  RawEvidenceTerminalFlushed raw_evidence_terminal_flushed() const noexcept {
    std::lock_guard<std::mutex> lock(raw_evidence_terminal_mutex_);
    return raw_evidence_terminal_;
  }
  /**
   * @brief 发布完整终页刷新/链证明回执。
   * 只复制到共享槽；方法本身不执行磁盘flush，也不重新验证链。
   * @param receipt writer已完成刷新后生成的组合记录。
   */
  void publish_raw_evidence_terminal_flushed(
      const RawEvidenceTerminalFlushed &receipt) noexcept {
    std::lock_guard<std::mutex> lock(raw_evidence_terminal_mutex_);
    raw_evidence_terminal_ = receipt;
  }
  /**
   * @brief 兼容发布仅含boot/session/run的最小回执。
   * 其余证明字段默认零/false，不能据此替代完整chain_valid或release证明。
   * @param boot_epoch 设备启动ID。
   * @param session_id 设备会话ID。
   * @param run_id 尝试ID。
   */
  void publish_raw_evidence_terminal_flushed(std::uint64_t boot_epoch,
                                             std::uint64_t session_id,
                                             std::uint64_t run_id) noexcept {
    publish_raw_evidence_terminal_flushed({boot_epoch, session_id, run_id});
  }
  /**
   * @brief 复制最新已写出的首原页回执。
   * @return 完整值副本；0 run表示未发布。
   */
  RawEvidenceFirstPageWritten raw_evidence_first_page_written() const noexcept {
    std::lock_guard<std::mutex> lock(raw_evidence_first_page_mutex_);
    return raw_evidence_first_page_;
  }
  /**
   * @brief 仅接受run_id更大的首原页写出记录。
   * 相同/更旧run不覆盖；跨run单调关系由collector/绑定表保证。
   * @param receipt writer发布的首原页回执。
   */
  void publish_raw_evidence_first_page_written(
      const RawEvidenceFirstPageWritten &receipt) noexcept {
    std::lock_guard<std::mutex> lock(raw_evidence_first_page_mutex_);
    if (receipt.run_id > raw_evidence_first_page_.run_id)
      raw_evidence_first_page_ = receipt;
  }
  /**
   * @brief 读取当前cleanup生命周期中已经落入writer的原始清理观测。
   * 此值即使原始页已写出也不是正常控制State授权；state_received_us只在producer时钟校验成功时非0。
   * @return ready且generation非0时返回快照，否则默认空记录。
   */
  DeviceManagementObservation raw_cleanup_observation() const noexcept {
    std::lock_guard<std::mutex> lock(raw_cleanup_mutex_);return ready.load(std::memory_order_acquire) && raw_cleanup_generation_.load(std::memory_order_acquire)!=0U?raw_cleanup_:DeviceManagementObservation{};
  }
  /**
   * @brief 读取cleanup生命周期代数。
   * @return 非零生命周期代或0（禁用/代数耗尽）；仍须检查ready，不表示消息新鲜。
   */
  std::uint64_t raw_cleanup_generation() const noexcept {return raw_cleanup_generation_.load(std::memory_order_acquire);}
  /**
   * @brief 推进cleanup代并清旧清理快照。
   * generation为0或UINT64_MAX时置0永久拒绝该代；防止排队旧record在断连后重新赋予新生命周期观测。
   */
  void reset_raw_cleanup_lifetime() noexcept {
    std::lock_guard<std::mutex> lock(raw_cleanup_mutex_);const auto generation=raw_cleanup_generation_.load(std::memory_order_relaxed);
    raw_cleanup_generation_=generation==0U || generation==UINT64_MAX?0U:generation+1U;raw_cleanup_={};
  }
  /**
   * @brief 仅向仍ready且generation匹配的清理槽发布已写记录。
   * 锁内再次检查ready和代数，避免writer排空旧队列跨连接发布。
   * @param state True为State，False为Status。
   * @param raw 调用期间有效的codec字节。
   * @param size 必须精确等于对应固定数组长度。
   * @param received 宿主steady接收µs。
   * @param producer_valid 仅State使用，源时钟校验失败时其接收水位置0。
   * @param generation 入collector时捕获的cleanup代。
   */
  void publish_raw_cleanup_observation(bool state,const std::uint8_t* raw,std::size_t size,std::uint64_t received,bool producer_valid,std::uint64_t generation) noexcept {
    std::lock_guard<std::mutex> lock(raw_cleanup_mutex_);
    if(!ready.load(std::memory_order_acquire) || generation==0U || generation!=raw_cleanup_generation_.load(std::memory_order_acquire))return;
    if(state && size==raw_cleanup_.state.size()) {std::copy_n(raw,size,raw_cleanup_.state.begin());raw_cleanup_.state_received_us=producer_valid?received:0U;}
    if(!state && size==raw_cleanup_.status.size()) {std::copy_n(raw,size,raw_cleanup_.status.begin());raw_cleanup_.status_received_us=received;}
  }
  /**
   * @brief 冻结配置用于注册表身份匹配；普通mutex保护单槽事务组合。ready与inactive_prime_allowed等原子只给局部可见性，不代替会话校验。
   */
  const TransportParameters parameters;
  std::mutex mutex;
  std::atomic<bool> ready{false};
  std::atomic<bool> inactive_prime_allowed{false};
  /**
   * @brief Complete后的新Begin屏障及六轴rad Hold容差；原始证据开关/首错失效标志独立，非零failure保持粘滞。
   */
  std::atomic<bool> post_complete_begin_ready{true};
  std::array<std::atomic<double>,6> post_complete_hold_tolerance_rad{};
  std::atomic<bool> raw_evidence_capture_enabled{false};
  // Zero means capture is disabled or has not failed; nonzero is sticky.
  std::atomic<std::uint8_t> raw_evidence_failure{0U};
  /**
   * @brief 最多一个共享请求，调用方与worker共享寿命；observation为正常最近帧，hold_drain为排空证明，不是无界消息队列。
   */
  std::shared_ptr<DeviceManagementRequest> queued;
  DeviceManagementObservation observation;
  HoldDrainSnapshot hold_drain;
 private:
  /**
   * @brief cleanup、管理序号及首/终页各有独立锁/水位；共享句柄保留不等于worker仍在线。
   */
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

/**
 * @file device_management_channel.hpp
 * @brief 定义gate与独占传输worker之间的单槽管理和只读证据交接。
 * 只有一个queued请求及最新State/Status；共享channel可活过worker供读取已发布诊断，但ready失效后不能沿用授权。普通observation与原始故障清理observation分开，后者经过writer写出及generation屏障仍不赋予正常控制资格。
 */
#pragma once
#include <array>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include "dummy_ros_control/wire_types.hpp"
#include "dummy_ros_control/device_transport.hpp"
#include "dummy_ref/manual_reference_policy.hpp"
#include "dummy_ref/session_lifecycle_policy.hpp"
#include "dummy_ros_control/raw_evidence_collector.hpp"

namespace dummy_ros_control {
// One pending request and one latest observation: capacity is independent of traffic rate.
/**
 * @brief 一个管理事务的定长请求/响应codec字节及原子canceled/failed/complete；响应先写字节再发布complete，调用方负责按协议同步观察。
 */
struct DeviceManagementRequest {
  std::uint64_t connection_generation=0U; // worker发送时固定，拒绝旧连接迟到回执。
  std::array<std::uint8_t, dummy_wire::DummyManageV2RequestCodec::kEncodedSize> request{};
  std::array<std::uint8_t, dummy_wire::DummyManageV2ResponseCodec::kEncodedSize> response{};
  std::atomic<bool> canceled{false}, failed{false}, complete{false};
};
/**
 * @brief 最新原始State/Status字节及HOST_TRANSPORT_STEADY_US接收时刻；rejected_ack_*记录同身份拒绝命令的独立水位，不能当作成功ACK。
 */
struct DeviceManagementObservation {
  std::array<std::uint8_t, dummy_wire::DummyJointStateV2Codec::kEncodedSize> state{};
  std::array<std::uint8_t, dummy_wire::DummyDeviceStatusV2Codec::kEncodedSize> status{};
  std::uint64_t state_received_us=0, status_received_us=0;
  std::uint64_t rejected_ack_boot_epoch=0, rejected_ack_session_id=0, rejected_ack_received_us=0;
  std::uint32_t rejected_ack_reference_epoch=0, rejected_ack_motion_epoch=0,
                rejected_ack_sequence=0;
};
/** @brief 原Worker D10经精确Query验证的COMMITTED绑定，只用于连接owner退休。 */
struct WorkerReleaseCommit {
  std::uint64_t request=0, operation=0, run=0, boot=0, session=0, generation=0;
  std::uint32_t reference=0, motion=0, management=0;
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
  // 仅退休owner：完整原链的末Disable全轴SENT；不替代旧完整故障相位审计。
  bool owner_retirement_disable_valid=false;
  std::uint32_t owner_retirement_motion_epoch=0U;
  std::uint64_t connection_generation=0U;

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
  // 独立UI诊断lane：不写normal observation、不生成writer/cleanup证明。
  bool publish_diagnostic_status(const dummy_wire::DummyDeviceStatusV2& value,std::uint64_t received) noexcept {
    if(!ready.load(std::memory_order_acquire) || received==0U || value.boot_epoch()==0U ||
       value.schema_major()!=2U || value.schema_minor()!=dummy_wire_schema::kSchemaMinor ||
       value.profile_id()!=parameters.profile_id || value.configured_mask()!=parameters.configured_mask ||
       !std::equal(parameters.device_id.begin(),parameters.device_id.end(),value.device_id()) ||
       !std::equal(parameters.contract_hash.begin(),parameters.contract_hash.end(),value.contract_sha256()) ||
       !std::equal(parameters.model_hash.begin(),parameters.model_hash.end(),value.model_sha256()) ||
       !std::equal(parameters.profile_hash.begin(),parameters.profile_hash.end(),value.profile_sha256()) ||
       !std::equal(parameters.firmware_hash.begin(),parameters.firmware_hash.end(),value.firmware_sha256()) ||
       !std::equal(parameters.policy_hash.begin(),parameters.policy_hash.end(),value.compiled_policy_sha256()))return false;
    std::array<std::uint8_t,dummy_wire::DummyDeviceStatusV2Codec::kEncodedSize> raw{};
    if(dummy_wire::DummyDeviceStatusV2Codec::encode(value,raw.data(),raw.size())!=dummy_wire::CodecResult::kOk)return false;
    std::lock_guard<std::mutex> lock(diagnostic_observation_mutex_);
    if(!ready.load(std::memory_order_acquire) || received<diagnostic_observation_.status_received_us)return false;
    for(std::size_t i=0;i<diagnostic_retired_count_;++i)if(diagnostic_retired_boots_[i]==value.boot_epoch())return false;
    if(diagnostic_status_.boot_epoch()!=0U && diagnostic_status_.boot_epoch()!=value.boot_epoch()) {
      if(diagnostic_retired_count_==diagnostic_retired_boots_.size())return false;
      diagnostic_retired_boots_[diagnostic_retired_count_++]=diagnostic_status_.boot_epoch();
      diagnostic_observation_={};diagnostic_clock_={};diagnostic_anchor_received_=0U;
    } else if(diagnostic_status_.boot_epoch()!=0U &&
       (static_cast<std::int32_t>(value.management_epoch()-diagnostic_status_.management_epoch())<0 ||
        static_cast<std::int32_t>(value.motion_epoch()-diagnostic_status_.motion_epoch())<0))return false;
    if(value.session_id()!=diagnostic_status_.session_id() || value.reference_epoch()!=diagnostic_status_.reference_epoch() ||
       value.motion_epoch()!=diagnostic_status_.motion_epoch() || value.management_epoch()!=diagnostic_status_.management_epoch() ||
       value.core_state()!=diagnostic_status_.core_state())diagnostic_observation_.state_received_us=0U;
    diagnostic_observation_.status=raw;
    diagnostic_status_=value;diagnostic_observation_.status_received_us=received;return true;
  }
  bool publish_diagnostic_state(const dummy_wire::DummyJointStateV2& value,std::uint64_t received) noexcept {
    if(!ready.load(std::memory_order_acquire) || received==0U || value.produced_at_us()==0U || value.state_seq()==0U ||
       received>static_cast<std::uint64_t>(INT64_MAX) || value.produced_at_us()>static_cast<std::uint64_t>(INT64_MAX) ||
       value.schema_major()!=2U || value.schema_minor()!=dummy_wire_schema::kSchemaMinor ||
       value.profile_id()!=parameters.profile_id || value.configured_mask()!=parameters.configured_mask ||
       !std::equal(parameters.device_id.begin(),parameters.device_id.end(),value.device_id()))return false;
    for(std::size_t axis=0;axis<6;++axis) {
      if((value.position_valid_mask()&(1U<<axis)) && !std::isfinite(value.position_rad()[axis]))return false;
      if((value.raw_position_valid_mask()&(1U<<axis)) && !std::isfinite(value.raw_motor_position_rev()[axis]))return false;
    }
    std::array<std::uint8_t,dummy_wire::DummyJointStateV2Codec::kEncodedSize> raw{};
    if(dummy_wire::DummyJointStateV2Codec::encode(value,raw.data(),raw.size())!=dummy_wire::CodecResult::kOk)return false;
    std::lock_guard<std::mutex> lock(diagnostic_observation_mutex_);
    if(!ready.load(std::memory_order_acquire) || diagnostic_observation_.status_received_us==0U ||
       received<diagnostic_observation_.status_received_us ||
       received-diagnostic_observation_.status_received_us>dummy_ref::host::kManualReferenceStatusMaxAgeUs ||
       value.boot_epoch()!=diagnostic_status_.boot_epoch() || value.session_id()!=diagnostic_status_.session_id() ||
       value.reference_epoch()!=diagnostic_status_.reference_epoch() || value.motion_epoch()!=diagnostic_status_.motion_epoch() ||
       value.management_epoch()!=diagnostic_status_.management_epoch() || value.core_state()!=diagnostic_status_.core_state())return false;
    const auto offset=static_cast<std::int64_t>(received)-static_cast<std::int64_t>(value.produced_at_us());
    auto next_clock=diagnostic_clock_;
    if(diagnostic_anchor_received_!=0U && !raw_fault_producer_sample_valid(received,value.produced_at_us(),value.state_seq(),true,
       diagnostic_clock_.last_sequence,diagnostic_clock_.last_produced,diagnostic_minimum_offset_,
       diagnostic_anchor_offset_,diagnostic_anchor_received_,next_clock,parameters.max_state_age_us))return false;
    diagnostic_observation_.state=raw;
    if(diagnostic_anchor_received_==0U) {
      diagnostic_anchor_received_=received;diagnostic_anchor_offset_=diagnostic_minimum_offset_=offset;
    } else diagnostic_minimum_offset_=std::min(diagnostic_minimum_offset_,offset);
    diagnostic_clock_={value.state_seq(),value.produced_at_us(),received};
    diagnostic_observation_.state_received_us=received;return true;
  }
  void reset_diagnostic_observation() noexcept {
    std::lock_guard<std::mutex> lock(diagnostic_observation_mutex_);
    diagnostic_observation_={};diagnostic_status_={};diagnostic_clock_={};
    diagnostic_anchor_received_=0U;diagnostic_retired_count_=0U;
  }
  DeviceManagementObservation diagnostic_observation() const noexcept {
    std::lock_guard<std::mutex> lock(diagnostic_observation_mutex_);
    return ready.load(std::memory_order_acquire)?diagnostic_observation_:DeviceManagementObservation{};
  }
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
  // 只保存原Worker D10的COMMITTED绑定；不改变Stop超时结论或控制权限。
  void publish_worker_release_commit(WorkerReleaseCommit value) noexcept {
    std::lock_guard<std::mutex> lock(worker_release_mutex_);
    value.generation=worker_connection_generation_.load();worker_release_commit_=value;
  }
  WorkerReleaseCommit worker_release_commit_snapshot() const noexcept {
    std::lock_guard<std::mutex> lock(worker_release_mutex_);return worker_release_commit_;
  }
  // Core的Safety子尝试只有自身Disable全轴SENT后才发布cleanup3/releaseCommitted。
  // 此处只消费该固件状态事实退休原owner，绝不声称旧Stop/原页完整或授权恢复。
  bool readonly_fault_owner_retirement_confirmed(std::uint64_t now,std::uint64_t boot,
      std::uint64_t session,std::uint32_t reference) const noexcept {
    if(!ready || !parameters.manual_reference_mode || !boot || !session || !reference)return false;
    WorkerReleaseCommit root;
    {std::lock_guard<std::mutex> lock(worker_release_mutex_);root=worker_release_commit_;}
    const auto current=diagnostic_observation();
    // 仅退休原owner，不验收Stop/原页/动作成功；旧审计首错原样保留。
    if(!root.request || !root.operation || !root.run || root.generation==0U || root.generation!=worker_connection_generation_.load() ||
       root.boot!=boot || root.session!=session || root.reference!=reference ||
       !current.state_received_us || !current.status_received_us ||
       now<current.state_received_us || now<current.status_received_us ||
       now-current.state_received_us>parameters.max_state_age_us ||
       now-current.status_received_us>dummy_ref::host::kManualReferenceStatusMaxAgeUs)return false;
    dummy_wire::DummyJointStateV2 state;dummy_wire::DummyDeviceStatusV2 status;
    if(dummy_wire::DummyJointStateV2Codec::decode(current.state.data(),current.state.size(),&state)!=dummy_wire::CodecResult::kOk ||
       dummy_wire::DummyDeviceStatusV2Codec::decode(current.status.data(),current.status.size(),&status)!=dummy_wire::CodecResult::kOk)return false;
    if(state.boot_epoch()!=boot || status.boot_epoch()!=boot || state.session_id()!=0U || status.session_id()!=0U ||
       state.reference_epoch()!=reference || status.reference_epoch()!=reference || state.core_state()!=9U || status.core_state()!=9U ||
       root.motion==UINT32_MAX || state.motion_epoch()!=root.motion+1U || status.motion_epoch()!=root.motion+1U ||
       state.management_epoch()!=root.management || status.management_epoch()!=root.management ||
       state.operation_id()!=root.operation || status.operation_id()!=root.operation ||
       status.run_id()<=root.run || status.attempt_id()!=status.run_id() || status.attempt_kind()!=2U || status.parent_run_id()!=root.run ||
       status.final_state()!=4U || status.failure_source()!=0x1002U || status.failure_axis_mask()!=63U ||
       status.cleanup_state()!=3U || !status.release_committed() || status.evidence_lost() ||
       status.real_motion_permitted() || status.permit_remaining_us()!=0U || status.can_failed_mask()!=0U ||
       state.raw_position_valid_mask()!=63U)return false;
    return root.generation==worker_connection_generation_.load() && ready;
  }

  // 原Motion根自主Fault释放：只关闭控制连接，绝不授予运动/清错或宣称Goal成功。
  bool autonomous_fault_owner_retirement_confirmed(std::uint64_t now,std::uint64_t boot,
      std::uint64_t session,std::uint32_t reference,std::uint64_t run) const noexcept {
    const auto generation=worker_connection_generation_.load();
    if(!ready || !parameters.manual_reference_mode || !boot || !session || !reference || !run || !generation)return false;
    const auto terminal=raw_evidence_terminal_flushed();
    const std::array<std::array<std::uint8_t,32>,4> hashes{
      parameters.contract_hash,parameters.model_hash,parameters.profile_hash,parameters.firmware_hash};
    if(terminal.connection_generation!=generation || terminal.boot_epoch!=boot || terminal.session_id!=session ||
       terminal.run_id!=run || terminal.attempt_kind!=1U || terminal.parent_run_id!=0U || terminal.final_state!=4U ||
       !terminal.chain_valid || !terminal.owner_retirement_disable_valid || !terminal.owner_retirement_motion_epoch ||
       !terminal.release_committed || terminal.evidence_lost || terminal.motion_complete ||
       !terminal.first_failure_source || !terminal.first_failure_axis_mask ||
       (terminal.first_failure_axis_mask & ~parameters.configured_mask)!=0U ||
       terminal.identity_hashes!=hashes || !terminal.terminal_group_count || !terminal.terminal_received_us ||
       std::all_of(terminal.terminal_chain.begin(),terminal.terminal_chain.end(),[](auto x){return x==0U;}))return false;
    const auto current=diagnostic_observation();
    if(current.state_received_us<terminal.terminal_received_us || current.status_received_us<terminal.terminal_received_us ||
       now<current.state_received_us || now<current.status_received_us ||
       now-current.state_received_us>parameters.max_state_age_us ||
       now-current.status_received_us>dummy_ref::host::kManualReferenceStatusMaxAgeUs)return false;
    dummy_wire::DummyJointStateV2 state;dummy_wire::DummyDeviceStatusV2 status;
    if(dummy_wire::DummyJointStateV2Codec::decode(current.state.data(),current.state.size(),&state)!=dummy_wire::CodecResult::kOk ||
       dummy_wire::DummyDeviceStatusV2Codec::decode(current.status.data(),current.status.size(),&status)!=dummy_wire::CodecResult::kOk)return false;
    // 诊断发布入口已经独立核完整设备/五hash、schema及源时钟；这里核同一终态现场。
    if(state.boot_epoch()!=boot || status.boot_epoch()!=boot || state.session_id()!=0U || status.session_id()!=0U ||
       state.reference_epoch()!=reference || status.reference_epoch()!=reference || state.core_state()!=9U || status.core_state()!=9U ||
       state.motion_epoch()!=terminal.owner_retirement_motion_epoch || status.motion_epoch()!=state.motion_epoch() ||
       state.management_epoch()!=status.management_epoch() || state.operation_id()!=status.operation_id() ||
       status.run_id()!=run || status.attempt_id()!=run || status.attempt_kind()!=1U || status.parent_run_id()!=0U ||
       status.final_state()!=4U || status.failure_source()!=terminal.first_failure_source ||
       status.failure_axis_mask()!=terminal.first_failure_axis_mask || status.cleanup_state()!=3U ||
       !status.release_committed() || status.evidence_lost() || status.real_motion_permitted() ||
       status.permit_remaining_us()!=0U || status.can_failed_mask()!=0U ||
       status.group_count()!=terminal.terminal_group_count)return false;
    // Status可在终页链发布前生成零摘要；非零摘要必须与本代已刷出的终页完全一致。
    const auto chain=status.evidence_chain_sha256();
    const bool nonzero=std::any_of(chain,chain+32,[](auto x){return x!=0U;});
    if(nonzero && !std::equal(terminal.terminal_chain.begin(),terminal.terminal_chain.end(),chain))return false;
    return ready && generation==worker_connection_generation_.load();
  }

  struct OwnedSessionCommit {
    std::uint64_t boot=0,session=0,generation=0,run=0,operation=0;
    std::uint32_t reference=0,motion=0,management=0,ticket=0;
  };
  // 只有本client真实COMMITTED成功回执可以绑定owner；Status中的任意非零会话不能自封所有者。
  bool observe_owned_management_commit(const dummy_wire::DummyManageV2Request& request,
      const dummy_wire::DummyManageV2Response& reply,std::uint64_t generation) noexcept {
    if(!ready || !generation || generation!=worker_connection_generation_.load() ||
       request.schema_major()!=2U || request.schema_minor()!=dummy_wire_schema::kSchemaMinor ||
       reply.schema_major()!=2U || reply.schema_minor()!=dummy_wire_schema::kSchemaMinor ||
       !request.request_id() || request.request_id()!=reply.request_id() ||
       !request.expected_boot_epoch() || reply.boot_epoch()!=request.expected_boot_epoch() ||
       !request.expected_session_id() || reply.session_id()!=request.expected_session_id() ||
       request.expected_profile_id()!=parameters.profile_id || reply.profile_id()!=parameters.profile_id ||
       request.configured_mask()!=parameters.configured_mask || reply.receipt()!=1U ||
       reply.transaction_phase()!=3U || reply.result_code()!=0U || !reply.operation_id() ||
       !std::equal(parameters.device_id.begin(),parameters.device_id.end(),request.device_id()) ||
       !std::equal(parameters.device_id.begin(),parameters.device_id.end(),reply.device_id()) ||
       !std::equal(parameters.client_instance_id.begin(),parameters.client_instance_id.end(),request.client_instance_id()) ||
       !std::equal(parameters.client_instance_id.begin(),parameters.client_instance_id.end(),reply.client_instance_id()))return false;
    std::lock_guard<std::mutex> lock(worker_release_mutex_);
    if(!ready || generation!=worker_connection_generation_.load())return false;
    if(owned_session_commit_.session &&
       (owned_session_commit_.boot!=reply.boot_epoch() || owned_session_commit_.session!=reply.session_id() ||
        reply.reference_epoch()<owned_session_commit_.reference || reply.motion_epoch()<owned_session_commit_.motion ||
        reply.management_epoch()<owned_session_commit_.management || reply.operation_id()<owned_session_commit_.operation))return false;
    attempted_boot_=reply.boot_epoch();ownership_attempted_=true;
    owned_session_commit_={reply.boot_epoch(),reply.session_id(),generation,reply.run_id(),reply.operation_id(),
      reply.reference_epoch(),reply.motion_epoch(),reply.management_epoch()};return true;
  }
  // 本地退休不证明设备已失能，也不清除审计错误。
  void note_management_attempt(std::uint64_t boot) noexcept {
    attempted_boot_=boot;ownership_attempted_=true;
  }
  bool local_readonly_retirement_allowed(std::uint64_t now) const noexcept {
    if(!parameters.manual_reference_mode)return false;
    std::unique_lock<std::mutex> request_lock(mutex,std::try_to_lock);
    if(!request_lock || (queued && !queued->canceled && !queued->failed && !queued->complete))return false;
    OwnedSessionCommit owner;
    {std::lock_guard<std::mutex> lock(worker_release_mutex_);owner=owned_session_commit_;}
    if(!ownership_attempted_.load() && !owner.session)return true;
    const auto old_boot=owner.boot ? owner.boot : attempted_boot_.load();
    const auto current=diagnostic_observation();
    if(!ready || !old_boot || !current.state_received_us || !current.status_received_us ||
       now<current.state_received_us || now<current.status_received_us ||
       now-current.state_received_us>parameters.max_state_age_us ||
       now-current.status_received_us>dummy_ref::host::kManualReferenceStatusMaxAgeUs)return false;
    dummy_wire::DummyJointStateV2 state;dummy_wire::DummyDeviceStatusV2 status;
    if(dummy_wire::DummyJointStateV2Codec::decode(current.state.data(),current.state.size(),&state)!=dummy_wire::CodecResult::kOk ||
       dummy_wire::DummyDeviceStatusV2Codec::decode(current.status.data(),current.status.size(),&status)!=dummy_wire::CodecResult::kOk)return false;
    // 新boot只使旧owner失效；不冒充原D10回执、失能证明或运动成功。
    return status.boot_epoch()!=0U && status.boot_epoch()!=old_boot && state.boot_epoch()==status.boot_epoch() &&
      status.session_id()==0U && state.session_id()==0U && !status.real_motion_permitted() && status.permit_remaining_us()==0U &&
      status.reference_epoch()==state.reference_epoch() && status.core_state()==(status.reference_epoch()==0U?1U:2U) &&
      state.core_state()==status.core_state() && status.first_failure_reason()==0U && state.first_failure_reason()==0U &&
      status.motion_epoch()==state.motion_epoch() && status.management_epoch()==state.management_epoch() &&
      status.operation_id()==state.operation_id();
  }
  std::atomic<bool> ownership_attempted_{false};
  std::atomic<std::uint64_t> attempted_boot_{0U};
  // 仅明确release意图的调用方消费：自主租期失能可退休owner，但不伪造D10/Goal/审计成功。
  bool readonly_released_owner_observed(std::uint64_t now,std::uint64_t boot,std::uint64_t session,
      std::uint32_t reference,std::uint64_t min_root_run) const noexcept {
    OwnedSessionCommit owner;
    {std::lock_guard<std::mutex> lock(worker_release_mutex_);owner=owned_session_commit_;}
    const auto generation=worker_connection_generation_.load();
    if(!ready || !parameters.manual_reference_mode || !session || !reference ||
       !generation || owner.generation!=generation || owner.boot!=boot || owner.session!=session || owner.reference!=reference)return false;
    const auto current=diagnostic_observation();
    if(!current.state_received_us || !current.status_received_us || now<current.state_received_us || now<current.status_received_us ||
       now-current.state_received_us>parameters.max_state_age_us ||
       now-current.status_received_us>dummy_ref::host::kManualReferenceStatusMaxAgeUs)return false;
    dummy_wire::DummyJointStateV2 state;dummy_wire::DummyDeviceStatusV2 status;
    if(dummy_wire::DummyJointStateV2Codec::decode(current.state.data(),current.state.size(),&state)!=dummy_wire::CodecResult::kOk ||
       dummy_wire::DummyDeviceStatusV2Codec::decode(current.status.data(),current.status.size(),&status)!=dummy_wire::CodecResult::kOk)return false;
    const bool lineage=(status.attempt_kind()==1U && status.parent_run_id()==0U && status.final_state()==2U) ||
      (status.attempt_kind()==3U && status.run_id()>1U && status.parent_run_id()==status.run_id()-1U && status.final_state()==8U);
    if(!dummy_ref::host::session_release_observed({true,true,true,status.boot_epoch(),status.core_state()==2U,
          state.core_state()==2U,status.session_id(),status.cleanup_state(),status.permit_remaining_us()}) ||
       state.boot_epoch()!=boot || status.boot_epoch()!=boot || state.session_id()!=0U ||
       state.reference_epoch()!=reference || status.reference_epoch()!=reference ||
       state.motion_epoch()!=status.motion_epoch() || state.management_epoch()!=status.management_epoch() ||
       state.operation_id()!=status.operation_id() || status.motion_epoch()<owner.motion ||
       status.management_epoch()<owner.management || status.operation_id()<owner.operation ||
       !status.run_id() || status.run_id()<min_root_run || status.run_id()<owner.run || status.attempt_id()!=status.run_id() || !lineage ||
       !status.release_committed() || status.evidence_lost() || status.real_motion_permitted() || status.can_failed_mask()!=0U ||
       status.first_failure_reason()!=0U || status.failure_source()!=0U)return false;
    return ready && generation==worker_connection_generation_.load();
  }

  // 仅验证本连接owner的SafetyStop上下文；不更新normal缓存或授权epoch。
  std::optional<OwnedSessionCommit> owner_safety_stop_snapshot(std::uint64_t now,std::uint64_t boot,
      std::uint64_t session,std::uint32_t reference) const noexcept {
    OwnedSessionCommit owner;
    {std::lock_guard<std::mutex> lock(worker_release_mutex_);owner=owned_session_commit_;}
    if(!ready || !parameters.manual_reference_mode || !session || !reference || owner.boot!=boot || owner.session!=session ||
       owner.reference!=reference || !owner.generation || owner.generation!=worker_connection_generation_.load())return {};
    const auto current=diagnostic_observation();DeviceManagementObservation normal;
    {std::lock_guard<std::mutex> lock(mutex);normal=observation;}
    if(!current.state_received_us || !current.status_received_us || !normal.state_received_us || !normal.status_received_us ||
       now<current.state_received_us || now<current.status_received_us ||
       now-current.state_received_us>parameters.max_state_age_us || now-current.status_received_us>dummy_ref::host::kManualReferenceStatusMaxAgeUs)return {};
    dummy_wire::DummyJointStateV2 state,normal_state;dummy_wire::DummyDeviceStatusV2 status,normal_status;
    if(dummy_wire::DummyJointStateV2Codec::decode(current.state.data(),current.state.size(),&state)!=dummy_wire::CodecResult::kOk ||
       dummy_wire::DummyJointStateV2Codec::decode(normal.state.data(),normal.state.size(),&normal_state)!=dummy_wire::CodecResult::kOk ||
       dummy_wire::DummyDeviceStatusV2Codec::decode(current.status.data(),current.status.size(),&status)!=dummy_wire::CodecResult::kOk ||
       dummy_wire::DummyDeviceStatusV2Codec::decode(normal.status.data(),normal.status.size(),&normal_status)!=dummy_wire::CodecResult::kOk)return {};
    if(state.boot_epoch()!=boot || status.boot_epoch()!=boot || state.session_id()!=session || status.session_id()!=session ||
       state.reference_epoch()!=reference || status.reference_epoch()!=reference ||
       state.motion_epoch()!=status.motion_epoch() || state.management_epoch()!=status.management_epoch() ||
       state.operation_id()!=status.operation_id() || state.motion_epoch()<owner.motion || state.management_epoch()<owner.management ||
       normal_state.boot_epoch()!=boot || normal_status.boot_epoch()!=boot || normal_state.session_id()!=session || normal_status.session_id()!=session ||
       normal_state.reference_epoch()!=reference || normal_status.reference_epoch()!=reference ||
       normal_state.motion_epoch()!=state.motion_epoch() || normal_status.motion_epoch()!=state.motion_epoch() ||
       normal_state.management_epoch()!=state.management_epoch() || normal_status.management_epoch()!=state.management_epoch() ||
       normal_state.operation_id()!=state.operation_id() || normal_status.operation_id()!=state.operation_id() ||
       !state.freshness_ticket() || normal_state.freshness_ticket()!=state.freshness_ticket())return {};
    owner.motion=state.motion_epoch();owner.management=state.management_epoch();owner.operation=state.operation_id();owner.ticket=state.freshness_ticket();
    return owner;
  }
  // 返回true仅表示排入一次停止意图，不表示RPC接纳、D10已发或物理失能。
  bool request_owner_safety_stop(std::uint64_t now,std::uint64_t boot,std::uint64_t session,std::uint32_t reference) noexcept {
    const auto owner=owner_safety_stop_snapshot(now,boot,session,reference);if(!owner)return false;
    std::lock_guard<std::mutex> lock(worker_release_mutex_);
    if(!ready || owner_safety_stop_requested_ || owner->generation!=worker_connection_generation_.load())return false;
    owner_safety_stop_requested_=true;owner_safety_stop_intent_=owner;return true;
  }
  // 只有worker消费，消费后再次校验最新源及normal tuple；过期/换代不重开停止截止。
  std::optional<OwnedSessionCommit> take_owner_safety_stop_request(std::uint64_t now) noexcept {
    std::optional<OwnedSessionCommit> intent;
    {std::lock_guard<std::mutex> lock(worker_release_mutex_);intent=owner_safety_stop_intent_;owner_safety_stop_intent_.reset();}
    if(!intent)return {};
    const auto current=owner_safety_stop_snapshot(now,intent->boot,intent->session,intent->reference);
    return current && current->generation==intent->generation?current:std::nullopt;
  }

  // 断连退休原D10证人；collector审计失败不影响此连接生命周期。
  void reset_worker_release_connection() noexcept {
    std::lock_guard<std::mutex> lock(worker_release_mutex_);
    worker_release_commit_={};owned_session_commit_={};owner_safety_stop_intent_.reset();owner_safety_stop_requested_=false;const auto old=worker_connection_generation_.load();
    worker_connection_generation_=old==0U || old==UINT64_MAX ? 0U : old+1U;
  }
  std::atomic<std::uint64_t> worker_connection_generation_{1U};
  mutable std::mutex worker_release_mutex_;
  WorkerReleaseCommit worker_release_commit_{};
  OwnedSessionCommit owned_session_commit_{};
  std::optional<OwnedSessionCommit> owner_safety_stop_intent_;
  bool owner_safety_stop_requested_=false;
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
  mutable std::mutex diagnostic_observation_mutex_;
  DeviceManagementObservation diagnostic_observation_{};
  dummy_wire::DummyDeviceStatusV2 diagnostic_status_{};
  RawFaultProducerClock diagnostic_clock_{};
  std::uint64_t diagnostic_anchor_received_=0U;
  std::int64_t diagnostic_anchor_offset_=0,diagnostic_minimum_offset_=0;
  std::array<std::uint64_t,4> diagnostic_retired_boots_{};
  std::size_t diagnostic_retired_count_=0;
  const TransportParameters parameters;
  mutable std::mutex mutex;
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

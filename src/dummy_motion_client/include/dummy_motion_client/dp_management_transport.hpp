/**
 * @file dp_management_transport.hpp
 * @brief 声明独立/共享worker管理传输和只读观测、释放/故障证据谓词。
 * DeviceWorker模式复用dummy_ros_control唯一通信owner的共享channel；StandaloneWorker自建dp_comm线程及原始观测缓存。接收时间均为宿主steady µs，helper只检查明示字段/水位，不把本地可用、hash一致、终页flush或状态形状单独提升为运动授权。
 */
#pragma once
#include <array>
#include <algorithm>
#include <string>
#include "dummy_motion_client/action_gate.hpp"
#include "dummy_motion_client/management_client.hpp"
namespace dummy_motion_client {
/**
 * @brief 明确端点、16字节设备/客户端ID、profile和配置mask；mask仅1或63，参数校验不等同在线能力bootstrap。
 */
struct DpManagementParameters {
  std::string endpoint;
  std::array<std::uint8_t,16> device{}, client{};
  std::uint32_t profile=0;
  std::uint8_t mask=0;
};
/**
 * @brief State/Status拥有字节副本及宿主接收µs；rejected_ack_*为设备拒绝独立水位。raw_cleanup_producer_valid仅清理lane源时序可信，不能给正常控制准入。
 */
struct DeviceObservation {
  std::vector<std::uint8_t> state, status;
  std::uint64_t state_received_us=0, status_received_us=0;
  std::uint64_t rejected_ack_boot_epoch=0, rejected_ack_session_id=0, rejected_ack_received_us=0;
  std::uint32_t rejected_ack_reference_epoch=0, rejected_ack_motion_epoch=0,
                rejected_ack_sequence=0;
  bool raw_cleanup_producer_valid=false;
};
/** @brief
 * 独立worker收到的原页codec字节及原Host接收微秒，不含身份/验链/释放裁决。 */
struct RawAttemptEvidence {
  std::vector<std::uint8_t> bytes;
  std::uint64_t received_us = 0U;
};
/**
 * @brief 已真实写出/flush终页的尝试身份与链/组证明。terminal_received_us是原宿主接收µs，非机械动作耗时；四类identity_hashes按契约/模型/profile/固件顺序。
 */
struct RawTerminalIdentity {
  std::uint64_t boot_epoch=0, session_id=0, run_id=0;
  std::uint64_t parent_run_id=0;
  std::uint8_t attempt_kind=0, final_state=0;
  std::uint16_t first_failure_source=0;
  std::uint8_t first_failure_axis_mask=0;
  bool evidence_lost=false, motion_complete=false, release_committed=false;
  bool chain_valid=false;
  std::uint32_t safety_hold_motion_epoch=0;
  bool safety_hold_valid=false;
  std::uint32_t attempt_motion_epoch=0U, fault_stop_motion_epoch=0U;
  std::uint64_t terminal_received_us=0U;
  std::array<std::uint8_t,32> terminal_chain{};
  bool full_fault_groups_valid=false;
  std::array<std::array<std::uint8_t,32>,4> identity_hashes{};
  std::uint32_t terminal_group_count=0U;
};
/**
 * @brief 首非终原页writer回执；full_six与strict_refresh_successor为校验条件摘要，final表示原页记录封口而非整个attempt结束。
 */
struct RawFirstPageWritten {
  std::uint64_t boot_epoch=0U, session_id=0U, run_id=0U;
  std::uint32_t motion_epoch=0U, evidence_seq=0U, group_count=0U;
  std::uint8_t attempt_kind=0U, category=0U;
  std::uint8_t expected_mask=0U, submitted_mask=0U, sent_mask=0U, failed_mask=0U;
  bool final=false, evidence_lost=false, chain_valid=false, full_six=false;
  bool strict_refresh_successor=false;
};
/**
 * @brief 仅比较首原页的非零完整尝试身份及motion。
 * @param receipt 首原页回执。
 * @param boot 预期非零boot。
 * @param session 预期非零session。
 * @param run 预期非零run。
 * @param motion_epoch 预期非零motion。
 * @return bool；不单独核chain_valid/full_six等内容证明。
 */
inline bool raw_first_page_matches(const RawFirstPageWritten &receipt,
                                   std::uint64_t boot, std::uint64_t session,
                                   std::uint64_t run, std::uint32_t motion_epoch) noexcept {
  return boot!=0U && session!=0U && run!=0U && motion_epoch!=0U &&
      receipt.boot_epoch==boot && receipt.session_id==session &&
      receipt.run_id==run && receipt.motion_epoch==motion_epoch;
}
/**
 * @brief 比较终页boot/session/run并要求run非零。
 * @param terminal 终页回执。
 * @param boot 预期boot。
 * @param session 预期session。
 * @param run 非零目标run。
 * @return bool；本函数不要求boot/session非零，也不检查chain/释放/成功标志，调用方须叠加。
 */
inline bool raw_terminal_matches(const RawTerminalIdentity &terminal,
                                 std::uint64_t boot,std::uint64_t session,
                                 std::uint64_t run) noexcept {
  return run!=0U && terminal.boot_epoch==boot &&
    terminal.session_id==session && terminal.run_id==run;
}
/**
 * @brief 对受限全六轴Fault清理组合核首/终原页及双fresh故障观测。
 * 还需页/组数/epoch、failure、release及四hash一致，观测晚于终页且非未来。当前Fault Status链可为0，只在完整原terminal权威和严格shape成立时允许；非零链必须完全匹配。此结果只支持故障清理验收，不恢复session/参考/许可或宣称运行成功。
 * @param terminal 已flush终页及完整fault组证明。
 * @param first 相同根run的严格全六轴首运动原页。
 * @param observation writer交付的原始清理State/Status与源时序标志。
 * @param boot 原运行boot。
 * @param session 原运行非零session，故障当前观测应归零。
 * @param run 根尝试ID。
 * @param active_epoch 原活动motion水位。
 * @param profile 非零profile ID。
 * @param mask 必须63。
 * @param device 16字节预期设备ID。
 * @param now 宿主steady µs。
 * @param state_max_age_us 故障State年龄预算µs，默认250ms；Status固定≤1秒。
 * @return bool；仅代码指定来源0x1003/0x100a、完整motion→stop→disable组和clear-authority Fault shape才通过。
 */
inline bool raw_fault_cleanup_matches(const RawTerminalIdentity &terminal,const RawFirstPageWritten &first,const DeviceObservation &observation,
    std::uint64_t boot,std::uint64_t session,std::uint64_t run,std::uint32_t active_epoch,
    std::uint32_t profile,std::uint8_t mask,const std::array<std::uint8_t,16> &device,std::uint64_t now,
    std::uint64_t state_max_age_us=250000U) noexcept {
  namespace wire=dummy_wire;
  if(!raw_first_page_matches(first,boot,session,run,terminal.attempt_motion_epoch) || !first.chain_valid ||
     !first.full_six || !first.strict_refresh_successor || first.evidence_lost || first.attempt_kind!=1U || first.category!=1U ||
     first.evidence_seq!=0U || !first.final || first.expected_mask!=63U || first.submitted_mask!=63U || first.sent_mask!=63U || first.failed_mask!=0U)return false;
  if(!raw_terminal_matches(terminal,boot,session,run) || !session || mask!=63U || !profile ||
     terminal.attempt_kind!=1U || terminal.parent_run_id!=0U || terminal.final_state!=4U ||
     !terminal.full_fault_groups_valid || !terminal.chain_valid || terminal.evidence_lost ||
     terminal.motion_complete || !terminal.release_committed || terminal.first_failure_source==0U ||
     terminal.first_failure_axis_mask!=63U || (terminal.first_failure_source!=0x1003U && terminal.first_failure_source!=0x100aU) || terminal.attempt_motion_epoch==0U ||
     active_epoch==UINT32_MAX || active_epoch<terminal.attempt_motion_epoch ||
     active_epoch-terminal.attempt_motion_epoch>1U || terminal.fault_stop_motion_epoch!=active_epoch+1U ||
     terminal.terminal_group_count==0U || !terminal.terminal_received_us ||
     !std::any_of(terminal.terminal_chain.begin(),terminal.terminal_chain.end(),[](auto x){return x!=0U;}))return false;
  if(!observation.raw_cleanup_producer_valid || !observation.state_received_us || !observation.status_received_us ||
     observation.state_received_us<terminal.terminal_received_us || observation.status_received_us<terminal.terminal_received_us ||
     observation.state_received_us>now || observation.status_received_us>now ||
     now-observation.state_received_us>state_max_age_us || now-observation.status_received_us>1000000U)return false;
  wire::DummyJointStateV2 state{};wire::DummyDeviceStatusV2 status{};
  if(wire::DummyJointStateV2Codec::decode(observation.state.data(),observation.state.size(),&state)!=wire::CodecResult::kOk ||
     wire::DummyDeviceStatusV2Codec::decode(observation.status.data(),observation.status.size(),&status)!=wire::CodecResult::kOk)return false;
  if(state.schema_major()!=2U || state.schema_minor()!=dummy_wire_schema::kSchemaMinor || status.schema_major()!=2U || status.schema_minor()!=dummy_wire_schema::kSchemaMinor ||
     state.boot_epoch()!=boot || status.boot_epoch()!=boot || state.session_id()!=0U || status.session_id()!=0U ||
     state.reference_epoch()!=0U || status.reference_epoch()!=0U || state.core_state()!=9U || status.core_state()!=9U ||
     state.motion_epoch()!=terminal.fault_stop_motion_epoch || status.motion_epoch()!=terminal.fault_stop_motion_epoch ||
     state.management_epoch()!=status.management_epoch() || state.profile_id()!=profile || status.profile_id()!=profile ||
     state.configured_mask()!=mask || status.configured_mask()!=mask ||
     !std::equal(device.begin(),device.end(),state.device_id()) || !std::equal(device.begin(),device.end(),status.device_id()) ||
     status.run_id()!=run || status.attempt_id()!=run || status.attempt_kind()!=1U || status.parent_run_id()!=0U ||
     status.final_state()!=4U || status.evidence_lost() || status.motion_complete() || !status.release_committed() ||
     status.real_motion_permitted() || status.permit_remaining_us()!=0U ||
     status.failure_source()!=terminal.first_failure_source || status.failure_axis_mask()!=mask ||
     status.group_count()!=terminal.terminal_group_count)return false;
  const bool status_has_chain=std::any_of(std::begin(status.evidence_chain_sha256()),std::end(status.evidence_chain_sha256()),[](auto b){return b!=0U;});
  // 当前MCU session清零后的Motion fault状态不导出chain；权威始终是上方完整验证+真实flush的原terminal。
  // 非零Status chain绝不允许不匹配；零只在上述完整fault4/root/epoch/clearAuthority同shape下成立。
  if(status_has_chain && !std::equal(terminal.terminal_chain.begin(),terminal.terminal_chain.end(),status.evidence_chain_sha256()))return false;
  return std::equal(terminal.identity_hashes[0].begin(),terminal.identity_hashes[0].end(),status.contract_sha256()) &&
    std::equal(terminal.identity_hashes[1].begin(),terminal.identity_hashes[1].end(),status.model_sha256()) &&
    std::equal(terminal.identity_hashes[2].begin(),terminal.identity_hashes[2].end(),status.profile_sha256()) &&
    std::equal(terminal.identity_hashes[3].begin(),terminal.identity_hashes[3].end(),status.firmware_sha256());
}

/**
 * @brief 根据原始宿主接收锚判断State/Status年龄。
 * 快照接收发生在调用者采样now之后时按零年龄处理，用于跨线程获取顺序；不解码、不检查源生产时间或身份，不能代替transport的源时钟校验。
 * @param observation 拥有接收时刻的快照。
 * @param now 调用方采样宿主µs。
 * @param state_max_age_us State允许年龄，默认250ms。
 * @return bool；两个接收锚非零，State≤给定预算、Status≤1秒。
 */
inline bool observation_is_fresh(const DeviceObservation &observation,std::uint64_t now,
                                 std::uint64_t state_max_age_us=250000U) noexcept {
  if(!observation.state_received_us || !observation.status_received_us) return false;
  // Receipt stamps are local. A receive after the caller sampled `now` has zero age.
  const auto state_age=now>observation.state_received_us ? now-observation.state_received_us : 0;
  const auto status_age=now>observation.status_received_us ? now-observation.status_received_us : 0;
  return state_age<=state_max_age_us && status_age<=1000000U;
}
/**
 * @brief 把短暂观测空档与持续丢失分开，不给旧消息续龄。
 */
class ObservationContinuity {
 public:
  /**
   * @brief Ready为当前状态满足，Wait为允许空档内等待，Lost为中断持续严格超过对应预算。
   */
  enum class Result { kReady, kWait, kLost };
  /**
   * @brief 固定State年龄及启动/运行连续性等待预算。
   * @param state_max_age_us State接收年龄上限µs。
   * @param bootstrap_limit_us 未enabled时允许中断的宿主µs。
   * @param running_limit_us enabled时允许中断的µs。
   */
  ObservationContinuity(std::uint64_t state_max_age_us,
                        std::uint64_t bootstrap_limit_us,
                        std::uint64_t running_limit_us) noexcept
      : state_max_age_us_(state_max_age_us), bootstrap_limit_us_(bootstrap_limit_us),
        running_limit_us_(running_limit_us) {}
  /**
   * @brief 结合State活性、调用方一致性和连续空档计时判定。
   * 启动时还要Status≤1秒；运行可用新鲜State加coherent，不能据此假定Status永远可忽略。计时只看宿主单调域，不外推设备状态。
   * @param observation 最新接收锚快照。
   * @param now 单调宿主µs。
   * @param enabled 运行/已使能阶段；True时本谓词不再要求Status当前年龄。
   * @param coherent 外层已经核对身份/生命周期组合一致。
   * @return Result；Ready清空中断起点，其他从首次未就绪时计总空档。
   */
  Result evaluate(const DeviceObservation &observation, std::uint64_t now,
                  bool enabled, bool coherent) noexcept {
    const bool state_live=observation.state_received_us!=0U &&
      (now<=observation.state_received_us || now-observation.state_received_us<=state_max_age_us_);
    const bool status_live=observation.status_received_us!=0U &&
      (now<=observation.status_received_us || now-observation.status_received_us<=1000000U);
    if(state_live && (enabled || status_live) && coherent) {
      interruption_started_us_=0U;
      return Result::kReady;
    }
    if(interruption_started_us_==0U) interruption_started_us_=now;
    const auto limit=enabled ? running_limit_us_ : bootstrap_limit_us_;
    return now-interruption_started_us_>limit ? Result::kLost : Result::kWait;
  }
 private:
  /**
   * @brief 所有预算和interruption_started为宿主µs；0起点表示尚无连续空档，不保存设备时钟。
   */
  std::uint64_t state_max_age_us_;
  std::uint64_t bootstrap_limit_us_, running_limit_us_;
  std::uint64_t interruption_started_us_=0U;
};
/**
 * @brief 最小boot/session/reference/motion四元组，无profile/hash或新鲜度字段，必须与外部验证组合使用。
 */
struct StateAuthority {
  std::uint64_t boot_epoch=0U, session_id=0U;
  std::uint32_t reference_epoch=0U, motion_epoch=0U;
};
/**
 * @brief 释放前original与当前Status/State身份及各自management/core值，release_stage来自调用方状态机；本结构只保存形状，不自行认证。
 */
struct ReleaseTransitionObservation {
  StateAuthority original{}, status{}, state{};
  std::uint32_t status_management_epoch=0U, state_management_epoch=0U,
                original_management_epoch=0U;
  std::uint16_t status_core_state=0U, state_core_state=0U;
  int release_stage=0;
};
/**
 * @brief 保存本次释放原始根身份以解释明确过渡，不重新绑定新owner。
 */
class ReleaseTransitionOrigin {
 public:
  /**
   * @brief 复制释放前身份与管理水位。
   * @param authority 原boot/session/reference/motion。
   * @param management_epoch 原管理epoch。
   */
  void capture(StateAuthority authority, std::uint32_t management_epoch) noexcept {
    authority_=authority;
    management_epoch_=management_epoch;
  }
  /**
   * @brief 以保存根和当前两个来源组装过渡观察值。
   * @param status Status身份。
   * @param state State身份。
   * @param status_management_epoch Status管理水位。
   * @param state_management_epoch State管理水位。
   * @param status_core_state Status生命周期编号。
   * @param state_core_state State生命周期编号。
   * @param release_stage 外层释放阶段。
   * @return ReleaseTransitionObservation值，仅组装不验证。
   */
  ReleaseTransitionObservation observation(StateAuthority status, StateAuthority state,
      std::uint32_t status_management_epoch, std::uint32_t state_management_epoch,
      std::uint16_t status_core_state, std::uint16_t state_core_state,
      int release_stage) const noexcept {
    return {authority_,status,state,status_management_epoch,state_management_epoch,
            management_epoch_,status_core_state,state_core_state,release_stage};
  }
 private:
  /**
   * @brief 原始释放四元组及管理epoch；应在明确开始释放时capture，不随状态到达漂移。
   */
  StateAuthority authority_{};
  std::uint32_t management_epoch_=0U;
};
/**
 * @brief 核双消息同boot/epoch且无owner/reference/permit的已释放Idle形状。
 * @tparam Status 状态类型。
 * @tparam State 实际State类型。
 * 此模板不检查消息新鲜度、device/profile、operation/root或原页链；名字中的confirmed仅指本谓词列出的合取，外层仍需完整释放事务。
 * @param status 提供wire Status访问器的对象。
 * @param state 提供wire State访问器的对象。
 * @return bool；release_committed来自Status。
 */
template <typename Status, typename State>
bool released_idle_status_confirmed(const Status &status, const State &state) noexcept {
  return status.boot_epoch()!=0U && status.boot_epoch()==state.boot_epoch() &&
    status.motion_epoch()==state.motion_epoch() &&
    status.management_epoch()==state.management_epoch() &&
    status.core_state()==1U && state.core_state()==1U &&
    status.session_id()==0U && state.session_id()==0U &&
    status.reference_epoch()==0U && state.reference_epoch()==0U &&
    status.release_committed() && !status.real_motion_permitted() &&
    status.permit_remaining_us()==0U;
}
/**
 * @brief manual释放保留同boot绝对参考的双Idle形状。
 * 非零expected_reference须由释放前已确认参考固定；新鲜度、部署身份、D10根/COMMITTED
 * 与全轴CAN SENT仍由外层独立核对，不能单用本谓词宣称失能。
 */
template <typename Status, typename State>
bool retained_reference_released_idle_observation(
    const Status &status, const State &state,
    std::uint32_t expected_reference) noexcept {
  return expected_reference != 0U && status.boot_epoch() != 0U &&
         status.boot_epoch() == state.boot_epoch() &&
         status.motion_epoch() == state.motion_epoch() &&
         status.management_epoch() == state.management_epoch() &&
         status.core_state() == 2U && state.core_state() == 2U &&
         status.session_id() == 0U && state.session_id() == 0U &&
         status.reference_epoch() == expected_reference &&
         state.reference_epoch() == expected_reference &&
         status.release_committed() && !status.real_motion_permitted() &&
         status.permit_remaining_us() == 0U;
}
/**
 * @brief 只容许明确释放stage2时的Status Stopping/State Idle乱序过渡。
 * 不放宽任意身份错配；原epoch为UINT32_MAX拒绝，避免溢出。仅允许继续等待同事务，不宣称COMMITTED或可再次运动。
 * @param value 包含原session/reference及双当前身份的值。
 * @param observation 原接收时间快照。
 * @param now 宿主µs。
 * @param state_max_age_us State接收年龄预算µs。
 * @param status_max_age_us Status独立接收年龄预算µs。
 * @param expected_retained_reference 仅显式manual非零参考启用Disabling→Idle保留参考等待。
 * @return bool；同boot/绑定owner，双epoch恰原值+1，State owner归零，双接收年龄满足。
 */
inline bool release_idle_transition_pending(const ReleaseTransitionObservation &value,
                                            const DeviceObservation &observation,
                                            std::uint64_t now,
                                            std::uint64_t state_max_age_us,
                                            std::uint64_t status_max_age_us = 1000000U,
                                            std::uint32_t expected_retained_reference = 0U) noexcept {
  const bool legacy_idle = expected_retained_reference == 0U &&
    value.status_core_state == 7U && value.state_core_state == 1U &&
    value.state.reference_epoch == 0U;
  const bool retained_idle = expected_retained_reference != 0U &&
    value.original.reference_epoch == expected_retained_reference &&
    value.status_core_state == 8U && value.state_core_state == 2U &&
    value.state.reference_epoch == expected_retained_reference;
  const auto fresh = [now](std::uint64_t received, std::uint64_t max_age) {
    return received != 0U && max_age != 0U &&
      (now <= received || now - received <= max_age);
  };
  // 仅等待已绑定D10的有限跨topic过渡，不确认释放或授予控制。
  return value.release_stage==2 && value.original.boot_epoch!=0U &&
    value.original.session_id!=0U && value.original.reference_epoch!=0U &&
    value.status.boot_epoch==value.original.boot_epoch &&
    value.status.session_id==value.original.session_id &&
    value.status.reference_epoch==value.original.reference_epoch &&
    (legacy_idle || retained_idle) &&
    value.state.boot_epoch==value.original.boot_epoch && value.state.session_id==0U &&
    value.original.motion_epoch!=UINT32_MAX &&
    value.status.motion_epoch==value.original.motion_epoch+1U &&
    value.state.motion_epoch==value.status.motion_epoch &&
    value.original_management_epoch!=UINT32_MAX &&
    value.status_management_epoch==value.original_management_epoch+1U &&
    value.status_management_epoch==value.state_management_epoch &&
    fresh(observation.state_received_us, state_max_age_us) &&
    fresh(observation.status_received_us, status_max_age_us);
}

/**
 * @brief 核故障标志与最小State授权四元组的允许前进量。
 * 不检查profile/ID/hash、源时钟或+1是否合法管理因果；这些必须由调用方补足，不能只靠epoch数值授权。
 * @param state 实际四元组。
 * @param expected 期望四元组。
 * @param fault 外部判定Fault时立即false。
 * @param motion_stable True只允许motion相等，False还允许恰+1。
 * @return bool；boot/session/reference必须完全相同，motion倒退或超过允许步长拒绝。
 */
inline bool state_authority_is_valid(const StateAuthority &state,
                                     const StateAuthority &expected,
                                     bool fault, bool motion_stable) noexcept {
  if(fault || state.boot_epoch!=expected.boot_epoch || state.session_id!=expected.session_id ||
     state.reference_epoch!=expected.reference_epoch || state.motion_epoch<expected.motion_epoch)
    return false;
  const auto motion_advance=state.motion_epoch-expected.motion_epoch;
  return motion_advance==0U || (!motion_stable && motion_advance==1U);
}
/**
 * @brief 管理client使用的唯一实现对象，支持独立连接或复用设备worker。
 * public操作可能分配响应vector和读取专用mutex，不声称所有接口都硬实时。发送/普通observe使用try_lock避免等共享主槽；首/终页回执读取则会取得各自专用锁。
 */
class DpManagementTransport final : public ManagementTransport {
 public:
  /**
   * @brief StandaloneWorker独占自建dp_comm Context/线程；DeviceWorker通过同身份共享channel交给现有DeviceTransport，避免另开第二个实际通信owner。
   */
  enum class Connection { kStandaloneWorker, kDeviceWorker };
  /**
   * @brief 保存明确连接参数和模式，不立即创建网络资源。
   * 匿名DpManagementParameters为值，匿名Connection默认StandaloneWorker。硬件集成需显式选择DeviceWorker；构造不保证endpoint在线或电机已授权。
   */
  explicit DpManagementTransport(DpManagementParameters, Connection=Connection::kStandaloneWorker);
  /**
   * @brief 调用stop关闭自身准入owner并join自有线程。
   * 共享模式不会停止dummy_ros_control拥有的通信线程或自动失能设备。
   */
  ~DpManagementTransport() override;
  /**
   * @brief 校验非空端点/ID、profile与掩码后启动所选模式。
   * 端点≤256且无空白，device/client非零，mask为1/63；DeviceWorker不新建Context，实际channel可稍后出现。
   * @return bool；共享模式仅建立可用准入owner，独立模式仅表示线程创建，不保证ready/设备授权。
   */
  bool start();
  /**
   * @brief 撤销尚未准入共享句柄的owner并结束自有worker。
   * 不发送D10、不停止共享DeviceTransport；DeviceWorker的ready仍按共享channel报告，不能由此推断设备停止或所有既发请求撤回。
   */
  void stop();
  /**
   * @brief 读取自有连接或共享worker管理端点可用性。
   * @return 独立connected或共享channel.ready；不检查session/参考/许可。
   */
  bool ready() const noexcept;
  /** @brief
   * 非阻塞取独立worker原页FIFO首项；共享模式、未连接、空或锁忙返回nullopt。
   * 固定设备路由仅限定来源；该wire没有boot/session/device字段，调用方必须绑定原Status/原链验收。
   * 队列容量16，溢出后拒收后续页并锁存失败；不会接入normal/rawcollector或授予权限。
   */
  std::optional<RawAttemptEvidence> take_attempt_evidence();
  /** @brief 独立原页队列是否曾溢出；sticky诊断，不能以取空队列清除此失败。 */
  bool attempt_evidence_overflowed() const noexcept;
  /**
   * @brief 从当前注册channel格式化固定宿主诊断。
   * 非控制读取接口：会分配/格式化；注册表只try一次，不通过control侧cached_channel更新锁。不是设备wire或实际传输证据。
   * @return JSON字符串或找不到channel时文本null。
   */
  std::string diagnostic_json() const;
  /**
   * @brief 读取共享collector粘滞首错编号。
   * @return 仅DeviceWorker可观察；独立模式/无channel也返回0，0不能单独证明采集已启用或完整。
   */
  std::uint8_t raw_evidence_failure() const noexcept;
  /**
   * @brief 查询共享worker是否配置原页捕获。
   * @return bool；开关不证明文件成功写出，仍须failure及页回执。
   */
  bool raw_evidence_capture_enabled() const noexcept;
  /**
   * @brief 查询旧motion完成证据后的下一Begin屏障。
   * @return 共享模式按channel标志；独立模式兼容返回true，不能当完整设备Ready证明。
   */
  bool post_complete_begin_ready() const noexcept;
  /**
   * @brief try_lock读取共享worker的命令排空现场。
   * @return optional值，非共享/未ready/主锁忙时为空；取得后仍要核身份、年龄和quiescent。
   */
  std::optional<HoldDrainEvidence> hold_drain_snapshot() const noexcept;
  /**
   * @brief 复制共享collector已写/flush终页回执。
   * 终页flush只是本机writer层，不意味着fsync、物理运动成功或全部停止验收完成。
   * @return 值或默认空值；不要求channel仍ready，历史回执须按boot/session/run再匹配。
   */
  RawTerminalIdentity raw_terminal_flushed() const noexcept;
  /**
   * @brief 读取writer写出后的故障清理原始观测。
   * raw_cleanup_producer_valid由非零可信State接收锚导出；此lane不恢复正常admission/控制资格。
   * @return DeviceObservation值；只在共享模式可用，默认空不可当有效状态。
   */
  DeviceObservation raw_cleanup_observation() const noexcept;
  // 仅供panel诊断发布，不得作为控制、释放或writer证据。
  DeviceObservation diagnostic_observation() const noexcept;
  /**
   * @brief 读取共享collector首非终原页的writer回执。
   * @return 值或默认空；需要按完整run/motion及proof标志组合核对。
   */
  RawFirstPageWritten raw_first_page_written() const noexcept;
  /**
   * @brief 尝试复制完整wire管理请求到有限单槽。
   * 匿名vector为原codec请求。主槽有未取消请求时拒绝，不排无界队列；锁忙admission最多100ms、2ms退避/50次，owner/ready失效或槽被占则失败。返回句柄不是设备Received，销毁只取消本地等待。
   * @return 唯一句柄或空；共享主锁忙时可返回尚在有限admission的SharedHandle。
   */
  std::unique_ptr<RequestHandle> send(const std::vector<std::uint8_t> &) override;
  /**
   * @brief 共享模式从同worker序号空间预留，独立模式保持建议值。
   * @param proposed 建议序号。
   * @return 实际request_seq；无共享channel时也回退proposed，调用方负责生命周期/准入。
   */
  std::uint32_t reserve_sequence(std::uint32_t proposed) noexcept override;
  /**
   * @brief try_lock复制最新正常观测，竞争/离线时可返上次缓存。
   * 缓存返回不更新state/status_received_us；独立worker此观测只核schema/device/profile/mask，没有硬件worker完整源时钟/能力/授权过滤。
   * @return optional拥有字节快照；非空可能是旧值，必须按原接收锚判年龄。
   */
  std::optional<DeviceObservation> observe();
  /**
   * @brief 为共享worker设置明确的inactive保持prime转交标志。
   * @param allowed True允许准备流程转交，False关闭。
   * @return 只共享模式且有channel时true；不是设备Enable或任意inactive目标许可。
   */
  bool allow_inactive_prime(bool allowed) noexcept;
  /**
   * @brief 发布Complete后同值Hold判断的逐轴容差。
   * @param values 按配置轴数1或6的有限正rad值。
   * @return bool；先全部校验再写原子容差，无channel/模式/长度不符拒绝。
   */
  bool set_post_complete_hold_tolerance(const std::vector<double> &values) noexcept;
 private:
  struct Impl;
  /**
   * @brief 唯一Impl拥有自有worker或共享channel缓存及准入owner，外部client只借用本transport接口。
   */
  std::unique_ptr<Impl> impl_;
};
/**
 * @brief 读取管理端宿主steady微秒时钟。
 * @return steady_clock epoch以来µs；与transport宿主时基同类，不能直接减MCU produced时间或ROS仿真时间。
 */
std::uint64_t management_now_us() noexcept;
}  // namespace dummy_motion_client

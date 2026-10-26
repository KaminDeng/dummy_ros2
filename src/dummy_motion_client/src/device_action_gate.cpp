/**
 * @file device_action_gate.cpp
 * @brief 公开FJT动作、显式bootstrap管理服务与唯一设备worker/JTC的集成入口。
 * ROS executor回调负责有限准入和排队，独立控制线程按prepare_update→manager read/update/write→cycle处理真实采样及管理状态机；传输复用dummy_ros_control的DeviceWorker，不另开USB owner。构造节点不等于标定、使能或授权动作。
 * 位置rad、速度rad/s、加速度rad/s²、轨迹相对秒；管理/接收/证据steady时间为宿主µs，ROS反馈header另用ROS时钟。Queued、Accepted、COMMITTED、实际Hold、Timeline排空与raw终页flush均为不同证据，不能互相冒充物理完成或正式资格。
 */
#include "controller_manager/controller_manager.hpp"
#include "dummy_motion_client/bootstrap_coordinator.hpp"
#include "dummy_motion_client/bootstrap_tick_policy.hpp"
#include "dummy_motion_client/device_profile.hpp"
#include "dummy_motion_client/dp_management_transport.hpp"
#include "dummy_motion_client/goal_timeline.hpp"
#include "dummy_motion_client/limited_goal_budget.hpp"
#include "dummy_motion_client/manual_panel_profile.hpp"
#include "dummy_motion_client/native_goal_validation.hpp"
#include "dummy_motion_client/panel_feedback.hpp"
#include "dummy_ros_control/command_channel.hpp"
#include "dummy_ros_control/diagnostic_journal.hpp"
#include "dummy_ros_control/diagnostic_json.hpp"
#include "dummy_ros_control/diagnostic_scope.hpp"
#include "dummy_ros_control/guarded_joint_trajectory_controller.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <algorithm>
#include <array>
#include <atomic>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <initializer_list>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

namespace dummy_motion_client {
namespace wire=dp::iface;
using Follow=control_msgs::action::FollowJointTrajectory;
using GoalHandle=rclcpp_action::ServerGoalHandle<Follow>;
using Controller=dummy_ros_control::GuardedJointTrajectoryController;

/**
 * @brief 核writer首原页属于指定同值Hold refresh后继身份且具严格六轴证明。
 * 只识别后继原页，不单独确认Status/State新鲜、原motion已完成或动作成功。
 * @param page 已写/刷新首非终原页。
 * @param identity 预期boot/session/reference/motion/run组合；首页helper使用其中boot/session/motion/run。
 * @return bool；seq0、根attempt/category1、全六轴expected/submitted/sent且无failed/evidence_lost等须同时满足。
 */
bool verified_refresh_successor(const RawFirstPageWritten &page,
                                const TimelineIdentity &identity) noexcept {
  return page.strict_refresh_successor && page.chain_valid && page.full_six &&
    page.evidence_seq==0U && page.group_count>=1U && page.attempt_kind==1U &&
    page.category==1U && page.expected_mask==63U && page.submitted_mask==63U &&
    page.sent_mask==63U && page.failed_mask==0U && page.final &&
    !page.evidence_lost && raw_first_page_matches(
      page,identity.boot,identity.session,identity.run_id,identity.motion_epoch);
}

/**
 * @brief 识别同根run已验链的无故障Cancel终页及后继安全Hold epoch。
 * 这是干净受控取消，不是成功运动或D10失能释放；时间/实际Hold仍由调用层检查。
 * @param terminal writer已flush的终页值。
 * @param run 被取消motion原身份。
 * @return bool；final7、非motion_complete/非release、零failure且safety_hold=原motion+1。
 */
bool clean_cancel_terminal(const RawTerminalIdentity &terminal,
                           const TimelineIdentity &run) noexcept {
  return run.motion_epoch!=UINT32_MAX &&
    raw_terminal_matches(terminal,run.boot,run.session,run.run_id) &&
    terminal.chain_valid && terminal.attempt_kind==1U &&
    terminal.parent_run_id==0U && terminal.final_state==7U &&
    terminal.first_failure_source==0U &&
    terminal.first_failure_axis_mask==0U && !terminal.evidence_lost &&
    !terminal.motion_complete && !terminal.release_committed &&
    terminal.safety_hold_valid &&
    terminal.safety_hold_motion_epoch==run.motion_epoch+1U;
}

/**
 * @brief 在原motion完成终页与紧邻refresh首组都写出后确认后继双Hold形状。
 * 本谓词不独立检查接收年龄，外层read_observation叠加双fresh/coherent；它解释已完成motion后的必要保持refresh，不给新目标授权。
 * @param status 当前已验证Status。
 * @param state 当前已验证State。
 * @param prior 前motion身份。
 * @param terminal 前run完成原终页flush回执。
 * @param successor run+1首原页写出回执。
 * @param raw_capture_enabled 必须启用raw捕获。
 * @param raw_failure 必须为0。
 * @param axis_mask 必须63。
 * @return bool；同boot/session/reference、run及motion恰+1，双Hold、全轴SENT和严格首页成立。
 */
bool timeline_completed_refresh_hold_confirmed(
    const wire::DummyDeviceStatusV2 &status,const wire::DummyJointStateV2 &state,
    const TimelineIdentity &prior,const RawTerminalIdentity &terminal,
    const RawFirstPageWritten &successor,bool raw_capture_enabled,
    std::uint8_t raw_failure,std::uint8_t axis_mask) noexcept {
  if(!raw_capture_enabled || raw_failure!=0U || axis_mask!=63U ||
     prior.boot==0U || prior.session==0U || prior.reference_epoch==0U ||
     prior.run_id==0U || prior.run_id==UINT64_MAX ||
     prior.motion_epoch==0U || prior.motion_epoch==UINT32_MAX ||
     !raw_terminal_matches(terminal,prior.boot,prior.session,prior.run_id) ||
     !terminal.chain_valid || terminal.attempt_kind!=1U ||
     terminal.parent_run_id!=0U || terminal.final_state!=6U ||
     !terminal.motion_complete || terminal.evidence_lost ||
     terminal.release_committed || terminal.first_failure_source!=0U ||
     terminal.first_failure_axis_mask!=0U) return false;
  const TimelineIdentity next{status.boot_epoch(),status.session_id(),
      status.reference_epoch(),status.motion_epoch(),status.run_id()};
  return next.boot==prior.boot && next.session==prior.session &&
    next.reference_epoch==prior.reference_epoch &&
    next.run_id==prior.run_id+1U && next.motion_epoch==prior.motion_epoch+1U &&
    status.attempt_kind()==1U && status.parent_run_id()==0U &&
    status.final_state()==0U && !status.motion_complete() &&
    !status.evidence_lost() &&
    status.core_state()==wire_value(WireCoreState::kHold) &&
    state.boot_epoch()==next.boot && state.session_id()==next.session &&
    state.reference_epoch()==next.reference_epoch &&
    state.motion_epoch()==next.motion_epoch &&
    state.management_epoch()==status.management_epoch() &&
    state.core_state()==wire_value(WireCoreState::kHold) && state.hold_ready() &&
    state.can_submitted_mask()==axis_mask && state.can_sent_mask()==axis_mask &&
    state.can_failed_mask()==0U && verified_refresh_successor(successor,next);
}

/**
 * @brief 核取消同run终页与当前后继Hold状态形状。
 * 本辅助未单独检查接收年龄/两management相等或raw失败，调用方再合取；不能凭名称认为已覆盖全部取消验收。
 * @param status 当前Status。
 * @param state 当前State。
 * @param run 被取消motion原身份。
 * @param raw_terminal 干净Cancel终页flush回执。
 * @param axis_mask 只支持完整63。
 * @return bool；同run/参考，motion恰+1，final7、双Hold和全配置轴提交/SENT。
 */
bool timeline_cancel_hold_confirmed(const wire::DummyDeviceStatusV2 &status,
                                    const wire::DummyJointStateV2 &state,
                                    const TimelineIdentity &run,
                                    const RawTerminalIdentity &raw_terminal,
                                    std::uint8_t axis_mask) noexcept {
  return axis_mask==63U && run.motion_epoch!=UINT32_MAX &&
    status.boot_epoch()==run.boot && status.session_id()==run.session &&
    status.reference_epoch()==run.reference_epoch && status.run_id()==run.run_id &&
    status.motion_epoch()==run.motion_epoch+1U &&
    state.boot_epoch()==run.boot && state.session_id()==run.session &&
    state.reference_epoch()==run.reference_epoch &&
    state.motion_epoch()==status.motion_epoch() &&
    status.attempt_kind()==1U && status.parent_run_id()==0U &&
    status.final_state()==7U && !status.motion_complete() && !status.evidence_lost() &&
    status.core_state()==wire_value(WireCoreState::kHold) &&
    state.core_state()==wire_value(WireCoreState::kHold) && state.hold_ready() &&
    state.can_submitted_mask()==axis_mask && state.can_sent_mask()==axis_mask &&
    clean_cancel_terminal(raw_terminal,run);
}

/**
 * @brief 仅在gate已有终态时转交Timeline/raw证据封口判定。
 * @param terminal ActionGate外部终态。
 * @param timeline_configured 是否配置时间线。
 * @param timeline_failed 时间线/证据是否失败。
 * @param hold_recorded 实际Hold事件已登记。
 * @param writer_drained 时间线writer已排空。
 * @param raw_failed 原页采集失败。
 * @param raw_terminal_flushed 相同goal run终页已写出/flush。
 * @return 无终态始终Wait；其余由timeline_success_decision返回封口层级。
 */
TimelineSuccessDecision timeline_terminal_decision(
    Terminal terminal,bool timeline_configured,bool timeline_failed,
    bool hold_recorded,bool writer_drained,bool raw_failed,
    bool raw_terminal_flushed) noexcept {
  if(terminal==Terminal::kNone) return TimelineSuccessDecision::kWait;
  return timeline_success_decision(timeline_configured,timeline_failed,
    hold_recorded,writer_drained,true,raw_failed,raw_terminal_flushed);
}

/**
 * @brief 保留首个动作错误并附加证据不完整。
 * 不把证据失败改写成动作完成，也不丢失前面的设备/路径首因。
 * @param first_error 已有错误原因。
 * @return 空或已为EVIDENCE_INCOMPLETE时返回该标记，否则以分号追加。
 */
std::string evidence_incomplete_result_error(const std::string &first_error) {
  if(first_error.empty() || first_error=="EVIDENCE_INCOMPLETE")
    return "EVIDENCE_INCOMPLETE";
  return first_error+";EVIDENCE_INCOMPLETE";
}

/**
 * @brief 把已证同值refresh后继映射回原goal的Hold身份用于封口。
 * 这里motion比较的是已确认后继Hold身份，不是旧Running epoch。仅关闭既有goal时间线，不把refresh当另一个用户目标。
 * @param observed 当前refresh身份。
 * @param confirmed_hold 原goal已经确认Hold的身份。
 * @param prior_hold_confirmed 此前Hold已证。
 * @param raw_terminal_flushed 原motion终页匹配且已flush。
 * @param successor_page_written 后继首页已验/写。
 * @param status_hold Status新鲜Hold。
 * @param state_hold State新鲜一致Hold。
 * @param refresh_status 当前是无故障未终结根refresh。
 * @return 原confirmed_hold或nullopt；run需恰+1，其余boot/session/reference/motion不变。
 */
std::optional<TimelineIdentity> timeline_refresh_closure_identity(
    const TimelineIdentity &observed, const TimelineIdentity &confirmed_hold,
    bool prior_hold_confirmed, bool raw_terminal_flushed,
    bool successor_page_written, bool status_hold, bool state_hold,
    bool refresh_status) noexcept {
  if(!prior_hold_confirmed || !raw_terminal_flushed ||
     !successor_page_written || !status_hold ||
     !state_hold || !refresh_status || confirmed_hold.run_id==UINT64_MAX ||
     observed.run_id!=confirmed_hold.run_id+1U ||
     observed.boot!=confirmed_hold.boot || observed.session!=confirmed_hold.session ||
     observed.reference_epoch!=confirmed_hold.reference_epoch ||
     observed.motion_epoch!=confirmed_hold.motion_epoch) return std::nullopt;
  return confirmed_hold;
}

/**
 * @brief 给refresh封口合取返回首个不满足项标签。
 * @param observed 当前身份。
 * @param confirmed_hold 原已证Hold身份。
 * @param prior_hold_confirmed 前Hold证明。
 * @param raw_terminal_matched 原终页身份匹配。
 * @param successor_page_verified 严格后继首页通过。
 * @param status_hold 当前Status条件。
 * @param state_hold 当前State条件。
 * @param refresh_status 根refresh形状。
 * @return 静态C标签或none，只供诊断。
 */
const char *refresh_closure_failure(const TimelineIdentity &observed,
    const TimelineIdentity &confirmed_hold,bool prior_hold_confirmed,
    bool raw_terminal_matched,bool successor_page_verified,
    bool status_hold,bool state_hold,bool refresh_status) noexcept {
  if(!prior_hold_confirmed) return "prior_hold";
  if(!raw_terminal_matched) return "raw_terminal";
  if(!successor_page_verified) return "successor_page";
  if(!status_hold) return "status_hold";
  if(!state_hold) return "state_hold";
  if(!refresh_status) return "refresh_status";
  if(confirmed_hold.run_id==UINT64_MAX || observed.run_id!=confirmed_hold.run_id+1U)
    return "run_id";
  if(observed.boot!=confirmed_hold.boot) return "boot";
  if(observed.session!=confirmed_hold.session) return "session";
  if(observed.reference_epoch!=confirmed_hold.reference_epoch) return "reference_epoch";
  if(observed.motion_epoch!=confirmed_hold.motion_epoch) return "motion_epoch";
  return "none";
}

/**
 * @brief 诊断已知refresh等待State的身份或宿主期限问题。
 * 此诊断把raw/首页等设为中性true，只定位允许短等的条件，不替代实际最终封口证据。
 * @param observed 当前候选身份。
 * @param confirmed_hold 已证原Hold身份。
 * @param prior_hold_confirmed 前Hold是否确立。
 * @param refresh_status 明确refresh状态形状。
 * @param first_seen_us 首次refresh Status原接收µs。
 * @param now 宿主µs。
 * @param age_budget_us 有限等待µs。
 * @return 静态首失败标签；none表示身份和等待窗满足。
 */
const char *refresh_wait_failure(const TimelineIdentity &observed,
    const TimelineIdentity &confirmed_hold,bool prior_hold_confirmed,bool refresh_status,
    std::uint64_t first_seen_us,std::uint64_t now,std::uint64_t age_budget_us) noexcept {
  const char *identity_failure=refresh_closure_failure(observed,confirmed_hold,
    prior_hold_confirmed,true,true,true,true,refresh_status);
  if(std::string_view(identity_failure)!="none") return identity_failure;
  if(first_seen_us==0U) return "first_seen_missing";
  if(now<first_seen_us) return "clock_regressed";
  if(now-first_seen_us>=age_budget_us) return "age_budget";
  return "none";
}

/**
 * @brief 仅给已知紧邻同值refresh一个等待独立State的有限窗口。
 * @param observed refresh候选身份。
 * @param confirmed_hold 已证Hold身份。
 * @param prior_hold_confirmed 必须已确立。
 * @param refresh_status 必须明确为refresh。
 * @param first_seen_us 原Status接收锚µs，非0。
 * @param now 宿主µs。
 * @param age_budget_us 等待预算µs。
 * @return bool：同身份/run+1且now位于[first_seen,first_seen+budget)；不延长原锚。
 */
bool timeline_refresh_wait_allowed(const TimelineIdentity &observed,
    const TimelineIdentity &confirmed_hold,bool prior_hold_confirmed,bool refresh_status,
    std::uint64_t first_seen_us,std::uint64_t now,std::uint64_t age_budget_us) noexcept {
  // Only a known refresh successor may wait for its independently delivered State.
  return timeline_refresh_closure_identity(observed,confirmed_hold,prior_hold_confirmed,
           true,true,true,true,refresh_status).has_value() && first_seen_us!=0U &&
    now>=first_seen_us && now-first_seen_us<age_budget_us;
}

/**
 * @brief 用CAS占一个显式管理请求槽。
 * 回调只排队，不发送管理或报告设备已执行；控制线程稍后按实际状态复查。
 * @param request 0为空的原子操作编号槽。
 * @param busy goal/管理占用标志。
 * @param renewal_reserved 续租预留标志。
 * @param operation 待排队wire操作整数。
 * @param available 调用方读得的准入快照。
 * @return bool；检查快照后CAS0→operation成功。
 */
bool queue_management_request(std::atomic<int> &request, const std::atomic<bool> &busy,
                              const std::atomic<bool> &renewal_reserved,
                              int operation, bool available) noexcept {
  int expected=0;
  return available && !busy.load() && !renewal_reserved.load() &&
    request.compare_exchange_strong(expected,operation);
}

/**
 * @brief 把一次SetPermit成功传播给ActionGate的本地代次锁存。
 * generation/applied_generation为宿主调用代，不是设备epoch；boot/session限定只应用当前请求，不拥有许可本体。
 */
struct PermitRefreshLatch {
  /**
   * @brief 记录新一轮SetPermit预期boot/session并递增请求代。
   * @param boot 请求时boot。
   * @param session 请求时session。
   */
  void begin(std::uint64_t boot, std::uint64_t session) noexcept {
    ++generation;
    request_boot=boot;
    request_session=session;
  }
  /**
   * @brief 判断当前身份是否仍可应用尚未传播的permit结果。
   * @param boot 当前boot。
   * @param session 当前session。
   * @return 本代尚未applied且身份相同为true。
   */
  bool eligible(std::uint64_t boot, std::uint64_t session) const noexcept {
    return generation!=applied_generation && boot==request_boot &&
      session==request_session;
  }
  /**
   * @brief 标记本地已把当前代permit快照成功交给gate，不发送设备请求。
   */
  void applied() noexcept { applied_generation=generation; }

  std::uint64_t generation=0, applied_generation=0;
  std::uint64_t request_boot=0, request_session=0;
};

/**
 * @brief 撤销未成功续租的本地预约，保留不属于它的goal busy。
 * 清本地状态不证明设备租期未变化，也不重发原管理；上层Bootstrap对未确认结果Fault。
 * @param request 管理操作单槽，仅尝试清RenewLease值。
 * @param busy 宿主占用标志。
 * @param reserved 预约标志。
 * @param started 是否已提交续租。
 * @param explicit_status 显式服务回执状态。
 * @param owns_busy True仅当此续租独占busy，RUN借用goal busy则false。
 */
void clear_failed_lease_renewal(std::atomic<int> &request, std::atomic<bool> &busy,
                                std::atomic<bool> &reserved, std::atomic<bool> &started,
                                std::atomic<int> &explicit_status,
                                bool owns_busy=true) noexcept {
  int renewal=static_cast<int>(wire_value(ManageOperation::kRenewLease));
  request.compare_exchange_strong(renewal,0);
  explicit_status=0;
  started=false;
  reserved=false;
  if(owns_busy) busy=false;
}

/**
 * @brief Running续租最低合取输入：双Running/同tuple/同goal run/真实且最近producer/transport及许可新鲜性，各由外层实际观察构造。
 */
struct RunLeaseEligibility {
  bool status_running=false, state_running=false, status_state_coherent=false;
  bool same_goal_run=false, producer_active=false, producer_recent=false;
  bool transport_ready=false;
};

/**
 * @brief 仅当全部Running生产链条件满足时允许新的RUN续租。
 * @param state 七项资格值。
 * @return 全部为true；不在此读取硬件或推导单项真实性。
 */
bool run_lease_eligible(const RunLeaseEligibility &state) noexcept {
  return state.status_running && state.state_running &&
    state.status_state_coherent && state.same_goal_run &&
    state.producer_active && state.producer_recent && state.transport_ready;
}

/**
 * @brief 只为已接纳续租遇到有界完成/取消冻结时保留producer_active。
 * 其他身份/最近采样/双fresh仍不变；该例外不能用于开启一个新的续租。
 * @param inputs 原始七项条件。
 * @param renewal_running 确实已有本RUN续租预约/处理。
 * @param bounded_completion gate已证明在原deadline内等待同根续租的冻结意图。
 * @return 复制后的条件，仅可能把producer_active改true。
 */
RunLeaseEligibility run_lease_continuation_inputs(RunLeaseEligibility inputs,
    bool renewal_running,bool bounded_completion) noexcept {
  if(renewal_running && bounded_completion) inputs.producer_active=true;
  return inputs;
}

/**
 * @brief 返回七项RUN资格的首失败标签用于固定诊断。
 * @param state 资格输入。
 * @return 静态标签或none，不更改状态。
 */
const char *run_lease_first_failed(const RunLeaseEligibility &state) noexcept {
  if(!state.status_running) return "status_running";
  if(!state.state_running) return "state_running";
  if(!state.status_state_coherent) return "status_state_coherent";
  if(!state.same_goal_run) return "same_goal_run";
  if(!state.producer_active) return "producer_active";
  if(!state.producer_recent) return "producer_recent";
  if(!state.transport_ready) return "transport_ready";
  return "none";
}

/**
 * @brief 安全计算宿主年龄供诊断。
 * @param now 宿主µs。
 * @param received_us 非零原接收/采样锚µs。
 * @return 正常差值；缺锚或时间倒退返回UINT64_MAX，不伪装0年龄。
 */
std::uint64_t run_lease_age_us(std::uint64_t now,std::uint64_t received_us) noexcept {
  return received_us!=0U && now>=received_us ? now-received_us : UINT64_MAX;
}

/**
 * @brief 检查ROS参数permit正数且不超过已验证profile预算。
 * @param requested_us 有符号输入µs。
 * @param profile manifest/activation已加载profile。
 * @return bool；不实际签发permit，也不等于session租期。
 */
bool validated_requested_motion_permit(std::int64_t requested_us,
                                       const DeviceProfile &profile) noexcept {
  return requested_us > 0 &&
    static_cast<std::uint64_t>(requested_us) <= profile.motion_permit_max_us;
}

/**
 * @brief 原子预留唯一续租操作并按Hold/RUN区分busy所有者。
 * @param request 管理单槽。
 * @param busy 当前goal/管理busy。
 * @param reserved 续租唯一预约门。
 * @param running True必须已由goal持busy；False自行抢busy。
 * @return bool；入槽失败回滚本次预约，只归还自己拥有的busy。
 */
bool queue_lease_renewal(std::atomic<int> &request, std::atomic<bool> &busy,
                         std::atomic<bool> &reserved, bool running) noexcept {
  bool expected=false;
  if(!reserved.compare_exchange_strong(expected,true)) return false;
  if(running) {
    if(!busy.load()) { reserved=false; return false; }
  } else if(!busy.compare_exchange_strong(expected,true)) {
    reserved=false;
    return false;
  }
  int no_request=0;
  if(request.compare_exchange_strong(no_request,
       static_cast<int>(wire_value(ManageOperation::kRenewLease)))) return true;
  if(!running) busy=false;
  reserved=false;
  return false;
}

// 调用方持有同一control admission锁；只退尚未提交的RUN预约，不动goal busy。
/**
 * @brief 在完成/取消明确意图出现时撤回尚未发出的RUN续租预约。
 * 不取消已经发送的续租，不赋新续租权限，不延长完成/停止deadline。
 * @param request 只CAS清RenewLease。
 * @param reserved 预约标志。
 * @param running RUN预约类型。
 * @param started 已开始提交则不可撤。
 * @param owns_busy 必须false，不能动goal busy。
 * @param renewal_inflight 已有RPC时不可撤。
 * @param bounded_intent gate证明的有界完成/取消意图。
 * @param explicit_status 若显式pending则改-1供只读服务报告。
 * @return bool；调用方已持同control admission锁。
 */
bool withdraw_queued_running_renewal(std::atomic<int> &request,
    std::atomic<bool> &reserved, std::atomic<bool> &running,
    std::atomic<bool> &started, bool owns_busy, bool renewal_inflight,
    bool bounded_intent, std::atomic<int> &explicit_status) noexcept {
  if(!reserved.load() || !running.load() || started.load() || owns_busy ||
     renewal_inflight || !bounded_intent) return false;
  int renewal=static_cast<int>(wire_value(ManageOperation::kRenewLease));
  if(!request.compare_exchange_strong(renewal,0)) return false;
  reserved=false; running=false; started=false;
  int pending=1;
  explicit_status.compare_exchange_strong(pending,-1);
  return true;
}

/**
 * @brief 一次性消费显式续租“撤回未发送”状态。
 * @param status -1表示待报告撤回。
 * @return CAS-1→0成功为true。
 */
bool take_withdrawn_lease_renewal(std::atomic<int> &status) noexcept {
  int withdrawn=-1;
  return status.compare_exchange_strong(withdrawn,0);
}

/**
 * @brief 仅在当前允许阶段一次性消费显式续租已确认提示。
 * @param explicit_status 2表示确认未读。
 * @param status_available 实际观测/监督允许读取。
 * @param operation_complete Bootstrap单步已完成。
 * @param hold 调用方接受的Hold或Running阶段布尔值。
 * @param last_operation 须RenewLease。
 * @return CAS2→0成功为true；不执行新的续租。
 */
bool take_confirmed_lease_renewal(std::atomic<int> &explicit_status,
                                  bool status_available, bool operation_complete, bool hold,
                                  ManageOperation last_operation) noexcept {
  int confirmed=2;
  return status_available && operation_complete && hold &&
    last_operation==ManageOperation::kRenewLease &&
    explicit_status.compare_exchange_strong(confirmed,0);
}

/**
 * @brief 按Fault/续租/已EnableHold/单步/等待优先级选服务文字。
 * @param phase Bootstrap阶段。
 * @param error Fault时借用非空错误。
 * @param renewal_complete 本次确认提示可读。
 * @param enabled 宿主已发使能流程标志。
 * @param gate_ready gate已握手Hold。
 * @param operation_complete 显式操作完成。
 * @return 静态或error借用文本；不是新设备操作。
 */
const char *bootstrap_status_message(BootstrapPhase phase, const char *error,
                                     bool renewal_complete, bool enabled,
                                     bool gate_ready, bool operation_complete) noexcept {
  if(phase==BootstrapPhase::kFault) return error;
  if(renewal_complete) return "LEASE_RENEWED";
  if(enabled && gate_ready) return "ENABLED_HOLD";
  return operation_complete ? "OPERATION_COMPLETE" : "BOOTSTRAP_PENDING";
}

/**
 * @brief 复用接口谓词检查双消息已释放Idle形状。
 * @param status 已验证Status。
 * @param state 已验证State。
 * @return bool；时间、释放根和原页证明仍由外层负责。
 */
bool released_idle_observation(const wire::DummyDeviceStatusV2 &status,
                               const wire::DummyJointStateV2 &state) noexcept {
  return released_idle_status_confirmed(status,state);
}

/**
 * @brief 比较Status/State的boot/session/reference/motion/management全部水位。
 * @param status Status值。
 * @param state State值。
 * @return bool；不独立检查device/profile/hash或新鲜度。
 */
bool bootstrap_authority_coherent(const wire::DummyDeviceStatusV2 &status,
                                  const wire::DummyJointStateV2 &state) noexcept {
  return state.boot_epoch()==status.boot_epoch() &&
    state.session_id()==status.session_id() &&
    state.reference_epoch()==status.reference_epoch() &&
    state.motion_epoch()==status.motion_epoch() &&
    state.management_epoch()==status.management_epoch();
}

/**
 * @brief 复用完整tuple一致性判断refresh双观察。
 * @param status Status值。
 * @param state State值。
 * @return bool，不自动继承旧Hold资格。
 */
bool refresh_observation_coherent(const wire::DummyDeviceStatusV2 &status,
                                  const wire::DummyJointStateV2 &state) noexcept {
  return bootstrap_authority_coherent(status,state);
}

/**
 * @brief 以正常取消管理receipt、干净终页及紧邻refresh原页确认后继Hold形状。
 * 只匹配原goal取消后保持刷新，不当成成功运动/失能释放；外层仍核双fresh及目标生命周期。
 * @param status 当前Status。
 * @param state 当前State。
 * @param run 原取消goal run身份。
 * @param terminal 原run终页flush。
 * @param successor 后继首页写出。
 * @param cancel_receipt gate已确认Cancel request/op对。
 * @param raw_capture_enabled 原页捕获必须开启。
 * @param raw_failure 必须0。
 * @param axis_mask 必须63。
 * @return bool；run/motion恰+1，同boot/session/reference且双Hold一致，首/终证据符合。
 */
bool timeline_cancel_refresh_hold_confirmed(
    const wire::DummyDeviceStatusV2 &status,
    const wire::DummyJointStateV2 &state,
    const TimelineIdentity &run,
    const RawTerminalIdentity &terminal,
    const RawFirstPageWritten &successor,
    const std::optional<CancelReceipt> &cancel_receipt,
    bool raw_capture_enabled,std::uint8_t raw_failure,
    std::uint8_t axis_mask) noexcept {
  if(!raw_capture_enabled || raw_failure!=0U || axis_mask!=63U ||
     !cancel_receipt || cancel_receipt->request_id==0U ||
     cancel_receipt->operation_id==0U || run.run_id==0U ||
     run.run_id==UINT64_MAX || run.motion_epoch==UINT32_MAX ||
     !clean_cancel_terminal(terminal,run)) return false;
  const TimelineIdentity next{status.boot_epoch(),status.session_id(),
      status.reference_epoch(),status.motion_epoch(),status.run_id()};
  return next.boot==run.boot && next.session==run.session &&
    next.reference_epoch==run.reference_epoch &&
    next.motion_epoch==run.motion_epoch+1U &&
    next.run_id==run.run_id+1U &&
    status.core_state()==wire_value(WireCoreState::kHold) &&
    status.attempt_kind()==1U && status.parent_run_id()==0U &&
    status.final_state()==0U && !status.motion_complete() &&
    !status.evidence_lost() &&
    refresh_observation_coherent(status,state) &&
    state.core_state()==wire_value(WireCoreState::kHold) &&
    state.hold_ready() && state.can_submitted_mask()==axis_mask &&
    state.can_sent_mask()==axis_mask &&
    verified_refresh_successor(successor,next);
}

/**
 * @brief 显式D10释放服务准入快照，汇总双fresh Hold、功能开发范围、gate与Bootstrap及占槽状态，不是设备释放结果。
 */
struct ReleaseAvailability {
  bool releasing=false;
  bool status_fresh=false;
  bool management_epoch_matches=false;
  bool functional_development_basis=false;
  bool enabled=false;
  bool gate_ready=false;
  bool permit_refresh_allowed=false;
  bool busy=false;
  bool management_request_pending=false;
  bool bootstrap_ready=false;
  bool status_hold=false;
  bool state_hold=false;
  bool transport_ready=false;
  bool manual_mode = false, manual_session_owned = false, manual_quiet = false;
};

enum class ManualReleaseDecision { kWait, kSubmit, kUnconfirmed };
class ManualReleaseIntent {
public:
  bool begin(const BootstrapSnapshot &snapshot, std::uint64_t now) noexcept {
    if (snapshot.boot_epoch == 0U || snapshot.session_id == 0U ||
        now > UINT64_MAX - 5000000U)
      return false;
    origin_ = snapshot;
    deadline_ = now + 5000000U;
    active_ = true;
    return true;
  }
  bool active() const noexcept { return active_; }
  ManualReleaseDecision decision(const BootstrapSnapshot &current,
                                 std::uint64_t now, bool slot_available,
                                 bool observation_ready,
                                 bool unknown_fault) const noexcept {
    if (!active_ || now >= deadline_ || unknown_fault ||
        current.boot_epoch != origin_.boot_epoch ||
        current.session_id != origin_.session_id ||
        current.reference_epoch != origin_.reference_epoch ||
        current.profile_id != origin_.profile_id ||
        current.configured_mask != origin_.configured_mask ||
        current.firmware_sha256 != origin_.firmware_sha256 ||
        current.contract_sha256 != origin_.contract_sha256 ||
        current.model_sha256 != origin_.model_sha256 ||
        current.profile_sha256 != origin_.profile_sha256)
      return ManualReleaseDecision::kUnconfirmed;
    return slot_available && observation_ready ? ManualReleaseDecision::kSubmit
                                               : ManualReleaseDecision::kWait;
  }
  bool expired(std::uint64_t now) const noexcept {
    return active_ && now >= deadline_;
  }

private:
  BootstrapSnapshot origin_{};
  std::uint64_t deadline_ = 0U;
  bool active_ = false;
};

inline bool
manual_coordinator_tick_required(bool started,
                                 ManualReferencePhase phase) noexcept {
  return started && phase != ManualReferencePhase::kReadOnly &&
         phase != ManualReferencePhase::kReleased;
}

template <typename Status, typename State>
bool readonly_released_idle_observation(const Status &status,
                                        const State &state) noexcept {
  const auto idle = status.reference_epoch() == 0U ? 1U : 2U;
  return status.boot_epoch() != 0U &&
         status.boot_epoch() == state.boot_epoch() &&
         status.session_id() == 0U && state.session_id() == 0U &&
         status.reference_epoch() == state.reference_epoch() &&
         status.motion_epoch() == state.motion_epoch() &&
         status.management_epoch() == state.management_epoch() &&
         status.core_state() == idle && state.core_state() == idle &&
         !status.real_motion_permitted() && status.permit_remaining_us() == 0U;
}

// 仅固定manual部署的调用者使用；空owner的保留参考必须有真实释放记录与有效绝对位置。
template <class Status, class State>
bool retained_manual_reference_reusable(const Status &status,
                                        const State &state) noexcept {
  return readonly_released_idle_observation(status, state) &&
         status.reference_epoch() != 0U && status.reference_kind() == 1U &&
         status.release_committed() && !status.evidence_lost() &&
         state.position_valid_mask() == 63U;
}

inline bool readonly_release_samples_fresh(std::uint64_t status_at,
                                           std::uint64_t state_at,
                                           std::uint64_t now,
                                           std::uint64_t max_age) noexcept {
  return status_at != 0U && state_at != 0U && now >= status_at &&
         now >= state_at &&
         now - status_at <= dummy_ref::host::kManualReferenceStatusMaxAgeUs &&
         now - state_at <= max_age;
}

/**
 * @brief 合取受控释放入口所需条件。
 * @param state 已经从实际观察构造的释放条件。
 * @return bool；需无release/busy/待管理、已Enable、双Hold/同管理水位、功能开发依据及transport可用。
 */
bool release_operation_available(const ReleaseAvailability &state) noexcept {
  return !state.releasing && state.status_fresh &&
         state.management_epoch_matches && state.functional_development_basis &&
         !state.busy && !state.management_request_pending &&
         state.bootstrap_ready && state.transport_ready &&
         ((state.manual_mode && state.manual_session_owned &&
           state.manual_quiet) ||
          (state.enabled &&
           (state.gate_ready || state.permit_refresh_allowed) &&
           state.status_hold && state.state_hold));
}

/**
 * @brief 用双重available复查和CAS排队一次D10。
 * @param available 控制线程公布的释放可用性。
 * @param request 0为空的管理槽。
 * @param busy 本次释放要独占的宿主标志。
 * @param stage 0空、1排队的释放阶段。
 * @return bool；中间任一CAS失败回滚stage/busy，不发送网络。
 */
bool queue_disable_and_release(const std::atomic<bool> &available,
                               std::atomic<int> &request, std::atomic<bool> &busy,
                               std::atomic<int> &stage) noexcept {
  bool idle=false;
  if(!available.load() || request.load()!=0 || !busy.compare_exchange_strong(idle,true))
    return false;
  int inactive=0;
  int no_request=0;
  if(!available.load() || !stage.compare_exchange_strong(inactive,1) ||
     !request.compare_exchange_strong(
       no_request,static_cast<int>(wire_value(ManageOperation::kDisableAndRelease)))) {
    stage=0;
    busy=false;
    return false;
  }
  return true;
}

/**
 * @brief 宿主动作拒绝类别；NotReady可在极窄续租占槽情形提供短期解释，Invalid/Busy不改成可重试成功。
 */
enum class GoalRejectionReason { kInvalid, kNotReady, kBusy };

/**
 * @brief 新goal共同前置快照：Enable/有效观察/gate Hold/真实port/已绑容差/旧motion完成屏障，不能只以transport ready接纳。
 */
struct AdmissionPrerequisites {
  bool enabled=false;
  bool observation_ready=false;
  bool gate_ready=false;
  bool port_ready=false;
  bool hold_tolerance_bound=false;
  bool transport_begin_ready=false;

  /**
   * @brief 查询新goal六项准入是否同时具备。
   * @return bool；额外续租预约/释放/证据/LIMITED预算还在节点ready快照叠加。
   */
  bool ready() const noexcept {
    return enabled && observation_ready && gate_ready && port_ready &&
      hold_tolerance_bound && transport_begin_ready;
  }
  /** @brief 控制线程发布六项只读诊断位，不替代准入或跨线程读取Port/Gate对象。
   */
  std::uint32_t diagnostic_bits() const noexcept {
    return (enabled ? 1U : 0U) | (observation_ready ? 2U : 0U) |
           (gate_ready ? 4U : 0U) | (port_ready ? 8U : 0U) |
           (hold_tolerance_bound ? 16U : 0U) |
           (transport_begin_ready ? 32U : 0U);
  }
};

/** @brief
 * manual许可恢复只排管理，不放行新goal；上一goal证据、下一Begin屏障及有限预算缺一拒绝。
 */
struct ManualPermitRestoreInputs {
  bool task_ready = false, enabled = false, no_goal = false,
       management_idle = false, fresh_bound_hold = false,
       permit_consumed = false, previous_goal_sealed = false,
       remaining_budget = false;
  std::uint32_t motion_epoch = 0, last_restored_epoch = 0;
  bool ready() const noexcept {
    return task_ready && enabled && no_goal && management_idle &&
           fresh_bound_hold && permit_consumed && previous_goal_sealed &&
           remaining_budget && motion_epoch != 0U &&
           motion_epoch != last_restored_epoch;
  }
};

/**
 * @brief 保留最近一次“仅因续租而NotReady”的目标UUID短期说明。
 * mutex保护ROS回调与只读服务；最多一个条目、两秒寿命，不排队重试goal，不改变原action拒绝响应。
 */
class GoalRejectionReceipt {
 public:
  using Clock=std::chrono::steady_clock;

  /**
   * @brief 仅在窄续租条件符合时保存拒绝UUID及宿主时间。
   * 不符合任一条件清旧条目，避免旧续租提示覆盖后来的真实故障或无效目标。
   * @param uuid 被拒绝goal的16字节ID。
   * @param reason 真实拒绝分类。
   * @param renewal_reserved 已有续租预约。
   * @param admission_prerequisites 其他goal前置全满足。
   * @param release_stage 须0。
   * @param evidence_incomplete 须false。
   * @param fault 须false。
   * @param now 宿主steady time_point。
   */
  void record(const rclcpp_action::GoalUUID &uuid,GoalRejectionReason reason,
              bool renewal_reserved,bool admission_prerequisites,int release_stage,
              bool evidence_incomplete,bool fault,
              Clock::time_point now) {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool eligible=reason==GoalRejectionReason::kNotReady && renewal_reserved &&
      admission_prerequisites &&
      release_stage==0 && !evidence_incomplete && !fault;
    if(!eligible) { latest_.reset(); return; }
    latest_=Entry{uuid,now};
  }

  /**
   * @brief 读取尚在两秒窗内的RENEWAL_PENDING:UUID提示。
   * @param now 宿主steady当前time_point。
   * @return optional拥有字符串；无条目、时钟倒退或超过两秒为空。
   */
  std::optional<std::string> message(Clock::time_point now) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!latest_ || now<latest_->at || now-latest_->at>std::chrono::seconds(2)) return {};
    static constexpr char digits[]="0123456789abcdef";
    std::string result="RENEWAL_PENDING:";
    result.reserve(result.size()+latest_->uuid.size()*2);
    for(const auto byte:latest_->uuid) {
      result.push_back(digits[byte>>4U]);
      result.push_back(digits[byte&0x0fU]);
    }
    return result;
  }

 private:
  /**
   * @brief 单个拒绝条目拥有UUID值及steady time_point；不保存用户Goal消息或设备授权。
   */
  struct Entry { rclcpp_action::GoalUUID uuid; Clock::time_point at; };
  mutable std::mutex mutex_;
  std::optional<Entry> latest_;
};

/**
 * @brief 把精确长度十六进制参数解码为定长字节值。
 * @tparam Size 输出字节数，当前用于16字节设备/客户端ID。
 * @param text 恰2×Size字符，逐对必须完全解析。
 * @return 数组，非法抛invalid_argument；本函数本身不拒绝全零，后续transport/start核零ID。
 */
template<std::size_t Size> std::array<std::uint8_t,Size> hex_id(const std::string &text) {
  if(text.size()!=Size*2) throw std::invalid_argument("invalid hexadecimal identity length");
  std::array<std::uint8_t,Size> bytes{};
  for(std::size_t i=0;i<Size;++i) {
    unsigned value=0; const auto result=std::from_chars(text.data()+i*2,text.data()+i*2+2,value,16);
    if(result.ec!=std::errc{} || result.ptr!=text.data()+i*2+2) throw std::invalid_argument("invalid hexadecimal identity");
    bytes[i]=value;
  }
  return bytes;
}

/**
 * @brief ROS取消回调到单控制消费者的固定8槽请求交接。
 * 每槽0空/1生产中/2ready，CAS占槽后复制shared_ptr并release发布；控制方只消费ready，不无限排队。shared_ptr保持GoalHandle寿命，取消仍绑定当前active对象。
 */
template<class Handle,std::size_t Capacity=8U> class PendingGoalCancels {
 public:
  /**
   * @brief 在首个空槽保存取消目标句柄。
   * @param goal 非空且已由准入层核对当前active的共享句柄。
   * @return bool；满槽拒绝，不覆盖未消费请求。
   */
  bool request(const std::shared_ptr<Handle> &goal) noexcept {
    if(!goal) return false;
    for(auto &slot:slots_) {
      unsigned empty=0U;
      if(!slot.state.compare_exchange_strong(empty,1U,std::memory_order_acquire)) continue;
      slot.goal=goal;
      slot.state.store(2U,std::memory_order_release);
      return true;
    }
    return false;
  }

  /**
   * @brief 控制方取走所有ready取消条目并判断是否匹配active。
   * @param active 当前非空活动目标。
   * @return 是否至少一个相同对象匹配；其他目标条目也被消费，不保留为未来取消。
   */
  bool take_for(const std::shared_ptr<Handle> &active) noexcept {
    if(!active) return false;
    bool matched=false;
    for(auto &slot:slots_) {
      if(slot.state.load(std::memory_order_acquire)!=2U) continue;
      auto requested=std::move(slot.goal);
      slot.state.store(0U,std::memory_order_release);
      matched=matched || requested==active;
    }
    return matched;
  }

 private:
  /**
   * @brief state原子发布shared_ptr值；只有生产者占1时写、消费者见2后move、归0后再复用，单消费者由控制admission保证。
   */
  struct Slot {
    std::atomic<unsigned> state{0U};
    std::shared_ptr<Handle> goal;
  };
  std::array<Slot,Capacity> slots_{};
};

/**
 * @brief 把goal生效/终结与ROS取消准入在同一mutex中串行。
 * 控制循环只try_lock，竞争则跳过本拍；ROS取消request使用lock_guard可能等待控制方。不能泛称所有取消回调都无锁非阻塞。
 */
template<class Handle> class GoalCancelAdmission {
 public:
  /**
   * @brief 持有或未取得的控制侧准入RAII锁，使用前必须测试bool。
   */
  class Control {
   public:
    /**
     * @brief 尝试取得控制侧mutex，不等待ROS回调。
     * @param owner 借用外层admission，须活过本Control。
     */
    explicit Control(GoalCancelAdmission &owner) noexcept
        :owner_(owner),lock_(owner.mutex_,std::try_to_lock) {}
    /**
     * @brief 查询本次try_lock是否拥有准入。
     * @return bool；false时不得调用open/close改active。
     */
    explicit operator bool() const noexcept { return lock_.owns_lock(); }
    /**
     * @brief 持锁登记当前允许接收取消的GoalHandle。
     * @param goal 已经控制方受理的共享句柄。
     */
    void open(const std::shared_ptr<Handle> &goal) noexcept { owner_.active_=goal; }
    /**
     * @brief 持锁清当前取消准入对象，终态后迟到取消不得影响下一goal。
     */
    void close() noexcept { owner_.active_.reset(); }

   private:
    /**
     * @brief owner借用，unique_lock在Control析构时归还；不得早于owner销毁。
     */
    GoalCancelAdmission &owner_;
    std::unique_lock<std::mutex> lock_;
  };

  /**
   * @brief 返回一次控制侧try_lock RAII对象。
   * @return Control，调用者测试后在同scope更新active准入。
   */
  Control try_control() noexcept { return Control(*this); }

  /**
   * @brief ROS取消回调等待准入锁后核目标恰为active并排有限槽。
   * @param goal 请求取消的对象。
   * @param pending 固定取消交接槽。
   * @return bool，空/已终结/另goal或槽满拒绝；不是设备Cancel已执行。
   */
  template<std::size_t Capacity>
  bool request(const std::shared_ptr<Handle> &goal,
               PendingGoalCancels<Handle,Capacity> &pending) {
    std::lock_guard<std::mutex> lock(mutex_);
    return goal && active_==goal && pending.request(goal);
  }

 private:
  std::mutex mutex_;
  /**
   * @brief 仅取消准入对象引用，受mutex保护；与DeviceActionGate控制线程持有的active_通过open/close交接一致。
   */
  std::shared_ptr<Handle> active_;
};

/**
 * @brief 把ActionPort的prime/执行/禁用落到实际JTC或直接Hold生产器。
 * 本对象在控制线程使用，借用/共享持有已发现Controller；命令始终经ros_control进程内通道和唯一DeviceTransport发送。direct_hold只用于没有active JTC时的保持prime，不具备独立轨迹执行能力。
 */
class GuardedPort final : public ActionPort {
 public:
  /**
   * @brief 对象生命周期固定诊断账本；controller共享持有插件，identity为设备命令完整tuple，uuid绑定当前goal摘要登记。
   */
  dummy_ros_control::DiagnosticJournal diagnostics;
  std::shared_ptr<Controller> controller;
  dummy_ros_control::CommandIdentity identity{};
  dummy_ros_control::GoalUuid uuid{};
  /**
   * @brief motion_generation是ActionGate代；started_us为宿主steady起步锚。executing/publishing/direct_hold是宿主生产状态，不是设备core或机械完成。
   */
  std::uint64_t motion_generation=0, started_us=0;
  bool executing=false, failed=false, publishing=false;
  bool direct_hold=false, direct_hold_started=false;
  unsigned refresh_diag_count=0U;
  /**
   * @brief active_count为1或6；trajectory存完整规范路径，hold_position为rad目标值，不可把它当实际反馈。
   */
  std::size_t active_count=0;
  Trajectory trajectory;
  Position hold_position{};
  /**
   * @brief 切到指定设备motion并用真实JTC update或直接通道产生首Hold目标。
   * 先发布当前identity；有controller时先取消旧sink并prime_position，再开启真实update sink。无controller只发进程内一次控制代并开启direct_hold；非零返回只证明本地快照，不是CAN SENT。
   * @param position 来自可信实际/保持快照的rad位置。
   * @param epoch 后继设备motion。
   * @param generation 本gate内部代。
   */
  void prime(const Position &position,std::uint32_t epoch,std::uint64_t generation) override {
    failed=false; publishing=false;
    executing=false; identity.motion_epoch=epoch; motion_generation=generation; hold_position=position;
    dummy_ros_control::set_command_identity(identity);
    if(controller) {
      direct_hold=false;
      controller->set_update_sink(nullptr,nullptr);
      if(!controller->prime_position(position)) { failed=true; return; }
      controller->set_update_sink(dummy_ros_control::controller_update_sink,nullptr);
    } else {
      const auto published = dummy_ros_control::publish_control_update(position.data(),active_count,0,nullptr);
      diagnostics.detail( "GATE_PRIME_DIAG published=%llu boot=%llu session=%llu ref=%u motion=%u ticket=%llu\n",
                   static_cast<unsigned long long>(published),
                   static_cast<unsigned long long>(identity.boot),
                   static_cast<unsigned long long>(identity.session), identity.reference_epoch,
                   identity.motion_epoch, static_cast<unsigned long long>(identity.ticket));
      if(published==0U) { failed=true; return; }
      direct_hold=true; direct_hold_started=true;
    }
    publishing=true;
  }
  /**
   * @brief 在JTC缺席时续同值Hold快照，并监督插件生命周期。
   * Controller不再active且正在执行轨迹则failed；若只是保持可退回direct_hold。刷新只产生本地新代，Complete后的同值吞并/必要刷新仍由worker严格判定，不能自行延设备Hold。
   */
  void refresh_direct_hold() {
    if(controller && controller->get_lifecycle_state().id()!=lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      controller->set_update_sink(nullptr,nullptr);
      controller.reset();
      if(executing) { failed=true; publishing=false; direct_hold=false; }
      else if(publishing) {
        dummy_ros_control::set_command_identity(identity);
        direct_hold=true; direct_hold_started=true;
      }
    }
    if(direct_hold && publishing && !controller) {
      const auto published=dummy_ros_control::publish_control_update(
          hold_position.data(),active_count,0,nullptr);
      if(refresh_diag_count++<3U)
        diagnostics.detail("GATE_HOLD_REFRESH_DIAG published=%llu motion=%u ticket=%llu\n",
                     static_cast<unsigned long long>(published),identity.motion_epoch,
                     static_cast<unsigned long long>(identity.ticket));
      if(published==0U) failed=true;
    }
  }
  /**
   * @brief 在控制拍接管非RT发现的active JTC。
   * 若原direct_hold，先用同hold_position prime插件再装sink；成功后停止direct生产，避免两个采样源并跑。
   * @param active 共享插件候选，必须已active且本port尚无controller。
   */
  void adopt_controller(std::shared_ptr<Controller> active) {
    if(controller || !active ||
       active->get_lifecycle_state().id()!=lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) return;
    if(direct_hold) {
      if(!active->prime_position(hold_position)) { failed=true; return; }
      active->set_update_sink(dummy_ros_control::controller_update_sink,nullptr);
    }
    controller=std::move(active);
    direct_hold=false;
    failed=false;
  }
  /**
   * @brief 判断本地生产链是否可用。
   * @return publishing且无failed，direct_hold或有效active controller成立；不等于设备许可或运动完成。
   */
  bool control_ready() const {
    return publishing && !failed && (direct_hold ||
      (controller && controller->get_lifecycle_state().id()==lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE));
  }
  /**
   * @brief 规范编码轨迹摘要并经JTC一次性凭据登记/提交。
   * 复制所有路点成ROS消息；UUID+全轨迹摘要相符且register/submit都成功才记executing及started_us。调用失败只置failed，由控制监督请求取消，不直接宣称动作已执行。
   * @param value 完整路径，rad/rad/s/rad/s²与相对秒。
   * @param generation gate已握手后的执行代。
   */
  void start(const Trajectory &value,std::uint64_t generation) override {
    auto message=std::make_shared<trajectory_msgs::msg::JointTrajectory>();
    message->joint_names=value.joints;
    for(const auto &point:value.points) {
      trajectory_msgs::msg::JointTrajectoryPoint item;
      item.positions=point.positions; item.velocities=point.velocities; item.accelerations=point.accelerations;
      item.time_from_start=rclcpp::Duration::from_seconds(point.time); message->points.push_back(std::move(item));
    }
    const dummy_ros_control::GoalCredential credential{uuid,dummy_ros_control::trajectory_digest(*message)};
    if(!controller || !controller->register_goal(credential) || !controller->submit_registered_goal(credential,message)) {
      failed=true; return;
    }
    trajectory=value; motion_generation=generation; started_us=management_now_us(); executing=true;
  }
  /**
   * @brief 冻结后继update生产，保留已经存在的最后命令代供commit。
   * 由同一控制线程串行调用；Complete冻结路径位于post-update，失权/显式释放等也可在pre-update调用。解除sink但保留identity供末seq收尾；不是设备失能操作。
   */
  void disable() override {
    // Keep the last generation publishable while Complete waits for accepted_seq.
    // 同控制线程串行冻结；Complete在update/write后调用，其他释放路径也可在prepare_update内调用。
    if(controller) controller->set_update_sink(nullptr,nullptr);
    publishing=false; executing=false;
    direct_hold=false;
  }
  /**
   * @brief 执行disable并要求gate走真实worker排空握手。
   * @return 始终true；实际排空/commit由后续观测证明。
   */
  bool freeze_hold() override { disable(); return true; }
  /**
   * @brief 撤销进程内命令授权并要求JTC取消/保持。
   * 不生成设备D10/Cancel成功回执，ActionGate/Bootstrap另行走管理协议。
   */
  void cancel() override {
    dummy_ros_control::clear_command_identity();
    if(controller) controller->cancel_execution();
  }
};

/**
 * @brief 核端口已绑定同boot/session/reference再发gate使能。
 * @param gate 借用ActionGate。
 * @param port 借用GuardedPort。
 * @param authorization Bootstrap真实许可快照。
 * @param now 宿主µs。
 * @return bool；gate请求受理后同步port预期motion，仍需后续Hold/prime握手。
 */
bool enable_gate_with_port_authority(ActionGate &gate, GuardedPort &port,
                                     const Authorization &authorization,
                                     std::uint64_t now) {
  const auto &identity=authorization.identity;
  if(identity.expected_boot_epoch()!=port.identity.boot ||
     identity.expected_session_id()!=port.identity.session ||
     identity.expected_reference_epoch()!=port.identity.reference_epoch ||
     !gate.enable(authorization,now)) return false;
  port.identity.motion_epoch=identity.expected_motion_epoch();
  return true;
}

/**
 * @brief 组合ROS公开action、显式管理服务和控制线程状态账本的节点。
 * executor回调只占有限原子/锁槽；prepare_update/cycle在同控制线程串行推进gate/bootstrap。DeviceWorker模式只借共享通信owner；多数状态由控制线程推进，ROS事件以原子意图、准入锁和goal mutex分别交接，不能把这些局部锁称为整个节点的全局事务。
 */
class DeviceActionGate final : public rclcpp::Node {
 public:
  /**
   * @brief 读取明确绑定参数/证据，创建共享管理适配、服务与公开action回调。
   * 参数错误/不可信bundle/activation/LIMITED时间线复用直接抛异常。构造只创建软件图和只读观察能力，不自动使能/采零；明确服务请求及新鲜真实状态才推进后续管理。
   */
  DeviceActionGate():Node("device_action_gate") {
    parameters_.endpoint=declare_parameter<std::string>("endpoint","");
    const auto device_id=declare_parameter<std::string>("device_id","");
    parameters_.device=hex_id<16>(device_id);
    parameters_.client=hex_id<16>(declare_parameter<std::string>("client_instance_id",""));
    const auto profile=declare_parameter<std::int64_t>("profile_id",0);
    const auto mask=declare_parameter<std::int64_t>("configured_mask",0);
    const auto session=declare_parameter<std::int64_t>("session_id",0);
    expected_session_id_ = static_cast<std::uint64_t>(session);
    /**
     * @brief permit为动作许可µs，renewal为会话租期µs，state_max_age为接收年龄µs；三者互不替代。正数/上限校验在创建管理对象前完成。
     */
    const auto permit_us=declare_parameter<std::int64_t>("permit_remaining_us",10000000);
    const auto renewal_us=declare_parameter<std::int64_t>("requested_session_lease_us",0);
    const auto state_max_age=declare_parameter<std::int64_t>("state_max_age_us",250000);
    if(profile<=0 || profile>UINT32_MAX || (mask!=1 && mask!=63) || session<=0 || permit_us<=0 ||
       renewal_us<0 || renewal_us>3600000000LL ||
       state_max_age<=0 || state_max_age>1000000)
      throw std::invalid_argument("profile/mask/session/permit duration required");
    state_max_age_us_=static_cast<std::uint64_t>(state_max_age);
    continuity_=ObservationContinuity(state_max_age_us_,2000000U,500000U);
    requested_lease_us_=static_cast<std::uint64_t>(renewal_us);
    parameters_.profile=profile; parameters_.mask=mask;
    controller_name_=declare_parameter<std::string>("guarded_controller","arm_controller");
    manual_reference_mode_ =
        declare_parameter<bool>("manual_reference_mode", false);
    operator_confirmed_double_limits_ =
        declare_parameter<bool>("operator_confirmed_double_limits", false);
    const auto bundle=declare_parameter<std::string>("contract_bundle","");
    const auto profile_name=declare_parameter<std::string>("profile_name","");
    if(bundle.empty() || profile_name.empty()) throw std::invalid_argument("contract bundle and profile_name are required");
    /**
     * @brief 实机bench必须显式提供activation pin/产物/回归/HIL/USB端点绑定，且只接受功能开发依据；仍为UNQUALIFIED范围，不因加载成功变成正式发布资格。
     */
    if (manual_reference_mode_) {
      manual_deployment_ = load_manual_panel_deployment(
          declare_parameter<std::string>("manual_deployment_root", ""),
          declare_parameter<std::string>("manual_deployment_sha256", ""),
          bundle);
      if (parameters_.profile != manual_deployment_->profile_id ||
          parameters_.mask != manual_deployment_->joint_mask ||
          parameters_.endpoint != manual_deployment_->endpoint ||
          device_id != manual_deployment_->device_id)
        throw std::invalid_argument(
            "manual deployment transport identity mismatch");
      device_profile_ = manual_deployment_->profile;
    } else if (profile_name == "f405_6dof_bench") {
      NoLoadDevActivationRequest activation;
      for(const auto &[name,value] : std::initializer_list<std::pair<const char*,std::string*>>{
            {"activation_path",&activation.activation_path},
            {"activation_sha256",&activation.expected_activation_sha256},
            {"source_bundle_root",&activation.source_bundle_root},
            {"prebuild_bundle_manifest_path",&activation.prebuild_bundle_manifest_path},
            {"compiled_policy_manifest_path",&activation.compiled_policy_manifest_path},
            {"firmware_image_path",&activation.firmware_image_path},
            {"host_executable_path",&activation.host_executable_path},
            {"map_file_path",&activation.map_file_path},
            {"resource_report_path",&activation.resource_report_path},
            {"d_regression_report_path",&activation.d_regression_report_path},
            {"d_regression_provenance_path",&activation.d_regression_provenance_path},
            {"hil_report_path",&activation.hil_report_path},
            {"hil_raw_index_path",&activation.hil_raw_index_path},
            {"hil_evidence_root",&activation.hil_evidence_root},
            {"elf_path",&activation.elf_path},
            {"resource_base_root",&activation.resource_base_root},
            {"usb_serial",&activation.usb_serial}}) {
        *value=declare_parameter<std::string>(name,"");
        if(value->empty()) throw std::invalid_argument(std::string("NO_LOAD_DEV requires ")+name);
      }
      activation.device_id=device_id;
      activation.endpoint=parameters_.endpoint;
      device_profile_=load_device_profile(bundle,profile_name,parameters_.profile,parameters_.mask,activation);
      if(!device_profile_.functional_development_basis)
        throw std::invalid_argument("NO_LOAD_DEV_FUNCTIONAL basis required");
    } else {
      device_profile_=load_device_profile(bundle,profile_name,parameters_.profile,parameters_.mask);
    }
    if(!validated_requested_motion_permit(permit_us,device_profile_))
      throw std::invalid_argument("permit duration exceeds verified profile limit");
    profile_=device_profile_.joints;
    /**
     * @brief LIMITED把目标数和boot/session固定，并要求一次性timeline路径与activation完全相同；预算消费另在控制线程最终受理目标时完成。
     */
    if(device_profile_.no_load_basis==NoLoadBasis::kFunctionalLimited &&
       !limited_goal_budget_.configure(device_profile_.goal_limit,
         device_profile_.limited_boot_epoch,static_cast<std::uint64_t>(session)))
      throw std::invalid_argument("invalid limited functional goal budget");
    const auto timeline_path=declare_parameter<std::string>("goal_timeline_path","");
    if(device_profile_.no_load_basis==NoLoadBasis::kFunctionalLimited &&
       timeline_path!=device_profile_.limited_goal_timeline_path)
      throw std::invalid_argument("limited functional invocation timeline mismatch");
    if(!timeline_path.empty()) {
      timeline_=std::make_unique<GoalTimeline>(timeline_path);
      if(timeline_->failed()) {
        if(device_profile_.no_load_basis==NoLoadBasis::kFunctionalLimited)
          throw std::invalid_argument("limited functional invocation already used or failed");
        evidence_incomplete_=true;
      }
    }
    /**
     * @brief 每秒只读一次证据粘滞失败并记固定诊断，不在ROS回调中把日志成功当作证据修复。
     */
    evidence_failure_timer_=create_wall_timer(std::chrono::seconds(1),[this] {
      if(evidence_incomplete_.load() && !evidence_failure_logged_.exchange(true)) {
        port_.diagnostics.reject_from_context(dummy_ros_control::DiagnosticReason::kEvidenceFailure,
          management_now_us(),"evidence_incomplete");
        port_.diagnostics.detail("EVIDENCE_INCOMPLETE timeline_failed=%d raw_failure=%u",
          timeline_ && timeline_->failed(),
          transport_ ? static_cast<unsigned>(transport_->raw_evidence_failure()) : 0U);
      }
    });
    port_.active_count=profile_.joints.size();
    /**
     * @brief 显式DeviceWorker复用ros_control通信channel，不创建第二个USB Context；start成功仅本地参数有效，ready/能力/许可随后独立观察。
     */
    transport_=std::make_unique<DpManagementTransport>(parameters_,DpManagementTransport::Connection::kDeviceWorker);
    if(!transport_->start()) throw std::invalid_argument("invalid device transport parameters");
    std::random_device random;
    const auto request_nonce=(static_cast<std::uint64_t>(random())<<32U) | random();
    client_=std::make_unique<WireManagementClient>(*transport_,static_cast<std::uint64_t>(session),
                                                   request_nonce);
    ManageRequest bootstrap_identity{};
    std::uint8_t device[16]{}, client_id[16]{};
    std::copy(parameters_.device.begin(),parameters_.device.end(),device);
    std::copy(parameters_.client.begin(),parameters_.client.end(),client_id);
    bootstrap_identity.set_device_id(device); bootstrap_identity.set_client_instance_id(client_id);
    bootstrap_identity.set_expected_session_id(static_cast<std::uint64_t>(session));
    /**
     * @brief 功能开发会话使用30秒保守租期模型，动作permit单独给定；gate的2秒握手期限与bootstrap释放5秒、底层Stop预算不是同一计时器。
     */
    bootstrap_ = std::make_unique<BootstrapCoordinator>(
        *client_, bootstrap_identity, device_profile_.policy_hash, permit_us,
        (device_profile_.functional_development_basis || manual_reference_mode_)
            ? 30000000U
            : 0U);
    gate_=std::make_unique<ActionGate>(*client_,port_,profile_,2000000);
    if (manual_deployment_)
      manual_reference_ = std::make_unique<ManualReferenceCoordinator>(
          *bootstrap_, manual_deployment_->reference);
    panel_status_publisher_ =
        create_publisher<dpsdk_interfaces::msg::DummyDeviceStatusV2>(
            "~/device_status", rclcpp::QoS(10));
    panel_state_publisher_ =
        create_publisher<dpsdk_interfaces::msg::DummyJointStateV2>(
            "~/device_state", rclcpp::QoS(10));
    panel_feedback_timer_ = create_wall_timer(
        std::chrono::milliseconds(20), [this] { publish_panel_feedback(); });
    if (manual_reference_) {
      manual_calibration_service_ = create_service<std_srvs::srv::Trigger>(
          "~/calibrate_manual_reference",
          [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                 std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            auto admission = cancel_admission_.try_control();
            const bool reuse =
                admission &&
                retained_manual_reference_reusable(panel_status_,
                                                   panel_state_) &&
                readonly_release_samples_fresh(
                    panel_status_received_us_, panel_state_received_us_,
                    management_now_us(), state_max_age_us_);
            if (!admission || (!operator_confirmed_double_limits_ && !reuse) ||
                !observation_ready_.load() || manual_started_ ||
                manual_calibration_requested_.load() || enabled_ ||
                busy_.load() || release_stage_.load() != 0 ||
                management_request_.load() != 0 || !bootstrap_ ||
                bootstrap_->phase() != BootstrapPhase::kIdle ||
                !manual_reference_ || latest_bootstrap_.session_id != 0U ||
                (latest_bootstrap_.reference_epoch != 0U && !reuse)) {
              response->message = "MANUAL_CALIBRATION_UNAVAILABLE";
              return;
            }
            if (!bootstrap_->begin_manual_permit_window(
                    latest_bootstrap_.boot_epoch, management_now_us())) {
              response->message = "MANUAL_TASK_PERMIT_UNAVAILABLE";
              return;
            }
            manual_request_boot_ = latest_bootstrap_.boot_epoch;
            manual_request_sequence_ = panel_state_.state_seq();
            manual_request_reference_ = latest_bootstrap_.reference_epoch;
            manual_request_reuse_ = reuse;
            manual_calibration_requested_ = true;
            response->success = true;
            response->message = "MANUAL_CALIBRATION_QUEUED";
          });
      manual_status_service_ = create_service<std_srvs::srv::Trigger>(
          "~/manual_reference_status",
          [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                 std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            auto admission = cancel_admission_.try_control();
            if (!admission) {
              response->message = "MANUAL_STATUS_BUSY";
              return;
            }
            if (manual_reference_->phase() == ManualReferencePhase::kFault ||
                !manual_error_.empty()) {
              response->message =
                  "MANUAL_FAILED:" +
                  (!manual_error_.empty()
                       ? manual_error_
                       : std::string(manual_reference_->error()));
              return;
            }
            response->success =
                observation_ready_.load() &&
                manual_reference_->phase() == ManualReferencePhase::kReady;
            response->message = response->success ? "READY" : "MANUAL_PENDING";
          });
    }
    /**
     * @brief ~/enable回调只排enable_requested并返回“queued”；真正自动bootstrap及EnableHold在控制拍执行，success不代表当前电机已使能。
     */
    enable_service_=create_service<std_srvs::srv::Trigger>("~/enable",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        response->success=!evidence_incomplete_.load() && release_stage_.load()==0 && enable_available_.load() &&
          !enable_requested_.exchange(true);
        response->message=response->success ? "Enable queued; wait for confirmed HOLD before submitting motion" :
          "Enable unavailable phase="+std::to_string(static_cast<unsigned>(bootstrap_->phase()))+
          " transport="+(transport_->ready()?"ready":"not_ready")+
          " boot="+std::to_string(latest_bootstrap_.boot_epoch)+
          " raw="+(latest_bootstrap_.raw_ready?"ready":"not_ready");
      });
    /**
     * @brief ~/bootstrap_status用try admission读一致流程状态，区分RELEASE_PENDING/UNCONFIRMED/RELEASED及证据不完整；显式续租确认/撤回提示一次消费，不冒充动作成功。
     */
    bootstrap_status_service_=create_service<std_srvs::srv::Trigger>("~/bootstrap_status",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        auto admission=cancel_admission_.try_control();
        if(!admission) { response->success=false; response->message="BOOTSTRAP_STATUS_UNAVAILABLE"; return; }
        if(fault_cleanup_confirmed_.load() && !evidence_incomplete_.load() && transport_->raw_evidence_failure()==0U) {
          response->success=true;response->message="FAULT_CLEANUP_CONFIRMED";return;
        }
        const int release=release_stage_.load();
        if(evidence_incomplete_.load()) {
          response->success=false;
          response->message=release==3 && observation_ready_.load() ?
            "EVIDENCE_INCOMPLETE;RELEASED_IDLE" :
            (release==2 ? "EVIDENCE_INCOMPLETE;RELEASE_PENDING" :
             "EVIDENCE_INCOMPLETE;RELEASE_UNCONFIRMED");
          return;
        }
        if(release!=0) {
          response->success=release==3 && observation_ready_.load();
          response->message=release==3 ?
            (response->success ? "RELEASED_IDLE" : "RELEASE_OBSERVATION_UNAVAILABLE") :
            (release==4 ? "RELEASE_UNCONFIRMED" : "RELEASE_PENDING");
          return;
        }
        const bool operation_complete=management_request_.load()==0 && bootstrap_ && bootstrap_->operation_complete();
        const bool renewal_pending=renewal_reserved_.load();
        if(bootstrap_->phase()!=BootstrapPhase::kFault &&
           take_withdrawn_lease_renewal(explicit_renewal_status_)) {
          response->success=false; response->message="LEASE_RENEWAL_WITHDRAWN_NOT_SENT"; return;
        }
        const bool renewal_complete=take_confirmed_lease_renewal(
          explicit_renewal_status_,observation_ready_.load() && !renewal_pending &&
          bootstrap_->phase()!=BootstrapPhase::kFault,
          operation_complete,
          gate_->phase()==Phase::kHold || gate_->phase()==Phase::kRunning,
          bootstrap_->last_operation());
        response->success=observation_ready_ && !renewal_pending &&
          (renewal_complete || (enabled_ ? ready_.load() : operation_complete)) &&
          bootstrap_->phase()!=BootstrapPhase::kFault;
        if(renewal_pending) response->message="LEASE_RENEWAL_PENDING";
        else if(bootstrap_->phase()==BootstrapPhase::kFault)
          response->message=bootstrap_status_message(bootstrap_->phase(),bootstrap_->error(),
            false,enabled_,gate_->ready(),operation_complete);
        else if(renewal_complete) response->message="LEASE_RENEWED";
        else response->message=bootstrap_status_message(bootstrap_->phase(),bootstrap_->error(),
          false,enabled_,gate_->ready(),operation_complete);
      });
    // 既有bootstrap/2秒receipt消息合同保持；独立只读CB组不占control admission锁。
    /**
     * @brief 独立互斥诊断回调组避免格式化JSON占用control admission；只读固定账本，但字符串分配不属于硬实时控制路径。
     */
    diagnostic_callback_group_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    diagnostic_snapshot_service_=create_service<std_srvs::srv::Trigger>("~/diagnostic_snapshot",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        response->success=true;
        response->message="{\"schema\":\"DUMMY_ROS_DIAGNOSTIC_SNAPSHOT_V1\",\"gate\":"+
          dummy_ros_control::diagnostic_journal_json(port_.diagnostics)+",\"worker\":"+
          (transport_?transport_->diagnostic_json():"null")+"}";
      },rclcpp::ServicesQoS(),diagnostic_callback_group_);
    /**
     * @brief 仅查询最近两秒、其他准入满足而被续租挡住的UUID提示；Fault/释放/丢证据不沿用该提示，也不会在服务内重提目标。
     */
    last_goal_rejection_service_=create_service<std_srvs::srv::Trigger>("~/last_goal_rejection",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        if(release_stage_.load()!=0 || evidence_incomplete_.load() ||
           bootstrap_fault_snapshot_.load()) {
          response->message="NO_RECENT_RENEWAL_REJECTION";
          return;
        }
        const auto message=goal_rejection_receipt_.message(GoalRejectionReceipt::Clock::now());
        response->success=message.has_value();
        response->message=message.value_or("NO_RECENT_RENEWAL_REJECTION");
      });
    /**
     * @brief 装配单步bootstrap操作服务，回调只CAS排管理编号。
     * 服务返回排队说明，控制线程复核最新观察后才start_operation；不在服务回调直接发送副作用管理请求。
     * @param name ROS私有服务名。
     * @param operation Select/Grant/Capture/SetPermit之一。
     */
    const auto add_operation_service=[this](const char *name,ManageOperation operation) {
      management_services_.push_back(create_service<std_srvs::srv::Trigger>(name,
        [this,operation](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          const bool available=operation==ManageOperation::kSetPermit ?
            permit_operation_available_.load() : explicit_operation_available_.load();
          response->success=release_stage_.load()==0 && available &&
            queue_management_request(management_request_,busy_,renewal_reserved_,
                                     static_cast<int>(wire_value(operation)),available);
          response->message=response->success ? "Operation queued; poll bootstrap_status for final result" :
            "Operation unavailable or another management request is pending";
        }));
    };
    add_operation_service("~/select_control_mode",ManageOperation::kSelectMode);
    add_operation_service("~/grant_session",ManageOperation::kGrantSession);
    add_operation_service("~/capture_bench_reference",ManageOperation::kCaptureReference);
    add_operation_service("~/set_bench_permit",ManageOperation::kSetPermit);
    /**
     * @brief 续租服务持try admission预留单槽，Hold续租拥有busy、RUN续租借goal busy；queued回应不是session已延长，需bootstrap_status确认。
     */
    management_services_.push_back(create_service<std_srvs::srv::Trigger>("~/renew_session_lease",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        auto admission=cancel_admission_.try_control();
        response->success=admission && release_stage_.load()==0 && renew_available_.load() &&
          reserve_lease_renewal(true);
        response->message=response->success ? "Lease renewal queued; poll bootstrap_status" :
          "Lease renewal unavailable or another operation is active";
      }));
    /**
     * @brief 仅在显式SetPermit已成功后排单步EnableHold；不会自动代办缺失模式/会话/参考步骤。
     */
    management_services_.push_back(create_service<std_srvs::srv::Trigger>("~/enable_hold",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        response->success=!evidence_incomplete_.load() && release_stage_.load()==0 &&
          single_enable_available_.load() &&
          !single_enable_requested_.exchange(true);
        response->message=response->success ? "EnableHold queued; poll bootstrap_status for confirmed HOLD" :
          "EnableHold requires a successfully completed SetBenchPermit";
      }));
    /**
     * @brief 释放回调只预留stage1/管理槽并立即关新goal准入，故返回success=false且RELEASE_QUEUED；只有后续COMMITTED+双fresh Idle才能报告释放成功。
     */
    management_services_.push_back(create_service<std_srvs::srv::Trigger>(
        "~/disable_and_release",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          auto admission = cancel_admission_.try_control();
          if (!admission) {
            response->message = "RELEASE_UNAVAILABLE";
            return;
          }
          const auto release_now = management_now_us();
          const bool readonly_fresh = readonly_release_samples_fresh(
              panel_status_received_us_, panel_state_received_us_, release_now,
              state_max_age_us_);
          if (manual_reference_mode_ && observation_ready_.load() &&
              !observation_lost_ &&
              panel_status_.boot_epoch() == latest_bootstrap_.boot_epoch &&
              !enabled_ && !busy_.load() && management_request_.load() == 0 &&
              !manual_calibration_requested_.load() &&
              bootstrap_->release_slot_available() && readonly_fresh &&
              readonly_released_idle_observation(panel_status_, panel_state_)) {
            readonly_release_confirmed_ = true;
            release_stage_ = 3;
            admission_prerequisites_ = false;
            ready_ = false;
            response->success = true;
            response->message = "RELEASED_IDLE";
            return;
          }
          // 同锁重核管理槽；合法在途只排清理意图，不能抢发D10或丢弃意图。
          if (manual_reference_mode_) {
            const bool manual_quiet =
                bootstrap_authority_coherent(panel_status_, panel_state_) &&
                ((panel_status_.core_state() ==
                      wire_value(WireCoreState::kHold) &&
                  panel_state_.core_state() ==
                      wire_value(WireCoreState::kHold)) ||
                 (panel_status_.core_state() ==
                      (panel_status_.reference_epoch() == 0U ? 1U : 2U) &&
                  panel_state_.core_state() == panel_status_.core_state()));
            release_available_ =
                release_stage_.load() == 0 && readonly_fresh &&
                observation_ready_.load() && !busy_.load() &&
                management_request_.load() == 0 &&
                !manual_calibration_requested_.load() &&
                panel_status_.session_id() == expected_session_id_ &&
                manual_quiet && transport_->ready() &&
                (bootstrap_->release_slot_available() ||
                 bootstrap_->management_operation_pending());
          } else if (!bootstrap_->release_slot_available())
            release_available_ = false;
          if (!queue_disable_and_release(release_available_,
                                         management_request_, busy_,
                                         release_stage_)) {
            response->message = "RELEASE_UNAVAILABLE";
            return;
          }
          if (manual_reference_mode_ &&
              !manual_release_intent_.begin(latest_bootstrap_, release_now)) {
            release_stage_ = 4;
            management_request_ = 0;
            response->message = "RELEASE_UNCONFIRMED";
            return;
          }
          admission_prerequisites_ = false;
          ready_ = false;
          response->success = false;
          response->message = "RELEASE_QUEUED";
        }));
    /**
     * @brief 公开FJT入口绑定三个不同回调：目标准入、取消准入、已接受句柄交接。ACCEPT_AND_EXECUTE仅ROS宿主接纳，实际Begin尚待控制拍重验和设备握手。
     */
    server_=rclcpp_action::create_server<Follow>(this,"/arm_controller/follow_joint_trajectory",
      [this](const rclcpp_action::GoalUUID &uuid,std::shared_ptr<const Follow::Goal> goal) {
        if(!validated_device_trajectory(*goal,profile_)) {
          port_.diagnostics.reject_from_context(dummy_ros_control::DiagnosticReason::kGoalValidation,
            management_now_us(),"kInvalid");
          goal_rejection_receipt_.record(uuid,GoalRejectionReason::kInvalid,
            renewal_reserved_.load(),false,release_stage_.load(),evidence_incomplete_.load(),
            bootstrap_fault_snapshot_.load(),GoalRejectionReceipt::Clock::now());
          port_.diagnostics.detail("DEVICE_GOAL_REJECTED_VALIDATION"); return rclcpp_action::GoalResponse::REJECT;
        }
        if(!ready_.load()) {
          const auto bits =
              admission_diagnostic_bits_.load(std::memory_order_acquire);
          char detail[96]{};
          std::snprintf(detail, sizeof(detail),
                        "kNotReady en=%u obs=%u gate=%u port=%u tol=%u "
                        "begin=%u ren=%u rel=%d ev=%u fault=%u",
                        bits & 1U, (bits >> 1U) & 1U, (bits >> 2U) & 1U,
                        (bits >> 3U) & 1U, (bits >> 4U) & 1U, (bits >> 5U) & 1U,
                        renewal_reserved_.load() ? 1U : 0U,
                        release_stage_.load(),
                        evidence_incomplete_.load() ? 1U : 0U,
                        bootstrap_fault_snapshot_.load() ? 1U : 0U);
          port_.diagnostics.reject_from_context(
              dummy_ros_control::DiagnosticReason::kGoalNotReady,
              management_now_us(), detail);
          goal_rejection_receipt_.record(uuid,GoalRejectionReason::kNotReady,
            renewal_reserved_.load(),admission_prerequisites_.load(),release_stage_.load(),
            evidence_incomplete_.load(),bootstrap_fault_snapshot_.load(),
            GoalRejectionReceipt::Clock::now());
          port_.diagnostics.detail("GATE_GOAL_REJECTED_DIAG reason=not_ready busy=%u observation=%u renewal=%u release=%d evidence=%u\n",
            busy_.load() ? 1U : 0U,observation_ready_.load() ? 1U : 0U,
            renewal_reserved_.load() ? 1U : 0U,release_stage_.load(),
            evidence_incomplete_.load() ? 1U : 0U);
          return rclcpp_action::GoalResponse::REJECT;
        }
        /**
         * @brief 原子busy保证最多一个宿主goal/独占管理；LIMITED模式取得busy后还重查ready，防读旧快照越过最后一次预算。
         */
        if(busy_.exchange(true)) {
          port_.diagnostics.reject_from_context(dummy_ros_control::DiagnosticReason::kGoalBusy,
            management_now_us(),"kBusy");
          goal_rejection_receipt_.record(uuid,GoalRejectionReason::kBusy,
            renewal_reserved_.load(),false,release_stage_.load(),evidence_incomplete_.load(),
            bootstrap_fault_snapshot_.load(),GoalRejectionReceipt::Clock::now());
          port_.diagnostics.detail("GATE_GOAL_REJECTED_DIAG reason=busy\n");
          return rclcpp_action::GoalResponse::REJECT;
        }
        // 取得唯一busy后复核ready，阻止读旧ready的callback跨最后预算消费。
        /**
         * @brief LIMITED预算在唯一busy控制受理后实际消费，且必须raw捕获已启用、boot/session匹配；回调中的ACCEPT不是预算兑现或设备运动保证。
         */
        if(device_profile_.no_load_basis==NoLoadBasis::kFunctionalLimited && !ready_.load()) {
          port_.diagnostics.reject_from_context(dummy_ros_control::DiagnosticReason::kGoalNotReady,
            management_now_us(),"budget_admission_changed");
          busy_=false;
          return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      /**
       * @brief 取消回调在admission mutex下核同一active并放8槽，可能等待锁；没有立即调用设备Cancel。
       */
      [this](std::shared_ptr<GoalHandle> handle) {
        return cancel_admission_.request(handle,cancel_requests_) ? rclcpp_action::CancelResponse::ACCEPT :
          rclcpp_action::CancelResponse::REJECT;
      },
      /**
       * @brief accept回调只在goal_mutex下交pending共享句柄，控制线程稍后move并复验，目标对象寿命由shared_ptr保持。
       */
      [this](std::shared_ptr<GoalHandle> handle) { std::lock_guard<std::mutex> lock(goal_mutex_); pending_=std::move(handle); });
  }
  /**
   * @brief 停本地生产/内部执行，停共享管理适配并关闭timeline writer。
   * 共享transport stop不停止ros_control拥有的设备worker；析构返回不能独立证明设备D10已确认。正常进程退出顺序在main先join控制线程，再按所有权销毁节点和manager。
   */
  ~DeviceActionGate() override { port_.disable(); port_.cancel(); transport_->stop(); if(timeline_) timeline_->close(); }
  /**
   * @brief 在非控制ROS定时回调发现指定名称的active受保护JTC。
   * 只动态转换到GuardedController后原子发布shared_ptr；真正adopt发生在控制线程，发现不等于动作接纳。
   * @param manager 借用ControllerManager，get_loaded_controllers会取其非RT锁。
   */
  void attach(controller_manager::ControllerManager &manager) {
    if(std::atomic_load(&discovered_controller_)) return;
    for(const auto &spec:manager.get_loaded_controllers()) {
      if(spec.info.name!=controller_name_) continue;
      auto controller=std::dynamic_pointer_cast<Controller>(spec.c);
      if(controller && controller->get_lifecycle_state().id()==lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
        std::atomic_store(&discovered_controller_,std::move(controller));
    }
  }
  /**
   * @brief 在manager update/write之后处理真实控制代、目标及结果封口。
   * 先try admission，忙就跳本拍；安装本线程诊断scope并捕获异常，异常撤准入并请求gate abort。活动goal终态或消失后close取消准入，避免迟到取消进入下一goal。
   */
  void cycle() noexcept {
    dummy_ros_control::HostDiagnosticScope diagnostic_scope(port_.diagnostics);
    auto admission=cancel_admission_.try_control();
    if(!admission) return;
    try {
      cycle_impl(admission);
      bootstrap_fault_snapshot_=bootstrap_->phase()==BootstrapPhase::kFault;
    }
    catch(const std::exception &error) {
      port_.diagnostics.reject_from_context(dummy_ros_control::DiagnosticReason::kGateFault,management_now_us(),error.what());
      port_.diagnostics.detail("DEVICE_GATE_FAULT: %s",error.what());
      bootstrap_fault_snapshot_=true;
      admission_prerequisites_=false;
      gate_->abort("INTERNAL_BRIDGE_FAILURE",management_now_us()); ready_=false;
    }
    if(!active_ || gate_->terminal()!=Terminal::kNone) admission.close();
  }
  /**
   * @brief 在产生本拍控制采样前刷新真实身份/票据和管理状态。
   * try admission后消费取消、绑定Hold容差、adopt插件并read_observation。证据失败/持续丢观察/续租失去生产者会封锁并中止；先gate tick及必要bootstrap tick再让manager采样，避免末样本冻结后被后到ticket错误退役。
   */
  void prepare_update() noexcept {
    dummy_ros_control::HostDiagnosticScope diagnostic_scope(port_.diagnostics);
    auto admission=cancel_admission_.try_control();
    if(!admission) return;
    try {
      const auto now=management_now_us();
      if(cancel_requests_.take_for(active_)) gate_->cancel(now);
      if(!hold_tolerance_bound_ && transport_->ready())
        hold_tolerance_bound_=transport_->set_post_complete_hold_tolerance(profile_.goal_tolerance);
      if((timeline_ && timeline_->failed()) || transport_->raw_evidence_failure()!=0U)
        evidence_incomplete_=true;
      if(evidence_incomplete_.load()) { admission_prerequisites_=false; ready_=false; }
      port_.adopt_controller(std::atomic_load(&discovered_controller_));
      if(port_.failed) { admission_prerequisites_=false; ready_=false; }
      const bool observation_ready=read_observation(now);
      if(!observation_ready) admission_prerequisites_=false;
      observation_ready_=observation_ready;
      if(renewal_running_.load() && !observation_ready)
        bootstrap_->invalidate("RUN_LEASE_PRODUCER_LOST");
      if(evidence_incomplete_.load()) {
        admission_prerequisites_=false;
        ready_=false;
        if(active_ && gate_->terminal()==Terminal::kNone)
          gate_->abort("EVIDENCE_INCOMPLETE",now);
      }
      if(!observation_ready_) {
        admission_prerequisites_=false;
        ready_=false; enable_available_=false; renew_available_=false;
        explicit_operation_available_=false; permit_operation_available_=false;
        single_enable_available_=false;
        if(observation_lost_) {
          if(bootstrap_) bootstrap_->invalidate("OBSERVATION_UNAVAILABLE");
          if(enabled_) { gate_->invalidate_authorization(now); port_.cancel(); }
        }
      }
      gate_->tick(now);
      if (bootstrap_ && (bootstrap_tick_required(
                             observation_ready_, observation_lost_,
                             release_transition_pending_, release_stage_.load(),
                             bootstrap_->phase(), bootstrap_->last_operation(),
                             bootstrap_->release_request_inflight()) ||
                         (manual_reference_ &&
                          manual_coordinator_tick_required(
                              manual_started_, manual_reference_->phase()))))
        bootstrap_tick(now);
      const bool fault=bootstrap_->phase()==BootstrapPhase::kFault;
      bootstrap_fault_snapshot_=fault;
      if(fault) admission_prerequisites_=false;
    } catch(const std::exception &error) {
      port_.diagnostics.reject_from_context(dummy_ros_control::DiagnosticReason::kObservationRejected,management_now_us(),error.what());
      port_.diagnostics.detail("DEVICE_GATE_OBSERVATION: %s",error.what());
      bootstrap_fault_snapshot_=true;
      admission_prerequisites_=false;
      gate_->abort("INTERNAL_BRIDGE_FAILURE",management_now_us()); ready_=false;
    }
    if(!active_ || gate_->terminal()!=Terminal::kNone) admission.close();
  }
  /**
   * @brief 复制gate对象生命周期固定诊断首现场与计数。
   * @return 值快照；不清首错，不格式化或写文件。
   */
  dummy_ros_control::DiagnosticSnapshot diagnostic_snapshot() const noexcept { return port_.diagnostics.snapshot(); }
  /**
   * @brief 读取首份本地publish参数诊断blob。
   * @return optional值；不是USB TX/CAN SENT原始证据。
   */
  std::optional<dummy_ros_control::DiagnosticBlob> diagnostic_blob() const noexcept { return port_.diagnostics.blob_record(); }

 private:
   bool
   observed_release_idle(const wire::DummyDeviceStatusV2 &status,
                         const wire::DummyJointStateV2 &state) const noexcept {
     if (readonly_release_confirmed_)
       return readonly_released_idle_observation(status, state);
     return manual_reference_mode_ && manual_release_reference_ != 0U
                ? retained_reference_released_idle_observation(
                      status, state, manual_release_reference_)
                : released_idle_observation(status, state);
   }
   // 在非控制executor发布；快照复制受同一个有限准入锁保护，DDS不占控制拍。
   void publish_panel_feedback() {
     wire::DummyDeviceStatusV2 status;
     wire::DummyJointStateV2 state;
     bool publish_status = false, publish_state = false;
     const auto diagnostic = transport_->diagnostic_observation();
     {
       auto admission = cancel_admission_.try_control();
       if (!admission)
         return;
       const auto now = management_now_us();
       if (panel_status_received_us_ != 0U &&
           panel_status_received_us_ > panel_status_published_us_ &&
           (now <= panel_status_received_us_ ||
            now - panel_status_received_us_ <= state_max_age_us_)) {
         status = panel_status_;
         panel_status_published_us_ = panel_status_received_us_;
         publish_status = true;
       }
       if (panel_state_received_us_ != 0U &&
           panel_state_received_us_ > panel_state_published_us_ &&
           (now <= panel_state_received_us_ ||
            now - panel_state_received_us_ <= state_max_age_us_)) {
         state = panel_state_;
         panel_state_published_us_ = panel_state_received_us_;
         publish_state = true;
       }
       // 非授权诊断只更新本次DDS消息和发布水位，绝不回填panel_控制/释放缓存。
       if (diagnostic.status_received_us > panel_status_published_us_ &&
           now >= diagnostic.status_received_us &&
           now - diagnostic.status_received_us <= state_max_age_us_ &&
           wire::DummyDeviceStatusV2Codec::decode(
               diagnostic.status.data(), diagnostic.status.size(), &status) ==
               wire::CodecResult::kOk) {
         panel_status_published_us_ = diagnostic.status_received_us;
         publish_status = true;
       }
       if (diagnostic.state_received_us > panel_state_published_us_ &&
           now >= diagnostic.state_received_us &&
           now - diagnostic.state_received_us <= state_max_age_us_ &&
           wire::DummyJointStateV2Codec::decode(
               diagnostic.state.data(), diagnostic.state.size(), &state) ==
               wire::CodecResult::kOk) {
         panel_state_published_us_ = diagnostic.state_received_us;
         publish_state = true;
       }
     }
     if (publish_status)
       panel_status_publisher_->publish(panel_status_message(status));
     if (publish_state)
       panel_state_publisher_->publish(panel_state_message(state));
   }
  /**
   * @brief 在已有准入scope下预约一次Hold或RUN续租。
   * @param explicit_request True保存显式服务的pending提示；自动预约不覆盖未读撤回提示。
   * @return bool；成功临时清ready并记录busy归属，未实际发送。
   */
  bool reserve_lease_renewal(bool explicit_request=false) noexcept {
    if(explicit_request && explicit_renewal_status_.load()<0) return false;
    const bool running=run_renew_available_.load();
    if(!queue_lease_renewal(management_request_,busy_,renewal_reserved_,running)) return false;
    renewal_owns_busy_=!running;
    renewal_running_=running;
    renewal_started_=false;
    if(explicit_request) explicit_renewal_status_=1;
    else if(explicit_renewal_status_.load()>=0) explicit_renewal_status_=0;
    ready_=false;
    return true;
  }
  /**
   * @brief 综合解码/身份/hash/时间与流程证据，发布管理和goal准入快照。
   * 普通观察来自唯一设备worker已源时钟校验缓存；本层再检查部署hash、boot/参考/epoch及宿主年龄，协调释放和refresh乱序。更新gate实际State与worker排空，维护Timeline/run归属、严格RUN续租前置及显式操作单槽。不能把返回true视为动作成功或全证据封口。
   * @param now 本控制拍宿主steady µs，可能早于刚取得的跨线程接收锚。
   * @return bool当前观察可用于推进；false可为有界等待或确定lost，分别看observation_lost/release_transition_pending。
   */
  bool read_observation(std::uint64_t now) {
    observation_lost_=false;
    release_transition_pending_=false;
    const auto observed=transport_->observe();
    /**
     * @brief 将暂时空观察/短失鲜与持续丢失分开。
     * 捕获now为本拍宿主时间，不改缓存接收锚续鲜。
     * @param value 保留原接收锚的DeviceObservation。
     * @param coherent 本函数已判断的身份/生命周期一致性。
     * @return 连续性Ready为true；Wait为false但尚未lost，超过启动2秒/运行0.5秒空档才Lost。
     */
    const auto wait_for_observation=[this,now](const DeviceObservation &value,bool coherent) {
      const auto result=continuity_.evaluate(value,now,enabled_,coherent);
      observation_lost_=result==ObservationContinuity::Result::kLost;
      return result==ObservationContinuity::Result::kReady;
    };
    if(!observed || observed->state.empty() || observed->status.empty()) {
      const auto identities=port_.diagnostics.context();
      port_.diagnostics.reject(dummy_ros_control::DiagnosticReason::kObservationRejected,{},identities.second,now,"empty_observation");
      port_.diagnostics.detail("DEVICE_BOOTSTRAP_INPUT_REJECTED_EMPTY");
      return wait_for_observation(observed.value_or(DeviceObservation{}),false);
    }
    wire::DummyDeviceStatusV2 status; wire::DummyJointStateV2 state;
    if(wire::DummyDeviceStatusV2Codec::decode(observed->status.data(),observed->status.size(),&status)!=wire::CodecResult::kOk ||
       wire::DummyJointStateV2Codec::decode(observed->state.data(),observed->state.size(),&state)!=wire::CodecResult::kOk) {
      const auto identities=port_.diagnostics.context();
      port_.diagnostics.reject(dummy_ros_control::DiagnosticReason::kObservationRejected,{},identities.second,now,"decode");
      port_.diagnostics.detail("DEVICE_BOOTSTRAP_INPUT_REJECTED_CODEC");
      observation_lost_=true; return false;
    }
    port_.diagnostics.publish_context(
      {state.boot_epoch(),state.session_id(),state.reference_epoch(),state.motion_epoch(),state.state_seq(),state.management_epoch(),true},
      {port_.identity.boot,port_.identity.session,port_.identity.reference_epoch,port_.identity.motion_epoch,
       dummy_ros_control::wire_producer_sequence(last_update_),0U,port_.identity.boot!=0U});
    /**
     * @brief 只对本goal接受后、同命令身份且更晚的typed reject水位中止，防止前一goal拒绝回执污染当前结果。
     */
    if (active_ && gate_->terminal()==Terminal::kNone &&
        observed->rejected_ack_received_us>last_typed_reject_seen_us_ &&
        observed->rejected_ack_sequence!=0U &&
        observed->rejected_ack_boot_epoch==port_.identity.boot &&
        observed->rejected_ack_session_id==port_.identity.session &&
        observed->rejected_ack_reference_epoch==port_.identity.reference_epoch &&
        observed->rejected_ack_motion_epoch==port_.identity.motion_epoch &&
        observed->rejected_ack_received_us>=goal_accepted_us_) {
      last_typed_reject_seen_us_=observed->rejected_ack_received_us;
      gate_->abort("DEVICE_COMMAND_REJECTED_DURING_ACK",now);
    }
    /**
     * @brief 记录确定观察拒绝并标lost。
     * @param reason 固定原因文本，如HASH/BOOT/身份漂移。
     * @return 始终false；调用方随lost撤控制资格。
     */
    const auto reject=[this,now](const char *reason) {
      port_.diagnostics.reject_from_context(dummy_ros_control::DiagnosticReason::kBootstrapRejected,now,reason);
      port_.diagnostics.detail("DEVICE_BOOTSTRAP_INPUT_REJECTED_%s",reason);
      observation_lost_=true;
      return false;
    };
    /**
     * @brief Status五hash与本地已验证bundle/activation对比，LIMITED另钉boot；不是只凭profile号或设备名字放行。
     */
    DeviceStatusIdentity status_identity{};
    std::copy(std::begin(status.contract_sha256()),std::end(status.contract_sha256()),status_identity.contract_hash.begin());
    std::copy(std::begin(status.model_sha256()),std::end(status.model_sha256()),status_identity.model_hash.begin());
    std::copy(std::begin(status.profile_sha256()),std::end(status.profile_sha256()),status_identity.profile_hash.begin());
    std::copy(std::begin(status.compiled_policy_sha256()),std::end(status.compiled_policy_sha256()),status_identity.policy_hash.begin());
    std::copy(std::begin(status.firmware_sha256()),std::end(status.firmware_sha256()),status_identity.firmware_hash.begin());
    if(!std::equal(parameters_.device.begin(),parameters_.device.end(),status.device_id()) ||
       status.profile_id()!=parameters_.profile || status.configured_mask()!=parameters_.mask ||
       !status_matches_bundle_identity(device_profile_,status_identity)) return reject("HASH");
    if(device_profile_.no_load_basis==NoLoadBasis::kFunctionalLimited &&
       status.boot_epoch()!=device_profile_.limited_boot_epoch) return reject("LIMITED_BOOT");
    if (state.boot_epoch() == status.boot_epoch() &&
        state.profile_id() == parameters_.profile &&
        state.configured_mask() == parameters_.mask &&
        std::equal(parameters_.device.begin(), parameters_.device.end(),
                   state.device_id())) {
      panel_status_ = status;
      panel_state_ = state;
      panel_status_received_us_ = observed->status_received_us;
      panel_state_received_us_ = observed->state_received_us;
    }
    if(status.boot_epoch()==0 || !status.motion_supported() ||
       status.core_state()==wire_value(WireCoreState::kFault)) return reject("MOTION");
    if ((status.session_id()!=0U && status.control_mode()==0U) ||
       (status.reference_epoch()!=0U && status.reference_kind()==0U) ||
       (status.real_motion_permitted() != (status.permit_remaining_us()!=0U))) return reject("AUTHORITY");
    const bool releasing=release_stage_.load()!=0;
    if(enabled_ && (status.boot_epoch()!=port_.identity.boot ||
       (!releasing && (status.session_id()!=port_.identity.session ||
                       status.reference_epoch()!=port_.identity.reference_epoch))))
      return reject("AUTHORITY_CHANGED");
    if(latest_bootstrap_.boot_epoch!=0U && status.boot_epoch()!=latest_bootstrap_.boot_epoch)
      return reject("BOOT_CHANGED");
    if(state.boot_epoch()!=status.boot_epoch() || state.profile_id()!=parameters_.profile ||
       state.configured_mask()!=parameters_.mask ||
       !std::equal(parameters_.device.begin(),parameters_.device.end(),state.device_id()))
      return reject("STATE_IDENTITY");
    /**
     * @brief State年龄用原宿主接收µs；now采样早于刚收到消息时按0年龄。源produced偏移/队列年龄已由DeviceTransport核对，本层不拿ROS时间减设备时间。
     */
    const bool state_fresh=observed->state_received_us!=0U &&
      (now<=observed->state_received_us || now-observed->state_received_us<=state_max_age_us_);
    const bool state_fault=state.core_state()==wire_value(WireCoreState::kFault);
    if(state_fresh && state_fault) return reject("STATE_FAULT");
    /**
     * @brief 运行阶段四元组稳定要求随gate状态收紧：已ready/正在执行时motion不可任意+1；过渡仅允许接口定义的单步，不因新epoch就自动授权。
     */
    if(enabled_ && !releasing && state_fresh &&
       !state_authority_is_valid(
         {state.boot_epoch(),state.session_id(),state.reference_epoch(),state.motion_epoch()},
         {port_.identity.boot,port_.identity.session,port_.identity.reference_epoch,
          port_.identity.motion_epoch},state_fault,gate_->ready() || port_.executing))
      return reject("STATE_AUTHORITY_CHANGED");
    const bool released_idle =
        releasing && observed_release_idle(status, state);
    if (readonly_release_confirmed_ && !released_idle)
      return reject("READONLY_RELEASE_AUTHORITY_CHANGED");
    const bool release_status_fresh=observed->status_received_us!=0U &&
      (now<=observed->status_received_us ||
       now-observed->status_received_us<=state_max_age_us_);
    /**
     * @brief 显式D10捕获原tuple后，仅接受同operation形状的Stopping Status/Idle State有限过渡；这时返回等待而不当身份失联或已释放。
     */
    const auto transition=release_transition_origin_.observation(
      {status.boot_epoch(),status.session_id(),status.reference_epoch(),status.motion_epoch()},
      {state.boot_epoch(),state.session_id(),state.reference_epoch(),state.motion_epoch()},
      status.management_epoch(),state.management_epoch(),
      status.core_state(),state.core_state(),
      release_stage_.load());
    if (release_idle_transition_pending(transition,*observed,now,state_max_age_us_) &&
        !status.real_motion_permitted() && status.permit_remaining_us()==0U &&
        state.freshness_ticket()!=0U &&
        (state.raw_position_valid_mask() & parameters_.mask)==parameters_.mask) {
      release_transition_pending_=true;
      return false;
    }
    const bool unconfirmed_idle=releasing &&
      status.core_state()==wire_value(WireCoreState::kUnreferencedIdle) &&
      state.core_state()==wire_value(WireCoreState::kUnreferencedIdle) && !released_idle;
    /**
     * @brief 启动要求双tuple一致；已enabled的持续State由worker/gate核身份，释放则额外要求Status按同state预算fresh及management相同，不借旧Status完成释放。
     */
    const bool coherent=!unconfirmed_idle && (!releasing || release_status_fresh) &&
      (!releasing || state.management_epoch()==status.management_epoch()) &&
      state.freshness_ticket()!=0U &&
      (released_idle || (state.raw_position_valid_mask() & parameters_.mask)==parameters_.mask) &&
      (releasing ? (state.session_id()==status.session_id() &&
                    state.reference_epoch()==status.reference_epoch()) :
                   (enabled_ || bootstrap_authority_coherent(status,state)));
    if(!wait_for_observation(*observed,coherent)) {
      port_.diagnostics.detail("DEVICE_BOOTSTRAP_INPUT_WAITING_FOR_COHERENT_OBSERVATION");
      return false;
    }
    if(enabled_ && !releasing) {
      port_.identity.boot=status.boot_epoch(); port_.identity.session=status.session_id();
      port_.identity.reference_epoch=status.reference_epoch();
      if(port_.identity.ticket!=state.freshness_ticket()) {
        port_.identity.ticket=state.freshness_ticket();
        if(port_.publishing) dummy_ros_control::set_command_identity(port_.identity);
      }
      /**
       * @brief 实际State与worker排空分别送ActionGate：wire管理成功不含实际位置，两者缺一不能伪造prime/Complete握手。
       */
      gate_->observe_state(observed->state.data(),observed->state.size(),now);
      if(const auto worker=transport_->hold_drain_snapshot()) gate_->observe_hold_drain(*worker,now);
    }
    /**
     * @brief bootstrap快照组合Status的模式/许可/hash与State的motion/管理/ticket；通过上面分阶段门控后发布。enabled过渡不要求两topic每字段同帧，不能将组合值当作跨topic原子事务。
     */
    latest_bootstrap_={status.boot_epoch(),status.session_id(),status.reference_epoch(),state.motion_epoch(),
      state.management_epoch(),state.freshness_ticket(),parameters_.profile,parameters_.mask,true,
      status.motion_supported(),status.control_mode(),status.reference_kind(),status.real_motion_permitted(),
      status.permit_remaining_us(),status.core_state(),state.core_state()};
    latest_bootstrap_.run_id=status.run_id();
    std::copy(std::begin(status.firmware_sha256()),std::end(status.firmware_sha256()),latest_bootstrap_.firmware_sha256.begin());
    std::copy(std::begin(status.contract_sha256()),std::end(status.contract_sha256()),latest_bootstrap_.contract_sha256.begin());
    std::copy(std::begin(status.model_sha256()),std::end(status.model_sha256()),latest_bootstrap_.model_sha256.begin());
    std::copy(std::begin(status.profile_sha256()),std::end(status.profile_sha256()),latest_bootstrap_.profile_sha256.begin());
    timeline_identity_={status.boot_epoch(),status.session_id(),status.reference_epoch(),
                        state.motion_epoch(),status.run_id()};
    timeline_refresh_closure_.reset();
    /**
     * @brief 原run完成需motion_complete/final6、双Hold、下一Begin屏障；启用raw时还核相同终页flush身份。该布尔随后再与双fresh/coherent合取。
     */
    const bool completed_hold=status.run_id()==timeline_run_id_ &&
      status.attempt_kind()==1U && status.parent_run_id()==0U &&
      status.final_state()==6U && status.motion_complete() &&
      !status.evidence_lost() &&
      status.core_state()==wire_value(WireCoreState::kHold) &&
      state.core_state()==wire_value(WireCoreState::kHold) && state.hold_ready() &&
      transport_->post_complete_begin_ready() &&
      (!transport_->raw_evidence_capture_enabled() ||
       raw_terminal_matches(transport_->raw_terminal_flushed(),
         status.boot_epoch(),status.session_id(),status.run_id()));
    const TimelineIdentity canceled_run{goal_boot_,goal_session_,
      goal_before_identity_.reference_epoch,timeline_run_motion_epoch_,timeline_run_id_};
    const auto cancel_terminal=transport_->raw_terminal_flushed();
    const auto cancel_successor=transport_->raw_first_page_written();
    const auto cancel_receipt=gate_->confirmed_cancel_receipt();
    const bool raw_capture=transport_->raw_evidence_capture_enabled();
    const auto raw_failure=transport_->raw_evidence_failure();
    const bool canceled_hold=timeline_cancel_hold_confirmed(status,state,
      {goal_boot_,goal_session_,timeline_identity_.reference_epoch,
       timeline_run_motion_epoch_,timeline_run_id_},
      cancel_terminal,parameters_.mask);
    /**
     * @brief 取消refresh闭合还要ROS目标确在canceling、gate终态Canceled及原goal run相同，不能将自主故障或别的goal取消当正常取消。
     */
    const bool cancel_gate_ready=gate_->terminal()==Terminal::kCanceled &&
      active_ && active_->is_canceling() &&
      goal_run_id_!=0U && goal_run_id_==timeline_run_id_;
    const bool canceled_refresh_hold=cancel_gate_ready &&
      timeline_cancel_refresh_hold_confirmed(status,state,canceled_run,
        cancel_terminal,cancel_successor,cancel_receipt,raw_capture,
        raw_failure,parameters_.mask);
    const TimelineIdentity completed_run{goal_boot_,goal_session_,
      goal_before_identity_.reference_epoch,timeline_run_motion_epoch_,timeline_run_id_};
    const bool completed_refresh_hold=goal_run_id_!=0U &&
      goal_run_id_==timeline_run_id_ &&
      timeline_completed_refresh_hold_confirmed(status,state,completed_run,
        cancel_terminal,cancel_successor,raw_capture,raw_failure,parameters_.mask);
    if(!cancel_refresh_diagnostic_logged_ && timeline_ && timeline_goal_open_ &&
       timeline_after_ && timeline_run_id_!=UINT64_MAX &&
       goal_run_id_!=0U && goal_run_id_==timeline_run_id_ &&
       active_ && active_->is_canceling() &&
       status.run_id()==timeline_run_id_+1U &&
       status.core_state()==wire_value(WireCoreState::kHold) &&
       !canceled_refresh_hold) {
      cancel_refresh_diagnostic_logged_=true;
      port_.diagnostics.detail(
        "CANCEL_REFRESH_DIAG run=%llu boot=%llu session=%llu ref=%u epoch=%u goal_run=%llu "
        "status_run=%llu status_boot=%llu status_session=%llu status_ref=%u status_epoch=%u "
        "status_core=%u status_kind=%u status_parent=%llu status_final=%u status_complete=%u status_lost=%u "
        "state_boot=%llu state_session=%llu state_ref=%u state_epoch=%u state_mgmt=%u "
        "status_mgmt=%u state_core=%u hold_ready=%u submitted=%u sent=%u "
        "terminal_run=%llu terminal_boot=%llu terminal_session=%llu terminal_kind=%u terminal_parent=%llu "
        "terminal_final=%u terminal_failure_source=%u terminal_failure_mask=%u terminal_lost=%u "
        "terminal_complete=%u terminal_release=%u terminal_chain=%u safety_valid=%u safety_epoch=%u terminal_ok=%u "
        "page_run=%llu page_boot=%llu page_session=%llu page_epoch=%u page_seq=%u page_count=%u "
        "page_kind=%u page_category=%u page_expected=%u page_submitted=%u page_sent=%u page_failed=%u "
        "page_final=%u page_lost=%u page_chain=%u page_six=%u page_strict=%u page_ok=%u "
        "receipt_request=%llu receipt_operation=%llu receipt_ok=%u raw_capture=%u raw_failure=%u "
        "status_fresh=%u state_fresh=%u coherent=%u status_received_us=%llu state_received_us=%llu now_us=%llu "
        "gate_terminal=%u gate_phase=%u active=%u active_canceling=%u gate_ready=%u axis_mask=%u\n",
        static_cast<unsigned long long>(canceled_run.run_id),
        static_cast<unsigned long long>(canceled_run.boot),
        static_cast<unsigned long long>(canceled_run.session),canceled_run.reference_epoch,
        canceled_run.motion_epoch,static_cast<unsigned long long>(goal_run_id_),
        static_cast<unsigned long long>(status.run_id()),
        static_cast<unsigned long long>(status.boot_epoch()),
        static_cast<unsigned long long>(status.session_id()),status.reference_epoch(),
        status.motion_epoch(),status.core_state(),status.attempt_kind(),
        static_cast<unsigned long long>(status.parent_run_id()),status.final_state(),
        status.motion_complete(),status.evidence_lost(),
        static_cast<unsigned long long>(state.boot_epoch()),
        static_cast<unsigned long long>(state.session_id()),state.reference_epoch(),
        state.motion_epoch(),state.management_epoch(),status.management_epoch(),
        state.core_state(),state.hold_ready(),state.can_submitted_mask(),state.can_sent_mask(),
        static_cast<unsigned long long>(cancel_terminal.run_id),
        static_cast<unsigned long long>(cancel_terminal.boot_epoch),
        static_cast<unsigned long long>(cancel_terminal.session_id),cancel_terminal.attempt_kind,
        static_cast<unsigned long long>(cancel_terminal.parent_run_id),cancel_terminal.final_state,
        cancel_terminal.first_failure_source,cancel_terminal.first_failure_axis_mask,
        cancel_terminal.evidence_lost,cancel_terminal.motion_complete,
        cancel_terminal.release_committed,cancel_terminal.chain_valid,
        cancel_terminal.safety_hold_valid,cancel_terminal.safety_hold_motion_epoch,
        clean_cancel_terminal(cancel_terminal,canceled_run),
        static_cast<unsigned long long>(cancel_successor.run_id),
        static_cast<unsigned long long>(cancel_successor.boot_epoch),
        static_cast<unsigned long long>(cancel_successor.session_id),
        cancel_successor.motion_epoch,cancel_successor.evidence_seq,
        cancel_successor.group_count,cancel_successor.attempt_kind,cancel_successor.category,
        cancel_successor.expected_mask,cancel_successor.submitted_mask,cancel_successor.sent_mask,
        cancel_successor.failed_mask,cancel_successor.final,cancel_successor.evidence_lost,
        cancel_successor.chain_valid,cancel_successor.full_six,
        cancel_successor.strict_refresh_successor,
        verified_refresh_successor(cancel_successor,
          {status.boot_epoch(),status.session_id(),status.reference_epoch(),
           status.motion_epoch(),status.run_id()}),
        static_cast<unsigned long long>(cancel_receipt ? cancel_receipt->request_id : 0U),
        static_cast<unsigned long long>(cancel_receipt ? cancel_receipt->operation_id : 0U),
        cancel_receipt.has_value(),raw_capture,raw_failure,release_status_fresh,state_fresh,
        refresh_observation_coherent(status,state),
        static_cast<unsigned long long>(observed->status_received_us),
        static_cast<unsigned long long>(observed->state_received_us),
        static_cast<unsigned long long>(now),static_cast<unsigned>(gate_->terminal()),
        static_cast<unsigned>(gate_->phase()),active_!=nullptr,
        active_ && active_->is_canceling(),cancel_gate_ready,parameters_.mask);
    }
    /**
     * @brief 只有目标真正AfterBegin后才根据原run/严格refresh证据登记Hold；下面Timeline原goal身份不因设备同值refresh开新run而被偷换。
     */
    if(timeline_ && timeline_goal_open_ && timeline_after_ &&
       release_status_fresh && state_fresh &&
       refresh_observation_coherent(status,state) &&
       (completed_hold || completed_refresh_hold || canceled_hold || canceled_refresh_hold)) {
      timeline_goal_hold_confirmed_=true;
      timeline_goal_hold_identity_=(completed_refresh_hold || canceled_refresh_hold) ?
        TimelineIdentity{goal_boot_,goal_session_,status.reference_epoch(),
          status.motion_epoch(),timeline_run_id_} : timeline_identity_;
    }
    if(timeline_ && timeline_goal_open_ && timeline_after_ &&
       status.run_id()!=timeline_run_id_) {
      const bool raw_capture_enabled=transport_->raw_evidence_capture_enabled();
      const auto raw_terminal=transport_->raw_terminal_flushed();
      const auto successor_page=transport_->raw_first_page_written();
      const bool raw_terminal_matched=raw_capture_enabled &&
        raw_terminal_matches(raw_terminal,goal_boot_,goal_session_,goal_run_id_);
      const bool successor_page_verified=raw_capture_enabled &&
        verified_refresh_successor(successor_page,timeline_identity_);
      const bool status_hold=release_status_fresh &&
        status.core_state()==wire_value(WireCoreState::kHold);
      const bool observation_coherent=refresh_observation_coherent(status,state);
      const bool state_hold=state_fresh &&
        state.core_state()==wire_value(WireCoreState::kHold) &&
        state.hold_ready() && observation_coherent;
      const bool refresh_status=status.attempt_kind()==1U &&
        status.parent_run_id()==0U && status.final_state()==0U &&
        !status.evidence_lost();
      /**
       * @brief 返回的是原goal已确认Hold身份，后继run只作为封口证人；最终Succeeded/Canceled/Aborted时间线不得凭新refresh run冒充原目标。
       */
      timeline_refresh_closure_=timeline_refresh_closure_identity(
        timeline_identity_,timeline_goal_hold_identity_,
        timeline_goal_hold_confirmed_,raw_terminal_matched,successor_page_verified,
        status_hold,state_hold,refresh_status);
      if(!timeline_refresh_closure_) {
        /**
         * @brief refresh短等锚取首次Status真实接收时刻并只赋一次，重复状态不重开窗口；未知run或tuple变化不能走此等待例外。
         */
        if(timeline_refresh_wait_started_us_==0U)
          timeline_refresh_wait_started_us_=observed->status_received_us;
        const TimelineIdentity candidate{status.boot_epoch(),status.session_id(),
          status.reference_epoch(),status.motion_epoch(),status.run_id()};
        const bool refresh_status=status.attempt_kind()==1U && status.parent_run_id()==0U &&
          status.final_state()==0U && !status.evidence_lost() &&
          status.core_state()==wire_value(WireCoreState::kHold);
        /**
         * @brief 已知同值refresh等待独立State期间保留旧已证goal身份，清当前Hold观察并返回false；原终页和双coherent Hold仍必须最终到齐。
         */
        if(timeline_refresh_wait_allowed(candidate,timeline_goal_hold_identity_,
             timeline_goal_hold_confirmed_,refresh_status,timeline_refresh_wait_started_us_,
             now,state_max_age_us_)) {
          // Keep the completed Motion identity while the new Status awaits State.
          // Raw terminal and coherent Hold are still mandatory to close the timeline.
          timeline_identity_=timeline_goal_hold_identity_;
          timeline_hold_observed_=false;
          return false;
        }
        /**
         * @brief 闭合失败粘滞EVIDENCE_INCOMPLETE并只记首份详细诊断，后续新goal不再准入；日志本身不补证据。
         */
        if(!evidence_incomplete_.exchange(true)) {
          const auto wait_age=now>=timeline_refresh_wait_started_us_ ?
            now-timeline_refresh_wait_started_us_ : 0U;
          port_.diagnostics.detail(
            "REFRESH_CLOSURE_DIAG closure_failure=%s wait_failure=%s prior_hold=%u "
            "raw_capture=%u raw_terminal_matched=%u raw_failure=%u "
            "successor_page_present=%u successor_page_verified=%u page_run=%llu page_boot=%llu "
            "page_session=%llu page_epoch=%u page_seq=%u page_count=%u page_kind=%u "
            "page_category=%u page_expected=%u page_submitted=%u page_sent=%u page_failed=%u "
            "page_final=%u page_lost=%u page_strict=%u page_chain=%u page_full_six=%u "
            "status_fresh=%u state_fresh=%u coherent=%u status_core=%u state_core=%u "
            "status_hold=%u state_hold=%u refresh_status=%u status_kind=%u status_parent=%llu "
            "status_final=%u status_lost=%u status_group_count=%u state_hold_ready=%u "
            "status_run=%llu status_boot=%llu status_session=%llu status_ref=%u status_epoch=%u "
            "status_mgmt=%u state_boot=%llu state_session=%llu state_ref=%u state_epoch=%u "
            "state_mgmt=%u hold_run=%llu hold_boot=%llu hold_session=%llu hold_ref=%u hold_epoch=%u "
            "goal_run=%llu goal_boot=%llu goal_session=%llu terminal_run=%llu terminal_boot=%llu "
            "terminal_session=%llu status_received_us=%llu state_received_us=%llu "
            "first_seen_us=%llu now_us=%llu wait_age_us=%llu wait_budget_us=%llu\n",
            refresh_closure_failure(timeline_identity_,timeline_goal_hold_identity_,
              timeline_goal_hold_confirmed_,raw_terminal_matched,successor_page_verified,
              status_hold,state_hold,refresh_status),
            refresh_wait_failure(candidate,timeline_goal_hold_identity_,
              timeline_goal_hold_confirmed_,refresh_status,timeline_refresh_wait_started_us_,
              now,state_max_age_us_),
            timeline_goal_hold_confirmed_,raw_capture_enabled,raw_terminal_matched,
            static_cast<unsigned>(transport_->raw_evidence_failure()),
            successor_page.run_id!=0U,successor_page_verified,
            static_cast<unsigned long long>(successor_page.run_id),
            static_cast<unsigned long long>(successor_page.boot_epoch),
            static_cast<unsigned long long>(successor_page.session_id),
            successor_page.motion_epoch,successor_page.evidence_seq,successor_page.group_count,
            successor_page.attempt_kind,successor_page.category,successor_page.expected_mask,
            successor_page.submitted_mask,successor_page.sent_mask,successor_page.failed_mask,
            successor_page.final,successor_page.evidence_lost,
            successor_page.strict_refresh_successor,successor_page.chain_valid,
            successor_page.full_six,release_status_fresh,state_fresh,observation_coherent,
            status.core_state(),state.core_state(),
            status_hold,state_hold,refresh_status,status.attempt_kind(),
            static_cast<unsigned long long>(status.parent_run_id()),status.final_state(),
            status.evidence_lost(),status.group_count(),state.hold_ready(),
            static_cast<unsigned long long>(status.run_id()),
            static_cast<unsigned long long>(status.boot_epoch()),
            static_cast<unsigned long long>(status.session_id()),status.reference_epoch(),
            status.motion_epoch(),status.management_epoch(),
            static_cast<unsigned long long>(state.boot_epoch()),
            static_cast<unsigned long long>(state.session_id()),state.reference_epoch(),
            state.motion_epoch(),state.management_epoch(),
            static_cast<unsigned long long>(timeline_goal_hold_identity_.run_id),
            static_cast<unsigned long long>(timeline_goal_hold_identity_.boot),
            static_cast<unsigned long long>(timeline_goal_hold_identity_.session),
            timeline_goal_hold_identity_.reference_epoch,timeline_goal_hold_identity_.motion_epoch,
            static_cast<unsigned long long>(goal_run_id_),
            static_cast<unsigned long long>(goal_boot_),
            static_cast<unsigned long long>(goal_session_),
            static_cast<unsigned long long>(raw_terminal.run_id),
            static_cast<unsigned long long>(raw_terminal.boot_epoch),
            static_cast<unsigned long long>(raw_terminal.session_id),
            static_cast<unsigned long long>(observed->status_received_us),
            static_cast<unsigned long long>(observed->state_received_us),
            static_cast<unsigned long long>(timeline_refresh_wait_started_us_),
            static_cast<unsigned long long>(now),static_cast<unsigned long long>(wait_age),
            static_cast<unsigned long long>(state_max_age_us_));
        }
      } else timeline_refresh_wait_started_us_=0U;
    }
    /**
     * @brief 以可信Running/gate阶段及前后run身份确认实际Begin；goal_run_id只在这之后固定，不能从ROS Accepted时的旧run猜。
     */
    const bool begin_observed=trusted_begin_observation(
      goal_before_identity_,timeline_identity_,
      state.core_state()==wire_value(WireCoreState::kRunning),
      gate_->phase()==Phase::kRunning);
    if(active_ && goal_run_id_==0U && begin_observed)
      goal_run_id_=timeline_identity_.run_id;
    timeline_status_core_state_=status.core_state();
    timeline_state_core_state_=state.core_state();
    timeline_hold_observed_=timeline_goal_hold_confirmed_ && state_fresh &&
      release_status_fresh &&
      refresh_observation_coherent(status,state) &&
      status.core_state()==wire_value(WireCoreState::kHold) &&
      state.core_state()==wire_value(WireCoreState::kHold) && state.hold_ready() &&
      (status.run_id()==timeline_run_id_ || timeline_refresh_closure_.has_value());
    if(timeline_ && timeline_goal_open_) {
      if(!timeline_after_ && begin_observed) {
        record_timeline(TimelineEvent::Kind::kAfterBegin,now);
        timeline_run_id_=timeline_identity_.run_id;
        timeline_run_motion_epoch_=timeline_identity_.motion_epoch;
        timeline_after_=true;
      }
      /**
       * @brief Feedback时间线每个新State序号一条，保存设备各轴feedback_seq与CAN年龄；事件时间用该State原宿主接收锚，而非处理拍刷新。
       */
      if(timeline_after_ && !timeline_terminal_ &&
         status.run_id()==timeline_run_id_ &&
         state.state_seq()!=timeline_feedback_state_seq_) {
        auto event=timeline_event(TimelineEvent::Kind::kFeedback,observed->state_received_us);
        event.feedback_seq=std::array<std::uint32_t,6>{};
        event.feedback_age_us=std::array<std::uint32_t,6>{};
        std::copy(std::begin(state.feedback_seq()),std::end(state.feedback_seq()),event.feedback_seq.begin());
        std::copy(std::begin(state.can_rx_age_us()),std::end(state.can_rx_age_us()),event.feedback_age_us.begin());
        if(!timeline_->record(event)) evidence_incomplete_=true;
        timeline_feedback_state_seq_=state.state_seq();
      }
      if(timeline_terminal_ && !timeline_hold_ && timeline_hold_observed_ &&
         (timeline_identity_.run_id==timeline_run_id_ || timeline_refresh_closure_) &&
         timeline_identity_.motion_epoch>=timeline_run_motion_epoch_) {
        record_timeline(TimelineEvent::Kind::kHold,now);
        timeline_hold_=true; timeline_goal_open_=false;
      }
    }
    /**
     * @brief 时间线Release事件还需双消息已释放Idle与Bootstrap确认request/op对；仅stage3字符串或发出D10不足以填Release证据。
     */
    if (timeline_ && !readonly_release_confirmed_ &&
        release_stage_.load() == 3 && !timeline_release_recorded_ &&
        observed_release_idle(status, state)) {
      const auto receipt=bootstrap_ ? bootstrap_->confirmed_release() : std::nullopt;
      if(receipt) {
        auto event=timeline_event(TimelineEvent::Kind::kRelease,now);
        event.release_request_id=receipt->request_id;
        event.release_operation_id=receipt->operation_id;
        timeline_release_recorded_=timeline_->record(event);
      }
      if(!timeline_release_recorded_) evidence_incomplete_=true;
    }
    if(bootstrap_) bootstrap_->observe(latest_bootstrap_,now);
    if (manual_reference_) {
      if (manual_calibration_requested_.exchange(false)) {
        const ConfirmedManualStart confirmed{
            manual_request_boot_, manual_request_sequence_,
            manual_deployment_->reference.deployment_sha256,
            operator_confirmed_double_limits_, manual_request_reuse_};
        manual_started_ = true;
        if (latest_bootstrap_.reference_epoch != manual_request_reference_ ||
            !manual_reference_->start(confirmed, latest_bootstrap_, now))
          manual_error_ = "MANUAL_START_REJECTED";
      }
      if (manual_started_ &&
          manual_reference_->phase() != ManualReferencePhase::kReady) {
        Position raw{}, position{};
        for (std::size_t axis = 0; axis < 6; ++axis) {
          raw[axis] = state.raw_motor_position_rev()[axis] *
                      6.2831853071795864769 *
                      manual_deployment_->direction[axis] /
                      manual_deployment_->gear_ratio[axis];
          position[axis] = state.position_rad()[axis];
        }
        manual_reference_->observe_raw(latest_bootstrap_, raw,
                                       state.state_seq(),
                                       observed->state_received_us);
        if ((state.position_valid_mask() & parameters_.mask) ==
            parameters_.mask)
          manual_reference_->observe(latest_bootstrap_, position,
                                     state.state_seq(),
                                     observed->state_received_us);
      }
    }
    const bool port_control_ready=port_.control_ready();
    const auto gate_phase=gate_->phase();
    const auto gate_terminal=gate_->terminal();
    const bool transport_ready=transport_->ready();
    const bool authority_coherent=bootstrap_authority_coherent(status,state);
    /**
     * @brief 新的RUN续租需真实双Running、同goal/run、端口实际执行且生产最近、transport可用、实际permit与双fresh；不是看到已有session就无限续。
     */
    const RunLeaseEligibility run_lease_inputs{
      enabled_ && status.core_state()==wire_value(WireCoreState::kRunning),
      state.core_state()==wire_value(WireCoreState::kRunning),
      authority_coherent,
      active_ && goal_run_id_!=0U && status.run_id()==goal_run_id_,
      port_.executing && port_control_ready && gate_phase==Phase::kRunning &&
        gate_terminal==Terminal::kNone,
      last_control_update_us_!=0U && now>=last_control_update_us_ &&
        now-last_control_update_us_<=state_max_age_us_,
      transport_ready && status.real_motion_permitted() &&
        state_fresh && release_status_fresh};
    // 完成冻结可与同轮RUN续租预约相遇；不为Completing授予新续租权限。
    ManageRequest completion_live;
    completion_live.set_operation(wire_value(ManageOperation::kComplete));
    completion_live.set_expected_boot_epoch(status.boot_epoch());
    completion_live.set_expected_session_id(status.session_id());
    completion_live.set_expected_reference_epoch(status.reference_epoch());
    completion_live.set_expected_motion_epoch(state.motion_epoch());
    completion_live.set_expected_management_epoch(state.management_epoch());
    completion_live.set_expected_profile_id(status.profile_id());
    completion_live.set_configured_mask(status.configured_mask());
    completion_live.set_device_id(status.device_id());
    std::uint8_t completion_client[16]{};
    std::copy(parameters_.client.begin(),parameters_.client.end(),completion_client);
    completion_live.set_client_instance_id(completion_client);
    /**
     * @brief 冻结完成想撤尚未发RUN续租时，额外要求双消息六轴提交/SENT/有效位及gate有界末采样完成意图。此处不让Completing开启新续租。
     */
    const bool completion_ack=run_lease_inputs.status_running && run_lease_inputs.state_running &&
      run_lease_inputs.status_state_coherent && run_lease_inputs.same_goal_run &&
      run_lease_inputs.producer_recent && run_lease_inputs.transport_ready &&
      status.can_submitted_mask()==parameters_.mask && status.can_sent_mask()==parameters_.mask &&
      status.can_failed_mask()==0U && state.can_submitted_mask()==parameters_.mask &&
      state.can_sent_mask()==parameters_.mask && state.can_failed_mask()==0U &&
      state.position_valid_mask()==parameters_.mask && state.velocity_valid_mask()==parameters_.mask &&
      state.raw_position_valid_mask()==parameters_.mask &&
      gate_->bounded_completion_intent(completion_live,state.committed_producer_seq(),now);
    auto cancel_live=completion_live;
    cancel_live.set_operation(wire_value(ManageOperation::kCancel));
    const bool cancel_intent=run_lease_inputs.status_running && run_lease_inputs.state_running &&
      run_lease_inputs.status_state_coherent && run_lease_inputs.same_goal_run &&
      run_lease_inputs.producer_recent && run_lease_inputs.transport_ready &&
      gate_->bounded_cancel_intent(cancel_live,now);
    /**
     * @brief 只撤尚未提交且不拥有goal busy的RUN预约；已有RPC必须在原完成/取消deadline里等匹配COMMITTED，不能声称撤销了远端续租。
     */
    (void)withdraw_queued_running_renewal(management_request_,renewal_reserved_,renewal_running_,
      renewal_started_,renewal_owns_busy_.load(),bootstrap_->renewal_request_inflight(),
      completion_ack || cancel_intent,explicit_renewal_status_);
    // 已接纳续租期间，冻结末样本可有界替代producer_active，其他新鲜度/身份条件不变。
    /**
     * @brief 既有续租遇到冻结末样本，可有界替代producer_active，其他fresh/身份仍全保留；run_renew_available随后仍取未放宽的原条件。
     */
    const auto renewal_continuation_inputs=run_lease_continuation_inputs(run_lease_inputs,
        renewal_running_.load(),gate_->completion_waiting_for_renewal(now) ||
          gate_->cancel_waiting_for_renewal(cancel_live,now));
    const bool run_lease_active=run_lease_eligible(renewal_continuation_inputs);
    // 新续租仍仅允许原始Running生产条件，不由completion defer开启。
    run_renew_available_=run_lease_eligible(run_lease_inputs);
    if(renewal_running_.load() && !run_lease_active) {
      if(!run_lease_diag_reported_) {
        run_lease_diag_reported_=true;
        port_.diagnostics.detail(
          "RUN_LEASE_DIAG first_false=%s now_us=%llu age_budget_us=%llu "
          "status_running=%d state_running=%d status_state_coherent=%d same_goal_run=%d "
          "producer_active=%d producer_recent=%d transport_ready_input=%d "
          "last_control_update_age_us=%llu status_age_us=%llu state_age_us=%llu "
          "last_control_update_us=%llu status_received_us=%llu state_received_us=%llu "
          "enabled=%d status_core=%u state_core=%u authority_coherent=%d "
          "active=%d goal_run=%llu status_run=%llu "
          "status_boot=%llu state_boot=%llu status_session=%llu state_session=%llu "
          "status_ref=%u state_ref=%u status_motion=%u state_motion=%u "
          "status_mgmt=%u state_mgmt=%u "
          "port_executing=%d port_control_ready=%d gate_phase=%u gate_terminal=%u "
          "transport_ready=%d real_motion_permitted=%d status_fresh=%d state_fresh=%d\n",
          run_lease_first_failed(run_lease_inputs),
          static_cast<unsigned long long>(now),
          static_cast<unsigned long long>(state_max_age_us_),
          run_lease_inputs.status_running,run_lease_inputs.state_running,
          run_lease_inputs.status_state_coherent,run_lease_inputs.same_goal_run,
          run_lease_inputs.producer_active,run_lease_inputs.producer_recent,
          run_lease_inputs.transport_ready,
          static_cast<unsigned long long>(run_lease_age_us(now,last_control_update_us_)),
          static_cast<unsigned long long>(run_lease_age_us(now,observed->status_received_us)),
          static_cast<unsigned long long>(run_lease_age_us(now,observed->state_received_us)),
          static_cast<unsigned long long>(last_control_update_us_),
          static_cast<unsigned long long>(observed->status_received_us),
          static_cast<unsigned long long>(observed->state_received_us),
          enabled_,static_cast<unsigned>(status.core_state()),
          static_cast<unsigned>(state.core_state()),authority_coherent,
          active_!=nullptr,static_cast<unsigned long long>(goal_run_id_),
          static_cast<unsigned long long>(status.run_id()),
          static_cast<unsigned long long>(status.boot_epoch()),
          static_cast<unsigned long long>(state.boot_epoch()),
          static_cast<unsigned long long>(status.session_id()),
          static_cast<unsigned long long>(state.session_id()),
          static_cast<unsigned>(status.reference_epoch()),
          static_cast<unsigned>(state.reference_epoch()),
          static_cast<unsigned>(status.motion_epoch()),
          static_cast<unsigned>(state.motion_epoch()),
          static_cast<unsigned>(status.management_epoch()),
          static_cast<unsigned>(state.management_epoch()),
          port_.executing,port_control_ready,static_cast<unsigned>(gate_phase),
          static_cast<unsigned>(gate_terminal),transport_ready,
          status.real_motion_permitted(),release_status_fresh,state_fresh);
      }
      bootstrap_->invalidate("RUN_LEASE_PRODUCER_LOST");
    }
    /**
     * @brief 控制线程消费一次ROS服务排队编号，再按当前真实快照提交；回调时的available只是旧快照，不能代替这里复核。
     */
    const int requested=management_request_.exchange(0);
    if(requested!=0) {
      explicit_operation_available_=false;
      const auto operation=static_cast<ManageOperation>(requested);
      int queued_release=1;
      if(operation!=ManageOperation::kDisableAndRelease ||
         release_stage_.compare_exchange_strong(queued_release,2)) {
        if(operation==ManageOperation::kDisableAndRelease) {
          port_.disable(); port_.cancel();
        }
        const bool renewal_binding_ready=!renewal_running_.load() || run_lease_active;
        /**
         * @brief 显式D10/续租/普通bootstrap操作分开进入对应状态机；失败清各自预约并封锁，SetPermit成功受理另建立一次性传播latch。
         */
        const bool started =
            operation == ManageOperation::kDisableAndRelease
                ? (manual_reference_mode_ ? (manual_release_waiting_ = true)
                                          : begin_release(now))
            : operation == ManageOperation::kRenewLease
                ? (renewal_binding_ready &&
                   bootstrap_->start_renew_lease(latest_bootstrap_,
                                                 requested_lease_us_, now))
                : bootstrap_->start_operation(latest_bootstrap_, operation,
                                              now);
        if(!started) {
          port_.diagnostics.detail("EXPLICIT_MANAGEMENT_OPERATION_REJECTED operation=%d",requested);
          if(operation==ManageOperation::kRenewLease) {
            bootstrap_->invalidate("LEASE_RENEWAL_REJECTED");
            clear_failed_lease_renewal(management_request_,busy_,renewal_reserved_,
                                       renewal_started_,explicit_renewal_status_,
                                       renewal_owns_busy_.load());
            renewal_running_=false;
            renew_available_=false; admission_prerequisites_=false; ready_=false;
          }
          if (operation == ManageOperation::kSetPermit &&
              manual_reference_mode_)
            bootstrap_->invalidate("MANUAL_PERMIT_RESTORE_REJECTED");
          if(operation==ManageOperation::kDisableAndRelease) release_stage_=4;
        } else if(operation==ManageOperation::kRenewLease) renewal_started_=true;
        else if(operation==ManageOperation::kSetPermit)
          permit_refresh_latch_.begin(latest_bootstrap_.boot_epoch,latest_bootstrap_.session_id);
      }
    }
    port_.identity.boot=status.boot_epoch(); port_.identity.session=status.session_id();
    port_.identity.reference_epoch=status.reference_epoch();
    if(port_.identity.ticket!=state.freshness_ticket()) {
      port_.identity.ticket=state.freshness_ticket();
      if(port_.publishing) dummy_ros_control::set_command_identity(port_.identity);
    }
    enable_available_ =
        !manual_reference_mode_ && !releasing && !enabled_ && bootstrap_ &&
        bootstrap_->phase() == BootstrapPhase::kIdle && transport_->ready();
    explicit_operation_available_ =
        !manual_reference_mode_ && !releasing && !enabled_ && bootstrap_ &&
        (bootstrap_->phase() == BootstrapPhase::kIdle ||
         bootstrap_->operation_complete()) &&
        transport_->ready();
    permit_operation_available_ =
        (!manual_reference_ ||
         manual_reference_->phase() == ManualReferencePhase::kReady) &&
        !releasing &&
        (!manual_reference_mode_ ||
         bootstrap_->manual_permit_remaining_us(now) != 0U) &&
        (explicit_operation_available_.load() ||
         (enabled_ && gate_->permit_refresh_allowed() && bootstrap_ &&
          (bootstrap_->ready() || bootstrap_->operation_complete()) &&
          transport_->ready()));
    single_enable_available_=!releasing && !enabled_ && bootstrap_ && bootstrap_->operation_complete() &&
      bootstrap_->last_operation()==ManageOperation::kSetPermit && transport_->ready();
    if(enable_available_ && enable_requested_.exchange(false)) bootstrap_->start(latest_bootstrap_,now);
    const bool lease_hold=enabled_ &&
      status.core_state()==wire_value(WireCoreState::kHold) &&
      state.core_state()==wire_value(WireCoreState::kHold) && state.hold_ready() &&
      status.session_id()==state.session_id() &&
      status.reference_epoch()==state.reference_epoch() &&
      status.motion_epoch()==state.motion_epoch() &&
      status.management_epoch()==state.management_epoch();
    /**
     * @brief Hold续租要求下一Begin屏障及空闲；RUN续租必须原始生产资格，并都处在Bootstrap有限lead/reserve窗口。预约本身先关ready，防goal与续租抢单槽。
     */
    renew_available_=!releasing && !evidence_incomplete_.load() &&
      requested_lease_us_!=0U &&
      ((lease_hold && release_status_fresh &&
        transport_->post_complete_begin_ready() &&
        (gate_->ready() || gate_->permit_refresh_allowed()) && !busy_.load()) ||
       run_lease_eligible(run_lease_inputs)) && management_request_.load()==0 &&
      bootstrap_ && bootstrap_->renewal_due(now,requested_lease_us_) && transport_->ready();
    if(renew_available_.load() && reserve_lease_renewal()) renew_available_=false;
    /**
     * @brief 释放可用快照仅发布给ROS服务准入，不发D10；功能开发范围、双Hold、无busy/管理和同epoch等要同时满足。
     */
    release_available_ = release_operation_available(
        {releasing, release_status_fresh && state_fresh,
         state.management_epoch() == status.management_epoch(),
         (device_profile_.functional_development_basis ||
          manual_reference_mode_),
         enabled_, gate_->ready(), gate_->permit_refresh_allowed(),
         busy_.load(), management_request_.load() != 0,
         bootstrap_ &&
             (bootstrap_->ready() || bootstrap_->operation_complete() ||
              (manual_reference_mode_ && bootstrap_->release_slot_available())),
         status.core_state() == wire_value(WireCoreState::kHold),
         state.core_state() == wire_value(WireCoreState::kHold),
         transport_->ready(), manual_reference_mode_,
         status.session_id() != 0U &&
             status.session_id() == expected_session_id_,
         bootstrap_authority_coherent(status, state) &&
             ((status.core_state() == wire_value(WireCoreState::kHold) &&
               state.core_state() == wire_value(WireCoreState::kHold)) ||
              (status.core_state() ==
                   (status.reference_epoch() == 0U ? 1U : 2U) &&
               state.core_state() == status.core_state()))});
    return true;
  }
  /**
   * @brief 发起Bootstrap D10并固定释放过渡原身份。
   * 后续观察只用此固定根解释Status/State乱序，不能随到达帧移动原身份。
   * @param now 宿主µs。
   * @return bool；start_release受理后capture原boot/session/reference/motion/management。
   */
  bool begin_release(std::uint64_t now) {
    const auto snapshot=latest_bootstrap_;
    manual_release_reference_ =
        manual_reference_ ? manual_reference_->qualified_reference_epoch() : 0U;
    if (!bootstrap_->start_release(
            snapshot, now, manual_reference_mode_,
            manual_reference_mode_
                ? std::optional<std::uint32_t>(manual_release_reference_)
                : std::nullopt))
      return false;
    release_transition_origin_.capture(
      {snapshot.boot_epoch,snapshot.session_id,snapshot.reference_epoch,snapshot.motion_epoch},
      snapshot.management_epoch);
    return true;
  }
  /**
   * @brief 推进显式释放、续租、permit传播及EnableHold集成。
   * 释放stage2仅在Bootstrap管理+Idle完成转3；Fault转4。续租清预约时只归还其自己busy；SetPermit在同boot/session latch下传gate。Enable先允许inactive prime并核port根身份，请求成功只置宿主enabled，仍需真实Hold握手。
   * @param now 宿主µs。
   */
  void bootstrap_tick(std::uint64_t now) {
    if(!bootstrap_) return;
    /**
     * @brief 一旦进入显式/证据驱动释放，不再走普通自动Enable；stage3后关闭inactive prime，stage4保未确认失败。
     */
    if(release_stage_.load()!=0) {
      if(bootstrap_->phase()!=BootstrapPhase::kFault) bootstrap_->tick(now);
      if (manual_reference_mode_ && release_stage_.load() == 2) {
        if (manual_release_intent_.expired(now)) {
          release_stage_ = 4;
          return;
        }
        if (manual_release_waiting_) {
          const auto decision = manual_release_intent_.decision(
              latest_bootstrap_, now, bootstrap_->release_slot_available(),
              observation_ready_.load(),
              bootstrap_->phase() == BootstrapPhase::kFault &&
                  !bootstrap_->release_slot_available());
          if (decision == ManualReleaseDecision::kUnconfirmed) {
            release_stage_ = 4;
            return;
          }
          if (decision == ManualReleaseDecision::kWait)
            return;
          if (!begin_release(now)) {
            release_stage_ = 4;
            return;
          }
          manual_release_waiting_ = false;
        }
      }
      if(bootstrap_->released()) {
        int started=2;
        if(release_stage_.compare_exchange_strong(started,3)) {
          enabled_=false;
          transport_->allow_inactive_prime(false);
        }
      } else if(bootstrap_->phase()==BootstrapPhase::kFault) release_stage_=4;
      return;
    }
    if(bootstrap_->phase()==BootstrapPhase::kFault && renewal_reserved_) {
      clear_failed_lease_renewal(management_request_,busy_,renewal_reserved_,
                                 renewal_started_,explicit_renewal_status_,
                                 renewal_owns_busy_.load());
      renewal_running_=false;
    }
    if(bootstrap_->phase()==BootstrapPhase::kFault) {
      if (manual_reference_ && manual_started_)
        manual_reference_->tick(now);
      report_bootstrap_fault(now);
      bootstrap_fault_snapshot_=true; admission_prerequisites_=false;
      ready_=false; renew_available_=false;
      if(enabled_) gate_->invalidate_authorization(now);
      return;
    }
    if(bootstrap_->phase()==BootstrapPhase::kIdle) return;
    bootstrap_->tick(now);
    if (manual_reference_ && manual_started_ &&
        manual_reference_->phase() != ManualReferencePhase::kReady &&
        manual_reference_->phase() != ManualReferencePhase::kFault)
      manual_reference_->tick(now);
    /**
     * @brief 结束续租预约时RUN不清goal busy；显式服务状态1→2供只读服务一次确认，失败则清状态，不假装租期已延长。
     */
    if(renewal_reserved_ && renewal_started_ &&
       (bootstrap_->operation_complete() || bootstrap_->phase()==BootstrapPhase::kFault)) {
      if(bootstrap_->operation_complete()) {
        int pending=1;
        explicit_renewal_status_.compare_exchange_strong(pending,2);
      } else explicit_renewal_status_=0;
      renewal_reserved_=false;
      renewal_started_=false;
      if(renewal_owns_busy_.load()) busy_=false;
      renewal_running_=false;
    }
    if(bootstrap_->phase()==BootstrapPhase::kFault) {
      if (manual_reference_ && manual_started_)
        manual_reference_->tick(now);
      report_bootstrap_fault(now);
      bootstrap_fault_snapshot_=true; admission_prerequisites_=false;
      ready_=false; renew_available_=false;
      if(enabled_) gate_->invalidate_authorization(now);
      return;
    }
    /**
     * @brief 重新SetPermit结果只传播一次且必须同boot/session；不能每拍把旧许可重新置有效，也不因此续session。
     */
    if(enabled_ && permit_refresh_latch_.eligible(latest_bootstrap_.boot_epoch,
                                                 latest_bootstrap_.session_id) &&
       bootstrap_->operation_complete() &&
       bootstrap_->last_operation()==ManageOperation::kSetPermit) {
      if(const auto authorization=bootstrap_->authorization())
        if(gate_->renew_permit(*authorization))
          permit_refresh_latch_.applied();
    }
    /**
     * @brief 显式单Enable和自动bootstrap Ready均须真实Authorization、worker inactive-prime准入和port同根；任一失败关临时prime并留不可ready。
     */
    if(!enabled_ && !evidence_incomplete_.load() && single_enable_requested_.exchange(false)) {
      const auto authorization=bootstrap_->authorization();
      if(!authorization || !transport_->allow_inactive_prime(true) ||
         !enable_gate_with_port_authority(*gate_,port_,*authorization,now)) {
        transport_->allow_inactive_prime(false); admission_prerequisites_=false; ready_=false; return;
      }
      enabled_=true;
    }
    if(!enabled_ && !evidence_incomplete_.load() && bootstrap_->ready()) {
      const auto authorization=bootstrap_->authorization();
      if(!authorization || !transport_->allow_inactive_prime(true) ||
         !enable_gate_with_port_authority(*gate_,port_,*authorization,now)) {
        transport_->allow_inactive_prime(false); admission_prerequisites_=false; ready_=false; return;
      }
      enabled_=true;
    }
    if(enabled_ && port_.direct_hold_started && !port_.direct_hold)
      transport_->allow_inactive_prime(false);
  }
  /**
   * @brief 仅一次保存Bootstrap首故障详细现场。
   * 固定journal首拒与可选详情分开，不在控制线程同步格式化输出stderr。
   * @param now 宿主µs。
   */
  void report_bootstrap_fault(std::uint64_t now) {
    if(bootstrap_fault_reported_) return;
    bootstrap_fault_reported_=true;
    port_.diagnostics.reject_from_context(dummy_ros_control::DiagnosticReason::kBootstrapRejected,now,bootstrap_->error());
    port_.diagnostics.detail(
      "BOOTSTRAP_FAULT_DIAG now=%llu error=%s boot=%llu session=%llu observation_lost=%d renewal_reserved=%d gate_phase=%u\n",
      static_cast<unsigned long long>(now),bootstrap_->error(),
      static_cast<unsigned long long>(latest_bootstrap_.boot_epoch),
      static_cast<unsigned long long>(latest_bootstrap_.session_id),
      observation_lost_,renewal_reserved_.load(),static_cast<unsigned>(gate_->phase()));
  }
  /**
   * @brief 从一致JTC实际/期望样本发ROS反馈并判断内部轨迹终态。
   * 反馈header使用ROS时钟；actual/desired/error位置为rad。每拍查path容差，末时刻后actual在goal容差且desired等末点才innerSucceeded；超过goal_time_tolerance+0.25秒还不到则内部Aborted。最终设备Complete/原页/Timeline仍在后续，不以此立即succeed外部Action。
   * @param now 宿主µs，用于elapsed相对秒。
   */
  void check_execution(std::uint64_t now) {
    if(!port_.executing || !port_.controller) return;
    const auto sample=port_.controller->execution_sample(); if(!sample) return;
    /**
     * @brief elapsed由宿主steady起步锚转秒；反馈的ROS header是另一个时钟域，不作票据/状态新鲜判断。
     */
    const auto elapsed=(now>port_.started_us ? now-port_.started_us : 0)*1e-6;
    if(active_ && active_->is_active()) {
      auto feedback=std::make_shared<Follow::Feedback>();
      feedback->header.stamp=this->now(); feedback->joint_names=profile_.joints;
      const auto count=profile_.joints.size();
      feedback->actual.positions.assign(sample->actual.begin(),sample->actual.begin()+count);
      feedback->desired.positions.assign(sample->desired.begin(),sample->desired.begin()+count);
      for(std::size_t i=0;i<count;++i) feedback->error.positions.push_back(sample->desired[i]-sample->actual[i]);
      feedback->actual.time_from_start=rclcpp::Duration::from_seconds(elapsed);
      feedback->desired.time_from_start=feedback->actual.time_from_start;
      feedback->error.time_from_start=feedback->actual.time_from_start;
      active_->publish_feedback(feedback);
    }
    const auto &trajectory=port_.trajectory;
    bool terminal=true;
    for(std::size_t i=0;i<trajectory.joints.size();++i) {
      const double error=std::abs(sample->actual[i]-sample->desired[i]);
      if(!std::isfinite(error) || error>trajectory.path_tolerance[i]) {
        gate_->inner_result(port_.motion_generation,Terminal::kAborted,"PATH_TOLERANCE_VIOLATED",now); return;
      }
      terminal=terminal && std::abs(sample->actual[i]-trajectory.points.back().positions[i])<=trajectory.goal_tolerance[i] &&
          std::abs(sample->desired[i]-trajectory.points.back().positions[i])<=1e-9;
    }
    if(elapsed<trajectory.points.back().time) return;
    if(terminal) gate_->inner_result(port_.motion_generation,Terminal::kSucceeded,"",now);
    else if(elapsed>trajectory.points.back().time+trajectory.goal_time_tolerance+0.25)
      gate_->inner_result(port_.motion_generation,Terminal::kAborted,"GOAL_TOLERANCE_VIOLATED",now);
  }
  /**
   * @brief 在持准入锁的后update阶段交接goal、采样、Timeline及ROS结果。
   * 先消费真实发布的JTC update或直接同值Hold代/排空，再取pending目标并复验profile/LIMITED预算，之后刷新direct Hold和检测执行。gate有终态还要核对已配置的原页/时间线封口，可有限等两秒；异常证据永不报Succeeded。
   * @param admission 已成功取得控制mutex的RAII对象，负责open/close当前取消目标。
   */
  void maybe_restore_manual_permit(std::uint64_t now) {
    if (!manual_reference_mode_ || !manual_reference_ || !bootstrap_)
      return;
    const bool fresh_hold =
        readonly_release_samples_fresh(panel_status_received_us_,
                                       panel_state_received_us_, now,
                                       state_max_age_us_) &&
        bootstrap_authority_coherent(panel_status_, panel_state_) &&
        panel_status_.core_state() == wire_value(WireCoreState::kHold) &&
        panel_state_.core_state() == wire_value(WireCoreState::kHold) &&
        panel_state_.hold_ready() &&
        panel_status_.session_id() == expected_session_id_ &&
        panel_status_.reference_epoch() ==
            manual_reference_->qualified_reference_epoch() &&
        panel_status_.reference_epoch() != 0U;
    const bool previous_sealed =
        timeline_goal_hold_confirmed_ && timeline_hold_ &&
        (!timeline_ || timeline_->drained()) &&
        transport_->post_complete_begin_ready() &&
        transport_->raw_evidence_capture_enabled() &&
        raw_terminal_matches(transport_->raw_terminal_flushed(), goal_boot_,
                             goal_session_, goal_run_id_);
    const ManualPermitRestoreInputs input{
        manual_reference_->phase() == ManualReferencePhase::kReady,
        enabled_,
        !active_ && !busy_.load(),
        management_request_.load() == 0 && !renewal_reserved_.load() &&
            release_stage_.load() == 0 &&
            (bootstrap_->ready() || bootstrap_->operation_complete()) &&
            !evidence_incomplete_.load() && transport_->ready() &&
            transport_->raw_evidence_failure() == 0U,
        fresh_hold,
        gate_->permit_refresh_allowed() &&
            !panel_status_.real_motion_permitted() &&
            panel_status_.permit_remaining_us() == 0U,
        previous_sealed,
        bootstrap_->manual_permit_remaining_us(now) != 0U,
        panel_state_.motion_epoch(),
        manual_last_permit_restore_epoch_};
    if (input.ready() &&
        queue_management_request(
            management_request_, busy_, renewal_reserved_,
            static_cast<int>(wire_value(ManageOperation::kSetPermit)), true)) {
      manual_last_permit_restore_epoch_ = input.motion_epoch;
      ready_ = false;
    }
  }

  void cycle_impl(GoalCancelAdmission<GoalHandle>::Control &admission) {
    static unsigned cycle_diag_count=0U;
    if(port_.direct_hold_started && cycle_diag_count++<3U)
      port_.diagnostics.detail("GATE_CYCLE_DIAG direct_hold=%d publishing=%d failed=%d enabled=%d observation_ready=%d gate_ready=%d error=%s\n",
                   port_.direct_hold,port_.publishing,port_.failed,enabled_,
                   observation_ready_.load(),gate_->ready(),gate_->error().c_str());
    const auto now=management_now_us();
    if(cancel_requests_.take_for(active_)) gate_->cancel(now);
    if((timeline_ && timeline_->failed()) || transport_->raw_evidence_failure()!=0U)
      evidence_incomplete_=true;
    if(evidence_incomplete_.load()) {
      if(device_profile_.no_load_basis==NoLoadBasis::kFunctionalLimited) { limited_goal_budget_.fail(); ready_=false; }
      admission_prerequisites_=false;
      ready_=false;
      if(active_ && gate_->terminal()==Terminal::kNone)
        gate_->abort("EVIDENCE_INCOMPLETE",now);
    }
    /**
     * @brief 登记JTC update或direct_hold真实发布的当前本地授权代（均不证明设备SENT），线序号由64位generation投影；record_update拒绝则中止，last_control_update_us只在成功时推进。
     */
    if(const auto update=dummy_ros_control::consume_control_update(last_update_)) {
      last_update_=update->generation;
      if(!gate_->record_update(port_.motion_generation,update->identity.motion_epoch,
          dummy_ros_control::wire_producer_sequence(update->generation),update->position,
          update->generation)) {
        gate_->abort("CONTROL_UPDATE_REJECTED",now);
      } else last_control_update_us_=now;
    }
    if(const auto drain=transport_->hold_drain_snapshot())
      gate_->observe_hold_drain(*drain,now);
    /**
     * @brief 控制方不等待accept回调的goal_mutex；取得pending后才固定goal接受时间、前身份及LIMITED消费，尚未假定设备已Begin。
     */
    { std::unique_lock<std::mutex> lock(goal_mutex_,std::try_to_lock);
      if(lock && pending_) {
        active_=std::move(pending_); port_.uuid=active_->get_goal_id();
        if(device_profile_.no_load_basis==NoLoadBasis::kFunctionalLimited &&
           (!transport_->raw_evidence_capture_enabled() ||
            !limited_goal_budget_.consume(port_.identity.boot,port_.identity.session))) {
          limited_goal_budget_.fail();
          auto rejected=std::make_shared<Follow::Result>();
          rejected->error_code=Follow::Result::INVALID_GOAL;
          rejected->error_string="LIMITED_FUNCTIONAL_BUDGET_EXHAUSTED_OR_IDENTITY";
          active_->abort(rejected); active_.reset(); busy_=false; ready_=false;
          return;
        }
        if(device_profile_.no_load_basis==NoLoadBasis::kFunctionalLimited &&
           !limited_goal_budget_.available()) ready_=false;
        goal_accepted_us_=now;
        goal_before_identity_=timeline_identity_;
        goal_boot_=timeline_identity_.boot;
        goal_session_=timeline_identity_.session;
        goal_run_id_=0U;
        timeline_terminal_started_us_=0U;
        if(timeline_) {
          timeline_uuid_=port_.uuid;
          timeline_digest_=dummy_ros_control::trajectory_digest(active_->get_goal()->trajectory);
          timeline_goal_open_=true; timeline_after_=timeline_terminal_=timeline_hold_=false;
          fault_cleanup_confirmed_=false;timeline_fault_cleanup_=false;
          timeline_goal_hold_confirmed_=false;
          cancel_refresh_diagnostic_logged_=false;
          timeline_refresh_wait_started_us_=0U;
          timeline_refresh_closure_.reset();
          record_timeline(TimelineEvent::Kind::kAccepted,now);
        }
        /**
         * @brief 控制方在ROS准入之后再次按当前profile转换并submit；失败直接abort本goal并清busy，不偷偷改成末点普通运动。
         */
        const auto trajectory=validated_device_trajectory(*active_->get_goal(),profile_);
        if(evidence_incomplete_.load() || !trajectory || !gate_->submit(*trajectory,now)) {
          auto result=std::make_shared<Follow::Result>(); result->error_code=Follow::Result::INVALID_GOAL;
          result->error_string=evidence_incomplete_.load() ? "EVIDENCE_INCOMPLETE" :
              (!trajectory ? "TRAJECTORY_LIMIT_VIOLATION" :
               (gate_->error().empty() ? "DEVICE_NOT_READY" : gate_->error()));
          if(device_profile_.no_load_basis==NoLoadBasis::kFunctionalLimited) { limited_goal_budget_.fail(); ready_=false; }
          active_->abort(result); active_.reset(); busy_=false;
        } else {
          admission.open(active_);
          if(timeline_) record_timeline(TimelineEvent::Kind::kBeforeBegin,now);
        }
      }
    }
    port_.refresh_direct_hold();
    if(port_.failed) gate_->abort("GUARDED_REGISTRATION_FAILED",now);
    check_execution(now);
    /**
     * @brief gate终态只进入结果封口阶段，先记录终态/可信Hold，再核writer与原run终页；不能因为内部JTC或管理Ok就立即返回ROS成功。
     */
    if(active_ && gate_->terminal()!=Terminal::kNone) {
      auto result=std::make_shared<Follow::Result>(); result->error_string=gate_->error();
      if(timeline_ && !timeline_terminal_) {
        const auto kind=gate_->terminal()==Terminal::kSucceeded ? TimelineEvent::Kind::kTerminalSuccess :
          gate_->terminal()==Terminal::kCanceled ? TimelineEvent::Kind::kTerminalCanceled :
          TimelineEvent::Kind::kTerminalAborted;
        record_timeline(kind,now);
        timeline_terminal_=true;
        if(timeline_hold_observed_ &&
           (timeline_identity_.run_id==timeline_run_id_ || timeline_refresh_closure_) &&
           timeline_identity_.motion_epoch>=timeline_run_motion_epoch_) {
          record_timeline(TimelineEvent::Kind::kHold,now);
          timeline_hold_=true; timeline_goal_open_=false;
        }
      }
      // 自治fault清理只读专属原raw快照；不把Fault授权清零当可运行/可续租状态。
      /**
       * @brief 自主Fault清理只读专属writer原始快照，要求四hash、首/终完整fault组及新鲜clear-authority Fault形状；确认清理仍保Aborted，不恢复正常控制/续租。
       */
      if(gate_->terminal()==Terminal::kAborted && !fault_cleanup_confirmed_.load() && timeline_after_) {
        const auto receipt=transport_->raw_terminal_flushed();
        const auto cleanup=transport_->raw_cleanup_observation();
        const auto sampled_after_read=management_now_us();
        wire::DummyDeviceStatusV2 cleanup_status{};
        const bool status_decoded=wire::DummyDeviceStatusV2Codec::decode(cleanup.status.data(),cleanup.status.size(),&cleanup_status)==wire::CodecResult::kOk;
        DeviceStatusIdentity cleanup_identity{};
        if(status_decoded) {
          std::copy(std::begin(cleanup_status.contract_sha256()),std::end(cleanup_status.contract_sha256()),cleanup_identity.contract_hash.begin());
          std::copy(std::begin(cleanup_status.model_sha256()),std::end(cleanup_status.model_sha256()),cleanup_identity.model_hash.begin());
          std::copy(std::begin(cleanup_status.profile_sha256()),std::end(cleanup_status.profile_sha256()),cleanup_identity.profile_hash.begin());
          std::copy(std::begin(cleanup_status.compiled_policy_sha256()),std::end(cleanup_status.compiled_policy_sha256()),cleanup_identity.policy_hash.begin());
          std::copy(std::begin(cleanup_status.firmware_sha256()),std::end(cleanup_status.firmware_sha256()),cleanup_identity.firmware_hash.begin());
        }
        if(transport_->raw_evidence_capture_enabled() && transport_->raw_evidence_failure()==0U && status_decoded &&
           status_matches_bundle_identity(device_profile_,cleanup_identity) &&
           raw_fault_cleanup_matches(receipt,transport_->raw_first_page_written(),cleanup,goal_boot_,goal_session_,goal_run_id_,timeline_run_motion_epoch_,
             parameters_.profile,parameters_.mask,parameters_.device,sampled_after_read,state_max_age_us_)) {
          auto event=timeline_event(TimelineEvent::Kind::kFaultCleanup,sampled_after_read);
          event.identity={goal_boot_,goal_session_,port_.identity.reference_epoch,timeline_run_motion_epoch_,goal_run_id_};
          event.status_core_state=9U;event.state_core_state=9U;
          event.fault_cleanup={receipt.boot_epoch,receipt.session_id,receipt.run_id,receipt.attempt_motion_epoch,
            timeline_run_motion_epoch_,receipt.fault_stop_motion_epoch,receipt.first_failure_source,
            receipt.first_failure_axis_mask,receipt.final_state,receipt.terminal_chain,
            receipt.full_fault_groups_valid,receipt.chain_valid,receipt.release_committed,
            receipt.evidence_lost,receipt.motion_complete,true};
          if(!timeline_ || timeline_->record(event)) {
            fault_cleanup_confirmed_=true;timeline_fault_cleanup_=true;timeline_goal_open_=false;
            admission_prerequisites_=false;ready_=false;enable_available_=false;renew_available_=false;
          } else evidence_incomplete_=true;
        }
      }
      const bool raw_capture=transport_->raw_evidence_capture_enabled();
      const auto raw_terminal=transport_->raw_terminal_flushed();
      const bool raw_terminal_ready=raw_terminal_matches(
        raw_terminal,goal_boot_,goal_session_,goal_run_id_);
      /**
       * @brief 普通目标在配置Timeline时须Hold/排空，启用raw时另须对应终页；两者均未配置时不额外等待它们。FaultCleanup专用闭合走不同条件，但仍要求writer无错和终页身份，不混作运动Succeeded。
       */
      const auto decision=fault_cleanup_confirmed_.load() && timeline_fault_cleanup_ && gate_->terminal()==Terminal::kAborted && (!timeline_ || timeline_->drained())
        ? timeline_success_decision(false,evidence_incomplete_.load() || (timeline_ && timeline_->failed()),
            false,!timeline_ || timeline_->drained(),true,transport_->raw_evidence_failure()!=0U,raw_terminal_ready)
        : raw_capture ? timeline_terminal_decision(
        gate_->terminal(),timeline_!=nullptr,
        evidence_incomplete_.load() || (timeline_ && timeline_->failed()),
        timeline_hold_,!timeline_ || timeline_->drained(),
        transport_->raw_evidence_failure()!=0U,raw_terminal_ready) :
        timeline_success_decision(timeline_!=nullptr,
          evidence_incomplete_.load() || (timeline_ && timeline_->failed()),
          timeline_hold_,!timeline_ || timeline_->drained());
      /**
       * @brief 终态后的异步证据从首次等待起使用2秒预算，期间关ready；超时粘滞不完整。明确设备typed命令拒绝不靠等日志掩盖首因。
       */
      if(decision==TimelineSuccessDecision::kWait &&
         result->error_string!="DEVICE_COMMAND_REJECTED_DURING_ACK") {
        if(timeline_terminal_started_us_==0U) timeline_terminal_started_us_=now;
        if(now-timeline_terminal_started_us_<=2000000U) {
          admission_prerequisites_=false;
          ready_=false;
          return;
        }
        evidence_incomplete_=true;
      }
      if(result->error_string=="DEVICE_COMMAND_REJECTED_DURING_ACK") {
        result->error_code=Follow::Result::PATH_TOLERANCE_VIOLATED;
        active_->abort(result);
      } else if(evidence_incomplete_.load() || decision==TimelineSuccessDecision::kEvidenceIncomplete) {
        evidence_incomplete_=true;
        result->error_code=Follow::Result::PATH_TOLERANCE_VIOLATED;
        result->error_string=evidence_incomplete_result_error(result->error_string);
        active_->abort(result);
      /**
       * @brief 回ROS FaultCleanup说明前再次复核原清理快照/终页；仍用abort且PATH_TOLERANCE_VIOLATED，不把已释放故障状态宣称动作通过。
       */
      } else if(fault_cleanup_confirmed_.load() && gate_->terminal()==Terminal::kAborted) {
        const auto final_cleanup=transport_->raw_cleanup_observation();
        if(transport_->raw_evidence_failure()!=0U || !raw_fault_cleanup_matches(
            transport_->raw_terminal_flushed(),transport_->raw_first_page_written(),final_cleanup,
            goal_boot_,goal_session_,goal_run_id_,timeline_run_motion_epoch_,parameters_.profile,
            parameters_.mask,parameters_.device,management_now_us(),state_max_age_us_)) {
          fault_cleanup_confirmed_=false;evidence_incomplete_=true;result->error_code=Follow::Result::PATH_TOLERANCE_VIOLATED;
          result->error_string=evidence_incomplete_result_error(result->error_string);active_->abort(result);
          active_.reset();busy_=false;return;
        }
        result->error_code=Follow::Result::PATH_TOLERANCE_VIOLATED;
        result->error_string=result->error_string.empty()?"FAULT_CLEANUP_CONFIRMED":result->error_string+";FAULT_CLEANUP_CONFIRMED";
        active_->abort(result);
      /**
       * @brief 紧贴ROS succeed边界再次读取异步writer状态和原run终页；两次之间可能出现I/O失败，故不能复用旧成功快照。
       */
      } else if(gate_->terminal()==Terminal::kSucceeded) {
        // Recheck the asynchronous writer immediately at the ROS result boundary.
        const auto final_raw_terminal=transport_->raw_terminal_flushed();
        if(transport_->raw_evidence_failure()!=0U || (timeline_ && timeline_->failed()) ||
           (raw_capture && !raw_terminal_matches(
             final_raw_terminal,goal_boot_,goal_session_,goal_run_id_))) {
          evidence_incomplete_=true;
          result->error_code=Follow::Result::PATH_TOLERANCE_VIOLATED;
          result->error_string=evidence_incomplete_result_error(result->error_string);
          active_->abort(result);
        } else active_->succeed(result);
      }
      else if(transport_->raw_evidence_failure()!=0U || (timeline_ && timeline_->failed()) ||
              (raw_capture && !raw_terminal_matches(transport_->raw_terminal_flushed(),
                goal_boot_,goal_session_,goal_run_id_))) {
        evidence_incomplete_=true;
        result->error_code=Follow::Result::PATH_TOLERANCE_VIOLATED;
        result->error_string=evidence_incomplete_result_error(result->error_string);
        active_->abort(result);
      } else if(gate_->terminal()==Terminal::kCanceled && active_->is_canceling()) active_->canceled(result);
      else { result->error_code=Follow::Result::PATH_TOLERANCE_VIOLATED; active_->abort(result); }
      if(device_profile_.no_load_basis==NoLoadBasis::kFunctionalLimited &&
         (gate_->terminal()!=Terminal::kSucceeded || evidence_incomplete_.load()))
        { limited_goal_budget_.fail(); ready_=false; }
      active_.reset(); busy_=false;
    }
    if(!fault_cleanup_confirmed_.load()) request_evidence_release(now);
    maybe_restore_manual_permit(now);
    /**
     * @brief 下一goal ready重新合成真实Gate/Port/观察/容差/Complete屏障，再扣除续租/释放/证据及LIMITED预算；不是单个长期锁存enable布尔值。
     */
    const AdmissionPrerequisites prerequisites{
      enabled_,observation_ready_.load(),gate_->ready(),port_.control_ready(),
      hold_tolerance_bound_,transport_->post_complete_begin_ready()};
    admission_diagnostic_bits_.store(prerequisites.diagnostic_bits(),
                                     std::memory_order_release);
    const bool fault=bootstrap_->phase()==BootstrapPhase::kFault;
    bootstrap_fault_snapshot_=fault;
    admission_prerequisites_=prerequisites.ready() && !fault;
    ready_ =
        admission_prerequisites_.load() && !renewal_reserved_.load() &&
        release_stage_.load() == 0 && !evidence_incomplete_.load() &&
        (!manual_reference_mode_ ||
         bootstrap_->manual_permit_remaining_us(now) != 0U) &&
        (device_profile_.no_load_basis != NoLoadBasis::kFunctionalLimited ||
         (limited_goal_budget_.available() &&
          transport_->raw_evidence_capture_enabled()));
  }
  /**
   * @brief 证据不完整且已在可信Hold、无active时请求开发态D10收尾。
   * 仅功能开发依据、无旧STOP_FAILED、Bootstrap可释放等条件成立才关闭生产并begin_release；未确认时stage4。不能在未知运动/失联状态把本地证据失败直接说成已安全释放。
   * @param now 宿主µs。
   */
  void request_evidence_release(std::uint64_t now) noexcept {
    if (!evidence_incomplete_.load() ||
        !(device_profile_.functional_development_basis ||
          manual_reference_mode_) ||
        active_ || release_stage_.load() != 0 || !enabled_ ||
        !observation_ready_.load() || !timeline_hold_observed_ ||
        management_request_.load() != 0 ||
        (gate_->terminal() != Terminal::kNone &&
         gate_->error() == "STOP_FAILED") ||
        !bootstrap_ ||
        !(bootstrap_->ready() || bootstrap_->operation_complete()))
      return;
    release_stage_=2;
    admission_prerequisites_=false;
    ready_=false;
    port_.disable(); port_.cancel();
    if(!begin_release(now)) release_stage_=4;
  }
  /**
   * @brief 从当前goal账本构造带明确身份/时钟的时间线事件。
   * 终态/Hold遇已验证refresh闭合时仍用原goal身份；正常Canceled另附已确认管理request/op对。ROS UUID、全轨迹摘要与设备run是独立标识。
   * @param kind 事件类别。
   * @param now 该事件宿主steady µs；Feedback可传原State接收锚。
   * @return 值对象，不执行写盘。
   */
  TimelineEvent timeline_event(TimelineEvent::Kind kind,std::uint64_t now) const noexcept {
    TimelineEvent event{};
    event.kind=kind; event.uuid=timeline_uuid_; event.trajectory_sha256=timeline_digest_;
    event.identity=timeline_refresh_closure_ &&
      (kind==TimelineEvent::Kind::kTerminalSuccess ||
       kind==TimelineEvent::Kind::kTerminalCanceled ||
       kind==TimelineEvent::Kind::kTerminalAborted ||
       kind==TimelineEvent::Kind::kHold) ?
      *timeline_refresh_closure_ : timeline_identity_;
    event.steady_us=now; event.axis_mask=parameters_.mask;
    event.status_core_state=timeline_status_core_state_;
    event.state_core_state=timeline_state_core_state_;
    if(kind==TimelineEvent::Kind::kTerminalCanceled && active_ && active_->is_canceling() &&
       timeline_after_ && goal_run_id_!=0U && goal_run_id_==timeline_run_id_ &&
       event.identity.boot==goal_boot_ && event.identity.session==goal_session_ &&
       event.identity.run_id==goal_run_id_) {
      if(const auto receipt=gate_->confirmed_cancel_receipt()) {
        event.cancel_request_id=receipt->request_id;
        event.cancel_operation_id=receipt->operation_id;
      }
    }
    return event;
  }
  /**
   * @brief 把值事件交有界Timeline，失败锁证据不完整。
   * true只表示记录入口接纳，不是writer排空、fsync或物理动作完成，结果边界还要drained/failed。
   * @param kind 事件类型。
   * @param now 宿主µs。
   * @return 未配置timeline兼容true，已配置时入队成功true，否则false。
   */
  bool record_timeline(TimelineEvent::Kind kind,std::uint64_t now) noexcept {
    if(!timeline_) return true;
    if(timeline_->record(timeline_event(kind,now))) return true;
    evidence_incomplete_=true;
    return false;
  }
  /**
   * @brief 冻结连接身份/profile值与已验证DeviceProfile；Controller发现共享指针原子交接，transport/client/bootstrap/gate按unique_ptr拥有且析构顺序需依赖仍存活。
   */
  DpManagementParameters parameters_;
  JointProfile profile_;
  DeviceProfile device_profile_;
  std::string controller_name_;
  GuardedPort port_;
  std::shared_ptr<Controller> discovered_controller_;
  std::unique_ptr<DpManagementTransport> transport_;
  std::unique_ptr<WireManagementClient> client_;
  std::unique_ptr<BootstrapCoordinator> bootstrap_;
  std::unique_ptr<ActionGate> gate_;
  /**
   * @brief ROS实体归Node持有；诊断组独立互斥以免JSON阻塞control admission。普通服务写原子意图，不直接改多数控制线程私有状态。
   */
  rclcpp_action::Server<Follow>::SharedPtr server_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr bootstrap_status_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr last_goal_rejection_service_;
  rclcpp::CallbackGroup::SharedPtr diagnostic_callback_group_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr diagnostic_snapshot_service_;
  std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> management_services_;
  /**
   * @brief ready是当前goal准入快照，busy是唯一goal或独占管理占槽；cancel队列有8槽，admission锁把目标生效/取消/终态串行。
   */
  std::atomic<bool> ready_{false}, busy_{false};
  PendingGoalCancels<GoalHandle> cancel_requests_;
  GoalCancelAdmission<GoalHandle> cancel_admission_;
  std::atomic<bool> admission_prerequisites_{false}, bootstrap_fault_snapshot_{false};
  std::atomic<bool> enable_requested_{false}, enable_available_{false};
  std::atomic<bool> single_enable_requested_{false}, single_enable_available_{false};
  std::atomic<bool> explicit_operation_available_{false};
  std::atomic<bool> permit_operation_available_{false};
  /**
   * @brief 续租reserved/started/running与owns_busy分开：RUN借goal busy、Hold自己拥有busy；失败/完成只能清自己的占用。
   */
  std::atomic<bool> renew_available_{false}, renewal_reserved_{false}, renewal_started_{false};
  std::atomic<bool> run_renew_available_{false}, renewal_running_{false};
  std::atomic<bool> renewal_owns_busy_{true};
  std::uint32_t manual_last_permit_restore_epoch_ = 0U;
  PermitRefreshLatch permit_refresh_latch_;
  /**
   * @brief 显式续租提示0无待报、1排队/等待、2已确认未读、-1撤回未发送未读；只读服务一次消费，不改变设备租约。
   */
  std::atomic<int> explicit_renewal_status_{0};
  std::uint64_t expected_session_id_ = 0U;
  ManualReleaseIntent manual_release_intent_;
  bool manual_release_waiting_ = false;
  bool readonly_release_confirmed_ = false;
  std::atomic<bool> release_available_{false};
  /**
   * @brief 释放stage：0无、1回调排队、2控制线程已发/等待、3管理+Idle已证、4未确认；management_request是wire操作编号单槽，非request_id。
   */
  std::atomic<int> release_stage_{0};
  std::atomic<int> management_request_{0};
  /**
   * @brief State年龄为宿主接收µs；continuity启动空档2秒/运行0.5秒是等待监督，绝不延长单帧实际年龄或设备票据。
   */
  std::uint64_t state_max_age_us_=250000U;
  ObservationContinuity continuity_{250000U,2000000U,500000U};
  bool observation_lost_=false;
  bool release_transition_pending_=false;
  ReleaseTransitionOrigin release_transition_origin_{};
  bool bootstrap_fault_reported_=false;
  bool run_lease_diag_reported_=false;
  std::atomic<bool> observation_ready_{false};
  std::uint64_t requested_lease_us_=0U;
  std::mutex goal_mutex_;
  std::atomic<std::uint32_t> admission_diagnostic_bits_{0U};
  GoalRejectionReceipt goal_rejection_receipt_;
  /**
   * @brief pending由accept回调在goal_mutex下交接，active只由control处理；last_update为宿主控制代，last_control_update_us为最近真实采样宿主锚。
   */
  std::shared_ptr<GoalHandle> pending_, active_;
  std::uint64_t last_update_=0;
  std::uint64_t last_control_update_us_=0;
  std::uint64_t last_typed_reject_seen_us_=0;
  /**
   * @brief LIMITED预算绑定特定boot/session和次数；enabled是宿主已启动使能流程，不等同gate ready/设备Hold。hold_tolerance_bound是worker实际接受容差设置。
   */
  LimitedGoalBudget limited_goal_budget_;
  bool manual_reference_mode_ = false,
       operator_confirmed_double_limits_ = false, manual_started_ = false;
  std::optional<ManualPanelDeployment> manual_deployment_;
  std::unique_ptr<ManualReferenceCoordinator> manual_reference_;
  std::atomic<bool> manual_calibration_requested_{false};
  std::uint64_t manual_request_boot_ = 0;
  std::uint32_t manual_request_sequence_ = 0, manual_release_reference_ = 0,
                manual_request_reference_ = 0;
  bool manual_request_reuse_ = false;
  std::string manual_error_;
  wire::DummyDeviceStatusV2 panel_status_;
  wire::DummyJointStateV2 panel_state_;
  std::uint64_t panel_status_received_us_ = 0, panel_state_received_us_ = 0;
  std::uint64_t panel_status_published_us_ = 0, panel_state_published_us_ = 0;
  rclcpp::Publisher<dpsdk_interfaces::msg::DummyDeviceStatusV2>::SharedPtr
      panel_status_publisher_;
  rclcpp::Publisher<dpsdk_interfaces::msg::DummyJointStateV2>::SharedPtr
      panel_state_publisher_;
  rclcpp::TimerBase::SharedPtr panel_feedback_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
      manual_calibration_service_,
      manual_status_service_;
  std::uint64_t goal_accepted_us_=0;
  bool enabled_=false;
  bool hold_tolerance_bound_=false;
  BootstrapSnapshot latest_bootstrap_{};
  /**
   * @brief timeline拥有异步writer；evidence_incomplete粘滞，首错只报一次。timeline_identity与goal_before/confirmedHold/refreshClosure分别保存观测、原goal和闭合证人，不互相覆盖假造run。
   */
  std::unique_ptr<GoalTimeline> timeline_;
  std::atomic<bool> evidence_incomplete_{false};
  rclcpp::TimerBase::SharedPtr evidence_failure_timer_;
  std::atomic<bool> evidence_failure_logged_{false};
  TimelineIdentity timeline_identity_{};
  TimelineIdentity timeline_goal_hold_identity_{};
  std::optional<TimelineIdentity> timeline_refresh_closure_;
  TimelineIdentity goal_before_identity_{};
  dummy_ros_control::GoalUuid timeline_uuid_{};
  dummy_ros_control::TrajectoryDigest timeline_digest_{};
  /**
   * @brief goal_boot/session/run固定实际Begin归属；terminal_started和refresh_wait_started为宿主µs总窗锚，后续消息不续窗。motion/core/State序号是设备水位。
   */
  std::uint64_t goal_boot_=0, goal_session_=0, goal_run_id_=0;
  std::uint64_t timeline_terminal_started_us_=0;
  std::uint64_t timeline_refresh_wait_started_us_=0;
  std::uint64_t timeline_run_id_=0;
  std::uint32_t timeline_run_motion_epoch_=0;
  std::uint16_t timeline_status_core_state_=0, timeline_state_core_state_=0;
  std::uint32_t timeline_feedback_state_seq_=0;
  /**
   * @brief 时间线open/AfterBegin/Terminal/Hold/Release分开，FaultCleanup独立闭合但保Aborted；有终态事件不表示writer已排空。
   */
  bool timeline_goal_open_=false, timeline_after_=false, timeline_terminal_=false;
  bool timeline_goal_hold_confirmed_=false;
  bool cancel_refresh_diagnostic_logged_=false;
  std::atomic<bool> fault_cleanup_confirmed_{false};
  bool timeline_fault_cleanup_=false;
  bool timeline_hold_=false, timeline_hold_observed_=false, timeline_release_recorded_=false;
};
}  // namespace dummy_motion_client

#ifndef DUMMY_ACTION_GATE_TEST
/**
 * @brief 创建ROS gate和ControllerManager，按真实控制拍顺序运行并正常join退出。
 * 20ms发现回调在非控制executor拿manager列表锁。控制线程按manager更新率用宿主steady sleep_until；每拍先刷新授权，再read/update/write，最后cycle冻结末采样/验收。正常shutdown后join控制线程再销毁gate/manager；控制与ROS时间域分别使用。
 * @param argc 进程参数数目。
 * @param argv ROS/硬件/控制器参数，转传manager选项。
 * @return 正常0，捕获初始化/运行异常返回1；不在此宣称设备释放已确认。
 */
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try {
    auto executor=std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto gate=std::make_shared<dummy_motion_client::DeviceActionGate>();
    auto manager_options=controller_manager::get_cm_node_options();
    manager_options.arguments(std::vector<std::string>(argv+1,argv+argc));
    auto manager=std::make_shared<controller_manager::ControllerManager>(executor,"controller_manager","",manager_options);
    executor->add_node(manager); executor->add_node(gate);
    // Controller list discovery takes the manager's non-RT lock. Keep it off
    // the update thread: loading waits for that thread to adopt the new list.
    /**
     * @brief 发现Controller的manager非RT锁只在executor回调中取得，避免加载/切换锁阻塞控制更新；active插件再原子交给control adopt。
     */
    auto discovery=gate->create_wall_timer(std::chrono::milliseconds(20),[&]{gate->attach(*manager);});
    /**
     * @brief 该独立线程为大多数gate状态唯一writer，不由MultiThreadedExecutor并发修改；回调交接通过原子/有限槽/准入锁。period取manager Hz，sleep_until使用宿主steady。
     */
    std::thread control([&] {
      const auto period=std::chrono::nanoseconds(1000000000ULL/std::max(1U,manager->get_update_rate()));
      auto next=std::chrono::steady_clock::now();
      while(rclcpp::ok()) {
        next+=period;
        // Refresh authority before producing a sample. Once the post-update
        // terminal check freezes that sample, a later ticket cannot retire it.
        /**
         * @brief 先接纳新鲜身份/ticket再生产本拍真实目标；后update cycle一旦冻结terminal样本，本拍之后的票据刷新不应退役该冻结代。
         */
        gate->prepare_update();
        if(manager->is_resource_manager_initialized()) {
          const auto time=manager->now(); const rclcpp::Duration duration(period);
          /**
           * @brief ROS time只供ControllerManager/JTC插值和消息时间；传输新鲜度、管理期限和证据监督仍用host steady µs，不跨域直接相减。
           */
          manager->read(time,duration); manager->update(time,duration); manager->write(time,duration);
        }
        gate->cycle();
        std::this_thread::sleep_until(next);
      }
    });
    /**
     * @brief 正常退出先停executor/ROS并join控制线程，再释放发现timer和Node/manager；节点析构清本地生产，实际Stop是否已确认必须看协议诊断/证据。
     */
    executor->spin(); rclcpp::shutdown(); control.join();
    discovery.reset();
    executor->remove_node(gate); gate.reset(); manager.reset();
  } catch(const std::exception &error) {
    fprintf(stderr,"device_action_gate: %s\n",error.what()); rclcpp::shutdown(); return 1;
  }
  return 0;
}
#endif

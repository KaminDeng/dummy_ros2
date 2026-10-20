/**
 * @file bootstrap_coordinator.cpp
 * @brief 实现设备模式/会话/参考/permit的串行bootstrap及有界续租/释放监督。
 * 管理成功不是实际状态已达到：自动下一步还要snapshot管理水位和阶段前置。Ready只说明bootstrap链，不隐式PrepareHold/Begin。所有now和deadline为宿主steady µs；snapshot新鲜度/源设备时钟由外层transport保证，本文件不直接读USB。
 */
#include "dummy_ros_control/diagnostic_scope.hpp"
#include "dummy_motion_client/bootstrap_coordinator.hpp"

#include <algorithm>
#include <cstdio>

namespace dummy_motion_client {
namespace {
// Preserve the 5 s confirmation reserve for both Hold and long RUN leases.
/**
 * @brief 续租lead为25秒、确认reserve为5秒；这些是宿主保守监督边界，不改变MCU会话或motion permit的公开规则。
 */
constexpr std::uint64_t kRenewLeadUs=25000000U;
constexpr std::uint64_t kRenewReserveUs=5000000U;
/**
 * @brief 比较bootstrap持续绑定的设备启动/profile与四类产物hash。
 * @param left 先前或候选快照。
 * @param right 另一份快照。
 * @return bool；不在此比较session/reference/motion/mask，调用阶段再核这些变化。
 */
bool same_binding(const BootstrapSnapshot &left, const BootstrapSnapshot &right) noexcept {
  return left.boot_epoch == right.boot_epoch && left.profile_id == right.profile_id &&
    left.firmware_sha256 == right.firmware_sha256 &&
    left.contract_sha256 == right.contract_sha256 &&
    left.model_sha256 == right.model_sha256 &&
    left.profile_sha256 == right.profile_sha256;
}
}  // namespace
/**
 * @brief 借用client并冻结会话模板、策略hash与预算。
 * 构造不发送管理请求；session租期与动作permit是两个预算，不可互相替代。
 * @param client 必须活过协调器的管理client。
 * @param identity 设备/客户端/期待session模板值。
 * @param compiled_policy_digest 32字节编译策略身份。
 * @param permit_remaining_us 本次将请求的motion permit时长µs。
 * @param configured_session_lease_us 可选固定session租期µs，0关闭保守自动续租时界。
 */
BootstrapCoordinator::BootstrapCoordinator(
    ManagementClient &client, ManageRequest identity,
    std::array<std::uint8_t, 32> compiled_policy_digest,
    std::uint64_t permit_remaining_us,
    std::uint64_t configured_session_lease_us)
    : client_(client), identity_(identity),
      compiled_policy_digest_(compiled_policy_digest),
      permit_remaining_us_(permit_remaining_us),
      configured_session_lease_us_(configured_session_lease_us) {}

/**
 * @brief 按最新快照填完整管理请求并记录提交水位。
 * SelectMode期望无session并请求position模式；RenewLease填requested_lease；SetPermit走专用submit_permit并绑定实际快照四hash。Grant/Renew记lease_submit锚，其他操作不重启租期。
 * @param operation 本次操作枚举。
 * @param now_us 原始宿主提交µs。
 * @return 非零request提交成功为true；0转Fault/BOOTSTRAP_TRANSPORT。
 */
bool BootstrapCoordinator::issue(ManageOperation operation, std::uint64_t now_us) {
  auto request = identity_;
  request.set_operation(wire_value(operation));
  request.set_expected_boot_epoch(snapshot_.boot_epoch);
  request.set_expected_session_id(
      operation == ManageOperation::kSelectMode ? 0U : identity_.expected_session_id());
  request.set_expected_reference_epoch(snapshot_.reference_epoch);
  request.set_expected_motion_epoch(snapshot_.motion_epoch);
  request.set_expected_management_epoch(snapshot_.management_epoch);
  request.set_expected_profile_id(snapshot_.profile_id);
  request.set_freshness_ticket(snapshot_.freshness_ticket);
  request.set_configured_mask(snapshot_.configured_mask);
  if (operation == ManageOperation::kSelectMode) request.set_requested_control_mode(1U);
  if (operation == ManageOperation::kRenewLease)
    request.set_requested_session_lease_us(requested_lease_us_);
  /**
   * @brief SetPermit请求策略hash/预算/操作员确认只构造模板；真正permit摘要由ManagementClient按可信binding和真实序号生成。
   */
  if (operation == ManageOperation::kSetPermit) {
    std::uint8_t digest[32]{};
    std::copy(compiled_policy_digest_.begin(), compiled_policy_digest_.end(), digest);
    request.set_compiled_policy_sha256(digest);
    request.set_permit_remaining_us(permit_remaining_us_);
    request.set_operator_confirmed(true);
  }
  issued_management_epoch_ = snapshot_.management_epoch;
  if (operation == ManageOperation::kSetPermit) {
    /**
     * @brief PermitBinding取当前实际boot/reference/profile和四类观测hash，不从请求自己声称的permit摘要获得授权。
     */
    const PermitBinding binding{snapshot_.boot_epoch, snapshot_.reference_epoch, snapshot_.profile_id,
      snapshot_.firmware_sha256, snapshot_.contract_sha256, snapshot_.model_sha256,
      snapshot_.profile_sha256};
    request_id_ = client_.submit_permit(request, binding, now_us);
  } else request_id_ = client_.submit(request, now_us);
  if (request_id_ == 0U) {
    phase_ = BootstrapPhase::kFault;
    error_ = "BOOTSTRAP_TRANSPORT";
    return false;
  }
  if (operation == ManageOperation::kGrantSession || operation == ManageOperation::kRenewLease)
    lease_submit_at_us_=now_us;
  return true;
}

/**
 * @brief 从可信只读快照请求整条自动bootstrap。
 * automatic=true后成功链为模式→会话→捕获参考→permit，之后Ready；不会自动使能或执行轨迹。
 * @param snapshot 外层已判定raw_ready和motion_supported的非零身份/票据快照。
 * @param now_us 宿主µs。
 * @return bool；仅Idle/OperationComplete且session模板/permit/策略有效时提交SelectMode。
 */
bool BootstrapCoordinator::start(const BootstrapSnapshot &snapshot, std::uint64_t now_us) {
  if ((phase_ != BootstrapPhase::kIdle && phase_ != BootstrapPhase::kOperationComplete) || snapshot.boot_epoch == 0U ||
      snapshot.profile_id == 0U || snapshot.configured_mask == 0U ||
      snapshot.freshness_ticket == 0U || !snapshot.raw_ready ||
      !snapshot.motion_supported || identity_.expected_session_id() == 0U ||
      permit_remaining_us_ == 0U ||
      std::all_of(compiled_policy_digest_.begin(), compiled_policy_digest_.end(), [](auto value) { return value == 0U; }))
    return false;
  snapshot_ = snapshot;
  confirmed_release_.reset();
  automatic_ = true;
  error_.clear();
  phase_ = BootstrapPhase::kSelectMode;
  return issue(ManageOperation::kSelectMode, now_us);
}

/**
 * @brief 显式提交允许的单步，不隐式推进后续动作。
 * 从Ready只能SetPermit；捕获需本session，permit还需非零参考；非Idle时同binding不得漂移。Select/Grant清旧释放凭据，其他操作不假装新设备。
 * @param snapshot 符合当前绑定/操作前置的快照。
 * @param operation 仅SelectMode/GrantSession/CaptureReference/SetPermit。
 * @param now_us 宿主µs。
 * @return bool；受理后等待结果，成功停OperationComplete。
 */
bool BootstrapCoordinator::start_operation(
    const BootstrapSnapshot &snapshot, ManageOperation operation, std::uint64_t now_us) {
  if ((phase_ != BootstrapPhase::kIdle && phase_ != BootstrapPhase::kOperationComplete &&
       !(phase_ == BootstrapPhase::kReady && operation == ManageOperation::kSetPermit)) ||
      snapshot.boot_epoch == 0U || snapshot.profile_id == 0U ||
      snapshot.configured_mask == 0U || snapshot.freshness_ticket == 0U ||
      !snapshot.raw_ready || !snapshot.motion_supported ||
      (phase_ != BootstrapPhase::kIdle && !same_binding(snapshot, snapshot_)) ||
      (operation == ManageOperation::kCaptureReference &&
       snapshot.session_id != identity_.expected_session_id()) ||
      (operation == ManageOperation::kSetPermit &&
       (snapshot.session_id != identity_.expected_session_id() || snapshot.reference_epoch == 0U))) return false;
  switch (operation) {
    case ManageOperation::kSelectMode: phase_ = BootstrapPhase::kSelectMode; break;
    case ManageOperation::kGrantSession: phase_ = BootstrapPhase::kGrantSession; break;
    case ManageOperation::kCaptureReference: phase_ = BootstrapPhase::kCaptureReference; break;
    case ManageOperation::kSetPermit: phase_ = BootstrapPhase::kSetPermit; break;
    default: return false;
  }
  snapshot_ = snapshot;
  if (operation == ManageOperation::kSelectMode || operation == ManageOperation::kGrantSession)
    confirmed_release_.reset();
  /**
   * @brief 显式操作成功后不继续自动链；调用者必须另行请求下一步，避免一次管理按钮变成完整采零/授权动作。
   */
  automatic_ = false;
  last_operation_ = operation;
  error_.clear();
  return issue(operation, now_us);
}

/**
 * @brief 在同设备身份和原RUN上显式请求有限session续租。
 * 保存run/motion/reference防回复期间换目标；不重新捕获参考、不修改motion permit或命令producer。配置租期为0时允许显式续租但不应用renewal_due自动窗口。
 * @param snapshot 当前可信快照，Running需双消息Running且run非零。
 * @param requested_lease_us 非零、最多1小时；配置监督存在时还必须renewal_due。
 * @param now_us 宿主µs。
 * @return bool；只提交RenewLease，不声明已延长。
 */
bool BootstrapCoordinator::start_renew_lease(
    const BootstrapSnapshot &snapshot, std::uint64_t requested_lease_us,
    std::uint64_t now_us) {
  if ((phase_ != BootstrapPhase::kReady && phase_ != BootstrapPhase::kOperationComplete) ||
      requested_lease_us == 0U || requested_lease_us > 3600000000ULL ||
      (configured_session_lease_us_ != 0U && !renewal_due(now_us,requested_lease_us)) ||
      snapshot.boot_epoch == 0U || snapshot.session_id != identity_.expected_session_id() ||
      snapshot.reference_epoch == 0U || snapshot.profile_id == 0U ||
      snapshot.configured_mask == 0U || snapshot.freshness_ticket == 0U ||
      !same_binding(snapshot, snapshot_) ||
      (snapshot.core_state == wire_value(WireCoreState::kRunning) &&
       (snapshot.state_core_state != wire_value(WireCoreState::kRunning) ||
        snapshot.run_id == 0U)) ||
      !snapshot.raw_ready || !snapshot.motion_supported) return false;
  snapshot_ = snapshot;
  /**
   * @brief 仅Running续租冻结run；Hold续租run取0。后续observe对Running这组三水位和双core要求保持不变。
   */
  renewing_run_id_ = snapshot.core_state == wire_value(WireCoreState::kRunning)
      ? snapshot.run_id : 0U;
  renewing_motion_epoch_ = snapshot.motion_epoch;
  renewing_reference_epoch_ = snapshot.reference_epoch;
  requested_lease_us_ = requested_lease_us;
  automatic_ = false;
  last_operation_ = ManageOperation::kRenewLease;
  error_.clear();
  phase_ = BootstrapPhase::kRenewLease;
  return issue(ManageOperation::kRenewLease, now_us);
}

/**
 * @brief 以提交锚保证期限计算续租窗口及是否真正延长。
 * 保守截止为原submit+term；不能将晚收到COMMITTED的宿主时刻当作设备重新签发点。所有溢出显式拒绝。
 * @param now_us 同一宿主steady µs。
 * @param requested_lease_us 拟请求租期，需>25秒且≤配置值。
 * @return bool：进入保证期限前25秒lead、未进最后5秒reserve且新请求能超出上次确认界。
 */
bool BootstrapCoordinator::renewal_due(std::uint64_t now_us,
                                      std::uint64_t requested_lease_us) const noexcept {
  if (configured_session_lease_us_ == 0U ||
      (phase_ != BootstrapPhase::kReady && phase_ != BootstrapPhase::kOperationComplete) ||
      lease_term_us_ == 0U || lease_guaranteed_until_us_ <= kRenewReserveUs ||
      requested_lease_us <= kRenewLeadUs ||
      requested_lease_us > configured_session_lease_us_ ||
      now_us > UINT64_MAX-requested_lease_us ||
      lease_confirmed_at_us_ > UINT64_MAX-lease_term_us_) return false;
  /**
   * @brief due_at与safe_cutoff都是原保证截止的减法边界；now必须在[lead起点,reserve起点)内，抵达reserve后不继续冒险续。
   */
  const auto safe_cutoff=lease_guaranteed_until_us_-kRenewReserveUs;
  const auto due_at=lease_guaranteed_until_us_ > kRenewLeadUs
      ? lease_guaranteed_until_us_-kRenewLeadUs : 0U;
  return now_us >= due_at && now_us < safe_cutoff &&
      now_us+requested_lease_us > lease_confirmed_at_us_+lease_term_us_;
}

/**
 * @brief 从同身份双Hold快照发起固定5秒释放事务。
 * 不在任意运动态隐式执行此分支；外层应先完成必要Cancel/保持。管理成功还须release_idle实际观测，不能以请求发送代替释放。
 * @param snapshot 本session、票据有效、raw/motion支持且Status/State均Hold。
 * @param now_us 宿主µs，能安全加5秒。
 * @return bool；建立ReleasePending和原deadline后提交D10。
 */
bool BootstrapCoordinator::start_release(const BootstrapSnapshot &snapshot,
                                         std::uint64_t now_us) {
  if ((phase_ != BootstrapPhase::kIdle && phase_ != BootstrapPhase::kReady &&
       phase_ != BootstrapPhase::kOperationComplete) ||
      (phase_ != BootstrapPhase::kIdle && !same_binding(snapshot, snapshot_)) ||
      snapshot.boot_epoch == 0U || snapshot.session_id != identity_.expected_session_id() ||
      snapshot.profile_id == 0U || snapshot.configured_mask == 0U ||
      snapshot.freshness_ticket == 0U || !snapshot.raw_ready || !snapshot.motion_supported ||
      snapshot.core_state != wire_value(WireCoreState::kHold) ||
      snapshot.state_core_state != wire_value(WireCoreState::kHold) ||
      now_us > UINT64_MAX - 5000000U) return false;
  snapshot_ = snapshot;
  confirmed_release_.reset();
  automatic_ = false;
  last_operation_ = ManageOperation::kDisableAndRelease;
  error_.clear();
  /**
   * @brief 本次释放总deadline固定在这里；Query链、早状态等待和管理结果都共享，后续不重启5秒。
   */
  release_deadline_us_ = now_us + 5000000U;
  phase_ = BootstrapPhase::kReleasePending;
  return issue(ManageOperation::kDisableAndRelease, now_us);
}

/**
 * @brief 检查快照的无owner/reference/permit双Idle形状。
 * 本辅助不自行检查接收年龄、request/operation或release_committed；调用位置已经/仍须管理成功证据，外层负责新鲜一致流。
 * @return bool；两core都UnreferencedIdle且实际许可为false/剩余0。
 */
bool BootstrapCoordinator::release_idle() const noexcept {
  return snapshot_.session_id == 0U && snapshot_.reference_epoch == 0U &&
      snapshot_.core_state == wire_value(WireCoreState::kUnreferencedIdle) &&
      snapshot_.state_core_state == wire_value(WireCoreState::kUnreferencedIdle) &&
      !snapshot_.real_motion_permitted && snapshot_.permit_remaining_us == 0U;
}

/**
 * @brief 在序列化控制上下文更新快照并监督绑定/释放变化。
 * Idle只存观察；释放到期、Running续租run/epoch/core漂移或设备hash/boot/session/mask错误转Fault。释放允许owner/reference明确归零；Released后重新出现owner/permit也失效。ReleaseAwaitIdle仅双无参考Idle才转Released。
 * @param snapshot 外层完成decode/身份与时间校验的合成State/Status值。
 * @param now_us 宿主µs。
 */
void BootstrapCoordinator::observe(const BootstrapSnapshot &snapshot, std::uint64_t now_us) noexcept {
  if (phase_ == BootstrapPhase::kIdle) {
    snapshot_ = snapshot;
    return;
  }
  if ((phase_ == BootstrapPhase::kReleasePending ||
       phase_ == BootstrapPhase::kReleaseAwaitIdle) && now_us >= release_deadline_us_) {
    if(auto* journal=dummy_ros_control::current_host_diagnostics) {
      const dummy_ros_control::DiagnosticIdentity observed{snapshot_.boot_epoch,snapshot_.session_id,snapshot_.reference_epoch,snapshot_.motion_epoch,0U,snapshot_.management_epoch,snapshot_.boot_epoch!=0U};
      const dummy_ros_control::DiagnosticIdentity expected{snapshot_.boot_epoch,identity_.expected_session_id(),snapshot_.reference_epoch,snapshot_.motion_epoch,0U,snapshot_.management_epoch,snapshot_.boot_epoch!=0U};
      journal->reject(dummy_ros_control::DiagnosticReason::kBootstrapReleaseTimeout,observed,expected,now_us,"RELEASE_TIMEOUT_DIAG",0U,{static_cast<unsigned>(phase_),request_id_,snapshot_.session_id,snapshot_.reference_epoch,snapshot_.core_state,snapshot_.state_core_state,snapshot_.permit_remaining_us,release_idle()?1U:0U,now_us-release_deadline_us_});
      journal->detail(
        "RELEASE_TIMEOUT_DIAG phase=%u request=%llu session=%llu ref=%u core=%u state_core=%u permit_us=%llu idle=%u overdue_us=%llu\n",
        static_cast<unsigned>(phase_), static_cast<unsigned long long>(request_id_),
        static_cast<unsigned long long>(snapshot_.session_id), snapshot_.reference_epoch,
        snapshot_.core_state, snapshot_.state_core_state,
        static_cast<unsigned long long>(snapshot_.permit_remaining_us),
        release_idle() ? 1U : 0U,
        static_cast<unsigned long long>(now_us-release_deadline_us_));
    } else std::fprintf(stderr,
        "RELEASE_TIMEOUT_DIAG phase=%u request=%llu session=%llu ref=%u core=%u state_core=%u permit_us=%llu idle=%u overdue_us=%llu\n",
        static_cast<unsigned>(phase_), static_cast<unsigned long long>(request_id_),
        static_cast<unsigned long long>(snapshot_.session_id), snapshot_.reference_epoch,
        snapshot_.core_state, snapshot_.state_core_state,
        static_cast<unsigned long long>(snapshot_.permit_remaining_us),
        release_idle() ? 1U : 0U,
        static_cast<unsigned long long>(now_us-release_deadline_us_));
    phase_ = BootstrapPhase::kFault;
    error_ = "RELEASE_UNCONFIRMED";
    confirmed_release_.reset();
    request_id_ = 0U;
    return;
  }
  /**
   * @brief 释放分支允许已确认根的session/reference归零，普通bootstrap不允许无关session切换；这只是过渡形状例外，不免除同binding/mask检查。
   */
  const bool releasing = phase_ == BootstrapPhase::kReleasePending ||
      phase_ == BootstrapPhase::kReleaseAwaitIdle || phase_ == BootstrapPhase::kReleased;
  /**
   * @brief RUN续租期间即使管理reply尚Pending，也不能借此忽略run/参考/motion变化或双消息不再Running；立即停止本协调器等待并报绑定变化。
   */
  if (phase_ == BootstrapPhase::kRenewLease && renewing_run_id_ != 0U &&
      (snapshot.run_id != renewing_run_id_ ||
       snapshot.motion_epoch != renewing_motion_epoch_ ||
       snapshot.reference_epoch != renewing_reference_epoch_ ||
       snapshot.core_state != wire_value(WireCoreState::kRunning) ||
       snapshot.state_core_state != wire_value(WireCoreState::kRunning))) {
    phase_ = BootstrapPhase::kFault;
    error_ = "RUN_LEASE_BINDING_CHANGED";
    request_id_ = 0U;
    return;
  }
  /**
   * @brief 基础hash/boot/profile绑定不变后再核session、配置mask及释放形状；失败清释放凭据，不能继承旧成功证据到新设备。
   */
  if (!same_binding(snapshot, snapshot_) ||
      (snapshot_.session_id != 0U && snapshot.session_id != snapshot_.session_id &&
       !(releasing && snapshot.session_id == 0U)) ||
      snapshot.configured_mask != snapshot_.configured_mask ||
      (releasing && snapshot.reference_epoch != snapshot_.reference_epoch &&
       snapshot.reference_epoch != 0U) ||
      (phase_ == BootstrapPhase::kReleased &&
       (snapshot.session_id != 0U || snapshot.reference_epoch != 0U ||
        snapshot.core_state != wire_value(WireCoreState::kUnreferencedIdle) ||
        snapshot.state_core_state != wire_value(WireCoreState::kUnreferencedIdle) ||
        snapshot.real_motion_permitted || snapshot.permit_remaining_us != 0U))) {
    phase_ = BootstrapPhase::kFault;
    error_ = "IDENTITY_CHANGED";
    confirmed_release_.reset();
    request_id_ = 0U;
    return;
  }
  snapshot_ = snapshot;
  if (phase_ == BootstrapPhase::kReleaseAwaitIdle && release_idle())
    phase_ = BootstrapPhase::kReleased;
}

/**
 * @brief 由外部监督将活动bootstrap置Fault并撤销本地结果。
 * Idle/Fault无动作；清request/confirmed_release，但不自发D10，设备停机仍由外层生命周期负责。
 * @param reason 非空C文本，复制到error_。
 */
void BootstrapCoordinator::invalidate(const char *reason) noexcept {
  if (phase_ == BootstrapPhase::kIdle || phase_ == BootstrapPhase::kFault) return;
  phase_ = BootstrapPhase::kFault;
  error_ = reason;
  confirmed_release_.reset();
  request_id_ = 0U;
}

/**
 * @brief 要求管理epoch严格高于本次提交水位并核阶段实际前置。
 * 检查的是snapshot实际反映的状态；仅client返回COMMITTED而snapshot未赶上时返回false，留待后续observe/tick。
 * @return bool；Select/Grant核模式与session，Capture核非零参考，Permit还核实际许可剩余。
 */
bool BootstrapCoordinator::precondition_for_next() const noexcept {
  if (snapshot_.management_epoch <= issued_management_epoch_) return false;
  switch (phase_) {
    case BootstrapPhase::kSelectMode:
      return snapshot_.session_id == 0U && snapshot_.control_mode != 0U;
    case BootstrapPhase::kCaptureReference:
      return snapshot_.session_id == identity_.expected_session_id() &&
          snapshot_.reference_epoch != 0U && snapshot_.reference_kind != 0U;
    case BootstrapPhase::kGrantSession:
      return snapshot_.session_id == identity_.expected_session_id() && snapshot_.control_mode != 0U;
    case BootstrapPhase::kSetPermit:
      return snapshot_.session_id == identity_.expected_session_id() && snapshot_.control_mode != 0U &&
          snapshot_.reference_epoch != 0U && snapshot_.reference_kind != 0U &&
          snapshot_.real_motion_permitted && snapshot_.permit_remaining_us != 0U;
    default: return false;
  }
}

/**
 * @brief 在实际阶段前置通过后提交自动下一步或标Ready。
 * 模式→会话→参考→permit→Ready，不包含Enable/Begin；失败issue可直接Fault，不能跳过中间证据。
 * @param now_us 宿主µs，传给下一管理根。
 */
void BootstrapCoordinator::advance(std::uint64_t now_us) {
  if (!precondition_for_next()) return;
  if (phase_ == BootstrapPhase::kSelectMode) {
    phase_ = BootstrapPhase::kGrantSession;
    issue(ManageOperation::kGrantSession, now_us);
  } else if (phase_ == BootstrapPhase::kGrantSession) {
    phase_ = BootstrapPhase::kCaptureReference;
    issue(ManageOperation::kCaptureReference, now_us);
  } else if (phase_ == BootstrapPhase::kCaptureReference) {
    phase_ = BootstrapPhase::kSetPermit;
    issue(ManageOperation::kSetPermit, now_us);
  } else if (phase_ == BootstrapPhase::kSetPermit) {
    phase_ = BootstrapPhase::kReady;
  }
}

/**
 * @brief 轮询管理结果并维护原租期/释放截止及自动推进。
 * 先监督保守租期最后5秒，再处理phase/释放5秒期限。管理Failed/异常转Fault；Grant/Renew需可信COMMITTED/opID，按原submit+term更新保证寿命。D10管理成功先留ConfirmedRelease，再等Idle而非直接Released。
 * @param now_us 宿主steady µs。
 */
void BootstrapCoordinator::tick(std::uint64_t now_us) {
  /**
   * @brief 即使phase已Ready，进入保证租期的最后5秒仍会先Fault；不能因Ready分支提早return而漏掉续租未确认监督。
   */
  if (lease_guaranteed_until_us_ > kRenewReserveUs &&
      now_us >= lease_guaranteed_until_us_-kRenewReserveUs &&
      phase_ != BootstrapPhase::kIdle && phase_ != BootstrapPhase::kFault &&
      phase_ != BootstrapPhase::kReleasePending &&
      phase_ != BootstrapPhase::kReleaseAwaitIdle && phase_ != BootstrapPhase::kReleased) {
    phase_ = BootstrapPhase::kFault;
    error_ = "LEASE_RENEWAL_UNCONFIRMED";
    request_id_ = 0U;
    return;
  }
  if (phase_ == BootstrapPhase::kIdle || phase_ == BootstrapPhase::kReady ||
      phase_ == BootstrapPhase::kReleased || phase_ == BootstrapPhase::kFault) return;
  if ((phase_ == BootstrapPhase::kReleasePending ||
       phase_ == BootstrapPhase::kReleaseAwaitIdle) && now_us >= release_deadline_us_) {
    phase_ = BootstrapPhase::kFault;
    error_ = "RELEASE_UNCONFIRMED";
    confirmed_release_.reset();
    request_id_ = 0U;
    return;
  }
  if (phase_ == BootstrapPhase::kReleaseAwaitIdle) return;
  /**
   * @brief 上一管理成功已清request但实际快照可能尚未满足下一步；继续advance检查原issued水位，不凭空再提交同一mutation。
   */
  if (request_id_ == 0U) {
    advance(now_us);
    return;
  }
  try {
    const auto reply = client_.poll(request_id_);
    if (!reply) return;
    if (phase_ == BootstrapPhase::kReleasePending &&
        (reply->result_valid || reply->rejected))
      {
if(auto* journal=dummy_ros_control::current_host_diagnostics) {
      const dummy_ros_control::DiagnosticIdentity observed{snapshot_.boot_epoch,snapshot_.session_id,snapshot_.reference_epoch,snapshot_.motion_epoch,0U,snapshot_.management_epoch,snapshot_.boot_epoch!=0U};
      const dummy_ros_control::DiagnosticIdentity expected{snapshot_.boot_epoch,identity_.expected_session_id(),snapshot_.reference_epoch,snapshot_.motion_epoch,0U,snapshot_.management_epoch,snapshot_.boot_epoch!=0U};
      journal->reject(dummy_ros_control::DiagnosticReason::kBootstrapReleaseReply,observed,expected,now_us,"RELEASE_REPLY_DIAG",0U,{request_id_,reply->request_id,reply->transaction_phase,reply->result_code,reply->result_valid?1U:0U,reply->succeeded?1U:0U,reply->rejected?1U:0U,reply->release_committed?1U:0U,reply->operation_id},!reply->succeeded || reply->rejected);
      journal->detail(
          "RELEASE_REPLY_DIAG request=%llu response=%llu phase=%u result=%u valid=%u success=%u rejected=%u release_committed=%u opid=%llu\n",
          static_cast<unsigned long long>(request_id_),
          static_cast<unsigned long long>(reply->request_id),
          reply->transaction_phase, reply->result_code,
          reply->result_valid ? 1U : 0U, reply->succeeded ? 1U : 0U,
          reply->rejected ? 1U : 0U, reply->release_committed ? 1U : 0U,
          static_cast<unsigned long long>(reply->operation_id));
    } else std::fprintf(stderr,
          "RELEASE_REPLY_DIAG request=%llu response=%llu phase=%u result=%u valid=%u success=%u rejected=%u release_committed=%u opid=%llu\n",
          static_cast<unsigned long long>(request_id_),
          static_cast<unsigned long long>(reply->request_id),
          reply->transaction_phase, reply->result_code,
          reply->result_valid ? 1U : 0U, reply->succeeded ? 1U : 0U,
          reply->rejected ? 1U : 0U, reply->release_committed ? 1U : 0U,
          static_cast<unsigned long long>(reply->operation_id));
    }
    /**
     * @brief 只有配置过session租期的Grant/Renew启用该严格期限确认；响应还须回显本根ID，不能误用其他操作的Ok续命。
     */
    const bool lease_operation=configured_session_lease_us_ != 0U &&
        (phase_ == BootstrapPhase::kGrantSession || phase_ == BootstrapPhase::kRenewLease);
    if (lease_operation && reply->request_id != request_id_) {
      phase_ = BootstrapPhase::kFault;
      error_ = "LEASE_RENEWAL_UNCONFIRMED";
      request_id_ = 0U;
      return;
    }
    /**
     * @brief 释放管理成功还必须明确phaseCommitted、codeOk、release_committed和非零operation；缺任一项为RELEASE_EVIDENCE_MISSING，不拿泛化succeeded布尔值验收。
     */
    if (phase_ == BootstrapPhase::kReleasePending && reply->result_valid && reply->succeeded &&
        (reply->request_id != request_id_ ||
         reply->transaction_phase != wire_value(WireTransactionPhase::kCommitted) ||
         reply->result_code != wire_value(WireResult::kOk) ||
         !reply->release_committed || reply->operation_id == 0U)) {
      phase_ = BootstrapPhase::kFault;
      error_ = "RELEASE_EVIDENCE_MISSING";
      confirmed_release_.reset();
      request_id_ = 0U;
      return;
    }
    if (reply->rejected || (reply->result_valid && !reply->succeeded)) {
      const auto failed_phase = phase_;
      phase_ = BootstrapPhase::kFault;
      confirmed_release_.reset();
      error_ = "BOOTSTRAP_REJECTED phase=" +
          std::to_string(static_cast<unsigned>(failed_phase)) +
          " result=" + std::to_string(reply->result_code) +
          " reason=" + std::to_string(reply->first_failure_reason);
      request_id_ = 0U;
      return;
    }
    if (reply->result_valid && reply->succeeded) {
      if (lease_operation &&
          (reply->transaction_phase != wire_value(WireTransactionPhase::kCommitted) ||
           reply->operation_id == 0U ||
           reply->result_code != wire_value(WireResult::kOk))) {
        phase_ = BootstrapPhase::kFault;
        error_ = "LEASE_RENEWAL_UNCONFIRMED";
        request_id_ = 0U;
        return;
      }
      if (configured_session_lease_us_ != 0U &&
          (phase_ == BootstrapPhase::kGrantSession || phase_ == BootstrapPhase::kRenewLease)) {
        lease_term_us_=phase_ == BootstrapPhase::kGrantSession
            ? configured_session_lease_us_ : requested_lease_us_;
        lease_confirmed_at_us_=now_us;
        if (lease_submit_at_us_ > UINT64_MAX-lease_term_us_) {
          phase_ = BootstrapPhase::kFault;
          error_ = "LEASE_DEADLINE_OVERFLOW";
          request_id_ = 0U;
          return;
        }
        /**
         * @brief 保证截止从最早submit时刻起算，回复等待耗时不会变成额外设备租期；另存confirmed_at仅用于判断下一续租是否实际延长。
         */
        lease_guaranteed_until_us_=lease_submit_at_us_+lease_term_us_;
      }
      /**
       * @brief 这里仅保存管理层request/operation对；实际双Idle未到则进入ReleaseAwaitIdle且继续用原总deadline。
       */
      if (phase_ == BootstrapPhase::kReleasePending) {
        confirmed_release_ = ConfirmedRelease{request_id_, reply->operation_id};
        request_id_ = 0U;
        phase_ = release_idle() ? BootstrapPhase::kReleased : BootstrapPhase::kReleaseAwaitIdle;
        return;
      }
      request_id_ = 0U;
      /**
       * @brief 单步调用在OperationComplete停住；自动链才advance，且仍需snapshot前置水位。
       */
      if (!automatic_) {
        phase_ = BootstrapPhase::kOperationComplete;
        return;
      }
      advance(now_us);
    }
  } catch (const std::exception &error) {
    const auto failed_phase=phase_;
    phase_ = BootstrapPhase::kFault;
    confirmed_release_.reset();
    error_ = std::string("BOOTSTRAP_UNCONFIRMED phase=")+
      std::to_string(static_cast<unsigned>(failed_phase))+" error="+error.what();
    request_id_ = 0U;
  }
}

/**
 * @brief 从当前可信快照导出ActionGate所需身份与三资格。
 * 带最新boot/epoch/ticket/profile/mask但不发送Enable；ready getter本身不保证仍能导出授权，实际运动还需ActionGate和worker。
 * @return optional Authorization；只在Ready或SetPermit单步成功、同session/模式/参考/实际permit仍有效时有值。
 */
std::optional<Authorization> BootstrapCoordinator::authorization() const {
  if (!ready() && !(operation_complete() && last_operation_ == ManageOperation::kSetPermit)) return {};
  if (snapshot_.session_id != identity_.expected_session_id() || snapshot_.control_mode == 0U ||
      snapshot_.reference_epoch == 0U || snapshot_.reference_kind == 0U ||
      !snapshot_.real_motion_permitted || snapshot_.permit_remaining_us == 0U) return {};
  auto request = identity_;
  request.set_expected_boot_epoch(snapshot_.boot_epoch);
  request.set_expected_reference_epoch(snapshot_.reference_epoch);
  request.set_expected_motion_epoch(snapshot_.motion_epoch);
  request.set_expected_management_epoch(snapshot_.management_epoch);
  request.set_expected_profile_id(snapshot_.profile_id);
  request.set_freshness_ticket(snapshot_.freshness_ticket);
  request.set_configured_mask(snapshot_.configured_mask);
  return Authorization{request, true, true, true};
}
}  // namespace dummy_motion_client

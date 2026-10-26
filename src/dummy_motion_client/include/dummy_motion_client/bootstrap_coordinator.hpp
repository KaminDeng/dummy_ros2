/**
 * @file bootstrap_coordinator.hpp
 * @brief 声明设备模式、会话、参考、许可及释放的串行bootstrap协调器。
 * 自动模式按SelectMode→GrantSession→CaptureReference→SetPermit推进；每步须管理成功且观察水位满足，Ready仅表示bootstrap链完成，电机PrepareHold/Begin仍由ActionGate发起。续租只扩session期限，不重建motion permit；释放还需双来源无owner Idle。
 */
#pragma once

#include <array>
#include <cstdint>
#include <optional>

#include "dummy_motion_client/action_gate.hpp"

namespace dummy_motion_client {
/**
 * @brief 宿主bootstrap阶段，不是设备core_state。OperationComplete是单步结束；ReleasePending/ReleaseAwaitIdle/Released区分管理结果与实际释放观测。
 */
enum class BootstrapPhase {
  kIdle,
  kSelectMode,
  kGrantSession,
  kCaptureReference,
  kSetPermit,
  kManualOperation,
  kRenewLease,
  kOperationComplete,
  kReady,
  kFault,
  kReleasePending,
  kReleaseAwaitIdle,
  kReleased
};

/**
 * @brief 上层已解码/判新鲜的Status与State综合快照：ID/epoch无单位，票据为设备值，permit_remaining为µs。raw_ready/motion_supported由调用方提供，本结构不自行读取硬件。
 */
struct BootstrapSnapshot {
  std::uint64_t boot_epoch = 0;
  std::uint64_t session_id = 0;
  std::uint32_t reference_epoch = 0;
  std::uint32_t motion_epoch = 0;
  std::uint32_t management_epoch = 0;
  std::uint64_t freshness_ticket = 0;
  std::uint32_t profile_id = 0;
  std::uint8_t configured_mask = 0;
  bool raw_ready = false;
  bool motion_supported = false;
  std::uint16_t control_mode = 0;
  std::uint16_t reference_kind = 0;
  bool real_motion_permitted = false;
  std::uint64_t permit_remaining_us = 0;
  /**
   * @brief core_state来源Status，state_core_state来源State；释放/Running续租需两者对齐，不能只取一个消息替代另一个。
   */
  std::uint16_t core_state = 0;
  std::uint16_t state_core_state = 0;
  /**
   * @brief 四类产物hash绑定持续bootstrap身份；compiled policy另由协调器构造值固定。run_id供Running续租同尝试核对。
   */
  std::array<std::uint8_t, 32> firmware_sha256{};
  std::array<std::uint8_t, 32> contract_sha256{};
  std::array<std::uint8_t, 32> model_sha256{};
  std::array<std::uint8_t, 32> profile_sha256{};
  std::uint64_t run_id = 0;
};

/**
 * @brief 成功释放管理请求的request/operation ID对；需released阶段才能把Idle观测也视为完整通过，单有该值不必已到Idle。
 */
struct ConfirmedRelease {
  std::uint64_t request_id = 0;
  std::uint64_t operation_id = 0;
};

/**
 * @brief 借用ManagementClient的非阻塞bootstrap/租期/释放状态机。
 * 调用方串行observe/tick；内部snapshot只是值缓存，不拥有设备或通信线程。所有now_*与lease/release截止采用同一宿主steady µs；源设备时间应先在transport层校验。
 */
class BootstrapCoordinator {
 public:
  /**
   * @brief 固定请求身份、策略摘要及本次许可/会话预算。
   * @param client 借用管理client，必须活过协调器。
   * @param identity 设备/客户端/会话身份模板值。
   * @param compiled_policy_digest 已绑定32字节编译策略hash。
   * @param permit_remaining_us 要请求的motion permit时长µs，不能为0才可自动start。
   * @param configured_session_lease_us 可选已配置会话租期µs；0关闭本地自动续租时界监督。
   */
  BootstrapCoordinator(ManagementClient &client, ManageRequest identity,
                       std::array<std::uint8_t, 32> compiled_policy_digest,
                       std::uint64_t permit_remaining_us,
                       std::uint64_t configured_session_lease_us = 0U);
  /**
   * @brief 由只读快照启动完整自动bootstrap链。
   * 仅Idle或OperationComplete可启动，要求本地session ID及非零permit/策略hash；清旧释放凭据，automatic=true。
   * @param snapshot 非零boot/profile/mask/ticket、raw_ready且motion_supported的可信快照。
   * @param now_us 宿主steady µs。
   * @return bool：SelectMode请求本地受理；不是会话/参考/使能已完成。
   */
  bool start(const BootstrapSnapshot &snapshot, std::uint64_t now_us);
  /**
   * @brief 显式启动允许的单步管理操作。
   * Capture需本session，Permit需本session+参考；从Ready只允许SetPermit刷新。方法不触发后续自动捕获，避免单步请求变成整条隐式动作链。
   * @param snapshot 同绑定且满足该操作前置的可信快照。
   * @param operation 仅SelectMode/GrantSession/CaptureReference/SetPermit。
   * @param now_us 宿主µs。
   * @return bool；成功进入对应等待阶段，完成后停OperationComplete。
   */
  bool start_operation(const BootstrapSnapshot &snapshot,
                       ManageOperation operation, std::uint64_t now_us,
                       bool operator_confirmed = false,
                       bool strict_evidence = false);
  /**
   * @brief 在同设备/参考/尝试上显式续session租期。
   * 保存Running run/reference/motion以拒绝续租期间切换；本操作不重设参考、不发新轨迹、不刷新motion permit。
   * @param snapshot 当前可信状态；Running时State也必须Running且run非零。
   * @param requested_lease_us 非零且≤1小时，已配置监督时还须满足renewal_due。
   * @param now_us 宿主µs。
   * @return bool；提交后等待管理COMMITTED，未确认不可当租期已延长。
   */
  bool start_renew_lease(const BootstrapSnapshot &snapshot,
                         std::uint64_t requested_lease_us, std::uint64_t now_us);
  // Available only when a configured session lease was supplied; uses Host-side
  // submit/commit bounds without changing the permit or command producer.
  /**
   * @brief 按保守宿主提交锚判断是否应续租且仍有确认余量。
   * lease_guaranteed_until从submit+term计算，非从回复时间起算；防止网络耗时被错误算作额外设备寿命。溢出和不延长期限均拒绝。
   * @param now_us 宿主µs。
   * @param requested_lease_us 拟续期限，须大于25秒且不超过配置租期。
   * @return bool；配置非零、阶段合适、剩余进入25秒lead且未触5秒reserve，同时能实际扩展保守期限。
   */
  bool renewal_due(std::uint64_t now_us, std::uint64_t requested_lease_us) const noexcept;
  /**
   * @brief
   * 默认仅双Hold释放；显式manual模式另允许同参考双Idle或无参考双UnreferencedIdle。
   * 需Idle/Ready/OperationComplete且身份未漂移；不是一条立即释放成功的同步调用。
   * @param snapshot 本session的可信双Hold快照。
   * @param now_us 宿主µs，须可安全加5秒。
   * @param manual_reference_release
   * 允许尚未Enable的双Idle清理，默认false沿用bench模式。
   * @param expected_retained_reference
   * manual模式显式0表示未资格清参考，非零固定已资格参考；nullopt兼容保留释放前参考。
   * @return bool；成功建立固定5秒总释放截止并提交D10。
   */
  /** @brief
   * 用户manual任务仅签发一次Host绝对permit窗口；不因Renew/Complete重开。 */
  bool begin_manual_permit_window(std::uint64_t boot,
                                  std::uint64_t issued_at_us) noexcept;
  /** @brief 已扣原签发年龄的剩余预算；失效/释放/Fault/到期均为0。 */
  std::uint64_t manual_permit_remaining_us(std::uint64_t now_us) const noexcept;
  bool start_release(
      const BootstrapSnapshot &snapshot, std::uint64_t now_us,
      bool manual_reference_release = false,
      std::optional<std::uint32_t> expected_retained_reference = std::nullopt);
  /**
   * @brief 更新可信快照并监督身份、RUN续租和释放Idle。
   * 绑定hash/boot/profile、session及mask漂移转Fault；Release允许明确归零身份，Released后又出现owner/permit也失效。ReleaseAwaitIdle收到双无参考Idle才转Released；此层不自行计算State原始生产时钟。
   * @param snapshot 调用方完成解码/新鲜度检查的最新合成值。
   * @param now_us 宿主µs。
   */
  void observe(const BootstrapSnapshot &snapshot, std::uint64_t now_us) noexcept;
  /**
   * @brief 由外层撤销当前bootstrap并保留原因。
   * Idle/Fault无额外动作；清释放凭据和当前请求，但不自行发网络Stop，外层须处理设备停止/资源。
   * @param reason 非空且调用期有效C文本，复制到error字符串。
   */
  void invalidate(const char *reason) noexcept;
  /**
   * @brief 轮询管理结果、推进自动链并监督租期/释放总期限。
   * 进入保守租期末5秒仍未确认续租即Fault；释放总5秒不因Query延长。管理拒绝/异常记录BOOTSTRAP_UNCONFIRMED；成功Grant/Renew后按原submit+term更新保证截止，自动下一步还须观察management_epoch推进。
   * @param now_us 同域宿主µs。
   */
  void tick(std::uint64_t now_us);
  /**
   * @brief 查询当前是否等待本地D10释放管理请求。
   * @return phase为ReleasePending、request非零且last_operation为D10才true；不是已完成释放。
   */
  bool release_request_inflight() const noexcept {
    return phase_==BootstrapPhase::kReleasePending && request_id_!=0U &&
        last_operation_==ManageOperation::kDisableAndRelease;
  }
  /**
   * @brief 查询是否有未结束的RenewLease管理根。
   * @return bool；只反映本协调器phase/request。
   */
  bool renewal_request_inflight() const noexcept {
    return phase_==BootstrapPhase::kRenewLease && request_id_!=0U &&
        last_operation_==ManageOperation::kRenewLease;
  }
  /**
   * @brief 读取宿主bootstrap阶段。
   * @return BootstrapPhase值。
   */
  BootstrapPhase phase() const noexcept { return phase_; }
  /**
   * @brief 查询自动链已到Ready。
   * @return bool；不等于设备已使能或已开始动作，authorization仍要检查实际许可快照。
   */
  bool ready() const noexcept { return phase_ == BootstrapPhase::kReady; }
  /**
   * @brief 查询显式单步已经终结。
   * @return bool；并非所有单步完成都能导出Authorization。
   */
  bool operation_complete() const noexcept { return phase_ == BootstrapPhase::kOperationComplete; }
  /**
   * @brief 查询管理释放成功且实际双Idle已观察。
   * @return bool；成立依赖外层快照新鲜/一致性保证。
   */
  bool management_operation_pending() const noexcept {
    return phase_ == BootstrapPhase::kSelectMode ||
           phase_ == BootstrapPhase::kGrantSession ||
           phase_ == BootstrapPhase::kCaptureReference ||
           phase_ == BootstrapPhase::kSetPermit ||
           phase_ == BootstrapPhase::kManualOperation ||
           phase_ == BootstrapPhase::kRenewLease;
  }
  bool release_slot_available() const noexcept {
    return phase_ == BootstrapPhase::kIdle ||
           phase_ == BootstrapPhase::kReady ||
           phase_ == BootstrapPhase::kOperationComplete ||
           (phase_ == BootstrapPhase::kFault && terminal_failure_observed_);
  }
  bool released() const noexcept { return phase_ == BootstrapPhase::kReleased; }
  /**
   * @brief 读取匹配成功COMMITTED释放的ID对。
   * @return optional副本；可在ReleaseAwaitIdle已有该值，所以不能单独替代released()。
   */
  std::optional<ConfirmedRelease> confirmed_release() const noexcept { return confirmed_release_; }
  /**
   * @brief 读取最近显式操作类别。
   * @return ManageOperation；自动链issue内部不按每步重写此字段，不宜当唯一当前phase指示。
   */
  ManageOperation last_operation() const noexcept { return last_operation_; }
  /**
   * @brief 借用内部错误字符串C视图。
   * @return 以NUL结尾指针；后续修改error或析构后失效，需跨事件保留时复制。
   */
  const char *error() const noexcept { return error_.c_str(); }
  /**
   * @brief 只在Ready或SetPermit单步成功后由当前快照导出三资格。
   * 构造值不发送Enable；外部ActionGate还须准备Hold、真实prime和运动握手。
   * @return optional完整请求身份模板；缺本session/参考/控制模式/实际permit时为空。
   */
  std::optional<Authorization> authorization() const;

 private:
  /**
   * @brief 从最新快照构造管理根并提交特殊permit或普通请求。
   * SetPermit带编译策略、操作员确认及可信binding调用submit_permit，绝不从文件自造摘要；Grant/Renew记录原始lease_submit锚。
   * @param operation 本步管理编号。
   * @param now_us 宿主提交µs。
   * @return bool；0请求ID转BOOTSTRAP_TRANSPORT Fault。
   */
  bool issue(ManageOperation operation, std::uint64_t now_us);
  /**
   * @brief 核管理水位已推进且当前阶段的实际状态满足。
   * @return bool；Select/Grant需模式与session，Capture需有效参考，Permit还需实际剩余许可非零。
   */
  bool precondition_for_next() const noexcept;
  /**
   * @brief 检查快照中无owner/reference/permit且双无参考Idle。
   * @return bool；此辅助函数不检查时间戳或release_committed，后者由管理成功分支/外层监督补充。
   */
  bool release_idle() const noexcept;
  /**
   * @brief 仅在实际快照前置通过时递交自动下一步。
   * 顺序Select→Grant→Capture→Permit→Ready，不隐式Enable或Begin。
   * @param now_us 宿主µs，供新管理根提交。
   */
  void advance(std::uint64_t now_us);
  /**
   * @brief client借用；identity和策略hash为值，permit_remaining与configured/requested_session_lease均µs且语义不同。
   */
  ManagementClient &client_;
  ManageRequest identity_{};
  std::array<std::uint8_t, 32> compiled_policy_digest_{};
  std::uint64_t permit_remaining_us_ = 0;
  bool manual_permit_window_ = false, manual_permit_closed_ = false;
  std::uint64_t manual_permit_deadline_us_ = 0, manual_permit_issued_at_us_ = 0,
                manual_permit_boot_ = 0, manual_permit_request_deadline_us_ = 0;
  std::uint32_t manual_permit_reference_ = 0;
  std::uint64_t configured_session_lease_us_ = 0;
  std::uint64_t requested_lease_us_ = 0;
  /**
   * @brief 租期submit/confirmed/guaranteed_until为宿主绝对µs，term为时长µs；保守截止取submit+term而不是确认+term。
   */
  std::uint64_t lease_submit_at_us_ = 0;
  std::uint64_t lease_confirmed_at_us_ = 0;
  std::uint64_t lease_guaranteed_until_us_ = 0;
  std::uint64_t lease_term_us_ = 0;
  /**
   * @brief Running续租固定run/motion/reference，期间变化立即Fault，不能把一run续租响应套到下一run。
   */
  std::uint64_t renewing_run_id_ = 0;
  std::uint32_t renewing_motion_epoch_ = 0;
  std::uint32_t renewing_reference_epoch_ = 0;
  /**
   * @brief 固定5秒释放总截止；snapshot不含接收时间，因此新鲜性必须由调用者提供。
   */
  std::uint64_t release_deadline_us_ = 0;
  bool terminal_failure_observed_ = false;
  bool manual_reference_release_ = false;
  std::uint32_t release_reference_epoch_ = 0;
  BootstrapSnapshot snapshot_{};
  BootstrapPhase phase_ = BootstrapPhase::kIdle;
  std::uint64_t request_id_ = 0;
  /**
   * @brief 管理成功凭据与issued_management_epoch前置水位；automatic决定成功后继续链还是停OperationComplete。
   */
  std::optional<ConfirmedRelease> confirmed_release_;
  std::uint32_t issued_management_epoch_ = 0;
  std::string error_;
  ManageOperation last_operation_ = ManageOperation::kSelectMode;
  bool automatic_ = true;
  bool operator_confirmed_ = false;
  bool strict_evidence_ = false;
  bool manual_committed_ = false;
  bool manual_policy_denied_ = false;

public:
  bool manual_policy_denied() const noexcept { return manual_policy_denied_; }
};
}  // namespace dummy_motion_client

/**
 * @file action_gate.hpp
 * @brief 声明设备管理握手、真实控制采样与内部轨迹执行之间的门控。
 * 调用方须在同一控制executor串行调用非阻塞事件方法；本类不拥有RobotCore、通信或JTC对象。Queued只说明入队，管理COMMITTED与同身份新鲜State、prime控制更新及必要worker排空证据共同决定状态推进；内部轨迹成功不能直接作为外部动作成功。
 */
#pragma once
#include <string>
#include <vector>
#include "dummy_motion_client/action_coordinator.hpp"
#include "dummy_motion_client/management_client.hpp"
namespace dummy_motion_client {
/**
 * @brief 一条关节路点：positions为rad，velocities为rad/s，accelerations为rad/s²；time为从轨迹开始起的秒数，不是宿主绝对时刻。
 */
struct TrajectoryPoint { std::vector<double> positions, velocities, accelerations; double time=0; };
/**
 * @brief 轴名和完整路点及位置容差；path/goal_tolerance为rad，goal_time_tolerance为秒。向量长度及轴序由validate_trajectory校验/规范化。
 */
struct Trajectory {
  std::vector<std::string> joints;
  std::vector<TrajectoryPoint> points;
  std::vector<double> path_tolerance, goal_tolerance;
  double goal_time_tolerance=0;
};
/**
 * @brief 与joints同轴序的允许范围：lower/upper及容差rad，velocity为rad/s、acceleration为rad/s²；来自已绑定profile，不是实测值。
 */
struct JointProfile {
  std::vector<std::string> joints;
  std::vector<double> lower, upper, velocity, acceleration, path_tolerance, goal_tolerance;
};
/**
 * @brief 按profile重排并校验完整轨迹及可选实测起点。
 * 前两个匿名引用依次为输入Trajectory和JointProfile。要求1..6轴集合完整唯一、时间非负严格递增、向量维度/有限性/逐点范围与显式动态限制满足；空/0容差用profile默认，非零不能更宽。终点显式速度须在1e-9 rad/s内为零；纯位置轨迹另查分段差分速度和相邻段加速度。此函数不做碰撞检查或完整插值曲线动态证明。
 * @param measured_start 可空六轴rad容器；非空时将前profile轴作为position-only首段起点。
 * @return 规范化轨迹或nullopt，不修改输入。
 */
std::optional<Trajectory> validate_trajectory(const Trajectory &, const JointProfile &,
                                              const Position *measured_start=nullptr);
/**
 * @brief 管理请求身份模板加session/reference/permit三个独立资格标志；标志由上层真实观测建立，不是自造设备许可。
 */
struct Authorization {
  ManageRequest identity{};
  bool session=false, reference=false, permit=false;
};
/**
 * @brief 已经匹配成功取消管理事务的原request_id和设备operation_id；仍需调用方结合终页/实际Hold等验收链，不代表失能释放。
 */
struct CancelReceipt {
  std::uint64_t request_id=0, operation_id=0;
};
/**
 * @brief worker排空现场：processed_generation是宿主64位代，committed_seq是32位线序号；position/tolerance为rad，observed/max_state_age为宿主steady µs。quiescent和disposition区分本地同值吞并与设备ACK。
 */
struct HoldDrainEvidence {
  std::uint64_t boot_epoch=0, session_id=0, processed_generation=0;
  std::uint32_t reference_epoch=0, motion_epoch=0, committed_seq=0;
  // 1: legitimate local no-op; 2: a device-acknowledged CAN group. Never a queued write.
  std::uint8_t disposition=0;
  bool quiescent=false;
  std::uint64_t observed_us=0, max_state_age_us=0;
  Position position{}, tolerance{};
};
/**
 * @brief 由运行集成层实现的JTC/采样sink控制端口。
 * 本类只定义借用回调，ActionGate不拥有具体JTC。disable/cancel/prime分别是集成动作，不能仅凭方法调用推断设备已受控停车、失能或发送了CAN。
 */
class ActionPort {
 public:
  /**
   * @brief 允许经基类指针析构具体端口；默认不发送设备请求。
   */
  virtual ~ActionPort() = default;
  // Called only after core acceptance: the actual update callback must publish the prime.
  /**
   * @brief 在设备core接纳后要求真实update回调产生保持prime。
   * 第一个匿名Position引用为完整rad保持目标；端口负责安装真实控制更新源，不能只修改本地变量就报采样完成。
   * @param epoch 已经接受的设备motion epoch。
   * @param generation 本ActionGate控制代，用于拒绝迟到回调。
   */
  virtual void prime(const Position &, std::uint32_t epoch, std::uint64_t generation)=0;
  /**
   * @brief 在prime/COMMITTED握手通过后启动内部轨迹执行。
   * 匿名Trajectory引用为已验证完整路径；端口若异步保存须自行确保数据寿命。调用后仍通过record_update和inner_result报告，不能直接判设备到位。
   * @param generation 绑定本次内部执行的门控代。
   */
  virtual void start(const Trajectory &, std::uint64_t generation)=0;
  /**
   * @brief 解除实际控制update生产sink，禁止继续提交旧目标。
   * 接口本身不保证设备失能；具体电机停止/释放走管理事务。
   */
  virtual void disable()=0;
  /**
   * @brief 要求内部控制器取消当前执行/转保持。
   * 不等同设备Cancel COMMITTED或D10释放，门控仍等待其独立收据与State。
   */
  virtual void cancel()=0;
  /**
   * @brief 尝试冻结Hold更新以先排空旧命令再Begin。
   * @return 默认false表示端口不提供该排空握手；true时ActionGate进入有期限的hold_drain等待。
   */
  virtual bool freeze_hold() { return false; }
};
// Serialize these nonblocking event handlers on the caller's control executor.
// The port owns the actual JTC registration and update sink; no RobotCore is copied here.
/**
 * @brief 借用管理client与执行port的单目标事件状态机。
 * phase描述协议/prime/执行阶段，terminal是外部目标结果；generation每次prime/停/故障递增以淘汰旧内部回调。所有now/deadline为同一宿主单调µs，轨迹point.time则是相对秒，两者不能混算。
 */
class ActionGate {
 public:
  /**
   * @brief 绑定依赖并保存配置与单次握手预算。
   * 匿名参数依次为ManagementClient借用、ActionPort借用、JointProfile值副本；依赖必须活过gate。构造不创建会话、使能或发轨迹。
   * @param timeout_us 管理/prime/排空等待预算µs，默认1秒。
   */
  ActionGate(ManagementClient &, ActionPort &, JointProfile, std::uint64_t timeout_us=1000000);
  /**
   * @brief 只在Idle且三项资格齐备时请求设备PrepareHold。
   * 匿名Authorization引用提供当前身份和session/reference/permit。实际Observe/prime/State握手通过后才进入Hold。
   * @param now 宿主单调µs。
   * @return bool：本地请求已建或可接受延期；不表示已进入Hold。
   */
  bool enable(const Authorization &, std::uint64_t now);
  /**
   * @brief 在空闲Hold替换同设备/会话/参考的已验证permit快照。
   * 匿名Authorization是外层已完成许可流程的值；本方法不发送SetPermit，也不续设备session租期。不能换device/client/profile/mask/boot/session/reference。
   * @return bool：通过绑定条件并更新本地授权。
   */
  bool renew_permit(const Authorization &);
  /**
   * @brief 验证轨迹后请求Begin或先冻结并等待Hold排空。
   * 匿名Trajectory为用户完整路径。ready且三资格有效才受理；freeze_hold=true时记录宿主冻结代和线序号，由worker+新鲜State共同证明排空后发Begin，使用同一截止。
   * @param now 宿主单调µs。
   * @return bool：本地受理目标；真正Running还需设备/prime握手。
   */
  bool submit(const Trajectory &, std::uint64_t now);
  /**
   * @brief 轮询管理、续租延迟及握手/排空超时。
   * 延期Cancel/Complete等待匹配续租COMMITTED且仍在原截止，刷新ticket后提交；超时/异常按当前阶段进入受控取消或Fault。不会靠tick次数延长原预算。
   * @param now 同域宿主单调µs。
   */
  void tick(std::uint64_t now);
  /**
   * @brief 消费匹配当前request的管理结果并与实际State证据合并。
   * 匿名ManageReply为已解码结果。响应本身通常没有snapshot，已有State时合并result_valid/succeeded再advance；Queued或仅接受不能直接启动路径。
   * @param now 宿主单调µs。
   */
  void observe(const ManageReply &, std::uint64_t now);
  /**
   * @brief 核对codec、身份、严格序号和有限实际位置后推进握手。
   * 核boot/session/reference/device/profile/mask并更新ticket/管理epoch；此层不独立映射produced时钟或计算transport年龄，调用方须先提供可信新鲜流。普通握手需实际prime采样和管理成功，Hold排空另核全配置轴submitted/sent与commit水位。
   * @param bytes 调用期有效wire State字节。
   * @param size 载荷字节数。
   * @param now 调用方提供的新鲜接收/观察宿主µs。
   * @return false拒绝载荷；true表示该帧被门控消费，可能因此进入故障/停止。
   */
  bool observe_state(const std::uint8_t *bytes, std::size_t size, std::uint64_t now);
  /**
   * @brief 登记真实控制update产生的有限目标代/线序号。
   * 匿名Position为此次rad目标。非Running阶段的每个采样及Running尚未sampled的首个采样须与prime_q在1e-9 rad内；只有Running阶段独立核profile上下界，所有阶段都核有限值。登记证明控制器生产该目标，不证明网络/CAN已发送。
   * @param generation 必须等本gate代。
   * @param epoch 必须等当前motion。
   * @param seq 非零且严格更新的32位producer序号。
   * @param producer_generation 宿主64位代，0时兼容回退为seq。
   * @return bool；冻结/错误phase、Running越界、任意阶段非有限或需要匹配prime时分歧均拒绝。
   */
  bool record_update(std::uint64_t generation, std::uint32_t epoch,
                     std::uint32_t seq, const Position &, std::uint64_t producer_generation=0);
  /**
   * @brief 接收worker排空现场并尝试完成/Begin屏障。
   * 匿名HoldDrainEvidence先保存用于Complete-Hold严格验证；Begin排空还要求processed_generation≥冻结代、同身份/epoch及非零commit，再等新鲜State提交水位。
   * @param now 宿主单调µs。
   */
  void observe_hold_drain(const HoldDrainEvidence &, std::uint64_t now);
  /**
   * @brief 处理当前代内部执行结果，成功也先冻结末采样再Complete。
   * 匿名Terminal为内部结果，匿名string为原因。成功必须已有末线序号且最后真实update目标位于终点容差，随后关闭sink、冻结目标与terminal_seq并进入Completing；尚未宣布外部Succeeded。
   * @param generation 内部执行代，旧代忽略。
   * @param now 宿主µs。
   */
  void inner_result(std::uint64_t generation, Terminal, const std::string &, std::uint64_t now);
  /**
   * @brief 对当前未结束请求/排空/延期完成发起正常用户取消。
   * stop(true,空原因)请求设备Cancel并等待后继Hold，不能直接把按钮点击当Canceled。
   * @param now 宿主µs。
   */
  void cancel(std::uint64_t now);
  /**
   * @brief 因异常中止并启动取消或故障流程。
   * 匿名string是异常原因；若正延期等待续租、尚无取消请求则直接Fault，避免无限延期。
   * @param now 宿主µs。
   */
  void abort(const std::string &, std::uint64_t now);
  /**
   * @brief 撤销会话资格并淘汰所有旧执行代。
   * 活动请求尝试停，随后关闭sink并Fault；本地失效不能作为设备释放成功凭证。
   * @param now 宿主µs。
   */
  void invalidate_authorization(std::uint64_t now);
  /**
   * @brief 查询空闲Hold且无待办/排空、permit仍可用。
   * @return bool；只作下一submit准入，不等于网络连接或机械安全证明。
   */
  bool ready() const { return phase_==Phase::kHold && request_==0 &&
      !hold_drain_pending_ && authorization_.permit; }
  /**
   * @brief 查询当前Hold是否可请求外层重新设置motion permit。
   * @return 无request、session/reference有效而permit已消费时true；不自行续租。
   */
  bool permit_refresh_allowed() const {
    return phase_==Phase::kHold && request_==0 && authorization_.session &&
        authorization_.reference && !authorization_.permit;
  }
  /**
   * @brief 核对取消尚未发送且仅等待同根在途续租。
   * @param live 当前外层Cancel意图身份。
   * @param now 宿主µs，必须早于原deadline。
   * @return bool；需冻结、正常用户取消、无错误、延期身份和matching renewal完全一致。
   */
  bool cancel_waiting_for_renewal(const ManageRequest &live, std::uint64_t now) const noexcept;
  /**
   * @brief 查询冻结完成请求是否受匹配在途续租暂占单槽。
   * @param now 宿主µs。
   * @return bool；不允许新采样，不延长完成deadline。
   */
  bool completion_waiting_for_renewal(std::uint64_t now) const noexcept;
  /**
   * @brief 证明本目标冻结且设备commit覆盖末序号的有限完成意图。
   * 只供协调撤回未发送RUN续租预约，不授权新采样、额外续租或新动作。
   * @param live 当前完整期望身份。
   * @param committed_seq 设备当前commit线序号。
   * @param now 宿主µs，须在deadline内。
   * @return bool；还需冻结目标等于最后采样且位于目标容差。
   */
  bool bounded_completion_intent(const ManageRequest &live, std::uint32_t committed_seq,
                                 std::uint64_t now) const noexcept;
  /**
   * @brief 识别本goal已经提交且仍在期限内的正常Cancel。
   * 用于取消未发出的RUN续租预约，不能据此延长Stop期限。
   * @param live 外层当前Cancel根身份。
   * @param now 宿主µs。
   * @return bool；不包含尚未发送、异常中止或身份改变情况。
   */
  bool bounded_cancel_intent(const ManageRequest &live, std::uint64_t now) const noexcept;
  /**
   * @brief 读取当前外部目标终态。
   * @return Terminal值；内部JTC终态与本值可能尚不同。
   */
  Terminal terminal() const { return terminal_; }
  /**
   * @brief 只导出已成功COMMITTED并回到Hold的正常取消事务ID。
   * @return optional CancelReceipt；需Canceled、同request/op/身份/epoch等全部满足。
   */
  std::optional<CancelReceipt> confirmed_cancel_receipt() const noexcept;
  /**
   * @brief 读取协议/prime/运行阶段。
   * @return Phase值，不是设备core_state。
   */
  Phase phase() const { return phase_; }
  /**
   * @brief 读取门控内部执行代。
   * @return 宿主64位generation，不是设备motion epoch或线producer_seq。
   */
  std::uint64_t generation() const { return generation_; }
  /**
   * @brief 借用当前错误原因文本。
   * @return const引用；gate后续事件可修改内容，引用不宜跨生命周期保存。
   */
  const std::string &error() const { return error_; }
 private:
  // 纯位置轨迹必须先由当前fresh实际Hold起点证明首槽/整段动态界限。
  bool valid_begin_trajectory(const Trajectory &, std::uint64_t now);
  /**
   * @brief 核对延期Cancel根仍恰等当前完整授权身份。
   * @return bool；不产生新管理权限或更新时间。
   */
  bool deferred_cancel_matches_authority() const noexcept;
  /**
   * @brief 核对延期Complete身份与当前授权/epoch一致。
   * @return bool；ticket可后续刷新，其余根身份不得漂移。
   */
  bool deferred_complete_matches_authority() const noexcept;
  /**
   * @brief 按授权模板提交管理操作并固定本次截止。
   * 匿名ManageOperation指定操作。Complete携冻结终seq/位置；只在单槽被同身份RenewLease占用时允许延期Complete或无错误正常Cancel，绝不延期别的未知事务。
   * @param now 宿主µs。
   * @param existing_deadline 非零时复用既有截止，0才用now+timeout。
   * @return bool；成功建立请求或匹配续租延期；否则fail。
   */
  bool issue(ManageOperation, std::uint64_t now, std::uint64_t existing_deadline=0);
  /**
   * @brief 在设备core接受后切代并要求真实端口更新保持目标。
   * 匿名ManageReply给设备实际/保持位置，匿名Phase指定prime阶段。先disable旧sink再递增generation、清采样水位；PrimeMotion还以实际起点重校验position-only首段动态。
   * @param now 宿主µs。
   * @return bool；motion起点重新验证失败则受控stop。
   */
  bool prime(const ManageReply &, Phase, std::uint64_t now);
  /**
   * @brief 记录取消/中止意图，冻结旧采样并请求设备Cancel。
   * 匿名string为原因。关闭sink、调用port.cancel并切代后进入Stopping；成功后继Hold再给Canceled/Aborted，失败为STOP_FAILED，不能把本地port.cancel视为设备确认。
   * @param canceled True为正常用户取消，False为异常中止。
   * @param now 宿主µs。
   */
  void stop(bool canceled, const std::string &, std::uint64_t now);
  /**
   * @brief 清等待、禁用端口并锁Fault/Aborted终态。
   * 匿名string为错误；保留先前原因，STOP_FAILED可覆盖。此处不新发网络Stop，必须由外部监督处理未确认停止。
   */
  void fail(const std::string &);
  /**
   * @brief 核管理成功、实际快照及真实prime采样水位。
   * 匿名ManageReply由State与管理结果合成；Queued或仅本地publish不满足此握手。
   * @return bool；snapshot_valid、当前epoch、sampled、非零accepted_seq覆盖prime_seq且result成功。
   */
  bool handshake(const ManageReply &) const;
  /**
   * @brief 按管理/State合证推进Enable、Begin、Complete或Cancel的prime阶段。
   * 匿名ManageReply须当前request；拒绝结果按阶段fail或stop，Queued/无accepted/无snapshot不推进。prime握手完成才调用port.start或标终态Hold。
   * @param now 宿主µs。
   */
  void advance(const ManageReply &, std::uint64_t now);
  /**
   * @brief 在Complete特殊分支要求管理成功、双新鲜Hold及worker排空一致后Succeeded。
   * worker必须quiescent、disposition合法、处理代覆盖prime代且commit被State覆盖；actualHold与prime目标在worker逐轴rad容差内。首/终页raw验收仍由上层集成监督。
   * @param now 宿主µs。
   */
  void try_completion_hold(std::uint64_t now);
  /**
   * @brief 借用client/port须活过gate；profile为const值副本，timeout为宿主µs。所有调用须串行，类内无并发mutex。
   */
  ManagementClient &client_; ActionPort &port_; const JointProfile profile_;
  const std::uint64_t timeout_;
  /**
   * @brief 本地授权快照与规范化轨迹；phase/terminal和operation各描述不同状态维度，不可用任一字段单独推断设备使能。
   */
  Authorization authorization_{}; Trajectory trajectory_{};
  Phase phase_=Phase::kIdle; Terminal terminal_=Terminal::kNone;
  ManageOperation operation_=ManageOperation::kEnable;
  /**
   * @brief request为当前管理根ID，deadline为宿主绝对µs，generation为内部控制代；0 request不总是空闲，可能正等待匹配续租。
   */
  std::uint64_t request_=0, deadline_=0, generation_=0;
  /**
   * @brief 延期Complete/Cancel保存占单槽续租ID及原始请求根；tick只在原deadline内等其COMMITTED，不能换身份重发。
   */
  std::uint64_t completing_after_renewal_=0U;
  ManageRequest deferred_complete_request_{};
  std::uint64_t canceling_after_renewal_=0U;
  ManageRequest deferred_cancel_request_{};
  std::uint64_t cancel_request_id_=0;
  /**
   * @brief 宿主代数/冻结代/排空代与下面32位线序号分开；completion_worker保存worker实际排空快照。
   */
  std::uint64_t last_producer_generation_=0, frozen_generation_=0, drained_generation_=0;
  std::uint64_t prime_producer_generation_=0, hold_state_observed_us_=0;
  HoldDrainEvidence completion_worker_{};
  /**
   * @brief epoch为设备motion，last/frozen/prime/hold_committed为线序号；位置last/prime/frozen单位rad，mask为配置轴提交/SENT观测。
   */
  std::uint32_t epoch_=0, last_seq_=0, frozen_seq_=0, prime_seq_=0;
  std::uint32_t hold_committed_seq_=0;
  std::uint32_t drained_committed_seq_=0;
  std::uint8_t hold_submitted_mask_=0, hold_sent_mask_=0;
  Position last_q_{}, prime_q_{}, frozen_q_{}, measured_hold_q_{};
  /**
   * @brief sampled表示真实update发生；frozen阻断后继采样；cancel_terminal区分正常取消与异常；排空状态须同时具worker和State证据。
   */
  bool sampled_=false, frozen_=false, cancel_terminal_=false, hold_drain_pending_=false;
  bool drain_state_fresh_=false, drain_worker_ready_=false;
  std::string error_;
  /**
   * @brief 管理结果与State快照分别保留再合成握手；state_seq为严格递增State水位，state_seen防首次零序号。
   */
  ManageReply management_evidence_{}, state_evidence_{};
  std::uint32_t state_seq_=0;
  bool state_seen_=false;
};
}  // namespace dummy_motion_client

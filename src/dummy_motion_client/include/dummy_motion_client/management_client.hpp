/**
 * @file management_client.hpp
 * @brief 声明单槽管理RPC客户端、wire结果解释和session续租因果关联。
 * 管理mutation只提交一次，Received/Pending后以新Query ID追问原请求；poll返回可含中间证据，不等于设备动作完成。RequestHandle负责本地等待寿命，销毁取消本地观察而不保证远端mutation撤销。接口调用由控制executor串行化，不是多线程请求队列。
 */
#pragma once
#include <array>
#include "dp_protocol_constants.h"
#include <cstdint>
#include <memory>
#include <optional>
#include <type_traits>
#include <vector>
#include "dummy_ros_control/wire_types.hpp"
#include "dummy_motion_client/bench_permit.hpp"

namespace dummy_motion_client {
/**
 * @brief ManageRequest为生成的wire请求类型，Position为六轴rad容器；身份字段/序号与角度不共用单位。
 */
using ManageRequest = dummy_wire::DummyManageV2Request;
using Position = std::array<double, 6>;
// Dummy wire-major-v2 stable values.
/**
 * @brief 稳定wire管理编号；Enable对应PrepareHold，Begin/Complete/Cancel改变动作阶段，RenewLease仅续session而非permit。
 */
enum class ManageOperation : std::uint16_t {
  kGetStatus=0, kSelectMode=1, kGrantSession=2, kCaptureReference=3,
  kClearReference=4, kSetPermit=5, kEnable=6, kBegin=7, kComplete=8,
  kCancel=9, kDisableAndRelease=10, kClearFault=11, kQuery=12,
  kConfirmManualHome=DP_PROTOCOL_DPSDK_INTERFACES_MSG_DUMMYPROTOCOLV2_MESSAGE_MANAGEMENT_OPERATION_CONFIRM_MANUAL_HOME,
  kBeginQualification=DP_PROTOCOL_DPSDK_INTERFACES_MSG_DUMMYPROTOCOLV2_MESSAGE_MANAGEMENT_OPERATION_BEGIN_QUALIFICATION,
  kValidateRest=DP_PROTOCOL_DPSDK_INTERFACES_MSG_DUMMYPROTOCOLV2_MESSAGE_MANAGEMENT_OPERATION_VALIDATE_REST,
  kCompleteQualification=DP_PROTOCOL_DPSDK_INTERFACES_MSG_DUMMYPROTOCOLV2_MESSAGE_MANAGEMENT_OPERATION_COMPLETE_QUALIFICATION,
  kRenewLease=20
};
/**
 * @brief 设备core_state常用值，区别于宿主ActionGate::Phase；Idle/EnablingHold/Hold/Running/Fault各需实际消息确认。
 */
enum class WireCoreState : std::uint16_t {
  kUnreferencedIdle=1, kIdle=2, kEnablingHold=3, kHold=4, kRunning=5, kFault=9
};
/**
 * @brief 设备receipt编号：Queued/Replayed表RPC接纳层级，Busy不是已开始操作。
 */
enum class WireReceipt : std::uint8_t { kQueued=1, kReplayed=2, kBusy=4 };
/**
 * @brief wire result编号；Pending=23表示需继续查，不可当成功。
 */
enum class WireResult : std::uint16_t { kOk=0, kPending=23 };
/**
 * @brief 设备事务阶段：Received/Prepared/Committed/Failed与ROS Action终态分开。
 */
enum class WireTransactionPhase : std::uint8_t { kIdle=0, kReceived=1, kPrepared=2, kCommitted=3, kFailed=4 };
/**
 * @brief 将强类型wire枚举转换为其底层整数。
 * @param value 枚举值。
 * @return underlying_type_t值，不校验是否属于已登记枚举。
 */
template <typename Enum> constexpr auto wire_value(Enum value) { return static_cast<std::underlying_type_t<Enum>>(value); }
/**
 * @brief 解码后的分层证据：queued、accepted、result_valid/succeeded/rejected并非同义。position/hold_position为rad；WireManagementClient本身不带实际快照(snapshot_valid=false)，State证据由ActionGate另合并。
 */
struct ManageReply {
  /**
   * @brief request_id保留原mutation根ID，operation_id是设备事务ID；Query自己的wire请求ID在client内部追踪，不冒充根。
   */
  std::uint64_t request_id=0, operation_id=0;
  bool queued=false, accepted=false, rejected=false, result_valid=false, succeeded=false;
  /**
   * @brief 实际位置/可保持标志来自可信State或测试端口，不能由普通RPC Ok自动补成true；release_committed也须与实际无owner Idle合证。
   */
  bool snapshot_valid=false, hold_available=false, hold_ready=false;
  bool release_committed=false;
  std::uint16_t result_code=0, first_failure_reason=0;
  std::uint8_t transaction_phase=0;
  std::uint32_t motion_epoch=0, accepted_seq=0;
  std::uint64_t session_id=0;
  std::uint32_t reference_epoch=0, management_epoch=0;
  Position position{}, hold_position{};
};
/**
 * @brief 只查询匹配RenewLease根的Pending/Committed/Failed，不等同permit剩余或所有动作结果。
 */
enum class RenewalCompletion { kPending, kCommitted, kFailed };
/**
 * @brief ActionGate/Bootstrap借用的抽象管理端口。
 * 默认permit/续租扩展按失败返回，避免替身或不支持实现意外赋予许可。submit/poll只负责管理因果，不直接控制轨迹。
 */
class ManagementClient {
 public:
  /**
   * @brief 支持经基类析构具体client；默认不发送释放。
   */
  virtual ~ManagementClient() = default;
  /**
   * @brief 提交一个非permit管理mutation。
   * 匿名ManageRequest为期待身份/操作模板；具体实现补schema、request ID/序号。
   * @param sent_at_us 原始宿主steady提交µs，用作因果/超时锚。
   * @return 非零原请求ID或0拒绝/忙；不表示设备已接受。
   */
  virtual std::uint64_t submit(const ManageRequest &, std::uint64_t sent_at_us)=0;
  /**
   * @brief 通过专用可信binding提交SetPermit。
   * 匿名参数依次为SetPermit请求及PermitBinding；不得绕过此入口用普通submit直接塞任意permit摘要。
   * @param sent_at_us 原始宿主提交µs。
   * @return 默认0不支持；具体实现仅在binding匹配且摘要构造成功时提交。
   */
  virtual std::uint64_t submit_permit(const ManageRequest &, const PermitBinding &,
                                      std::uint64_t sent_at_us) { return 0U; }
  /**
   * @brief 非阻塞读取指定根请求的下一证据/结果。
   * @param request_id 原mutation请求ID。
   * @return optional ManageReply；无进展为空，具体实现可抛传输/身份异常。
   */
  virtual std::optional<ManageReply> poll(std::uint64_t request_id)=0;
  /**
   * @brief 寻找与当前Complete/Cancel身份完全一致的在途续租。
   * 匿名ManageRequest是待协调的Complete或Cancel根，不允许借任意Busy请求延期。
   * @return 默认0；具体实现匹配时返回占单槽的原RenewLease请求ID。
   */
  virtual std::uint64_t matching_inflight_renewal(const ManageRequest &) const { return 0U; }
  /**
   * @brief 查询某个续租根的可信完成层级。
   * @return 默认Failed；匿名uint64为原RenewLease请求ID，不是operation_id。
   */
  virtual RenewalCompletion renewal_completion(std::uint64_t) const { return RenewalCompletion::kFailed; }
};
// Bind this port to the selected device's manage_v2 endpoint. A handle owns the
// native request until its reply or explicit destruction. Poll never blocks.
/**
 * @brief 将固定wire请求绑定到选定manage_v2端点的传输抽象。
 * 提供单个本地等待句柄，不授予session/动作许可。poll不等待网络，但实现可能分配字节容器及使用try_lock，因此不声称硬实时无分配。
 */
class ManagementTransport {
 public:
  /**
   * @brief 唯一拥有一次本地原生/共享请求等待的抽象句柄。
   */
  class RequestHandle {
   public:
    /**
     * @brief 销毁本地等待并通知实现取消观察，不能保证远端mutation未执行。
     */
    virtual ~RequestHandle() = default;
    /**
     * @brief 取一次完整响应字节或返回无进展。
     * @return optional拥有字节副本；失败可抛runtime_error，具体实现一次响应不重复交付。
     */
    virtual std::optional<std::vector<std::uint8_t>> poll()=0;
  };
  /**
   * @brief 允许经基类析构传输实现；具体实现负责线程join。
   */
  virtual ~ManagementTransport() = default;
  /**
   * @brief 尝试将完整wire请求交给有限传输槽。
   * 匿名vector为调用期借用请求字节；实现必须复制或拥有必要内容后返回，不保存临时vector指针。
   * @return 唯一等待句柄或空；有句柄也可能尚在有限本地admission，不能当设备已收到。
   */
  virtual std::unique_ptr<RequestHandle> send(const std::vector<std::uint8_t> &)=0;
  /**
   * @brief 为共享请求序号空间预留序号。
   * @param proposed 调用者建议的非零序号。
   * @return 默认原样返回；DeviceWorker实现与共享channel串行分配以避冲突。
   */
  virtual std::uint32_t reserve_sequence(std::uint32_t proposed) noexcept { return proposed; }
};
/**
 * @brief 管理一次mutation与后续Query链的单槽wire客户端。
 * 借用transport；handle为唯一所有权；current为当前wire Query，original始终保原mutation与其原sent_at。无内部并发锁，调用方须串行使用。
 */
class WireManagementClient final : public ManagementClient {
 public:
  /**
   * @brief 采用默认请求ID/序号起点绑定transport。
   * @param transport 借用传输对象，须长于client寿命。
   */
  explicit WireManagementClient(ManagementTransport &transport) : transport_(transport) {}
  /**
   * @brief 按session混合请求ID命名空间，nonce默认0。
   * @param transport 借用传输。
   * @param session_id 本次会话ID，作为ID域种子，不在此授予会话。
   */
  WireManagementClient(ManagementTransport &transport, std::uint64_t session_id)
      : WireManagementClient(transport, session_id, 0U) {}
  /**
   * @brief 按session与process nonce初始化请求域。
   * next_id与next_seq跳过0；混合用于降低域重复，不是密码学认证或全局唯一性证明。
   * @param transport 借用传输。
   * @param session_id 设备会话ID。
   * @param process_nonce 宿主进程随机/调用方nonce，与session异或后混合。
   */
  WireManagementClient(ManagementTransport &transport, std::uint64_t session_id,
                       std::uint64_t process_nonce)
      : transport_(transport),
        next_id_(session_request_namespace(session_id ^ process_nonce)),
        next_seq_(static_cast<std::uint32_t>(session_id) + 1U) {
    if (next_id_ == 0U) next_id_ = 1U;
    if (next_seq_ == 0U) next_seq_ = 1U;
  }
  /**
   * @brief 仅在无句柄/待发Query时提交非SetPermit请求。
   * 匿名ManageRequest被复制，补当前schema和新ID/序号；原始mutation不会因Query丢包而重发。
   * @param sent_at_us 宿主µs原提交锚。
   * @return 原mutation request_id，不能提交或本地发送失败0。
   */
  std::uint64_t submit(const ManageRequest &, std::uint64_t sent_at_us) override;
  /**
   * @brief 校验操作员确认、策略及binding并构造可信permit摘要。
   * 匿名参数为请求和PermitBinding；要求操作SetPermit、非零permit、策略非零且boot/reference/profile匹配，摘要还绑定owner/session/实际预留sequence。普通submit明确拒绝SetPermit。
   * @param sent_at_us 宿主原提交µs。
   * @return 非零根请求ID或0。
   */
  std::uint64_t submit_permit(const ManageRequest &, const PermitBinding &,
                              std::uint64_t sent_at_us) override;
  /**
   * @brief 核响应身份、operation因果并自动转后续Query。
   * 仅Query本地失败可有限重试最多3次，绝不重发Enable/Begin。每轮Query使用新wire ID追问原根并保持原sent_at；Cancel/Release不做9次poll节流，其余操作保留节流。terminal需Committed/Failed且非Pending；success不含实际位置，snapshot_valid保持false。
   * @param request_id 原mutation根ID；不同ID立即返回空。
   * @return 中间/终态ManageReply或无进展；codec/身份/终receipt/重试耗尽抛runtime_error。
   */
  std::optional<ManageReply> poll(std::uint64_t request_id) override;
  /**
   * @brief 核Complete/Cancel与占用单槽RenewLease所有根身份相同。
   * 匿名请求须Complete或Cancel，比较设备/客户端/boot/session/reference/motion/management/profile/mask，不能用不同身份续租作为延期借口。
   * @return 匹配的原RenewLease ID，否则0。
   */
  std::uint64_t matching_inflight_renewal(const ManageRequest &) const override;
  /**
   * @brief 查询当前/刚结束续租根是否COMMITTED且epoch绑定未变。
   * @return Pending、Committed或Failed；匿名ID未匹配也Failed。
   */
  RenewalCompletion renewal_completion(std::uint64_t) const override;
  /**
   * @brief 读取整个mutation/Query链最初宿主提交锚。
   * @return µs，不因每轮Query更新。
   */
  std::uint64_t original_sent_at_us() const { return sent_at_us_; }
 private:
  /**
   * @brief 用固定64位混合将会话域映射到请求起点。
   * @param session_id 会话与nonce异或后的输入种子。
   * @return 64位混合值，调用构造会把0改1；非密码学哈希。
   */
  static constexpr std::uint64_t session_request_namespace(std::uint64_t session_id) noexcept {
    std::uint64_t mixed = session_id + 0x9e3779b97f4a7c15ULL;
    mixed = (mixed ^ (mixed >> 30U)) * 0xbf58476d1ce4e5b9ULL;
    mixed = (mixed ^ (mixed >> 27U)) * 0x94d049bb133111ebULL;
    return mixed ^ (mixed >> 31U);
  }
  /**
   * @brief 分配下一个本地请求ID并跳过0。
   * @return 原next_id后递增值；不声明跨重启绝对唯一。
   */
  std::uint64_t take_request_id() noexcept {
    if (next_id_ == 0U) next_id_ = 1U;
    return next_id_++;
  }
  /**
   * @brief 通过transport预留共享非零线序号并推进建议水位。
   * @return uint32 request_seq；默认transport直接用建议值，DeviceWorker协调共享序列。
   */
  std::uint32_t take_request_sequence() noexcept {
    if (next_seq_ == 0U) next_seq_ = 1U;
    const auto sequence = transport_.reserve_sequence(next_seq_);
    next_seq_ = sequence + 1U;
    return sequence;
  }
  /**
   * @brief 编码并替换本地等待句柄，Query可进入pending_send。
   * 匿名请求为本轮原mutation或Query。先reset旧未送Query再替换，避免取消事务卡在旧查询占槽；仅本地取消观察，远端状态仍须查。
   * @return 存在句柄或Query可待发为true；mutation无句柄则false。
   */
  bool send(const ManageRequest &);
  /**
   * @brief 最近续租根结果缓存；Committed还要求reference/motion/management与原续租一致，不因任何Ok误判续租成功。
   */
  std::uint64_t completed_renewal_id_=0U;
  RenewalCompletion completed_renewal_result_=RenewalCompletion::kFailed;
  /**
   * @brief 借用传输与唯一handle；original为mutation不变根，current为实际wire Query；序号/operation与原提交时间各自分开。
   */
  ManagementTransport &transport_;
  std::unique_ptr<ManagementTransport::RequestHandle> handle_;
  ManageRequest original_{};
  ManageRequest current_{};
  /**
   * @brief 只用于尚未本地接纳的Query；query_delay_polls按轮询次数退避、query_retries限制重试，不是新的目标时间预算。
   */
  bool pending_send_=false;
  unsigned query_delay_polls_=0;
  unsigned query_retries_=0;
  /**
   * @brief next_id/wire_id为宿主请求ID，operation_id为设备事务ID，sent_at_us为原宿主µs；accepted_缓存是否见过设备接纳。
   */
  std::uint64_t next_id_=1, wire_id_=0, operation_id_=0, sent_at_us_=0;
  std::uint32_t next_seq_=1;
  bool accepted_=false;
};
}  // namespace dummy_motion_client

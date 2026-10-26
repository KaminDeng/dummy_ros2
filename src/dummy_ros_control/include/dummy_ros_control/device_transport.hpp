/**
 * @file device_transport.hpp
 * @brief 声明独占通信worker及位置命令、ACK、心跳与能力校验辅助值。
 * ROS实时read/write不直接创建Context/节点或做网络I/O；DeviceTransport私有worker拥有dp_comm上下文和执行器。源设备produced_at_us与宿主steady_us分域，worker先校正队列年龄再向硬件提供有截止的状态；重连不续用旧命令意图。
 */
#pragma once
#include "dp_protocol_constants.h"
#include "dummy_ref/manual_reference_policy.hpp"
#include "dummy_ros_control/diagnostic_journal.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>

#include "dummy_ros_control/command_producer.hpp"

namespace dummy_ros_control {
class GuardedZenbeddedSystem;
struct DeviceManagementChannel;

/** @brief
 * 管理RPC显式键与wire请求绑定；同wire重试保持键，避免重连时复用SDK自动计数缓存。
 */
template <typename Request>
std::uint32_t management_rpc_correlation_id(const Request &request) noexcept {
  return dummy_ref::host::management_rpc_correlation_id(request);
}
/** @brief SDK纯策略转发；保留Native
 * Query特例及稳定零值回退，不触发SDK自动分配。 */
template <typename Request>
std::uint32_t management_rpc_idempotency_key(const Request &request) noexcept {
  return dummy_ref::host::management_rpc_idempotency_key(
      request,
      request.operation() ==
          DP_PROTOCOL_DPSDK_INTERFACES_MSG_DUMMYPROTOCOLV2_MESSAGE_MANAGEMENT_OPERATION_GET_OPERATION);
}

/**
 * @brief 传输配置值：身份是16/32字节定长数组，位置rad、速度rad/s、加速度rad/s²，所有*_us为µs；raw路径仅诊断采集，不授予运动。
 */
struct TransportParameters {
  std::string endpoint;
  std::array<std::uint8_t, 16> device_id{};
  std::array<std::uint8_t, 16> client_instance_id{};
  std::uint32_t profile_id = 0U;
  std::uint8_t configured_mask = 0U;
  /**
   * @brief 状态新鲜预算及发布周期µs；heartbeat=0保留5周期停止规则，ACK=0回退max_state_age，stop确认=0也回退状态预算。
   */
  std::uint64_t max_state_age_us = 100000U;
  std::uint64_t state_publish_period_us = 10000U;
  // Zero retains the transport's legacy 5x state-period Stop threshold.
  std::uint64_t heartbeat_stop_budget_us = 0U;
  // Zero uses max_state_age_us until an observed host ACK budget is configured.
  std::uint64_t group_sent_ack_timeout_us = 0U;
  std::uint64_t stop_confirmation_timeout_us = 0U;
  std::array<std::uint8_t, 32> contract_hash{};
  std::array<std::uint8_t, 32> model_hash{};
  std::array<std::uint8_t, 32> profile_hash{};
  std::array<std::uint8_t, 32> firmware_hash{};
  std::array<std::uint8_t, 32> policy_hash{};
  /**
   * @brief 六轴设备位置上下界rad及动态上限；即使仅配置J1，参数解析仍要求完整六轴向量。
   */
  std::array<double, 6> position_min{};
  std::array<double, 6> position_max{};
  std::array<double, 6> velocity_max{};
  std::array<double, 6> acceleration_max{};
  std::uint64_t control_period_us = 0U;
  std::string raw_evidence_path;
  bool manual_reference_mode = false;
};

/**
 * @brief 判断历史已释放Idle的本地上下文；run/boot和链绑定历史，写页/终页水位防止把本次已运行记录当冷启动。
 */
struct HistoricalIdleBaselineContext {
  std::uint64_t current_boot = 0U, historical_run_boot = 0U,
                historical_run_id = 0U,
                last_status_run_id = 0U, pages_written = 0U,
                terminal_written_run_id = 0U;
  std::array<std::uint8_t, 32> historical_chain{};
  bool manual_reference_mode = false;
  std::uint32_t historical_reference_epoch = 0U;
};

/**
 * @brief 仅在无本次证据写入且历史释放链相符时识别冷Idle基线。
 * 接纳已释放根/释放子尝试；manual模式另允许当前已清除的REST超时根或已知失能超时Release故障历史。
 * 历史失败仍是失败，仅作为读取基线，不能复用历史运动授权。
 * @param value 提供状态访问器的设备Status对象。
 * @param context 当前/历史boot-run、链及本地采集水位。
 * @return
 * bool；要求session/permit为零、已释放Idle且无evidence_lost；manual允许冻结的非零参考。
 * 清除REST超时后的失败ledger仅提供只读基线，不构成成功终态或运动授权。
 */
template <typename Status>
bool historical_released_idle_baseline(const Status &value,
                                       const HistoricalIdleBaselineContext &context) noexcept {
  const bool released_attempt =
      (value.attempt_kind() == 1U && value.parent_run_id() == 0U &&
       value.final_state() == 2U) ||
      (value.attempt_kind() == 3U && value.run_id() > 1U &&
       value.parent_run_id() == value.run_id()-1U && value.final_state() == 8U);
  // ClearFault只清当前适配器故障，故意保留旧失败ledger；不能要求它伪装成功终态。
  const bool cleared_rest_timeout =
      context.manual_reference_mode && value.attempt_kind() == 1U &&
      value.parent_run_id() == 0U && value.final_state() == 4U &&
      value.core_state() == 2U && value.reference_epoch() != 0U &&
      value.cleanup_state() == 3U && value.first_failure_reason() == 0U &&
      value.can_failed_mask() == 0U && value.failure_source() == 0x1067U;
  // 首样本超时已由显式ClearFault清除；历史Fault4/source仍保留，只承认已释放只读基线。
  const bool cleared_first_sample_timeout =
      context.manual_reference_mode && value.attempt_kind() == 1U &&
      value.parent_run_id() == 0U && value.final_state() == 4U &&
      value.core_state() == 2U && value.reference_epoch() != 0U &&
      value.cleanup_state() == 3U && value.first_failure_reason() == 0U &&
      value.can_failed_mask() == 0U && value.failure_source() == 0x1024U;
  // 已知失能超时的Release失败账本在ClearFault后仍保持Fault终态，不能伪装为成功释放。
  const bool cleared_release_timeout =
      context.manual_reference_mode &&
      value.attempt_kind() ==
          DP_PROTOCOL_DPSDK_INTERFACES_MSG_DUMMYPROTOCOLV2_MESSAGE_ATTEMPT_KIND_RELEASE &&
      value.run_id() > 1U && value.parent_run_id() == value.run_id() - 1U &&
      value.final_state() ==
          DP_PROTOCOL_DPSDK_INTERFACES_MSG_DUMMYPROTOCOLV2_MESSAGE_FINAL_STATE_FAULT_LATCHED_AND_RELEASED &&
      value.core_state() == 2U && value.reference_epoch() != 0U &&
      value.cleanup_state() == 3U && value.first_failure_reason() == 0U &&
      value.can_failed_mask() == 0U && value.failure_source() == 0x1064U;
  const bool clean_idle =
      value.boot_epoch() != 0U && value.run_id() != 0U &&
      value.session_id() == 0U &&
      ((value.reference_epoch() == 0U && value.core_state() == 1U) ||
       (context.manual_reference_mode && value.reference_epoch() != 0U &&
        value.core_state() == 2U)) &&
      value.release_committed() && !value.real_motion_permitted() &&
      value.permit_remaining_us() == 0U && !value.evidence_lost();
  const bool cold_start = context.current_boot == 0U &&
      context.historical_run_boot == 0U && context.historical_run_id == 0U &&
      context.last_status_run_id == 0U &&
      context.pages_written == 0U && context.terminal_written_run_id == 0U;
  const bool same_historical_run =
      context.historical_run_id != 0U &&
      context.historical_run_boot == value.boot_epoch() &&
      context.historical_run_id == value.run_id() &&
      (!context.manual_reference_mode ||
       context.historical_reference_epoch == value.reference_epoch()) &&
      std::equal(context.historical_chain.begin(),
                 context.historical_chain.end(),
                 std::begin(value.evidence_chain_sha256()));
  return (released_attempt || cleared_rest_timeout ||
          cleared_first_sample_timeout || cleared_release_timeout) &&
         clean_idle && context.pages_written == 0U &&
         context.terminal_written_run_id == 0U &&
         (cold_start || same_historical_run);
}

// 当前v2 Status携带策略摘要、逐轴限额与控制/发布周期；bootstrap会将这些证据
// 与部署参数逐项匹配。字段缺失或不一致时不能建立位置控制能力。
/**
 * @brief 设备能力中的单轴位置闭区间，单位rad；合法能力要求lower<upper。
 */
struct PositionLimit {
  double lower = 0.0;
  double upper = 0.0;
};

constexpr std::uint64_t kPositionOnlyCapability = 1ULL << 2U;

/**
 * @brief 位置模式完整能力证据：五种身份hash、六轴位置/速度/加速度、控制/状态µs周期和配置掩码。has_*显式标识是否真的取得证据。
 */
struct PositionOnlyCapability {
  std::array<std::uint8_t, 32> contract_hash{};
  std::array<std::uint8_t, 32> model_hash{};
  std::array<std::uint8_t, 32> profile_hash{};
  std::array<std::uint8_t, 32> firmware_hash{};
  std::array<std::uint8_t, 32> policy_hash{};
  std::array<PositionLimit, 6> limits{};
  std::array<double, 6> velocity_max{};
  std::array<double, 6> acceleration_max{};
  std::uint64_t capabilities = 0U;
  std::uint64_t control_timebase_us = 0U;
  std::uint64_t state_publish_period_us = 0U;
  std::uint8_t configured_mask = 0U;
  bool has_policy_hash = true;
  bool has_limits = true;
  bool has_control_timebase = true;
};

/**
 * @brief 核对预期与观测的完整位置能力及全部六轴限制。
 * 预期capabilities必须恰为position-only位；观测可包含其他能力但必须含此位。不能因Status中有部分hash就伪造缺失policy/limits/timebase资格。
 * @param expected 部署规定的位置能力，掩码只能1或63。
 * @param observed 从设备bootstrap证据获得的能力。
 * @return bool；hash、周期、限制精确一致且动态上限有限正数。
 */
inline bool position_only_capability_matches(const PositionOnlyCapability &expected,
                                             const PositionOnlyCapability &observed) noexcept {
  if (!(expected.has_policy_hash && expected.has_limits && expected.has_control_timebase &&
        observed.has_policy_hash && observed.has_limits && observed.has_control_timebase &&
        (expected.configured_mask == 0x01U || expected.configured_mask == 0x3fU) &&
        observed.configured_mask == expected.configured_mask &&
        expected.capabilities == kPositionOnlyCapability &&
        (observed.capabilities & expected.capabilities) == expected.capabilities &&
        expected.control_timebase_us != 0U && expected.state_publish_period_us != 0U &&
        observed.control_timebase_us == expected.control_timebase_us &&
        observed.state_publish_period_us == expected.state_publish_period_us &&
        expected.contract_hash == observed.contract_hash && expected.model_hash == observed.model_hash &&
        expected.profile_hash == observed.profile_hash && expected.firmware_hash == observed.firmware_hash &&
        expected.policy_hash == observed.policy_hash)) return false;
  for (std::size_t axis = 0U; axis < expected.limits.size(); ++axis) {
    if (expected.limits[axis].lower != observed.limits[axis].lower ||
        expected.limits[axis].upper != observed.limits[axis].upper ||
        !std::isfinite(expected.limits[axis].lower) || !std::isfinite(expected.limits[axis].upper) ||
        expected.limits[axis].lower >= expected.limits[axis].upper ||
        !std::isfinite(expected.velocity_max[axis]) || expected.velocity_max[axis] <= 0.0 ||
        !std::isfinite(expected.acceleration_max[axis]) || expected.acceleration_max[axis] <= 0.0 ||
        observed.velocity_max[axis] != expected.velocity_max[axis] ||
        observed.acceleration_max[axis] != expected.acceleration_max[axis]) return false;
  }
  return true;
}

/**
 * @brief 验证六轴目标有限且位于能力位置闭区间。
 * 本辅助函数允许lower==upper的点区间；完整能力匹配函数另要求严格上下界。这里不做速度/碰撞检查。
 * @param position 六轴rad位置。
 * @param capability 有limits且配置掩码63的能力。
 * @return bool；这里只支持完整六轴，单J1掩码返回false。
 */
inline bool position_only_positions_valid(const std::array<double, 6> &position,
                                          const PositionOnlyCapability &capability) noexcept {
  if (!capability.has_limits || capability.configured_mask != 0x3fU) return false;
  for (std::size_t axis = 0U; axis < position.size(); ++axis) {
    const auto &limit = capability.limits[axis];
    if (!std::isfinite(position[axis]) || !std::isfinite(limit.lower) ||
        !std::isfinite(limit.upper) || limit.lower > limit.upper ||
        position[axis] < limit.lower || position[axis] > limit.upper) return false;
  }
  return true;
}

/**
 * @brief 心跳纯判定结果；kStop只是要求调用方启动停止流程，不是已停车回执。
 */
enum class HeartbeatAction : std::uint8_t { kNone, kWarn, kStop };

/**
 * @brief 根据已接纳状态的宿主到达时间判断间隔。
 */
class TransportHeartbeat {
 public:
  /**
   * @brief 保存名义状态发布周期。
   * @param period_us 周期µs，0禁用判定。
   */
  explicit TransportHeartbeat(std::uint64_t period_us) noexcept : period_us_(period_us) {}
  /**
   * @brief 记录已接纳状态到达宿主的时间。
   * @param now_us 宿主steady微秒，非设备produced_at。
   */
  void observe_state(std::uint64_t now_us) noexcept { last_state_us_ = now_us; }
  /**
   * @brief 清掉上次到达时间，等待新状态建立心跳。
   */
  void reset() noexcept { last_state_us_ = 0U; }
  /**
   * @brief 使用旧规则：2周期警告、5周期停止。
   * @param now_us 宿主steady微秒。
   * @return HeartbeatAction；无基线、时间未前进或周期0时kNone。
   */
  HeartbeatAction poll(std::uint64_t now_us) const noexcept {
    if (last_state_us_ == 0U || now_us <= last_state_us_ || period_us_ == 0U) return HeartbeatAction::kNone;
    const auto elapsed = now_us - last_state_us_;
    if (elapsed >= period_us_ * 5U) return HeartbeatAction::kStop;
    if (elapsed >= period_us_ * 2U) return HeartbeatAction::kWarn;
    return HeartbeatAction::kNone;
  }
  /**
   * @brief 使用明确停止年龄而保留2周期警告。
   * @param now_us 宿主steady微秒。
   * @param max_state_age_us 非零停止预算µs。
   * @return elapsed严格大于预算为Stop；否则达到2周期为Warn。
   */
  HeartbeatAction poll(std::uint64_t now_us, std::uint64_t max_state_age_us) const noexcept {
    if (last_state_us_ == 0U || now_us <= last_state_us_ ||
        period_us_ == 0U || max_state_age_us == 0U) return HeartbeatAction::kNone;
    const auto elapsed = now_us - last_state_us_;
    if (elapsed > max_state_age_us) return HeartbeatAction::kStop;
    if (elapsed >= period_us_ * 2U) return HeartbeatAction::kWarn;
    return HeartbeatAction::kNone;
  }

 private:
  /**
   * @brief period与last_state均为宿主µs量；不假设与设备启动时间具有同一epoch。
   */
  std::uint64_t period_us_ = 0U;
  std::uint64_t last_state_us_ = 0U;
};

// At most one message may be awaiting firmware commitment and one newer
// message may wait behind it. Transport acceptance is deliberately not an ack.
/**
 * @brief 一个待设备commit的命令加一个可替换的更新目标。
 * worker单上下文使用，无内部锁；offer覆盖pending使积压容量固定。网络publish接纳并不直接清in_flight，只有同线序号commit或明确丢弃才清。
 */
class LatestOnlyCommandStream {
 public:
  /**
   * @brief 用最新目标替换尚未发送的pending。
   * @param record 新的完整命令代记录。
   */
  void offer(const CommandGenerationRecord &record) noexcept { pending_ = record; }
  /**
   * @brief 仅在没有in-flight时把pending移入在途。
   * @return 要发送的记录或nullopt；转移不代表网络发送成功。
   */
  std::optional<CommandGenerationRecord> begin_send() noexcept {
    if (in_flight_.has_value()) return std::nullopt;
    if (!pending_.has_value()) return std::nullopt;
    in_flight_ = pending_;
    pending_.reset();
    return in_flight_;
  }
  /**
   * @brief 读取当前在途值副本。
   * @return optional记录。
   */
  std::optional<CommandGenerationRecord> in_flight_record() const noexcept { return in_flight_; }
  /**
   * @brief 明确清除当前在途记录。
   * 调用方用于拒绝/错误处理；不顺带清pending，也不声明设备未执行。
   */
  void discard_in_flight() noexcept { in_flight_.reset(); }
  // A reconnect starts a new capability/session/permit/qualification cycle;
  // command intent from the previous link is never resumed.
  /**
   * @brief 同时丢弃在途和pending，阻断跨重连续跑。
   */
  void close_session() noexcept {
    in_flight_.reset();
    pending_.reset();
  }
  /**
   * @brief 只按当前在途记录的线序号确认commit并清在途。
   * @param committed_sequence 已通过更外层身份验证的设备commit序号。
   * @return 匹配且清除时true；本方法不单独核对boot/session。
   */
  bool acknowledge_committed(std::uint32_t committed_sequence) noexcept {
    if (!in_flight_.has_value() ||
        wire_producer_sequence(in_flight_->generation) != committed_sequence) return false;
    in_flight_.reset();
    return true;
  }
  /**
   * @brief 查询在途槽占用。
   * @return bool。
   */
  bool has_in_flight() const noexcept { return in_flight_.has_value(); }
  /**
   * @brief 查询更新目标槽占用。
   * @return bool。
   */
  bool has_pending() const noexcept { return pending_.has_value(); }
  /**
   * @brief 读取pending宿主代数。
   * @return 无pending时0，否则其64位generation。
   */
  std::uint64_t pending_generation() const noexcept {
    return pending_.has_value() ? pending_->generation : 0U;
  }

 private:
  /**
   * @brief 两个可选值容量固定；pending可被新控制更新覆盖，in_flight等待固件commit。
   */
  std::optional<CommandGenerationRecord> in_flight_;
  std::optional<CommandGenerationRecord> pending_;
};

/**
 * @brief 把一次本地发送和后续固件commit身份/序号绑定。
 * 本类实际认可同boot/session/reference/motion及committed_sequence；can_sent_mask参数当前不参与判定。不能将此辅助对象的成功扩张为每轴物理动作完成或CAN总线最终证据。
 */
class GroupSentAck {
 public:
  /**
   * @brief 建立ACK等待身份、旧commit/拒绝计数与饱和截止。
   * @param record 实际发送的控制代与身份。
   * @param sent_at_us 宿主本地publish成功时刻µs。
   * @param timeout_us 等待预算µs。
   * @param reject_count 发送前设备拒绝计数水位。
   * @param committed_before 发送前commit序号。
   */
  void begin(const CommandGenerationRecord &record, std::uint64_t sent_at_us,
             std::uint64_t timeout_us, std::uint32_t reject_count = 0U,
             std::uint32_t committed_before = 0U) noexcept {
    identity_ = record.identity;
    sequence_ = wire_producer_sequence(record.generation);
    reject_count_ = reject_count;
    committed_before_ = committed_before;
    deadline_us_ = sent_at_us > UINT64_MAX - timeout_us ? UINT64_MAX : sent_at_us + timeout_us;
    pending_ = true;
  }
  /**
   * @brief 识别同身份下拒绝数增加但commit仍停在旧水位。
   * @param boot 观测boot。
   * @param session 观测session。
   * @param reference_epoch 观测参考epoch。
   * @param motion_epoch 观测运动epoch。
   * @param committed_sequence 设备当前commit。
   * @param reject_count 设备当前拒绝次数。
   * @param observed_at_us 宿主接收µs，截止时刻仍可接纳。
   * @return bool；不修改等待状态。
   */
  bool command_rejected(std::uint64_t boot, std::uint64_t session,
                        std::uint32_t reference_epoch, std::uint32_t motion_epoch,
                        std::uint32_t committed_sequence, std::uint32_t reject_count,
                        std::uint64_t observed_at_us) const noexcept {
    return pending_ && observed_at_us <= deadline_us_ &&
           boot == identity_.boot && session == identity_.session &&
           reference_epoch == identity_.reference_epoch &&
           motion_epoch == identity_.motion_epoch &&
           committed_sequence == committed_before_ && reject_count > reject_count_;
  }
  /**
   * @brief 按身份和本命令commit序号确认后重置等待。
   * 未命名can_sent_mask参数保留API兼容但不检查，真实SENT证据需原始页/其他校验。
   * @param boot 设备boot。
   * @param session 设备session。
   * @param reference_epoch 参考epoch。
   * @param motion_epoch 运动epoch。
   * @param committed_sequence 设备commit序号。
   * @param observed_at_us 宿主接收µs。
   * @return bool；超过deadline或任一身份/序号不符则false。
   */
  bool observe(std::uint64_t boot, std::uint64_t session, std::uint32_t reference_epoch,
               std::uint32_t motion_epoch, std::uint32_t committed_sequence,
               std::uint8_t /* can_sent_mask */, std::uint64_t observed_at_us) noexcept {
    if (!pending_ || observed_at_us > deadline_us_ ||
        boot != identity_.boot || session != identity_.session ||
        reference_epoch != identity_.reference_epoch || motion_epoch != identity_.motion_epoch ||
        committed_sequence != sequence_) return false;
    reset();
    return true;
  }
  /**
   * @brief 检查等待是否严格超过截止。
   * @param now_us 宿主steady µs。
   * @return pending且now>deadline时true。
   */
  bool overdue(std::uint64_t now_us) const noexcept {
    return pending_ && now_us > deadline_us_;
  }
  /**
   * @brief 查询是否等待commit。
   * @return bool。
   */
  bool pending() const noexcept { return pending_; }
  /**
   * @brief 取得当前/最近等待的32位线序号。
   * @return 序号；reset不会把sequence清零。
   */
  std::uint32_t sequence() const noexcept { return sequence_; }
  /**
   * @brief 清pending与deadline，保留诊断身份/序号字段。
   */
  void reset() noexcept { pending_ = false; deadline_us_ = 0U; }

 private:
  /**
   * @brief 发送身份和线序号；deadline为宿主µs；reject_count/committed_before是发送前水位，pending控制有效性。
   */
  CommandIdentity identity_{};
  std::uint32_t sequence_ = 0U;
  std::uint64_t deadline_us_ = 0U;
  std::uint32_t reject_count_ = 0U, committed_before_ = 0U;
  bool pending_ = false;
};

/**
 * @brief 区分状态失鲜、ACK已超时以及状态先失鲜但ACK尚未到期；最后一种不能声称CAN SENT已确定失败。
 */
struct TransportStopCauses {
  bool state_stale = false;
  bool ack_timed_out = false;
  bool ack_unconfirmed_on_state_stale_stop = false;
};

/**
 * @brief 独立分类状态年龄与ACK期限，不混用两种原因。
 * @param ack 当前ACK等待对象。
 * @param now_us 宿主当前µs。
 * @param last_state_us 最后已接纳状态到达宿主µs，0表示无基线。
 * @param max_state_age_us 状态最大宿主年龄µs。
 * @return TransportStopCauses三个原因标志，供worker选择/记录停止流程。
 */
inline TransportStopCauses classify_transport_stop_causes(
    const GroupSentAck &ack, std::uint64_t now_us, std::uint64_t last_state_us,
    std::uint64_t max_state_age_us) noexcept {
  const bool state_stale = last_state_us != 0U && now_us > last_state_us &&
      now_us - last_state_us > max_state_age_us;
  const bool ack_timed_out = ack.overdue(now_us);
  return {state_stale, ack_timed_out,
          state_stale && ack.pending() && !ack_timed_out};
}

/**
 * @brief 解析硬件参数并核对身份/六轴限制/时间预算。
 * 要求身份hash、控制/状态周期及完整六轴限制；可选状态/ACK/停止预算一旦显式提供须非零且不超过1秒；省略ACK/停止项才保留结构默认0回退。心跳预算须至少2状态周期且不大于max_state_age。解析成功只验证本地配置，在线能力仍须bootstrap核对。
 * @param values URDF硬件字符串参数映射。
 * @param joint_count 1或6，决定所需配置掩码1/63。
 * @param output 非空输出，仅所有检查成功后替换。
 * @return bool；失败保留output原值。
 */
bool parse_transport_parameters(
    const std::unordered_map<std::string, std::string> &values,
    std::size_t joint_count, TransportParameters *output);

/**
 * @brief 宿主采样统计值：计数按对象累计，各*_us为宿主微秒；commands_published仅本地接纳，stop_requests_confirmed才表示停止事务确认，诊断/证据状态另列。
 */
struct TransportStatistics {
  DiagnosticSnapshot diagnostics{};
  std::uint64_t state_callbacks = 0U;
  std::uint64_t state_messages = 0U;
  std::uint64_t status_messages = 0U;
  std::uint64_t stale_run_status_rejections = 0U;
  std::uint64_t rejected_states = 0U;
  std::uint64_t state_decode_rejections = 0U;
  std::uint64_t state_identity_rejections = 0U;
  std::uint64_t state_authority_rejections = 0U;
  std::uint64_t state_sequence_rejections = 0U;
  std::uint64_t state_ingest_rejections = 0U;
  std::uint64_t stop_latched_state_rejections = 0U;
  std::uint64_t commands_published = 0U;
  std::uint64_t rejected_commands = 0U;
  std::uint64_t invalidations = 0U;
  std::uint64_t stop_requests_sent = 0U;
  std::uint64_t stop_request_failures = 0U;
  std::uint64_t stop_requests_confirmed = 0U;
  std::uint64_t unconfirmed_stop_requests = 0U;
  std::uint64_t heartbeat_warnings = 0U;
  std::uint64_t heartbeat_stops = 0U;
  std::uint64_t group_sent_ack_timeouts = 0U;
  std::uint64_t group_sent_ack_misses = 0U;
  std::uint64_t state_stale_stops = 0U;
  std::uint64_t raw_idle_reobservations = 0U;
  // Stop began with an unconfirmed ACK before its timeout; CAN SENT is unknown.
  std::uint64_t ack_unconfirmed_on_state_stale_stop = 0U;
  std::uint64_t bootstrap_rejections = 0U;
  std::uint64_t stale_payload_rejections = 0U;
  std::uint64_t clock_skew_rejections = 0U;
  std::uint64_t state_heartbeat_gap_stops = 0U;
  std::uint64_t last_state_gap_us = 0U;
  std::uint64_t raw_evidence_written = 0U;
  std::uint64_t raw_evidence_pages_written = 0U;
  std::uint64_t raw_evidence_last_terminal_written_run = 0U;
  std::uint8_t raw_evidence_failure = 0U;
  bool ready = false;
  bool initialization_failed = false;
};

// Context, node, publishers, subscriptions and executor live exclusively on
// the worker thread. start/stop are non-RT lifecycle operations.
/**
 * @brief 持有独占worker与实现对象的非可复制生命周期封装。
 * system是借用引用，必须活过transport；Impl独占Context/节点/执行器及证据collector，管理channel共享句柄可在worker退出后保留只读诊断。start/stop会创建/等待线程，属于非实时生命周期接口。
 */
class DeviceTransport final {
 public:
  /**
   * @brief 绑定硬件系统并复制已解析配置。
   * @param system 生命周期长于本对象的目标系统。
   * @param parameters 端点/身份/策略限制及证据路径值。
   */
  DeviceTransport(GuardedZenbeddedSystem &system, TransportParameters parameters);
  /**
   * @brief 析构调用stop等待worker及证据writer结束。
   */
  ~DeviceTransport();
  /**
   * @brief 禁止复制通信所有者，避免两对象共享同一worker资源。
   */
  DeviceTransport(const DeviceTransport &) = delete;
  /**
   * @brief 禁止复制赋值通信所有者。
   */
  DeviceTransport &operator=(const DeviceTransport &) = delete;
  /**
   * @brief 启动证据采集及通信worker，不同步等待在线授权。
   * 返回true不等于ready、设备连接、参考、许可或ROS硬件已激活；具体结果从统计/管理channel观察。
   * @return true线程已启动或已运行；collector/线程创建失败false。
   */
  bool start();
  /**
   * @brief 有界等待已有停止确认机会，再结束worker与证据writer。
   * 等待预算按独立stop_confirmation_timeout或状态年龄回退；退出后检查不完整/未对账原始证据。函数void，不能把返回当作设备释放已确认。
   */
  void stop();
  /**
   * @brief 复制原子计数及证据/诊断状态。
   * @return TransportStatistics快照；各计数独立采样，不是跨字段事务。
   */
  TransportStatistics statistics() const noexcept;
  /**
   * @brief 读取对象生命周期首错和计数。
   * @return DiagnosticSnapshot值副本。
   */
  DiagnosticSnapshot diagnostic_snapshot() const noexcept;
  /**
   * @brief 读取固定首份本地参数原始诊断。
   * @return optional blob；LOCAL_PUBLISH_ARGUMENT不是硬件收据。
   */
  std::optional<DiagnosticBlob> diagnostic_blob() const noexcept;
  /**
   * @brief 按索引读取固定追加诊断槽。
   * @param index 槽索引，容量由DiagnosticJournal限定。
   * @return optional详情，未发布/越界为空。
   */
  std::optional<DiagnosticDetail> diagnostic_detail_record(std::size_t index) const noexcept;
  /**
   * @brief 尝试取得注册且身份匹配的共享管理channel。
   * 无名参数是期望TransportParameters；函数只try_lock不等待，但句柄存在不表示channel.ready或授权有效。
   * @return 共享句柄，注册锁忙/无对象/端点设备客户端profile掩码不符时为空。
   */
  static std::shared_ptr<DeviceManagementChannel> management_channel(const TransportParameters &) noexcept;

 private:
  struct Impl;
  /**
   * @brief 唯一拥有的实现；内部worker/collector管理资源，禁止从RT调用start/stop。
   */
  std::unique_ptr<Impl> impl_;
};
/**
 * @brief 读取通信宿主steady时钟。
 * @return steady_clock epoch以来µs；不是ROS仿真时钟或设备启动时刻。
 */
std::uint64_t transport_steady_now_us() noexcept;
/**
 * @brief 以饱和加法建立宿主状态失鲜截止。
 * @param produced_at_us 调用方选择的同域锚；worker实际传宿主接收时刻。
 * @param max_age_us 允许年龄µs。
 * @return 锚+预算，溢出返回UINT64_MAX。
 */
inline std::uint64_t saturating_state_deadline(std::uint64_t produced_at_us,
                                              std::uint64_t max_age_us) noexcept {
  return produced_at_us > UINT64_MAX - max_age_us ? UINT64_MAX : produced_at_us + max_age_us;
}
}  // namespace dummy_ros_control

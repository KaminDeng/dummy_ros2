/**
 * @file device_transport.cpp
 * @brief ROS控制硬件的独占通信worker、管理事务、源时钟校验及原始证据交接。
 * 控制器只在进程内产生rad目标快照；本文件的worker拥有dp_comm Context/节点/执行器，读取有限发布队列并核对授权、票据和设备commit。Status用于能力/生命周期，State用于实测及源时序，Evidence用于原页链；三者不能互相替代。
 * 管理提交经历本地接纳、设备Pending/COMMITTED、匹配新鲜状态等不同阶段；Stop还需同operation的释放Idle确认。collector运行期记录由私有writer写出/flush（start/stop另有文件头/收尾I/O），不把本地enqueue或flush冒充USB/CAN SENT、物理到位或断电持久性。
 */
#include "dummy_ros_control/device_transport.hpp"
#include "dummy_ros_control/diagnostic_journal.hpp"

#include <algorithm>
#include <atomic>
#include <charconv>
#include <chrono>
#include <cstring>
#include <optional>
#include <random>
#include <sstream>
#include <thread>

#include "dp_comm/dp_comm.hpp"
#include "dummy_ref/wire_contract.hpp"
#include "dummy_ros_control/guarded_zenbedded_system.hpp"
#include "dummy_ros_control/command_channel.hpp"
#include "dummy_ros_control/device_management_channel.hpp"
#include "dummy_ros_control/state_validator.hpp"
#include "dummy_ros_control/raw_evidence_collector.hpp"
#include "dummy_ros_control/management_admission_retry.hpp"

namespace dummy_ros_control {
namespace {
/**
 * @brief wire为生成codec与服务类型的别名；以下kWire常量对应设备协议编号，不是ROS lifecycle枚举。
 */
namespace wire = dp::iface;
using State = wire::DummyJointStateV2;
using Status = wire::DummyDeviceStatusV2;
using Evidence = wire::DummyAttemptEvidenceV2;
using Command = wire::DummyJointCommandV2;
using Management = dp::protocol::services::DummyManageV2;
constexpr std::uint16_t kWireEnablingHold = 3U;
constexpr std::uint16_t kWireHold = 4U;
constexpr std::uint16_t kWireDisabling = 8U;
constexpr std::uint16_t kWireStopping = 7U;
constexpr std::uint16_t kWireDisableAndRelease = 10U;
constexpr std::uint16_t kWireGetOperation = 12U;
constexpr std::uint16_t kWirePendingResult = 23U;
/**
 * @brief 进程内弱注册表只定位与参数身份一致的共享管理channel，不拥有transport线程；lookup采用try_lock，锁忙返回空。
 */
std::mutex channel_registry_mutex;
std::weak_ptr<DeviceManagementChannel> channel_registry;

/**
 * @brief 建立进程级停止请求ID分配器。
 * 随机设备只在首次初始化时使用；Impl构造主动触发，避免第一次Stop路径才进行随机源初始化。它不是设备operation_id。
 * @return 静态原子计数器引用；随机非零起点避免与外部请求序列简单重合。
 */
std::atomic<std::uint64_t> &stop_request_ids() {
  static std::atomic<std::uint64_t> ids([] {
    std::random_device random;
    const auto seed = (static_cast<std::uint64_t>(random()) << 32U) | random();
    return seed == 0U ? 1U : seed;
  }());
  return ids;
}

/**
 * @brief 用CAS分配独立Stop/Query请求ID。
 * 竞争失败重新读取，不在此创建网络请求；request_seq另由共享channel分配。
 * @return 分配的非零ID；计数溢出至0后保留0作为耗尽失败。
 */
std::uint64_t take_stop_request_id() noexcept {
  auto &ids = stop_request_ids();
  auto current = ids.load(std::memory_order_relaxed);
  while (current != 0U && !ids.compare_exchange_weak(
      current, current + 1U, std::memory_order_relaxed)) {}
  return current;
}

/**
 * @brief 逐对十六进制解析非零16字节设备/客户端ID。
 * @param text 必须恰32个十六进制字符。
 * @param id 非空16字节输出，失败时可能已写入部分，调用者使用临时配置承接。
 * @return bool；全零或任何解析/长度失败为false。
 */
bool parse_id(const std::string &text, std::array<std::uint8_t, 16> *id) {
  if (text.size() != 32U) return false;
  unsigned combined = 0U;
  for (std::size_t index = 0; index < id->size(); ++index) {
    unsigned value = 0U;
    const char *begin = text.data() + index * 2U;
    const auto parsed = std::from_chars(begin, begin + 2, value, 16);
    if (parsed.ec != std::errc{} || parsed.ptr != begin + 2) return false;
    (*id)[index] = static_cast<std::uint8_t>(value);
    combined |= value;
  }
  return combined != 0U;
}

/**
 * @brief 解析非零32字节身份摘要。
 * @param text 必须恰64个十六进制字符。
 * @param hash 非空输出数组，失败不保证保留旧内容。
 * @return bool；这里只核对格式/非零，不校验对应文件内容。
 */
bool parse_hash(const std::string &text, std::array<std::uint8_t, 32> *hash) {
  if (text.size() != 64U) return false;
  unsigned combined = 0U;
  for (std::size_t index = 0U; index < hash->size(); ++index) {
    unsigned value = 0U;
    const char *begin = text.data() + index * 2U;
    const auto parsed = std::from_chars(begin, begin + 2, value, 16);
    if (parsed.ec != std::errc{} || parsed.ptr != begin + 2) return false;
    (*hash)[index] = static_cast<std::uint8_t>(value);
    combined |= value;
  }
  return combined != 0U;
}

/**
 * @brief 读取完整六轴有限数值且拒绝尾随额外数据。
 * 不在此解释单位；调用字段分别为rad、rad/s或rad/s²。失败可能改部分临时数组。
 * @param text 六个数，可由流空白或紧邻逗号分隔。
 * @param values 非空六轴输出数组。
 * @return bool；每项必须有限，六项后仅允许空白直到EOF。
 */
bool parse_six_doubles(const std::string &text, std::array<double, 6> *values) {
  std::istringstream input(text);
  for (std::size_t axis = 0U; axis < values->size(); ++axis) {
    if (!(input >> (*values)[axis]) || !std::isfinite((*values)[axis])) return false;
    if (axis + 1U != values->size() && input.peek() == ',') input.get();
  }
  input >> std::ws;
  return input.eof();
}

/**
 * @brief 解析完整非零uint32十进制参数。
 * 因此可选预算显式传字符串0不会走默认回退；只有省略该字段才保留结构的默认0。
 * @param text 不带未消费字符的整数文本。
 * @param value 非空输出地址。
 * @return bool；0、溢出或未完全解析均拒绝。
 */
bool parse_number(const std::string &text, std::uint32_t *value) {
  const auto parsed = std::from_chars(text.data(), text.data() + text.size(), *value);
  return parsed.ec == std::errc{} && parsed.ptr == text.data() + text.size() && *value != 0U;
}
/**
 * @brief 按模2^32检查同水位或较新序号。
 * @param value 候选epoch/序号。
 * @param reference 参考水位。
 * @return 模差小于2^31为true，包含相等；与严格sequence_newer不同。
 */
bool sequence_at_or_after(std::uint32_t value, std::uint32_t reference) {
  return static_cast<std::uint32_t>(value - reference) < 0x80000000U;
}
/**
 * @brief 标记会改变运动/参考/资格epoch的管理操作。
 * 这是后续响应更新授权水位的分类表，不表示仅凭操作号即可执行；查询还必须绑定先前请求。
 * @param operation wire管理操作编号。
 * @return 列出的Capture/Clear/Prepare/Start/Complete/Cancel/Release/清故障及资格操作为true。
 */
bool motion_authorizing_operation(std::uint16_t operation) noexcept {
  switch (operation) {
    case 3U:   // Capture reference
    case 4U:   // Clear reference
    case 6U:   // Prepare hold
    case 7U:   // Start motion
    case 8U:   // Complete motion
    case 9U:   // Cancel motion
    case 10U:  // Disable and release
    case 11U:  // Clear fault
    case 16U:  // Confirm manual home
    case 17U:  // Begin qualification
    case 18U:  // Validate rest
    case 19U:  // Complete qualification
      return true;
    default:
      return false;
  }
}
/**
 * @brief 当前worker线程借用的固定诊断journal；非此scope不输出detail，不跨线程自动传播。
 */
thread_local DiagnosticJournal* worker_diagnostics=nullptr;
/**
 * @brief 临时安装并在退出时恢复当前线程诊断目标。
 * journal由管理channel拥有，scope不释放它；可嵌套，不能据scope存在推断设备已连接。
 */
struct WorkerDiagnosticScope {
  DiagnosticJournal* prior;
  /**
   * @brief 保存旧指针并安装本worker的journal。
   * @param journal 寿命覆盖scope的对象引用。
   */
  explicit WorkerDiagnosticScope(DiagnosticJournal& journal) noexcept :prior(worker_diagnostics) {worker_diagnostics=&journal;}
  /**
   * @brief 恢复进入前的同线程诊断目标。
   */
  ~WorkerDiagnosticScope() {worker_diagnostics=prior;}
};
/**
 * @brief 把格式与值参数复制到固定诊断槽。
 * 没有scope就不记录；worker不在此做printf格式化或stderr写入，详情开关和容量由journal控制。
 * @param format 调用期有效的C格式文本。
 * @param args 数值或C文本值参数包，journal复制后不借用其指针。
 */
template<class... Args> void diagnostic_detail(const char* format,Args... args) noexcept {
  if(worker_diagnostics)worker_diagnostics->detail(format,args...);
}
/**
 * @brief 提取管理请求期待的设备身份与请求序号用于日志。
 * @param request 已解码请求。
 * @return DiagnosticIdentity；valid=true只代表请求字段已提取，不是在线身份验证成功。
 */
DiagnosticIdentity request_diagnostic_identity(const Management::Request& request) noexcept {
  return {request.expected_boot_epoch(),request.expected_session_id(),request.expected_reference_epoch(),request.expected_motion_epoch(),
          request.request_seq(),request.expected_management_epoch(),true};
}
/**
 * @brief 为collector固定客户端及四类契约产物hash。
 * @param parameters 已解析传输配置。
 * @return RawEvidenceIdentity值；编译策略hash仍在bootstrap核对，不在此首链seed结构内。
 */
RawEvidenceIdentity raw_evidence_identity(const TransportParameters &parameters) noexcept {
  return {parameters.client_instance_id, parameters.contract_hash, parameters.model_hash,
          parameters.profile_hash, parameters.firmware_hash};
}
/**
 * @brief 尝试从固定请求缓冲取日志请求ID。
 * @param pending 具有固定codec长度的管理请求值。
 * @return request_id，codec失败为0。
 */
std::uint64_t management_request_id(const DeviceManagementRequest &pending) noexcept {
  Management::Request decoded;
  return wire::DummyManageV2RequestCodec::decode(pending.request.data(),pending.request.size(),&decoded)
      ==wire::CodecResult::kOk ? decoded.request_id() : 0U;
}
/**
 * @brief 记录管理交接/准入失败并限额保存可选详情。
 * 固定首拒独立于细节限额；async/reply/其他交接三类详情各最多128次，全进程累计。此日志不证明设备收到了请求。
 * @param pending 需要说明的管理事务。
 * @param reason 非空原因文本；含would_block视为暂时准入而不计硬拒绝。
 * @param result dp_comm本地结果，默认Ok用于无额外结果的路径。
 */
void log_management_handoff(const DeviceManagementRequest &pending,const char *reason,
                            dp::comm::Result result=dp::comm::Result::kOk) noexcept {
  if(worker_diagnostics && std::strstr(reason,"would_block")==nullptr) {
    Management::Request decoded;
    const bool valid=wire::DummyManageV2RequestCodec::decode(pending.request.data(),pending.request.size(),&decoded)==wire::CodecResult::kOk;
    worker_diagnostics->reject(DiagnosticReason::kManagementRejected,{},
      valid ? request_diagnostic_identity(decoded) : DiagnosticIdentity{},transport_steady_now_us(),reason,static_cast<std::uint32_t>(result));
  }
  static std::atomic<unsigned> async_count{0U}, reply_count{0U}, handoff_count{0U};
  auto &count=std::strcmp(reason,"async_request_failed")==0 ? async_count :
      std::strcmp(reason,"reply_failed")==0 ? reply_count : handoff_count;
  if(count.fetch_add(1U,std::memory_order_relaxed)>=128U) return;
  diagnostic_detail("MANAGE_WORKER_DIAG reason=%s request_id=%llu result=%u\n",
      reason,static_cast<unsigned long long>(management_request_id(pending)),
      static_cast<unsigned>(result));
}
}  // namespace

/**
 * @brief 读取本传输唯一宿主计时时基。
 * @return steady_clock epoch以来µs；与ROS仿真时间和设备produced_at_us均不同域。
 */
std::uint64_t transport_steady_now_us() noexcept {
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}

/**
 * @brief 在临时对象中解析URDF硬件配置并一次性提交输出。
 * 必需端点、设备/客户端、五类hash、完整六轴限制和非零控制/发布周期；所有数值有限，上下界严格递增，速度/加速度正。可选状态/ACK/Stop预算显式出现须非零且≤1秒；心跳预算在2发布周期与状态年龄之间。raw路径此处只做基本绝对路径/长度检查，真正无链接独占创建由collector负责。
 * @param values 硬件参数字符串字典。
 * @param joint_count 只允许1或6，掩码必须相应为1/63。
 * @param output 非空输出指针，只有所有验证通过才替换。
 * @return bool；失败不会把部分解析配置暴露到output。
 */
bool parse_transport_parameters(
    const std::unordered_map<std::string, std::string> &values,
    std::size_t joint_count, TransportParameters *output) {
  if (!output || (joint_count != 1U && joint_count != 6U)) return false;
  /**
   * @brief 配置字段齐全只是本地启动前提；之后Status仍须逐项证明同hash、位置模式能力、限制及周期，不能用配置值冒充观测。
   */
  for (const char *key : {"endpoint", "device_id", "client_instance_id", "profile_id", "configured_mask",
                          "contract_sha256", "model_sha256", "profile_sha256", "firmware_sha256",
                          "compiled_policy_sha256", "position_min_rad", "position_max_rad",
                          "velocity_max_rad_s", "acceleration_max_rad_s", "control_period_us",
                          "state_publish_period_us"})
    if (values.find(key) == values.end()) return false;
  TransportParameters parsed;
  parsed.endpoint = values.at("endpoint");
  std::uint32_t mask = 0U;
  if (parsed.endpoint.empty() || parsed.endpoint.size() > 256U ||
      parsed.endpoint.find_first_of("\n\r\t ") != std::string::npos ||
      !parse_id(values.at("device_id"), &parsed.device_id) ||
      !parse_id(values.at("client_instance_id"), &parsed.client_instance_id) ||
      !parse_number(values.at("profile_id"), &parsed.profile_id) ||
      !parse_number(values.at("configured_mask"), &mask) ||
      mask != (joint_count == 1U ? 1U : 63U)) return false;
  parsed.configured_mask = static_cast<std::uint8_t>(mask);
  std::uint32_t control_period_us = 0U;
  std::uint32_t state_period_us = 0U;
  if (!parse_hash(values.at("contract_sha256"), &parsed.contract_hash) ||
      !parse_hash(values.at("model_sha256"), &parsed.model_hash) ||
      !parse_hash(values.at("profile_sha256"), &parsed.profile_hash) ||
      !parse_hash(values.at("firmware_sha256"), &parsed.firmware_hash) ||
      !parse_hash(values.at("compiled_policy_sha256"), &parsed.policy_hash) ||
      !parse_six_doubles(values.at("position_min_rad"), &parsed.position_min) ||
      !parse_six_doubles(values.at("position_max_rad"), &parsed.position_max) ||
      !parse_six_doubles(values.at("velocity_max_rad_s"), &parsed.velocity_max) ||
      !parse_six_doubles(values.at("acceleration_max_rad_s"), &parsed.acceleration_max) ||
      !parse_number(values.at("control_period_us"), &control_period_us) ||
      !parse_number(values.at("state_publish_period_us"), &state_period_us)) return false;
  parsed.control_period_us = control_period_us;
  parsed.state_publish_period_us = state_period_us;
  for (std::size_t axis = 0U; axis < parsed.position_min.size(); ++axis)
    if (parsed.position_min[axis] >= parsed.position_max[axis] || parsed.velocity_max[axis] <= 0.0 ||
        parsed.acceleration_max[axis] <= 0.0) return false;
  const auto age = values.find("max_state_age_us");
  if (age != values.end()) {
    std::uint32_t age_us = 0U;
    if (!parse_number(age->second, &age_us) || age_us > 1000000U) return false;
    parsed.max_state_age_us = age_us;
  }
  const auto ack_timeout = values.find("group_sent_ack_timeout_us");
  if (ack_timeout != values.end()) {
    std::uint32_t timeout_us = 0U;
    if (!parse_number(ack_timeout->second, &timeout_us) || timeout_us > 1000000U)
      return false;
    parsed.group_sent_ack_timeout_us = timeout_us;
  }
  const auto stop_timeout = values.find("stop_confirmation_timeout_us");
  if (stop_timeout != values.end()) {
    std::uint32_t timeout_us = 0U;
    // 显式0与省略一致：使用max_state_age_us作为停止确认总预算。
    if ((stop_timeout->second != "0" &&
         !parse_number(stop_timeout->second, &timeout_us)) ||
        timeout_us > 1000000U)
      return false;
    parsed.stop_confirmation_timeout_us = timeout_us;
  }
  /**
   * @brief heartbeat预算与ACK/Stop确认预算分别保存；未配置心跳预算用旧5周期规则，不自动延长状态有效截止。
   */
  const auto heartbeat_budget = values.find("heartbeat_stop_budget_us");
  if (heartbeat_budget != values.end()) {
    std::uint32_t budget_us = 0U;
    if (!parse_number(heartbeat_budget->second, &budget_us) ||
        budget_us > parsed.max_state_age_us ||
        budget_us < parsed.state_publish_period_us * 2U) return false;
    parsed.heartbeat_stop_budget_us = budget_us;
  }
  const auto manual_mode = values.find("manual_reference_mode");
  if (manual_mode != values.end()) {
    if (manual_mode->second != "true" && manual_mode->second != "false")
      return false;
    parsed.manual_reference_mode = manual_mode->second == "true";
    if (parsed.manual_reference_mode && joint_count != 6U)
      return false;
  }
  const auto raw_evidence_path = values.find("raw_evidence_path");
  if (raw_evidence_path != values.end()) {
    if (raw_evidence_path->second.empty() || raw_evidence_path->second.front() != '/' ||
        raw_evidence_path->second.size() > 256U ||
        raw_evidence_path->second.find_first_of("\n\r\t") != std::string::npos) return false;
    parsed.raw_evidence_path = raw_evidence_path->second;
  }
  *output = std::move(parsed);
  return true;
}

/**
 * @brief 一个硬件对象的通信、状态水位和管理/证据状态机。
 * 除对外atomic统计和锁保护channel外，大部分字段只由本worker回调/循环访问；所有Context及RequestHandle都在run_connected局部同线程生灭。system为借用，management共享持有；collector writer独立线程，不拥有运动控制。
 */
struct DeviceTransport::Impl {
  /**
   * @brief system必须活过Impl；parameters冻结部署值；management共享保留诊断，raw_evidence独占捕获文件/writer，evidence_bindings有限绑定run身份。
   */
  GuardedZenbeddedSystem &system;
  const TransportParameters parameters;
  const std::shared_ptr<DeviceManagementChannel> management;
  RawEvidenceCollector raw_evidence;
  RawEvidenceBindings evidence_bindings;
  /**
   * @brief 故障清理lane使用正常Running流预先建立的源时钟/boot锚。故障原始页可归档和辅助清理，但绝不因此开放正常命令或管理admission。
   */
  RawFaultProducerClock raw_fault_clock{};
  std::uint64_t raw_fault_trusted_boot=0U,raw_fault_trusted_produced=0U,raw_fault_trusted_anchor_received=0U;
  std::uint32_t raw_fault_trusted_sequence=0U;
  std::int64_t raw_fault_trusted_offset=0,raw_fault_trusted_anchor_offset=0;
  DiagnosticJournal& diagnostics=management->diagnostics;
  /**
   * @brief running控制线程寿命；ready是通信端点可用，failed是当前连接失败观察，均不能当成设备授权标志。统计计数按对象寿命积累。
   */
  std::atomic<bool> running{false}, ready{false}, failed{false};
  std::thread worker;
  std::atomic<std::uint64_t> states{0}, statuses{0}, stale_run_status_rejections{0};
  std::atomic<std::uint64_t> rejected_states{0}, commands{0};
  std::atomic<std::uint64_t> state_callbacks{0}, state_decode_rejections{0};
  std::atomic<std::uint64_t> state_identity_rejections{0}, state_authority_rejections{0};
  std::atomic<std::uint64_t> state_sequence_rejections{0}, state_ingest_rejections{0};
  std::atomic<std::uint64_t> stop_latched_state_rejections{0}, group_sent_ack_misses{0};
  std::atomic<std::uint64_t> rejected_commands{0}, invalidations{0};
  std::atomic<std::uint64_t> stop_requests_sent{0}, stop_request_failures{0};
  std::atomic<std::uint64_t> stop_requests_confirmed{0}, unconfirmed_stop_requests{0};
  std::atomic<bool> stop_confirmation_pending{false};
  // Host-only已接纳State目标元数据；64项覆盖名义20ms cadence的250k窗口(最多13项)，不承诺H压力。
  // 未过期槽耗尽则该episode拒绝，不改变command/evidence queue或任何MCU资源。
  /**
   * @brief 同一次Cancel episode已接纳的EnablingHold目标候选：position为rad，produced_us为设备µs，received_us为宿主µs；64槽只覆盖有限时间窗，不动态扩容。
   */
  struct CancelPrimeSource {
    CommandIdentity root{};
    std::uint32_t management=0U, state_seq=0U, profile=0U;
    std::uint8_t mask=0U;
    std::uint64_t produced_us=0U, received_us=0U;
    std::array<double,6> position{};
  };
  /**
   * @brief 候选环一旦需要覆盖仍在250ms内的槽就锁存overflow并拒绝该episode；成功本地enqueue后锁定target，后续重试不得改安全目标。
   */
  std::array<CancelPrimeSource,64> cancel_prime_sources{};
  std::size_t cancel_prime_source_count=0U, cancel_prime_source_next=0U;
  bool cancel_prime_source_overflow=false, cancel_prime_target_latched=false, cancel_prime_diagnostic_emitted=false;
  std::uint64_t cancel_prime_episode_request=0U;
  CancelPrimeSource cancel_prime_target{};
  std::uint8_t cancel_prime_latest_core=0U;

  std::atomic<std::uint64_t> heartbeat_warnings{0}, heartbeat_stops{0}, bootstrap_rejections{0};
  std::atomic<std::uint64_t> group_sent_ack_timeouts{0};
  std::atomic<std::uint64_t> state_stale_stops{0}, ack_unconfirmed_on_state_stale_stop{0};
  std::atomic<std::uint64_t> raw_idle_reobservations{0};
  std::atomic<std::uint64_t> stale_payload_rejections{0}, clock_skew_rejections{0};
  std::atomic<std::uint64_t> state_heartbeat_gap_stops{0}, last_state_gap_us{0};
  /**
   * @brief status缓存最近经身份/水位过滤的设备状态，能力是否匹配另由bootstrap_ready表示；boot为当前连接绑定。run水位和historical_chain区分历史已释放Idle与本次必须采集的尝试。
   */
  Status status{};
  std::uint64_t boot = 0U, producer_anchor_us = 0U;
  std::uint64_t last_status_run_boot = 0U, last_status_run_session = 0U;
  std::uint64_t last_status_run_id = 0U;
  std::uint64_t historical_run_boot = 0U, historical_run_id = 0U;
  std::array<std::uint8_t, 32> historical_chain{};
  std::uint32_t historical_reference_epoch = 0U;
  /**
   * @brief offset=宿主接收µs−设备生产µs，可带符号；minimum偏移估计传输排队额外年龄，初锚还限制长期校正漂移，禁止直接比较不同epoch。
   */
  std::int64_t producer_offset_us = 0, clock_anchor_offset_us = 0;
  std::uint64_t clock_anchor_received_us = 0U;
  /**
   * @brief last_produced为设备µs、last_received/status_received/callback_received为宿主µs；callback计到达，accepted水位只在通过后推进，两者故障定位不能混用。
   */
  std::uint64_t last_produced_us = 0U, last_received_us = 0U;
  std::uint64_t last_status_received_us = 0U;
  std::uint64_t last_callback_received_us = 0U, last_callback_produced_us = 0U;
  std::uint32_t last_callback_sequence = 0U;
  std::uint64_t last_rejection_log_us = 0U;
  std::uint32_t last_sequence = 0U;
  std::uint64_t last_state_boot = 0U;
  std::uint64_t last_state_session = 0U;
  std::uint32_t last_state_reference = 0U, last_state_motion = 0U;
  std::uint32_t last_state_management_epoch = 0U;
  unsigned complete_binding_diag_count=0U;
  unsigned command_prime_reject_diag_count=0U;
  /**
   * @brief 授权motion/management是已绑定管理结果或接受Status建立的水位；不同于仅从任意State看到的epoch，变更时旧命令必须失效。
   */
  std::uint32_t authorized_motion_epoch = 0U;
  std::uint32_t authorized_management_epoch = 0U;
  /**
   * @brief Complete事务摘要：terminal_seq冻结末命令，queue_drained表示宿主队列已消费过，dispatched仅在本地async_request接纳后置位。started_us为宿主两秒总窗口锚，不随Query重启。
   */
  struct PendingComplete {
    std::uint64_t request_id=0U, boot_epoch=0U, session_id=0U, started_us=0U;
    std::uint32_t reference_epoch=0U, motion_epoch=0U, management_epoch=0U, terminal_seq=0U;
    bool dispatched=false, queue_drained=false;
  } pending_complete;
  /**
   * @brief last_ack绑定设备commit，last_processed_hold_generation是宿主已处理代；disposition=1本地同值吞并、2设备ACK，排空快照必须保留区别。
   */
  CommandIdentity last_ack_identity{};
  std::uint32_t last_ack_seq=0U;
  CommandIdentity last_processed_hold_identity{};
  std::uint64_t last_processed_hold_generation=0U;
  std::uint8_t last_processed_hold_disposition=0U;
  std::array<double,6> last_processed_hold_position{};
  bool hold_drain_rejected=false;
  /**
   * @brief 匹配COMMITTED Complete响应的后继身份与终序号；仅收到Status先到EnablingHold不等于此结构已确认。
   */
  struct ConfirmedComplete {
    std::uint64_t boot_epoch=0U, session_id=0U;
    std::uint32_t reference_epoch=0U, motion_epoch=0U, terminal_seq=0U;
  } confirmed_complete;
  /**
   * @brief Complete前run与后继Hold epoch的边界；first_prime_seq/position保留首保持命令，禁止确认期间无新Begin就让不同目标溜入CAN账本。
   */
  struct CompleteBoundary {
    std::uint64_t boot_epoch=0U, session_id=0U;
    std::uint32_t reference_epoch=0U, hold_epoch=0U, terminal_seq=0U, first_prime_seq=0U;
    std::uint64_t prior_run_id=0U;
    std::array<double,6> first_prime_position{};
  } complete_boundary;
  mutable bool complete_boundary_hold_diag_logged=false;
  /**
   * @brief 旧motion完成原页链和Status一致后才开放下一Begin；refresh每秒至多发必要同值保持刷新，仍使用正常票据/ACK，并不凭本地no-op延长设备Hold。
   */
  bool exported_motion_ready=false;
  std::array<std::uint8_t,32> exported_motion_chain{};
  std::uint64_t refresh_active_run_id=0U, last_refresh_acked_us=0U;
  std::uint32_t refresh_in_flight_seq=0U;
  std::array<std::uint8_t,32> last_terminal_chain{};
  std::uint64_t last_terminal_chain_run=0U;
  /**
   * @brief 这里可暂存已通过collector链校验但writer尚未flush的终页摘要；只有后续writer水位才能证明写出，不能混作完整flush回执。
   */
  RawEvidenceTerminalFlushed accepted_completed_terminal{};
  /**
   * @brief 8槽已接纳完成链证人用于拒绝同run后续Status自相矛盾；只是有限历史，不是无界数据库。
   */
  struct CompletedTerminalChain {
    std::uint64_t boot_epoch=0U, session_id=0U, run_id=0U;
    std::array<std::uint8_t,32> chain{};
  };
  std::array<CompletedTerminalChain,8U> completed_terminal_chains{};
  std::size_t next_completed_terminal_chain=0U;
  std::uint64_t last_clean_cancel_boot=0U, last_clean_cancel_session=0U,
                last_clean_cancel_run=0U;
  /**
   * @brief 最近新鲜Hold State的实际位置rad与commit序号；received_us是宿主到达时刻，目标同值判断不得从动画/期望值反推。
   */
  struct ObservedHold {
    std::uint64_t boot_epoch=0U, session_id=0U, received_us=0U;
    std::uint32_t reference_epoch=0U, motion_epoch=0U, committed_seq=0U;
    std::array<double,6> position{};
  } observed_hold;
  /**
   * @brief 票据值来自设备；last_ticket_received记录该票据第一次在匹配身份帧出现的宿主时间，同票据重复State不续50ms发送窗口。
   */
  std::uint64_t last_freshness_ticket = 0U;
  std::uint64_t last_ticket_received_us = 0U;
  std::uint64_t last_motion_request_id = 0U;
  std::uint64_t last_motion_operation_id = 0U;
  bool has_sequence = false;
  /**
   * @brief 命令流最多一个in-flight及一个可替换pending；group_sent_ack等待同身份固件commit，publisher Ok只代表本地接纳。heartbeat只使用通过校验State的宿主到达时间。
   */
  LatestOnlyCommandStream command_stream;
  GroupSentAck group_sent_ack;
  TransportHeartbeat heartbeat;
  bool bootstrap_ready = false;
  bool heartbeat_warned = false;
  bool heartbeat_stopped = false;
  bool command_sent = false;
  bool logged_prime_transfer = false;
  bool logged_copy_state_failure = false;
  bool logged_publish_attempt = false;
  bool logged_publish_success = false;
  std::uint32_t enabling_transfer_log_count = 0U;
  std::uint32_t enabling_copy_failure_log_count = 0U;
  std::uint32_t enabling_publish_log_count = 0U;
  std::uint32_t ack_mismatch_diag_count = 0U;
  std::uint64_t last_enable_visibility_log_us = 0U;
  /**
   * @brief stop_latched封锁普通控制，stop_idle_observed仅记已观察raw Idle；ever_authorized/command_published阻止有历史动作的连接走无owner原始Idle重观察捷径。
   */
  bool stop_latched = false;
  bool stop_idle_observed = false;
  bool ever_authorized_this_boot = false;
  bool command_published_this_boot = false;
  bool last_observation_raw_idle = false;
  /**
   * @brief 允许受严格前后证据限定的“原页先于Status”暂绑定；后续Status、终页链及writer flush必须对账，否则采集粘滞失败。
   */
  std::optional<RawEvidenceBinding> provisional_evidence_binding;
  std::optional<std::array<std::uint8_t, 32U>> provisional_release_chain;
  /**
   * @brief 释放事务根：保存发送前expected epoch及收到COMMITTED后的实际epoch/operation/run；sent/committed/idle_confirmed是三个不同阶段，sent_us固定宿主发送锚。
   */
  struct GateRelease {
    std::uint64_t request_id = 0U, boot_epoch = 0U, session_id = 0U;
    std::uint32_t reference_epoch = 0U, expected_motion_epoch = 0U;
    std::uint32_t expected_management_epoch = 0U;
    std::uint32_t motion_epoch = 0U, management_epoch = 0U;
    bool committed = false;
    bool idle_confirmed = false;
    bool sent = false;
    std::uint64_t sent_us = 0U, operation_id = 0U, run_id = 0U;
    std::uint32_t released_reference_epoch = 0U;
  } gate_release;
  /**
   * @brief 早到Idle State隔离单槽，绑定本次request_id和原接收时刻；不提前写权威状态，COMMITTED后按原年龄重放。
   */
  struct EarlyReleaseIdle {
    State state{};
    std::uint64_t request_id = 0U, received_us = 0U;
  } early_release_idle;
  /**
   * @brief 早到释放Status也只存定长原codec字节单槽；672字节static_assert固定宿主边界，既不改设备资源也不增加队列。
   */
  struct EarlyWorkerStatus {
    std::array<std::uint8_t,wire::DummyDeviceStatusV2Codec::kEncodedSize> raw{};
    std::uint64_t request_id=0U, received_us=0U;
  } early_worker_status;
  static_assert(sizeof(EarlyWorkerStatus)==672U,"Host早Status单槽固定边界");

  /**
   * @brief 16个退役boot拒绝迟到旧启动帧；耗尽时关闭worker而非覆盖历史导致旧boot复活。
   */
  std::array<std::uint64_t, 16> retired_boots{};
  std::size_t retired_count = 0U;

  /**
   * @brief 绑定系统与配置，创建共享channel及collector并登记弱引用。
   * 预热随机Stop ID分配器；注册表仅在旧弱引用已过期时登记本channel，避免另一个活对象被静默覆盖。尚未创建Context或线程。
   * @param target 寿命覆盖本实现的硬件系统。
   * @param config 已解析传输配置，移动保存为const。
   */
  Impl(GuardedZenbeddedSystem &target, TransportParameters config)
      : system(target), parameters(std::move(config)),
        management(std::make_shared<DeviceManagementChannel>(parameters)),
        raw_evidence(raw_evidence_identity(parameters), management.get()),
        heartbeat(parameters.state_publish_period_us) {
    (void)stop_request_ids();
    std::lock_guard<std::mutex> lock(channel_registry_mutex);
    if(channel_registry.expired()) channel_registry=management;
  }

  /**
   * @brief 校验Complete请求恰绑定当前可用motion/management水位。
   * 此检查只准入本地Complete流程，不表示末命令已commit或管理请求已经发送。
   * @param decoded 待发送的已解码管理请求。
   * @return bool；仅operation8、非零boot、同设备客户端/profile/mask及当前身份，且motion未到UINT32_MAX。
   */
  bool matches_complete_authority(const Management::Request &decoded) const noexcept {
    return decoded.operation()==8U && boot!=0U &&
               decoded.expected_boot_epoch()==boot &&
               decoded.expected_session_id()==status.session_id() &&
               decoded.expected_reference_epoch()==status.reference_epoch() &&
               decoded.expected_motion_epoch()==authorized_motion_epoch &&
               decoded.expected_management_epoch()==authorized_management_epoch &&
               decoded.expected_motion_epoch()!=UINT32_MAX &&
               decoded.expected_profile_id()==parameters.profile_id &&
               decoded.configured_mask()==parameters.configured_mask &&
               std::equal(parameters.device_id.begin(),parameters.device_id.end(),decoded.device_id()) &&
               std::equal(parameters.client_instance_id.begin(),parameters.client_instance_id.end(),
                          decoded.client_instance_id());
  }

  // 只有transport实际接纳发送后，才允许该Complete解释后继epoch。
  /**
   * @brief 在async_request实际接纳后记录Complete后继边界。
   * 将未来Hold epoch设为旧motion+1，关闭下一Begin屏障，记录dispatched=true和原始terminal_seq；不能在仅deferred尚未发送时调用它授权后继epoch。
   * @param request 本地发送成功的Complete原请求。
   */
  void observe_sent_complete(const Management::Request &request) noexcept {
    complete_boundary={request.expected_boot_epoch(),request.expected_session_id(),
      request.expected_reference_epoch(),request.expected_motion_epoch()+1U,
      request.terminal_producer_seq()};
    complete_boundary.prior_run_id=status.run_id();
    management->post_complete_begin_ready.store(false,std::memory_order_release);
    pending_complete={request.request_id(),request.expected_boot_epoch(),
      request.expected_session_id(),transport_steady_now_us(),
      request.expected_reference_epoch(),request.expected_motion_epoch(),
      request.expected_management_epoch(),request.terminal_producer_seq(),true,true};
  }

  /**
   * @brief 撤销当前控制/命令链及部分时钟水位，阻断重连自动续跑。
   * 清Complete/refresh/排空/ACK，丢弃pending及发布队列并计拒绝；boot与票据归零，心跳重置。保留诊断首错与对象统计，不发送新网络请求；真正停止须先建立管理根身份。
   * @param retired_authority 可空旧Status；非空只撤销旧授权并保留仍新鲜测量，否则全部传输状态失效。
   * @param reset_watermarks True清授权epoch和源时钟/序号锚，False保留必要因果水位。
   */
  void invalidate(const Status *retired_authority = nullptr,
                  bool reset_watermarks = false) noexcept {
    early_release_idle={};early_worker_status={};
    pending_complete={};
    confirmed_complete={};
    complete_boundary={};
    management->post_complete_begin_ready.store(false,std::memory_order_release);
    complete_boundary_hold_diag_logged=false;
    exported_motion_ready=false;
    exported_motion_chain={};
    accepted_completed_terminal={};
    completed_terminal_chains={};
    next_completed_terminal_chain=0U;
    refresh_active_run_id=last_refresh_acked_us=0U;
    refresh_in_flight_seq=0U;
    observed_hold={};
    last_processed_hold_identity={};
    last_processed_hold_generation=0U;
    last_processed_hold_disposition=0U;
    last_processed_hold_position={};
    hold_drain_rejected=false;
    { std::lock_guard<std::mutex> lock(management->mutex); management->hold_drain={}; }
    CommandIdentity identity{};
    if (retired_authority != nullptr) {
      identity = CommandIdentity{
          retired_authority->boot_epoch(), retired_authority->session_id(),
          retired_authority->reference_epoch(), retired_authority->motion_epoch(), 0U};
    }
    if (retired_authority != nullptr) system.invalidate_authority_state(identity);
    else system.invalidate_transport_state();
    bootstrap_ready = false;
    group_sent_ack.reset();
    command_sent = false;
    command_stream.close_session();
    CommandGenerationRecord record{};
    while (system.take_publish_record(&record)) ++rejected_commands;
    boot = 0U;
    last_motion_request_id = 0U;
    last_motion_operation_id = 0U;
    if (reset_watermarks) {
      authorized_motion_epoch = 0U;
      authorized_management_epoch = 0U;
      has_sequence = false;
      last_state_management_epoch=0U;
      producer_anchor_us = 0U;
      producer_offset_us = 0;
      clock_anchor_offset_us = 0;
      clock_anchor_received_us = 0U;
    }
    last_received_us = 0U;
    last_freshness_ticket = 0U;
    last_ticket_received_us = 0U;
    heartbeat.reset();
    ++invalidations;
  }

  /**
   * @brief 核对消息共同的设备ID/profile/mask及非零boot。
   * @tparam Message 生成wire消息类型。
   * @param message 具备对应访问器的State或Status。
   * @return bool；不在此检查schema、session或hash能力。
   */
  template <typename Message>
  bool matches_common(const Message &message) const noexcept {
    return
           std::equal(parameters.device_id.begin(), parameters.device_id.end(), message.device_id()) &&
           message.profile_id() == parameters.profile_id &&
           message.configured_mask() == parameters.configured_mask && message.boot_epoch() != 0U;
  }

  /**
   * @brief 在共同身份上增加当前Status schema检查。
   * @param message 候选Status。
   * @return bool；成功仍须后续能力/hash/epoch/run检查。
   */
  bool matches_status(const Status &message) const noexcept {
    return message.schema_major() == 2U && message.schema_minor() == dummy_ref::protocol::kSchemaMinor && matches_common(message);
  }

  /**
   * @brief 核对State schema与共同设备身份。
   * @param message 候选State。
   * @return bool；不单独授权session或校验时间。
   */
  bool matches_state(const State &message) const noexcept {
    return message.schema_major() == 2U && message.schema_minor() == dummy_ref::protocol::kSchemaMinor && matches_common(message);
  }

  /**
   * @brief 把Status尝试身份登记给collector，或识别严格历史Idle。
   * 历史已释放Idle仅在没有本轮写页/终页且旧链相同时记历史基线；它不要求重采历史原页，也不开放控制。
   * @param value 已匹配共同身份/能力的Status候选。
   * @return bool；evidence_lost拒绝，run0无新尝试，普通run需绑定表和expect_run均接纳。
   */
  bool bind_evidence_status(const Status &value) noexcept {
    if (value.evidence_lost()) return false;
    if (value.run_id() == 0U) return true;
    if (historical_released_idle_baseline(
            value,
            {boot, historical_run_boot, historical_run_id, last_status_run_id,
             raw_evidence.pages_written(),
             raw_evidence.terminal_written_run_id(), historical_chain,
             parameters.manual_reference_mode, historical_reference_epoch})) {
      if (historical_run_id == 0U) {
        historical_run_boot = value.boot_epoch();
        historical_run_id = value.run_id();
        historical_reference_epoch = value.reference_epoch();
        std::copy(std::begin(value.evidence_chain_sha256()),
                  std::end(value.evidence_chain_sha256()), historical_chain.begin());
      }
      return true;
    }
    if (value.boot_epoch() == historical_run_boot &&
        value.run_id() == historical_run_id) return false;
    std::uint64_t evidence_owner = value.session_id();
    // Disable可能在两个Status之间完成。仅新Release证据子链继承已绑定父owner；session0绝不恢复控制授权。
    if (parameters.manual_reference_mode && evidence_owner == 0U &&
        find_evidence_binding(value.run_id()) == nullptr) {
      const auto *parent = find_evidence_binding(value.parent_run_id());
      const bool released_child =
          parent != nullptr && parent->session_id != 0U &&
          parent->boot_epoch == status.boot_epoch() &&
          value.boot_epoch() == status.boot_epoch() && parent->run_id == status.run_id() &&
          parent->session_id == status.session_id() &&
          parent->run_id != UINT64_MAX &&
          value.run_id() == parent->run_id + 1U &&
          value.attempt_kind() ==
              DP_PROTOCOL_DPSDK_INTERFACES_MSG_DUMMYPROTOCOLV2_MESSAGE_ATTEMPT_KIND_RELEASE &&
          value.final_state() ==
              DP_PROTOCOL_DPSDK_INTERFACES_MSG_DUMMYPROTOCOLV2_MESSAGE_FINAL_STATE_RELEASED &&
          value.core_state() == 2U && value.reference_epoch() != 0U &&
          value.reference_epoch() == status.reference_epoch() &&
          status.motion_epoch() != UINT32_MAX &&
          value.motion_epoch() == status.motion_epoch() + 1U &&
          status.management_epoch() != UINT32_MAX &&
          value.management_epoch() == status.management_epoch() + 1U &&
          value.profile_id() == status.profile_id() &&
          value.configured_mask() == status.configured_mask() &&
          value.release_committed() && value.cleanup_state() == 3U &&
          value.first_failure_reason() == 0U && value.can_failed_mask() == 0U &&
          !value.real_motion_permitted() && value.permit_remaining_us() == 0U &&
          std::any_of(std::begin(value.evidence_chain_sha256()),
                      std::end(value.evidence_chain_sha256()),
                      [](auto b) { return b != 0U; });
      if (released_child)
        evidence_owner = parent->session_id;
    }
    // Acquire/恢复已有参考可以在首CAN前建立/重基准attempt；终页携创建epoch，
    // EnableHold首CAN则携后继epoch。只认经此函数绑定身份的零组Idle根Status。
    const bool pre_motion_origin = value.session_id() != 0U &&
        value.reference_epoch() != 0U && value.motion_epoch() != 0U &&
        value.core_state() == 2U && value.attempt_kind() == 1U &&
        value.attempt_id() == value.run_id() &&
        value.parent_run_id() == 0U && value.final_state() == 0U &&
        value.group_count() == 0U && !value.motion_complete() && !value.release_committed() &&
        value.failure_source() == 0U && value.failure_axis_mask() == 0U &&
        value.first_failure_reason() == 0U && value.can_failed_mask() == 0U &&
        std::all_of(std::begin(value.evidence_chain_sha256()),
            std::end(value.evidence_chain_sha256()), [](auto byte) { return byte == 0U; });
    return evidence_bindings.observe({value.run_id(), value.boot_epoch(),
                                      evidence_owner, value.parent_run_id(),
                                      value.attempt_kind()},
                                     raw_evidence.terminal_written_run_id()) &&
           raw_evidence.expect_run(value.run_id(), pre_motion_origin ? value.motion_epoch() : 0U);
  }

  /**
   * @brief 查找当前有限证据身份表中的run。
   * @param run_id 设备尝试ID。
   * @return 借用槽指针或nullptr；后续observe可回收，立即使用而不跨更新保存。
   */
  const RawEvidenceBinding *find_evidence_binding(std::uint64_t run_id) const noexcept {
    return evidence_bindings.find(run_id);
  }

  /**
   * @brief 比较消息与当前Status的session/reference/motion。
   * @tparam Message 生成消息类型。
   * @param message 具有三类身份访问器的消息。
   * @return bool；不比较boot/profile/hash，调用方须先核共同身份。
   */
  template <typename Message>
  bool matches_authority(const Message &message) const noexcept {
    return message.session_id() == status.session_id() &&
           message.reference_epoch() == status.reference_epoch() &&
           message.motion_epoch() == status.motion_epoch();
  }

  /**
   * @brief 用已授权motion水位匹配State会话/参考。
   * @param message 候选State。
   * @return bool；与matches_authority相比motion取authorized_motion_epoch，可处理管理响应先于Status的时序。
   */
  bool matches_state_authority(const State &message) const noexcept {
    return message.session_id() == status.session_id() &&
           message.reference_epoch() == status.reference_epoch() &&
           message.motion_epoch() == authorized_motion_epoch;
  }

  /**
   * @brief 判断原始未授权Idle失联是否可仅重建只读基线。
   * 有过控制历史的连接不能借此省略Stop；允许时worker只invalidate后重观察，不恢复旧运动目标。
   * @return bool；从未授权、从未本地发命令、无active/in-flight/ACK/Stop且最后为raw Idle才允许。
   */
  bool recoverable_raw_idle_gap() const noexcept {
    return boot != 0U && status.boot_epoch() == boot && has_sequence &&
           last_state_boot == boot && last_observation_raw_idle &&
           (status.core_state() == 1U || status.core_state() == 2U) &&
           status.session_id() == 0U &&
           ((status.reference_epoch() == 0U && status.motion_epoch() == 0U &&
             authorized_motion_epoch == 0U && !ever_authorized_this_boot) ||
            (parameters.manual_reference_mode && historical_run_boot == boot &&
             historical_run_id != 0U && historical_reference_epoch != 0U &&
             status.reference_epoch() == historical_reference_epoch &&
             status.run_id() == historical_run_id &&
             status.release_committed() && !status.real_motion_permitted() &&
             status.permit_remaining_us() == 0U)) &&
           !command_published_this_boot && !system.commands_active() &&
           !command_stream.has_in_flight() && !group_sent_ack.pending() &&
           !stop_latched &&
           !stop_confirmation_pending.load(std::memory_order_acquire);
  }

  /**
   * @brief 隔离本次释放Stopping状态后早到的Idle State候选。
   * 候选本身不确认释放，保存后还要等COMMITTED及新鲜度重验证，不能因为看到core=Idle就越过事务。
   * @param message 同boot且session/reference归零的State。
   * @return bool；要求Status为同释放transaction/op/run、motion与management恰expected+1，无剩余许可。
   */
  bool released_reference_shape(std::uint16_t core,
                                std::uint32_t reference) const noexcept {
    return (core == 1U && reference == 0U) ||
           (parameters.manual_reference_mode &&
            gate_release.reference_epoch != 0U && core == 2U &&
            reference == gate_release.reference_epoch);
  }

  bool stopping_release_idle_candidate(const State &message) const noexcept {
    return gate_release.sent && gate_release.request_id != 0U &&
           gate_release.session_id != 0U &&
           gate_release.reference_epoch != 0U &&
           gate_release.expected_motion_epoch != UINT32_MAX &&
           gate_release.expected_management_epoch != UINT32_MAX &&
           boot == gate_release.boot_epoch && status.boot_epoch() == boot &&
           status.core_state() == kWireStopping &&
           status.transaction_phase() == 2U && status.operation_id() != 0U &&
           status.run_id() != 0U &&
           status.session_id() == gate_release.session_id &&
           status.reference_epoch() == gate_release.reference_epoch &&
           status.motion_epoch() == gate_release.expected_motion_epoch + 1U &&
           status.management_epoch() ==
               gate_release.expected_management_epoch + 1U &&
           !status.real_motion_permitted() &&
           status.permit_remaining_us() == 0U && message.boot_epoch() == boot &&
           released_reference_shape(message.core_state(),
                                    message.reference_epoch()) &&
           message.session_id() == 0U &&
           message.motion_epoch() == status.motion_epoch() &&
           message.management_epoch() == status.management_epoch() &&
           message.operation_id() == status.operation_id() &&
           message.transaction_phase() == 3U;
  }

  /**
   * @brief 允许处于释放过渡Status时观察同事务raw Idle。
   * 只解决Status/State到达乱序，不重新授予正常控制，后续仍走raw Idle有效位、源时序和释放再验证。
   * @param message 待判State。
   * @return bool；受Disabling或已COMMITTED的Stopping分支严格限定。
   */
  bool provisional_release_idle(const State &message) const noexcept {
    const bool disabling =
        gate_release.request_id != 0U && boot == gate_release.boot_epoch &&
        status.boot_epoch() == boot && status.core_state() == kWireDisabling &&
        status.session_id() == gate_release.session_id &&
        status.reference_epoch() == gate_release.reference_epoch &&
        sequence_at_or_after(status.motion_epoch(),
                             gate_release.expected_motion_epoch) &&
        sequence_at_or_after(status.management_epoch(),
                             gate_release.expected_management_epoch) &&
        (!gate_release.committed ||
         (status.motion_epoch() == gate_release.motion_epoch &&
          status.management_epoch() == gate_release.management_epoch)) &&
        !status.real_motion_permitted() && status.permit_remaining_us() == 0U &&
        message.boot_epoch() == boot &&
        released_reference_shape(message.core_state(),
                                 message.reference_epoch()) &&
        message.session_id() == 0U &&
        message.motion_epoch() == status.motion_epoch() &&
        message.management_epoch() == status.management_epoch();
    // 已匹配本次COMMITTED时，由同operation的新鲜State延续观察；旧过渡Status只作tuple证人。
    const bool stopping = gate_release.committed && stopping_release_idle_candidate(message) &&
        gate_release.operation_id==message.operation_id() && gate_release.run_id==status.run_id() &&
        gate_release.motion_epoch==message.motion_epoch() &&
        gate_release.management_epoch==message.management_epoch();
    return disabling || stopping;
  }

  /**
   * @brief 校验无参考坐标但有原始电机圈数的只读Idle状态。
   * 这些原始rev不被当作关节rad或运动起点；校验成功也会让硬件普通控制缓存失效。
   * @param message Idle/Readonly类State。
   * @param expected 已对齐源时钟的序号/年龄期望。
   * @param provisional_release 是否已满足本次释放过渡例外。
   * @return bool；位置有效掩码须0，raw掩码覆盖配置轴，原始圈数有限，序号/管理水位/年龄均满足。
   */
  bool raw_idle_observation_valid(const State &message,
                                  const StateExpectation &expected,
                                  bool provisional_release = false) const noexcept {
    if ((message.core_state() != 1U && message.core_state() != 2U) ||
        (!provisional_release && status.core_state() != message.core_state()) ||
        (message.position_valid_mask() != 0U &&
         !(parameters.manual_reference_mode && message.core_state() == 2U &&
           message.reference_epoch() == status.reference_epoch() &&
           !status.real_motion_permitted() &&
           status.permit_remaining_us() == 0U &&
           (message.position_valid_mask() & parameters.configured_mask) ==
               parameters.configured_mask &&
           std::all_of(std::begin(message.position_rad()),
                       std::end(message.position_rad()),
                       [](double q) { return std::isfinite(q); }))) ||
        (message.raw_position_valid_mask() & parameters.configured_mask) !=
            parameters.configured_mask ||
        !sequence_at_or_after(message.management_epoch(),
                              status.management_epoch()) ||
        (has_sequence && !sequence_newer(message.state_seq(), last_sequence)) ||
        message.produced_at_us() < expected.request_started_at_us ||
        message.produced_at_us() > expected.now_us ||
        expected.now_us - message.produced_at_us() > expected.max_age_us)
      return false;
    for (std::size_t axis = 0U; axis < parameters.position_min.size(); ++axis)
      if ((parameters.configured_mask & (1U << axis)) != 0U &&
          !std::isfinite(message.raw_motor_position_rev()[axis])) return false;
    return true;
  }

  /**
   * @brief 要求COMMITTED与双新鲜Status/State共同确认gate释放Idle。
   * 可建立idle_confirmed一次性锁存；检查的Status与State接收年龄均不能超max_state_age。
   * @param now 宿主steady当前µs。
   * @return bool；同boot及释放后motion/management、无session/reference/permit、release_committed，且无worker Stop未确认。
   */
  bool confirmed_gate_released_idle(std::uint64_t now) const noexcept {
    return gate_release.committed && boot == gate_release.boot_epoch &&
           bootstrap_ready && last_observation_raw_idle &&
           last_state_boot == boot && last_state_session == 0U &&
           last_state_reference == status.reference_epoch() &&
           last_state_motion == gate_release.motion_epoch &&
           last_state_management_epoch == gate_release.management_epoch &&
           status.boot_epoch() == boot &&
           released_reference_shape(status.core_state(),
                                    status.reference_epoch()) &&
           status.session_id() == 0U &&
           status.motion_epoch() == gate_release.motion_epoch &&
           status.management_epoch() == gate_release.management_epoch &&
           status.release_committed() && !status.real_motion_permitted() &&
           status.permit_remaining_us() == 0U && last_received_us != 0U &&
           now >= last_received_us &&
           now - last_received_us <= parameters.max_state_age_us &&
           last_status_received_us != 0U && now >= last_status_received_us &&
           now - last_status_received_us <= parameters.max_state_age_us &&
           !stop_confirmation_pending.load(std::memory_order_acquire);
  }

  /**
   * @brief 查询已证释放后仍保持同一Idle身份的锁存。
   * 仅避免已释放且无owner时再把停发心跳当运动失联Stop；它不提供新的管理准入或新鲜运动反馈。
   * @return bool；保留完整身份/许可/状态条件，但不重复年龄检查。
   */
  bool released_idle_latched() const noexcept {
    return gate_release.idle_confirmed && gate_release.committed &&
           status.reference_epoch() == gate_release.released_reference_epoch &&
           !stop_latched &&
           !stop_confirmation_pending.load(std::memory_order_acquire) &&
           boot == gate_release.boot_epoch && last_observation_raw_idle &&
           last_state_boot == boot && last_state_session == 0U &&
           last_state_reference == status.reference_epoch() &&
           last_state_motion == gate_release.motion_epoch &&
           last_state_management_epoch == gate_release.management_epoch &&
           status.boot_epoch() == boot &&
           released_reference_shape(status.core_state(),
                                    status.reference_epoch()) &&
           status.session_id() == 0U &&
           status.motion_epoch() == gate_release.motion_epoch &&
           status.management_epoch() == gate_release.management_epoch &&
           status.release_committed() && !status.real_motion_permitted() &&
           status.permit_remaining_us() == 0U;
  }

  /**
   * @brief 核对gate发送的D10/Query COMMITTED后绑定释放结果。
   * 校验ID/schema/设备客户端、phase3成功、operation非零、release_committed且非motion_complete；记录实际epoch/run。随后只重放同request、发送后到达且仍新鲜的早到Idle，保持原received_us。
   * @param pending 含原请求及已发布完整响应的事务。
   */
  void observe_gate_release_reply(const DeviceManagementRequest &pending) noexcept {
    if (gate_release.request_id == 0U || !gate_release.sent) return;
    Management::Request request;
    Management::Response response;
    if (wire::DummyManageV2RequestCodec::decode(pending.request.data(), pending.request.size(),
            &request) != wire::CodecResult::kOk ||
        wire::DummyManageV2ResponseCodec::decode(pending.response.data(), pending.response.size(),
            &response) != wire::CodecResult::kOk) return;
    if (!((request.operation() == kWireDisableAndRelease &&
           request.request_id() == gate_release.request_id) ||
          (request.operation() == kWireGetOperation &&
           request.queried_operation_id() == gate_release.request_id)) ||
        request.expected_boot_epoch() != gate_release.boot_epoch ||
        request.expected_session_id() != gate_release.session_id ||
        request.expected_reference_epoch() != gate_release.reference_epoch ||
        request.expected_motion_epoch() != gate_release.expected_motion_epoch ||
        request.expected_management_epoch() !=
            gate_release.expected_management_epoch ||
        response.request_id() != request.request_id() ||
        response.boot_epoch() != gate_release.boot_epoch ||
        !((response.session_id() == gate_release.session_id &&
           response.reference_epoch() == gate_release.reference_epoch) ||
          (response.session_id() == 0U &&
           (response.reference_epoch() == 0U ||
            (parameters.manual_reference_mode &&
             response.reference_epoch() == gate_release.reference_epoch)))) ||
        !sequence_at_or_after(response.motion_epoch(),
                              gate_release.expected_motion_epoch) ||
        !sequence_at_or_after(response.management_epoch(),
                              gate_release.expected_management_epoch) ||
        response.schema_major() != 2U ||
        response.schema_minor() != dummy_ref::protocol::kSchemaMinor ||
        response.profile_id() != parameters.profile_id ||
        response.receipt() != 1U || response.transaction_phase() != 3U ||
        response.result_code() != 0U || response.operation_id() == 0U ||
        !response.release_committed() || response.motion_complete() ||
        !std::equal(parameters.device_id.begin(), parameters.device_id.end(),
                    response.device_id()) ||
        !std::equal(parameters.client_instance_id.begin(),
                    parameters.client_instance_id.end(),
                    response.client_instance_id()))
      return;
    gate_release.motion_epoch = response.motion_epoch();
    gate_release.management_epoch = response.management_epoch();
    gate_release.operation_id=response.operation_id();
    gate_release.run_id=response.run_id();
    gate_release.committed = true;
    // 原Gate D10已严格COMMITTED时立即保存绑定；后续Safety抢占Query不抹去此事实。
    // 只供同连接代次、同根的fresh Fault子尝试退休，不更改Stop/原页审计结果。
    if(response.session_id()==0U && response.run_id()!=0U)
      management->publish_worker_release_commit({gate_release.request_id,response.operation_id(),
        response.run_id(),gate_release.boot_epoch,gate_release.session_id,0U,
        gate_release.reference_epoch,response.motion_epoch(),response.management_epoch()});
    const auto candidate=early_release_idle;
    early_release_idle={};early_worker_status={};
    const auto now=transport_steady_now_us();
    if(candidate.request_id==gate_release.request_id && candidate.received_us>=gate_release.sent_us &&
       now>=candidate.received_us && now-candidate.received_us<=parameters.max_state_age_us &&
       provisional_release_idle(candidate.state))
      process_decoded_state(candidate.state,candidate.received_us,true);
  }

  /**
   * @brief 从严格绑定的成功管理结果推进授权epoch水位。
   * Complete支持已发送且两秒内的Pending→Query后继；请求/响应均核boot/session/reference/profile/设备客户端，成功COMMITTED才按允许操作更新epoch。Complete被拒绝时退役旧状态；Pending只记录operation归属，不直接当完成授权。
   * @param pending 原请求和对应响应固定codec字节。
   */
  void accept_management_authority(const DeviceManagementRequest &pending) noexcept {
    Management::Request request;
    Management::Response response;
    if (wire::DummyManageV2RequestCodec::decode(pending.request.data(), pending.request.size(), &request) !=
            wire::CodecResult::kOk ||
        wire::DummyManageV2ResponseCodec::decode(pending.response.data(), pending.response.size(), &response) !=
            wire::CodecResult::kOk) return;
    /**
     * @brief Complete响应可能来自原operation8或查询其request_id的operation12；必须与保留的已dispatched摘要相符，不能让任意Query响应推进motion。
     */
    const bool complete_reply = pending_complete.request_id!=0U && pending_complete.dispatched &&
        transport_steady_now_us()-pending_complete.started_us<=2000000U &&
        (last_motion_operation_id==0U || response.operation_id()==last_motion_operation_id) &&
        (request.operation()==8U && request.request_id()==pending_complete.request_id ||
         request.operation()==kWireGetOperation &&
           request.queried_operation_id()==pending_complete.request_id) &&
        request.expected_boot_epoch()==pending_complete.boot_epoch &&
        request.expected_session_id()==pending_complete.session_id &&
        request.expected_reference_epoch()==pending_complete.reference_epoch &&
        request.expected_motion_epoch()==pending_complete.motion_epoch &&
        request.expected_management_epoch()==pending_complete.management_epoch;
    /**
     * @brief Status已先到后继Hold epoch时，接受匹配原Complete结果的受限例外，避免把本次合法后继误当旧请求越权。
     */
    const bool staged_complete = complete_reply &&
        status.boot_epoch()==pending_complete.boot_epoch &&
        status.session_id()==pending_complete.session_id &&
        status.reference_epoch()==pending_complete.reference_epoch &&
        authorized_motion_epoch==pending_complete.motion_epoch+1U &&
        status.motion_epoch()==authorized_motion_epoch &&
        response.motion_epoch()==authorized_motion_epoch &&
        sequence_at_or_after(response.management_epoch(),status.management_epoch());
    const bool invalid_request_binding =
        request.schema_major() != 2U || request.schema_minor() != dummy_ref::protocol::kSchemaMinor ||
        response.schema_major() != 2U || response.schema_minor() != dummy_ref::protocol::kSchemaMinor ||
        request.request_id() == 0U || response.request_id() != request.request_id() ||
        response.operation_id() == 0U || boot == 0U ||
        request.expected_boot_epoch() != boot || response.boot_epoch() != boot ||
        request.expected_session_id() != status.session_id() ||
        response.session_id() != status.session_id() ||
        request.expected_reference_epoch() != status.reference_epoch() ||
        response.reference_epoch() != status.reference_epoch() ||
        (!staged_complete && request.expected_motion_epoch() != authorized_motion_epoch) ||
        (!staged_complete && !sequence_at_or_after(request.expected_management_epoch(), status.management_epoch())) ||
        request.expected_profile_id() != parameters.profile_id ||
        response.profile_id() != parameters.profile_id ||
        request.configured_mask() != parameters.configured_mask ||
        !std::equal(parameters.device_id.begin(), parameters.device_id.end(), request.device_id()) ||
        !std::equal(parameters.device_id.begin(), parameters.device_id.end(), response.device_id()) ||
        !std::equal(parameters.client_instance_id.begin(), parameters.client_instance_id.end(),
                    request.client_instance_id()) ||
        !std::equal(parameters.client_instance_id.begin(), parameters.client_instance_id.end(),
                    response.client_instance_id());
    if (invalid_request_binding) return;
    if (complete_reply && response.transaction_phase()==4U) {
      invalidate(&status,true);
      return;
    }
    if (motion_authorizing_operation(request.operation()) &&
        request.request_id() == last_motion_request_id &&
        (response.transaction_phase() == 2U || response.transaction_phase() == 3U) &&
        (response.result_code() == 0U || response.result_code() == 23U))
      last_motion_operation_id = response.operation_id();
    if (response.transaction_phase() != 3U || response.result_code() != 0U ||
        !(motion_authorizing_operation(request.operation()) || complete_reply ||
          (request.operation() == 12U && request.queried_operation_id() == last_motion_request_id &&
           last_motion_operation_id != 0U && response.operation_id() == last_motion_operation_id)) ||
        authorized_motion_epoch == UINT32_MAX ||
        (!staged_complete && response.motion_epoch() != authorized_motion_epoch + 1U) ||
        response.management_epoch() == request.expected_management_epoch() ||
        !sequence_at_or_after(response.management_epoch(), request.expected_management_epoch())) return;
    authorized_motion_epoch = response.motion_epoch();
    authorized_management_epoch = response.management_epoch();
    if (complete_reply) {
      confirmed_complete={response.boot_epoch(),response.session_id(),
        response.reference_epoch(),response.motion_epoch(),response.committed_producer_seq()};
      pending_complete={};
    } else if (motion_authorizing_operation(request.operation())) confirmed_complete={};
    ever_authorized_this_boot = true;
    if (request.operation() != kWireDisableAndRelease) stop_latched = false;
  }

  /**
   * @brief 消费同身份固件commit序号并更新命令流/排空水位。
   * group_sent_ack与command_stream必须都认可commit才清command_sent；Hold时记录该宿主代真正ACK过，refresh记录其接收锚。can_sent_mask此辅助ACK当前不参与判定，完整SENT证明仍看原页/其他路径。
   * @tparam Message State或Status生成类型。
   * @param message 已通过外层身份过滤的State或Status。
   */
  template <typename Message>
  void acknowledge_command(const Message &message) noexcept {
    if (!group_sent_ack.pending()) return;
    const auto acknowledged=command_stream.in_flight_record();
    if (group_sent_ack.observe(message.boot_epoch(), message.session_id(),
          message.reference_epoch(), message.motion_epoch(),
          message.committed_producer_seq(), message.can_sent_mask(),
          transport_steady_now_us()) &&
        command_stream.acknowledge_committed(message.committed_producer_seq())) {
      last_ack_identity={message.boot_epoch(),message.session_id(),
        message.reference_epoch(),message.motion_epoch(),0U};
      last_ack_seq=message.committed_producer_seq();
      if(acknowledged && message.core_state()==kWireHold &&
         acknowledged->identity.boot==message.boot_epoch() &&
         acknowledged->identity.session==message.session_id() &&
         acknowledged->identity.reference_epoch==message.reference_epoch() &&
         acknowledged->identity.motion_epoch==message.motion_epoch()) {
        if(acknowledged->generation>last_processed_hold_generation) {
          last_processed_hold_identity=acknowledged->identity;
          last_processed_hold_generation=acknowledged->generation;
          last_processed_hold_disposition=2U;
          last_processed_hold_position=acknowledged->position;
        }
        hold_drain_rejected=false;
      }
      if(refresh_in_flight_seq==message.committed_producer_seq()) {
        last_refresh_acked_us=transport_steady_now_us();
        refresh_in_flight_seq=0U;
      }
      command_sent = false;
    } else {
      ++group_sent_ack_misses;
      diagnostics.reject(DiagnosticReason::kCommandRejected,
        {message.boot_epoch(),message.session_id(),message.reference_epoch(),message.motion_epoch(),message.committed_producer_seq(),message.management_epoch(),true},
        {boot,status.session_id(),status.reference_epoch(),authorized_motion_epoch,group_sent_ack.sequence(),status.management_epoch(),boot!=0U},
        transport_steady_now_us(),"ack_identity_or_sequence");
      if (ack_mismatch_diag_count++ < 10U)
        diagnostic_detail("COMMAND_ACK_DIAG expected_seq=%u observed_seq=%u status_seq=%u sent_mask=%u session=%llu/%llu motion=%u/%u\n",
                group_sent_ack.sequence(),message.committed_producer_seq(),
                status.committed_producer_seq(),message.can_sent_mask(),
                static_cast<unsigned long long>(message.session_id()),
                static_cast<unsigned long long>(status.session_id()),
                message.motion_epoch(),authorized_motion_epoch);
    }
  }

  /**
   * @brief Complete边界写入分类：Allowed可尝试送，LocalNoOp只本地同值吞并，Conflict请求Stop。
   */
  enum class PostCompleteWrite : std::uint8_t { kAllowed, kLocalNoOp, kConflict };
  /**
   * @brief 避免Complete后尚无新Begin的控制更新制造不同目标CAN组。
   * 校验首prime、已COMMITTED或新鲜Hold及逐轴容差；后继同值可LocalNoOp，但不冒称CAN SENT也不续设备Hold预算。每秒必要refresh仍要同值、正常票据与ACK；不一致目标为Conflict。
   * @param record 当前欲发送的rad目标及宿主代。
   * @param now 宿主steady当前µs。
   * @return 分类值；非该边界、未被判为terminal之后的序列或受限首prime/refresh可Allowed，其余按同值/冲突分流。
   */
  PostCompleteWrite post_complete_write(const CommandGenerationRecord &record,
                                        std::uint64_t now) const noexcept {
    const bool boundary=complete_boundary.boot_epoch==record.identity.boot &&
      complete_boundary.session_id==record.identity.session &&
      complete_boundary.reference_epoch==record.identity.reference_epoch &&
      complete_boundary.hold_epoch==record.identity.motion_epoch;
    if(!boundary) return PostCompleteWrite::kAllowed;
    const bool committed=confirmed_complete.boot_epoch==record.identity.boot &&
      confirmed_complete.session_id==record.identity.session &&
      confirmed_complete.reference_epoch==record.identity.reference_epoch &&
      confirmed_complete.motion_epoch==record.identity.motion_epoch;
    const bool fresh_hold=observed_hold.received_us!=0U && now>=observed_hold.received_us &&
      now-observed_hold.received_us<=parameters.max_state_age_us &&
      status.core_state()==kWireHold &&
      record.identity.boot==observed_hold.boot_epoch &&
      record.identity.session==observed_hold.session_id &&
      record.identity.reference_epoch==observed_hold.reference_epoch &&
      record.identity.motion_epoch==observed_hold.motion_epoch;
    const bool pending=pending_complete.request_id!=0U &&
      now>=pending_complete.started_us && now-pending_complete.started_us<=2000000U &&
      pending_complete.boot_epoch==record.identity.boot &&
      pending_complete.session_id==record.identity.session &&
      pending_complete.reference_epoch==record.identity.reference_epoch &&
      pending_complete.motion_epoch!=UINT32_MAX &&
      pending_complete.motion_epoch+1U==record.identity.motion_epoch;
    // HOLD Status可先于匹配State到达；只吞已证全轴SENT的首prime同值后继，不产生新CAN。
    if(!committed && !fresh_hold && status.core_state()==kWireHold && pending &&
       pending_complete.dispatched && complete_boundary.first_prime_seq!=0U &&
       status.boot_epoch()==record.identity.boot && status.session_id()==record.identity.session &&
       status.reference_epoch()==record.identity.reference_epoch &&
       status.motion_epoch()==record.identity.motion_epoch &&
       status.management_epoch()==authorized_management_epoch &&
       status.committed_producer_seq()==complete_boundary.first_prime_seq &&
       last_ack_seq==complete_boundary.first_prime_seq &&
       last_ack_identity.boot==record.identity.boot && last_ack_identity.session==record.identity.session &&
       last_ack_identity.reference_epoch==record.identity.reference_epoch &&
       last_ack_identity.motion_epoch==record.identity.motion_epoch &&
       status.can_submitted_mask()==parameters.configured_mask &&
       status.can_sent_mask()==parameters.configured_mask && status.can_failed_mask()==0U) {
      for(std::size_t axis=0U;axis<record.position.size();++axis) {
        if((parameters.configured_mask & (1U<<axis))==0U) continue;
        const auto tolerance=management->post_complete_hold_tolerance_rad[axis].load(std::memory_order_acquire);
        if(!std::isfinite(record.position[axis]) || !std::isfinite(tolerance) || tolerance<0.0 ||
           std::abs(record.position[axis]-complete_boundary.first_prime_position[axis])>tolerance)
          return PostCompleteWrite::kConflict;
      }
      return PostCompleteWrite::kLocalNoOp;
    }
    if(!committed && !fresh_hold && status.core_state()==kWireEnablingHold && pending) {
      const auto sequence=wire_producer_sequence(record.generation);
      if(complete_boundary.first_prime_seq==0U ||
         sequence==complete_boundary.first_prime_seq) return PostCompleteWrite::kAllowed;
      for(std::size_t axis=0U;axis<record.position.size();++axis) {
        if((parameters.configured_mask & (1U<<axis))==0U) continue;
        const auto tolerance=management->post_complete_hold_tolerance_rad[axis].load(
            std::memory_order_acquire);
        if(!std::isfinite(record.position[axis]) || !std::isfinite(tolerance) ||
           tolerance<0.0 ||
           std::abs(record.position[axis]-complete_boundary.first_prime_position[axis])>tolerance)
          return PostCompleteWrite::kConflict;
      }
      return PostCompleteWrite::kLocalNoOp;
    }
    /**
     * @brief 已导出旧motion完成证明后允许首个或已ACK一秒后的必要Hold refresh；重试只能保留同线序号，新目标仍须另走Begin。
     */
    if(exported_motion_ready && fresh_hold) {
      const auto seq=wire_producer_sequence(record.generation);
      const bool retry=refresh_in_flight_seq==seq;
      const bool first=last_refresh_acked_us==0U && refresh_in_flight_seq==0U;
      const bool repeat=last_refresh_acked_us!=0U && refresh_in_flight_seq==0U &&
        refresh_active_run_id>complete_boundary.prior_run_id &&
        now>=last_refresh_acked_us && now-last_refresh_acked_us>=1000000U;
      if(retry || first || repeat) {
        for(std::size_t axis=0U;axis<record.position.size();++axis) {
          if((parameters.configured_mask & (1U<<axis))==0U) continue;
          const auto tolerance=management->post_complete_hold_tolerance_rad[axis].load(
              std::memory_order_acquire);
          if(!std::isfinite(record.position[axis]) || !std::isfinite(tolerance) ||
             tolerance<=0.0 ||
             std::abs(record.position[axis]-observed_hold.position[axis])>tolerance)
            return PostCompleteWrite::kConflict;
        }
        return PostCompleteWrite::kAllowed;
      }
    }
    const auto terminal_seq=committed ? confirmed_complete.terminal_seq :
      fresh_hold ? observed_hold.committed_seq : complete_boundary.terminal_seq;
    if(!sequence_newer(wire_producer_sequence(record.generation),terminal_seq))
      return PostCompleteWrite::kAllowed;
    PostCompleteWrite result=PostCompleteWrite::kLocalNoOp;
    double max_delta=0.0, max_tolerance=0.0;
    if(!committed && !fresh_hold) result=PostCompleteWrite::kConflict;
    if(fresh_hold) for(std::size_t axis=0U;axis<record.position.size();++axis) {
      if((parameters.configured_mask & (1U<<axis))==0U) continue;
      const auto tolerance=management->post_complete_hold_tolerance_rad[axis].load(
          std::memory_order_acquire);
      const auto delta=std::abs(record.position[axis]-observed_hold.position[axis]);
      max_delta=std::max(max_delta,delta);
      max_tolerance=std::max(max_tolerance,tolerance);
      if(!std::isfinite(record.position[axis]) || !std::isfinite(tolerance) ||
         tolerance<0.0 || delta>tolerance)
        result=PostCompleteWrite::kConflict;
    }
    if(!complete_boundary_hold_diag_logged && status.core_state()==kWireHold) {
      complete_boundary_hold_diag_logged=true;
      diagnostic_detail(
          "POST_COMPLETE_WRITE seq=%u action=%u pending_age_us=%llu confirmed=%d hold_age_us=%llu delta_rad=%.9f tolerance_rad=%.9f\n",
          wire_producer_sequence(record.generation),static_cast<unsigned>(result),
          static_cast<unsigned long long>(pending_complete.request_id!=0U &&
              now>=pending_complete.started_us ? now-pending_complete.started_us : 0U),
          committed,static_cast<unsigned long long>(fresh_hold ? now-observed_hold.received_us : 0U),
          max_delta,max_tolerance);
    }
    return result;
  }

  /**
   * @brief 拒绝已冻结终序号之后的旧motion更新。
   * 防止控制器在Complete等待期间又产生采样，把终组SENT后的新点写入旧motion证据账本。
   * @param record 待送控制记录。
   * @return bool；同Complete根身份且线序号严格较terminal_seq新即被取代。
   */
  bool superseded_by_pending_complete(const CommandGenerationRecord &record) const noexcept {
    return pending_complete.request_id!=0U && pending_complete.terminal_seq!=0U &&
      record.identity.boot==pending_complete.boot_epoch &&
      record.identity.session==pending_complete.session_id &&
      record.identity.reference_epoch==pending_complete.reference_epoch &&
      record.identity.motion_epoch==pending_complete.motion_epoch &&
      sequence_newer(wire_producer_sequence(record.generation),pending_complete.terminal_seq);
  }

  /**
   * @brief 首次本地提交尝试时锁定新Hold首prime的序号和目标。
   * 只写空first_prime槽，不覆盖已有值；调用处包括本地Ok/Busy/QueueFull，保留同一候选便于安全重试，不能由此声称已发送。
   * @param record 对应complete_boundary且core为EnablingHold的记录。
   */
  void reserve_complete_prime(const CommandGenerationRecord &record) noexcept {
    if(complete_boundary.first_prime_seq!=0U ||
       complete_boundary.boot_epoch!=record.identity.boot ||
       complete_boundary.session_id!=record.identity.session ||
       complete_boundary.reference_epoch!=record.identity.reference_epoch ||
       complete_boundary.hold_epoch!=record.identity.motion_epoch ||
       status.core_state()!=kWireEnablingHold) return;
    complete_boundary.first_prime_seq=wire_producer_sequence(record.generation);
    complete_boundary.first_prime_position=record.position;
  }

  /**
   * @brief 按具体原因保存拒绝身份并以1秒频率存详情。
   * 固定journal首错不受日志节流影响；details区分callback到达与已接受水位，便于识别解码/身份/时钟/年龄/锁争用。
   * @param reason 非空分支原因文本。
   * @param value 被拒State，源时间仍保留。
   * @param received_us 该消息最初宿主接收µs。
   */
  void log_state_rejection(const char *reason, const State &value,
                           std::uint64_t received_us) noexcept {
    const DiagnosticIdentity observed{value.boot_epoch(),value.session_id(),value.reference_epoch(),
      value.motion_epoch(),value.state_seq(),value.management_epoch(),true};
    const DiagnosticIdentity expected{boot,status.session_id(),status.reference_epoch(),
      authorized_motion_epoch,last_sequence,status.management_epoch(),boot!=0U};
    const auto kind=std::strcmp(reason,"identity_or_bootstrap")==0 ? DiagnosticReason::kStateIdentity :
      std::strcmp(reason,"authority")==0 ? DiagnosticReason::kStateAuthority :
      std::strcmp(reason,"sequence_or_timestamp")==0 ? DiagnosticReason::kStateSequence :
      (std::strcmp(reason,"clock_skew")==0 || std::strcmp(reason,"timestamp_overflow")==0) ? DiagnosticReason::kStateClock :
      std::strcmp(reason,"codec")==0 ? DiagnosticReason::kStateCodec :
      std::strcmp(reason,"stop_latched")==0 ? DiagnosticReason::kStateStopLatched :
      std::strcmp(reason,"ingest")==0 ? DiagnosticReason::kStateIngest : DiagnosticReason::kStateAge;
    diagnostics.reject(kind,observed,expected,received_us,reason);
    if (received_us - last_rejection_log_us < 1000000U) return;
    last_rejection_log_us = received_us;
    diagnostic_detail( "device transport state rejected: reason=%s seq=%u produced_us=%llu received_us=%llu boot=%llu session=%llu ref=%u motion=%u expected_session=%llu expected_ref=%u expected_motion=%u status_motion=%u state_management=%u status_management=%u\n",
            reason, value.state_seq(), static_cast<unsigned long long>(value.produced_at_us()),
            static_cast<unsigned long long>(received_us),
            static_cast<unsigned long long>(value.boot_epoch()),
            static_cast<unsigned long long>(value.session_id()),
            value.reference_epoch(), value.motion_epoch(),
            static_cast<unsigned long long>(status.session_id()), status.reference_epoch(),
            authorized_motion_epoch, status.motion_epoch(), value.management_epoch(),
            status.management_epoch());
  }

  /**
   * @brief 在worker自有Stop中隔离COMMITTED前先到的严格Idle Status。
   * 只存早到单槽，不修改权威Status/boot/授权；等QueryResult证实相同operation/run后再重放。
   * @param v 候选Status。
   * @return bool；要求当前D10根的boot/run/expected+1 epoch、根尝试已释放终态、五hash和非零链一致。
   */
  bool worker_idle_status_candidate(const Status& v) const noexcept {
    return stop_confirmation_pending.load() && gate_release.sent &&
           !gate_release.committed && gate_release.request_id &&
           gate_release.run_id && matches_status(v) &&
           v.boot_epoch() == gate_release.boot_epoch &&
           released_reference_shape(v.core_state(), v.reference_epoch()) &&
           v.session_id() == 0U &&
           gate_release.expected_motion_epoch != UINT32_MAX &&
           gate_release.expected_management_epoch != UINT32_MAX &&
           gate_release.expected_motion_epoch != UINT32_MAX &&
           gate_release.expected_management_epoch != UINT32_MAX &&
           v.motion_epoch() == gate_release.expected_motion_epoch + 1U &&
           v.management_epoch() ==
               gate_release.expected_management_epoch + 1U &&
           v.operation_id() != 0U && v.transaction_phase() == 3U &&
           v.run_id() == gate_release.run_id && v.attempt_id() == v.run_id() &&
           v.attempt_kind() == 1U && v.parent_run_id() == 0U &&
           v.final_state() == 2U && v.release_committed() &&
           !v.evidence_lost() && !v.motion_complete() &&
           !v.real_motion_permitted() && v.permit_remaining_us() == 0U &&
           v.first_failure_reason() == 0U && v.failure_source() == 0U &&
           v.failure_axis_mask() == 0U &&
           std::equal(parameters.contract_hash.begin(),
                      parameters.contract_hash.end(), v.contract_sha256()) &&
           std::equal(parameters.model_hash.begin(),
                      parameters.model_hash.end(), v.model_sha256()) &&
           std::equal(parameters.profile_hash.begin(),
                      parameters.profile_hash.end(), v.profile_sha256()) &&
           std::equal(parameters.firmware_hash.begin(),
                      parameters.firmware_hash.end(), v.firmware_sha256()) &&
           std::equal(parameters.policy_hash.begin(),
                      parameters.policy_hash.end(),
                      v.compiled_policy_sha256()) &&
           std::any_of(std::begin(v.evidence_chain_sha256()),
                       std::end(v.evidence_chain_sha256()),
                       [](auto c) { return c != 0U; });
  }
  /**
   * @brief 识别本次worker Stop的早到原始Idle State候选。
   * 不能仅凭session=0放行；它依赖已发本次Stop的固定根身份及后续完整COMMITTED屏障。
   * @param v State候选。
   * @return bool；需新源时间/序号、expected+1 epoch、phase3和完整有限raw rev，position_valid=0。
   */
  bool worker_idle_state_candidate(const State& v) const noexcept {
    return stop_confirmation_pending.load() && gate_release.sent &&
           !gate_release.committed && gate_release.request_id &&
           matches_state(v) && v.boot_epoch() == gate_release.boot_epoch &&
           released_reference_shape(v.core_state(), v.reference_epoch()) &&
           v.session_id() == 0U &&
           gate_release.expected_motion_epoch != UINT32_MAX &&
           gate_release.expected_management_epoch != UINT32_MAX &&
           v.motion_epoch() == gate_release.expected_motion_epoch + 1U &&
           v.management_epoch() ==
               gate_release.expected_management_epoch + 1U &&
           v.operation_id() != 0U && v.transaction_phase() == 3U &&
           v.first_failure_reason() == 0U && has_sequence &&
           v.produced_at_us() > last_produced_us &&
           sequence_newer(v.state_seq(), last_sequence) &&
           (v.position_valid_mask() == 0U ||
            (parameters.manual_reference_mode &&
             v.reference_epoch() == gate_release.reference_epoch &&
             (v.position_valid_mask() & parameters.configured_mask) ==
                 parameters.configured_mask)) &&
           (v.raw_position_valid_mask() & parameters.configured_mask) ==
               parameters.configured_mask &&
           std::all_of(std::begin(v.raw_motor_position_rev()),
                       std::end(v.raw_motor_position_rev()),
                       [](double q) { return std::isfinite(q); });
  }
  /**
   * @brief 从dp_comm借用Status复制值并按需把原字节入collector。
   * 接收锚在复制前取宿主steady时间；拷贝失败直接返回，原始归档在正常授权过滤之前保留观测，随后process_decoded_status才决定权威状态。
   * @param borrowed 只在回调期有效的Status借用。
   * @param context 非空Impl指针，由同worker订阅绑定。
   */
  static void receive_status(dp::comm::BorrowedMessage<Status> borrowed, void *context) noexcept {
    auto &self = *static_cast<Impl *>(context);
    const auto received_anchor_us=transport_steady_now_us();
    Status value;
    if (borrowed.copy_to(&value) != dp::comm::Result::kOk) return;
    (void)self.management->publish_diagnostic_status(value, received_anchor_us);
    if (!self.parameters.raw_evidence_path.empty()) {
      std::array<std::uint8_t, wire::DummyDeviceStatusV2Codec::kEncodedSize> raw{};
      if (wire::DummyDeviceStatusV2Codec::encode(value, raw.data(), raw.size()) != wire::CodecResult::kOk ||
          !self.raw_evidence.offer_observation(RawEvidenceCollector::Kind::kStatus,
              raw.data(), raw.size(), received_anchor_us, value.boot_epoch(), value.session_id()))
        self.failed = true;
    }
    self.process_decoded_status(value,received_anchor_us);
  }
  /**
   * @brief 处理能力bootstrap、run/epoch水位、证据绑定与命令ACK。
   * 先拒绝错schema/设备、退役boot和旧run；早Idle隔离，不提前授权。核完整hash/limits/timebase能力，协调Complete合法后继及临时证据run，对已接受完成链的矛盾锁存失败。能力不匹配会撤销硬件控制资格，但仍可发布Status只读观测并继续检查拒绝计数/commit；不能把收到观测等同bootstrap就绪。
   * @param value 已复制Status。
   * @param received_us 原始宿主接收µs。
   * @param replay True表示重放早到单槽，仍用原接收锚并重新检查年龄。
   */
  void process_decoded_status(const Status& value,std::uint64_t received_us,bool replay=false) {
    auto& self=*this;
    if(replay && (transport_steady_now_us()<received_us || transport_steady_now_us()-received_us>parameters.max_state_age_us))return;
    if(!replay)++self.statuses;
    if (!self.matches_status(value)) return;
    if (std::find(self.retired_boots.begin(), self.retired_boots.end(), value.boot_epoch()) !=
        self.retired_boots.end()) return;
    /**
     * @brief 有限完成链证人约束同run之后的Status：同一尝试不能在已经验过终链后改kind/parent、丢证据或换摘要，否则collector失效。
     */
    if(value.final_state()==6U && value.run_id()!=0U) {
      for(const auto &terminal:self.completed_terminal_chains) {
        if(terminal.run_id!=value.run_id() ||
           terminal.boot_epoch!=value.boot_epoch() ||
           terminal.session_id!=value.session_id()) continue;
        if(value.attempt_kind()!=1U || value.parent_run_id()!=0U ||
           !value.motion_complete() || value.evidence_lost() ||
           !std::equal(terminal.chain.begin(),terminal.chain.end(),
                       value.evidence_chain_sha256())) {
          self.raw_evidence.reject_invalid();
          self.failed=true;
          return;
        }
      }
    }
    if (value.boot_epoch() == self.last_status_run_boot &&
        value.session_id() == self.last_status_run_session && value.run_id() != 0U &&
        value.run_id() < self.last_status_run_id) {
      ++self.stale_run_status_rejections;
      return;
    }
    /**
     * @brief 早Idle Status仅隔离，不覆盖当前Stopping身份；保留原received_us以免QueryResult到达后人为续新鲜度。
     */
    if (!replay && self.worker_idle_status_candidate(value) && received_us>=self.gate_release.sent_us) {
      if(wire::DummyDeviceStatusV2Codec::encode(value,self.early_worker_status.raw.data(),self.early_worker_status.raw.size())==wire::CodecResult::kOk)
        self.early_worker_status={self.early_worker_status.raw,self.gate_release.request_id,received_us};
      return; // 候选仅隔离存储；不修改authoritative status/boot/observations或授权。
    }
    if (self.stop_latched && self.status.boot_epoch() == value.boot_epoch() &&
        (value.motion_epoch() != self.authorized_motion_epoch ||
         !sequence_at_or_after(value.management_epoch(), self.authorized_management_epoch))) return;
    if (self.status.boot_epoch() == value.boot_epoch()) {
      if (!sequence_at_or_after(value.management_epoch(), self.status.management_epoch())) return;
      if (value.management_epoch() == self.status.management_epoch()) {
        const bool same_owner_reference = value.session_id() == self.status.session_id() &&
            value.reference_epoch() == self.status.reference_epoch();
        const bool released_idle =
            value.session_id() == 0U &&
            self.released_reference_shape(value.core_state(),
                                          value.reference_epoch()) &&
            value.release_committed() && !value.real_motion_permitted() &&
            value.permit_remaining_us() == 0U;
        if ((!same_owner_reference && !released_idle) ||
            !sequence_at_or_after(value.motion_epoch(), self.status.motion_epoch())) return;
      }
    }
    if (self.boot != 0U && self.boot != value.boot_epoch()) {
      if (self.retired_count >= self.retired_boots.size()) {
        self.invalidate();
        self.failed = true;
        self.running = false;
        return;
      }
      self.retired_boots[self.retired_count++] = self.boot;
      self.invalidate(nullptr, true);
    }
    if (self.historical_run_boot != 0U &&
        self.historical_run_boot != value.boot_epoch()) {
      self.historical_run_boot = 0U;
      self.historical_run_id = 0U;
      self.historical_chain.fill(0U);
      self.historical_reference_epoch = 0U;
    }
    /**
     * @brief 只有已发Complete两秒窗口内、同session/reference且motion恰+1的EnablingHold/Hold能解释预期后继；其他epoch变化退役旧控制/时钟水位。
     */
    const bool expected_complete_advance=self.pending_complete.request_id!=0U &&
        self.pending_complete.dispatched &&
        (value.core_state()==kWireEnablingHold || value.core_state()==kWireHold) &&
        value.boot_epoch()==self.pending_complete.boot_epoch &&
        value.session_id()==self.pending_complete.session_id &&
        value.reference_epoch()==self.pending_complete.reference_epoch &&
        self.authorized_motion_epoch==self.pending_complete.motion_epoch &&
        self.pending_complete.motion_epoch!=UINT32_MAX &&
        value.motion_epoch()==self.pending_complete.motion_epoch+1U &&
        sequence_at_or_after(value.management_epoch(),self.pending_complete.management_epoch) &&
        transport_steady_now_us()-self.pending_complete.started_us<=2000000U;
    if (self.boot != 0U && !expected_complete_advance &&
        (value.session_id() != self.status.session_id() ||
         value.reference_epoch() != self.status.reference_epoch() ||
         (value.motion_epoch() != self.status.motion_epoch() &&
          value.motion_epoch() != self.authorized_motion_epoch))) self.invalidate(&self.status, true);
    if(self.raw_fault_trusted_boot!=0U && self.raw_fault_trusted_boot!=value.boot_epoch()) {
      self.raw_fault_trusted_boot=0U;self.raw_fault_clock={};
      self.management->reset_raw_cleanup_lifetime();
    }
    if (self.status.boot_epoch() != value.boot_epoch()) {
      self.ever_authorized_this_boot = false;
      self.command_published_this_boot = false;
      self.last_observation_raw_idle = false;
    }
    /**
     * @brief 能力证据完全取设备Status的五hash、六轴设备限制和控制/发布周期；expected另取冻结部署值，逐项比对，不以expected回填缺失观测。
     */
    PositionOnlyCapability observed{};
    observed.configured_mask = value.configured_mask();
    observed.capabilities = value.capabilities();
    std::copy(std::begin(value.contract_sha256()), std::end(value.contract_sha256()), observed.contract_hash.begin());
    std::copy(std::begin(value.model_sha256()), std::end(value.model_sha256()), observed.model_hash.begin());
    std::copy(std::begin(value.profile_sha256()), std::end(value.profile_sha256()), observed.profile_hash.begin());
    std::copy(std::begin(value.firmware_sha256()), std::end(value.firmware_sha256()), observed.firmware_hash.begin());
    std::copy(std::begin(value.compiled_policy_sha256()), std::end(value.compiled_policy_sha256()),
              observed.policy_hash.begin());
    for (std::size_t axis = 0U; axis < observed.limits.size(); ++axis) {
      observed.limits[axis] = {value.position_min_rad()[axis], value.position_max_rad()[axis]};
      observed.velocity_max[axis] = value.velocity_max_rad_s()[axis];
      observed.acceleration_max[axis] = value.acceleration_max_rad_s()[axis];
    }
    observed.control_timebase_us = value.control_period_us();
    observed.state_publish_period_us = value.state_publish_period_us();
    PositionOnlyCapability expected{};
    expected.configured_mask = self.parameters.configured_mask;
    expected.capabilities = kPositionOnlyCapability;
    expected.contract_hash = self.parameters.contract_hash;
    expected.model_hash = self.parameters.model_hash;
    expected.profile_hash = self.parameters.profile_hash;
    expected.firmware_hash = self.parameters.firmware_hash;
    expected.policy_hash = self.parameters.policy_hash;
    expected.control_timebase_us = self.parameters.control_period_us;
    expected.state_publish_period_us = self.parameters.state_publish_period_us;
    for (std::size_t axis = 0U; axis < expected.limits.size(); ++axis) {
      expected.limits[axis] = {self.parameters.position_min[axis], self.parameters.position_max[axis]};
      expected.velocity_max[axis] = self.parameters.velocity_max[axis];
      expected.acceleration_max[axis] = self.parameters.acceleration_max[axis];
    }
    /**
     * @brief bootstrap_ready只说明所需位置能力匹配，不能替代准备Hold、动作许可/资格及当前State新鲜度。
     */
    self.bootstrap_ready = position_only_capability_matches(expected, observed);
    if (self.provisional_evidence_binding && self.provisional_release_chain &&
        self.last_terminal_chain_run == self.provisional_evidence_binding->run_id &&
        self.raw_evidence.terminal_written_run_id() == self.provisional_evidence_binding->run_id &&
        self.last_terminal_chain == *self.provisional_release_chain) {
      self.provisional_evidence_binding.reset();
      self.provisional_release_chain.reset();
    }
    /**
     * @brief 原页先到推断的run必须由后续Status对账；session归零仅在明确已释放Idle例外允许，链一旦出现须与后来的原始终链一致。
     */
    if (self.provisional_evidence_binding && self.bootstrap_ready) {
      const auto &pending = *self.provisional_evidence_binding;
      if (value.run_id() == pending.run_id) {
        if (value.boot_epoch() != pending.boot_epoch ||
            value.attempt_kind() != pending.attempt_kind ||
            value.parent_run_id() != pending.parent_run_id || value.evidence_lost()) {
          self.raw_evidence.reject_invalid();
          self.failed = true;
          return;
        }
        if (value.session_id() != pending.session_id) {
          const bool released_idle =
              value.session_id() == 0U &&
              self.released_reference_shape(value.core_state(),
                                            value.reference_epoch()) &&
              value.final_state() == 2U && value.release_committed() &&
              !value.real_motion_permitted() &&
              value.permit_remaining_us() == 0U;
          if (!released_idle) {
            self.raw_evidence.reject_invalid();
            self.failed = true;
            return;
          }
        }
        const bool has_chain = std::any_of(std::begin(value.evidence_chain_sha256()),
            std::end(value.evidence_chain_sha256()),
            [](auto byte) { return byte != 0U; });
        if (has_chain) {
          std::array<std::uint8_t, 32U> chain{};
          std::copy(std::begin(value.evidence_chain_sha256()),
              std::end(value.evidence_chain_sha256()), chain.begin());
          if ((self.provisional_release_chain && chain != *self.provisional_release_chain) ||
              (self.last_terminal_chain_run == pending.run_id &&
               chain != self.last_terminal_chain)) {
            self.raw_evidence.reject_invalid();
            self.failed = true;
            return;
          }
          self.provisional_release_chain = chain;
          if (self.last_terminal_chain_run == pending.run_id &&
              self.raw_evidence.terminal_written_run_id() == pending.run_id) {
            self.provisional_evidence_binding.reset();
            self.provisional_release_chain.reset();
          }
        }
      } else if (value.boot_epoch() != pending.boot_epoch ||
                 value.run_id() > pending.run_id ||
                 (value.session_id() != pending.session_id && value.session_id() != 0U)) {
        self.raw_evidence.reject_invalid();
        self.failed = true;
        return;
      }
    }
    if (!self.parameters.raw_evidence_path.empty() && self.bootstrap_ready &&
        !self.bind_evidence_status(value)) {
      diagnostic_detail(
          "RAW_BIND_DIAG run=%llu boot=%llu session=%llu kind=%u parent=%llu core=%u final=%u lost=%u terminal_written_run=%llu last_status_run=%llu failure=%u\n",
          static_cast<unsigned long long>(value.run_id()),
          static_cast<unsigned long long>(value.boot_epoch()),
          static_cast<unsigned long long>(value.session_id()), value.attempt_kind(),
          static_cast<unsigned long long>(value.parent_run_id()), value.core_state(),
          value.final_state(), value.evidence_lost() ? 1U : 0U,
          static_cast<unsigned long long>(self.raw_evidence.terminal_written_run_id()),
          static_cast<unsigned long long>(self.last_status_run_id),
          static_cast<unsigned>(self.raw_evidence.failure()));
      self.diagnostics.reject_from_context(DiagnosticReason::kEvidenceFailure,received_us,"raw_status_binding");
      self.raw_evidence.reject_invalid();
      self.failed = true;
      return;
    }
    self.boot = value.boot_epoch();
    if (self.authorized_motion_epoch == 0U ||
        sequence_at_or_after(value.motion_epoch(), self.authorized_motion_epoch))
      self.authorized_motion_epoch = value.motion_epoch();
    if (self.authorized_management_epoch == 0U ||
        sequence_at_or_after(value.management_epoch(), self.authorized_management_epoch))
      self.authorized_management_epoch = value.management_epoch();
    self.status = value;
    self.last_status_received_us = received_us;
    if(self.complete_boundary.boot_epoch==0U && !self.stop_latched)
      self.management->post_complete_begin_ready.store(true,std::memory_order_release);
    /**
     * @brief Complete后下一Begin屏障依赖旧run的完成Status和终页链，启用捕获时还需writer终页水位；已导出后再出现矛盾则封锁并Stop。
     */
    if(value.boot_epoch()==self.complete_boundary.boot_epoch &&
       value.session_id()==self.complete_boundary.session_id &&
       value.reference_epoch()==self.complete_boundary.reference_epoch &&
       value.motion_epoch()==self.complete_boundary.hold_epoch) {
      const bool chain_nonzero=std::any_of(std::begin(value.evidence_chain_sha256()),
          std::end(value.evidence_chain_sha256()),[](auto byte){return byte!=0U;});
      const bool prior_run=value.run_id()==self.complete_boundary.prior_run_id;
      const bool contradictory=value.evidence_lost() ||
        (self.exported_motion_ready && prior_run &&
         (value.core_state()!=kWireHold || value.attempt_kind()!=1U ||
          value.parent_run_id()!=0U || value.final_state()!=6U ||
          !value.motion_complete() || !chain_nonzero ||
          !std::equal(self.exported_motion_chain.begin(),self.exported_motion_chain.end(),
                      value.evidence_chain_sha256())));
      if(contradictory) {
        self.exported_motion_ready=false;
        self.management->post_complete_begin_ready.store(false,std::memory_order_release);
        self.system.request_transport_stop();
      } else if(value.core_state()==kWireHold && prior_run && value.run_id()!=0U &&
         value.attempt_kind()==1U && value.final_state()==6U && value.motion_complete() &&
         chain_nonzero && (!self.parameters.raw_evidence_path.empty() ?
           self.raw_evidence.terminal_written_run_id()==value.run_id() &&
             self.last_terminal_chain_run==value.run_id() &&
             std::equal(self.last_terminal_chain.begin(),self.last_terminal_chain.end(),
                        value.evidence_chain_sha256()) : true))
      {
        self.exported_motion_ready=true;
        std::copy(std::begin(value.evidence_chain_sha256()),
                  std::end(value.evidence_chain_sha256()),self.exported_motion_chain.begin());
      }
      if(self.exported_motion_ready)
        self.management->post_complete_begin_ready.store(true,std::memory_order_release);
      if(self.exported_motion_ready && value.run_id()>self.complete_boundary.prior_run_id &&
         value.attempt_kind()==1U && value.final_state()==0U)
        self.refresh_active_run_id=value.run_id();
    }
    if (value.boot_epoch() != self.last_status_run_boot ||
        value.session_id() != self.last_status_run_session) {
      self.last_status_run_boot = value.boot_epoch();
      self.last_status_run_session = value.session_id();
      self.last_status_run_id = value.run_id();
    } else if (value.run_id() > self.last_status_run_id) self.last_status_run_id = value.run_id();
    if (value.session_id() != 0U || value.reference_epoch() != 0U ||
        value.motion_epoch() != 0U) self.ever_authorized_this_boot = true;
    if (!self.bootstrap_ready) {
      ++self.bootstrap_rejections;
      self.system.invalidate_transport_state();
    }
    /**
     * @brief 同身份拒绝数增加且commit停旧水位，明确记rejected_ack给gate，清in-flight并Stop；不能把这种设备拒绝伪装成普通无更新。
     */
    if (!self.stop_latched && self.group_sent_ack.command_rejected(
            value.boot_epoch(), value.session_id(), value.reference_epoch(),
            value.motion_epoch(), value.committed_producer_seq(),
            value.command_reject_count(), transport_steady_now_us())) {
      const auto pending=self.command_stream.in_flight_record();
      if (pending) {
        std::lock_guard<std::mutex> lock(self.management->mutex);
        auto &observation=self.management->observation;
        observation.rejected_ack_boot_epoch=pending->identity.boot;
        observation.rejected_ack_session_id=pending->identity.session;
        observation.rejected_ack_reference_epoch=pending->identity.reference_epoch;
        observation.rejected_ack_motion_epoch=pending->identity.motion_epoch;
        observation.rejected_ack_sequence=self.group_sent_ack.sequence();
        observation.rejected_ack_received_us=transport_steady_now_us();
      }
      self.diagnostics.reject(DiagnosticReason::kCommandRejected,
        {value.boot_epoch(),value.session_id(),value.reference_epoch(),value.motion_epoch(),value.committed_producer_seq(),value.management_epoch(),true},
        {self.boot,self.status.session_id(),self.status.reference_epoch(),self.authorized_motion_epoch,self.group_sent_ack.sequence(),self.status.management_epoch(),self.boot!=0U},
        received_us,"device_rejected_during_ack");
      diagnostic_detail("DEVICE_COMMAND_REJECTED_DURING_ACK seq=%u rejected=%u committed=%u\n",
              self.group_sent_ack.sequence(),value.command_reject_count(),
              value.committed_producer_seq());
      self.group_sent_ack.reset();
      self.command_stream.discard_in_flight();
      self.command_sent=false;
      self.hold_drain_rejected=true;
      self.stop_latched=true;
      self.system.request_transport_stop();
    } else self.acknowledge_command(value);
    {
      std::lock_guard<std::mutex> lock(self.management->mutex);
      auto &observation=self.management->observation;
      if(wire::DummyDeviceStatusV2Codec::encode(value,observation.status.data(),observation.status.size())==wire::CodecResult::kOk)
        observation.status_received_us=received_us;
    }
  }

  /**
   * @brief 复制借用State、保留原始观测，再交源时序/授权校验。
   * callback计数包括后续拒绝的帧；collector故障清理producer校验使用预存正常流锚，不授予正常State资格。copy失败记录固定拒绝，不把未解码字节当State。
   * @param borrowed 回调期借用State消息。
   * @param context 非空Impl指针，由worker订阅持有关系保证。
   */
  static void receive_state(dp::comm::BorrowedMessage<State> borrowed, void *context) noexcept {
    auto &self = *static_cast<Impl *>(context);
    ++self.state_callbacks;
    const auto now = transport_steady_now_us();
    State value;
    if (borrowed.copy_to(&value) != dp::comm::Result::kOk) {
      ++self.state_decode_rejections;
      ++self.rejected_states;
      const auto identities=self.diagnostics.context();
      self.diagnostics.reject(DiagnosticReason::kStateRejected,{},identities.second,now,"decode");
      return;
    }
    (void)self.management->publish_diagnostic_state(value, now);
    if (!self.parameters.raw_evidence_path.empty()) {
      std::array<std::uint8_t, wire::DummyJointStateV2Codec::kEncodedSize> raw{};
      if (wire::DummyJointStateV2Codec::encode(value, raw.data(), raw.size()) != wire::CodecResult::kOk ||
          !self.raw_evidence.offer_observation(RawEvidenceCollector::Kind::kState,
              raw.data(), raw.size(), now, value.boot_epoch(), value.session_id(),
              value.schema_major()==2U && value.schema_minor()==dummy_ref::protocol::kSchemaMinor && value.boot_epoch()==self.raw_fault_trusted_boot &&
              value.profile_id()==self.parameters.profile_id && value.configured_mask()==self.parameters.configured_mask &&
              std::equal(std::begin(value.device_id()),std::end(value.device_id()),self.parameters.device_id.begin()) &&
              value.core_state()==9U && value.session_id()==0U && value.reference_epoch()==0U &&
              raw_fault_producer_sample_valid(now,value.produced_at_us(),value.state_seq(),self.raw_fault_trusted_boot!=0U,
                self.raw_fault_trusted_sequence,self.raw_fault_trusted_produced,self.raw_fault_trusted_offset,self.raw_fault_trusted_anchor_offset,
                self.raw_fault_trusted_anchor_received,self.raw_fault_clock,self.parameters.max_state_age_us)))
        self.failed = true;
    }
    ++self.states;
    self.last_callback_received_us = now;
    self.last_callback_produced_us = value.produced_at_us();
    self.last_callback_sequence = value.state_seq();
    self.process_decoded_state(value,now);
  }

  /**
   * @brief 在严格身份与双时钟锚下接纳State并发布新鲜实际缓存。
   * 先核bootstrap/boot/授权或受限释放Idle例外，再核生产时间/序号递增，估计排队年龄并限制偏移校正。普通状态交system.ingest，raw Idle只更新只读观测且使普通硬件缓存失效。只有通过后才刷新heartbeat、票据水位、源锚及host deadline；锁忙/非法帧不会续龄。
   * @param value 复制的State值，position为rad、velocity为rad/s、raw为电机rev。
   * @param now 最初宿主接收µs，不是设备生产时间；重放也保持原值。
   * @param replay True时额外计入从原接收至现在的宿主年龄，防早到缓存续鲜。
   */
  void process_decoded_state(const State &value,std::uint64_t now,bool replay=false) {
    diagnostics.publish_context(
      {value.boot_epoch(),value.session_id(),value.reference_epoch(),value.motion_epoch(),value.state_seq(),value.management_epoch(),true},
      {boot,status.session_id(),status.reference_epoch(),authorized_motion_epoch,last_sequence,status.management_epoch(),boot!=0U});
    auto &self=*this;
    if(!replay && self.worker_idle_state_candidate(value) && now>=self.gate_release.sent_us)
      self.early_release_idle={value,self.gate_release.request_id,now};
    if (!self.bootstrap_ready || !self.matches_state(value) || self.boot == 0U ||
        value.boot_epoch() != self.boot) {
      ++self.state_identity_rejections;
      self.log_state_rejection("identity_or_bootstrap", value, now);
      ++self.rejected_states;
      return;
    }
    const auto actual_now=transport_steady_now_us();
    if(!self.gate_release.committed && self.stopping_release_idle_candidate(value) &&
       now>=self.gate_release.sent_us && actual_now>=self.last_status_received_us &&
       self.last_status_received_us!=0U &&
       actual_now-self.last_status_received_us<=self.parameters.max_state_age_us)
      self.early_release_idle={value,self.gate_release.request_id,now};
    const bool provisional_release = self.provisional_release_idle(value);
    if (!self.matches_state_authority(value) && !provisional_release) {
      ++self.state_authority_rejections;
      self.log_state_rejection("authority", value, now);
      ++self.rejected_states;
      return;
    }
    /**
     * @brief 同producer由boot/session/reference/motion四元组决定；同域内produced与32位state_seq都必须严格前进，不能用新宿主接收时间掩盖重放载荷。
     */
    const bool same_producer = self.last_state_boot == value.boot_epoch() &&
        self.last_state_session == value.session_id() &&
        self.last_state_reference == value.reference_epoch() &&
        self.last_state_motion == value.motion_epoch();
    // Clock epochs differ. The smallest observed receive-minus-produce offset
    // bounds later queueing delay without assuming the clocks are synchronized.
    const auto producer_anchor = self.has_sequence ? self.producer_anchor_us : value.produced_at_us();
    if (value.produced_at_us() == 0U ||
        (same_producer && value.produced_at_us() <= self.last_produced_us) ||
        (same_producer && !sequence_newer(value.state_seq(), self.last_sequence)) ||
        now > static_cast<std::uint64_t>(INT64_MAX) ||
        value.produced_at_us() > static_cast<std::uint64_t>(INT64_MAX)) {
      if (same_producer && value.produced_at_us() != 0U &&
          value.produced_at_us() <= self.last_produced_us &&
          self.last_produced_us - value.produced_at_us() > self.parameters.max_state_age_us)
        ++self.stale_payload_rejections;
      ++self.state_sequence_rejections;
      self.log_state_rejection("sequence_or_timestamp", value, now);
      ++self.rejected_states;
      return;
    }
    /**
     * @brief offset=host_receive−device_produced；其最小观测近似固定epoch偏差加最短传输延迟。后续offset高出的部分才是额外queue_age，不假定两时钟同步。
     */
    const auto observed_offset = static_cast<std::int64_t>(now) -
        static_cast<std::int64_t>(value.produced_at_us());
    /**
     * @brief 源时钟相对宿主突然快进会让offset大降：相对初锚最多允许状态预算加经过时间/1000的漂移带，超出记clock_skew拒绝。
     */
    if (self.has_sequence && observed_offset < self.clock_anchor_offset_us) {
      const auto cumulative_correction = static_cast<std::uint64_t>(self.clock_anchor_offset_us) -
          static_cast<std::uint64_t>(observed_offset);
      const auto elapsed_us = now - self.clock_anchor_received_us;
      if (cumulative_correction > self.parameters.max_state_age_us + elapsed_us / 1000U) {
        ++self.clock_skew_rejections;
        self.log_state_rejection("clock_skew", value, now);
        ++self.rejected_states;
        return;
      }
    }
    /**
     * @brief 最小offset只允许有限向下校正；不能每帧无条件重新对齐，否则长期旧载荷或异常源时钟会被重新解释为新鲜。
     */
    auto minimum_offset = self.has_sequence ? self.producer_offset_us : observed_offset;
    if (observed_offset < minimum_offset) {
      const auto correction = static_cast<std::uint64_t>(minimum_offset) -
          static_cast<std::uint64_t>(observed_offset);
      if (correction > self.parameters.max_state_age_us) {
        ++self.clock_skew_rejections;
        self.log_state_rejection("clock_skew", value, now);
        ++self.rejected_states;
        return;
      }
      minimum_offset = observed_offset;
    }
    /**
     * @brief queue_age是同一偏移基准下的非负额外排队µs；只使用差值而非跨epoch绝对时间比较。
     */
    const auto queue_age = static_cast<std::uint64_t>(observed_offset) -
        static_cast<std::uint64_t>(minimum_offset);
    /**
     * @brief 重放还必须加上缓存驻留的宿主年龄，且不得倒退last_received；使用budget−queue_age防加法溢出。原锚保持，不把重放处理时刻当新到达。
     */
    if(replay) {
      const auto actual_now=transport_steady_now_us();
      const auto budget=self.parameters.max_state_age_us;
      if(actual_now<now || now<self.last_received_us ||
         queue_age>budget || actual_now-now>budget-queue_age) {
        ++self.stale_payload_rejections;
        self.log_state_rejection("replay_age",value,now);
        ++self.rejected_states;
        return;
      }
    }
    if (value.produced_at_us() > UINT64_MAX - queue_age) {
      ++self.state_sequence_rejections;
      self.log_state_rejection("timestamp_overflow", value, now);
      ++self.rejected_states;
      return;
    }
    /**
     * @brief expected.now_us构造成device_produced+queue_age，因此validate_state的年龄运算处于源时间域；host stale deadline则在后面单独按原host receive建立。
     */
    const StateExpectation expected{self.boot, self.parameters.profile_id, self.last_sequence,
        self.has_sequence, self.parameters.configured_mask, producer_anchor,
        value.produced_at_us() + queue_age, self.parameters.max_state_age_us};
    std::uint8_t bytes[wire::DummyJointStateV2Codec::kEncodedSize]{};
    if (wire::DummyJointStateV2Codec::encode(value, bytes, sizeof(bytes)) != wire::CodecResult::kOk) {
      ++self.state_decode_rejections;
      self.log_state_rejection("codec", value, now);
      ++self.rejected_states;
      return;
    }
    const bool raw_idle = self.raw_idle_observation_valid(value, expected, provisional_release);
    /**
     * @brief Stop锁存期间不接纳普通控制State；只有严格raw Idle释放路径可继续观察，避免旧运动状态重开控制资格。
     */
    if (self.stop_latched && !raw_idle) {
      ++self.stop_latched_state_rejections;
      self.log_state_rejection("stop_latched", value, now);
      ++self.rejected_states;
      return;
    }
    /**
     * @brief system.ingest还会检查配置轴位置/有效速度有限性及try_lock发布；失败不推进本地接受水位，缺有效位置时进一步全部失效。
     */
    if (!raw_idle && !self.system.ingest_wire_state(bytes, sizeof(bytes), expected)) {
      if (queue_age > self.parameters.max_state_age_us) ++self.stale_payload_rejections;
      ++self.state_ingest_rejections;
      self.log_state_rejection(queue_age > self.parameters.max_state_age_us ?
          "age" : "ingest", value, now);
      ++self.rejected_states;
      if ((value.position_valid_mask() & self.parameters.configured_mask) != self.parameters.configured_mask)
        self.system.invalidate_transport_state();
      return;
    }
    // Validation above has bound the fresh State to the current boot/profile and
    // Status session/reference/motion/management identity. Native retains the same
    // producer high-water mark across management transitions; a new ROS process
    // must not restart below the device's admitted/committed sequence.
    if (!advance_control_sequence_floor(std::max(value.admitted_producer_seq(),
                                               value.committed_producer_seq()))) {
      ++self.rejected_states;
      self.invalidate();
      return;
    }
    if (raw_idle) self.system.invalidate_transport_state();
    if (raw_idle && self.stop_latched) self.stop_idle_observed = true;
    self.last_observation_raw_idle = raw_idle;
    self.heartbeat.observe_state(now);
    self.heartbeat_warned = false;
    self.heartbeat_stopped = false;
    /**
     * @brief 记录票据在当前完整身份中首次接收时刻；同ticket反复出现不得重置50ms发送窗，窗口是宿主保守上界而非签发时钟。
     */
    if (self.last_freshness_ticket != value.freshness_ticket() ||
        self.last_state_boot != value.boot_epoch() ||
        self.last_state_session != value.session_id() ||
        self.last_state_reference != value.reference_epoch() ||
        self.last_state_motion != value.motion_epoch())
      self.last_ticket_received_us = now;
    self.last_freshness_ticket = value.freshness_ticket();
    if (!self.has_sequence) {
      self.producer_anchor_us = value.produced_at_us();
      self.clock_anchor_offset_us = observed_offset;
      self.clock_anchor_received_us = now;
    }
    /**
     * @brief 只有通过正常Running身份/时序的State才能建立故障清理可信源锚；故障session0原页不能反过来初始化这个正常锚。
     */
    if(value.core_state()==5U && value.session_id()!=0U && value.reference_epoch()!=0U) {
      if(self.raw_fault_trusted_boot!=value.boot_epoch())self.raw_fault_clock={};
      self.raw_fault_trusted_boot=value.boot_epoch();self.raw_fault_trusted_sequence=value.state_seq();
      self.raw_fault_trusted_produced=value.produced_at_us();self.raw_fault_trusted_offset=minimum_offset;
      self.raw_fault_trusted_anchor_offset=self.clock_anchor_offset_us;self.raw_fault_trusted_anchor_received=self.clock_anchor_received_us;
    }
    self.producer_offset_us = minimum_offset;
    self.last_produced_us = value.produced_at_us();
    if (!raw_idle) self.acknowledge_command(value);
    {
      std::lock_guard<std::mutex> lock(self.management->mutex);
      std::copy(std::begin(bytes),std::end(bytes),self.management->observation.state.begin());
      self.management->observation.state_received_us=now;
    }
    /**
     * @brief 普通State设置host失鲜截止并记录实际Hold；raw Idle保留原始观测供管理，但不导出可控制的关节参考缓存。
     */
    if (!raw_idle) {
      self.system.set_state_deadline(saturating_state_deadline(now, self.parameters.max_state_age_us));
      self.observed_hold={};
      if(value.core_state()==kWireHold && value.hold_ready() &&
         (value.position_valid_mask() & self.parameters.configured_mask)==self.parameters.configured_mask) {
        self.observed_hold.boot_epoch=value.boot_epoch();
        self.observed_hold.session_id=value.session_id();
        self.observed_hold.reference_epoch=value.reference_epoch();
        self.observed_hold.motion_epoch=value.motion_epoch();
        self.observed_hold.committed_seq=value.committed_producer_seq();
        self.observed_hold.received_us=now;
        std::copy(std::begin(value.position_rad()),std::end(value.position_rad()),
                  self.observed_hold.position.begin());
      }
    }
    self.last_sequence = value.state_seq();
    self.last_state_boot = value.boot_epoch();
    self.last_state_session = value.session_id();
    self.last_state_reference = value.reference_epoch();
    self.last_state_motion = value.motion_epoch();
    self.last_state_management_epoch = value.management_epoch();
    self.has_sequence = true;
    self.last_received_us = now;
    self.cancel_prime_latest_core=value.core_state();
    /**
     * @brief 仅收集本Cancel episode的已接纳EnablingHold实测目标；配置轴须全部有限，64槽在250ms窗口内耗尽时不覆盖而锁overflow。
     */
    if(value.core_state()==kWireEnablingHold && self.cancel_prime_episode_request!=0U &&
       value.position_valid_mask()==self.parameters.configured_mask) {
      bool finite=true;
      for(std::size_t axis=0;axis<6;++axis)
        if(self.parameters.configured_mask & (1U<<axis))finite=finite && std::isfinite(value.position_rad()[axis]);
      if(finite) {
        auto &source=self.cancel_prime_sources[self.cancel_prime_source_next];
        if(self.cancel_prime_source_count==self.cancel_prime_sources.size() &&
           (now<source.received_us || now-source.received_us<=250000U))self.cancel_prime_source_overflow=true;
        else {
          source.root={value.boot_epoch(),value.session_id(),value.reference_epoch(),value.motion_epoch(),value.freshness_ticket()};
          source.management=value.management_epoch();source.state_seq=value.state_seq();source.profile=value.profile_id();
          source.mask=value.configured_mask();source.produced_us=value.produced_at_us();source.received_us=now;
          std::copy(std::begin(value.position_rad()),std::end(value.position_rad()),source.position.begin());
          self.cancel_prime_source_next=(self.cancel_prime_source_next+1U)%self.cancel_prime_sources.size();
          if(self.cancel_prime_source_count<self.cancel_prime_sources.size())++self.cancel_prime_source_count;
        }
      }
    }

  }

  /**
   * @brief 把设备原页绑定到已知或严格受限推断run并交collector验链。
   * 历史已释放run原页忽略；新run通常须由Status绑定，只有紧邻已验证完成/干净取消后的全六轴首运动页允许临时推断run+1。collector核序号/链后仅记“已接受终页”，writer flush回执仍另查。后续Status和终链不一致则粘滞Invalid。
   * @param borrowed 回调期借用AttemptEvidence原页。
   * @param context 非空Impl指针。
   */
  static void receive_evidence(dp::comm::BorrowedMessage<Evidence> borrowed, void *context) noexcept {
    auto &self = *static_cast<Impl *>(context);
    Evidence value;
    if (borrowed.copy_to(&value) != dp::comm::Result::kOk) {
      diagnostic_detail("RAW_EVIDENCE_DIAG copy_failed\n");
      self.raw_evidence.reject_invalid();
      self.failed = true;
      return;
    }
    if (self.boot == self.historical_run_boot && self.historical_run_id != 0U &&
        value.run_id() == self.historical_run_id) return;
    RawEvidencePage page{};
    const auto received_at_us = transport_steady_now_us();
    auto *binding = self.find_evidence_binding(value.run_id());
    /**
     * @brief 以下多个布尔量分别描述正常完成或干净取消的前一run证据与新鲜Hold组合；它们是处理Evidence/Status到达乱序的窄例外，不是任意未知run准入。
     */
    const bool canceled_refresh_first_page =
        self.last_clean_cancel_run != 0U &&
        self.last_clean_cancel_run == self.status.run_id() &&
        self.last_clean_cancel_boot == self.boot &&
        self.last_clean_cancel_session == self.status.session_id() &&
        self.status.core_state() == 7U && self.status.final_state() == 0U;
    const bool completed_refresh_first_page =
        self.status.core_state() == kWireHold &&
        self.status.final_state() == 6U && self.status.motion_complete() &&
        self.last_terminal_chain_run == self.status.run_id() &&
        std::any_of(std::begin(self.status.evidence_chain_sha256()),
            std::end(self.status.evidence_chain_sha256()),
            [](auto byte) { return byte != 0U; }) &&
        std::equal(self.last_terminal_chain.begin(), self.last_terminal_chain.end(),
            self.status.evidence_chain_sha256()) &&
        self.raw_evidence.failure() == RawEvidenceCollector::Failure::kNone;
    /**
     * @brief 管理channel回执来自writer写出/flush；accepted_completed_terminal可能仅验链尚未写出。下面按writer水位明确区分，不能把两个结构同名字段误当同一层级。
     */
    // writer先在锁下发布回执，再release发布水位；先acquire水位后复制回执，
    // 新水位必能看见对应回执。后续资格判断共用该快照，避免旧副本/新水位混读。
    const auto terminal_written_run = self.raw_evidence.terminal_written_run_id();
    const auto flushed_terminal = !binding && value.evidence_seq() == 0U
        ? self.management->raw_evidence_terminal_flushed() : RawEvidenceTerminalFlushed{};
    // A successor page may arrive after terminal chain validation but before
    // the writer publishes its flush receipt. Binding is provisional only.
    const bool terminal_writer_completed = self.accepted_completed_terminal.run_id != 0U &&
        terminal_written_run >= self.accepted_completed_terminal.run_id;
    const auto &completed_terminal = terminal_writer_completed ||
        (flushed_terminal.run_id >= self.accepted_completed_terminal.run_id &&
         flushed_terminal.run_id != 0U)
        ? flushed_terminal : self.accepted_completed_terminal;
    const bool completed_terminal_page_first_page =
        self.status.core_state() == kWireEnablingHold &&
        self.status.final_state() == 0U &&
        self.status.motion_epoch() == value.motion_epoch() &&
        completed_terminal.boot_epoch == self.boot &&
        completed_terminal.boot_epoch == self.status.boot_epoch() &&
        completed_terminal.session_id == self.status.session_id() &&
        completed_terminal.run_id == self.status.run_id() &&
        completed_terminal.run_id == self.last_terminal_chain_run &&
        completed_terminal.attempt_kind == 1U &&
        completed_terminal.parent_run_id == 0U &&
        completed_terminal.final_state == 6U &&
        completed_terminal.motion_complete && !completed_terminal.evidence_lost &&
        !completed_terminal.release_committed && completed_terminal.chain_valid &&
        completed_terminal.first_failure_source == 0U &&
        completed_terminal.first_failure_axis_mask == 0U &&
        self.raw_evidence.failure() == RawEvidenceCollector::Failure::kNone;
    const bool prior_provisional_completed = self.provisional_evidence_binding &&
        self.provisional_evidence_binding->run_id == self.status.run_id() &&
        self.provisional_evidence_binding->boot_epoch == completed_terminal.boot_epoch &&
        self.provisional_evidence_binding->session_id == completed_terminal.session_id &&
        (!self.provisional_release_chain ||
         *self.provisional_release_chain == self.last_terminal_chain) &&
        completed_terminal_page_first_page;
    /**
     * @brief 后继run推断还要求最后接纳的实际Hold State恰是当前接收水位，身份和管理epoch与Status一致，且宿主年龄不超预算。
     */
    const bool fresh_matching_hold_state =
        self.observed_hold.received_us != 0U &&
        self.observed_hold.received_us == self.last_received_us &&
        received_at_us >= self.observed_hold.received_us &&
        received_at_us - self.observed_hold.received_us <= self.parameters.max_state_age_us &&
        self.observed_hold.boot_epoch == self.boot &&
        self.observed_hold.session_id == self.status.session_id() &&
        self.observed_hold.reference_epoch == self.status.reference_epoch() &&
        self.observed_hold.motion_epoch == self.status.motion_epoch() &&
        self.last_state_management_epoch == self.status.management_epoch();
    const bool fresh_completed_terminal_state = completed_terminal_page_first_page &&
        fresh_matching_hold_state;
    const bool flushed_canceled_terminal =
        flushed_terminal.boot_epoch == self.boot &&
        flushed_terminal.session_id == self.status.session_id() &&
        flushed_terminal.run_id == self.status.run_id() &&
        flushed_terminal.run_id == self.last_terminal_chain_run &&
        flushed_terminal.run_id == terminal_written_run &&
        flushed_terminal.attempt_kind == 1U && flushed_terminal.parent_run_id == 0U &&
        flushed_terminal.final_state == 7U && flushed_terminal.chain_valid &&
        !flushed_terminal.evidence_lost && !flushed_terminal.motion_complete &&
        !flushed_terminal.release_committed &&
        flushed_terminal.first_failure_source == 0U &&
        flushed_terminal.first_failure_axis_mask == 0U &&
        flushed_terminal.safety_hold_valid &&
        flushed_terminal.safety_hold_motion_epoch == self.status.motion_epoch() &&
        self.raw_evidence.failure() == RawEvidenceCollector::Failure::kNone;
    // 取消终页与后继首页可在同一批接收，不能以writer调度快慢决定是否接受原页。
    // 尚未flush时只借用collector同生产者的完整验链/安全Hold证人作provisional绑定；
    // 已有flush水位却回执不匹配时仍拒绝，最终ROS结果也继续要求实际flush。
    const bool accepted_cancel_pending =
        terminal_written_run < self.status.run_id() &&
        self.raw_evidence.accepted_cancel_hold(self.boot,self.status.session_id(),
            self.status.run_id(),self.status.motion_epoch());
    const bool fresh_canceled_terminal_state = canceled_refresh_first_page &&
        (flushed_canceled_terminal || accepted_cancel_pending) &&
        self.status.motion_epoch() == value.motion_epoch() &&
        self.raw_evidence.failure() == RawEvidenceCollector::Failure::kNone &&
        fresh_matching_hold_state;
    const bool fresh_canceled_hold_terminal_state =
        self.last_clean_cancel_run != 0U && self.last_clean_cancel_run == self.status.run_id() &&
        self.last_clean_cancel_boot == self.boot && self.last_clean_cancel_session == self.status.session_id() &&
        self.status.core_state() == kWireHold &&
        (self.status.final_state() == 0U ||
         (self.status.final_state() == 7U &&
          self.status.first_failure_reason() == 0U && self.status.failure_source() == 0U &&
          self.status.failure_axis_mask() == 0U &&
          std::any_of(std::begin(self.status.evidence_chain_sha256()),
              std::end(self.status.evidence_chain_sha256()), [](auto byte){return byte!=0U;}) &&
          std::equal(self.last_terminal_chain.begin(), self.last_terminal_chain.end(),
              self.status.evidence_chain_sha256()))) &&
        !self.status.motion_complete() && !self.status.release_committed() &&
        flushed_terminal.boot_epoch == self.boot &&
        flushed_terminal.session_id == self.status.session_id() &&
        flushed_terminal.run_id == self.status.run_id() &&
        flushed_terminal.run_id == self.last_terminal_chain_run &&
        flushed_terminal.run_id == terminal_written_run &&
        flushed_terminal.attempt_kind == 1U && flushed_terminal.parent_run_id == 0U &&
        flushed_terminal.final_state == 7U && flushed_terminal.chain_valid &&
        !flushed_terminal.evidence_lost && !flushed_terminal.motion_complete &&
        !flushed_terminal.release_committed &&
        flushed_terminal.first_failure_source == 0U &&
        flushed_terminal.first_failure_axis_mask == 0U &&
        flushed_terminal.safety_hold_valid &&
        flushed_terminal.safety_hold_motion_epoch == self.status.motion_epoch() &&
        self.status.motion_epoch() == value.motion_epoch() &&
        self.raw_evidence.failure() == RawEvidenceCollector::Failure::kNone &&
        fresh_matching_hold_state;
    /**
     * @brief 未知run只有seq0、根attempt、category1、所有六轴expected/submitted/sent=63且failed=0、无终态/错误的首组，及run恰前run+1时才考虑推断。
     */
    const bool next_motion_first_page = !binding &&
        (!self.provisional_evidence_binding || prior_provisional_completed) &&
        value.schema_major() == 2U && value.schema_minor() == dummy_ref::protocol::kSchemaMinor &&
        value.evidence_seq() == 0U && value.attempt_kind() == 1U &&
        value.parent_run_id() == 0U && value.category() == 1U &&
        value.group_count() >= 1U && value.group_id() != 0U &&
        value.motion_epoch() != 0U && value.final() && !value.evidence_lost() &&
        value.final_state() == 0U && value.terminal_reason() == 0U &&
        !value.motion_complete() && !value.release_committed() &&
        value.first_failure_source() == 0U && value.first_failure_axis_mask() == 0U &&
        value.expected_mask() == 0x3fU && value.submitted_mask() == 0x3fU &&
        value.sent_mask() == 0x3fU && value.failed_mask() == 0U &&
        !self.stop_latched && !self.failed && self.boot != 0U &&
        self.status.boot_epoch() == self.boot && self.status.session_id() != 0U &&
        self.status.reference_epoch() != 0U &&
        self.status.attempt_kind() == 1U && self.status.parent_run_id() == 0U &&
        !self.status.evidence_lost() &&
        self.status.run_id() != 0U && self.status.run_id() != UINT64_MAX &&
        (completed_refresh_first_page || completed_terminal_page_first_page ||
         canceled_refresh_first_page || fresh_canceled_hold_terminal_state) &&
        ((self.last_status_received_us != 0U &&
          received_at_us >= self.last_status_received_us &&
          received_at_us - self.last_status_received_us <= self.parameters.max_state_age_us) ||
         fresh_completed_terminal_state || fresh_canceled_terminal_state || fresh_canceled_hold_terminal_state) &&
        value.run_id() == self.status.run_id() + 1U;
    /**
     * @brief 推断绑定存为provisional而非最终真相；Status后到仍须对账原parent/session/链，stop时未对上也令采集失败。
     */
    if (next_motion_first_page) {
      const RawEvidenceBinding inferred{value.run_id(), self.status.boot_epoch(),
          self.status.session_id(), 0U, 1U};
      if (self.evidence_bindings.observe(inferred,
              self.raw_evidence.terminal_written_run_id()) &&
          self.raw_evidence.expect_run(inferred.run_id)) {
        self.provisional_evidence_binding = inferred;
        self.provisional_release_chain.reset();
        binding = self.find_evidence_binding(value.run_id());
      }
    }
    if (value.schema_major() != 2U || value.schema_minor() != dummy_ref::protocol::kSchemaMinor ||
        !binding || binding->attempt_kind != value.attempt_kind() ||
        binding->parent_run_id != value.parent_run_id() ||
        wire::DummyAttemptEvidenceV2Codec::encode(value, page.raw.data(), page.raw.size()) !=
            wire::CodecResult::kOk) {
      diagnostic_detail(
          "RAW_EVIDENCE_DIAG rejected run=%llu seq=%u kind=%u parent=%llu binding=%u binding_kind=%u binding_parent=%llu status_run=%llu status_core=%u status_motion=%u page_motion=%u terminal_written_run=%llu terminal_receipt_run=%llu terminal_receipt_final=%u failure=%u\n",
          static_cast<unsigned long long>(value.run_id()), value.evidence_seq(),
          value.attempt_kind(), static_cast<unsigned long long>(value.parent_run_id()),
          binding ? 1U : 0U, binding ? binding->attempt_kind : 0U,
          static_cast<unsigned long long>(binding ? binding->parent_run_id : 0U),
          static_cast<unsigned long long>(self.status.run_id()),
          self.status.core_state(), self.status.motion_epoch(), value.motion_epoch(),
          static_cast<unsigned long long>(self.raw_evidence.terminal_written_run_id()),
          static_cast<unsigned long long>(flushed_terminal.run_id),
          flushed_terminal.final_state,
          static_cast<unsigned>(self.raw_evidence.failure()));
      self.raw_evidence.reject_invalid();
      self.failed = true;
      return;
    }
    std::copy(std::begin(value.previous_chain_sha256()),
              std::end(value.previous_chain_sha256()), page.previous_chain_sha256.begin());
    std::copy(std::begin(value.chain_sha256()),
              std::end(value.chain_sha256()), page.chain_sha256.begin());
    /**
     * @brief Evidence页本身不携完整boot/session时由已验证binding补入collector包装；实际codec原字节不改写，链验证仍用原page.raw。
     */
    page.boot_epoch = binding->boot_epoch;
    page.session_id = binding->session_id;
    page.run_id = value.run_id();
    page.parent_run_id = value.parent_run_id();
    page.evidence_seq = value.evidence_seq();
    page.group_count = value.group_count();
    page.terminal_reason = value.terminal_reason();
    page.attempt_kind = value.attempt_kind();
    page.category = value.category();
    page.final_state = value.final_state();
    page.final = value.final() != 0U;
    page.evidence_lost = value.evidence_lost();
    if (!self.raw_evidence.offer_page(page, received_at_us)) {
      diagnostic_detail(
          "RAW_EVIDENCE_DIAG collector_rejected run=%llu seq=%u kind=%u category=%u failure=%u\n",
          static_cast<unsigned long long>(page.run_id), page.evidence_seq,
          page.attempt_kind, page.category,
          static_cast<unsigned>(self.raw_evidence.failure()));
      self.failed = true;
    }
    /**
     * @brief category4才是attempt终页；last_terminal_chain代表已验链，不等于磁盘flush。保留完成/干净取消摘要以支持受限后继run，flush水位继续由writer发布。
     */
    else if(page.category==4U) {
      self.last_terminal_chain_run=page.run_id;
      self.last_terminal_chain=page.chain_sha256;
      self.accepted_completed_terminal = page.attempt_kind==1U &&
          page.parent_run_id==0U && page.final_state==6U &&
          value.motion_complete() && !page.evidence_lost &&
          !value.release_committed() && value.first_failure_source()==0U &&
          value.first_failure_axis_mask()==0U
          ? RawEvidenceTerminalFlushed{page.boot_epoch,page.session_id,page.run_id,
                page.parent_run_id,page.attempt_kind,page.final_state,0U,0U,
                false,true,false,true,0U,false}
          : RawEvidenceTerminalFlushed{};
      if(self.accepted_completed_terminal.run_id!=0U) {
        auto &witness=self.completed_terminal_chains[self.next_completed_terminal_chain];
        witness={page.boot_epoch,page.session_id,page.run_id,page.chain_sha256};
        self.next_completed_terminal_chain=
            (self.next_completed_terminal_chain+1U)%self.completed_terminal_chains.size();
      }
      if (page.attempt_kind == 1U && page.parent_run_id == 0U &&
          page.final_state == 7U && value.first_failure_source() == 0U &&
          value.first_failure_axis_mask() == 0U && !page.evidence_lost &&
          !value.motion_complete() && !value.release_committed()) {
        self.last_clean_cancel_boot=page.boot_epoch;
        self.last_clean_cancel_session=page.session_id;
        self.last_clean_cancel_run=page.run_id;
      } else {
        self.last_clean_cancel_run=0U;
      }
      if (self.provisional_evidence_binding && self.provisional_release_chain &&
          self.provisional_evidence_binding->run_id == page.run_id &&
          *self.provisional_release_chain != page.chain_sha256) {
        self.raw_evidence.reject_invalid();
        self.failed = true;
      }
    }
  }

  /**
   * @brief dp_comm响应回调复制固定响应字节并发布complete。
   * ResponseInfo当前未使用；只检查RPC/codec成功，管理phase/result/身份由消费方进一步核对。字节写完后complete release，读取方acquire后才可解码；失败置failed而非假造complete。
   * @param result 本地RPC返回状态。
   * @param response 仅回调期有效的响应指针，失败可空。
   * @param context 仍存活的DeviceManagementRequest地址；RequestHandle与shared_ptr由循环同上下文管理。
   */
  static void management_reply(dp::comm::Result result,const Management::Response *response,
      const dp::comm::ResponseInfo &,void *context) {
    auto &request=*static_cast<DeviceManagementRequest *>(context);
    if(result!=dp::comm::Result::kOk || !response ||
       wire::DummyManageV2ResponseCodec::encode(*response,request.response.data(),request.response.size())!=wire::CodecResult::kOk) {
      log_management_handoff(request,"reply_failed",result);
      request.failed=true; return;
    }
    request.complete.store(true,std::memory_order_release);
  }

  /**
   * @brief 用当前可用授权水位构造D10失能释放根请求。
   * 绑定设备/客户端、session/reference、授权motion/management和最近票据，request_id及request_seq使用不同分配器；构造尚未发送，且不保证当时票据仍在设备接受窗口。
   * @return 共享请求对象；缺boot/profile/非零授权epoch、ID耗尽或编码失败返回空。
   */
  std::shared_ptr<DeviceManagementRequest> make_stop_request() noexcept {
    if (status.boot_epoch() == 0U || status.profile_id() != parameters.profile_id ||
        authorized_motion_epoch == 0U || authorized_management_epoch == 0U) return {};
    Management::Request request;
    request.set_schema_major(2U);
    request.set_schema_minor(dummy_ref::protocol::kSchemaMinor);
    std::uint8_t device[16]{}, client[16]{};
    std::copy(parameters.device_id.begin(), parameters.device_id.end(), device);
    std::copy(parameters.client_instance_id.begin(), parameters.client_instance_id.end(), client);
    request.set_device_id(device);
    request.set_client_instance_id(client);
    const auto request_id = take_stop_request_id();
    if (request_id == 0U) return {};
    request.set_request_id(request_id);
    request.set_request_seq(management->reserve_request_sequence());
    request.set_operation(kWireDisableAndRelease);
    request.set_expected_boot_epoch(status.boot_epoch());
    request.set_expected_session_id(status.session_id());
    request.set_expected_reference_epoch(status.reference_epoch());
    request.set_expected_motion_epoch(authorized_motion_epoch);
    request.set_expected_management_epoch(authorized_management_epoch);
    request.set_expected_profile_id(parameters.profile_id);
    request.set_freshness_ticket(last_freshness_ticket);
    request.set_configured_mask(parameters.configured_mask);
    auto pending = std::make_shared<DeviceManagementRequest>();
    if (wire::DummyManageV2RequestCodec::encode(
            request, pending->request.data(), pending->request.size()) != wire::CodecResult::kOk) return {};
    return pending;
  }

  /**
   * @brief 检查Stop/Query响应与原D10根的基本身份绑定。
   * 调用方再按阶段加严格条件，不能把此基础匹配结果当完整停车确认。
   * @param original 最初D10请求，整个停止episode固定。
   * @param sent 本次实际发送D10或查原request_id的GetOperation。
   * @param response 待检查响应。
   * @param released_identity True允许响应session/reference已归零，其他身份仍须匹配。
   * @return bool；不单独判断Pending/COMMITTED、result成功、release_committed或Idle。
   */
  bool stop_reply_matches(const Management::Request &original,
                          const Management::Request &sent,
                          const Management::Response &response,
                          bool released_identity = false) const noexcept {
    const bool same_owner = response.session_id() == original.expected_session_id() &&
        response.reference_epoch() == original.expected_reference_epoch();
    const bool released_owner =
        released_identity && response.session_id() == 0U &&
        (response.reference_epoch() == 0U ||
         (parameters.manual_reference_mode &&
          response.reference_epoch() == original.expected_reference_epoch()));
    return original.operation() == kWireDisableAndRelease &&
        (sent.operation() == kWireDisableAndRelease ||
         (sent.operation() == kWireGetOperation &&
          sent.queried_operation_id() == original.request_id() &&
          sent.configured_mask() == original.configured_mask())) &&
        response.schema_major() == 2U && response.schema_minor() == dummy_ref::protocol::kSchemaMinor &&
        response.receipt() == 1U && response.request_id() == sent.request_id() &&
        response.boot_epoch() == original.expected_boot_epoch() &&
        (same_owner || released_owner) &&
        response.profile_id() == original.expected_profile_id() &&
        sequence_at_or_after(response.motion_epoch(), original.expected_motion_epoch()) &&
        sequence_at_or_after(response.management_epoch(), original.expected_management_epoch()) &&
        std::equal(parameters.device_id.begin(), parameters.device_id.end(), response.device_id()) &&
        std::equal(parameters.client_instance_id.begin(), parameters.client_instance_id.end(),
                   response.client_instance_id());
  }

  /**
   * @brief 复制原停止根并分配新请求ID/序号查询原事务。
   * 查询不是重复发送D10，不改变原停止总deadline；新RPC自己的本地等待期限不能扩展总确认预算。
   * @param original 原始D10请求；保留其期待身份/掩码。
   * @return GetOperation共享请求或空；queried_operation_id取原request_id。
   */
  std::shared_ptr<DeviceManagementRequest> make_stop_query(
      const Management::Request &original) noexcept {
    Management::Request query = original;
    const auto request_id = take_stop_request_id();
    if (request_id == 0U) return {};
    query.set_request_id(request_id);
    query.set_request_seq(management->reserve_request_sequence());
    query.set_operation(kWireGetOperation);
    query.set_queried_operation_id(original.request_id());
    auto pending = std::make_shared<DeviceManagementRequest>();
    if (wire::DummyManageV2RequestCodec::encode(
            query, pending->request.data(), pending->request.size()) != wire::CodecResult::kOk) return {};
    return pending;
  }

  /**
   * @brief 消费硬件有限队列并在所有本地准入满足时尝试发布位置命令。
   * Complete冻结旧终序号，后继同值可本地吞并，不同值请求Stop。候选须当前authority、完整身份、bootstrap、非零线序号/票据和有效硬件状态；safe_cancel_prime额外逐值匹配本episode实测来源并固定安全目标。Busy/QueueFull保留同一未送代，Ok才计本地publish并建立commit等待，其他失败丢在途。
   * @param publisher 同worker拥有的Command发布器。
   * @param cancel_prime 可空的本Cancel根请求，仅允许专用安全prime例外。
   * @param cancel_deadline_us 该Cancel→Disable原停止总截止，宿主µs。
   * @param cancel_run 本Cancel绑定run，不能任意换新run。
   */
  void publish_pending(dp::comm::Publisher<Command> &publisher,
                       const Management::Request *cancel_prime = nullptr,
                       std::uint64_t cancel_deadline_us = 0U, std::uint64_t cancel_run = 0U) noexcept {
    CommandGenerationRecord next{};
    /**
     * @brief worker唯一消费者先把4槽队列排入“一个in-flight+最新pending”；因此不会无限积压旧采样，Complete/本地no-op在进入网络账本前处理。
     */
    while (system.take_publish_record(&next)) {
      // Complete freezes the terminal sequence; a later old-epoch update must
      // not enter REF's motion ledger after that terminal group was SENT.
      if(superseded_by_pending_complete(next)) continue;
      // No new-epoch write is authorized before a new Begin. Local no-ops do
      // not claim CAN SENT or extend REF's Hold budget; divergent writes Stop.
      const auto action=post_complete_write(next,transport_steady_now_us());
      if(action==PostCompleteWrite::kConflict) {
        ++rejected_commands;
        hold_drain_rejected=true;
        system.request_transport_stop();
      } else if(action==PostCompleteWrite::kAllowed) command_stream.offer(next);
      if(action==PostCompleteWrite::kLocalNoOp &&
         next.identity.boot==status.boot_epoch() &&
         next.identity.session==status.session_id() &&
         next.identity.reference_epoch==status.reference_epoch() &&
         next.identity.motion_epoch==authorized_motion_epoch &&
         status.core_state()==kWireHold) {
        last_processed_hold_identity=next.identity;
        last_processed_hold_generation=next.generation;
        last_processed_hold_disposition=1U;
        last_processed_hold_position=next.position;
      }
    }
    /**
     * @brief queue_drained仅说明本轮已检查硬件队列，不等于末组commit；run_connected还须等terminal_seq匹配ACK、无in-flight/pending才真正发送Complete。
     */
    if(pending_complete.request_id!=0U && !pending_complete.dispatched)
      pending_complete.queue_drained=true;
    /**
     * @brief 已本地发送成功且等待设备commit时不得再选新命令；本地Busy/QueueFull尚未成功则保留原in-flight记录重试，不能每次制造新producer_seq。
     */
    const auto candidate = command_sent ? std::optional<CommandGenerationRecord>{}
                                        : (command_stream.has_in_flight()
                                               ? command_stream.in_flight_record()
                                               : command_stream.begin_send());
    if (!candidate.has_value()) return;
    const auto &record = *candidate;
    if(!command_sent && superseded_by_pending_complete(record)) {
      command_stream.discard_in_flight();
      return;
    }
    const auto post_complete=!command_sent ? post_complete_write(record,transport_steady_now_us()) :
      PostCompleteWrite::kAllowed;
    if(post_complete!=PostCompleteWrite::kAllowed) {
      if(post_complete==PostCompleteWrite::kLocalNoOp && status.core_state()==kWireHold &&
         record.identity.boot==status.boot_epoch() && record.identity.session==status.session_id() &&
         record.identity.reference_epoch==status.reference_epoch() &&
         record.identity.motion_epoch==authorized_motion_epoch &&
         record.generation>last_processed_hold_generation) {
        last_processed_hold_identity=record.identity;
        last_processed_hold_generation=record.generation;
        last_processed_hold_disposition=1U;
        last_processed_hold_position=record.position;
      }
      command_stream.discard_in_flight();
      if(post_complete==PostCompleteWrite::kConflict) {
        ++rejected_commands;
        system.request_transport_stop();
      }
      if(post_complete==PostCompleteWrite::kConflict) hold_drain_rejected=true;
      return;
    }
    JointStateSnapshot snapshot{};
    // 安全目标须来自本Cancel episode已接纳的EnablingHold帧；发送ticket独立保原50ms门。
    bool safe_cancel_prime=false;
    CancelPrimeSource prime_source{};
    const auto prime_now=transport_steady_now_us();
    /**
     * @brief 安全Cancel prime不是普通允许发送：要求原Cancel根、双fresh实际状态、后继epoch、同run且无故障，候选目标必须逐轴精确等于本episode已接纳EnablingHold源帧。
     */
    if(stop_confirmation_pending.load(std::memory_order_acquire) && cancel_prime && !stop_latched &&
       cancel_prime->request_id()==cancel_prime_episode_request && !cancel_prime_source_overflow &&
       prime_now<=cancel_deadline_us && last_received_us!=0U && last_status_received_us!=0U &&
       prime_now>=last_received_us && prime_now>=last_status_received_us &&
       prime_now-last_received_us<=parameters.max_state_age_us && prime_now-last_status_received_us<=parameters.max_state_age_us &&
       cancel_prime->expected_motion_epoch()!=UINT32_MAX && cancel_prime->expected_management_epoch()!=UINT32_MAX &&
       (status.core_state()==kWireEnablingHold || status.core_state()==7U) && cancel_prime_latest_core==kWireEnablingHold && cancel_run!=0U && status.run_id()==cancel_run &&
       status.attempt_id()==cancel_run && status.attempt_kind()==1U && status.parent_run_id()==0U &&
       !status.evidence_lost() && !status.release_committed() && !status.motion_complete() &&
       status.boot_epoch()==cancel_prime->expected_boot_epoch() &&
       status.session_id()==cancel_prime->expected_session_id() && status.reference_epoch()==cancel_prime->expected_reference_epoch() &&
       status.motion_epoch()==cancel_prime->expected_motion_epoch()+1U && status.management_epoch()==cancel_prime->expected_management_epoch()+1U &&
       status.profile_id()==cancel_prime->expected_profile_id() && status.configured_mask()==cancel_prime->configured_mask() &&
       last_state_boot==status.boot_epoch() && last_state_session==status.session_id() && last_state_reference==status.reference_epoch() &&
       last_state_motion==status.motion_epoch() && last_state_management_epoch==status.management_epoch() &&
       std::equal(parameters.device_id.begin(),parameters.device_id.end(),cancel_prime->device_id()) &&
       std::equal(parameters.client_instance_id.begin(),parameters.client_instance_id.end(),cancel_prime->client_instance_id()) &&
       sequence_newer(wire_producer_sequence(record.generation),status.committed_producer_seq())) {
      /**
       * @brief 核对一个Cancel安全prime来源仍新鲜且目标逐值相同。
       * 该局部谓词不替代后续50ms发送ticket窗；安全目标来源和本次发送票据分开核。
       * @param source 64槽或已锁定首目标中的来源记录。
       * @return bool；250ms来源窗、完整身份/profile/mask及所有配置轴rad值精确一致。
       */
      const auto matches=[&](const CancelPrimeSource &source) {
        if(source.received_us==0U || prime_now<source.received_us || prime_now-source.received_us>250000U ||
           source.root.boot!=record.identity.boot || source.root.session!=record.identity.session ||
           source.root.reference_epoch!=record.identity.reference_epoch || source.root.motion_epoch!=record.identity.motion_epoch ||
           source.management!=status.management_epoch() || source.profile!=parameters.profile_id || source.mask!=parameters.configured_mask)return false;
        for(std::size_t axis=0;axis<record.position.size();++axis)
          if(parameters.configured_mask & (1U<<axis))
            if(!std::isfinite(record.position[axis]) || record.position[axis]!=source.position[axis])return false;
        return true;
      };
      if(cancel_prime_target_latched) {
        safe_cancel_prime=matches(cancel_prime_target);prime_source=cancel_prime_target;
      } else for(std::size_t i=0;i<cancel_prime_source_count;++i)
        if(matches(cancel_prime_sources[i])) {safe_cancel_prime=true;prime_source=cancel_prime_sources[i];break;}
    }
    /**
     * @brief inactive prime用于明确准备保持或Cancel过渡；绝非允许所有inactive硬件目标发送。其余authority/epoch/bootstrap/Stop条件仍逐次检查。
     */
    const bool inactive_prime = safe_cancel_prime || management->inactive_prime_allowed.load(std::memory_order_acquire) ||
        status.core_state() == kWireEnablingHold;
    if (wire_producer_sequence(record.generation) == 0U || !command_record_current(record) ||
        (!system.commands_active() && !inactive_prime) || boot == 0U ||
        record.identity.boot != boot || record.identity.session != status.session_id() ||
        record.identity.reference_epoch != status.reference_epoch() ||
        record.identity.motion_epoch != authorized_motion_epoch || record.identity.ticket == 0U ||
        !bootstrap_ready || stop_latched || (stop_confirmation_pending.load(std::memory_order_acquire) && !safe_cancel_prime)) {
      if (command_prime_reject_diag_count++ < 16U) diagnostic_detail(
          "COMMAND_PRIME_DIAG reject seq=%u current=%d active=%d inactive_prime=%d boot=%llu/%llu session=%llu/%llu ref=%u/%u motion=%u/%u ticket=%llu ready=%d stop=%d\n",
          wire_producer_sequence(record.generation), command_record_current(record),
          system.commands_active(), inactive_prime,
          static_cast<unsigned long long>(record.identity.boot), static_cast<unsigned long long>(boot),
          static_cast<unsigned long long>(record.identity.session),
          static_cast<unsigned long long>(status.session_id()), record.identity.reference_epoch,
          status.reference_epoch(), record.identity.motion_epoch, authorized_motion_epoch,
          static_cast<unsigned long long>(record.identity.ticket), bootstrap_ready, stop_latched);
      ++rejected_commands;
      hold_drain_rejected=true;
      command_stream.discard_in_flight();
      command_sent = false;
      return;
    }
    // On a busy state read, keep the original unsent generation. Authority and
    // freshness are checked again before every retry; no new sample is made.
    /**
     * @brief try_lock忙或状态不可用时只保留原未送记录；每次回来重新核身份和新鲜度，不用重复采样的新generation掩盖迟到。
     */
    if(!system.copy_state(&snapshot)) {
      if (record.identity.motion_epoch == 1U && enabling_copy_failure_log_count++ < 3U)
        diagnostic_detail( "COMMAND_ENABLE_DIAG copy_state_unavailable seq=%u status_core=%u status_motion=%u authorized_motion=%u\n",
                wire_producer_sequence(record.generation), status.core_state(),
                status.motion_epoch(), authorized_motion_epoch);
      if (!logged_copy_state_failure) {
        logged_copy_state_failure = true;
        diagnostic_detail( "COMMAND_PRIME_DIAG copy_state_unavailable seq=%u boot=%llu status_motion=%u authorized_motion=%u\n",
                wire_producer_sequence(record.generation),
                static_cast<unsigned long long>(boot), status.motion_epoch(), authorized_motion_epoch);
      }
      return;
    }
    // The current device profiles expire tickets after 50 ms from issuance.
    // First receipt is only an upper bound on issuance, so repeated States
    // carrying the same ticket must not renew the send window.
    const auto publish_now = transport_steady_now_us();
    /**
     * @brief 设备票据公开过期为签发后50ms；宿主首次收到只能作保守发送窗锚，同ticket重复帧不续窗，发送时还核最近State身份。
     */
    constexpr std::uint64_t kTicketReceiveWindowUs = 50000U;
    if (last_freshness_ticket == 0U || last_ticket_received_us == 0U ||
        publish_now < last_ticket_received_us ||
        publish_now - last_ticket_received_us >= kTicketReceiveWindowUs ||
        last_state_boot != record.identity.boot ||
        last_state_session != record.identity.session ||
        last_state_reference != record.identity.reference_epoch ||
        last_state_motion != record.identity.motion_epoch) return;
    if(exported_motion_ready && complete_boundary.boot_epoch==record.identity.boot &&
       complete_boundary.session_id==record.identity.session &&
       complete_boundary.hold_epoch==record.identity.motion_epoch &&
       sequence_newer(wire_producer_sequence(record.generation),complete_boundary.terminal_seq))
      refresh_in_flight_seq=wire_producer_sequence(record.generation);
    Command message;
    message.set_schema_major(2U);
    message.set_schema_minor(dummy_ref::protocol::kSchemaMinor);
    std::uint8_t device[16]{}, client[16]{};
    std::copy(parameters.device_id.begin(), parameters.device_id.end(), device);
    std::copy(parameters.client_instance_id.begin(), parameters.client_instance_id.end(), client);
    message.set_device_id(device);
    message.set_client_instance_id(client);
    message.set_boot_epoch(record.identity.boot);
    message.set_session_id(record.identity.session);
    message.set_reference_epoch(record.identity.reference_epoch);
    message.set_motion_epoch(record.identity.motion_epoch);
    message.set_profile_id(parameters.profile_id);
    message.set_producer_seq(wire_producer_sequence(record.generation));
    message.set_freshness_ticket(last_freshness_ticket);
    message.set_configured_mask(parameters.configured_mask);
    double position[6]{};
    for (std::size_t axis = 0U; axis < record.position.size(); ++axis)
      position[axis] = (parameters.configured_mask & (1U << axis)) != 0U
                           ? record.position[axis]
                           : 0.0;
    message.set_position_rad(position);
    /**
     * @brief publisher返回的是本地接纳层级。Busy/QueueFull暂不新建ACK；Ok才开始等待设备commit，不能直接宣称USB写出或CAN SENT。
     */
    const auto published=publisher.publish(message);
    if(published==dp::comm::Result::kOk || published==dp::comm::Result::kBusy ||
       published==dp::comm::Result::kQueueFull) reserve_complete_prime(record);
    if(published!=dp::comm::Result::kOk && published!=dp::comm::Result::kBusy &&
       published!=dp::comm::Result::kQueueFull &&
       refresh_in_flight_seq==wire_producer_sequence(record.generation)) {
      refresh_in_flight_seq=0U;
      system.request_transport_stop();
    }
    if (message.motion_epoch() == 1U && enabling_publish_log_count++ < 3U)
      diagnostic_detail( "COMMAND_ENABLE_DIAG publish seq=%u result=%u status_core=%u authorized_motion=%u\n",
              message.producer_seq(), static_cast<unsigned>(published), status.core_state(),
              authorized_motion_epoch);
    if (!logged_publish_attempt || (published==dp::comm::Result::kOk && !logged_publish_success)) {
      logged_publish_attempt = true;
      if (published==dp::comm::Result::kOk) logged_publish_success = true;
      diagnostic_detail( "COMMAND_PRIME_DIAG publish seq=%u result=%u boot=%llu session=%llu ref=%u motion=%u\n",
              message.producer_seq(), static_cast<unsigned>(published),
              static_cast<unsigned long long>(message.boot_epoch()),
              static_cast<unsigned long long>(message.session_id()),
              message.reference_epoch(), message.motion_epoch());
    }
    if(published==dp::comm::Result::kBusy || published==dp::comm::Result::kQueueFull) return;
    if(published==dp::comm::Result::kOk) {
      if(safe_cancel_prime && !cancel_prime_target_latched) {cancel_prime_target=prime_source;cancel_prime_target_latched=true;}
      /**
       * @brief 安全prime日志/blob记录原publisher参数的本地成功enqueue观察；encoded_hex不等于transport原始TX或设备原页。
       */
      if(safe_cancel_prime && !cancel_prime_diagnostic_emitted) {
        // 原publisher参数的本地成功enqueue观测，不声明USB TX或CAN SENT。
        std::array<std::uint8_t,wire::DummyJointCommandV2Codec::kEncodedSize> encoded{};
        const auto encoded_result=wire::DummyJointCommandV2Codec::encode(message,encoded.data(),encoded.size());

        diagnostic_detail("STOP_SAFE_PRIME_DIAG version=1 scope=LOCAL_PUBLISH_ARGUMENT result=OK_LOCAL_ENQUEUE cancel_rid=%llu run=%llu producer=%u boot=%llu session=%llu ref=%u motion=%u mgmt=%u profile=%u mask=%u matched_source_state_seq=%u source_produced_us=%llu source_received_us=%llu source_received_clock=HOST_TRANSPORT_STEADY_US source_ticket=%llu send_ticket=%llu send_ticket_first_received_us=%llu publish_now_us=%llu source_is_equivalent_target_frame=1 encoded_size=%zu encoded_status=%s encoded_hex=",
          (unsigned long long)cancel_prime->request_id(),(unsigned long long)cancel_run,message.producer_seq(),
          (unsigned long long)message.boot_epoch(),(unsigned long long)message.session_id(),message.reference_epoch(),message.motion_epoch(),
          prime_source.management,message.profile_id(),message.configured_mask(),prime_source.state_seq,
          (unsigned long long)prime_source.produced_us,(unsigned long long)prime_source.received_us,
          (unsigned long long)prime_source.root.ticket,(unsigned long long)message.freshness_ticket(),
          (unsigned long long)last_ticket_received_us,(unsigned long long)publish_now,encoded.size(),
          encoded_result==wire::CodecResult::kOk?"OK":"UNKNOWN");
        if(worker_diagnostics)worker_diagnostics->blob("STOP_SAFE_PRIME_LOCAL_PUBLISH_ARGUMENT",
           encoded_result==wire::CodecResult::kOk ? encoded.data() : nullptr,encoded.size());

        cancel_prime_diagnostic_emitted=true;
      }

      ++commands;
      command_published_this_boot = true;
      command_sent = true;
      ack_mismatch_diag_count = 0U;
      const auto ack_timeout_us = parameters.group_sent_ack_timeout_us != 0U
          ? parameters.group_sent_ack_timeout_us : parameters.max_state_age_us;
      group_sent_ack.begin(record, transport_steady_now_us(), ack_timeout_us,
                           status.command_reject_count(), status.committed_producer_seq());
    } else {
      ++rejected_commands;
      hold_drain_rejected=true;
      command_stream.discard_in_flight();
      command_sent = false;
    }
  }

  /**
   * @brief 创建一次连接的全部通信资源并在同worker推进管理/命令/状态机。
   * 资源为局部RAII值，所有dp_comm句柄只在本线程使用/析构。主循环先处理取消/管理单槽/Complete屏障/停止优先及有限准入，再向spin传入2ms空闲等待预算收响应（维护与回调执行不受此硬上界约束），核Stop终态与双fresh Idle，最后检查心跳/ACK并转交发布命令和Hold排空快照。没有任何“线程启动即授权电机”的捷径。
   * @return true表示running被外层正常清除后退出；Context/端点/执行器失败或spin错误返回false供外层重连。
   */
  bool run_connected() {
    /**
     * @brief 本次连接使用固定dp_comm静态存储及当前codec最大载荷；Context采用StaticOnly，不因管理/State流量无限申请队列。此局部存储必须活过其Context/端点句柄。
     */
    const dp::comm::StaticStorageConfig storage_config{1U, 2U, 3U, 0U, 8U,
        DP_PROTOCOL_PRODUCT_POSIX_DUMMY_REF_ROS_SDK_RUNTIME_MESSAGE_BYTES, 96U, 1U, 1U, 1U, 1U};
    dp::comm::StaticStorage storage(storage_config);
    const dp::comm::TransportConfig transport{"peer", parameters.endpoint.c_str(), "", "dummy_ros_control"};
    auto initialized = dp::comm::Context::init(storage,
        dp::comm::InitOptions{dp::comm::MemoryMode::kStaticOnly, &transport, nullptr});
    if (!initialized.has_value()) {
      diagnostics.reject_from_context(DiagnosticReason::kManagementRejected,transport_steady_now_us(),"context_init_failed");
      diagnostic_detail( "device transport context init failed result=%u endpoint=%s\\n",
              static_cast<unsigned>(initialized.result()), parameters.endpoint.c_str());
      return false;
    }
    /**
     * @brief 所有权从Expected移入本地Context；后续节点/订阅/发布器/请求句柄在同worker生灭，ROS实时线程只接触值快照和有限channel。
     */
    auto context = std::move(initialized).take_value();
    dp::comm::NodeOptions options{};
    options.product_name = "dummy_robot";
    std::copy(parameters.client_instance_id.begin(), parameters.client_instance_id.end(), options.device_id.bytes);
    options.node_name = "ros_control";
    auto created = context.create_node(options);
    if (!created.has_value()) {
      diagnostics.reject_from_context(DiagnosticReason::kManagementRejected,transport_steady_now_us(),"node_create_failed");
      diagnostic_detail( "device transport node create failed result=%u\\n",
              static_cast<unsigned>(created.result()));
      return false;
    }
    auto node = std::move(created).take_value();
    static constexpr char hex[] = "0123456789abcdef";
    /**
     * @brief 路由以部署设备ID编码成固定小写十六进制，分别创建state_v2/status_v2/command_v2/manage_v2；客户端节点ID则是本次client_instance，不混作目标device。
     */
    std::string route = "/dpsdk/dummy_robot/";
    for (auto byte : parameters.device_id) { route += hex[byte >> 4U]; route += hex[byte & 15U]; }
    route += "/ref_control/";
    auto state_result = node.create_subscription<State>((route + DP_PROTOCOL_TOPIC_DUMMY_STATE_ENDPOINT).c_str(), DUMMY_REF_SELECTED_QOS(DUMMY_STATE, DUMMY_ROS_SDK), receive_state, this);
    auto status_result = node.create_subscription<Status>((route + DP_PROTOCOL_TOPIC_DUMMY_STATUS_ENDPOINT).c_str(), DUMMY_REF_SELECTED_QOS(DUMMY_STATUS, DUMMY_ROS_SDK), receive_status, this);
    auto command_result = node.create_publisher<Command>((route + DP_PROTOCOL_TOPIC_DUMMY_COMMAND_ENDPOINT).c_str(), DUMMY_REF_SELECTED_QOS(DUMMY_COMMAND, DUMMY_ROS_SDK));
    auto management_result=node.create_client<Management>((route+DP_PROTOCOL_TOPIC_DUMMY_MANAGE_ENDPOINT).c_str());
    if (!state_result.has_value() || !status_result.has_value() || !command_result.has_value() || !management_result.has_value()) {
      diagnostics.reject_from_context(DiagnosticReason::kManagementRejected,transport_steady_now_us(),"endpoint_setup_failed");
      diagnostic_detail( "device transport endpoint setup failed state=%u status=%u command=%u management=%u\\n",
              static_cast<unsigned>(state_result.result()), static_cast<unsigned>(status_result.result()),
              static_cast<unsigned>(command_result.result()), static_cast<unsigned>(management_result.result()));
      return false;
    }
    auto state_subscription = std::move(state_result).take_value();
    auto status_subscription = std::move(status_result).take_value();
    /**
     * @brief 只有明确启用raw_evidence_path才订阅attempt_evidence_v2；State/Status归档同样受该开关控制，但collector是否成功另由首错/flush水位判定。
     */
    std::optional<dp::comm::Subscription<Evidence>> evidence_subscription;
    if (!parameters.raw_evidence_path.empty()) {
      auto evidence_result = node.create_subscription<Evidence>(
          (route + DP_PROTOCOL_TOPIC_DUMMY_ATTEMPT_EVIDENCE_ENDPOINT).c_str(), DUMMY_REF_SELECTED_QOS(DUMMY_ATTEMPT_EVIDENCE, DUMMY_ROS_SDK), receive_evidence, this);
      if (!evidence_result.has_value()) {
        diagnostics.reject_from_context(DiagnosticReason::kManagementRejected,transport_steady_now_us(),"evidence_subscription_failed");
        diagnostic_detail( "device transport evidence subscription failed result=%u\n",
                static_cast<unsigned>(evidence_result.result()));
        return false;
      }
      evidence_subscription.emplace(std::move(evidence_result).take_value());
    }
    auto command_publisher = std::move(command_result).take_value();
    auto management_client=std::move(management_result).take_value();
    dp::comm::StaticExecutor executor;
    const auto add_result = executor.add_node(node);
    const auto activate_result = add_result == dp::comm::Result::kOk ? node.activate() : add_result;
    if (add_result != dp::comm::Result::kOk || activate_result != dp::comm::Result::kOk) {
      diagnostics.reject_from_context(DiagnosticReason::kManagementRejected,transport_steady_now_us(),"activate_failed");
      diagnostic_detail( "device transport activate failed add=%u activate=%u\\n",
              static_cast<unsigned>(add_result), static_cast<unsigned>(activate_result));
      return false;
    }
    /**
     * @brief 连接重新建立时清所有故障源锚和cleanup生命周期，旧writer排队记录带旧generation无法发布到新只读清理槽。
     */
    raw_fault_clock={};raw_fault_trusted_boot=0U;raw_fault_trusted_produced=0U;raw_fault_trusted_sequence=0U;
    raw_fault_trusted_anchor_received=0U;raw_fault_trusted_offset=0;raw_fault_trusted_anchor_offset=0;
    management->reset_raw_cleanup_lifetime();
    producer_anchor_us = 0U;
    producer_offset_us = 0;
    clock_anchor_offset_us = 0;
    clock_anchor_received_us = 0U;
    /**
     * @brief 此ready只说明本地Context/路由/执行器已建好并激活节点；bootstrap_ready、当前State、设备会话与许可仍须随后独立建立。
     */
    ready = true;
    management->ready=true;
    /**
     * @brief active_management保持回调context寿命；processed_management防同响应重复归档；management_handle拥有异步RPC等待，reset只结束本地句柄，不保证远端操作已撤回。
     */
    std::shared_ptr<DeviceManagementRequest> active_management;
    std::shared_ptr<DeviceManagementRequest> processed_management;
    std::optional<Management::Request> deferred_complete;
    // 以下RPC均显式correlation/idempotency=0：未SENT Busy只能是短锁拒绝。
    // Retry保留原wire身份；有非零关联键或已SENT句柄时不启用Busy重试。
    /**
     * @brief Complete、普通管理、Stop、Stop Query各自有限准入状态；只重试已知未发且correlation/idempotency为0的Busy/QueueFull，绝不盲重发已发送RPC。
     */
    std::optional<ManagementAdmissionRetry> complete_admission_retry;
    std::optional<Management::Request> admission_retry_request;
    std::optional<ManagementAdmissionRetry> admission_retry_state;
    std::shared_ptr<DeviceManagementRequest> pending_stop;
    std::shared_ptr<DeviceManagementRequest> stop_query;
    std::optional<ManagementAdmissionRetry> stop_admission_retry;
    std::optional<ManagementAdmissionRetry> stop_query_admission_retry;
    Management::Request stop_original;
    // Host-only bounded单事务摘要；local enqueue不冒设备SENT/epoch权威。
    /**
     * @brief 单个owned_cancel只保存本worker确实本地接纳过的Cancel根；run/op/committed/pending标志分别追踪事务证据，不能拿Pending直接授予后继epoch。
     */
    std::optional<Management::Request> owned_cancel;
    std::uint64_t owned_cancel_run=0U, owned_cancel_operation=0U;
    bool owned_cancel_committed=false, owned_cancel_invalid=false, owned_cancel_received_pending=false, stop_wait_cancel=false;
    /**
     * @brief 核对外部排队查询是否专门追问本次owned Cancel。
     * 用于Stop等待Cancel时的唯一管理例外，其他管理请求不能插队延长停机窗口。
     * @param q GetOperation候选请求。
     * @return bool；操作12、queried原request_id及完整expected身份/设备客户端/mask全一致。
     */
    auto cancel_query_matches=[&](const Management::Request &q) {
      if(!owned_cancel) return false;
      const auto &o=*owned_cancel;
      return q.operation()==12U && q.queried_operation_id()==o.request_id() &&
        q.expected_boot_epoch()==o.expected_boot_epoch() && q.expected_session_id()==o.expected_session_id() &&
        q.expected_reference_epoch()==o.expected_reference_epoch() && q.expected_motion_epoch()==o.expected_motion_epoch() &&
        q.expected_management_epoch()==o.expected_management_epoch() && q.expected_profile_id()==o.expected_profile_id() &&
        q.configured_mask()==o.configured_mask() &&
        std::equal(std::begin(q.device_id()),std::end(q.device_id()),o.device_id()) &&
        std::equal(std::begin(q.client_instance_id()),std::end(q.client_instance_id()),o.client_instance_id());
    };
    /**
     * @brief 停止阶段分开跟踪首Received/Pending、终Query COMMITTED、匹配Idle和固定deadline；next_stop_query仅查询退避，不重启总确认计时。
     */
    bool stop_received = false;
    bool stop_terminal = false;
    bool stop_receipt_diagnostic_emitted = false;
    std::uint64_t stop_deadline_us = 0U, next_stop_query_us = 0U;
    std::uint64_t stop_terminal_at_us = 0U, stop_operation_id = 0U;bool stop_candidate_revalidated=false;
    std::optional<dp::comm::RequestHandle<Management>> management_handle;
    /**
     * @brief 本地async_request接纳后保存管理发送根并把请求原字节入collector。
     * D10重建本次释放根和sent_us，Cancel重置64槽安全目标episode并绑定当时run；这些都只声明本地enqueue。raw记录用host发送观察时间，不冒称设备Received。
     * @param pending 仍被shared_ptr持有的完整请求对象。
     */
    auto capture_sent_request = [&](DeviceManagementRequest &pending) {
      pending.connection_generation=management->worker_connection_generation_.load();
      Management::Request sent;
      if(stop_confirmation_pending.load() && wire::DummyManageV2RequestCodec::decode(pending.request.data(),pending.request.size(),&sent)==wire::CodecResult::kOk &&
         sent.operation()==kWireDisableAndRelease && sent.request_id()==stop_original.request_id()) {
        early_release_idle={};early_worker_status={};
        gate_release={sent.request_id(),sent.expected_boot_epoch(),sent.expected_session_id(),sent.expected_reference_epoch(),sent.expected_motion_epoch(),sent.expected_management_epoch(),0U,0U,false,false,false,0U,0U,status.run_id()};
      }
      if(wire::DummyManageV2RequestCodec::decode(pending.request.data(),pending.request.size(),&sent)==wire::CodecResult::kOk &&
         sent.operation()==kWireDisableAndRelease && sent.request_id()==gate_release.request_id) {
        gate_release.sent=true; gate_release.sent_us=transport_steady_now_us();
      }
      Management::Request cancel;
      /**
       * @brief 每次新Cancel本地接纳都建立新的安全prime episode；来源槽/已锁目标/overflow全部清零，禁止沿用上一次Cancel的实测目标。
       */
      if(wire::DummyManageV2RequestCodec::decode(pending.request.data(),pending.request.size(),&cancel)==wire::CodecResult::kOk && cancel.operation()==9U) {
        cancel_prime_source_count=0U;cancel_prime_source_next=0U;cancel_prime_source_overflow=false;
        cancel_prime_target_latched=false;cancel_prime_diagnostic_emitted=false;cancel_prime_target={};cancel_prime_episode_request=cancel.request_id();
        owned_cancel=cancel; owned_cancel_run=status.run_id(); owned_cancel_operation=0U;
        owned_cancel_committed=false;owned_cancel_received_pending=false;
        owned_cancel_invalid=owned_cancel_run==0U || cancel.expected_boot_epoch()!=status.boot_epoch() ||
          cancel.expected_session_id()!=status.session_id() || cancel.expected_reference_epoch()!=status.reference_epoch() ||
          cancel.expected_motion_epoch()!=status.motion_epoch() || cancel.expected_management_epoch()!=status.management_epoch() ||
          cancel.expected_profile_id()!=parameters.profile_id || cancel.configured_mask()!=parameters.configured_mask ||
          !std::equal(parameters.device_id.begin(),parameters.device_id.end(),cancel.device_id()) ||
          !std::equal(parameters.client_instance_id.begin(),parameters.client_instance_id.end(),cancel.client_instance_id());
      }
      if (!parameters.raw_evidence_path.empty() &&
          !raw_evidence.offer_management_request(pending.request.data(), pending.request.size(),
                                                 transport_steady_now_us())) failed = true;
    };
    /**
     * @brief 归档一次完整响应并专门核对owned Cancel因果结果。
     * Cancel原请求或其严格Query必须回显同身份与receipt，Rejected/不合法phase令owned_cancel_invalid；只有phase3成功且epoch恰+1才committed。普通complete在此只归档，授权还由后续accept_management_authority处理。
     * @param pending 可空共享事务；已processed或未complete则无操作。
     */
    auto capture_completed_reply = [&](const std::shared_ptr<DeviceManagementRequest> &pending) {
      if (!pending || pending == processed_management ||
          !pending->complete.load(std::memory_order_acquire)) return;
      Management::Request q; Management::Response r;
      if(wire::DummyManageV2RequestCodec::decode(pending->request.data(),pending->request.size(),&q)==wire::CodecResult::kOk &&
         wire::DummyManageV2ResponseCodec::decode(pending->response.data(),pending->response.size(),&r)==wire::CodecResult::kOk)
        management->observe_owned_management_commit(q,r,pending->connection_generation);
      if(owned_cancel && wire::DummyManageV2RequestCodec::decode(pending->request.data(),pending->request.size(),&q)==wire::CodecResult::kOk &&
         (q.request_id()==owned_cancel->request_id() || cancel_query_matches(q))) {
        if(wire::DummyManageV2ResponseCodec::decode(pending->response.data(),pending->response.size(),&r)!=wire::CodecResult::kOk ||
           r.request_id()!=q.request_id() || r.receipt()!=1U || r.schema_major()!=2U || r.schema_minor()!=dummy_ref::protocol::kSchemaMinor ||
           r.boot_epoch()!=owned_cancel->expected_boot_epoch() || r.session_id()!=owned_cancel->expected_session_id() ||
           r.reference_epoch()!=owned_cancel->expected_reference_epoch() || r.profile_id()!=owned_cancel->expected_profile_id() ||
           !std::equal(parameters.device_id.begin(),parameters.device_id.end(),r.device_id()) ||
           !std::equal(parameters.client_instance_id.begin(),parameters.client_instance_id.end(),r.client_instance_id())) owned_cancel_invalid=true;
        else if(r.transaction_phase()==4U) owned_cancel_invalid=true;
        else if(r.transaction_phase()==3U) {
          const bool valid=r.result_code()==0U && r.operation_id()!=0U && r.run_id()==owned_cancel_run &&
            r.attempt_id()==owned_cancel_run && r.attempt_kind()==1U && r.parent_run_id()==0U &&
            !r.release_committed() && !r.motion_complete() && owned_cancel->expected_motion_epoch()!=UINT32_MAX &&
            owned_cancel->expected_management_epoch()!=UINT32_MAX &&
            r.motion_epoch()==owned_cancel->expected_motion_epoch()+1U && r.management_epoch()==owned_cancel->expected_management_epoch()+1U;
          if(valid) { owned_cancel_committed=true; owned_cancel_operation=r.operation_id(); }
          else owned_cancel_invalid=true;
        } else if(r.result_code()!=23U || (r.transaction_phase()!=1U && r.transaction_phase()!=2U)) owned_cancel_invalid=true;
        else owned_cancel_received_pending=true;
      }
      if (!parameters.raw_evidence_path.empty() &&
          !raw_evidence.offer_management_response(pending->response.data(),
              pending->response.size(), transport_steady_now_us())) failed = true;
      processed_management = pending;
    };
    /**
     * @brief 在停止episode尚待确认时锁存UNCONFIRMED并清本地停止等待资源。
     * 一次exchange保证每episode只累计一次未确认；清早Idle/Query/准入/句柄，不虚构设备已释放，也不启动新确认窗口掩盖超时。
     * @param reason 失败原因C文本，复制到固定诊断。
     */
    auto fail_stop = [&](const char *reason) {
      if (!stop_confirmation_pending.exchange(false)) return;
      early_release_idle={};early_worker_status={};stop_candidate_revalidated=false;if(gate_release.request_id==stop_original.request_id())gate_release={};
      diagnostics.reject_from_context(DiagnosticReason::kStopUnconfirmed,transport_steady_now_us(),reason);
      diagnostic_detail( "device transport UNCONFIRMED_STOP: %s\n", reason);
      ++unconfirmed_stop_requests;
      pending_stop.reset();
      stop_query.reset();
      stop_admission_retry.reset();
      stop_query_admission_retry.reset();
      management_handle.reset();
    };
    /**
     * @brief 连接调度循环：所有管理句柄/回调上下文在这里串行交接；读共享queued只在短mutex内move出，网络RPC不持channel mutex执行。
     */
    while (running.load(std::memory_order_acquire)) {
      // 显式owner停止只转发现有SafetyStop机器，绝不从诊断lane恢复控制授权。
      if(const auto intent=management->take_owner_safety_stop_request(transport_steady_now_us())) {
        if(!stop_latched && !stop_confirmation_pending.load(std::memory_order_acquire) &&
           status.boot_epoch()==intent->boot && status.session_id()==intent->session && status.reference_epoch()==intent->reference &&
           authorized_motion_epoch==intent->motion && authorized_management_epoch==intent->management &&
           last_freshness_ticket==intent->ticket)
          system.request_transport_stop();
      }

      /**
       * @brief 调用方取消等待时清本地RequestHandle；已发送Complete的根摘要保留用于后继Query，尚未发的deferred不具有后继epoch权威。
       */
      if(active_management && active_management->canceled) {
        log_management_handoff(*active_management,"canceled");
        management_handle.reset(); active_management.reset();
        deferred_complete.reset(); complete_admission_retry.reset();
        // 已发送Complete的句柄在Pending→Query时退休，事务身份仍须保留。
        // 未发送的deferred Complete不具备后继epoch授权；Stop/失效/终态另行退役。
        if (!pending_complete.dispatched) pending_complete={};
        admission_retry_request.reset(); admission_retry_state.reset();
      }
      std::shared_ptr<DeviceManagementRequest> queued;
      { std::lock_guard<std::mutex> lock(management->mutex); queued=std::move(management->queued); }
      Management::Request queued_stop_query;
      /**
       * @brief Stop确认期通常拒绝全部新管理，只保留正在等待owned Cancel的严格同根Query例外。拒绝置请求failed供外部观察，不悄悄丢弃。
       */
      const bool allowed_cancel_query=stop_wait_cancel && queued &&
        wire::DummyManageV2RequestCodec::decode(queued->request.data(),queued->request.size(),&queued_stop_query)==wire::CodecResult::kOk &&
        cancel_query_matches(queued_stop_query);
      if (queued && stop_confirmation_pending.load(std::memory_order_acquire) && !allowed_cancel_query)
        { log_management_handoff(*queued,"stop_pending");
          queued->failed.store(true, std::memory_order_release); }
      else if(queued) {
        if(admission_retry_request && active_management && !active_management->canceled)
          active_management->failed.store(true,std::memory_order_release);
        admission_retry_request.reset(); admission_retry_state.reset();
        if(deferred_complete) {
          if(active_management) {
            log_management_handoff(*active_management,"complete_superseded_by_queue");
            active_management->failed=true;
          }
          deferred_complete.reset();
          complete_admission_retry.reset();
          pending_complete={};
        }
        /**
         * @brief 新单槽管理取代旧等待句柄；对旧准入或deferred显式标failed，固定大小交接不形成无界请求堆积。
         */
        management_handle.reset(); active_management=std::move(queued);
        Management::Request decoded;
        if(wire::DummyManageV2RequestCodec::decode(active_management->request.data(),active_management->request.size(),&decoded)!=wire::CodecResult::kOk) {
          log_management_handoff(*active_management,"decode_failed");
          active_management->failed=true;
        }
        else {
          management->observe_request_sequence(decoded.request_seq());
          if (decoded.operation() == kWireDisableAndRelease) {
            early_release_idle={};early_worker_status={};
            gate_release = {decoded.request_id(), decoded.expected_boot_epoch(),
                decoded.expected_session_id(), decoded.expected_reference_epoch(),
                decoded.expected_motion_epoch(), decoded.expected_management_epoch(),
                0U, 0U, false, false};
          } else if (decoded.operation() != kWireGetOperation && decoded.operation() != 0U &&
                     decoded.operation() != 1U) {
            gate_release = {};
          }
          /**
           * @brief Complete必须先匹配当前完整授权tuple；诊断只记录有限次详细现场，失败仍明确failed，不发送错绑定Complete。
           */
          const bool valid_complete=matches_complete_authority(decoded);
          if(decoded.operation()==8U) {
            if(complete_binding_diag_count++<3U)
              diagnostic_detail(
                  "COMPLETE_BINDING_DIAG valid=%d req=%llu boot=%llu/%llu session=%llu/%llu ref=%u/%u motion=%u/%u management=%u/%u status_management=%u state_management=%u profile=%u/%u mask=%u/%u device_match=%d client_match=%d terminal_seq=%u\n",
                  valid_complete,static_cast<unsigned long long>(decoded.request_id()),
                  static_cast<unsigned long long>(decoded.expected_boot_epoch()),
                  static_cast<unsigned long long>(boot),
                  static_cast<unsigned long long>(decoded.expected_session_id()),
                  static_cast<unsigned long long>(status.session_id()),
                  decoded.expected_reference_epoch(),status.reference_epoch(),
                  decoded.expected_motion_epoch(),authorized_motion_epoch,
                  decoded.expected_management_epoch(),authorized_management_epoch,
                  status.management_epoch(),last_state_management_epoch,
                  decoded.expected_profile_id(),parameters.profile_id,
                  decoded.configured_mask(),parameters.configured_mask,
                  std::equal(parameters.device_id.begin(),parameters.device_id.end(),decoded.device_id()),
                  std::equal(parameters.client_instance_id.begin(),parameters.client_instance_id.end(),
                             decoded.client_instance_id()),decoded.terminal_producer_seq());
            if(!valid_complete) {
              log_management_handoff(*active_management,"complete_binding_failed");
              active_management->failed=true; continue;
            }
          }
          /**
           * @brief 非零terminal_seq的Complete先defer：冻结末命令并关闭下一Begin，等待发布队列检查、精确末ACK和全部在途排空，不能先终结设备motion再追送旧采样。
           */
          if(valid_complete && decoded.terminal_producer_seq()!=0U) {
            complete_boundary={boot,status.session_id(),status.reference_epoch(),
              authorized_motion_epoch+1U,decoded.terminal_producer_seq()};
            complete_boundary.prior_run_id=status.run_id();
            complete_boundary_hold_diag_logged=false;
            exported_motion_ready=false;
            management->post_complete_begin_ready.store(false,std::memory_order_release);
            refresh_active_run_id=last_refresh_acked_us=0U;
            refresh_in_flight_seq=0U;
            pending_complete={decoded.request_id(),boot,status.session_id(),
              transport_steady_now_us(),status.reference_epoch(),authorized_motion_epoch,
              authorized_management_epoch,decoded.terminal_producer_seq()};
            deferred_complete=decoded;
          }
          if(deferred_complete) continue;
          /**
           * @brief 此处的1秒是RPC句柄等待时间，不是停止总预算。显式0 correlation/idempotency保证本地Busy可分类为未发的短准入拒绝；有handle后不再走这类重试。
           */
          auto sent = management_client.async_request(
              decoded, management_reply, active_management.get(),
              dp::comm::Duration::from_milliseconds(1000),
              management_rpc_correlation_id(decoded),
              management_rpc_idempotency_key(decoded));
          if(sent.has_value()) {
            capture_sent_request(*active_management);
            if(valid_complete) observe_sent_complete(decoded);
            if (motion_authorizing_operation(decoded.operation())) {
              last_motion_request_id = decoded.request_id();
              last_motion_operation_id = 0U;
            }
            management_handle.emplace(std::move(sent).take_value());
          }
          else {
            if(ManagementAdmissionRetry::retryable(sent.result(),true) &&
               (decoded.operation()!=8U || decoded.terminal_producer_seq()==0U)) {
              admission_retry_request=decoded;
              admission_retry_state.emplace(transport_steady_now_us(),true);
              (void)admission_retry_state->on_result(sent.result(),transport_steady_now_us(),false);
              log_management_handoff(*active_management,"async_request_would_block",sent.result());
            } else {
              log_management_handoff(*active_management,"async_request_failed",sent.result());
              active_management->failed=true;
            }
          }
        }
      }
      const auto stop_kind = system.take_stop_request_kind();
      /**
       * @brief Complete放行需末ACK恰等冻结terminal_seq且身份一致；更晚ACK/更晚在途或超过2秒说明边界失效，结束deferred而不是猜已完整发送。
       */
      if(deferred_complete && pending_complete.queue_drained &&
         stop_kind == GuardedZenbeddedSystem::StopRequestKind::kNone) {
        const auto &request=*deferred_complete;
        const bool same_ack=last_ack_seq==pending_complete.terminal_seq &&
          last_ack_identity.boot==pending_complete.boot_epoch &&
          last_ack_identity.session==pending_complete.session_id &&
          last_ack_identity.reference_epoch==pending_complete.reference_epoch &&
          last_ack_identity.motion_epoch==pending_complete.motion_epoch;
        const bool newer_ack=last_ack_identity.boot==pending_complete.boot_epoch &&
          last_ack_identity.session==pending_complete.session_id &&
          last_ack_identity.motion_epoch==pending_complete.motion_epoch &&
          sequence_newer(last_ack_seq,pending_complete.terminal_seq);
        const auto in_flight=command_stream.in_flight_record();
        const bool newer_in_flight=in_flight &&
          in_flight->identity.boot==pending_complete.boot_epoch &&
          in_flight->identity.session==pending_complete.session_id &&
          in_flight->identity.motion_epoch==pending_complete.motion_epoch &&
          sequence_newer(wire_producer_sequence(in_flight->generation),pending_complete.terminal_seq);
        const auto elapsed=transport_steady_now_us()-pending_complete.started_us;
        if(newer_ack || newer_in_flight || elapsed>2000000U ||
           (complete_admission_retry &&
            complete_admission_retry->expired(transport_steady_now_us()))) {
          log_management_handoff(*active_management,"complete_deferred_failed");
          active_management->failed=true;
          deferred_complete.reset(); pending_complete={};
          complete_admission_retry.reset();
        /**
         * @brief 只有没有任何待commit或pending命令才提交Complete；有限准入重试不改变原末序列/身份，停止请求到来立即优先。
         */
        } else if(same_ack && !group_sent_ack.pending() &&
                  !command_stream.has_in_flight() && !command_stream.has_pending() &&
                  (!complete_admission_retry ||
                   complete_admission_retry->ready(transport_steady_now_us()))) {
          auto sent = management_client.async_request(
              request, management_reply, active_management.get(),
              dp::comm::Duration::from_milliseconds(1000),
              management_rpc_correlation_id(request),
              management_rpc_idempotency_key(request));
          if(sent.has_value()) {
            capture_sent_request(*active_management);
            pending_complete.dispatched=true;
            last_motion_request_id=request.request_id();
            last_motion_operation_id=0U;
            management_handle.emplace(std::move(sent).take_value());
            deferred_complete.reset(); complete_admission_retry.reset();
          } else {
            const auto retry_now=transport_steady_now_us();
            if(ManagementAdmissionRetry::retryable(sent.result(),true) && !complete_admission_retry)
              complete_admission_retry.emplace(retry_now,true);
            if(complete_admission_retry &&
               complete_admission_retry->on_result(sent.result(),retry_now,
                   stop_confirmation_pending.load(std::memory_order_acquire))==
                   ManagementAdmissionRetry::Decision::kRetry) {
              log_management_handoff(*active_management,"async_request_would_block",sent.result());
            } else {
              log_management_handoff(*active_management,"async_request_failed",sent.result());
              active_management->failed=true; pending_complete={};
              deferred_complete.reset(); complete_admission_retry.reset();
            }
          }
        }
      }
      /**
       * @brief 已经严格锁存释放Idle时，框架deactivate无需再造一条D10；Safety或尚未释放的情况仍走正常停止根。
       */
      if (stop_kind == GuardedZenbeddedSystem::StopRequestKind::kDeactivation &&
          released_idle_latched()) continue;
      if (stop_kind != GuardedZenbeddedSystem::StopRequestKind::kNone) {
        /**
         * @brief 新Stop撞到已有确认时标旧episode未确认并失效，不重开原250ms/配置窗口。用重复请求延长期限会掩盖真实停机延迟，故此处分支直接continue。
         */
        if(stop_confirmation_pending.load(std::memory_order_acquire)) {
          fail_stop("new Stop superseded pending confirmation"); stop_wait_cancel=false;
          stop_latched=true; invalidate(); continue; // 不重启原250k窗口。
        }
        std::array<std::uint8_t,wire::DummyManageV2RequestCodec::kEncodedSize> cancel_retry_bytes{};
        const bool keep_exact_cancel_query_retry=admission_retry_request && admission_retry_state && active_management &&
          !active_management->canceled && cancel_query_matches(*admission_retry_request) &&
          wire::DummyManageV2RequestCodec::encode(*admission_retry_request,cancel_retry_bytes.data(),cancel_retry_bytes.size())==wire::CodecResult::kOk &&
          cancel_retry_bytes==active_management->request;
        if(!keep_exact_cancel_query_retry) {admission_retry_request.reset(); admission_retry_state.reset();}
        if(deferred_complete) {
          if(active_management) {
            log_management_handoff(*active_management,"complete_superseded_by_stop");
            active_management->failed=true;
          }
          deferred_complete.reset(); pending_complete={};
          complete_admission_retry.reset();
        }
        stop_latched = true;
        if (stop_confirmation_pending.load(std::memory_order_acquire))
          fail_stop("new Stop superseded pending confirmation");
        stop_confirmation_pending = true;
        stop_idle_observed = false;stop_candidate_revalidated=false;early_release_idle={};early_worker_status={};
        stop_received = false;
        stop_terminal = false;
        stop_terminal_at_us = 0U;
        stop_operation_id = 0U;
        stop_receipt_diagnostic_emitted = false;
        /**
         * @brief 总Stop截止只在本次新停止根建立一次：独立stop_confirmation_timeout优先，否则max_state_age回退，所有Cancel等待/准入/Query/Idle共享此锚。
         */
        const auto stop_timeout_us = parameters.stop_confirmation_timeout_us != 0U
            ? parameters.stop_confirmation_timeout_us : parameters.max_state_age_us;
        stop_deadline_us = saturating_state_deadline(transport_steady_now_us(), stop_timeout_us);
        Management::Request current;
        const bool cancel_inflight=owned_cancel &&
          (last_motion_request_id==owned_cancel->request_id() || (active_management &&
           wire::DummyManageV2RequestCodec::decode(active_management->request.data(),active_management->request.size(),&current)==wire::CodecResult::kOk &&
           (current.operation()==9U || cancel_query_matches(current))));
        /**
         * @brief Stop若发现已发owned Cancel，先在同一总deadline等其当前严格tuple；清普通命令与active，但保留必要Status/State根，避免错误地用已退役epoch直接D10。
         */
        if(cancel_inflight) {
          stop_wait_cancel=true;
          // 原Stop deadline已经开始；只暂停命令，不清掉需接纳终态的原observations/root。
          stop_latched=false; system.invalidate_transport_state(); clear_command_identity();
          group_sent_ack.reset(); command_stream.close_session();
          if(owned_cancel_invalid) fail_stop("pending Cancel identity unavailable");
          continue;
        }
        if (active_management && !active_management->complete.load(std::memory_order_acquire)) {
          log_management_handoff(*active_management,"superseded_by_stop");
          active_management->failed.store(true, std::memory_order_release);
        }
        management_handle.reset();
        /**
         * @brief 先从仍可用授权tuple构造D10，再invalidate控制状态；若先清boot/epoch将失去合法停止根。构造/发送失败显式UNCONFIRMED，不能宣称已失能。
         */
        active_management = make_stop_request();
        if (!active_management) {
          ++stop_request_failures;
          fail_stop("no authorized stop tuple");
        } else {
          Management::Request decoded;
          if (wire::DummyManageV2RequestCodec::decode(active_management->request.data(),
                active_management->request.size(), &decoded) != wire::CodecResult::kOk) {
            active_management->failed = true;
            ++stop_request_failures;
            fail_stop("Stop request encode/decode failed");
          } else {
            stop_original=decoded;
            auto sent = management_client.async_request(
                decoded, management_reply, active_management.get(),
                dp::comm::Duration::from_milliseconds(1000),
                management_rpc_correlation_id(decoded),
                management_rpc_idempotency_key(decoded));
            if (sent.has_value()) {
              capture_sent_request(*active_management);
              management_handle.emplace(std::move(sent).take_value());
              ++stop_requests_sent;
              pending_stop = active_management;
              stop_original = decoded;
            } else {
              if(ManagementAdmissionRetry::retryable(sent.result(),true)) {
                pending_stop=active_management;
                stop_original=decoded;
                stop_admission_retry.emplace(transport_steady_now_us(),true);
                (void)stop_admission_retry->on_result(sent.result(),transport_steady_now_us(),false);
                log_management_handoff(*active_management,"stop_would_block",sent.result());
              } else {
                log_management_handoff(*active_management,"async_request_failed",sent.result());
                active_management->failed = true;
                ++stop_request_failures;
                fail_stop("Stop request dispatch failed");
              }
            }
          }
        }
        invalidate();
      }
      /**
       * @brief Cancel→Disable只可由已COMMITTED取消的双fresh Hold，或严格已接纳Stopping/EnablingHold当前双fresh tuple过渡；Pending标记本身绝不授予新epoch。原Stop总deadline始终运行。
       */
      if(stop_wait_cancel) {
        capture_completed_reply(active_management);
        const auto now=transport_steady_now_us();
        if(!stop_confirmation_pending.load(std::memory_order_acquire) || owned_cancel_invalid || now>stop_deadline_us ||
           raw_evidence.failure()!=RawEvidenceCollector::Failure::kNone) {
          stop_latched=true; fail_stop("Cancel-to-Disable authority unavailable/expired"); stop_wait_cancel=false;
        } else {
          const auto &o=*owned_cancel;
          const bool fresh=last_status_received_us!=0U && last_received_us!=0U && now>=last_status_received_us && now>=last_received_us &&
            now-last_status_received_us<=parameters.max_state_age_us && now-last_received_us<=parameters.max_state_age_us;
          /**
           * @brief 取消已确认路径要求同run/根attempt/无错误的final7 Hold，最后State与Status全部epoch一致；位置目标不来自期望动画。
           */
          const bool same=owned_cancel_committed && fresh && observed_hold.received_us==last_received_us && observed_hold.received_us!=0U &&
            observed_hold.boot_epoch==o.expected_boot_epoch() && observed_hold.session_id==o.expected_session_id() &&
            observed_hold.reference_epoch==o.expected_reference_epoch() && observed_hold.motion_epoch==o.expected_motion_epoch()+1U &&
            status.boot_epoch()==o.expected_boot_epoch() && status.session_id()==o.expected_session_id() &&
            status.reference_epoch()==o.expected_reference_epoch() && status.run_id()==owned_cancel_run && status.core_state()==4U &&
            status.final_state()==7U && status.attempt_kind()==1U && status.parent_run_id()==0U && !status.evidence_lost() &&
            !status.release_committed() &&
            status.motion_epoch()==o.expected_motion_epoch()+1U && status.management_epoch()==o.expected_management_epoch()+1U &&
            last_state_boot==status.boot_epoch() && last_state_session==status.session_id() && last_state_reference==status.reference_epoch() &&
            last_state_motion==status.motion_epoch() && last_state_management_epoch==status.management_epoch();
          State accepted_stop;
          bool current_stopping=false;
          /**
           * @brief 尚Pending时额外读取已经过正常State校验的channel原字节，严格核当前sequence及双fresh同tuple后才允许沿原D10转换，避免把任意过渡消息升为授权。
           */
          if(owned_cancel_received_pending && !owned_cancel_committed && fresh &&
             o.expected_motion_epoch()!=UINT32_MAX && o.expected_management_epoch()!=UINT32_MAX &&
             (status.core_state()==7U || status.core_state()==kWireEnablingHold) &&
             status.boot_epoch()==o.expected_boot_epoch() && status.session_id()==o.expected_session_id() &&
             status.reference_epoch()==o.expected_reference_epoch() && status.profile_id()==o.expected_profile_id() &&
             status.configured_mask()==o.configured_mask() && status.run_id()==owned_cancel_run &&
             status.attempt_id()==owned_cancel_run && status.attempt_kind()==1U && status.parent_run_id()==0U &&
             status.final_state()==0U && !status.evidence_lost() && !status.release_committed() && !status.motion_complete() &&
             status.first_failure_reason()==0U && status.failure_source()==0U &&
             status.motion_epoch()==o.expected_motion_epoch()+1U && status.management_epoch()==o.expected_management_epoch()+1U &&
             authorized_motion_epoch==status.motion_epoch() && authorized_management_epoch==status.management_epoch() &&
             last_state_boot==status.boot_epoch() && last_state_session==status.session_id() &&
             last_state_reference==status.reference_epoch() && last_state_motion==status.motion_epoch() &&
             last_state_management_epoch==status.management_epoch() &&
             std::equal(parameters.device_id.begin(),parameters.device_id.end(),o.device_id()) &&
             std::equal(parameters.client_instance_id.begin(),parameters.client_instance_id.end(),o.client_instance_id())) {
            {
              std::lock_guard<std::mutex> lock(management->mutex);
              current_stopping=management->observation.state_received_us==last_received_us &&
                wire::DummyJointStateV2Codec::decode(management->observation.state.data(),management->observation.state.size(),&accepted_stop)==wire::CodecResult::kOk;
            }
            current_stopping=current_stopping && (accepted_stop.core_state()==7U || accepted_stop.core_state()==kWireEnablingHold) &&
              accepted_stop.state_seq()==last_sequence && accepted_stop.first_failure_reason()==0U &&
              accepted_stop.boot_epoch()==status.boot_epoch() && accepted_stop.session_id()==status.session_id() &&
              accepted_stop.reference_epoch()==status.reference_epoch() && accepted_stop.motion_epoch()==status.motion_epoch() &&
              accepted_stop.management_epoch()==status.management_epoch() && accepted_stop.profile_id()==status.profile_id() &&
              accepted_stop.configured_mask()==status.configured_mask() && accepted_stop.position_valid_mask()==parameters.configured_mask;
          }
          /**
           * @brief 复用当前已验证epoch生成D10，并停止旧Cancel查询重试；不会重置Stop deadline或把Cancel Pending直接当成功。
           */
          if(same || current_stopping) {
            // Pending不授epoch；仅已接纳的双fresh当前严格tuple可走原D10。
            stop_latched=true; management_handle.reset();
            if(current_stopping) {
              if(active_management)active_management->canceled.store(true,std::memory_order_release);
              admission_retry_request.reset();admission_retry_state.reset();
            }
            active_management=make_stop_request();
            if(!active_management || wire::DummyManageV2RequestCodec::decode(active_management->request.data(),active_management->request.size(),&stop_original)!=wire::CodecResult::kOk)
              fail_stop("Cancel-to-Disable current tuple unavailable");
            else {
              auto sent = management_client.async_request(
                  stop_original, management_reply, active_management.get(),
                  dp::comm::Duration::from_milliseconds(1000),
                  management_rpc_correlation_id(stop_original),
                  management_rpc_idempotency_key(stop_original));
              pending_stop=active_management;
              if(sent.has_value()) {capture_sent_request(*active_management);management_handle.emplace(std::move(sent).take_value());++stop_requests_sent;}
              else if(ManagementAdmissionRetry::retryable(sent.result(),true)) {
                stop_admission_retry.emplace(now,true);
                (void)stop_admission_retry->on_result(sent.result(),now,false);
                log_management_handoff(*active_management,"stop_would_block",sent.result());
              } else fail_stop("Cancel-to-Disable dispatch failed");
            }
            stop_wait_cancel=false;
          }
        }
      }
      /**
       * @brief 本地Stop准入既受100ms重试上限，也受整个Stop deadline约束；哪个先到就失败，不因1秒RPC默认值而额外等待。
       */
      if(stop_admission_retry &&
         (stop_admission_retry->expired(transport_steady_now_us()) ||
          transport_steady_now_us()>stop_deadline_us)) {
        ++stop_request_failures;
        fail_stop("Stop request admission deadline expired");
      }
      if(stop_confirmation_pending.load(std::memory_order_acquire) && pending_stop &&
         stop_admission_retry && stop_admission_retry->ready(transport_steady_now_us())) {
        auto sent = management_client.async_request(
            stop_original, management_reply, pending_stop.get(),
            dp::comm::Duration::from_milliseconds(1000),
            management_rpc_correlation_id(stop_original),
            management_rpc_idempotency_key(stop_original));
        if(sent.has_value()) {
          capture_sent_request(*pending_stop);
          management_handle.emplace(std::move(sent).take_value());
          ++stop_requests_sent;
          stop_admission_retry.reset();
        } else if(stop_admission_retry->on_result(sent.result(),transport_steady_now_us(),false)==
                  ManagementAdmissionRetry::Decision::kRetry) {
          log_management_handoff(*pending_stop,"stop_would_block",sent.result());
        } else {
          log_management_handoff(*pending_stop,"async_request_failed",sent.result());
          ++stop_request_failures;
          fail_stop("Stop request dispatch failed");
        }
      }
      /**
       * @brief Query同样只重试未发送准入：序号/原D10绑定不变，超过原deadline即UNCONFIRMED。
       */
      if(stop_query_admission_retry &&
         (stop_query_admission_retry->expired(transport_steady_now_us()) ||
          transport_steady_now_us()>stop_deadline_us))
        fail_stop("Stop QueryResult admission deadline expired");
      if(stop_confirmation_pending.load(std::memory_order_acquire) && stop_query &&
         stop_query_admission_retry &&
         stop_query_admission_retry->ready(transport_steady_now_us())) {
        Management::Request query;
        if(wire::DummyManageV2RequestCodec::decode(stop_query->request.data(),
              stop_query->request.size(),&query)!=wire::CodecResult::kOk) {
          fail_stop("Stop QueryResult decode failed");
        } else {
          auto sent = management_client.async_request(
              query, management_reply, stop_query.get(),
              dp::comm::Duration::from_milliseconds(1000),
              management_rpc_correlation_id(query),
              management_rpc_idempotency_key(query));
          if(sent.has_value()) {
            capture_sent_request(*stop_query);
            active_management=stop_query;
            management_handle.emplace(std::move(sent).take_value());
            stop_query_admission_retry.reset();
          } else if(stop_query_admission_retry->on_result(sent.result(),transport_steady_now_us(),false)==
                    ManagementAdmissionRetry::Decision::kRetry) {
            log_management_handoff(*stop_query,"stop_query_would_block",sent.result());
          } else {
            log_management_handoff(*stop_query,"async_request_failed",sent.result());
            fail_stop("Stop QueryResult dispatch failed");
          }
        }
      }
      /**
       * @brief 普通管理准入超100ms后明确failed并清请求；即使稍后网络恢复也不自动回放旧身份请求。
       */
      if(admission_retry_state && admission_retry_state->expired(transport_steady_now_us())) {
        if(active_management) active_management->failed.store(true,std::memory_order_release);
        admission_retry_request.reset(); admission_retry_state.reset();
      }
      if(admission_retry_request && admission_retry_state && active_management &&
         !active_management->canceled &&
         admission_retry_state->ready(transport_steady_now_us()) &&
         (!stop_confirmation_pending.load(std::memory_order_acquire) ||
          (stop_wait_cancel && cancel_query_matches(*admission_retry_request)))) {
        const auto &request=*admission_retry_request;
        if(request.operation()==8U && !matches_complete_authority(request)) {
          log_management_handoff(*active_management,"complete_retry_binding_failed");
          active_management->failed=true;
          admission_retry_request.reset(); admission_retry_state.reset();
          continue;
        }
        auto sent = management_client.async_request(
            request, management_reply, active_management.get(),
            dp::comm::Duration::from_milliseconds(1000),
            management_rpc_correlation_id(request),
            management_rpc_idempotency_key(request));
        if(sent.has_value()) {
          capture_sent_request(*active_management);
          if(request.operation()==8U) observe_sent_complete(request);
          if(motion_authorizing_operation(request.operation())) {
            last_motion_request_id=request.request_id();
            last_motion_operation_id=0U;
          }
          management_handle.emplace(std::move(sent).take_value());
          admission_retry_request.reset(); admission_retry_state.reset();
        } else if(admission_retry_state->on_result(sent.result(),transport_steady_now_us(),false)==
                  ManagementAdmissionRetry::Decision::kRetry) {
          log_management_handoff(*active_management,"async_request_would_block",sent.result());
        } else {
          log_management_handoff(*active_management,"async_request_failed",sent.result());
          active_management->failed.store(true,std::memory_order_release);
          admission_retry_request.reset(); admission_retry_state.reset();
        }
      }
      /**
       * @brief 为本worker的spin传入2ms空闲等待预算；维护与event_budget内回调执行不保证2ms完成。以下处理刚release发布的complete字节，Context和RequestHandle始终在同线程，不让RT线程等待网络。
       */
      const auto spun=executor.spin_some(dp::comm::Duration::from_milliseconds(2U));
      /**
       * @brief D10首次响应必须精确为Received/Pending(result23,phase1,operation_id0)并通过根身份；它只准许继续查结果，不等于设备已执行释放。
       */
      if (pending_stop && !stop_received && (pending_stop->complete.load(std::memory_order_acquire) ||
                                             pending_stop->failed.load(std::memory_order_acquire))) {
        capture_completed_reply(pending_stop);
        Management::Response response;
        const bool decoded = pending_stop->complete.load(std::memory_order_acquire) &&
            wire::DummyManageV2ResponseCodec::decode(pending_stop->response.data(),
                pending_stop->response.size(), &response) == wire::CodecResult::kOk;
        const bool valid_pending = decoded &&
            stop_reply_matches(stop_original, stop_original, response, true) &&
            response.result_code() == kWirePendingResult && response.transaction_phase() == 1U &&
            response.operation_id() == 0U;
        if (!valid_pending) {
          diagnostics.reject(DiagnosticReason::kManagementRejected,
            {response.boot_epoch(),response.session_id(),response.reference_epoch(),response.motion_epoch(),0U,response.management_epoch(),decoded},
            request_diagnostic_identity(stop_original),transport_steady_now_us(),"stop_receipt_mismatch");
          if (!stop_receipt_diagnostic_emitted) {
            stop_receipt_diagnostic_emitted = true;
            diagnostic_detail( "device transport Stop receipt mismatch: decoded=%u req_id=%llu req_boot=%llu req_session=%llu req_ref=%u req_motion=%u req_management=%u resp_id=%llu resp_boot=%llu resp_session=%llu resp_ref=%u resp_motion=%u resp_management=%u resp_opid=%llu resp_phase=%u resp_result=%u\n",
                    decoded ? 1U : 0U,
                    static_cast<unsigned long long>(stop_original.request_id()),
                    static_cast<unsigned long long>(stop_original.expected_boot_epoch()),
                    static_cast<unsigned long long>(stop_original.expected_session_id()),
                    stop_original.expected_reference_epoch(), stop_original.expected_motion_epoch(),
                    stop_original.expected_management_epoch(),
                    static_cast<unsigned long long>(response.request_id()),
                    static_cast<unsigned long long>(response.boot_epoch()),
                    static_cast<unsigned long long>(response.session_id()),
                    response.reference_epoch(), response.motion_epoch(), response.management_epoch(),
                    static_cast<unsigned long long>(response.operation_id()),
                    response.transaction_phase(), response.result_code());
          }
          fail_stop("invalid Stop Received/Pending receipt");
        } else {
          stop_received = true;
          next_stop_query_us = transport_steady_now_us();
          management_handle.reset();
        }
      }
      /**
       * @brief GetOperation响应按阶段分流：phase2 Pending以一个控制周期退避；phase3成功还需非零op和session/reference归零，其他组合一律失败。
       */
      if (stop_query && (stop_query->complete.load(std::memory_order_acquire) ||
                         stop_query->failed.load(std::memory_order_acquire))) {
        capture_completed_reply(stop_query);
        Management::Request query;
        Management::Response response;
        if (!stop_query->complete.load(std::memory_order_acquire) ||
            wire::DummyManageV2RequestCodec::decode(stop_query->request.data(),
                stop_query->request.size(), &query) != wire::CodecResult::kOk ||
            wire::DummyManageV2ResponseCodec::decode(stop_query->response.data(),
                stop_query->response.size(), &response) != wire::CodecResult::kOk ||
            !stop_reply_matches(stop_original, query, response, true)) {
          fail_stop("invalid Stop QueryResult response");
        } else if (response.result_code() == kWirePendingResult &&
                   response.transaction_phase() == 2U && response.operation_id() == 0U) {
          stop_query.reset();
          management_handle.reset();
          next_stop_query_us = saturating_state_deadline(transport_steady_now_us(),
              parameters.control_period_us);
        } else if (response.result_code() == 0U &&
                   response.transaction_phase() == 3U &&
                   response.operation_id() != 0U &&
                   response.release_committed() &&
                   response.session_id() == 0U &&
                   (response.reference_epoch() == 0U ||
                    (parameters.manual_reference_mode &&
                     response.reference_epoch() ==
                         stop_original.expected_reference_epoch()))) {
          // 同一次Worker原D10的COMMITTED证人仅用于后续只读退休，不改Stop原deadline。
          management->publish_worker_release_commit({stop_original.request_id(),response.operation_id(),
            response.run_id(),response.boot_epoch(),stop_original.expected_session_id(),0U,
            stop_original.expected_reference_epoch(),response.motion_epoch(),response.management_epoch()});
          authorized_motion_epoch = response.motion_epoch();
          authorized_management_epoch = response.management_epoch();
          stop_idle_observed = false;
          stop_terminal = true;
          stop_terminal_at_us = transport_steady_now_us();
          stop_operation_id = response.operation_id();
          /**
           * @brief COMMITTED后才能取早到Status/State重放；两个缓存都须绑定同request、发送后到达、在原deadline和年龄内，且op/run/epoch与终响应精确相同。
           */
          const auto cached_status=early_worker_status;
          const auto cached_state=early_release_idle;
          early_worker_status={};early_release_idle={};
          Status staged{};
          const auto replay_now=transport_steady_now_us();
          /**
           * @brief 启用原页采集时额外要求对应终链已经writer写出/flush且与缓存Status链相同；只有原内存终页验过而未写出不足以用缓存完成停机证明。
           */
          const bool valid_cache=replay_now<=stop_deadline_us && cached_status.request_id==stop_original.request_id() && cached_state.request_id==stop_original.request_id() &&
            cached_status.received_us>=gate_release.sent_us && cached_state.received_us>=gate_release.sent_us &&
            replay_now>=cached_status.received_us && replay_now>=cached_state.received_us &&
            replay_now-cached_status.received_us<=parameters.max_state_age_us && replay_now-cached_state.received_us<=parameters.max_state_age_us &&
            wire::DummyDeviceStatusV2Codec::decode(cached_status.raw.data(),cached_status.raw.size(),&staged)==wire::CodecResult::kOk &&
            worker_idle_status_candidate(staged) && staged.operation_id()==response.operation_id() && staged.run_id()==response.run_id() && response.run_id()!=0U &&
            staged.motion_epoch()==response.motion_epoch() && staged.management_epoch()==response.management_epoch() &&
            cached_state.state.operation_id()==response.operation_id() && cached_state.state.motion_epoch()==response.motion_epoch() &&
            cached_state.state.management_epoch()==response.management_epoch() &&
            (parameters.raw_evidence_path.empty() || (last_terminal_chain_run==response.run_id() && raw_evidence.terminal_written_run_id()==response.run_id() &&
              std::equal(last_terminal_chain.begin(),last_terminal_chain.end(),staged.evidence_chain_sha256()) && raw_evidence.failure()==RawEvidenceCollector::Failure::kNone));
          gate_release.committed=true;gate_release.motion_epoch=response.motion_epoch();gate_release.management_epoch=response.management_epoch();gate_release.operation_id=response.operation_id();gate_release.run_id=response.run_id();
          /**
           * @brief 重放调用仍用原received_us而非当前时刻；处理后核真正接纳水位等于缓存水位，明确记录commit处理和revalidated宿主时间。
           */
          if(valid_cache) {
            process_decoded_status(staged,cached_status.received_us,true);
            process_decoded_state(cached_state.state,cached_state.received_us,true);
            stop_candidate_revalidated=last_status_received_us==cached_status.received_us && last_received_us==cached_state.received_us && last_observation_raw_idle;
            if(stop_candidate_revalidated && transport_steady_now_us()<=stop_deadline_us) {
              const auto revalidated_us=transport_steady_now_us();
              diagnostics.record_release_idle({stop_original.request_id(),response.operation_id(),response.run_id(),
                gate_release.sent_us,cached_state.received_us,cached_status.received_us,stop_terminal_at_us,revalidated_us});
              diagnostic_detail("OWNED_RELEASE_IDLE_REVALIDATION_DIAG version=1 clock=HOST_TRANSPORT_STEADY_US request_id=%llu operation_id=%llu run_id=%llu sent_us=%llu state_received_us=%llu status_received_us=%llu commit_processed_us=%llu revalidated_us=%llu\n",(unsigned long long)stop_original.request_id(),(unsigned long long)response.operation_id(),(unsigned long long)response.run_id(),(unsigned long long)gate_release.sent_us,(unsigned long long)cached_state.received_us,(unsigned long long)cached_status.received_us,(unsigned long long)stop_terminal_at_us,(unsigned long long)revalidated_us);
            }

          }

          stop_query.reset();
          management_handle.reset();
        } else
          fail_stop("Stop QueryResult failed or malformed");
      }
      const auto stop_now = transport_steady_now_us();
      /**
       * @brief 最终停止成功为合取：Query已COMMITTED+非零operation、Status/State双fresh原始Idle、同motion/management、无owner/reference/permit且release_committed，并仍在固定deadline内。缓存例外也需已完成同事务重验证。
       */
      const bool fresh_released_idle =
          stop_terminal && stop_idle_observed && stop_terminal_at_us != 0U &&
          stop_operation_id != 0U &&
          status.boot_epoch() == stop_original.expected_boot_epoch() &&
          released_reference_shape(status.core_state(),
                                   status.reference_epoch()) &&
          status.session_id() == 0U && status.release_committed() &&
          !status.real_motion_permitted() &&
          status.permit_remaining_us() == 0U &&
          status.motion_epoch() == authorized_motion_epoch &&
          status.management_epoch() == authorized_management_epoch &&
          status.operation_id() == stop_operation_id &&
          status.transaction_phase() == 3U &&
          last_state_boot == status.boot_epoch() && last_state_session == 0U &&
          last_state_reference == status.reference_epoch() &&
          last_state_motion == status.motion_epoch() &&
          last_state_management_epoch == status.management_epoch() &&
          last_observation_raw_idle &&
          (stop_candidate_revalidated &&
                   gate_release.request_id == stop_original.request_id() &&
                   gate_release.committed &&
                   gate_release.operation_id == stop_operation_id
               ? last_received_us >= gate_release.sent_us &&
                     last_status_received_us >= gate_release.sent_us
               : last_received_us >= stop_terminal_at_us &&
                     last_status_received_us >= stop_terminal_at_us) &&
          stop_now >= last_received_us &&
          stop_now - last_received_us <= parameters.max_state_age_us &&
          stop_now >= last_status_received_us &&
          stop_now - last_status_received_us <= parameters.max_state_age_us;
      if (stop_confirmation_pending.load(std::memory_order_acquire) &&
          fresh_released_idle && stop_now<=stop_deadline_us) {
        /**
         * @brief 只有上方完整合取成立才累计confirmed并清pending；诊断记录原始总预算及宿主时间，不把它说成设备IRQ/机械停止延迟。
         */
        ++stop_requests_confirmed;
        stop_confirmation_pending = false;
        pending_stop.reset();
        // 仅复用原Worker的两个Host时间锚；不改变Stop顺序/预算，也不冒Device/IRQ时延。
        const auto diagnostic_timeout_us = parameters.stop_confirmation_timeout_us != 0U
            ? parameters.stop_confirmation_timeout_us : parameters.max_state_age_us;
        const bool diagnostic_start_known = diagnostic_timeout_us != 0U &&
            stop_deadline_us != UINT64_MAX && stop_deadline_us >= diagnostic_timeout_us;
        diagnostics.record_stop_policy({diagnostic_start_known,
          diagnostic_start_known ? stop_deadline_us-diagnostic_timeout_us : 0U,
          stop_now,stop_deadline_us,diagnostic_timeout_us,stop_original.request_id(),stop_operation_id,
          request_diagnostic_identity(stop_original),stop_original.expected_profile_id(),stop_original.configured_mask(),
          status.run_id(),status.motion_epoch(),status.management_epoch()});
        diagnostic_detail(
            "STOP_POLICY_DIAG version=1 clock=HOST_TRANSPORT_STEADY_US start_known=%u "
            "started_us=%llu confirmed_us=%llu deadline_us=%llu timeout_us=%llu "
            "request_id=%llu operation_id=%llu boot=%llu session=%llu ref=%u motion=%u mgmt=%u profile=%u mask=%u "
            "status_run=%llu status_motion=%u status_mgmt=%u\n",
            static_cast<unsigned>(diagnostic_start_known),
            static_cast<unsigned long long>(diagnostic_start_known ? stop_deadline_us-diagnostic_timeout_us : 0U),
            static_cast<unsigned long long>(stop_now), static_cast<unsigned long long>(stop_deadline_us),
            static_cast<unsigned long long>(diagnostic_timeout_us),
            static_cast<unsigned long long>(stop_original.request_id()),
            static_cast<unsigned long long>(stop_operation_id),
            static_cast<unsigned long long>(stop_original.expected_boot_epoch()),
            static_cast<unsigned long long>(stop_original.expected_session_id()),
            static_cast<unsigned>(stop_original.expected_reference_epoch()),
            static_cast<unsigned>(stop_original.expected_motion_epoch()),
            static_cast<unsigned>(stop_original.expected_management_epoch()),
            static_cast<unsigned>(stop_original.expected_profile_id()), static_cast<unsigned>(stop_original.configured_mask()),
            static_cast<unsigned long long>(status.run_id()), static_cast<unsigned>(status.motion_epoch()),
            static_cast<unsigned>(status.management_epoch()));
      } else if (stop_confirmation_pending.load(std::memory_order_acquire) &&
                 stop_now > stop_deadline_us) fail_stop("terminal QueryResult/Idle deadline expired");
      /**
       * @brief 仍Pending时构造新的GetOperation查询原request_id，而不重发D10；next_stop_query只影响查询频率，不延长确认截止。
       */
      else if (stop_confirmation_pending.load(std::memory_order_acquire) && stop_received &&
               !stop_terminal && !stop_query && stop_now >= next_stop_query_us) {
        stop_query = make_stop_query(stop_original);
        if (!stop_query) fail_stop("Stop QueryResult construction failed");
        else {
          Management::Request query;
          if (wire::DummyManageV2RequestCodec::decode(stop_query->request.data(),
                  stop_query->request.size(), &query) != wire::CodecResult::kOk)
            fail_stop("Stop QueryResult decode failed");
          else {
            auto sent = management_client.async_request(
                query, management_reply, stop_query.get(),
                dp::comm::Duration::from_milliseconds(1000),
                management_rpc_correlation_id(query),
                management_rpc_idempotency_key(query));
            if (sent.has_value()) {
              capture_sent_request(*stop_query);
              active_management = stop_query;
              management_handle.emplace(std::move(sent).take_value());
            } else {
              if(ManagementAdmissionRetry::retryable(sent.result(),true)) {
                stop_query_admission_retry.emplace(transport_steady_now_us(),true);
                (void)stop_query_admission_retry->on_result(sent.result(),transport_steady_now_us(),false);
                log_management_handoff(*stop_query,"stop_query_would_block",sent.result());
              } else {
                log_management_handoff(*stop_query,"async_request_failed",sent.result());
                fail_stop("Stop QueryResult dispatch failed");
              }
            }
          }
        }
      }
      /**
       * @brief 普通管理完整响应归档后分别推进释放根和epoch授权；complete标志仅保证响应字节完整，操作是否成功由两个专用解释器判断。
       */
      if (active_management && active_management != processed_management &&
          active_management->complete.load(std::memory_order_acquire)) {
        capture_completed_reply(active_management);
        observe_gate_release_reply(*active_management);
        accept_management_authority(*active_management);
      }
      if(spun!=dp::comm::Result::kOk && spun!=dp::comm::Result::kBusy) {
        diagnostic_detail("device transport spin result=%u\n",static_cast<unsigned>(spun));
        return false;
      }
      if (!gate_release.idle_confirmed &&
          confirmed_gate_released_idle(transport_steady_now_us())) {
        gate_release.released_reference_epoch = status.reference_epoch();
        gate_release.idle_confirmed = true;
      }
      dp::comm::LinkEvent event;
      /**
       * @brief 链路不Ready立即invalidate旧控制和命令意图；外层连接循环可重建Context，但不会沿用旧会话目标自动续跑。
       */
      while (context.take_link_event(&event)) {
        if (event.state != dp::comm::LinkState::kReady) invalidate();
      }
      const auto now = transport_steady_now_us();
      const bool released_idle = released_idle_latched();
      const auto stop_causes = classify_transport_stop_causes(
          group_sent_ack, now, last_received_us, parameters.max_state_age_us);
      const auto heartbeat_action = parameters.heartbeat_stop_budget_us == 0U
          ? heartbeat.poll(now)
          : heartbeat.poll(now, parameters.heartbeat_stop_budget_us);
      /**
       * @brief 只有从未授权/发命令的只读raw Idle可在心跳间隙后重新观察；已控制连接必须按普通心跳/状态失鲜Stop，不能借只读捷径掩盖失联。
       */
      const bool reobserve_raw_idle = !released_idle && recoverable_raw_idle_gap() &&
          (heartbeat_action == HeartbeatAction::kStop || stop_causes.state_stale);
      if (reobserve_raw_idle) {
        ++raw_idle_reobservations;
        invalidate();
      }
      /**
       * @brief 心跳Warn只记一次直至下一接纳State；独立ACK超时与状态年龄分别分类，不能把ACK尚未到期时的状态失鲜说成CAN SENT已失败。
       */
      if (!released_idle && heartbeat_action == HeartbeatAction::kWarn && !heartbeat_warned) {
        ++heartbeat_warnings;
        heartbeat_warned = true;
      }
      if (!released_idle && !reobserve_raw_idle && heartbeat_action == HeartbeatAction::kStop &&
          !heartbeat_stopped && !stop_latched) {
        diagnostic_detail( "device transport state heartbeat stop: gap_us=%llu period_us=%llu callbacks=%llu decoded=%llu rejected=%llu decode=%llu identity=%llu authority=%llu sequence=%llu clock=%llu age=%llu ingest=%llu stop_latched=%llu ack_misses=%llu last_callback_seq=%u last_callback_produced_us=%llu last_callback_received_us=%llu last_accepted_seq=%u last_accepted_produced_us=%llu last_accepted_received_us=%llu\n",
                static_cast<unsigned long long>(now - last_received_us),
                static_cast<unsigned long long>(parameters.state_publish_period_us),
                static_cast<unsigned long long>(state_callbacks.load()),
                static_cast<unsigned long long>(states.load()),
                static_cast<unsigned long long>(rejected_states.load()),
                static_cast<unsigned long long>(state_decode_rejections.load()),
                static_cast<unsigned long long>(state_identity_rejections.load()),
                static_cast<unsigned long long>(state_authority_rejections.load()),
                static_cast<unsigned long long>(state_sequence_rejections.load()),
                static_cast<unsigned long long>(clock_skew_rejections.load()),
                static_cast<unsigned long long>(stale_payload_rejections.load()),
                static_cast<unsigned long long>(state_ingest_rejections.load()),
                static_cast<unsigned long long>(stop_latched_state_rejections.load()),
                static_cast<unsigned long long>(group_sent_ack_misses.load()),
                last_callback_sequence,
                static_cast<unsigned long long>(last_callback_produced_us),
                static_cast<unsigned long long>(last_callback_received_us), last_sequence,
                static_cast<unsigned long long>(last_produced_us),
                static_cast<unsigned long long>(last_received_us));
        ++state_heartbeat_gap_stops;
        last_state_gap_us = now - last_received_us;
        ++heartbeat_stops;
        heartbeat_stopped = true;
        stop_latched = true;
        system.request_transport_stop();
      }
      /**
       * @brief ACK超期锁存Stop并清等待；局部publish早已成功也不能因此跳过设备commit确认。
       */
      if (!released_idle && !reobserve_raw_idle && stop_causes.ack_timed_out) {
        diagnostic_detail( "device transport group sent stop: unacked_seq=%u sent_mask=%u required_mask=%u\n",
                group_sent_ack.sequence(), status.can_sent_mask(), parameters.configured_mask);
        ++heartbeat_stops;
        ++group_sent_ack_timeouts;
        group_sent_ack.reset();
        stop_latched = true;
        system.request_transport_stop();
      }
      /**
       * @brief 状态先失鲜时可能尚未到ACK截止，独立累计ack_unconfirmed原因并说明SENT未知；然后停止普通控制，不续状态deadline。
       */
      if (!released_idle && !reobserve_raw_idle && stop_causes.state_stale) {
        ++state_stale_stops;
        if (stop_causes.ack_unconfirmed_on_state_stale_stop) {
          ++ack_unconfirmed_on_state_stale_stop;
          diagnostic_detail( "device transport state stale stop: ACK unconfirmed before deadline; CAN SENT unknown\n");
        }
        if (!stop_latched) {
          stop_latched = true;
          system.request_transport_stop();
        }
        invalidate();
      }
      /**
       * @brief 准备Hold/EnablingHold及受限Cancel安全prime期间可由worker主动转交控制快照，避免框架inactive不调用write造成prime不可见；transfer_busy仍与ControllerManager串行。
       */
      if (management->inactive_prime_allowed.load(std::memory_order_acquire) ||
          status.core_state() == kWireEnablingHold ||
          (stop_wait_cancel && owned_cancel && !owned_cancel_invalid && !owned_cancel_committed &&
           !stop_latched && transport_steady_now_us()<=stop_deadline_us &&
           cancel_prime_latest_core==kWireEnablingHold)) {
        const bool transferred = system.transfer_control_update();
        if (status.core_state() == kWireEnablingHold &&
            ((transferred && enabling_transfer_log_count < 3U) || enabling_transfer_log_count == 0U)) {
          ++enabling_transfer_log_count;
          diagnostic_detail( "COMMAND_ENABLE_DIAG transfer=%d inactive=%d status_motion=%u authorized_motion=%u\n",
                  transferred, !system.commands_active(), status.motion_epoch(), authorized_motion_epoch);
        }
        if (transferred && !logged_prime_transfer) {
          logged_prime_transfer = true;
          diagnostic_detail( "COMMAND_PRIME_DIAG transfer inactive=%d core_state=%u boot=%llu status_motion=%u authorized_motion=%u\n",
                  !system.commands_active(), status.core_state(),
                  static_cast<unsigned long long>(boot), status.motion_epoch(), authorized_motion_epoch);
        }
        if (status.core_state() == kWireEnablingHold && !transferred) {
          const auto observed_at = transport_steady_now_us();
          if (observed_at - last_enable_visibility_log_us >= 1000000U) {
            last_enable_visibility_log_us = observed_at;
            const auto visible = consume_control_update(0U);
            diagnostic_detail( "COMMAND_ENABLE_DIAG visible=%d generation=%llu current=%d inactive_allowed=%d\n",
                    visible.has_value(),
                    static_cast<unsigned long long>(visible ? visible->generation : 0U),
                    visible && command_record_current(*visible),
                    management->inactive_prime_allowed.load(std::memory_order_acquire));
          }
        }
      }
      publish_pending(command_publisher,
          stop_wait_cancel && owned_cancel && !owned_cancel_invalid && !owned_cancel_committed ? &*owned_cancel : nullptr,
          stop_deadline_us, owned_cancel_run);
      /**
       * @brief 排空快照不是只看队列空：还要求Hold、无Stop/拒绝、无本地pending/in-flight/ACK，并且最近处理Hold代与最近设备ACK身份完全一致。position/tolerance单位rad，观察锚保持原State时间。
       */
      HoldDrainSnapshot drain{};
      if(status.core_state()==kWireHold && !stop_latched && !command_sent &&
         !hold_drain_rejected &&
         !command_stream.has_pending() && !command_stream.has_in_flight() &&
         !group_sent_ack.pending() && last_processed_hold_generation!=0U &&
         last_ack_seq!=0U &&
         last_processed_hold_identity.boot==last_ack_identity.boot &&
         last_processed_hold_identity.session==last_ack_identity.session &&
         last_processed_hold_identity.reference_epoch==last_ack_identity.reference_epoch &&
         last_processed_hold_identity.motion_epoch==last_ack_identity.motion_epoch) {
        drain={last_ack_identity.boot,last_ack_identity.session,
               last_processed_hold_generation,last_ack_identity.reference_epoch,
               last_ack_identity.motion_epoch,last_ack_seq};
        drain.disposition=last_processed_hold_disposition;
        drain.quiescent=true;
        drain.observed_us=observed_hold.received_us;
        drain.max_state_age_us=parameters.max_state_age_us;
        drain.position=last_processed_hold_position;
        for(std::size_t axis=0;axis<drain.tolerance.size();++axis)
          drain.tolerance[axis]=management->post_complete_hold_tolerance_rad[axis].load(
            std::memory_order_acquire);
      }
      { std::lock_guard<std::mutex> lock(management->mutex); management->hold_drain=drain; }
    }
    /**
     * @brief 正常离开连接循环时，未完成管理事务显式failed；若Stop还待终Query/Idle则标未确认。局部句柄随后同线程RAII释放。
     */
    if(active_management && !active_management->complete) {
      log_management_handoff(*active_management,"worker_exited");
      active_management->failed=true;
    }
    if (stop_confirmation_pending.load(std::memory_order_acquire))
      fail_stop("transport ended before terminal QueryResult/Idle");
    return true;
  }

  /**
   * @brief 安装worker诊断scope并重复建立连接，退出前统一失效管理/控制。
   * run_connected异常转failed；每次断连清ready及cleanup代、拒绝排队管理、invalidate并消费无连接Stop作为未确认。running仍真时宿主休眠100ms重试；重连需要重新能力/会话/许可周期，不恢复旧目标。
   */
  void run() noexcept {
    WorkerDiagnosticScope diagnostic_scope(diagnostics);
    while (running.load(std::memory_order_acquire)) {
      try { failed = !run_connected(); }
      catch (...) { failed = true; }
      if (stop_confirmation_pending.exchange(false)) {
        diagnostics.reject_from_context(DiagnosticReason::kStopUnconfirmed,transport_steady_now_us(),"worker_exit_before_confirmation");
      diagnostic_detail( "device transport UNCONFIRMED_STOP: worker exited before confirmation\n");
        ++unconfirmed_stop_requests;
      }
      /**
       * @brief 离开连接后首先关闭对外ready并推进cleanup生命周期，writer尚在排空的旧State/Status不能给新连接重新发布“可用”原始观测。
       */
      ready = false;
      management->ready = false;
      management->reset_worker_release_connection();
      management->reset_raw_cleanup_lifetime();
      management->reset_diagnostic_observation();
      management->inactive_prime_allowed=false;
      {
        std::lock_guard<std::mutex> lock(management->mutex);
        if(management->queued) {
          log_management_handoff(*management->queued,"worker_unavailable");
          management->queued->failed=true;
        }
        management->queued.reset(); management->observation={};
      }
      invalidate();
      /**
       * @brief 没有连通worker却有待Stop意图时明确记录发送失败/UNCONFIRMED；清本地active或销毁线程不能替代设备停机回执。
       */
      if (system.take_stop_request()) {
        diagnostics.reject_from_context(DiagnosticReason::kStopUnconfirmed,transport_steady_now_us(),"no_connected_worker");
        diagnostic_detail( "device transport UNCONFIRMED_STOP: no connected worker\n");
        ++stop_request_failures;
        ++unconfirmed_stop_requests;
      }
      if (running) std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
};

/**
 * @brief 建立唯一Impl并移动配置，暂不启动通信线程。
 * Impl构造会注册共享管理channel及预热Stop ID；真正Context和dp_comm端点仍延迟到worker。
 * @param system 必须活过transport的硬件对象。
 * @param parameters 明确端点和身份/限制值。
 */
DeviceTransport::DeviceTransport(GuardedZenbeddedSystem &system, TransportParameters parameters)
    : impl_(std::make_unique<Impl>(system, std::move(parameters))) {}
/**
 * @brief 调用stop有序等待worker与collector writer。
 * 析构是非实时操作；设备释放是否得到证明仍从统计和诊断判断，不能从析构返回推导。
 */
DeviceTransport::~DeviceTransport() { stop(); }
/**
 * @brief 先启动可选独占证据文件，再创建通信worker。
 * start不会同步等待bootstrap、State或设备授权；若旧线程已结束但仍joinable先join。证据启动失败不创建通信线程，线程创建异常会停止collector。
 * @return true表示已运行或线程创建成功；collector/线程启动失败false。
 */
bool DeviceTransport::start() {
  if (impl_->running.exchange(true)) return true;
  if (impl_->worker.joinable()) impl_->worker.join();
  if (!impl_->parameters.raw_evidence_path.empty() &&
      !impl_->raw_evidence.start(impl_->parameters.raw_evidence_path)) {
    impl_->running = false;
    impl_->failed = true;
    impl_->diagnostics.reject_from_context(DiagnosticReason::kEvidenceFailure,transport_steady_now_us(),"raw_capture_start_failed");
    return false;
  }
  try { impl_->worker = std::thread([this] { impl_->run(); }); }
  catch (...) {
    impl_->diagnostics.reject_from_context(DiagnosticReason::kManagementRejected,transport_steady_now_us(),"worker_thread_start_failed");
    impl_->running = false; impl_->raw_evidence.stop(); return false;
  }
  return true;
}
/**
 * @brief 给已有Stop确认一次配置预算的处理机会，再join并封口采集。
 * 仅当worker正在运行且有待Stop/待确认时才短轮询；之后running=false、join通信worker、stop writer。若临时run/Status链/终页flush未对账，显式reject_invalid；任何采集首错反映initialization_failed，不将线程结束伪造释放成功。
 */
void DeviceTransport::stop() {
  if (impl_->running.load(std::memory_order_acquire) &&
      (impl_->system.stop_request_pending() ||
       impl_->stop_confirmation_pending.load(std::memory_order_acquire))) {
    const auto timeout_us = impl_->parameters.stop_confirmation_timeout_us != 0U
        ? impl_->parameters.stop_confirmation_timeout_us : impl_->parameters.max_state_age_us;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::microseconds(timeout_us);
    while (impl_->running.load(std::memory_order_acquire) &&
           (impl_->system.stop_request_pending() ||
            impl_->stop_confirmation_pending.load(std::memory_order_acquire)) &&
           std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  impl_->running = false;
  if (impl_->worker.joinable()) impl_->worker.join();
  impl_->raw_evidence.stop();
  if (impl_->provisional_evidence_binding) {
    const auto run = impl_->provisional_evidence_binding->run_id;
    const bool release_reconciled = impl_->provisional_release_chain &&
        impl_->raw_evidence.terminal_written_run_id() == run &&
        impl_->last_terminal_chain_run == run &&
        *impl_->provisional_release_chain == impl_->last_terminal_chain;
    if (!release_reconciled) impl_->raw_evidence.reject_invalid();
  }
  if (impl_->raw_evidence.failure() != RawEvidenceCollector::Failure::kNone) impl_->failed = true;
}
/**
 * @brief 复制管理channel对象生命周期诊断首现场与计数。
 * @return DiagnosticSnapshot；不清首错，不输出JSON或stdio。
 */
DiagnosticSnapshot DeviceTransport::diagnostic_snapshot() const noexcept { return impl_->diagnostics.snapshot(); }
/**
 * @brief 读取首个本地publish参数原始诊断blob。
 * @return optional值；LOCAL_PUBLISH_ARGUMENT不代表实际USB传输或设备SENT。
 */
std::optional<DiagnosticBlob> DeviceTransport::diagnostic_blob() const noexcept { return impl_->diagnostics.blob_record(); }
/**
 * @brief 读取一个已经发布的有限追加详情槽。
 * @param index DiagnosticJournal槽索引，越界/未ready返回空。
 * @return optional值副本；格式化由非控制消费侧完成。
 */
std::optional<DiagnosticDetail> DeviceTransport::diagnostic_detail_record(std::size_t index) const noexcept { return impl_->diagnostics.detail_record(index); }
/**
 * @brief 汇总原子计数、journal与collector水位为非事务快照。
 * state_callbacks为到达，state_messages为复制成功后处理，rejected_*区分原因；commands_published只本地接纳，stop_requests_confirmed才是完整Stop协议确认。多个原子分别采样，不保证所有字段同一瞬间。
 * @return TransportStatistics；ready还要求采集无首错，initialization_failed包含collector失败。
 */
TransportStatistics DeviceTransport::statistics() const noexcept {
  TransportStatistics result{};
  result.diagnostics=impl_->diagnostics.snapshot();
  result.state_callbacks = impl_->state_callbacks.load();
  result.state_messages = impl_->states.load();
  result.status_messages = impl_->statuses.load();
  result.stale_run_status_rejections = impl_->stale_run_status_rejections.load();
  result.rejected_states = impl_->rejected_states.load();
  result.state_decode_rejections = impl_->state_decode_rejections.load();
  result.state_identity_rejections = impl_->state_identity_rejections.load();
  result.state_authority_rejections = impl_->state_authority_rejections.load();
  result.state_sequence_rejections = impl_->state_sequence_rejections.load();
  result.state_ingest_rejections = impl_->state_ingest_rejections.load();
  result.stop_latched_state_rejections = impl_->stop_latched_state_rejections.load();
  result.commands_published = impl_->commands.load();
  result.rejected_commands = impl_->rejected_commands.load();
  result.invalidations = impl_->invalidations.load();
  result.stop_requests_sent = impl_->stop_requests_sent.load();
  result.stop_request_failures = impl_->stop_request_failures.load();
  result.stop_requests_confirmed = impl_->stop_requests_confirmed.load();
  result.unconfirmed_stop_requests = impl_->unconfirmed_stop_requests.load();
  result.heartbeat_warnings = impl_->heartbeat_warnings.load();
  result.heartbeat_stops = impl_->heartbeat_stops.load();
  result.group_sent_ack_timeouts = impl_->group_sent_ack_timeouts.load();
  result.group_sent_ack_misses = impl_->group_sent_ack_misses.load();
  result.state_stale_stops = impl_->state_stale_stops.load();
  result.raw_idle_reobservations = impl_->raw_idle_reobservations.load();
  result.ack_unconfirmed_on_state_stale_stop =
      impl_->ack_unconfirmed_on_state_stale_stop.load();
  result.bootstrap_rejections = impl_->bootstrap_rejections.load();
  result.stale_payload_rejections = impl_->stale_payload_rejections.load();
  result.clock_skew_rejections = impl_->clock_skew_rejections.load();
  result.state_heartbeat_gap_stops = impl_->state_heartbeat_gap_stops.load();
  result.last_state_gap_us = impl_->last_state_gap_us.load();
  result.raw_evidence_written = impl_->raw_evidence.written();
  result.raw_evidence_pages_written = impl_->raw_evidence.pages_written();
  result.raw_evidence_last_terminal_written_run =
      impl_->raw_evidence.terminal_written_run_id();
  result.raw_evidence_failure = static_cast<std::uint8_t>(impl_->raw_evidence.failure());
  result.ready = impl_->ready.load() &&
      impl_->raw_evidence.failure() == RawEvidenceCollector::Failure::kNone;
  result.initialization_failed = impl_->failed.load() ||
      impl_->raw_evidence.failure() != RawEvidenceCollector::Failure::kNone;
  return result;
}
/**
 * @brief 用try_lock查找与调用参数身份一致的当前共享channel。
 * 查找不等待注册锁，取得句柄也不保证ready、能力、状态新鲜或设备授权；它保留的是可读账本与单槽管理交接。
 * @param parameters 期望endpoint/device/client/profile/mask组合。
 * @return shared_ptr或空；锁忙/弱引用失效/身份不匹配均空。
 */
std::shared_ptr<DeviceManagementChannel> DeviceTransport::management_channel(const TransportParameters &parameters) noexcept {
  std::unique_lock<std::mutex> lock(channel_registry_mutex,std::try_to_lock);
  if(!lock) return {};
  const auto channel=channel_registry.lock();
  if(!channel) return {};
  const auto &actual=channel->parameters;
  if(actual.endpoint!=parameters.endpoint || actual.device_id!=parameters.device_id ||
     actual.client_instance_id!=parameters.client_instance_id || actual.profile_id!=parameters.profile_id ||
     actual.configured_mask!=parameters.configured_mask) return {};
  return channel;
}
}  // namespace dummy_ros_control

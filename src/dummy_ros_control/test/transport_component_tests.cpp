/**
 * @file transport_component_tests.cpp
 * @brief 生产transport与受控SIL peer的生命周期/证据场景回归。
 *
 * 本文件以生产Host DeviceTransport配合本地dp_comm/pico SIL peer覆盖反馈准入、Complete/Cancel/Stop与raw采集顺序。大部分peer回执是固定wire夹具；只有另两个Core专用源才接入RobotCore，不可据本文件的合成COMMITTED宣称真实Core执行。
 * PTY字节桥模拟串行端点，pipe传递同一宿主ABI的刺激/观测对象；这里没有打开真实USB或电机端口。
 * 角位置rad、速度rad/s、加速度rad/s²，raw电机位置rev；Source produced_at_us与Host接收/期限µs分属不同时钟域。
 * 本地发送、RPC回调complete、业务事务COMMITTED、CAN全组SENT、raw文件flush及物理完成是不同证据层，须看各分支具体断言。
 * 程序仅执行CLI选择的场景，未选分支不能计为覆盖；构建成功不等于全部分支已运行。
 */
/**
 * @brief 选择取消后Hold Status使用final7而非旧final0；仅影响本测试构造的刺激。
 * 与cancel_status_bad_flags组合覆盖终态、摘要及failure字段矛盾，不能把模式名当设备事实。
 */
static bool cancel_status_terminal7_variant=false;
static bool manual_history_mode = false;
static bool owner_safety_stop_case = false;
static bool root_presence_case=false;
/** @brief 完成Status先于writer flush的离线乱序回归。 */
static bool complete_status_before_flush_case=false;
/** @brief 常驻回环router上的Boot恢复场景；与PTY物理重连资格分别取证。 */
static bool root_tcp_boot_case=false;
static bool manual_release_mode = false;
static bool manual_fault_diagnostic_mode = false;
static bool manual_recovered_idle_mode = false;
static unsigned manual_recovered_idle_fault = 0U;
static bool manual_recovered_release_mode = false;
static unsigned released_child_case = 0U;

/**
 * @brief 取消Status负例编号：1..4改final为4/5/6/9，5改链、6清链，7/8/9设置failure source/axis/reason。
 * 0保持原本干净Status；所有编号在main选择，后续均要求失败关闭。
 */
static unsigned cancel_status_bad_flags=0U;
/**
 * @brief 将取消后的State改Stopping7且hold_ready=false，隔离“有终态Status但没有就绪Hold反馈”的负例。
 */
static bool cancel_state_stopping_notready=false;
/**
 * @brief 使用Hold4 Status变体；cancel_hold_case为71..82，运行时映射到raw_order_case59..70。
 */
static bool cancel_hold_status_variant=false;
/**
 * @brief 保存CLI的Hold Status变体编号，数值不是设备状态码或协议序号。
 */
static unsigned cancel_hold_case=0;
#include "wire_operation.hpp"
#include "historical_idle_baseline_fixture.hpp"
#include <array>
#include <atomic>
#include <cerrno>
#include <csignal>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <dlfcn.h>
#include <poll.h>
#include <stdlib.h>
#include <sys/wait.h>
#include <unistd.h>
#include <termios.h>
#include <thread>
#include <openssl/sha.h>

#include "dp_comm/dp_comm.hpp"
#include "dummy_ref/wire_contract.hpp"
#include "root_component_stimulus.hpp"
#include "dummy_ref/root_transport.hpp"
#include "root_owned_fields_fixture.hpp"
#include "attempt_evidence.hpp"
#include "dummy_ref/root_deployment.hpp"
#include "dummy_ref/root_endpoint_qos.hpp"
#include "dp_host_instance_identity.h"
#include "dp_comm_port_api.h"
#include "dummy_ros_control/command_channel.hpp"
#include "dummy_ros_control/device_transport.hpp"
#include "dummy_ros_control/raw_evidence_collector.hpp"
#include "dummy_ros_control/device_management_channel.hpp"
#include "dummy_ros_control/guarded_zenbedded_system.hpp"

// 只在本测试进程拦截文件flush；生产collector仍执行真实写入与真实fflush。
// 暂停终页flush而不阻塞通信worker，复现验链与写出回执之间的窗口。
static std::atomic<bool> cancel_flush_armed{false}, cancel_flush_entered{false}, cancel_flush_resume{false};
// 捕获回执getter的mutex，只在副本已复制、mutex已解锁但getter尚未返回时暂停worker。
// 此时放行writer并等水位发布，确定性构造“旧回执副本 + 随后发布的新水位”。
static thread_local bool capture_cancel_receipt_mutex=false;
static std::atomic<pthread_mutex_t*> cancel_receipt_mutex{nullptr};
static std::atomic<bool> cancel_receipt_armed{false}, cancel_receipt_entered{false}, cancel_receipt_resume{false};
static bool cancel_pending_flush_case=false;
static unsigned cancel_pending_flush_fault=0U;
static bool cancel_origin_case=false;
static unsigned cancel_origin_fault=0U;
extern "C" int pthread_mutex_unlock(pthread_mutex_t* mutex) noexcept {
  using Unlock=int(*)(pthread_mutex_t*);
  static const auto actual=reinterpret_cast<Unlock>(::dlsym(RTLD_NEXT,"pthread_mutex_unlock"));
  const int result=actual(mutex);
  if(capture_cancel_receipt_mutex)cancel_receipt_mutex=mutex;
  if(mutex==cancel_receipt_mutex.load() && cancel_receipt_armed.exchange(false)) {
    cancel_receipt_entered=true;
    while(!cancel_receipt_resume.load())std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return result;
}
extern "C" int fflush(FILE* file) {
  using Flush=int(*)(FILE*);
  static const auto actual=reinterpret_cast<Flush>(::dlsym(RTLD_NEXT,"fflush"));
  if(file && ::fileno(file)>2 && cancel_flush_armed.exchange(false)) {
    cancel_flush_entered=true;
    while(!cancel_flush_resume.load())std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  if(file && ::fileno(file)>2 && cancel_flush_entered.load() && cancel_pending_flush_fault==6U) {errno=EIO;return EOF;}
  return actual(file);
}

namespace {
/**
 * @brief 保存完整普通场景收尾时的worker结构化诊断，供main核Stop时限/身份。
 * 只有该主文件直接保存并检查快照，不能推广为另两文件的stderr文本路径。
 */
static dummy_ros_control::DiagnosticSnapshot final_worker_diagnostic_snapshot;
/**
 * @brief 显式选择Cancel与生命周期Stop重叠的源码场景；普通CLI分支不自动启用。
 */
static bool source_cancel_coordination=false;
/**
 * @brief 指向argv中负例后缀（或正例nullptr）；用于破坏Cancel Query身份/阶段或Hold，原字符串生存期覆盖整个场景。
 */
static const char* source_cancel_fault=nullptr;
using State=dummy_ref::messages::State;
using Status=dummy_ref::messages::Status;
using Evidence=dummy_ref::messages::Evidence;
using Command=dummy_ref::messages::Command;
using Management=dummy_ref::root::Manage;
using namespace dummy_ros_control;
/**
 * @brief 检查采集尚未绑定时可接受的历史released Idle基线。
 * @return 0全部关系成立；301..322分别定位身份、终态、页数、链或历史水位条件。
 * @details 首先允许普通motion与有parent的独立release历史终态；再逐项破坏boot/session/permit/lost/core及历史上下文。
 * 非零历史链在当前采集还没有页时可保留；已落页、已绑定current_boot或历史run/链不匹配都不能当作无害启动基线。
 */
int test_released_idle_startup_baseline(){return root_fixture::historical_idle_baseline_cases();}
/**
 * @brief 用人工Host µs检查一代命令的精确组ACK及超时分类。
 * @return true表示身份/producer序号匹配、拒绝计数推进和State超龄归因均符合预期。
 * @details generation42、boot7/session8/ref9/motion10；从1000µs起100000µs预算。旧组全mask不能确认本组；精确State committed序号可在Status mask为0时确认。
 * 迟于期限、错误四元身份或错误序号不确认；command_rejected必须是新拒绝计数且精确待确认上下文。
 */
bool test_group_sent_ack() {
  CommandGenerationRecord record{};
  record.generation = 42U;
  record.identity = {7U, 8U, 9U, 10U, 11U};
  GroupSentAck first;
  first.begin(record, 1000U, 100000U);
  if (!first.pending() || first.overdue(11000U) || first.overdue(101000U)) return false;
  // A previous group's full mask does not prove this sequence was committed.
  if (first.observe(7U, 8U, 9U, 10U, 41U, 0x3fU, 11000U)) return false;
  if (first.observe(7U, 8U, 9U, 10U, 42U, 0x3fU, 101001U)) return false;
  if (!first.overdue(101001U)) return false;

  GroupSentAck second;
  second.begin(record, 1000U, 100000U);
  if (second.observe(6U, 8U, 9U, 10U, 42U, 0x3fU, 11000U) ||
      second.observe(7U, 6U, 9U, 10U, 42U, 0x3fU, 11000U) ||
      second.observe(7U, 8U, 6U, 10U, 42U, 0x3fU, 11000U) ||
      second.observe(7U, 8U, 9U, 6U, 42U, 0x3fU, 11000U) ||
      second.observe(7U, 8U, 9U, 10U, 43U, 0x3fU, 11000U)) return false;
  // Status can be suppressed or retain an old/empty mask; State's exact
  // committed sequence is sufficient evidence of the complete CAN group.
  if (!second.observe(7U, 8U, 9U, 10U, 42U, 0U, 11000U) || second.pending() ||
      second.overdue(200000U)) return false;

  GroupSentAck rejected;
  rejected.begin(record, 1000U, 100000U, 3U, 41U);
  if (rejected.command_rejected(7U, 8U, 9U, 10U, 41U, 3U, 11000U) ||
      rejected.command_rejected(6U, 8U, 9U, 10U, 41U, 4U, 11000U) ||
      rejected.command_rejected(7U, 6U, 9U, 10U, 41U, 4U, 11000U) ||
      rejected.command_rejected(7U, 8U, 6U, 10U, 41U, 4U, 11000U) ||
      rejected.command_rejected(7U, 8U, 9U, 6U, 41U, 4U, 11000U) ||
      rejected.command_rejected(7U, 8U, 9U, 10U, 41U, 2U, 11000U) ||
      rejected.command_rejected(7U, 8U, 9U, 10U, 41U, 4U, 101001U) ||
      rejected.command_rejected(7U, 8U, 9U, 10U, 42U, 4U, 11000U) ||
      !rejected.command_rejected(7U, 8U, 9U, 10U, 41U, 4U, 11000U) ||
      !rejected.pending()) return false;

  /**
   * @brief State先超龄时记录ACK未确认但不重复算ACK timeout；同时超过两个门槛时则保留各自原因。
   * 这是停止原因分类的枚举/计数关系，不发送CAN或证明设备已停止。
   */
  GroupSentAck default_budget;
  default_budget.begin(record, 1010U, 100000U);
  const auto state_stale_first = classify_transport_stop_causes(
      default_budget, 101001U, 1000U, 100000U);
  if (!state_stale_first.state_stale || state_stale_first.ack_timed_out ||
      !state_stale_first.ack_unconfirmed_on_state_stale_stop) return false;

  GroupSentAck simultaneous;
  simultaneous.begin(record, 1000U, 100000U);
  const auto both_overdue = classify_transport_stop_causes(
      simultaneous, 101001U, 1000U, 100000U);
  if (!both_overdue.state_stale || !both_overdue.ack_timed_out ||
      both_overdue.ack_unconfirmed_on_state_stale_stop) return false;
  return true;
}
/**
 * @brief 区分旧周期倍数Stop和显式250ms硬件State年龄预算。
 * @return true表示2周期告警、预算截止边界、reset后未观测State不误停均通过。
 * @note 时间是同域人工µs，不包含主机调度最坏延迟证明。
 */
bool test_hardware_state_age_heartbeat() {
  TransportHeartbeat legacy(10000U);
  legacy.observe_state(100U);
  if (legacy.poll(50099U) == HeartbeatAction::kStop ||
      legacy.poll(50100U) != HeartbeatAction::kStop) return false;
  TransportHeartbeat heartbeat(20000U);
  heartbeat.observe_state(1000U);
  if (heartbeat.poll(40999U, 250000U) != HeartbeatAction::kNone ||
      heartbeat.poll(41000U, 250000U) != HeartbeatAction::kWarn ||
      heartbeat.poll(102758U, 250000U) != HeartbeatAction::kWarn ||
      heartbeat.poll(251000U, 250000U) != HeartbeatAction::kWarn ||
      heartbeat.poll(251001U, 250000U) != HeartbeatAction::kStop) return false;
  heartbeat.reset();
  return heartbeat.poll(300000U, 250000U) == HeartbeatAction::kNone;
}
/**
 * @brief 参数解析只允许显式心跳Stop预算处于两State周期到max_state_age之间。
 * @return true表示默认0、显式250000接受，而250001及39999拒绝。
 * @details State周期20000µs、max_state_age250000µs；端点serial/test仅为解析字符串，此函数不启动通信。
 */
bool test_heartbeat_budget_parameter() {
  const std::string hash(64U, '1');
  std::unordered_map<std::string, std::string> values{
      {"endpoint", "serial/test"},
      {"device_id", "01000000000000000000000000000000"},
      {"client_instance_id", "02000000000000000000000000000000"},
      {"profile_id", "1"}, {"configured_mask", "63"},
      {"contract_sha256", hash}, {"model_sha256", hash},
      {"profile_sha256", hash}, {"firmware_sha256", hash},
      {"compiled_policy_sha256", hash},
      {"position_min_rad", "-1,-1,-1,-1,-1,-1"},
      {"position_max_rad", "1,1,1,1,1,1"},
      {"velocity_max_rad_s", "1,1,1,1,1,1"},
      {"acceleration_max_rad_s", "1,1,1,1,1,1"},
      {"control_period_us", "10000"},
      {"state_publish_period_us", "20000"},
      {"max_state_age_us", "250000"}};
  TransportParameters parsed{};
  if (!parse_transport_parameters(values, 6U, &parsed) ||
      parsed.heartbeat_stop_budget_us != 0U) return false;
  values["heartbeat_stop_budget_us"] = "250000";
  if (!parse_transport_parameters(values, 6U, &parsed) ||
      parsed.heartbeat_stop_budget_us != 250000U) return false;
  values["heartbeat_stop_budget_us"] = "250001";
  if (parse_transport_parameters(values, 6U, &parsed)) return false;
  values["heartbeat_stop_budget_us"] = "39999";
  return !parse_transport_parameters(values, 6U, &parsed);
}
/**
 * @brief 用正式lifecycle_to_wire获取Disabling编码，避免测试把Stopping和Disabling混用。
 * @return Disabling对应wire core_state；仅转换内存快照。
 */
std::uint16_t ref_disabling_wire_state() {
  dummy_ref::JointSnapshot snapshot{};
  snapshot.lifecycle=dummy_ref::LifecycleState::kDisabling;
  return dummy_ref::comm::lifecycle_to_wire(snapshot);
}
/**
 * @brief 父进程经pipe送给peer的一次发布刺激，属于同一宿主ABI对象而非wire包。
 * status/state/evidence各自保存完整消息值；send_*决定本次发布哪些topic，done通知退出而不发布。
 * 默认发Status和State，不发Evidence；这样可独立安排回执、Status、State、原页的到达顺序。
 */
using Stimulus=root_fixture::Stimulus;;
/**
 * @brief 普通场景读回的实际Stop request_id，供同一进程两次运行比较不重复；不是预填期望值。
 */
std::uint64_t observed_stop_request_id = 0U;
/**
 * @brief 对peer首次State publish注入一次QueueFull，检查有界重试不会把暂时背压变成测试假失败。
 */
bool inject_peer_state_queue_full_once = false;
/**
 * @brief 专用于release Query回执故障：1错op、2错motion、3错management、4仍Pending、5未release_committed。
 */
unsigned release_receipt_fault=0U;
// 同一 peer 跨 exec 回归专用；普通组件场景不启用。
/**
 * @brief 被transport_stop_exec_tests.cpp复用时保留跨Host exec的合成设备缓存。
 * 普通本文件入口默认false；不代表本main自行执行了跨exec回归。
 */
bool persistent_stop_cache_test = false;
/**
 * @brief 由复用夹具设置的旧Stop回执重放负例开关；要求收到匹配Idle也不能确认新请求。
 */
bool replayed_stop_host = false;
/**
 * @brief 预期Stop业务operation_id，普通场景900；跨exec使用者按轮次设置。
 * operation_id与原RPC request_id/Query自己的request_id是不同关联字段。
 */
std::uint64_t exec_expected_stop_operation = 900U;
/**
 * @brief 跨fork共享区中保存前三轮Stop身份及首轮终态，普通组件场景不分配它。
 * round是设备缓存轮次；stop保留原Disable请求，terminal保留原Query响应，request_ids检查每个新Host没有复用Stop逻辑ID。
 */
struct PersistentStopCache {
  unsigned round = 0U;
  Management::Request stop=owned_fixture::request();
  Management::Response terminal=owned_fixture::response();dp::comm::SourceId terminal_source{};
  std::uint64_t request_ids[3]{};
};
/**
 * @brief 非拥有指针，指向调用者建立的共享mmap对象；本文件普通模式保持nullptr。
 */
PersistentStopCache *persistent_stop_cache = nullptr;
/**
 * @brief 对peer自身消息发送做有限QueueFull重试。
 * @tparam Publish 零参数发送回调，返回dp_comm Result。
 * @tparam Pump 零参数执行器推进回调。
 * @param publish 保留原消息对象的发送动作，不重建管理身份。
 * @param pump 背压时消费本地执行器工作，允许Ok/Busy继续。
 * @return 最多16次发布的最终结果；非QueueFull立即返回，pump硬错误也立即返回。
 * @note 只有两次尝试之间sleep1ms，最多pump15次；本辅助不是生产管理事务无限重试器。
 */
template <typename Publish, typename Pump>
dp::comm::Result publish_peer_with_retry(Publish publish, Pump pump) {
  constexpr unsigned kMaxAttempts = 16U;
  for (unsigned attempt = 0U; attempt < kMaxAttempts; ++attempt) {
    const auto result = publish();
    if (result != dp::comm::Result::kQueueFull || attempt + 1U == kMaxAttempts)
      return result;
    const auto pumped = pump();
    if (pumped != dp::comm::Result::kOk && pumped != dp::comm::Result::kBusy)
      return pumped;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return dp::comm::Result::kQueueFull;
}
/**
 * @brief 检查一次QueueFull可恢复、持续QueueFull恰16次和硬错误不重试。
 * @return 0通过；351瞬时背压节奏错误；352有限尝试数错误；353非重试错误未立即结束。
 * @par 内部回调
 * pump只加计数并Ok；transient第一次QueueFull后Ok，congested始终QueueFull，invalid始终InvalidArgument。均不做真实通信。
 */
int test_peer_publish_backpressure() {
  unsigned attempts = 0U;
  unsigned pumps = 0U;
  const auto pump = [&] { ++pumps; return dp::comm::Result::kOk; };
  const auto transient = [&] {
    return ++attempts == 1U ? dp::comm::Result::kQueueFull : dp::comm::Result::kOk;
  };
  if (publish_peer_with_retry(transient, pump) != dp::comm::Result::kOk ||
      attempts != 2U || pumps != 1U) return 351;
  attempts = 0U;
  pumps = 0U;
  const auto congested = [&] { ++attempts; return dp::comm::Result::kQueueFull; };
  if (publish_peer_with_retry(congested, pump) != dp::comm::Result::kQueueFull ||
      attempts != 16U || pumps != 15U) return 352;
  attempts = 0U;
  pumps = 0U;
  const auto invalid = [&] { ++attempts; return dp::comm::Result::kInvalidArgument; };
  if (publish_peer_with_retry(invalid, pump) != dp::comm::Result::kInvalidArgument ||
      attempts != 1U || pumps != 0U) return 353;
  return 0;
}
/**
 * @brief peer服务回调的本地状态，生命周期覆盖node/executor。
 * commands_fd把收到Command对象回传父进程；stops_fd回传Cancel/Complete/Disable及相应Query便于核真实发送身份。
 * 其余字段只为受控故障/顺序注入，不是生产设备状态存储。
 */
/**
 * @brief 连接两对PTY master的双向字节桥；端点仍走真实串行transport实现。
 * 桥工作线程只搬字节，不解释Dummy消息；20ms poll避免析构时无限睡眠。
 * 两个slave端点分别给Host与peer；不打开物理串口，不能据此宣称USB电气/CAN链路合格。
 */
class SerialBridge {
 public:
  /**
   * @brief 建立两对非阻塞PTY并取得slave路径。
   * @return 两对open/grant/unlock/ptsname全部成功为true；任一步失败false，已开fd留给析构回收。
   * @note 端点后缀baudrate=115200是transport解析配置，在PTY上不等于真实线速测量。
   */
  bool open_pair() {
    for (int index = 0; index < 2; ++index) {
      masters_[index] = ::posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
      if (masters_[index] < 0 || ::grantpt(masters_[index]) != 0 ||
          ::unlockpt(masters_[index]) != 0) return false;
      char name[128]{};
      if (::ptsname_r(masters_[index], name, sizeof(name)) != 0) return false;
      // PTY初始默认echo会把首个INIT反射给Host；在任何连接前设置叶层原始模式。
      const int slave=::open(name,O_RDWR|O_NOCTTY|O_NONBLOCK);
      termios attributes{};
      if(slave<0)return false;
      if(::tcgetattr(slave,&attributes)!=0){::close(slave);return false;}
      ::cfmakeraw(&attributes);
      const int configured=::tcsetattr(slave,TCSANOW,&attributes);::close(slave);
      if(configured!=0)return false;
      endpoints_[index] = std::string("serial/") + name + "#baudrate=115200";
    }
    return true;
  }
  /**
   * @brief 返回一侧串行slave端点。
   * @param index 调用方限定为0或1；本函数不做越界检查。
   * @return 类内字符串引用，必须在bridge生存期内使用。
   */
  const std::string &endpoint(int index) const { return endpoints_[index]; }
  /**
   * @brief fork后子进程关闭继承的master副本，由父进程桥独占转发。
   * @return 无；不改变父进程自己的fd表。
   */
  void close_child_fds() const { for (int fd : masters_) ::close(fd); }
  /**
   * @brief 开启父进程字节转发线程。
   * @return 无；本夹具每对象只调用一次，不支持重复start自动join旧线程。
   * @par 内部线程
   * 轮询任一master的可读事件，按方向调用forward；running=false后退出，由析构join。
   */
  void start() {
    running_ = true;
    worker_ = std::thread([this] {
      while (running_) {
        pollfd fds[2]{{masters_[0], POLLIN, 0}, {masters_[1], POLLIN, 0}};
        if (::poll(fds, 2, 20) <= 0) continue;
        if (fds[0].revents & POLLIN) forward(0, 1);
        if (fds[1].revents & POLLIN) forward(1, 0);
      }
    });
  }
  /**
   * @brief 停转发、join线程再关闭master；保证线程不继续访问已销毁对象或已关闭fd。
   * @return 无。
   */
  ~SerialBridge() {
    running_ = false;
    if (worker_.joinable()) worker_.join();
    for (int fd : masters_) if (fd >= 0) ::close(fd);
  }
 private:
  /**
   * @brief 非阻塞转发最多一次read取得的512字节批次。
   * @param source 源master索引0/1。
   * @param target 目标master索引0/1。
   * @return 无；EINTR重试，EAGAIN等可写，其他错误/EOF返回而不伪装写满。
   * @note offset只按实际写字节推进；退出running可中断未完成转发，这不是可靠存储队列。
   */
  void forward(int source, int target) {
    std::uint8_t bytes[512];
    const auto count = ::read(masters_[source], bytes, sizeof(bytes));
    if (count <= 0) return;
    for (ssize_t offset = 0; running_ && offset < count;) {
      const auto written = ::write(masters_[target], bytes + offset,
                                   static_cast<std::size_t>(count - offset));
      if (written > 0) offset += written;
      else if (written < 0 && errno == EINTR) continue;
      else if (written < 0 && errno == EAGAIN) {
        pollfd writable{masters_[target], POLLOUT, 0};
        ::poll(&writable, 1, 20);
      } else return;
    }
  }
  /**
   * @brief PTY master拥有句柄，-1表示未创建；析构只关闭非负fd。
   */
  int masters_[2]{-1, -1};
  /**
   * @brief 对应两个slave的transport端点文本，open_pair后固定。
   */
  std::string endpoints_[2];
  /**
   * @brief 父进程线程运行标志；与fork后子进程的副本不是跨进程同步变量。
   */
  std::atomic<bool> running_{false};
  /**
   * @brief 拥有字节桥线程；析构负责join，避免悬空this。
   */
  std::thread worker_;
};
/**
 * @brief 以steady_clock最多等3s，每2ms重评谓词。
 * @tparam Predicate 可调用bool谓词，可在其中发刺激/读统计。
 * @param predicate 满足预期时返回true的检查动作；到期还会再调用一次。
 * @return 最终谓词值，false让调用者返回自己的场景失败码。
 * @note 不重置生产Stop原始deadline；测试等待期限与业务确认预算分开。
 */
template <typename Predicate> bool wait_for(Predicate predicate) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (!predicate() && std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  return predicate();
}
/**
 * @brief Host侧驱动一个受控transport场景并读取真实worker观测。
 * @param endpoint Host PTY串行端点。
 * @param input peer→Host命令/R通知pipe读端。
 * @param output Host→peer Stimulus写端。
 * @param stops_input peer→Host管理操作观测读端。
 * @param six_axis true使用mask63/六位置，false使用J1 mask1。
 * @param ack_timeout_mode 缩短组ACK预算为30000µs并持续提供未ACK新State。
 * @param default_ack_age_mode 停发State，区分State超龄与ACK未确认/超时。
 * @param never_complete_stop peer一直Pending，收尾必须保留unconfirmed。
 * @param heartbeat_budget_mode 使用250ms显式State间隔预算。
 * @param inactive_prime_mode 未激活硬件下允许测试首个保持目标发布。
 * @param cold_idle_gap_mode 冷Idle断流重观测，不能伪造已授权参考反馈。
 * @param cold_fault_gap_mode 同为冷态但Status Fault，断流不得走无害Idle例外。
 * @param raw_capture_mode 开启本地raw文件采集。
 * @param raw_bad_release_mode 注入不相关parent的释放Status。
 * @param raw_incomplete_mode 已expect但无终态即stop，要求Incomplete。
 * @param raw_many_mode 顺序采集run3..11并拒绝旧run Status。
 * @param raw_order_case raw/ACK/cleanup场景编号，见main模式表与各分支。
 * @param complete_order_mode Complete顺序/保持排空/刷新场景编号。
 * @param release_deactivate_mode 显式释放与生命周期停机顺序编号。
 * @param ticket_mode 1续票据后发pending，2旧票据不发，3授权变化不得借续票据发目标。
 * @param reject_during_ack_mode 待ACK时收到明确新拒绝，须停机而非等ACK超时。
 * @param sequence_rebase_mode Stop/Query从最近真实管理序号后继续。
 * @param uncommitted_stop_mode Stop Query终态缺release_committed。
 * @param uncommitted_idle_status_mode Idle Status缺release_committed。
 * @return 0仅表示所选分支断言成立；非零为具体准备/期限/字段/统计失败码，不作为协议业务码。
 * @details 先构造能力hash、数值限制与合成双时钟，再启动生产DeviceTransport。多数子场景独立提前return，后面普通完整场景不会同时执行。
 * 局部lambda捕获的对象由本栈帧持有，transport.stop或析构负责停worker后离开栈；测试只在channel mutex内读写非原子共享观测。
 */
int run_parent(const std::string &endpoint, int input, int output, int stops_input,
               bool six_axis, bool ack_timeout_mode, bool default_ack_age_mode,
               bool never_complete_stop, bool heartbeat_budget_mode,
               bool inactive_prime_mode, bool cold_idle_gap_mode,
               bool cold_fault_gap_mode, bool raw_capture_mode,
               bool raw_bad_release_mode, bool raw_incomplete_mode,
               bool raw_many_mode, unsigned raw_order_case, unsigned complete_order_mode,
               unsigned release_deactivate_mode, unsigned ticket_mode,
               bool reject_during_ack_mode, bool sequence_rebase_mode,
               bool uncommitted_stop_mode, bool uncommitted_idle_status_mode) {
  pollfd response{input, POLLIN, 0};
  char ready{};

  TransportParameters parameters{};
  parameters.endpoint = endpoint;
  parameters.device_id[0] = 1U;
  parameters.client_instance_id[0] = 2U;
  parameters.profile_id = 1U;
  parameters.configured_mask = six_axis ? 63U : 1U;
  const bool cold_gap_mode = cold_idle_gap_mode || cold_fault_gap_mode;
  /**
   * @brief max_state_age为Host反馈年龄门槛，按场景选250ms/200ms/1s。
   * heartbeat_stop_budget是“接收间隔”预算，ACK timeout是“某一代组确认”预算；三者不能互相代替。
   * source协调模式额外固定Stop确认250ms，后续Query或第二Stop不得把原窗口重新起算。
   */
  parameters.max_state_age_us = (heartbeat_budget_mode || cold_gap_mode ||
      release_deactivate_mode >= 8U || raw_order_case >= 51U) ? 250000U :
      (default_ack_age_mode ? 200000U : 1000000U);
  if (heartbeat_budget_mode || cold_gap_mode || release_deactivate_mode >= 8U ||
      raw_order_case >= 59U)
    parameters.heartbeat_stop_budget_us = 250000U;
  if (raw_order_case == 56U) parameters.heartbeat_stop_budget_us = 500000U;
  if (ack_timeout_mode) parameters.group_sent_ack_timeout_us = 30000U;
  if (reject_during_ack_mode) parameters.group_sent_ack_timeout_us = 250000U;
  if ((complete_order_mode>=6U && complete_order_mode<=8U) || complete_order_mode==10U ||
      complete_order_mode==27U)
    parameters.group_sent_ack_timeout_us = 30000U;
  if (release_deactivate_mode==4U && source_cancel_coordination) parameters.stop_confirmation_timeout_us=250000U;
  parameters.control_period_us = 10000U;
  if (release_deactivate_mode == 6U) parameters.heartbeat_stop_budget_us = 500000U;
  parameters.state_publish_period_us = (heartbeat_budget_mode || cold_gap_mode ||
      release_deactivate_mode >= 8U || raw_order_case >= 51U) ? 20000U :
      (six_axis ? 50000U : 1000000U);
  parameters.contract_hash[0] = 1U;
  parameters.model_hash[0] = 2U;
  parameters.profile_hash[0] = 3U;
  parameters.firmware_hash[0] = 4U;
  parameters.policy_hash[0] = 5U;
  /**
   * @brief 普通能力fixture统一位置[-1,1]rad、速度1rad/s、加速度2rad/s²。
   * hash首字节1..5只为区分合同/模型/profile/固件/策略，不是部署产品的真实身份摘要。
   */
  parameters.position_min.fill(-1.0);
  parameters.position_max.fill(1.0);
  parameters.velocity_max.fill(1.0);
  parameters.acceleration_max.fill(2.0);
  std::string capture_path;
  /**
   * @brief raw临时路径的栈作用域清理器，只借用capture_path引用。
   * 它先于transport构造，因此析构逆序使worker/文件先收尾再unlink。
   */
  struct CaptureCleanup {
    /**
     * @brief 引用外层路径，空值表示未创建capture；引用必须比清理器活得更久。
     */
    const std::string &path;
    /**
     * @brief 离开场景时删除非空临时文件，防止大量case留下输出。
     * @return 无；unlink失败未改变业务返回码。
     */
    ~CaptureCleanup() { if (!path.empty()) ::unlink(path.c_str()); }
  } capture_cleanup{capture_path};
  if (raw_capture_mode || replayed_stop_host) {
    char path[] = "/tmp/dummy-transport-raw-XXXXXX";
    const int fd = ::mkstemp(path);
    if (fd < 0) return 137;
    ::close(fd);
    ::unlink(path);
    capture_path = path;
    parameters.raw_evidence_path = capture_path;
  }
  parameters.manual_reference_mode =
      owner_safety_stop_case || manual_history_mode || manual_release_mode ||
      manual_fault_diagnostic_mode || manual_recovered_idle_mode ||
      released_child_case != 0U;
  GuardedZenbeddedSystem system;
  DeviceTransport transport(system, parameters);
  if (!transport.start() || !wait_for([&] { return transport.statistics().ready; })) {
    const auto failure=transport.diagnostic_snapshot();std::fprintf(stderr,"Host ready failed: %s\n",failure.first.detail.data());return 11;
  }
  // 先启动实际Host连接，peer的Root reader安装可需要底层会话握手；随后仍核真实R。
  if (::poll(&response, 1, 3000) != 1 || ::read(input, &ready, 1) != 1 || ready != 'R') return 10;
  Stimulus valid{};
  (void)0;
  (void)0;
  (void)0;
  (void)0;
  std::uint8_t device[16]{1U};
  valid.status.domain_device_id().emplace().assign(device,16);
  valid.state.domain_device_id().emplace().assign(device,16);
  valid.status_source.boot_id=(7U);
  valid.state_source.boot_id=(7U);
  valid.status.session_id()=(8U);
  valid.state.session_id()=(8U);
  if(complete_order_mode>=18U && complete_order_mode<=27U) {
    valid.status.run_id()=(1U);
    valid.status.attempt_kind()=(1U);
  }
  if (raw_capture_mode) {
    valid.status.run_id()=(raw_order_case >= 34U && raw_order_case <= 58U ? 1U :
        ((raw_order_case >= 24U && raw_order_case <= 33U) || raw_order_case >= 59U) ? 123U :
        raw_order_case >= 20U && raw_order_case <= 23U ? 121U :
        raw_order_case != 0U ? 1U : 3U);
    valid.status.attempt_kind()=(1U);
  }
  if(owner_safety_stop_case) {
    valid.status.core_state()=(4);valid.state.core_state()=(4);
    valid.status.management_epoch()=(1);valid.state.management_epoch()=(1);
    valid.status.operation_id()=(42);valid.state.operation_id()=(42);
  }
  valid.status.reference_epoch()=(9U);
  valid.state.reference_epoch()=(9U);
  valid.status.motion_epoch()=(10U);
  if (reject_during_ack_mode) valid.status.command_reject_count()=(3U);
  valid.state.motion_epoch()=(10U);
  if (cold_gap_mode) {
    valid.status.session_id()=(0U);
    valid.state.session_id()=(0U);
    valid.status.reference_epoch()=(0U);
    valid.state.reference_epoch()=(0U);
    valid.status.motion_epoch()=(0U);
    valid.state.motion_epoch()=(0U);
    valid.status.core_state()=(1U);
    valid.state.core_state()=(1U);
  }
  if (ack_timeout_mode || default_ack_age_mode || release_deactivate_mode != 0U) {
    valid.status.management_epoch()=(1U);
    valid.state.management_epoch()=(1U);
    valid.status.can_sent_mask()=(parameters.configured_mask);
  }
  valid.status.profile_id()=(1U);
  valid.state.profile_id()=(1U);
  valid.status.configured_mask()=(parameters.configured_mask);
  root_fixture::set_configured(valid.state,parameters.configured_mask);
  root_fixture::set_position_quality(valid.state,parameters.configured_mask);
  if (cold_gap_mode) {
    root_fixture::set_position_quality(valid.state,0U);
    root_fixture::set_raw_quality(valid.state,parameters.configured_mask);
    const double idle_raw[6]{0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
    root_fixture::set_raw_position(valid.state,idle_raw);
  }
  if (inactive_prime_mode) {
    valid.state.committed_producer_seq()=(44U);
    valid.state.admitted_producer_seq()=(44U);
  }
  if (manual_history_mode || manual_fault_diagnostic_mode ||
      manual_recovered_idle_mode) {
    valid.status.reference_epoch()=(1);
    valid.state.reference_epoch()=(1);
    valid.status.motion_epoch()=(113);
    valid.state.motion_epoch()=(113);
    valid.status.management_epoch()=(173);
    valid.state.management_epoch()=(173);
    valid.status.core_state()=(2);
    valid.state.core_state()=(2);
    root_fixture::set_position_quality(valid.state,parameters.configured_mask);
    valid.status.run_id()=(113);
    valid.status.attempt_kind()=(1);
    valid.status.final_state()=(manual_fault_diagnostic_mode ? 4 : 2);
    valid.status.release_committed()=(true);
    if (manual_fault_diagnostic_mode) {
      valid.status.core_state()=(9);
      valid.state.core_state()=(9);
    }
    if (manual_recovered_idle_mode) {
      valid.status.final_state()=(4);
      valid.status.cleanup_state()=(3);
      valid.status.failure_source()=(manual_recovered_release_mode ? 0x1064
                                                                    : 0x1067);
      if (manual_recovered_release_mode) {
        valid.status.attempt_kind()=(3);
        valid.status.parent_run_id()=(112);
      }
    }
  }
  if (released_child_case) {
    valid.status.core_state()=(2);
    valid.state.core_state()=(2);
    valid.status.management_epoch()=(1);
    valid.state.management_epoch()=(1);
    root_fixture::set_raw_quality(valid.state,63);
  }
  valid.status.capabilities()=(kPositionOnlyCapability);
  valid.status.control_period_us()=(parameters.control_period_us);
  valid.status.state_publish_period_us()=(parameters.state_publish_period_us);
  std::uint8_t contract_hash[32]{};
  std::uint8_t model_hash[32]{};
  std::uint8_t profile_hash[32]{};
  std::uint8_t firmware_hash[32]{};
  std::uint8_t policy_hash[32]{};
  contract_hash[0] = 1U; model_hash[0] = 2U; profile_hash[0] = 3U;
  firmware_hash[0] = 4U; policy_hash[0] = 5U;
  valid.status.contract_sha256().emplace().assign(contract_hash,32);
  valid.status.model_sha256().emplace().assign(model_hash,32);
  valid.status.profile_sha256().emplace().assign(profile_hash,32);
  valid.status.firmware_sha256().emplace().assign(firmware_hash,32);
  valid.status.compiled_policy_sha256().emplace().assign(policy_hash,32);
  const double minimum[6]{-1, -1, -1, -1, -1, -1};
  const double maximum[6]{1, 1, 1, 1, 1, 1};
  const double velocity[6]{1, 1, 1, 1, 1, 1};
  const double acceleration[6]{2, 2, 2, 2, 2, 2};
  valid.status.position_min_rad().emplace().assign(minimum,6);
  valid.status.position_max_rad().emplace().assign(maximum,6);
  valid.status.velocity_max_rad_s().emplace().assign(velocity,6);
  valid.status.acceleration_max_rad_s().emplace().assign(acceleration,6);
  valid.state.state_seq()=(1U);
  const double positions[6]{0.25, six_axis ? 0.2 : NAN, six_axis ? 0.3 : NAN,
                           six_axis ? 0.4 : NAN, six_axis ? 0.5 : NAN, six_axis ? 0.6 : NAN};
  root_fixture::set_position(valid.state,positions);
  if (ticket_mode != 0U) valid.state.freshness_ticket()=(9007199254740996ULL);
  /**
   * @brief Host steady起点和人工Source域5,000,000,000µs偏移。
   * 内部device_now_us返回该偏移加Host经过时间，用大偏移检查不能把Source绝对值直接当Host时间。
   * Core专用peer随后会重写实际Core身份/票据，但该Source时间模拟仍由刺激提供；不是实测设备时钟同步。
   */
  const auto host_clock_anchor_us = transport_steady_now_us();
  const auto device_now_us = [&] {
    return 5000000000ULL + transport_steady_now_us() - host_clock_anchor_us;
  };
  /**
   * @brief 普通模式票据从大于2^53的uint64递增，捕获错误的浮点中转；并非运行时从设备签发的真实票据。
   */
  std::uint64_t synthetic_ticket = 9007199254740997ULL;
  /**
   * @par 内部send回调
   * 按值接收Stimulus，不污染原夹具；Source时间0自动填device_now_us，普通非ticket模式每次发State递增票据。
   * 返回pipe是否恰好写满一个宿主对象；这只确认刺激交给peer，不表示wire发布成功或Host已接纳。
   */
  auto send = [&](Stimulus stimulus) {
    if (manual_release_mode) {
      if ((*stimulus.status.core_state()) == 1U &&
          (*stimulus.status.session_id()) == 0U) {
        stimulus.status.core_state()=(2);
        stimulus.status.reference_epoch()=(9);
      }
      if ((*stimulus.state.core_state()) == 1U &&
          (*stimulus.state.session_id()) == 0U) {
        stimulus.state.core_state()=(2);
        stimulus.state.reference_epoch()=(9);
        root_fixture::set_position_quality(stimulus.state,parameters.configured_mask);
      }
    }
    if ((*stimulus.state.produced_at_us()) == 0U)
      stimulus.state.produced_at_us()=(device_now_us());
    if (ticket_mode == 0U && stimulus.send_state)
      stimulus.state.freshness_ticket()=(++synthetic_ticket);
    return (root_fixture::write_stimulus(output,stimulus));
  };
  /**
   * @par 内部send_rejected_state回调
   * 保存原state_messages/rejected_states，反复send到两个计数都增加才返回true。
   * 它证明至少一条消息进入接收并被拒绝；是否保留旧缓存、是否命中某一具体reason由调用处额外断言。
   */
  auto send_rejected_state = [&](Stimulus stimulus) {
    const auto before = transport.statistics();
    return wait_for([&] {
      if (!send(stimulus)) return false;
      const auto after = transport.statistics();
      return after.state_messages > before.state_messages &&
             after.rejected_states > before.rejected_states;
    });
  };
  JointStateSnapshot snapshot{};
  const auto channel = DeviceTransport::management_channel(parameters);
  if (!channel) return 55;
  /** @brief 按夹具声明的反馈周期持续发布当前样本，直到实际观测命中本步骤。
   * @details publish成功只证明本地接纳；State最新值合并/有界入口允许丢弃。
   * 不重试管理RPC、不推进业务身份，不改wait_for原3s总期限或生产heartbeat预算。 */
  const auto feedback_until = [&](Stimulus stimulus, auto observed) {
    auto next_send = std::chrono::steady_clock::now();
    bool pipe_ok = true;
    const bool reached = wait_for([&] {
      const auto now = std::chrono::steady_clock::now();
      if (now >= next_send) {
        pipe_ok = send(stimulus);
        next_send = now + std::chrono::microseconds(parameters.state_publish_period_us);
        if (!pipe_ok) return true;
      }
      return observed();
    });
    return pipe_ok && reached;
  };
  const auto observed_status_matches = [&](const Status& expected,const dp::comm::SourceId& expected_source) {
    const auto observation = [&] {
      std::lock_guard<std::mutex> lock(channel->mutex);
      return channel->observation;
    }();
    Status seen=owned_fixture::status();dp::comm::SourceId seen_source{};
    return observation.status_received_us != 0U &&
        ((seen=observation.status),(seen_source=observation.status_source),message_fields_present(seen)) == true &&
        seen_source.boot_id == expected_source.boot_id && (*seen.session_id()) == (*expected.session_id()) &&
        (*seen.reference_epoch()) == (*expected.reference_epoch()) && (*seen.motion_epoch()) == (*expected.motion_epoch()) &&
        (*seen.management_epoch()) == (*expected.management_epoch());
  };
  const auto observed_state_matches = [&](const State& expected,const dp::comm::SourceId& expected_source) {
    // 用正常接纳快照；独立diagnostic raw时钟故意保留跨authority水位，不能充当本步骤成功。
    const auto observation = [&] {
      std::lock_guard<std::mutex> lock(channel->mutex);
      return channel->observation;
    }();
    State seen=owned_fixture::state();dp::comm::SourceId seen_source{};
    return observation.state_received_us != 0U &&
        ((seen=observation.state),(seen_source=observation.state_source),message_fields_present(seen)) == true &&
        seen_source.boot_id == expected_source.boot_id && (*seen.session_id()) == (*expected.session_id()) &&
        (*seen.reference_epoch()) == (*expected.reference_epoch()) && (*seen.motion_epoch()) == (*expected.motion_epoch()) &&
        (*seen.management_epoch()) == (*expected.management_epoch()) && (*seen.state_seq()) == (*expected.state_seq());
  };
  /** @brief Root元数据/文件格式负例：用真实Status heartbeat作处理屏障，坏State不得进入业务回调或续state_received_us。
   * @details 正式发送叶层见publish_partial_state；不以旧DTO.boot/schema字段代替Root权威元信息。 */
  const auto reject_root_frame=[&](Stimulus stimulus) {
    auto heartbeat=stimulus;heartbeat.send_state=false;heartbeat.send_status=true;
    const auto drain_before=transport.statistics().status_messages;
    if(!wait_for([&]{return send(heartbeat) && transport.statistics().status_messages>=drain_before+3;}))return false;
    const auto before=transport.statistics();const auto observed=channel->diagnostic_observation().state_received_us;
    stimulus.send_status=true;
    if(!wait_for([&]{return send(stimulus) && transport.statistics().status_messages>=before.status_messages+3;}))return false;
    const auto after=transport.statistics();
    return after.state_callbacks==before.state_callbacks && channel->diagnostic_observation().state_received_us==observed;
  };
  if(root_presence_case) {
    valid.state.hold_ready()=(true);
    if(!wait_for([&]{return send(valid) && system.copy_state(&snapshot) && snapshot.position[0]==0.25;}))return 901;
    auto heartbeat=valid;heartbeat.send_state=false;
    const auto status_before=transport.statistics().status_messages;
    if(!wait_for([&]{return send(heartbeat) && transport.statistics().status_messages>=status_before+3;}))return 902;
    const auto anchor=channel->diagnostic_observation().state_received_us;
    const auto callbacks=transport.statistics().state_callbacks;
    for(unsigned missing=1;missing<=2;++missing) {
      auto partial=valid;partial.omit_state_field=missing;partial.state.state_seq()=(2);
      const auto until=std::chrono::steady_clock::now()+std::chrono::milliseconds(100);
      while(std::chrono::steady_clock::now()<until) {
        if(!send(partial))return 903;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
      const auto observation=channel->diagnostic_observation();
      if(observation.state_received_us!=anchor || transport.statistics().state_callbacks!=callbacks ||
         !system.copy_state(&snapshot) || snapshot.position[0]!=0.25)return 904;
      State retained=owned_fixture::state();dp::comm::SourceId retained_source{};
      if(((retained=observation.state),(retained_source=observation.state_source),message_fields_present(retained))!=true || !(*retained.hold_ready()))return 905;
    }
    if(transport.statistics().status_messages<=status_before+3)return 906;
    double zero[6]{};root_fixture::set_position(valid.state,zero);valid.state.hold_ready()=(false);valid.state.state_seq()=(3);
    if(!wait_for([&]{
      if(!send(valid) || !system.copy_state(&snapshot) || snapshot.position[0]!=0)return false;
      const auto observation=channel->diagnostic_observation();State current=owned_fixture::state();dp::comm::SourceId current_source{};
      return observation.state_received_us>anchor && ((current=observation.state),(current_source=observation.state_source),message_fields_present(current))==true && !(*current.hold_ready()) && (*current.state_seq())==3;
    }))return 907;
    transport.stop();return 0;
  }
  std::shared_ptr<DeviceManagementRequest> captured_release;
  std::shared_ptr<DeviceManagementRequest> captured_query;
  std::uint64_t early_idle_received_upper=0U, early_idle_received_lower=0U, early_idle_callback_count=0U;
  /**
   * @brief 测试显式配置六轴保持容差0.003rad；用于区分同值/小偏差被抑制和0.02rad越界目标。
   * 这是本地准入容差，不代表实际电机到位精度。
   */
  for(auto &value:channel->post_complete_hold_tolerance_rad)
    value.store(0.003,std::memory_order_release);
  /**
   * @brief 先只发State而无Status绑定；必须计拒绝且copy_state仍不可用，防止单条State自建授权。
   */
  if (released_child_case) {
    if (!wait_for([&] {
          send(valid);
          std::lock_guard<std::mutex> lock(channel->mutex);
          return channel->observation.state_received_us != 0U;
        }))
      return 870;
    auto released = valid;
    released.status.session_id()=(0);
    released.state.session_id()=(0);
    released.status.run_id()=((*valid.status.run_id()) + 1);
    released.status.parent_run_id()=((*valid.status.run_id()));
    released.status.attempt_kind()=(3);
    released.status.motion_epoch()=(11);
    released.state.motion_epoch()=(11);
    released.status.management_epoch()=(2);
    released.state.management_epoch()=(2);
    released.status.final_state()=(8);
    released.status.release_committed()=(true);
    released.status.cleanup_state()=(3);
    released.state.state_seq()=(2);
    std::uint8_t chain[32]{1};
    released.status.evidence_chain_sha256().emplace().assign(chain,32);
    if (released_child_case == 2U)
      released.status.parent_run_id()=(99);
    if (released_child_case == 3U)
      released.status.release_committed()=(false);
    if (released_child_case == 4U) {
      released.status.reference_epoch()=(10);
      released.state.reference_epoch()=(10);
    }
    if (released_child_case == 5U) {
      released.status.motion_epoch()=(12);
      released.state.motion_epoch()=(12);
    }
    if (released_child_case == 6U) {
      released.status.management_epoch()=(3);
      released.state.management_epoch()=(3);
    }
    if (released_child_case == 1U) {
      if (!wait_for([&] {
            send(released);
            std::lock_guard<std::mutex> lock(channel->mutex);
            Status observed=owned_fixture::status();dp::comm::SourceId observed_source{};
            State observed_state=owned_fixture::state();dp::comm::SourceId observed_state_source{};
            return ((observed=channel->observation.status),(observed_source=channel->observation.status_source),message_fields_present(observed)) == true &&
                   ((observed_state=channel->observation.state),(observed_state_source=channel->observation.state_source),message_fields_present(observed_state)) == true &&
                   (*observed.session_id()) == 0U &&
                   (*observed.run_id()) == (*released.status.run_id()) &&
                   (*observed_state.session_id()) == 0U;
          }))
        return 871;
      if (transport.statistics().raw_evidence_failure != 0U ||
          system.copy_state(&snapshot))
        return 872;
    } else {
      if (!wait_for([&] {
            send(released);
            return transport.statistics().raw_evidence_failure != 0U;
          }))
        return 873;
      std::lock_guard<std::mutex> lock(channel->mutex);
      Status observed=owned_fixture::status();dp::comm::SourceId observed_source{};
      if (((observed=channel->observation.status),(observed_source=channel->observation.status_source),message_fields_present(observed)) != true ||
          (*observed.session_id()) != 8U)
        return 874;
    }
    transport.stop();
    return 0;
  }
  if (manual_fault_diagnostic_mode) {
    auto status_only = valid;
    status_only.send_state = false;
    if (!wait_for([&] {
          send(status_only);
          return channel->diagnostic_observation().status_received_us != 0U;
        }))
      return 850;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    auto state_only = valid;
    state_only.send_status = false;
    if (!send(state_only) || !wait_for([&] {
          return channel->diagnostic_observation().state_received_us != 0U;
        }))
      return 850;
    auto diagnostic = channel->diagnostic_observation();
    Status observed_status=owned_fixture::status();dp::comm::SourceId observed_status_source{};
    State observed_state=owned_fixture::state();dp::comm::SourceId observed_state_source{};
    if (((observed_status=diagnostic.status),(observed_status_source=diagnostic.status_source),message_fields_present(observed_status)) != true ||
        ((observed_state=diagnostic.state),(observed_state_source=diagnostic.state_source),message_fields_present(observed_state)) != true ||
        (*observed_status.core_state()) != 9U ||
        (*observed_state.core_state()) != 9U ||
        (*observed_state.reference_epoch()) != 1U ||
        root_fixture::position_quality(observed_state) != 63U ||
        system.copy_state(&snapshot) ||
        channel->observation.state_received_us != 0U ||
        channel->raw_cleanup_observation().state_received_us != 0U)
      return 851;
    const auto source_stamp = diagnostic.state_received_us;
    auto repeated = valid;
    repeated.send_status = false;
    if (!send(repeated))
      return 852;
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    if (channel->diagnostic_observation().state_received_us != source_stamp)
      return 853;
    auto next = valid;
    next.send_status = false;
    next.state.state_seq()=((*valid.state.state_seq()) + 1U);
    if (!send(next) || !wait_for([&] {
          return channel->diagnostic_observation().state_received_us >
                 source_stamp;
        }))
      return 854;
    diagnostic = channel->diagnostic_observation();
    auto wrong = valid;
    wrong.send_state = false;
    std::uint8_t hash[32]{99};
    wrong.status.model_sha256().emplace().assign(hash,32);
    if (!send(wrong))
      return 855;
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    if (channel->diagnostic_observation().status_received_us !=
        diagnostic.status_received_us)
      return 856;
    transport.stop();
    return 0;
  }
  auto unbound = valid;
  unbound.send_status = false;
  if (!wait_for([&] { send(unbound); return transport.statistics().rejected_states != 0U; }) ||
      system.copy_state(&snapshot)) {
    const auto stats=transport.statistics();
    std::fprintf(stderr,"ROOT_UNBOUND_FAILED status=%llu state=%llu callbacks=%llu rejected=%llu\n",
      (unsigned long long)stats.status_messages,(unsigned long long)stats.state_messages,
      (unsigned long long)stats.state_callbacks,(unsigned long long)stats.rejected_states);
    return 48;
  }
  if (manual_recovered_idle_mode && manual_recovered_idle_fault) {
    auto bad = valid;
    if (manual_recovered_idle_fault == 1U)
      bad.status.first_failure_reason()=(0x1067);
    if (manual_recovered_idle_fault == 2U) {
      bad.status.core_state()=(9);
      bad.state.core_state()=(9);
    }
    if (manual_recovered_idle_fault == 3U)
      bad.status.failure_source()=(0x1068);
    if (manual_recovered_idle_fault == 4U)
      bad.status.cleanup_state()=(2);
    if (manual_recovered_idle_fault == 6U)
      bad.status.release_committed()=(false);
    if (manual_recovered_idle_fault == 7U)
      bad.status.attempt_kind()=(2);
    if (manual_recovered_idle_fault == 5U) {
      if (!wait_for([&] {
            send(valid);
            std::lock_guard<std::mutex> lock(channel->mutex);
            return channel->observation.state_received_us != 0U;
          }))
        return 860;
      bad.status.reference_epoch()=(2);
      bad.state.reference_epoch()=(2);
      bad.state.state_seq()=(2);
    }
    const auto rejected_before = transport.statistics().rejected_states;
    if (!wait_for([&] {
          send(bad);
          const auto stats = transport.statistics();
          return manual_recovered_idle_fault == 5U
                     ? stats.rejected_states > rejected_before
                     : stats.raw_evidence_failure != 0U;
        }))
      return 861;
    {
      std::lock_guard<std::mutex> lock(channel->mutex);
      if (manual_recovered_idle_fault != 5U &&
          channel->observation.state_received_us != 0U)
        return 862;
      if (manual_recovered_idle_fault == 5U) {
        State state=owned_fixture::state();dp::comm::SourceId state_source{};
        Status status=owned_fixture::status();dp::comm::SourceId status_source{};
        if (((state=channel->observation.state),(state_source=channel->observation.state_source),message_fields_present(state)) != true ||
            ((status=channel->observation.status),(status_source=channel->observation.status_source),message_fields_present(status)) != true ||
            (*state.reference_epoch()) != 1U || (*status.reference_epoch()) != 1U)
          return 865;
      }
    }
    if (system.copy_state(&snapshot))
      return 863;
    transport.stop();
    return 0;
  }
  if (release_deactivate_mode==4U && source_cancel_coordination) { valid.status.run_id()=(9U); valid.status.attempt_id()=(9U); valid.status.attempt_kind()=(1U); }
  /**
   * @brief 显式释放/生命周期子场景的独立入口，完成后直接return不跑后续普通运动流程。
   * release80默认Pending、Query81指向80；少数模式直接Committed或完全无gate reply。编号细节见main表。
   */
  if (release_deactivate_mode != 0U) {
    if (!wait_for([&] { send(valid); return system.copy_state(&snapshot); })) return 230;
    if (release_deactivate_mode != 4U) {
      Management::Request release=owned_fixture::request();
      (void)0;
      (void)0;
      release.domain_device_id().emplace().assign(device,16);
      std::uint8_t client[16]{2U};
      release.client_instance_id().emplace().assign(client,16);
      release.request_id()=(80U);
      release.request_seq()=(80U);
      release.operation()=(10U);
      release.expected_boot_epoch()=(7U);
      release.expected_session_id()=(8U);
      release.expected_reference_epoch()=(9U);
      release.expected_motion_epoch()=(10U);
      release.expected_management_epoch()=(1U);
      release.expected_profile_id()=(1U);
      release.configured_mask()=(parameters.configured_mask);
      if (release_deactivate_mode != 5U) {
        auto explicit_release = std::make_shared<DeviceManagementRequest>();
        if (((explicit_release->request=release),message_fields_present(release)) != true) return 231;
        {
          std::lock_guard<std::mutex> lock(channel->mutex);
          channel->queued = explicit_release;
        }
        if (!wait_for([&] { return explicit_release->complete.load(); })) return 232;
        captured_release = explicit_release;
        Management::Response release_receipt=owned_fixture::response();dp::comm::SourceId release_receipt_source{};
        if (((release_receipt=explicit_release->response),(release_receipt_source=explicit_release->response_source),message_fields_present(release_receipt)) != true ||
            (*release_receipt.transaction_phase()) != (release_deactivate_mode == 7U ? 3U : 1U))
          return 233;
        // GUI49：普通Gate D10直接COMMITTED也必须归档原根，不能只等后续worker Query。
        if(release_deactivate_mode==7U && !wait_for([&] {
          const auto root=channel->worker_release_commit_snapshot();
          return root.request==80U && root.operation==80U && root.run==2U &&
            root.boot==7U && root.session==8U && root.reference==9U && root.generation!=0U;
        }))return 388;
        /**
         * @brief 先缓存Stopping7 Status与同op42的早到Idle State，但没有强COMMITTED时不能替换已接纳State。
         * 先给六个错误State（Pending/op0/错op/boot/motion/management）分别验证拒绝；合法早Idle也仅为待后续回执解锁的候选。
         */
        if (release_deactivate_mode >= 11U && release_deactivate_mode <= 18U) {
          auto stopping=valid; stopping.send_state=false;
          dummy_ref::JointSnapshot stopping_snapshot{};
          stopping_snapshot.lifecycle=dummy_ref::LifecycleState::kStopping;
          stopping.status.core_state()=(dummy_ref::comm::lifecycle_to_wire(stopping_snapshot));
          stopping.status.motion_epoch()=(11U); stopping.status.management_epoch()=(2U);
          stopping.status.operation_id()=(42U); stopping.status.transaction_phase()=(2U);
          stopping.status.run_id()=(2U);
          if(!send(stopping) || !wait_for([&] {
                std::lock_guard<std::mutex> lock(channel->mutex); Status observed=owned_fixture::status();dp::comm::SourceId observed_source{};
                return ((observed=channel->observation.status),(observed_source=channel->observation.status_source),message_fields_present(observed))==true &&
                    (*observed.core_state())==7U && (*observed.operation_id())==42U;
              })) return 375;
          /**
           * @brief 组合陈旧候选负例：新epoch先校准，再连续发新鲜State避免心跳先Stop掩盖候选年龄。
           * 保留早Idle的原Source产生时刻，不因后来Query回执重新盖接收时间戳来变新鲜。
           */
          if(release_deactivate_mode==17U) {
            // 新Status epoch会清时钟水位；先接纳真实当前epoch的old-owner State重新校准。
            auto anchor=valid; anchor.send_status=false;
            anchor.state.core_state()=(7U); anchor.state.motion_epoch()=(11U);
            anchor.state.management_epoch()=(2U); anchor.state.operation_id()=(42U);
            anchor.state.transaction_phase()=(2U); anchor.state.state_seq()=(2U);
            if(!send(anchor) || !wait_for([&] {
                  std::lock_guard<std::mutex> lock(channel->mutex); State observed=owned_fixture::state();dp::comm::SourceId observed_source{};
                  return ((observed=channel->observation.state),(observed_source=channel->observation.state_source),message_fields_present(observed))==true &&
                      (*observed.state_seq())==2U && (*observed.session_id())==8U && (*observed.motion_epoch())==11U;
                })) return 384;
            // 旧payload仍晚于首校准anchor；持续fresh State避免heartbeat先Stop掩盖年龄守卫。
            for(std::uint32_t seq=3U;seq<=13U;++seq) {
              std::this_thread::sleep_for(std::chrono::milliseconds(20));
              anchor.state.state_seq()=(seq); anchor.state.produced_at_us()=(device_now_us());
              if(!send(anchor) || !wait_for([&] {
                    std::lock_guard<std::mutex> lock(channel->mutex); State observed=owned_fixture::state();dp::comm::SourceId observed_source{};
                    return ((observed=channel->observation.state),(observed_source=channel->observation.state_source),message_fields_present(observed))==true &&
                        (*observed.state_seq())==seq;
                  })) return 385;
            }
            const auto before_refresh=transport.statistics().status_messages;
            if(!send(stopping) || !wait_for([&] {
                  return transport.statistics().status_messages>before_refresh;
                })) return 386;
          }
          auto early=valid; early.send_status=false;
          early.state.core_state()=(1U); early.state.session_id()=(0U);
          early.state.reference_epoch()=(0U); early.state.motion_epoch()=(11U);
          early.state.management_epoch()=(2U); early.state.operation_id()=(42U);
          early.state.transaction_phase()=(3U); early.state.state_seq()=(2U);
          root_fixture::set_position_quality(early.state,0U); root_fixture::set_raw_quality(early.state,parameters.configured_mask);
          const double raw[6]{.1,.2,.3,.4,.5,.6}; root_fixture::set_raw_position(early.state,raw);
          for(unsigned fault=0;fault<6;++fault) {
            auto wrong=early;
            if(fault==0) wrong.state.transaction_phase()=(2U);
            if(fault==1) wrong.state.operation_id()=(0U);
            if(fault==2) wrong.state.operation_id()=(43U);
            if(fault==3) wrong.state_source.boot_id=(8U);
            if(fault==4) wrong.state.motion_epoch()=(12U);
            if(fault==5) wrong.state.management_epoch()=(3U);
            if(!send_rejected_state(wrong)) return 376;
          }
          if(release_deactivate_mode==17U) {
            early.state.state_seq()=(20U);
            early.state.produced_at_us()=(device_now_us()-200000U);
          }
          const auto before_early=transport.statistics();
          early_idle_received_lower=transport_steady_now_us();
          if(!send(early) || !wait_for([&] {
                return transport.statistics().rejected_states>before_early.rejected_states;
              })) return 377;
          early_idle_received_upper=transport_steady_now_us();
          early_idle_callback_count=transport.statistics().state_callbacks;
          if(release_deactivate_mode==17U) std::this_thread::sleep_for(std::chrono::milliseconds(100));
          // 未有COMMITTED时不更新已接纳State。下一真实Query解锁同op缓存。
        }
        /**
         * @brief 只有正式Disabling8且权限/permit已清时，才允许同tuple临时Idle反馈续鲜。
         * Stopping7不能冒充Disabling8；仍有permit、错boot/motion/management的Idle也不能走此边界例外。
         */
        if (release_deactivate_mode == 9U) {
          auto disabling = valid;
          disabling.send_state = false;
          disabling.status.core_state()=(ref_disabling_wire_state());
          disabling.status.motion_epoch()=(11U);
          disabling.status.management_epoch()=(2U);
          const auto statuses_before = transport.statistics().status_messages;
          if (!send(disabling) || !wait_for([&] {
                return transport.statistics().status_messages > statuses_before;
              })) return 249;
          auto early_idle = valid;
          early_idle.send_status = false;
          early_idle.state.core_state()=(1U);
          early_idle.state.session_id()=(0U);
          early_idle.state.reference_epoch()=(0U);
          early_idle.state.motion_epoch()=(11U);
          early_idle.state.management_epoch()=(2U);
          root_fixture::set_position_quality(early_idle.state,0U);
          root_fixture::set_raw_quality(early_idle.state,parameters.configured_mask);
          const double raw[6]{0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
          root_fixture::set_raw_position(early_idle.state,raw);
          // 正式Stopping7不能冒充Disabling8，其他tuple相同仍拒临时Idle。
          auto stopping = disabling;
          dummy_ref::JointSnapshot stopping_snapshot{};
          stopping_snapshot.lifecycle=dummy_ref::LifecycleState::kStopping;
          stopping.status.core_state()=(dummy_ref::comm::lifecycle_to_wire(stopping_snapshot));
          if(!send(stopping) || !wait_for([&] {
                std::lock_guard<std::mutex> lock(channel->mutex);
                Status observed=owned_fixture::status();dp::comm::SourceId observed_source{};
                return ((observed=channel->observation.status),(observed_source=channel->observation.status_source),message_fields_present(observed))==true &&
                    (*observed.core_state())==dummy_ref::comm::lifecycle_to_wire(stopping_snapshot);
              }) || !send_rejected_state(early_idle)) return 371;
          auto permitted = disabling;
          permitted.status.real_motion_permitted()=(true);
          permitted.status.permit_remaining_us()=(1000U);
          if(!send(permitted) || !wait_for([&] {
                std::lock_guard<std::mutex> lock(channel->mutex);
                Status observed=owned_fixture::status();dp::comm::SourceId observed_source{};
                return ((observed=channel->observation.status),(observed_source=channel->observation.status_source),message_fields_present(observed))==true &&
                    (*observed.permit_remaining_us())==1000U;
              }) || !send_rejected_state(early_idle)) return 372;
          if(!send(disabling) || !wait_for([&] {
                std::lock_guard<std::mutex> lock(channel->mutex);
                Status observed=owned_fixture::status();dp::comm::SourceId observed_source{};
                return ((observed=channel->observation.status),(observed_source=channel->observation.status_source),message_fields_present(observed))==true &&
                    (*observed.core_state())==ref_disabling_wire_state() && (*observed.permit_remaining_us())==0U;
              })) return 373;
          auto wrong_management = early_idle;
          wrong_management.state.management_epoch()=(3U);
          if(!send_rejected_state(wrong_management)) return 374;
          auto wrong_boot = early_idle;
          wrong_boot.state_source.boot_id=(8U);
          if (!send_rejected_state(wrong_boot)) return 250;
          auto wrong_epoch = early_idle;
          wrong_epoch.state.motion_epoch()=(12U);
          if (!send_rejected_state(wrong_epoch)) return 251;
          for (std::uint32_t seq = 2U; seq <= 16U; ++seq) {
            early_idle.state.state_seq()=(seq);
            if (!wait_for([&] {
                  send(early_idle);
                  std::lock_guard<std::mutex> lock(channel->mutex);
                  State observed=owned_fixture::state();dp::comm::SourceId observed_source{};
                  return ((observed=channel->observation.state),(observed_source=channel->observation.state_source),message_fields_present(observed)) == true &&
                      (*observed.state_seq()) == seq;
                })) return 249;
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
          }
        }
        if (release_deactivate_mode != 7U) {
          release.request_id()=(81U);
          release.request_seq()=(81U);
          release.operation()=(12U);
          release.queried_operation_id()=(80U);
          auto query = std::make_shared<DeviceManagementRequest>();
          if (((query->request=release),message_fields_present(release)) != true) return 237;
          {
            std::lock_guard<std::mutex> lock(channel->mutex);
            channel->queued = query;
          }
          if (!wait_for([&] { return query->complete.load(); })) return 238;
          captured_query = query;
          /**
           * @brief complete原子标志可先于worker应用reply；再观察50ms窗口，确保错/待定/未Committed回执不会稍后偷偷采用早Idle。
           * mode12的强回执op43仍可允许后续全新op43 Idle，不能因为旧缓存op42不匹配而永久拒绝正确反馈。
           */
          if(release_deactivate_mode>=12U && release_deactivate_mode<=17U) {
            // callback置complete先于worker应用reply；覆盖随后处理窗口，避免即时false假PASS。
            const auto check_until=transport_steady_now_us()+50000U;
            bool adopted=false;
            do {
              {
                std::lock_guard<std::mutex> lock(channel->mutex); State observed=owned_fixture::state();dp::comm::SourceId observed_source{};
                adopted=((observed=channel->observation.state),(observed_source=channel->observation.state_source),message_fields_present(observed))==true && (*observed.session_id())==0U;
              }
              if(adopted) break;
              std::this_thread::sleep_for(std::chrono::milliseconds(2));
            } while(transport_steady_now_us()<check_until);
            if(adopted) return 379;
            if(release_deactivate_mode==12U) {
              // 强receipt本身有效；旧缓存op42失配不能否决op43的后续合法Idle。
              auto final_idle=valid;
              final_idle.status.core_state()=(1U); final_idle.status.session_id()=(0U);
              final_idle.status.reference_epoch()=(0U); final_idle.status.motion_epoch()=(11U);
              final_idle.status.management_epoch()=(2U); final_idle.status.run_id()=(2U);
              final_idle.status.operation_id()=(43U); final_idle.status.transaction_phase()=(3U);
              final_idle.status.release_committed()=(true); final_idle.status.permit_remaining_us()=(0U);
              final_idle.state.core_state()=(1U); final_idle.state.session_id()=(0U);
              final_idle.state.reference_epoch()=(0U); final_idle.state.motion_epoch()=(11U);
              final_idle.state.management_epoch()=(2U); final_idle.state.operation_id()=(43U);
              final_idle.state.transaction_phase()=(3U); final_idle.state.state_seq()=(3U);
              root_fixture::set_position_quality(final_idle.state,0U); root_fixture::set_raw_quality(final_idle.state,parameters.configured_mask);
              const double raw[6]{.1,.2,.3,.4,.5,.6}; root_fixture::set_raw_position(final_idle.state,raw);
              if(!send(final_idle) || !wait_for([&] {
                    std::lock_guard<std::mutex> lock(channel->mutex); State state=owned_fixture::state();dp::comm::SourceId state_source{}; Status status=owned_fixture::status();dp::comm::SourceId status_source{};
                    return ((state=channel->observation.state),(state_source=channel->observation.state_source),message_fields_present(state))==true &&
                        ((status=channel->observation.status),(status_source=channel->observation.status_source),message_fields_present(status))==true &&
                        (*state.state_seq())==3U && (*state.session_id())==0U && (*status.session_id())==0U && (*status.operation_id())==43U;
                  })) return 382;
              if(transport.statistics().stop_requests_sent!=0U) return 383;
            }
            return 0;
          }
          Management::Response release_reply=owned_fixture::response();dp::comm::SourceId release_reply_source{};
          if (((release_reply=query->response),(release_reply_source=query->response_source),message_fields_present(release_reply)) != true ||
              !(*release_reply.release_committed()) || (*release_reply.transaction_phase()) != 3U)
            return 239;
          /**
           * @brief 强回执后采用已收到的同op早Idle，要求接收Host时间仍落在首次发送窗口。
           * state_callbacks计数不再增加，证明使用了原候选，不是靠重发一条新State掩盖缓存解锁问题。
           */
          if((release_deactivate_mode==11U || release_deactivate_mode==18U) && !wait_for([&] {
                std::lock_guard<std::mutex> lock(channel->mutex); State observed=owned_fixture::state();dp::comm::SourceId observed_source{};
                return ((observed=channel->observation.state),(observed_source=channel->observation.state_source),message_fields_present(observed))==true &&
                    (*observed.state_seq())==2U && (*observed.session_id())==0U && (*observed.operation_id())==42U &&
                    channel->observation.state_received_us>=early_idle_received_lower &&
                    channel->observation.state_received_us<=early_idle_received_upper;
              })) return 378;
          if((release_deactivate_mode==11U || release_deactivate_mode==18U) && transport.statistics().state_callbacks!=early_idle_callback_count)
            return 380;
          /**
           * @brief COMMITTED已锁定后让Stopping Status故意落后约500ms，但持续20ms提供当前Idle State。
           * 要求不新增heartbeat/Stop，区分合法状态传播落后与真实State断流。
           */
          if(release_deactivate_mode==18U) {
            // 真COMMITTED已锁定；继续20ms新鲜Idle State，旧Stopping Status故意保持500ms。
            auto idle=valid; idle.send_status=false;
            idle.state.core_state()=(1U); idle.state.session_id()=(0U); idle.state.reference_epoch()=(0U);
            idle.state.motion_epoch()=(11U); idle.state.management_epoch()=(2U);
            idle.state.operation_id()=(42U); idle.state.transaction_phase()=(3U);
            root_fixture::set_position_quality(idle.state,0U); root_fixture::set_raw_quality(idle.state,parameters.configured_mask);
            const double raw[6]{.1,.2,.3,.4,.5,.6}; root_fixture::set_raw_position(idle.state,raw);
            for(std::uint32_t seq=3U;seq<=28U;++seq) {
              std::this_thread::sleep_for(std::chrono::milliseconds(20));
              idle.state.state_seq()=(seq); idle.state.produced_at_us()=(device_now_us());
              if(!send(idle) || !wait_for([&] {
                    std::lock_guard<std::mutex> lock(channel->mutex); State observed=owned_fixture::state();dp::comm::SourceId observed_source{};
                    return ((observed=channel->observation.state),(observed_source=channel->observation.state_source),message_fields_present(observed))==true && (*observed.state_seq())==seq;
                  })) return 387;
              if(transport.statistics().heartbeat_stops!=0U || transport.statistics().stop_requests_sent!=0U) return 388;
            }
          }
        }
      }
      /**
       * @brief 缺最终Idle的释放不能永久豁免心跳；要求state_heartbeat_gap_stops出现后结束。
       * 此断言只检查停机触发，不检查远端物理停止。
       */
      if (release_deactivate_mode == 10U) {
        if (!wait_for([&] {
              return transport.statistics().state_heartbeat_gap_stops != 0U;
            })) return 252;
        transport.stop();
        return 0;
      }
      auto released = valid;
      released.status.core_state()=(1U);
      released.state.core_state()=(1U);
      released.status.session_id()=(0U);
      released.state.session_id()=(0U);
      released.status.reference_epoch()=(0U);
      released.state.reference_epoch()=(0U);
      released.status.motion_epoch()=(11U);
      released.state.motion_epoch()=(11U);
      released.status.management_epoch()=(2U);
      released.state.management_epoch()=(2U);
      released.status.release_committed()=(release_deactivate_mode != 2U);
      released.status.permit_remaining_us()=(0U);
      root_fixture::set_position_quality(released.state,0U);
      root_fixture::set_raw_quality(released.state,parameters.configured_mask);
      const double idle_raw[6]{0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
      root_fixture::set_raw_position(released.state,idle_raw);
      released.state.state_seq()=(2U);
      if(release_deactivate_mode==11U || release_deactivate_mode==18U) {
        released.state.state_seq()=(release_deactivate_mode==18U ? 29U : 3U); released.state.operation_id()=(42U);
        released.state.transaction_phase()=(3U);
        released.status.run_id()=(2U); released.status.operation_id()=(42U);
        released.status.transaction_phase()=(3U);
      }
      if (release_deactivate_mode == 9U) released.state.state_seq()=(17U);
      /**
       * @brief 已有强release结果但旧owner Disabling Status落后时，持续提供当前Idle State。
       * 29个20ms新State必须不新增Stop；最终再补Status，不把Status先到当作唯一合法顺序。
       */
      if (release_deactivate_mode == 6U) {
        auto old_owner_status = valid;
        old_owner_status.send_state = false;
        old_owner_status.status.core_state()=(ref_disabling_wire_state());
        old_owner_status.status.motion_epoch()=(11U);
        old_owner_status.status.management_epoch()=(2U);
        const auto before_status = transport.statistics().status_messages;
        if (!send(old_owner_status) || !wait_for([&] {
              return transport.statistics().status_messages > before_status;
            })) return 240;
        auto early_idle_state = released;
        early_idle_state.send_status = false;
        if (!wait_for([&] {
              send(early_idle_state);
              std::lock_guard<std::mutex> lock(channel->mutex);
              State observed=owned_fixture::state();dp::comm::SourceId observed_source{};
              return channel->observation.state_received_us != 0U &&
                  ((observed=channel->observation.state),(observed_source=channel->observation.state_source),message_fields_present(observed)) == true &&
                  (*observed.state_seq()) == 2U;
            })) return 241;
        for (std::uint32_t seq = 3U; seq <= 31U; ++seq) {
          early_idle_state.state.state_seq()=(seq);
          if (!wait_for([&] {
                send(early_idle_state);
                std::lock_guard<std::mutex> lock(channel->mutex);
                State observed=owned_fixture::state();dp::comm::SourceId observed_source{};
                return ((observed=channel->observation.state),(observed_source=channel->observation.state_source),message_fields_present(observed)) == true &&
                    (*observed.state_seq()) == seq;
              })) return 242;
          std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        if (transport.statistics().heartbeat_stops != 0U ||
            transport.statistics().stop_requests_sent != 0U) return 243;
        released.state.state_seq()=(32U);
      }
      if (!wait_for([&] {
            send(released);
            std::lock_guard<std::mutex> lock(channel->mutex);
            State observed=owned_fixture::state();dp::comm::SourceId observed_source{};
            return channel->observation.state_received_us != 0U &&
                ((observed=channel->observation.state),(observed_source=channel->observation.state_source),message_fields_present(observed)) == true &&
                (*observed.state_seq()) == (*released.state.state_seq());
          })) return 234;
      if((release_deactivate_mode==11U || release_deactivate_mode==18U) && !wait_for([&] {
            std::lock_guard<std::mutex> lock(channel->mutex); Status observed=owned_fixture::status();dp::comm::SourceId observed_source{};
            return ((observed=channel->observation.status),(observed_source=channel->observation.status_source),message_fields_present(observed))==true &&
                (*observed.core_state())==1U && (*observed.session_id())==0U && (*observed.run_id())==2U &&
                (*observed.operation_id())==42U && (*observed.release_committed());
          })) return 381;
      if (release_deactivate_mode == 8U || release_deactivate_mode == 9U) {
        std::this_thread::sleep_for(std::chrono::milliseconds(310));
        const auto stats = transport.statistics();
        if (stats.state_heartbeat_gap_stops != 0U || stats.stop_requests_sent != 0U ||
            stats.unconfirmed_stop_requests != 0U) return 248;
      }
    }
    if (release_deactivate_mode==4U && source_cancel_coordination) {
      Management::Request cancel=owned_fixture::request();
      (void)0;(void)0;
      cancel.domain_device_id().emplace().assign(device,16);std::uint8_t cancel_client[16]{2U};cancel.client_instance_id().emplace().assign(cancel_client,16);
      cancel.request_id()=(4693763926145815138ULL);cancel.request_seq()=(91U);cancel.operation()=(9U);
      cancel.expected_boot_epoch()=(7U);cancel.expected_session_id()=(8U);cancel.expected_reference_epoch()=(9U);
      cancel.expected_motion_epoch()=(10U);cancel.expected_management_epoch()=(1U);cancel.expected_profile_id()=(1U);
      cancel.configured_mask()=(63U);
      auto pending_cancel=std::make_shared<DeviceManagementRequest>();
      if(((pending_cancel->request=cancel),message_fields_present(cancel))!=true)return 801;
      {std::lock_guard<std::mutex>lock(channel->mutex);channel->queued=pending_cancel;}
      pollfd overlap_event{stops_input,POLLIN,0};Management::Request actual_cancel=owned_fixture::request();
      if(::poll(&overlap_event,1,3000)!=1 || (root_fixture::read_value(stops_input,actual_cancel)==false)
          ||(*actual_cancel.operation())!=9U ||(*actual_cancel.request_id())!=(*cancel.request_id()))return 802;
      // 真正System生命周期callback，Worker自身取StopKind、组实际Disable/Query并处理peer回调。
      system.on_deactivate(rclcpp_lifecycle::State());
      Management::Request query=cancel;query.request_id()=(4693763926145815139ULL);query.request_seq()=(92U);
      query.operation()=(12U);query.queried_operation_id()=((*cancel.request_id()));
      if(source_cancel_fault && !std::strcmp(source_cancel_fault,"foreign_query"))query.queried_operation_id()=((*cancel.request_id())+100U);
      auto pending_query=std::make_shared<DeviceManagementRequest>();
      if(((pending_query->request=query),message_fields_present(query))!=true)return 807;
      bool query_enqueued=false;
      if(!wait_for([&]{if(query_enqueued)return true;std::lock_guard<std::mutex>lock(channel->mutex);if(channel->queued)return false;channel->queued=pending_query;query_enqueued=true;return true;}))return 808;
      auto held=valid;held.status.core_state()=(4U);held.status.motion_epoch()=(11U);held.status.management_epoch()=(2U);
      held.status.operation_id()=(9009U);held.status.transaction_phase()=(3U);held.status.final_state()=(7U);
      held.status.attempt_kind()=(1U);held.status.run_id()=(9U);held.status.attempt_id()=(9U);
      held.state.core_state()=(4U);held.state.hold_ready()=(true);
      if(source_cancel_fault && (!std::strcmp(source_cancel_fault,"nonhold") || !std::strcmp(source_cancel_fault,"second_stop"))){held.state.core_state()=(5U);held.state.hold_ready()=(false);}held.state.motion_epoch()=(11U);held.state.management_epoch()=(2U);
      held.state.state_seq()=(2U);held.state.produced_at_us()=(device_now_us());
      // 原borrowed Status/State真实callback，而非手写authority成员。
      if(!send(held))return 809;
      Management::Request actual_disable=owned_fixture::request();
      const auto cancel_wait_started=std::chrono::steady_clock::now();
      const auto cancel_wait_until=cancel_wait_started+std::chrono::milliseconds(source_cancel_fault?300:200);
      bool second_stop_requested=false;

      while(::poll(&overlap_event,1,0)!=1 && std::chrono::steady_clock::now()<cancel_wait_until) {
        held.state.state_seq()=((*held.state.state_seq())+1U);held.state.produced_at_us()=(device_now_us());
        if(!send(held))return 809;
        if(source_cancel_fault && !std::strcmp(source_cancel_fault,"second_stop")) {
          const auto elapsed=std::chrono::steady_clock::now()-cancel_wait_started;
          if(!second_stop_requested && elapsed>=std::chrono::milliseconds(40)) {system.request_transport_stop();second_stop_requested=true;}
          if(transport.statistics().unconfirmed_stop_requests==1U) {
            if(!second_stop_requested || elapsed>std::chrono::milliseconds(100))return 814;
            break;
          }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
      if(source_cancel_fault) {
        const auto stats=transport.statistics();transport.stop();
        if(::poll(&overlap_event,1,0)==1 ||stats.stop_requests_sent!=0U ||stats.stop_requests_confirmed!=0U ||stats.unconfirmed_stop_requests!=1U)return 813;
        std::fprintf(stderr,"GREEN_NEGATIVE_SOURCE noBlindDisable reason=%s unconfirmed=1 original250k\n",source_cancel_fault);return 0;
      }
      if(::poll(&overlap_event,1,0)!=1 ||(root_fixture::read_value(stops_input,actual_disable)==false)
         ||(*actual_disable.operation())!=10U)return 803;
      std::fprintf(stderr,"OVERLAP_ACTUAL_TX cancel_motion=%u cancel_mgmt=%u disable_motion=%u disable_mgmt=%u\n",
          (*actual_cancel.expected_motion_epoch()),(*actual_cancel.expected_management_epoch()),
          (*actual_disable.expected_motion_epoch()),(*actual_disable.expected_management_epoch()));
      if((*actual_disable.expected_motion_epoch())!=11U ||(*actual_disable.expected_management_epoch())!=2U)return 805;
      Management::Request actual_query=owned_fixture::request();
      if(::poll(&overlap_event,1,3000)!=1 ||(root_fixture::read_value(stops_input,actual_query)==false)||(*actual_query.operation())!=12U)return 810;
      auto idle=held;idle.status.session_id()=(0U);idle.status.reference_epoch()=(0U);idle.status.motion_epoch()=(12U);idle.status.management_epoch()=(3U);
      idle.status.core_state()=(1U);idle.status.release_committed()=(true);idle.status.operation_id()=(900U);idle.status.transaction_phase()=(3U);
      idle.state.session_id()=(0U);idle.state.reference_epoch()=(0U);idle.state.motion_epoch()=(12U);idle.state.management_epoch()=(3U);idle.state.core_state()=(1U);
      root_fixture::set_position_quality(idle.state,0U);root_fixture::set_raw_quality(idle.state,63U);double raw_position[6]{.1,.2,.3,.4,.5,.6};root_fixture::set_raw_position(idle.state,raw_position);
      idle.state.state_seq()=(3U);idle.state.produced_at_us()=(device_now_us());
      if(!wait_for([&]{send(idle);return transport.statistics().stop_requests_confirmed==1U;}))return 811;
      const auto stats=transport.statistics();transport.stop();
      if(stats.stop_requests_sent!=1U ||stats.unconfirmed_stop_requests!=0U)return 812;
      std::fprintf(stderr,"GREEN_SOURCE_CALLBACK exactCancelCOMMITTED/freshHold/singleNewDisable/freshIdle confirmed=1\n");
      return 0;
      /**
       * @brief 以下RED旧tuple检查位于无条件return0之后，当前控制流不可达。
       * 保留的文字/返回805/806不代表本次正例还在执行该旧故障分支；实际正反断言已经在前面完成。
       */
      // RED：当前Worker已知道有未完成Cancel却继续用同旧tuple发Disable，无法合法Stop确认。
      if((*actual_disable.expected_motion_epoch())==(*actual_cancel.expected_motion_epoch()) &&
         (*actual_disable.expected_management_epoch())==(*actual_cancel.expected_management_epoch())) {
        std::fprintf(stderr,"RED_ORIGINAL_STOP_OVERLAP old_tuple_disable_and_QueryRejected_rc10 confirmed=%llu\n",
             static_cast<unsigned long long>(transport.statistics().stop_requests_confirmed));
        return 805;
      }
      // 此RED fixture不能把仅换epoch或盲重试当GREEN；合法候选需另完整确认场景。
      return 806;
    }
    const auto before = transport.statistics().stop_requests_sent;
    if (release_deactivate_mode == 3U) system.on_error(rclcpp_lifecycle::State());
    else system.on_deactivate(rclcpp_lifecycle::State());
    /**
     * @brief 普通deactivate是否需要额外Stop由已有强release+Idle证明决定。
     * mode1/6/7/8/9/11/18要求不重复发；on_error及无证明路径仍发Stop。这里只按发送计数断言，不能将“已发送”扩为“已确认”。
     */
    const bool expect_stop = release_deactivate_mode != 1U && release_deactivate_mode != 6U && release_deactivate_mode != 11U && release_deactivate_mode != 18U &&
        release_deactivate_mode != 7U &&
        release_deactivate_mode != 8U && release_deactivate_mode != 9U;
    if (expect_stop && !wait_for([&] {
          return transport.statistics().stop_requests_sent > before;
        })) return 235;
    if (!expect_stop) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      if (transport.statistics().stop_requests_sent != before) return 236;
    }
    transport.stop();
    /**
     * @brief stop关闭writer后逐记录重读raw文件，检查release/Query请求响应交替、中央规范记录和独立容器来源身份。
     * Host记录时间单调；默认Pending再Query有4条管理记录，直接Committed模式只有2条。不重算物理运动结果。
     */
    if (raw_capture_mode && (release_deactivate_mode == 1U || release_deactivate_mode == 7U)) {
      std::FILE *file = std::fopen(capture_path.c_str(), "rb");
      if (!file || ::fseek(file, 8L, SEEK_SET) != 0) return 244;
      const auto le = [](const std::uint8_t *bytes, std::size_t width) {
        std::uint64_t value = 0U;
        for (std::size_t i = 0U; i < width; ++i) value |= std::uint64_t{bytes[i]} << (8U * i);
        return value;
      };
      unsigned management_records = 0U;
      std::uint64_t previous_time = 0U;
      bool valid_capture = true;
      std::uint8_t header[kCaptureHeaderSize]{};
      while (valid_capture) {
        const auto header_bytes = std::fread(header, 1U, sizeof(header), file);
        if (header_bytes == 0U && std::feof(file)) break;
        if (header_bytes != sizeof(header)) { valid_capture = false; break; }
        const auto size = static_cast<std::uint16_t>(header[1] | (header[2] << 8U));
        std::array<std::uint8_t, dp::protocol::canonical::Traits<Status>::kMaxEncodedSize> raw{};
        const auto time = le(header + 3U, 8U);
        if (size == 0U || size > raw.size() ||
            std::fread(raw.data(), 1U, size, file) != size ||
            time == 0U || time < previous_time) { valid_capture = false; break; }
        previous_time = time;
        if (header[0] == 4U || header[0] == 5U) {
          const bool is_request = (management_records % 2U) == 0U;
          const auto &expected = management_records < 2U ? captured_release : captured_query;
          if (!expected || management_records >= (release_deactivate_mode == 7U ? 2U : 4U) ||
              header[0] != (is_request ? 4U : 5U) ||
              size > (is_request ? dp::protocol::canonical::Traits<Management::Request>::kMaxEncodedSize :
                                    dp::protocol::canonical::Traits<Management::Response>::kMaxEncodedSize)) {
            valid_capture = false;
            break;
          }
          if (is_request) {
            Management::Request request=owned_fixture::request();
            valid_capture = root_fixture::canonical_equal(raw.data(),size,expected->request) &&
                dp::protocol::canonical::Traits<Management::Request>::decode(raw.data(),size,&request) ==
                    true &&
                root_fixture::capture_source(header).boot_id!=0U && (*request.expected_boot_epoch())==(*expected->request.expected_boot_epoch()) &&
                le(header + 19U, 8U) == (*request.expected_session_id()) &&
                le(header + 27U, 8U) == 0U &&
                le(header + 35U, 4U) == (*request.request_seq()) &&
                (*request.operation()) == (management_records == 0U ? 10U : 12U) &&
                (*request.request_id()) == (management_records == 0U ? 80U : 81U) &&
                (management_records == 0U || (*request.queried_operation_id()) == 80U);
          } else {
            Management::Response response=owned_fixture::response();dp::comm::SourceId response_source{};
            valid_capture = root_fixture::canonical_equal(raw.data(),size,expected->response) &&
                ((response_source=root_fixture::capture_source(header)),dp::protocol::canonical::Traits<Management::Response>::decode(raw.data(),size,&response)) ==
                    true &&
                le(header + 11U, 8U) == response_source.boot_id &&
                le(header + 19U, 8U) == (*response.session_id()) &&
                le(header + 27U, 8U) == (*response.run_id()) &&
                le(header + 35U, 4U) == 0U && (*response.run_id()) == 2U &&
                (*response.request_id()) == (management_records == 1U ? 80U : 81U) &&
                (*response.transaction_phase()) ==
                    (management_records == 1U && release_deactivate_mode == 1U ? 1U : 3U) &&
                (management_records == 1U && release_deactivate_mode == 1U ?
                    !(*response.release_committed()) : (*response.release_committed()));
          }
          ++management_records;
        }
      }
      std::fclose(file);
      if (!valid_capture || management_records != (release_deactivate_mode == 7U ? 2U : 4U))
        return 247;
    }
    return 0;
  }
  /**
   * @brief 冷Idle/Fault断流：无reference时raw位置与可控制关节反馈分开。
   * Idle超预算可请求重新观测而不发Stop，Fault则必须停机且不能采用同一旧State。随后验证错hash与重复seq不能恢复冷态绑定。
   */
  if (cold_gap_mode) {
    if (!wait_for([&] {
          send(valid);
          std::lock_guard<std::mutex> lock(channel->mutex);
          State observed=owned_fixture::state();dp::comm::SourceId observed_source{};
          return channel->observation.state_received_us != 0U &&
              ((observed=channel->observation.state),(observed_source=channel->observation.state_source),message_fields_present(observed)) == true &&
              (*observed.state_seq()) == 1U;
        })) return 154;
    if (cold_fault_gap_mode) {
      auto fault = valid;
      fault.send_state = false;
      fault.status.core_state()=(9U);
      const auto before = transport.statistics().status_messages;
      if (!send(fault) || !wait_for([&] {
            return transport.statistics().status_messages > before;
          })) return 155;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(102));
    if (transport.statistics().heartbeat_stops != 0U) return 156;
    if (!wait_for([&] {
          const auto stats = transport.statistics();
          return cold_fault_gap_mode ? stats.heartbeat_stops != 0U :
              stats.raw_idle_reobservations != 0U;
        })) return 157;
    if (cold_fault_gap_mode) {
      if (transport.statistics().raw_idle_reobservations != 0U) return 158;
      if (!send_rejected_state(valid) || system.copy_state(&snapshot)) return 159;
    } else {
      if (transport.statistics().stop_requests_sent != 0U ||
          transport.statistics().unconfirmed_stop_requests != 0U) return 160;
      auto wrong_hash = valid;
      wrong_hash.send_state = false;
      std::uint8_t mismatched_hash[32]{};
      mismatched_hash[0] = 9U;
      wrong_hash.status.contract_sha256().emplace().assign(mismatched_hash,32);
      const auto bootstrap_before = transport.statistics().bootstrap_rejections;
      if (!send(wrong_hash) || !wait_for([&] {
            return transport.statistics().bootstrap_rejections > bootstrap_before;
          })) return 164;
      auto premature = valid;
      premature.send_status = false;
      premature.state.state_seq()=(2U);
      if (!send_rejected_state(premature)) return 165;
      auto status_only = valid;
      status_only.send_state = false;
      const auto before = transport.statistics().status_messages;
      if (!send(status_only) || !wait_for([&] {
            return transport.statistics().status_messages > before;
          })) return 161;
      auto stale = valid;
      stale.send_status = false;
      if (!send_rejected_state(stale)) return 162;
      auto next = stale;
      next.state.state_seq()=(2U);
      if (!wait_for([&] {
            send(next);
            std::lock_guard<std::mutex> lock(channel->mutex);
            State observed=owned_fixture::state();dp::comm::SourceId observed_source{};
            return ((observed=channel->observation.state),(observed_source=channel->observation.state_source),message_fields_present(observed)) == true &&
                (*observed.state_seq()) == 2U;
          }) || transport.statistics().stop_requests_sent != 0U) return 163;
    }
    transport.stop();
    return 0;
  }
  /**
   * @brief 历史released Idle采集基线组：15忽略迟到历史页；16未Committed失败；17新boot失败；18陌生session Status保留原值；19转活动态失败。
   * 这些分支只覆盖新采集的历史例外，不为正在运行的run放宽证据要求。
   */
  if (raw_order_case >= 15U && raw_order_case <= 19U) {
    auto historical = valid;
    historical.status.run_id()=(2U);
    historical.status.attempt_kind()=(1U);
    historical.status.parent_run_id()=(0U);
    historical.status.session_id()=(0U);
    historical.status.reference_epoch()=(0U);
    historical.status.core_state()=(1U);
    historical.status.final_state()=(2U);
    historical.status.release_committed()=(raw_order_case != 16U);
    historical.status.permit_remaining_us()=(0U);
    historical.state.session_id()=(0U);
    historical.state.reference_epoch()=(0U);
    historical.state.core_state()=(1U);
    root_fixture::set_position_quality(historical.state,0U);
    root_fixture::set_raw_quality(historical.state,parameters.configured_mask);
    const double raw_motor_rev[6]{0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
    root_fixture::set_raw_position(historical.state,raw_motor_rev);
    if (!send(historical)) return 285;
    if (raw_order_case == 16U) {
      if (!wait_for([&] { return transport.statistics().raw_evidence_failure != 0U; }))
        return 288;
      transport.stop();
      return 0;
    }
    if (!wait_for([&] {
          std::lock_guard<std::mutex> lock(channel->mutex);
          Status observed=owned_fixture::status();dp::comm::SourceId observed_source{};
          return ((observed=channel->observation.status),(observed_source=channel->observation.status_source),message_fields_present(observed)) == true && (*observed.run_id()) == 2U;
        })) return 285;
    if (raw_order_case >= 17U) {
      auto changed = historical;
      changed.send_state = false;
      if (raw_order_case == 17U) changed.status_source.boot_id=(8U);
      if (raw_order_case == 18U) changed.status.session_id()=(9U);
      if (raw_order_case == 19U) {
        changed.status.core_state()=(4U);
        changed.status.final_state()=(0U);
      }
      const auto status_count = transport.statistics().status_messages;
      if (raw_order_case == 18U) {
        if (!send(changed) || !wait_for([&] {
              return transport.statistics().status_messages > status_count;
            })) return 289;
        bool retained = false;
        {
          std::lock_guard<std::mutex> lock(channel->mutex);
          Status observed=owned_fixture::status();dp::comm::SourceId observed_source{};
          retained = ((observed=channel->observation.status),(observed_source=channel->observation.status_source),message_fields_present(observed)) == true && (*observed.session_id()) == 0U &&
              transport.statistics().raw_evidence_failure == 0U;
        }
        transport.stop();
        return retained ? 0 : 290;
      }
      if (!send(changed) || !wait_for([&] {
            return transport.statistics().raw_evidence_failure != 0U;
          })) return 289;
      transport.stop();
      return 0;
    }
    auto stale_page = historical;
    stale_page.send_state = false;
    stale_page.send_status = false;
    stale_page.send_evidence = true;
    (void)0;
    (void)0;
    stale_page.evidence.run_id()=(2U);
    stale_page.evidence.attempt_id()=(2U);
    stale_page.evidence.evidence_seq()=(0U);
    stale_page.evidence.attempt_kind()=(1U);
    stale_page.evidence.parent_run_id()=(0U);
    stale_page.evidence.motion_epoch()=(10U);
    stale_page.evidence.category()=(2U);
    stale_page.evidence.final()=(1U);
    stale_page.evidence.group_count()=(1U);
    stale_page.evidence.group_id()=(2U);
    stale_page.evidence.expected_mask()=(63U);
    stale_page.evidence.submitted_mask()=(63U);
    stale_page.evidence.sent_mask()=(63U);
    if (!send(stale_page)) return 286;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    const auto captured = transport.statistics();
    transport.stop();
    return captured.ready && !captured.initialization_failed &&
        captured.raw_evidence_failure == 0U &&
        captured.raw_evidence_pages_written == 0U ? 0 : 287;
  }
  /**
   * @brief 六轴冷态raw准入先保留raw电机rev但不产生copy_state控制快照。
   * 重复序号、raw缺轴、Source过旧/超前、错误core或raw NaN均拒绝且不刷新Host接收时间；stale/skew分别核计数。
   */
  if (six_axis && !heartbeat_budget_mode) {
    auto cold = valid;
    cold.status.core_state()=(1U);
    cold.state.core_state()=(1U);
    root_fixture::set_position_quality(cold.state,0U);
    root_fixture::set_velocity_quality(cold.state,63U);
    root_fixture::set_raw_quality(cold.state,63U);
    const double raw_motor_rev[6]{0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
    root_fixture::set_raw_position(cold.state,raw_motor_rev);
    if (!wait_for([&] {
          send(cold);
          std::lock_guard<std::mutex> lock(channel->mutex);
          State observed=owned_fixture::state();dp::comm::SourceId observed_source{};
          return channel->observation.state_received_us != 0U &&
              ((observed=channel->observation.state),(observed_source=channel->observation.state_source),message_fields_present(observed)) == true &&
              (*observed.core_state()) == 1U && root_fixture::raw_quality(observed) == 63U;
        }) || system.copy_state(&snapshot)) return 84;
    std::uint64_t cold_received_us = 0U;
    {
      std::lock_guard<std::mutex> lock(channel->mutex);
      cold_received_us = channel->observation.state_received_us;
    }
    if (!send_rejected_state(cold)) return 85;
    {
      std::lock_guard<std::mutex> lock(channel->mutex);
      if (channel->observation.state_received_us != cold_received_us) return 86;
    }
    for (unsigned fault = 0U; fault < 5U; ++fault) {
      auto wrong = cold;
      const auto diagnostics_before = transport.statistics();
      wrong.state.state_seq()=(2U);
      if (fault == 0U) root_fixture::set_raw_quality(wrong.state,0U);
      if (fault == 1U) wrong.state.produced_at_us()=(device_now_us() - 2000000U);
      if (fault == 2U) wrong.state.produced_at_us()=(device_now_us() + 2000000U);
      if (fault == 3U) wrong.state.core_state()=(4U);
      if (fault == 4U) {
        const double bad_raw[6]{NAN, 0.2, 0.3, 0.4, 0.5, 0.6};
        root_fixture::set_raw_position(wrong.state,bad_raw);
      }
      /** @brief 过旧/超前负例等待本次对应类别；总拒绝可能仍属于前一个在途刺激。 */
      const bool rejected = (fault == 1U || fault == 2U)
          ? wait_for([&] {
              if (!send(wrong)) return false;
              const auto after = transport.statistics();
              return fault == 1U
                  ? after.stale_payload_rejections > diagnostics_before.stale_payload_rejections
                  : after.clock_skew_rejections > diagnostics_before.clock_skew_rejections;
            })
          : send_rejected_state(wrong);
      if (!rejected || system.copy_state(&snapshot)) return 87 + fault;
      const auto diagnostics_after = transport.statistics();
      if (fault == 1U && diagnostics_after.stale_payload_rejections <=
              diagnostics_before.stale_payload_rejections) return 120;
      if (fault == 2U && diagnostics_after.clock_skew_rejections <=
              diagnostics_before.clock_skew_rejections) return 121;
      std::lock_guard<std::mutex> lock(channel->mutex);
      if (channel->observation.state_received_us != cold_received_us) return 92 + fault;
    }
    valid.state.state_seq()=(2U);
  }
  if(raw_order_case>=87U && raw_order_case<=96U) {valid.status.core_state()=(3U);valid.state.core_state()=(4U);valid.state.can_sent_mask()=(63U);valid.status.can_sent_mask()=(63U);}
  /**
   * @brief public Stop失效之后的raw cleanup通道与普通控制缓存分开。
   * 83原Run锚+新鲜同boot Fault可保留raw；84只有Hold无Run锚、85错boot、86重复Source产生时刻都不得写raw cleanup观测。
   */
  if((raw_order_case>=83U && raw_order_case<=86U)) {valid.status.core_state()=(raw_order_case==84U?4U:5U);valid.state.core_state()=(raw_order_case==84U?4U:5U);}
  if(!(raw_order_case>=83U && raw_order_case<=86U) && !wait_for([&] { send(valid); return system.copy_state(&snapshot); })) {
    const auto stats=transport.statistics();const auto diagnostic=transport.diagnostic_snapshot();
    std::fprintf(stderr,"ROOT_BOOTSTRAP_FAILED status=%llu state=%llu callbacks=%llu rejected=%llu first=%s\n",
      (unsigned long long)stats.status_messages,(unsigned long long)stats.state_messages,
      (unsigned long long)stats.state_callbacks,(unsigned long long)stats.rejected_states,diagnostic.first.detail.data());
    return 12;
  }
  if(owner_safety_stop_case) {
    // 先通过真实PTY管理回调建立本client COMMITTED owner，而非直接伪造所有权槽。
    auto pending=std::make_shared<DeviceManagementRequest>();Management::Request request=owned_fixture::request();
    (void)0;(void)0;request.request_id()=(44);request.request_seq()=(1);request.operation()=(1);
    std::uint8_t request_client[16];
    std::copy(parameters.client_instance_id.begin(),parameters.client_instance_id.end(),request_client);
    request.domain_device_id().emplace().assign((valid.status.domain_device_id()->data()),16);request.client_instance_id().emplace().assign(request_client,16);
    request.expected_boot_epoch()=(7);request.expected_session_id()=(8);request.expected_reference_epoch()=(9);
    request.expected_motion_epoch()=(9);request.expected_management_epoch()=(0);request.expected_profile_id()=(1);request.configured_mask()=(63);
    if(((pending->request=request),message_fields_present(request))!=true)return 880;
    {std::lock_guard<std::mutex> lock(channel->mutex);channel->queued=pending;}
    if(!wait_for([&]{send(valid);return pending->complete.load();}))return 881;
    // 错链原页使真实collector粘错；诊断源继续新鲜，不能将该错误当授权或清除它。
    auto bad=valid;bad.send_status=false;bad.send_state=false;bad.send_evidence=true;
    (void)0;(void)0;
    bad.evidence.run_id()=(3);bad.evidence.attempt_id()=(3);bad.evidence.attempt_kind()=(1);bad.evidence.category()=(1);
    bad.evidence.group_id()=(1);bad.evidence.group_count()=(1);bad.evidence.final()=(1);
    if(!send(bad) || !wait_for([&]{send(valid);return channel->raw_evidence_failure.load()!=0;}))return 882;
    if(!wait_for([&]{send(valid);return channel->request_owner_safety_stop(
        static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()),7,8,9);}))return 883;
    const auto now=static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count());
    if(channel->request_owner_safety_stop(now,7,8,9))return 884;
    pollfd stop_fd{stops_input,POLLIN,0};Management::Request stop=owned_fixture::request();
    if(::poll(&stop_fd,1,1000)!=1 || (root_fixture::read_value(stops_input,stop)==false) || (*stop.operation())!=10U)return 885;
    transport.stop();
    const auto stats=transport.statistics();
    return stats.stop_requests_sent==1U && channel->raw_evidence_failure.load()!=0U?0:886;
  }
  if((raw_order_case>=83U && raw_order_case<=86U)) {
    if(!send(valid))return 333;
    if(!wait_for([&] {
      std::lock_guard<std::mutex> lock(channel->mutex);State accepted{};dp::comm::SourceId accepted_source{};
      return ((accepted=channel->observation.state),(accepted_source=channel->observation.state_source),message_fields_present(accepted))==true && (*accepted.core_state())==(raw_order_case==84U?4U:5U);
    }))return 333;
    State established{};dp::comm::SourceId established_source{};
    {std::lock_guard<std::mutex> lock(channel->mutex);((established=channel->observation.state),(established_source=channel->observation.state_source),message_fields_present(established));}
    const auto before=transport.statistics().invalidations;
    system.request_transport_stop();
    if(!wait_for([&]{return transport.statistics().invalidations>before;}))return 330;
    auto fault=valid;fault.send_status=false;fault.state.core_state()=(9U);
    fault.state.session_id()=(0U);fault.state.reference_epoch()=(0U);fault.state.motion_epoch()=(11U);
    fault.state.state_seq()=((*established.state_seq())+1U);fault.state.produced_at_us()=(device_now_us());
    if(raw_order_case==85U)fault.state_source.boot_id=(17U);
    if(raw_order_case==86U)fault.state.produced_at_us()=((*established.produced_at_us()));
    const auto received_before=transport.statistics().state_messages;
    if(!send(fault)||!wait_for([&]{return transport.statistics().state_messages>received_before;}))return 331;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    const auto sample=channel->raw_cleanup_observation();
    std::printf("public_stop_invalidations=%llu normal_admission_state=%u rawCleanupStateReceived=%llu\n",
      static_cast<unsigned long long>(transport.statistics().invalidations),system.copy_state(&snapshot),static_cast<unsigned long long>(sample.state_received_us));
    std::printf(" mode=%u acceptedRunCore=%u acceptedSeq=%u faultSeq=%u producerBefore=%llu producerFault=%llu\n",raw_order_case,(*established.core_state()),(*established.state_seq()),(*fault.state.state_seq()),static_cast<unsigned long long>((*established.produced_at_us())),static_cast<unsigned long long>((*fault.state.produced_at_us())));
    transport.stop();const bool expected=raw_order_case==83U;return (sample.state_received_us!=0U)==expected?0:332;
  }


  // 正式State ACK先于Status Hold：不能丢掉已确认generation，也不能凭Status Hold制造ACK。
  /**
   * @brief State精确ACK早于Status Hold的排空见证。
   * 87应记录processed_generation/committed_seq/完整身份并quiescent；88无ACK、89部分mask且无ACK、90错epoch、91Run态、92错session、93陈旧、94错ref、95错boot、96先Stop均不能制造quiescent。
   * Status变Hold本身不补出State未证明的ACK。
   */
  if(raw_order_case>=87U && raw_order_case<=96U) {
    channel->inactive_prime_allowed.store(true,std::memory_order_release);
    State accepted=owned_fixture::state();dp::comm::SourceId accepted_source{};{std::lock_guard<std::mutex> lock(channel->mutex);((accepted=channel->observation.state),(accepted_source=channel->observation.state_source),message_fields_present(accepted));}
    set_command_identity({7U,8U,9U,10U,(*accepted.freshness_ticket())});
    const auto generation=publish_control_update(positions,6U,0,nullptr);
    if(!generation || !system.transfer_control_update() || ::poll(&response,1,1000)!=1) return 370;
    Command command;
    if((root_fixture::read_value(input,command)==false)||(*command.producer_seq())!=generation||(*command.motion_epoch())!=10U) return 371;
    auto ack=valid;ack.send_status=false;ack.state.state_seq()=(3U);
    ack.state.committed_producer_seq()=((*command.producer_seq()));

    if(raw_order_case==88U)ack.state.committed_producer_seq()=(0U);
    // 原GroupSentAck以committed序号为权威，partial mask本例仍须无committed ACK。
    if(raw_order_case==89U){ack.state.committed_producer_seq()=(0U);ack.state.can_sent_mask()=(31U);}
    if(raw_order_case==90U)ack.state.motion_epoch()=(11U);
    if(raw_order_case==91U)ack.state.core_state()=(5U);
    if(raw_order_case==92U)ack.state.session_id()=(9U);
    if(raw_order_case==93U)ack.state.produced_at_us()=(device_now_us()-2000000U);
    if(raw_order_case==94U)ack.state.reference_epoch()=(19U);
    if(raw_order_case==95U)ack.state_source.boot_id=(17U);
    const auto state_before=transport.statistics().state_messages;
    if(!send(ack)||!wait_for([&]{return transport.statistics().state_messages>state_before;}))return 372;
    if(raw_order_case==87U ||raw_order_case==88U ||raw_order_case==89U ||raw_order_case==91U ||raw_order_case==96U) {
      if(!wait_for([&]{return system.copy_state(&snapshot)&&snapshot.envelope.state_seq==3U;}))return 372;
    }

    {std::lock_guard<std::mutex> lock(channel->mutex);Status cached=owned_fixture::status();dp::comm::SourceId cached_source{};
      ((cached=channel->observation.status),(cached_source=channel->observation.status_source),message_fields_present(cached));
      if((*cached.core_state())!=3U || channel->hold_drain.quiescent)return 373;
    }

    if(raw_order_case==96U) {
      const auto invalidated=transport.statistics().invalidations;
      system.request_transport_stop();
      if(!wait_for([&]{return transport.statistics().invalidations>invalidated;}))return 376;
    }
    auto hold=valid;hold.send_state=false;hold.status.core_state()=(4U);
    if(raw_order_case!=87U &&raw_order_case!=96U)hold.status.can_sent_mask()=(0U);

    hold.status.committed_producer_seq()=(raw_order_case==87U?(*command.producer_seq()):0U);
    const auto statuses_before=transport.statistics().status_messages;
    if(!send(hold)||!wait_for([&]{if(raw_order_case==96U)return transport.statistics().status_messages>statuses_before;std::lock_guard<std::mutex> lock(channel->mutex);Status cached=owned_fixture::status();dp::comm::SourceId cached_source{};
      return ((cached=channel->observation.status),(cached_source=channel->observation.status_source),message_fields_present(cached))==true&&(*cached.core_state())==4U;})) return 374;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    bool passed=false;
    {std::lock_guard<std::mutex> lock(channel->mutex);const auto d=channel->hold_drain;
      std::printf("StateACKBeforeStatusHold generation=%llu seq=%u processed=%llu committed=%u quiescent=%u disposition=%u boot=%llu session=%llu ref=%u epoch=%u\n",
        static_cast<unsigned long long>(generation),(*command.producer_seq()),static_cast<unsigned long long>(d.processed_generation),d.committed_seq,d.quiescent,d.disposition,static_cast<unsigned long long>(d.boot_epoch),static_cast<unsigned long long>(d.session_id),d.reference_epoch,d.motion_epoch);
      passed=d.processed_generation==generation&&d.committed_seq==(*command.producer_seq())&&d.quiescent&&d.disposition==2U&&d.boot_epoch==7U&&d.session_id==8U&&d.reference_epoch==9U&&d.motion_epoch==10U;
    }

    if(raw_order_case!=87U) {
      std::lock_guard<std::mutex> lock(channel->mutex);
      passed=!channel->hold_drain.quiescent&&channel->hold_drain.processed_generation==0U;
    }
    transport.stop();return passed?0:375;

  }
  /**
   * @brief Complete时序子场景：请求47绑定旧motion10/management10，完成后保持motion11。
   * 这里的RPC complete原子标志只表示响应已到；Query47关联结果、Status/State时序、终末producer ACK与后续刷新仍有独立门槛。
   */
  if (complete_order_mode != 0U) {
    valid.status.management_epoch()=(10U);
    valid.state.management_epoch()=(10U);
    valid.state.state_seq()=(2U);
    if (!wait_for([&] { send(valid); return system.copy_state(&snapshot) &&
                              snapshot.envelope.state_seq == 2U; })) return 166;
    std::uint32_t terminal_sequence=0U;
    /**
     * @brief 先发送两代旧epoch目标；mode12故意不ACK最后一代。
     * Complete须等terminal_producer_seq已确认后才送peer，之后新提交的旧epoch目标只能排空/抑制，不能穿过冻结边界。
     */
    if(complete_order_mode==11U || complete_order_mode==12U) {
      channel->inactive_prime_allowed.store(true,std::memory_order_release);
      set_command_identity({7U,8U,9U,10U,9007199254740997ULL});
      for(unsigned sample=0U;sample<2U;++sample) {
        const auto generation=publish_control_update(positions,six_axis ? 6U : 1U,0,nullptr);
        if(!generation || !system.transfer_control_update() || ::poll(&response,1,3000)!=1)
          return 187;
        Command command;
        if((root_fixture::read_value(input,command)==false) ||
           (*command.motion_epoch())!=10U || (*command.producer_seq())!=generation) return 188;
        terminal_sequence=(*command.producer_seq());
        valid.state.state_seq()=(3U+sample);
        valid.state.committed_producer_seq()=(terminal_sequence);
        if(complete_order_mode==12U && sample==1U) continue;
        if(!wait_for([&] { send(valid); return system.copy_state(&snapshot) &&
                                  snapshot.envelope.state_seq==3U+sample; })) return 189;
      }
    }
    Management::Request complete=owned_fixture::request();
    (void)0; (void)0;
    complete.domain_device_id().emplace().assign(device,16);
    const std::uint8_t client[16]{2U};
    complete.client_instance_id().emplace().assign(client,16);
    complete.request_id()=(47U); complete.request_seq()=(47U);
    complete.operation()=(8U);
    complete.expected_boot_epoch()=(7U); complete.expected_session_id()=(8U);
    complete.expected_reference_epoch()=(9U); complete.expected_motion_epoch()=(10U);
    complete.expected_management_epoch()=(10U); complete.expected_profile_id()=(1U);
    complete.configured_mask()=(parameters.configured_mask);
    if(complete_order_mode==15U)
      complete.expected_management_epoch()=(11U);
    if(complete_order_mode==11U || complete_order_mode==12U)
      complete.terminal_producer_seq()=(terminal_sequence);
    auto pending=std::make_shared<DeviceManagementRequest>();
    if (((pending->request=complete),message_fields_present(complete))!=true) return 167;
    { std::lock_guard<std::mutex> lock(channel->mutex); channel->queued=pending; }
    /**
     * @brief Complete的expected_management_epoch故意错为11；要求本地failed且100ms没有operation8发送。
     * 拒绝发生在发送准入层，不能声称远端执行后回滚。
     */
    if(complete_order_mode==15U) {
      pollfd observed_request{stops_input,POLLIN,0};
      if(::poll(&observed_request,1,100)!=0 ||
         !wait_for([&] { return pending->failed.load(std::memory_order_acquire); })) return 196;
      transport.stop();
      return 0;
    }
    if(complete_order_mode==17U || (complete_order_mode>=30U && complete_order_mode<=41U)) {
      pollfd observed_request{stops_input,POLLIN,0};
      Management::Request sent_complete=owned_fixture::request();
      if(::poll(&observed_request,1,1000)!=1 ||
         (root_fixture::read_value(stops_input,sent_complete)==false) ||
         (*sent_complete.operation())!=8U) return 205;
      auto enabling=valid;
      enabling.status.motion_epoch()=(11U);
      enabling.status.core_state()=(3U);
      enabling.state.motion_epoch()=(11U);
      enabling.state.core_state()=(3U);
      enabling.state.state_seq()=(3U);
      enabling.send_state=false;
      if(!send(enabling) || !wait_for([&] {
          std::lock_guard<std::mutex> lock(channel->mutex);
          Status observed=owned_fixture::status();dp::comm::SourceId observed_source{};
          return ((observed=channel->observation.status),(observed_source=channel->observation.status_source),message_fields_present(observed))==true &&
              (*observed.motion_epoch())==11U;
        })) return 199;
      enabling.send_status=false;
      enabling.send_state=true;
      if(!wait_for([&] { send(enabling); return system.copy_state(&snapshot) &&
                                snapshot.envelope.state_seq==3U; })) return 200;
      channel->inactive_prime_allowed.store(true,std::memory_order_release);
      set_command_identity({7U,8U,9U,11U,9007199254740997ULL});
      const auto first=publish_control_update(positions,six_axis ? 6U : 1U,0,nullptr);
      if(!first || !system.transfer_control_update() || ::poll(&response,1,3000)!=1) return 201;
      Command first_command;
      if((root_fixture::read_value(input,first_command)==false) ||
         (*first_command.motion_epoch())!=11U || (*first_command.producer_seq())!=first) return 202;
      double second_position[6]{};
      std::copy(std::begin(positions),std::end(positions),second_position);
      /**
       * @brief 原Complete句柄置canceled是本地句柄清理，不是设备Cancel COMMITTED。
       * 以新鲜Hold State核同值目标抑制；31错boot、32错epoch、34陈旧载荷应保留旧seq3且无新发布，33偏移0.02rad应拒绝。
       */
      if(complete_order_mode>=30U && complete_order_mode<=34U) {
        // 原Complete句柄结束并转Query时canceled清理；不能把它当真实Complete终态。
        pending->canceled.store(true,std::memory_order_release);
        enabling.state.core_state()=(4U);
        enabling.state.hold_ready()=(true);
        root_fixture::set_position_quality(enabling.state,parameters.configured_mask);
        enabling.state.committed_producer_seq()=((*first_command.producer_seq()));
        enabling.state.state_seq()=(4U);
        if(complete_order_mode==31U || complete_order_mode==32U || complete_order_mode==34U) {
          const auto before_invalid=transport.statistics();
          if(complete_order_mode==31U) enabling.state_source.boot_id=(8U);
          if(complete_order_mode==32U) enabling.state.motion_epoch()=(12U);
          if(complete_order_mode==34U) enabling.state.produced_at_us()=(device_now_us()-2000000U);
          if(!send_rejected_state(enabling)) return 368;
          const auto after_invalid=transport.statistics();
          const bool no_admission=system.copy_state(&snapshot) && snapshot.envelope.state_seq==3U;
          transport.stop();
          return no_admission && after_invalid.rejected_states>before_invalid.rejected_states &&
              after_invalid.commands_published==before_invalid.commands_published ? 0 : 369;
        }
        if(!wait_for([&] { send(enabling); return system.copy_state(&snapshot) &&
                                  snapshot.envelope.state_seq==4U; })) return 364;
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        const auto before_same=transport.statistics();
        if(complete_order_mode==33U) second_position[0]+=0.02;
        const auto same=publish_control_update(second_position,six_axis ? 6U : 1U,0,nullptr);
        if(!same || !system.transfer_control_update()) return 365;
        // 等待owner消费此代际，不能用加长sleep掩盖Stop。
        for(unsigned n=0;n<8U;++n) {
          enabling.state.state_seq()=(5U+n);
          if(!send(enabling)) return 366;
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        const auto after_same=transport.statistics();
        transport.stop();
        if(complete_order_mode==33U)
          return after_same.rejected_commands>before_same.rejected_commands &&
              after_same.commands_published==before_same.commands_published ? 0 : 370;
        return after_same.rejected_commands==before_same.rejected_commands &&
            after_same.stop_requests_sent==before_same.stop_requests_sent &&
            after_same.commands_published==before_same.commands_published ? 0 : 367;
      }
      /**
       * @brief Status先报告Hold时须具精确producer序号与完整SENT，才可把同值prime当已排空。
       * 35同值不重复发布；36偏移目标拒绝；37无SENT、40错序号、41部分SENT不能凭Hold字样放行。38错epoch失效；39保鲜State但让pending过2.1s，不能借旧Status继续抑制新命令。
       */
      if(complete_order_mode>=35U && complete_order_mode<=41U) {
        auto hold_status=enabling; hold_status.send_state=false; hold_status.send_status=true;
        hold_status.status.core_state()=(4U);
        hold_status.status.committed_producer_seq()=((*first_command.producer_seq()));
        hold_status.status.can_submitted_mask()=(parameters.configured_mask);
        hold_status.status.can_sent_mask()=(parameters.configured_mask);
        if(complete_order_mode==37U) hold_status.status.can_sent_mask()=(0U);
        if(complete_order_mode==40U) hold_status.status.committed_producer_seq()=((*first_command.producer_seq())+1U);
        if(complete_order_mode==41U) hold_status.status.can_sent_mask()=(parameters.configured_mask & ~1U);
        if(complete_order_mode==38U) hold_status.status.motion_epoch()=(12U);
        const auto before_status_stats=transport.statistics();
        const auto before_status=before_status_stats.status_messages;
        if(!send(hold_status) || !wait_for([&] {
              std::lock_guard<std::mutex> lock(channel->mutex); Status observed=owned_fixture::status();dp::comm::SourceId observed_source{};
              return ((observed=channel->observation.status),(observed_source=channel->observation.status_source),message_fields_present(observed))==true &&
                  (*observed.core_state())==4U && transport.statistics().status_messages>before_status;
            })) return 389;
        if(complete_order_mode==36U) second_position[0]+=0.02;
        std::uint32_t following_state_seq=4U;
        if(complete_order_mode==39U) {
          const auto started=transport_steady_now_us(); std::uint32_t seq=4U;
          while(transport_steady_now_us()-started<2100000U) {
            enabling.state.state_seq()=(seq++); enabling.state.produced_at_us()=(device_now_us());
            if(!send(enabling)) return 394;
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
          }
          following_state_seq=seq;
          const auto expired_elapsed=transport_steady_now_us()-started;
          std::fprintf(stderr,"EXPIRED_COMPLETE_PENDING_WITNESS elapsed_us=%llu send_status=%d last_state_seq=%u\n",
              static_cast<unsigned long long>(expired_elapsed),enabling.send_status,seq-1U);
          if(expired_elapsed<2100000U) return 397;
          std::lock_guard<std::mutex> lock(channel->mutex); Status observed=owned_fixture::status();dp::comm::SourceId observed_source{};
          if(enabling.send_status || ((observed=channel->observation.status),(observed_source=channel->observation.status_source),message_fields_present(observed))!=true || (*observed.core_state())!=4U ||
              transport.statistics().heartbeat_stops!=0U || transport.statistics().stop_requests_sent!=0U ||
              transport.statistics().invalidations!=before_status_stats.invalidations) return 396;
        }
        const auto before_same=transport.statistics();
        const auto same=publish_control_update(second_position,six_axis ? 6U : 1U,0,nullptr);
        const bool transferred=same && system.transfer_control_update();
        if(!transferred) {
          const auto rejected=transport.statistics();
          transport.stop();
          // 错Status epoch已失效授权，控制交换应直接拒绝，不要求它进入owner再拒绝。
          return complete_order_mode==38U && rejected.invalidations>before_status_stats.invalidations &&
              rejected.commands_published==before_same.commands_published ? 0 : 390;
        }
        for(unsigned n=0;n<8U;++n) {
          enabling.state.state_seq()=(following_state_seq+n);
          if(!send(enabling)) return 391;
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        const auto after=transport.statistics();
        transport.stop();
        if(complete_order_mode==38U) return after.invalidations>before_status_stats.invalidations &&
            after.commands_published==before_same.commands_published ? 0 : 395;
        if(complete_order_mode==36U || complete_order_mode==37U || complete_order_mode==39U || complete_order_mode==40U || complete_order_mode==41U)
          return after.rejected_commands>before_same.rejected_commands &&
            after.commands_published==before_same.commands_published ? 0 : 392;
        return after.rejected_commands==before_same.rejected_commands &&
            after.stop_requests_sent==before_same.stop_requests_sent &&
            after.commands_published==before_same.commands_published ? 0 : 393;
      }
      second_position[0]+=0.0008;
      const auto second=publish_control_update(second_position,six_axis ? 6U : 1U,0,nullptr);
      const auto after_first=transport.statistics();
      if(!second || !system.transfer_control_update()) return 203;
      enabling.state.state_seq()=(4U);
      enabling.state.committed_producer_seq()=((*first_command.producer_seq()));
      if(!wait_for([&] { send(enabling); return system.copy_state(&snapshot) &&
                                snapshot.envelope.state_seq==4U; }) ||
         ::poll(&response,1,100)!=0 ||
         transport.statistics().commands_published!=after_first.commands_published ||
         transport.statistics().group_sent_ack_timeouts!=0U) return 204;
      transport.stop();
      return 0;
    }
    if(complete_order_mode==11U || complete_order_mode==12U) {
      pollfd observed_request{stops_input,POLLIN,0};
      Management::Request sent_complete=owned_fixture::request();
      if(complete_order_mode==12U) {
        if(::poll(&observed_request,1,100)!=0) return 192;
        if(!wait_for([&] { send(valid); return system.copy_state(&snapshot) &&
                                  snapshot.envelope.state_seq==4U; })) return 193;
      }
      if(::poll(&observed_request,1,1000)!=1 ||
         (root_fixture::read_value(stops_input,sent_complete)==false) ||
         (*sent_complete.operation())!=8U || (*sent_complete.terminal_producer_seq())!=terminal_sequence)
        return 168;
      const auto before=transport.statistics();
      for(unsigned sample=0U;sample<3U;++sample) {
        const auto generation=publish_control_update(positions,six_axis ? 6U : 1U,0,nullptr);
        if(!generation || !system.transfer_control_update()) return 190;
      }
      if(::poll(&response,1,100)!=0 ||
         transport.statistics().commands_published!=before.commands_published ||
         transport.statistics().group_sent_ack_timeouts!=before.group_sent_ack_timeouts)
        return 191;
      transport.stop();
      return 0;
    }
    if (!wait_for([&] { return pending->complete.load(); })) return 168;
    if(complete_order_mode==28U || complete_order_mode==29U) {
      pollfd observed_request{stops_input,POLLIN,0};
      Management::Request sent_complete=owned_fixture::request();
      if(::poll(&observed_request,1,1000)!=1 ||
         (root_fixture::read_value(stops_input,sent_complete)==false) ||
         (*sent_complete.request_id())!=47U || (*sent_complete.operation())!=8U ||
         (*sent_complete.terminal_producer_seq())!=0U) return 363;
    }
    auto next=valid;
    next.status.motion_epoch()=(11U);
    next.state.motion_epoch()=(11U);
    next.state.state_seq()=(3U);
    next.status.core_state()=(4U);
    next.state.core_state()=(4U);
    next.state.hold_ready()=(true);
    const auto baseline=transport.statistics();
    /**
     * @par 内部send_part回调
     * status/state布尔选择只发布完成后的哪一半，Source时刻每次刷新；调用点安排Status先、State先、Query先的可接受顺序与超时负例。
     */
    auto send_part=[&](bool status,bool state) {
      auto part=next;
      part.send_status=status; part.send_state=state;
      part.state.produced_at_us()=(device_now_us());
      return send(part);
    };
    if(complete_order_mode==4U || complete_order_mode==29U) {
      next.status.motion_epoch()=(12U);
      if(!send_part(true,false) || !wait_for([&] {
          return transport.statistics().invalidations>baseline.invalidations;
        })) return 177;
      transport.stop();
      return 0;
    }
    if(complete_order_mode==9U) {
      next.status.core_state()=(9U);
      if(!send_part(true,false) || !wait_for([&] {
          std::lock_guard<std::mutex> lock(channel->mutex);
          Status observed=owned_fixture::status();dp::comm::SourceId observed_source{};
          return ((observed=channel->observation.status),(observed_source=channel->observation.status_source),message_fields_present(observed))==true && (*observed.motion_epoch())==11U;
        }) || transport.statistics().invalidations<=baseline.invalidations) return 184;
      transport.stop();
      return 0;
    }
    if(complete_order_mode==5U) {
      if(!send_part(false,true) || !wait_for([&] {
          return transport.statistics().state_authority_rejections>
            baseline.state_authority_rejections;
        }) || !wait_for([&] {
          return transport.statistics().state_heartbeat_gap_stops>
            baseline.state_heartbeat_gap_stops;
        })) return 178;
      transport.stop();
      return 0;
    }
    /**
     * @par 内部complete_query回调
     * 复制原Complete身份改QueryResult48/seq48，queried_operation_id固定47，排入channel后等RPC回调complete。
     * 其返回值不是MotionCompletedHolding/已落盘证明；后续仍检查各状态和资格位。
     */
    auto complete_query=[&] {
      auto query=complete;
      query.operation()=(12U); query.request_id()=(48U); query.request_seq()=(48U);
      query.queried_operation_id()=(47U);
      auto requested=std::make_shared<DeviceManagementRequest>();
      if(((requested->request=query),message_fields_present(query))!=true) return false;
      { std::lock_guard<std::mutex> lock(channel->mutex); channel->queued=requested; }
      return wait_for([&] { return requested->complete.load(); });
    };
    if(complete_order_mode==1U || complete_order_mode==28U || (complete_order_mode>=6U && complete_order_mode<=8U) ||
       complete_order_mode==10U || complete_order_mode==13U || complete_order_mode==14U ||
       (complete_order_mode>=18U && complete_order_mode<=27U)) {
      if(!send_part(true,false) || !wait_for([&] {
          return transport.statistics().status_messages>baseline.status_messages;
        }) || transport.statistics().invalidations!=baseline.invalidations) return 169;
      if(!send_part(false,true) || !wait_for([&] {
          return system.copy_state(&snapshot) && snapshot.envelope.state_seq==3U;
        })) return 170;
      if(complete_order_mode!=8U && complete_order_mode!=10U &&
         complete_order_mode!=14U && !complete_query()) return 171;
    } else if(complete_order_mode==2U) {
      if(!send_part(false,true) || !wait_for([&] {
          return transport.statistics().state_messages>baseline.state_messages;
        })) return 172;
      if(!send_part(true,false) || !wait_for([&] {
          return transport.statistics().status_messages>baseline.status_messages;
        })) return 173;
      next.state.state_seq()=(4U);
      if(!send_part(false,true) || !wait_for([&] {
          return system.copy_state(&snapshot) && snapshot.envelope.state_seq==4U;
        }) || !complete_query()) return 174;
    } else {
      if(!complete_query() || !send_part(false,true) || !wait_for([&] {
          return system.copy_state(&snapshot) && snapshot.envelope.state_seq==3U;
        }) || !send_part(true,false)) return 175;
    }
    if(complete_order_mode==14U) {
      for(unsigned sample=0U;sample<110U;++sample) {
        next.state.state_seq()=(4U+sample);
        if(!send_part(false,true)) return 195;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
    }
    /**
     * @brief 完成导出资格组：Status final6/motion_complete与非零链仅建立本地post_complete_begin_ready门槛。
     * 19无链、21错run、22evidence_lost禁止刷新；23/24/25之后的矛盾Status必须撤销已建立资格。这里不是独立重算该Status声称的全部raw链。
     */
    if(complete_order_mode>=18U && complete_order_mode<=27U) {
      auto exported=next;
      exported.send_state=false;
      exported.status.final_state()=(6U);
      exported.status.motion_complete()=(true);
      if(complete_order_mode==21U) exported.status.run_id()=(2U);
      if(complete_order_mode==22U) exported.status.evidence_lost()=(true);
      std::uint8_t chain[32]{};
      if(complete_order_mode!=19U) chain[0]=0x5aU;
      exported.status.evidence_chain_sha256().emplace().assign(chain,32);
      if(!send(exported) || !wait_for([&] {
          std::lock_guard<std::mutex> lock(channel->mutex);
          Status observed=owned_fixture::status();dp::comm::SourceId observed_source{};
          return ((observed=channel->observation.status),(observed_source=channel->observation.status_source),message_fields_present(observed))==true &&
              (*observed.final_state())==6U;
        })) return 206;
      if(channel->post_complete_begin_ready.load(std::memory_order_acquire) !=
         (complete_order_mode!=19U && complete_order_mode!=21U && complete_order_mode!=22U))
        return 217;
      if(complete_order_mode>=23U && complete_order_mode<=25U) {
        auto contradiction=exported;
        if(complete_order_mode==23U) contradiction.status.evidence_lost()=(true);
        else if(complete_order_mode==24U) {
          contradiction.status.final_state()=(0U);
          contradiction.status.motion_complete()=(false);
          std::uint8_t empty_chain[32]{};
          contradiction.status.evidence_chain_sha256().emplace().assign(empty_chain,32);
        } else {
          contradiction.status.attempt_kind()=(2U);
          contradiction.status.parent_run_id()=(1U);
        }
        if(!send(contradiction) || !wait_for([&] {
            std::lock_guard<std::mutex> lock(channel->mutex);
            Status observed=owned_fixture::status();dp::comm::SourceId observed_source{};
            return ((observed=channel->observation.status),(observed_source=channel->observation.status_source),message_fields_present(observed))==true &&
                (complete_order_mode==23U ? (*observed.evidence_lost()) :
                 complete_order_mode==24U ? (*observed.final_state())==0U :
                 (*observed.attempt_kind())==2U);
          }) || channel->post_complete_begin_ready.load(std::memory_order_acquire)) return 219;
        channel->inactive_prime_allowed.store(true,std::memory_order_release);
        set_command_identity({7U,8U,9U,11U,9007199254740997ULL});
        const auto before=transport.statistics();
        const auto generation=publish_control_update(positions,six_axis ? 6U : 1U,0,nullptr);
        if(!generation || !system.transfer_control_update() || ::poll(&response,1,100)!=0 ||
           transport.statistics().commands_published!=before.commands_published) return 220;
        transport.stop();
        return 0;
      }
      channel->inactive_prime_allowed.store(true,std::memory_order_release);
      set_command_identity({7U,8U,9U,11U,9007199254740997ULL});
      /**
       * @brief 刷新目标先入队再清身份应被拒绝；重新收到新State并建立当前身份后，新generation可恢复。
       * 被拒代次不能冒充已处理/已确认水位。
       */
      if(complete_order_mode==26U) {
        const auto before=transport.statistics();
        const auto stale=publish_control_update(positions,six_axis ? 6U : 1U,0,nullptr);
        if(!stale || !system.transfer_control_update()) return 221;
        clear_command_identity();
        if(!wait_for([&] {
            return transport.statistics().rejected_commands>before.rejected_commands;
          }) || ::poll(&response,1,100)!=0) return 222;
        next.state.state_seq()=(4U);
        next.send_status=false;
        if(!send(next) || !wait_for([&] {
            return system.copy_state(&snapshot) && snapshot.envelope.state_seq==4U;
          })) return 258;
        set_command_identity({7U,8U,9U,11U,9007199254740997ULL});
        const auto fresh=publish_control_update(positions,six_axis ? 6U : 1U,0,nullptr);
        if(!fresh || !system.transfer_control_update() || ::poll(&response,1,1000)!=1)
          return 223;
        Command recovered;
        if((root_fixture::read_value(input,recovered)==false) ||
           (*recovered.producer_seq())!=fresh) return 224;
        transport.stop();
        return 0;
      }
      const auto before_refresh=transport.statistics();
      const auto refresh=publish_control_update(positions,six_axis ? 6U : 1U,0,nullptr);
      if(!refresh || !system.transfer_control_update()) return 207;
      if(complete_order_mode==19U || complete_order_mode==21U || complete_order_mode==22U) {
        if(::poll(&response,1,100)!=0 ||
           transport.statistics().commands_published!=before_refresh.commands_published) return 208;
        transport.stop();
        return 0;
      }
      if(::poll(&response,1,1000)!=1) return 209;
      Command first_refresh;
      if((root_fixture::read_value(input,first_refresh)==false) ||
         (*first_refresh.motion_epoch())!=11U || (*first_refresh.producer_seq())!=refresh) return 210;
      /**
       * @brief 发送刷新后、尚无State精确ACK时不得提前将hold_drain标quiescent。
       */
      if(complete_order_mode==20U) {
        std::lock_guard<std::mutex> lock(channel->mutex);
        if(channel->hold_drain.processed_generation>=refresh || channel->hold_drain.quiescent)
          return 259;
      }
      if(complete_order_mode==27U) {
        for(unsigned sample=0U;sample<8U;++sample) {
          next.state.state_seq()=(4U+sample);
          if(!send_part(false,true)) return 225;
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        if(!wait_for([&] {
            return transport.statistics().group_sent_ack_timeouts>
                before_refresh.group_sent_ack_timeouts;
          })) return 226;
        transport.stop();
        return 0;
      }
      next.state.state_seq()=(4U);
      next.state.committed_producer_seq()=((*first_refresh.producer_seq()));
      if(!send_part(false,true) || !wait_for([&] {
          return system.copy_state(&snapshot) && snapshot.envelope.state_seq==4U;
        }) || transport.statistics().group_sent_ack_timeouts!=0U) return 211;
      /**
       * @brief 收到精确ACK后生成quiescent排空回执；冷却期同值目标抑制但processed_generation仍推进。
       * 随后切新run、持续保鲜超过1s再刷新，证明不是永久丢弃保持目标；已退休身份的被拒代次不得推进排空水位。
       */
      if(complete_order_mode==20U) {
        if(!wait_for([&] {
            std::lock_guard<std::mutex> lock(channel->mutex);
            return channel->hold_drain.processed_generation>=refresh &&
                   channel->hold_drain.committed_seq==refresh &&
                   channel->hold_drain.quiescent && channel->hold_drain.disposition==2U;
          })) return 260;
        const auto before_cooldown=transport.statistics();
        const auto suppressed=publish_control_update(positions,six_axis ? 6U : 1U,0,nullptr);
        if(!suppressed || !system.transfer_control_update() || ::poll(&response,1,100)!=0 ||
           transport.statistics().commands_published!=before_cooldown.commands_published)
          return 218;
        if(!wait_for([&] {
            std::lock_guard<std::mutex> lock(channel->mutex);
            return channel->hold_drain.processed_generation>=suppressed &&
                   channel->hold_drain.committed_seq==refresh &&
                   channel->hold_drain.motion_epoch==11U && channel->hold_drain.quiescent &&
                   channel->hold_drain.disposition==1U && channel->hold_drain.observed_us!=0U &&
                   channel->hold_drain.max_state_age_us!=0U &&
                   channel->hold_drain.position[0]==positions[0] &&
                   channel->hold_drain.tolerance[0]==0.003;
          })) return 261;
        auto active_run=exported;
        active_run.status.run_id()=(2U);
        active_run.status.final_state()=(0U);
        active_run.status.motion_complete()=(false);
        std::uint8_t empty_chain[32]{};
        active_run.status.evidence_chain_sha256().emplace().assign(empty_chain,32);
        if(!send(active_run) || !wait_for([&] {
            std::lock_guard<std::mutex> lock(channel->mutex);
            Status observed=owned_fixture::status();dp::comm::SourceId observed_source{};
            return ((observed=channel->observation.status),(observed_source=channel->observation.status_source),message_fields_present(observed))==true &&
                (*observed.run_id())==2U;
          })) return 212;
        for(unsigned i=0U;i<55U;++i) {
          next.state.state_seq()=(5U+i);
          if(!send_part(false,true)) return 213;
          if(i%12U==0U && !send(active_run)) return 214;
          std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        const auto before_rejected=transport.statistics();
        const auto rejected=publish_control_update(positions,six_axis ? 6U : 1U,0,nullptr);
        if(!rejected || !system.transfer_control_update()) return 262;
        clear_command_identity();
        if(!wait_for([&] {
            return transport.statistics().rejected_commands>before_rejected.rejected_commands;
          })) return 263;
        {
          std::lock_guard<std::mutex> lock(channel->mutex);
          if(channel->hold_drain.processed_generation>=rejected) return 264;
        }
        set_command_identity({7U,8U,9U,11U,9007199254740997ULL});
        const auto next_refresh=publish_control_update(positions,six_axis ? 6U : 1U,0,nullptr);
        if(!next_refresh || !system.transfer_control_update() || ::poll(&response,1,1000)!=1)
          return 215;
        Command second_refresh;
        if((root_fixture::read_value(input,second_refresh)==false) ||
           (*second_refresh.producer_seq())!=next_refresh) return 216;
      }
      transport.stop();
      return 0;
    }
    /**
     * @brief Complete待定/保持抑制组：0或0.0008rad小变化不得误发布/误计ACK超时。
     * mode7的0.02rad偏移超容差应触发Stop；mode14让pending经过约2.2s而继续供新State，避免用State失鲜解释结果。
     */
    if((complete_order_mode>=6U && complete_order_mode<=8U) ||
       complete_order_mode==10U || complete_order_mode==13U || complete_order_mode==14U) {
      channel->inactive_prime_allowed.store(true,std::memory_order_release);
      set_command_identity({7U,8U,9U,11U,9007199254740997ULL});
      double hold[6]{};
      std::copy(std::begin(positions),std::end(positions),hold);
      if(complete_order_mode==7U) hold[0]+=0.02;
      if(complete_order_mode==13U || complete_order_mode==14U) hold[0]+=0.0008;
      const auto generation=publish_control_update(hold,six_axis ? 6U : 1U,0,nullptr);
      if(!generation || !system.transfer_control_update()) return 179;
      if(complete_order_mode==6U || complete_order_mode==7U || complete_order_mode==8U ||
         complete_order_mode==10U || complete_order_mode==13U || complete_order_mode==14U) {
        if(::poll(&response,1,100)!=0) return 185;
      } else {
        if(::poll(&response,1,3000)!=1) return 180;
        Command command;
        if((root_fixture::read_value(input,command)==false) ||
           (*command.motion_epoch())!=11U || (*command.producer_seq())!=generation) return 180;
      }
      for(unsigned i=0U;i<8U;++i) {
        next.state.state_seq()=(4U+i);
        if(!send_part(false,true)) return 181;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      if((complete_order_mode==6U || complete_order_mode==8U || complete_order_mode==10U ||
          complete_order_mode==13U || complete_order_mode==14U) &&
         (transport.statistics().commands_published!=baseline.commands_published ||
          transport.statistics().group_sent_ack_timeouts!=baseline.group_sent_ack_timeouts ||
          transport.statistics().stop_requests_sent!=baseline.stop_requests_sent)) return 182;
      if(complete_order_mode==7U &&
         (transport.statistics().commands_published!=baseline.commands_published ||
          !wait_for([&] { return transport.statistics().stop_requests_sent>
            baseline.stop_requests_sent; }))) return 194;
      if(complete_order_mode==10U && !complete_query()) return 186;
      transport.stop();
      return 0;
    }
    if(transport.statistics().invalidations!=baseline.invalidations ||
       transport.statistics().heartbeat_stops!=baseline.heartbeat_stops) return 176;
    transport.stop();
    return 0;
  }
  /**
   * @brief raw采集场景入口；多数按raw_order_case提前结束，不继续执行普通目标/Stop全集。
   * 采集成功以页面数、失败码、终态flush及文件顺序判断；不把Status宣称的chain当独立原页验证。
   */
  if (raw_capture_mode) {
    if (raw_order_case != 0U) {
      /**
       * @par 内部publish_page（顺序场景）
       * run/sequence标识证据链位置；category1/2/3/4选择motion/stop/disable/terminal，group_count为报告组数，final_state只写终态。
       * previous输入前链，next输出本页SHA；kind/parent绑定attempt身份，deliver=false只算字节/摘要而不发页，专供Status先到测试。
       * 各raw_order_case在此单独改motion、kind、previous、mask等负例。链输入为previous+canonical prefix+evidence_lost；不包含当前chain自身。
       * 返回true仅表示编码及（选中时）pipe发送成功，真实collector拒绝/落盘由调用点另等统计。
       */
      auto publish_page = [&](std::uint64_t run, unsigned sequence, unsigned category,
                              unsigned group_count, unsigned final_state,
                              const std::array<std::uint8_t, 32U> &previous,
                              std::array<std::uint8_t, 32U> *next,
                              unsigned kind = 1U, std::uint64_t parent = 0U,
                              bool deliver = true) {
        auto stimulus = valid;
        stimulus.send_status = false;
        stimulus.send_state = false;
        stimulus.send_evidence = true;
        auto &message = stimulus.evidence;
        (void)0;
        (void)0;
        message.run_id()=(run);
        message.attempt_id()=(run);
        message.evidence_seq()=(sequence);
        message.attempt_kind()=(run == 2U &&
            (raw_order_case == 4U || (raw_order_case == 11U && sequence == 1U)) ? 2U : kind);
        message.parent_run_id()=(parent);
        message.motion_epoch()=(raw_order_case >= 43U && raw_order_case <= 50U ?
            (run == 1U ? 1U : run == 2U && sequence == 0U ? 3U : 5U) :
            (raw_order_case >= 34U && raw_order_case <= 42U) ||
                (raw_order_case >= 51U && raw_order_case <= 58U) ?
            (run == 1U ? 1U : 3U) : 10U);
        if (run == 2U && sequence == 0U && raw_order_case == 38U)
          message.motion_epoch()=(0U);
        if (raw_order_case >= 59U && run == 123U)
          message.motion_epoch()=(category == 2U ? 10U : 9U);
        if (raw_order_case == 70U && run == 124U && sequence == 0U)
          message.motion_epoch()=(11U);
        if (run == 124U && sequence == 0U && raw_order_case == 31U)
          message.motion_epoch()=(0U);
        message.category()=(category);
        message.final()=(1U);
        message.group_count()=(group_count);
        if (category == 4U) {
          message.final_state()=(final_state);
          if (((run == 1U && raw_order_case >= 34U && raw_order_case <= 58U &&
                raw_order_case != 41U) ||
               (run >= 2U && raw_order_case >= 43U && raw_order_case <= 50U)))
            message.motion_complete()=(true);
        }
        else {
          message.group_id()=(run);
          message.expected_mask()=(63U);
          message.submitted_mask()=(63U);
          message.sent_mask()=(63U);
          if ((run == 124U && sequence == 0U && raw_order_case == 30U) ||
              (run == 2U && sequence == 0U && raw_order_case == 40U))
            message.sent_mask()=(31U);
        }
        if(cancel_pending_flush_case) {
          if((cancel_pending_flush_fault==1U && run==123U && category==2U) ||
             (cancel_pending_flush_fault==3U && run==124U && sequence==0U))message.sent_mask()=(31U);
          if(cancel_pending_flush_fault==4U && run==124U && sequence==0U)message.motion_epoch()=(11U);
        }
        if(cancel_origin_case && run==123U) {
          message.motion_epoch()=(category==4U ?
            (cancel_origin_fault==1U ? 48U : cancel_origin_fault==2U || cancel_origin_fault==3U ? 50U : cancel_origin_fault==6U ? 53U : 49U) :
            category==2U ? 52U : sequence==0U ? 50U : 51U);
        }
        if(cancel_origin_case && run==124U) {
          message.motion_epoch()=(52U);
          if(category==4U)message.motion_complete()=(true);
        }
        std::uint8_t previous_digest[32]{};
        std::copy(previous.begin(), previous.end(), previous_digest);
        if (run == 2U && (raw_order_case == 5U || raw_order_case == 55U ||
                          (raw_order_case == 12U && sequence == 1U))) previous_digest[0] ^= 1U;
        message.previous_chain_sha256().emplace().assign(previous_digest,32);
        if(!dummy_ref::comm::attempt_evidence::next_chain(previous,message,next))return false;
        std::uint8_t next_digest[32]{};
        std::copy(next->begin(), next->end(), next_digest);
        message.chain_sha256().emplace().assign(next_digest,32);
        return !deliver || send(stimulus);
      };
      /**
       * @par 内部seed_for回调
       * 输入run，输出以run小端、boot7及client/四hash填充192字节后的SHA链起点；未写区为零。
       * 它是当前测试身份的genesis，不是可信外部签名，也不是包含firmware.bin内容的摘要。
       */
      auto seed_for = [&](std::uint64_t run) {
        /**
         * @brief 无组首终态使用run3/boot7及固定身份构造genesis；terminal_reason14解释0组终态。
         * 释放后落页仍应在39字节头中保留原boot7/session8，而不是改成已清零owner。
         */
        std::array<std::uint8_t, 192U> seed{};
        for (unsigned i = 0U; i < 8U; ++i)
          seed[i] = static_cast<std::uint8_t>(run >> (8U * i));
        seed[8] = 7U;
        std::copy(parameters.client_instance_id.begin(), parameters.client_instance_id.end(), seed.begin() + 16U);
        std::copy(parameters.contract_hash.begin(), parameters.contract_hash.end(), seed.begin() + 32U);
        std::copy(parameters.model_hash.begin(), parameters.model_hash.end(), seed.begin() + 64U);
        std::copy(parameters.profile_hash.begin(), parameters.profile_hash.end(), seed.begin() + 96U);
        std::copy(parameters.firmware_hash.begin(), parameters.firmware_hash.end(), seed.begin() + 128U);
        std::array<std::uint8_t, 32U> previous{};
        ::SHA256(seed.data(), seed.size(), previous.data());
        return previous;
      };
      if(cancel_origin_case) {
        // 实机顺序：Acquire Idle epoch48，恢复参考后零组Idle epoch49，
        // EnableHold首CAN50，Begin CAN51，Cancel安全Hold52，终页attempt epoch49。
        auto origin=valid;origin.send_state=false;origin.status.core_state()=(2U);
        origin.status.attempt_id()=(cancel_origin_fault==5U ? 999U : 123U);
        origin.status.group_count()=(cancel_origin_fault==4U ? 1U : 0U);origin.status.final_state()=(0U);
        auto send_origin=[&](unsigned epoch) {
          origin.status.motion_epoch()=(epoch);
          return send(origin) && wait_for([&]{std::lock_guard<std::mutex> lock(channel->mutex);
            Status observed=owned_fixture::status();dp::comm::SourceId observed_source{};return ((observed=channel->observation.status),(observed_source=channel->observation.status_source),message_fields_present(observed))==true &&
              (*observed.run_id())==123U && (*observed.motion_epoch())==epoch && (*observed.core_state())==2U;});
        };
        if(!send_origin(48U) || !send_origin(cancel_origin_fault==6U ? 53U : 49U))return 410;
        std::array<std::uint8_t,32U> first{},motion{},safety{},terminal{};
        if(!publish_page(123U,0U,1U,1U,0U,seed_for(123U),&first) ||
           !wait_for([&]{return transport.statistics().raw_evidence_pages_written==1U;}))return 411;
        // 已接纳首CAN后，迟到的“零组Idle”不能改写已冻结的attempt来源。
        if(cancel_origin_fault==3U && !send_origin(50U))return 412;
        if(!publish_page(123U,1U,1U,2U,0U,first,&motion) ||
           !publish_page(123U,2U,2U,3U,0U,motion,&safety) ||
           !publish_page(123U,3U,4U,3U,7U,safety,&terminal) ||
           !wait_for([&]{return transport.statistics().raw_evidence_last_terminal_written_run==123U;}))return 413;
        const auto receipt=channel->raw_evidence_terminal_flushed();
        std::fprintf(stderr,"cancel-origin: pre_motion=%u first_can=50 safety=52 terminal=%u safety_valid=%u\n",
          cancel_origin_fault==6U ? 53U : 49U,
          cancel_origin_fault==1U ? 48U : cancel_origin_fault==2U || cancel_origin_fault==3U ? 50U : cancel_origin_fault==6U ? 53U : 49U,
          receipt.safety_hold_valid);
        if(cancel_origin_fault==0U && receipt.safety_hold_valid) {
          auto hold=valid;hold.status.motion_epoch()=(52U);hold.state.motion_epoch()=(52U);
          hold.status.core_state()=(4U);hold.state.core_state()=(4U);hold.state.hold_ready()=(true);
          hold.state.state_seq()=(100U);hold.status.final_state()=(7U);hold.status.group_count()=(3U);
          std::uint8_t digest[32]{};std::copy(terminal.begin(),terminal.end(),digest);
          hold.status.evidence_chain_sha256().emplace().assign(digest,32);
          if(!send(hold) || !wait_for([&]{std::lock_guard<std::mutex> lock(channel->mutex);
            State observed=owned_fixture::state();dp::comm::SourceId observed_source{};return ((observed=channel->observation.state),(observed_source=channel->observation.state_source),message_fields_present(observed))==true && (*observed.state_seq())==100U;}))return 416;
          std::array<std::uint8_t,32U> next_first{},next_terminal{};
          if(!publish_page(124U,0U,1U,1U,0U,seed_for(124U),&next_first) ||
             !wait_for([&]{return transport.statistics().raw_evidence_pages_written==5U ||
               transport.statistics().raw_evidence_failure!=0U;}) ||
             transport.statistics().raw_evidence_failure!=0U)return 417;
          hold.status.run_id()=(124U);hold.status.final_state()=(0U);hold.status.group_count()=(1U);
          std::fill(std::begin(digest),std::end(digest),0U);hold.status.evidence_chain_sha256().emplace().assign(digest,32);
          hold.state.state_seq()=(101U);
          if(!send(hold) || !wait_for([&]{std::lock_guard<std::mutex> lock(channel->mutex);
            Status observed=owned_fixture::status();dp::comm::SourceId observed_source{};return ((observed=channel->observation.status),(observed_source=channel->observation.status_source),message_fields_present(observed))==true && (*observed.run_id())==124U;}) ||
             !publish_page(124U,1U,4U,1U,6U,next_first,&next_terminal) ||
             !wait_for([&]{return transport.statistics().raw_evidence_last_terminal_written_run==124U;}))return 418;
          hold.status.final_state()=(6U);hold.status.motion_complete()=(true);hold.state.state_seq()=(102U);
          std::copy(next_terminal.begin(),next_terminal.end(),digest);hold.status.evidence_chain_sha256().emplace().assign(digest,32);
          if(!send(hold) || !wait_for([&]{std::lock_guard<std::mutex> lock(channel->mutex);
            Status observed=owned_fixture::status();dp::comm::SourceId observed_source{};return ((observed=channel->observation.status),(observed_source=channel->observation.status_source),message_fields_present(observed))==true && (*observed.run_id())==124U && (*observed.final_state())==6U;}))return 419;
        }
        transport.stop();
        return receipt.boot_epoch==7U && receipt.session_id==8U && receipt.run_id==123U &&
          receipt.chain_valid && receipt.final_state==7U && receipt.safety_hold_motion_epoch==52U &&
          receipt.safety_hold_valid==(cancel_origin_fault==0U) &&
          transport.statistics().raw_evidence_failure==0U ? 0 : 414;
      }
      /**
       * @brief 多个后继run在Status落后时依赖前run终态flush精确绑定。
       * 43依次允许run1→2→3；44旧run回执、45错boot、46错session、47错run、48缺终态均失败且不得多落页。
       * 49迟到终态Status链错误使failure=6，50迟到正确链可匹配；验证原页/Status异步顺序，非物理动作顺序。
       */
      if (raw_order_case >= 43U && raw_order_case <= 50U) {
        valid.status.core_state()=(3U);
        valid.status.final_state()=(0U);
        valid.status.motion_epoch()=(3U);
        valid.status.management_epoch()=(7U);
        valid.state.motion_epoch()=(3U);
        valid.state.management_epoch()=(7U);
        std::array<std::uint8_t, 32U> first_page{}, first_terminal{};
        std::array<std::uint8_t, 32U> second_page{}, second_terminal{}, third_page{};
        if (!send(valid) || !wait_for([&] {
              std::lock_guard<std::mutex> lock(channel->mutex);
              Status observed=owned_fixture::status();dp::comm::SourceId observed_source{};
              return ((observed=channel->observation.status),(observed_source=channel->observation.status_source),message_fields_present(observed)) == true && (*observed.run_id()) == 1U;
            }) || !publish_page(1U, 0U, 1U, 1U, 0U, seed_for(1U), &first_page) ||
            !wait_for([&] { return transport.statistics().raw_evidence_pages_written == 1U; }) ||
            !publish_page(1U, 1U, 4U, 1U, 6U, first_page, &first_terminal) ||
            !wait_for([&] { return transport.statistics().raw_evidence_last_terminal_written_run == 1U; }))
          return 323;
        const auto stale_receipt = channel->raw_evidence_terminal_flushed();
        if (!publish_page(2U, 0U, 1U, 1U, 0U, seed_for(2U), &second_page) ||
            !wait_for([&] { return transport.statistics().raw_evidence_pages_written == 3U ||
                transport.statistics().raw_evidence_failure != 0U; }) ||
            transport.statistics().raw_evidence_failure != 0U) return 324;
        auto second_status = valid;
        second_status.send_state = false;
        second_status.status.run_id()=(2U);
        second_status.status.motion_epoch()=(5U);
        second_status.status.management_epoch()=(8U);
        if (!send(second_status) || !wait_for([&] {
              std::lock_guard<std::mutex> lock(channel->mutex);
              Status observed=owned_fixture::status();dp::comm::SourceId observed_source{};
              return ((observed=channel->observation.status),(observed_source=channel->observation.status_source),message_fields_present(observed)) == true && (*observed.run_id()) == 2U &&
                  (*observed.core_state()) == 3U && (*observed.final_state()) == 0U &&
                  (*observed.motion_epoch()) == 5U;
            })) return 325;
        // 真实回程时序：后继run已临时绑定；完成Status早于终页flush，
        // flush后首个刷新页不能要求设备重复发送同一份Status。
        if (complete_status_before_flush_case) cancel_flush_armed=true;
        if (complete_status_before_flush_case) {
          if (!publish_page(2U, 1U, 4U, 1U, 6U, second_page, &second_terminal) ||
              !wait_for([&] { return cancel_flush_entered.load(); })) return 420;
          auto early_final=second_status;
          early_final.status.core_state()=(4U);early_final.status.final_state()=(6U);
          early_final.status.motion_complete()=(true);
          std::uint8_t digest[32]{};std::copy(second_terminal.begin(),second_terminal.end(),digest);
          early_final.status.evidence_chain_sha256().emplace().assign(digest,32);
          if (!send(early_final) || !wait_for([&] {
              std::lock_guard<std::mutex> lock(channel->mutex);Status observed=owned_fixture::status();dp::comm::SourceId observed_source{};
              return ((observed=channel->observation.status),(observed_source=channel->observation.status_source),message_fields_present(observed))==true &&
                  (*observed.run_id())==2U && (*observed.final_state())==6U;
            })) {cancel_flush_resume=true;return 421;}
          cancel_flush_resume=true;
        }
        if (raw_order_case != 48U &&
            ((!complete_status_before_flush_case && !publish_page(2U, 1U, 4U, 1U, 6U, second_page, &second_terminal)) ||
             !wait_for([&] {
               const auto receipt = channel->raw_evidence_terminal_flushed();
               return transport.statistics().raw_evidence_last_terminal_written_run == 2U &&
                   receipt.run_id == 2U && receipt.final_state == 6U &&
                   receipt.chain_valid && receipt.motion_complete;
             }))) return 326;
        {
          std::lock_guard<std::mutex> lock(channel->mutex);
          Status observed=owned_fixture::status();dp::comm::SourceId observed_source{};
          if (((observed=channel->observation.status),(observed_source=channel->observation.status_source),message_fields_present(observed)) != true || (*observed.run_id()) != 2U ||
              (!complete_status_before_flush_case && (*observed.final_state()) != 0U) ||
              (!complete_status_before_flush_case && std::any_of(observed.evidence_chain_sha256()->begin(),
                  observed.evidence_chain_sha256()->end(),
                  [](std::uint8_t byte) { return byte != 0U; }))) return 327;
        }
        if (raw_order_case >= 44U && raw_order_case <= 47U) {
          auto receipt = raw_order_case == 44U ? stale_receipt :
              channel->raw_evidence_terminal_flushed();
          if (raw_order_case == 45U) receipt.boot_epoch = 8U;
          if (raw_order_case == 46U) receipt.session_id = 9U;
          if (raw_order_case == 47U) receipt.run_id = 3U;
          channel->publish_raw_evidence_terminal_flushed(receipt);
        }
        const auto before_third = transport.statistics();
        if (!publish_page(3U, 0U, 1U, 1U, 0U, seed_for(3U), &third_page) ||
            !wait_for([&] {
              const auto stats = transport.statistics();
              return stats.raw_evidence_pages_written > before_third.raw_evidence_pages_written ||
                  stats.raw_evidence_failure != 0U;
            })) return 328;
        if (raw_order_case == 49U || raw_order_case == 50U) {
          auto late_status = second_status;
          late_status.status.core_state()=(4U);
          late_status.status.final_state()=(6U);
          late_status.status.motion_complete()=(true);
          std::uint8_t late_digest[32]{};
          std::copy(second_terminal.begin(), second_terminal.end(), late_digest);
          if (raw_order_case == 49U) late_digest[0] ^= 1U;
          late_status.status.evidence_chain_sha256().emplace().assign(late_digest,32);
          const auto before_late = transport.statistics().status_messages;
          if (!send(late_status) || !wait_for([&] {
                const auto stats = transport.statistics();
                return stats.status_messages > before_late ||
                    stats.raw_evidence_failure != 0U;
              })) return 335;
          if (raw_order_case == 49U) {
            const auto rejected = transport.statistics();
            transport.stop();
            const auto stopped = transport.statistics();
            return rejected.raw_evidence_failure == 6U &&
                stopped.raw_evidence_failure == 6U &&
                stopped.raw_evidence_pages_written == 5U ? 0 : 336;
          }
          if (!wait_for([&] {
                std::lock_guard<std::mutex> lock(channel->mutex);
                Status observed=owned_fixture::status();dp::comm::SourceId observed_source{};
                return ((observed=channel->observation.status),(observed_source=channel->observation.status_source),message_fields_present(observed)) == true && (*observed.run_id()) == 2U &&
                    (*observed.final_state()) == 6U &&
                    std::equal(second_terminal.begin(), second_terminal.end(),
                               (observed.evidence_chain_sha256()->data()));
              }) || transport.statistics().raw_evidence_failure != 0U) return 337;
        }
        if (raw_order_case >= 44U && raw_order_case <= 48U) {
          transport.stop();
          const auto result = transport.statistics();
          return result.raw_evidence_failure == 6U &&
              result.raw_evidence_pages_written == before_third.raw_evidence_pages_written ? 0 : 329;
        }
        if (transport.statistics().raw_evidence_failure != 0U ||
            transport.statistics().raw_evidence_pages_written != 5U) return 330;
        auto third_status = second_status;
        third_status.status.run_id()=(3U);
        third_status.status.core_state()=(4U);
        third_status.status.management_epoch()=(9U);
        if (!send(third_status) || !wait_for([&] {
              std::lock_guard<std::mutex> lock(channel->mutex);
              Status observed=owned_fixture::status();dp::comm::SourceId observed_source{};
              return ((observed=channel->observation.status),(observed_source=channel->observation.status_source),message_fields_present(observed)) == true && (*observed.run_id()) == 3U;
            })) return 331;
        std::array<std::uint8_t, 32U> third_terminal{};
        if (!publish_page(3U, 1U, 4U, 1U, 6U, third_page, &third_terminal) ||
            !wait_for([&] { return transport.statistics().raw_evidence_last_terminal_written_run == 3U; }))
          return 332;
        auto third_final = third_status;
        third_final.status.final_state()=(6U);
        third_final.status.motion_complete()=(true);
        std::uint8_t third_digest[32]{};
        std::copy(third_terminal.begin(), third_terminal.end(), third_digest);
        third_final.status.evidence_chain_sha256().emplace().assign(third_digest,32);
        if (!send(third_final) || !wait_for([&] {
              std::lock_guard<std::mutex> lock(channel->mutex);
              Status observed=owned_fixture::status();dp::comm::SourceId observed_source{};
              return ((observed=channel->observation.status),(observed_source=channel->observation.status_source),message_fields_present(observed)) == true && (*observed.run_id()) == 3U &&
                  (*observed.final_state()) == 6U &&
                  std::equal(third_terminal.begin(), third_terminal.end(),
                             (observed.evidence_chain_sha256()->data()));
            })) return 333;
        transport.stop();
        const auto result = transport.statistics();
        return result.raw_evidence_failure == 0U &&
            result.raw_evidence_pages_written == 6U &&
            result.raw_evidence_last_terminal_written_run == 3U ? 0 : 334;
      }
      /**
       * @brief 完成Hold刷新首原页可早于后继Status，但必须有已flush的前run完成证据。
       * 34正常，35/36错回执boot/session，37无前终态，38零motion，39跳run，40部分mask，41无motion_complete，42清回执均拒绝。
       * 51..58进一步让Status过250ms而State保鲜：51允许；52/53/54错身份，55坏链，56State也失鲜，57非Hold，58回执chain_valid=false均拒绝。
       */
      if ((raw_order_case >= 34U && raw_order_case <= 42U) ||
          (raw_order_case >= 51U && raw_order_case <= 58U)) {
        valid.status.core_state()=(3U);
        valid.status.final_state()=(0U);
        valid.status.motion_epoch()=(3U);
        valid.status.management_epoch()=(7U);
        valid.state.motion_epoch()=(3U);
        valid.state.management_epoch()=(7U);
        if (raw_order_case >= 51U) {
          valid.state.core_state()=(4U);
          valid.state.hold_ready()=(true);
          root_fixture::set_position_quality(valid.state,63U);
        }
        std::array<std::uint8_t, 32U> first_group{}, first_terminal{}, next_group{}, next_terminal{};
        if (!send(valid) || !wait_for([&] {
              std::lock_guard<std::mutex> lock(channel->mutex);
              Status observed=owned_fixture::status();dp::comm::SourceId observed_source{};
              return ((observed=channel->observation.status),(observed_source=channel->observation.status_source),message_fields_present(observed)) == true && (*observed.run_id()) == 1U &&
                  (*observed.core_state()) == 3U && (*observed.final_state()) == 0U &&
                  (*observed.motion_epoch()) == 3U && (*observed.management_epoch()) == 7U;
            }) || !publish_page(1U, 0U, 1U, 1U, 0U, seed_for(1U), &first_group) ||
            !wait_for([&] { return transport.statistics().raw_evidence_pages_written == 1U; }))
          return 307;
        if (raw_order_case != 37U &&
            (!publish_page(1U, 1U, 4U, 1U, 6U, first_group, &first_terminal) ||
             !wait_for([&] {
               const auto receipt = channel->raw_evidence_terminal_flushed();
               return transport.statistics().raw_evidence_last_terminal_written_run == 1U &&
                   receipt.boot_epoch == 7U && receipt.session_id == 8U &&
                   receipt.run_id == 1U && receipt.attempt_kind == 1U &&
                   receipt.final_state == 6U && receipt.chain_valid &&
                   receipt.motion_complete == (raw_order_case != 41U);
             }))) return 308;
        if (raw_order_case == 35U || raw_order_case == 36U || raw_order_case == 42U ||
            (raw_order_case >= 52U && raw_order_case <= 54U) || raw_order_case == 58U) {
          // Isolate receipt identity and publication order after the real terminal flush.
          auto receipt = channel->raw_evidence_terminal_flushed();
          if (raw_order_case == 35U || raw_order_case == 52U) receipt.boot_epoch = 8U;
          if (raw_order_case == 36U || raw_order_case == 53U) receipt.session_id = 9U;
          if (raw_order_case == 54U) receipt.run_id = 3U;
          if (raw_order_case == 58U) receipt.chain_valid = false;
          if (raw_order_case == 42U) receipt = {};
          channel->publish_raw_evidence_terminal_flushed(receipt);
        }
        {
          const auto before = transport.statistics();
          std::lock_guard<std::mutex> lock(channel->mutex);
          Status observed=owned_fixture::status();dp::comm::SourceId observed_source{};
          if (((observed=channel->observation.status),(observed_source=channel->observation.status_source),message_fields_present(observed)) != true || (*observed.run_id()) != 1U ||
              (*observed.core_state()) != 3U || (*observed.final_state()) != 0U ||
              (*observed.motion_epoch()) != 3U || (*observed.management_epoch()) != 7U ||
              before.raw_evidence_last_terminal_written_run !=
                  (raw_order_case == 37U ? 0U : 1U)) return 316;
        }
        if (raw_order_case >= 51U) {
          auto ongoing = valid;
          ongoing.send_status = false;
          if (raw_order_case == 57U) {
            ongoing.state.core_state()=(6U);
            ongoing.state.hold_ready()=(false);
          }
          /**
           * @brief 用Host经过300ms刻意陈旧Status；20ms推进State，选定负例则不保鲜或改为非Hold。
           * 末尾直接核state/status接收年龄，避免只凭sleep猜测各谓词实际达到。
           */
          const auto start = transport_steady_now_us();
          std::uint32_t seq = 2U;
          while (transport_steady_now_us() - start <= 300000U) {
            if (raw_order_case != 56U) {
              ongoing.state.state_seq()=(seq++);
              if (!send(ongoing) || !wait_for([&] {
                    std::lock_guard<std::mutex> lock(channel->mutex);
                    State observed=owned_fixture::state();dp::comm::SourceId observed_source{};
                    return ((observed=channel->observation.state),(observed_source=channel->observation.state_source),message_fields_present(observed)) == true &&
                        (*observed.state_seq()) == (*ongoing.state.state_seq());
                  })) return 338;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
          }
          if (raw_order_case != 56U) {
            ongoing.state.state_seq()=(seq);
            if (!send(ongoing) || !wait_for([&] {
                  std::lock_guard<std::mutex> lock(channel->mutex);
                  State observed=owned_fixture::state();dp::comm::SourceId observed_source{};
                  return ((observed=channel->observation.state),(observed_source=channel->observation.state_source),message_fields_present(observed)) == true &&
                      (*observed.state_seq()) == (*ongoing.state.state_seq());
                })) return 339;
          }
          std::lock_guard<std::mutex> lock(channel->mutex);
          const auto now = transport_steady_now_us();
          State observed_state=owned_fixture::state();dp::comm::SourceId observed_state_source{};
          if (((observed_state=channel->observation.state),(observed_state_source=channel->observation.state_source),message_fields_present(observed_state)) != true ||
              (*observed_state.core_state()) != (raw_order_case == 57U ? 6U : 4U) ||
              (*observed_state.hold_ready()) != (raw_order_case != 57U) ||
              root_fixture::position_quality(observed_state) != 63U) return 341;
          if (channel->observation.status_received_us == 0U ||
              now - channel->observation.status_received_us <= 250000U ||
              (raw_order_case != 56U &&
               (channel->observation.state_received_us == 0U ||
                now - channel->observation.state_received_us >= 100000U)) ||
              (raw_order_case == 56U &&
               (channel->observation.state_received_us == 0U ||
                now - channel->observation.state_received_us <= 250000U))) return 340;
        }
        const auto before_page = transport.statistics();
        if (!publish_page(raw_order_case == 39U ? 3U : 2U, 0U, 1U, 1U, 0U,
                          seed_for(raw_order_case == 39U ? 3U : 2U), &next_group) ||
            !wait_for([&] {
              const auto observed = transport.statistics();
              return observed.raw_evidence_pages_written > before_page.raw_evidence_pages_written ||
                  observed.raw_evidence_failure != 0U;
            })) return 310;
        const auto first_page = transport.statistics();
        if (raw_order_case != 34U && raw_order_case != 51U) {
          transport.stop();
          return first_page.raw_evidence_failure == (raw_order_case == 55U ? 2U : 6U) &&
              first_page.raw_evidence_pages_written == before_page.raw_evidence_pages_written ? 0 : 311;
        }
        if (first_page.raw_evidence_failure != 0U ||
            first_page.raw_evidence_pages_written != 3U ||
            first_page.status_messages != before_page.status_messages ||
            first_page.raw_evidence_last_terminal_written_run != 1U) {
          std::fprintf(stderr, "hold refresh first page: failure=%u pages=%llu status_delta=%llu terminal=%llu\n",
              static_cast<unsigned>(first_page.raw_evidence_failure),
              static_cast<unsigned long long>(first_page.raw_evidence_pages_written),
              static_cast<unsigned long long>(first_page.status_messages - before_page.status_messages),
              static_cast<unsigned long long>(first_page.raw_evidence_last_terminal_written_run));
          return 312;
        }
        auto next_status = valid;
        next_status.send_state = false;
        next_status.status.run_id()=(2U);
        next_status.status.core_state()=(4U);
        if (!send(next_status) || !wait_for([&] {
              std::lock_guard<std::mutex> lock(channel->mutex);
              Status observed=owned_fixture::status();dp::comm::SourceId observed_source{};
              return ((observed=channel->observation.status),(observed_source=channel->observation.status_source),message_fields_present(observed)) == true && (*observed.run_id()) == 2U;
            }) || !publish_page(2U, 1U, 4U, 1U, 6U, next_group, &next_terminal) ||
            !wait_for([&] {
              return transport.statistics().raw_evidence_last_terminal_written_run == 2U;
            })) return 313;
        auto final_status = next_status;
        final_status.status.final_state()=(6U);
        std::uint8_t digest[32]{};
        std::copy(next_terminal.begin(), next_terminal.end(), digest);
        final_status.status.evidence_chain_sha256().emplace().assign(digest,32);
        if (!send(final_status) || !wait_for([&] {
              std::lock_guard<std::mutex> lock(channel->mutex);
              Status observed=owned_fixture::status();dp::comm::SourceId observed_source{};
              return ((observed=channel->observation.status),(observed_source=channel->observation.status_source),message_fields_present(observed)) == true && (*observed.run_id()) == 2U &&
                  (*observed.final_state()) == 6U;
            })) return 314;
        transport.stop();
        return transport.statistics().raw_evidence_failure == 0U &&
            transport.statistics().raw_evidence_pages_written == 4U ? 0 : 315;
      }
      /**
       * @brief 取消后release/刷新首页早到组，按原run及已flush安全Hold证明接纳后继。
       * 20..23为独立release attempt（kind3/parent），24..33为新motion刷新；59..70附加安全Hold页与陈旧Status/新鲜State组合。
       * 各负例的确切字段变化见main编号表和publish_page，而非仅用“先到”名称推断都应通过。
       */
      if ((raw_order_case >= 20U && raw_order_case <= 33U) ||
          (raw_order_case >= 59U && raw_order_case <= 70U)) {
        const bool refresh = raw_order_case >= 24U;
        const auto motion_run = refresh ? 123U : 121U;
        const auto next_run = refresh ? 124U : 122U;
        std::array<std::uint8_t, 32U> motion_group{}, motion_terminal{};
        std::array<std::uint8_t, 32U> release_page{}, release_terminal{};
        if (refresh) valid.status.core_state()=(cancel_hold_status_variant ? 4U : 7U);
        if (!send(valid) || !wait_for([&] {
              std::lock_guard<std::mutex> lock(channel->mutex);
              Status observed=owned_fixture::status();dp::comm::SourceId observed_source{};
              return ((observed=channel->observation.status),(observed_source=channel->observation.status_source),message_fields_present(observed)) == true && (*observed.run_id()) == motion_run &&
                  (!refresh || ((*observed.core_state()) == (cancel_hold_status_variant ? 4U : 7U) && (*observed.final_state()) == 0U &&
                    std::all_of(observed.evidence_chain_sha256()->begin(),
                        observed.evidence_chain_sha256()->end(),
                        [](auto byte) { return byte == 0U; })));
            }) || !publish_page(motion_run, 0U, raw_order_case >= 59U ? 1U : 2U, 1U, 0U, seed_for(motion_run), &motion_group) ||
            !wait_for([&] { return transport.statistics().raw_evidence_pages_written == 1U; })) return 288;
        /**
         * @brief 当前取消run先motion页，再安全保持页，最后final7；共享flush必须chain_valid且hold_epoch10。
         * 终态motion_epoch仍指原attempt9，安全保持页使用后继10；不能用终态本身的epoch代替已完整SENT的hold epoch。
         */
        if (raw_order_case >= 59U) {
          std::array<std::uint8_t, 32U> safety_group{};
          if (!publish_page(motion_run, 1U, 2U, 2U, 0U, motion_group, &safety_group) ||
              !wait_for([&] { return transport.statistics().raw_evidence_pages_written == 2U; }))
            return 288;
          motion_group = safety_group;
        }
        if(cancel_pending_flush_case) {
          // 与实机相同：旧Stopping Status超过254480us，但Hold State持续新鲜。
          auto hold=valid;hold.send_status=false;hold.state.core_state()=(4U);
          hold.state.hold_ready()=(true);root_fixture::set_position_quality(hold.state,63U);
          if(cancel_pending_flush_fault==2U) {hold.state.core_state()=(3U);hold.state.hold_ready()=(false);}
          std::uint64_t status_at=0U;
          {std::lock_guard<std::mutex> lock(channel->mutex);status_at=channel->observation.status_received_us;}
          std::uint32_t sequence=2U;
          do {
            hold.state.state_seq()=(sequence++);
            if(!send(hold) || !wait_for([&]{std::lock_guard<std::mutex> lock(channel->mutex);
              State state=owned_fixture::state();dp::comm::SourceId state_source{};return ((state=channel->observation.state),(state_source=channel->observation.state_source),message_fields_present(state))==true && (*state.state_seq())==(*hold.state.state_seq());}))return 390;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
          } while(transport_steady_now_us()-status_at<254480U);
          struct ResumeFlush {~ResumeFlush(){cancel_flush_resume=true;cancel_receipt_resume=true;}} resume_flush;
          cancel_flush_armed=true;
          if(!publish_page(motion_run,2U,4U,2U,7U,motion_group,&motion_terminal) ||
             !wait_for([]{return cancel_flush_entered.load();}))return 391;
          if(transport.statistics().raw_evidence_last_terminal_written_run==motion_run ||
             channel->raw_evidence_terminal_flushed().run_id==motion_run)return 392;
          if(cancel_pending_flush_fault==7U) {
            capture_cancel_receipt_mutex=true;
            (void)channel->raw_evidence_terminal_flushed();
            capture_cancel_receipt_mutex=false;
            if(cancel_receipt_mutex.load()==nullptr)return 406;
            cancel_receipt_armed=true;
          }
          if(!publish_page(next_run,0U,1U,1U,0U,seed_for(next_run),&release_page))return 393;
          if(cancel_pending_flush_fault==7U) {
            if(!wait_for([]{return cancel_receipt_entered.load();}))return 407;
            cancel_flush_resume=true;
            if(!wait_for([&]{return transport.statistics().raw_evidence_last_terminal_written_run==motion_run;}))return 408;
            std::fprintf(stderr,"cancel-consumer-snapshot: old receipt copied, writer watermark=%llu before getter returns\n",
              (unsigned long long)transport.statistics().raw_evidence_last_terminal_written_run);
            cancel_receipt_resume=true;
          }
          // 后发State到达确认worker已处理前面的原页；普通模式writer仍暂停，
          // consumer_snapshot模式则已确定性完成writer发布。
          hold.state.state_seq()=(sequence++);
          if(!send(hold) || !wait_for([&]{std::lock_guard<std::mutex> lock(channel->mutex);
            State state=owned_fixture::state();dp::comm::SourceId state_source{};return ((state=channel->observation.state),(state_source=channel->observation.state_source),message_fields_present(state))==true && (*state.state_seq())==(*hold.state.state_seq());}))return 394;
          const auto pending=transport.statistics();
          std::fprintf(stderr,"cancel-pending-window: status_age=%llu failure=%u terminal_written=%llu\n",
            (unsigned long long)(transport_steady_now_us()-status_at),unsigned(pending.raw_evidence_failure),
            (unsigned long long)pending.raw_evidence_last_terminal_written_run);
          if(cancel_pending_flush_fault>=1U && cancel_pending_flush_fault<=4U) {
            cancel_flush_resume=true;transport.stop();
            return pending.raw_evidence_failure==6U ? 0 : 401;
          }
          if(pending.raw_evidence_failure!=0U)return 395;
          if(cancel_pending_flush_fault!=7U && channel->raw_evidence_terminal_flushed().run_id==motion_run)return 396;
          cancel_flush_resume=true;
          if(cancel_pending_flush_fault==6U) {
            if(!wait_for([&]{return transport.statistics().raw_evidence_failure!=0U;}))return 402;
            transport.stop();
            return transport.statistics().raw_evidence_failure==5U &&
              channel->raw_evidence_terminal_flushed().run_id!=motion_run ? 0 : 403;
          }
          if(!wait_for([&]{return transport.statistics().raw_evidence_pages_written==4U;}))return 397;
          auto successor=hold;successor.send_status=true;successor.status.run_id()=(next_run);
          successor.status.core_state()=(4U);successor.status.final_state()=(0U);
          successor.state.state_seq()=(sequence++);
          if(!send(successor) || !wait_for([&]{std::lock_guard<std::mutex> lock(channel->mutex);
            Status status=owned_fixture::status();dp::comm::SourceId status_source{};return ((status=channel->observation.status),(status_source=channel->observation.status_source),message_fields_present(status))==true && (*status.run_id())==next_run;}))return 398;
          if(!publish_page(next_run,1U,4U,1U,6U,release_page,&release_terminal) ||
             !wait_for([&]{return transport.statistics().raw_evidence_last_terminal_written_run==next_run;}))return 399;
          successor.status.final_state()=(6U);
          std::uint8_t final_chain[32]{};std::copy(release_terminal.begin(),release_terminal.end(),final_chain);
          if(cancel_pending_flush_fault==5U)final_chain[0]^=1U;
          successor.status.evidence_chain_sha256().emplace().assign(final_chain,32);successor.state.state_seq()=(sequence++);
          if(!send(successor) || !wait_for([&]{
            if(transport.statistics().raw_evidence_failure!=0U)return true;
            std::lock_guard<std::mutex> lock(channel->mutex);Status status=owned_fixture::status();dp::comm::SourceId status_source{};
            return ((status=channel->observation.status),(status_source=channel->observation.status_source),message_fields_present(status))==true &&
              (*status.run_id())==next_run && (*status.final_state())==6U;
          }))return 404;
          transport.stop();
          return transport.statistics().raw_evidence_failure==(cancel_pending_flush_fault==5U ? 6U : 0U) ? 0 : 400;
        }
        if (!publish_page(motion_run, raw_order_case >= 59U ? 2U : 1U, 4U,
                          raw_order_case >= 59U ? 2U : 1U, 7U, motion_group, &motion_terminal) ||
            !wait_for([&] { return transport.statistics().raw_evidence_last_terminal_written_run == motion_run; }))
          return 288;
        if (raw_order_case >= 59U) {
          const auto receipt = channel->raw_evidence_terminal_flushed();
          if (receipt.boot_epoch != 7U || receipt.session_id != 8U ||
              receipt.run_id != motion_run || receipt.final_state != 7U ||
              !receipt.chain_valid || !receipt.safety_hold_valid ||
              receipt.safety_hold_motion_epoch != 10U) return 357;
          /**
           * @brief 显式终态Status变体把已采集final7摘要写入Hold Status，再分别破坏final/chain/failure。
           * 检查后继页拒绝，所以不把“Status final7”单字段当作安全保持证明。
           */
          if(cancel_status_terminal7_variant){
            auto hold_terminal_status=valid;hold_terminal_status.send_state=false;
            hold_terminal_status.status.core_state()=(4U);hold_terminal_status.status.final_state()=(7U);
            std::uint8_t digest[32]{};std::copy(motion_terminal.begin(),motion_terminal.end(),digest);
            hold_terminal_status.status.evidence_chain_sha256().emplace().assign(digest,32);
            if(cancel_status_bad_flags>=1U && cancel_status_bad_flags<=4U)hold_terminal_status.status.final_state()=(cancel_status_bad_flags==1U?4U:cancel_status_bad_flags==2U?5U:cancel_status_bad_flags==3U?6U:9U);
            if(cancel_status_bad_flags==5U){digest[0]^=1U;hold_terminal_status.status.evidence_chain_sha256().emplace().assign(digest,32);}
            if(cancel_status_bad_flags==6U){std::fill(std::begin(digest),std::end(digest),0U);hold_terminal_status.status.evidence_chain_sha256().emplace().assign(digest,32);}
            if(cancel_status_bad_flags==7U)hold_terminal_status.status.failure_source()=(1U);
            if(cancel_status_bad_flags==8U)hold_terminal_status.status.failure_axis_mask()=(1U);
            if(cancel_status_bad_flags==9U)hold_terminal_status.status.first_failure_reason()=(1U);
            if(!send(hold_terminal_status) || !wait_for([&]{std::lock_guard<std::mutex> lock(channel->mutex);Status observed=owned_fixture::status();dp::comm::SourceId observed_source{};return ((observed=channel->observation.status),(observed_source=channel->observation.status_source),message_fields_present(observed))==true && (*observed.core_state())==4U && (*observed.final_state())==(*hold_terminal_status.status.final_state()) && (*observed.run_id())==motion_run;}))return 363;
          }
          /**
           * @brief 先得到真实本地writer终态flush，再只改共享回执某字段隔离资格负例。
           * 这类测试故意公开发布错误回执，不等于生产writer会自然产生该错误组合。
           */
          auto invalid_receipt = receipt;
          if (raw_order_case == 60U) invalid_receipt.boot_epoch = 8U;
          if (raw_order_case == 61U) invalid_receipt.session_id = 9U;
          if (raw_order_case == 62U) invalid_receipt.run_id = 122U;
          if (raw_order_case == 63U) invalid_receipt.chain_valid = false;
          if (raw_order_case == 68U) invalid_receipt.safety_hold_valid = false;
          if (raw_order_case == 69U) invalid_receipt.safety_hold_motion_epoch = 11U;
          if ((raw_order_case >= 60U && raw_order_case <= 63U) ||
              raw_order_case == 68U || raw_order_case == 69U)
            channel->publish_raw_evidence_terminal_flushed(invalid_receipt);
          auto ongoing = valid;
          ongoing.send_status = false;
          ongoing.state.core_state()=(4U);
          ongoing.state.hold_ready()=(true);
          root_fixture::set_position_quality(ongoing.state,63U);
          if (raw_order_case == 65U) ongoing.state.core_state()=(cancel_state_stopping_notready?7U:6U);
          if (raw_order_case == 66U || cancel_state_stopping_notready) ongoing.state.hold_ready()=(false);
          if (raw_order_case == 67U) ongoing.state.management_epoch()=(1U);
          const auto start = transport_steady_now_us();
          std::uint32_t seq = 2U;
          std::uint64_t accepted_state_at = 0U;
          while (transport_steady_now_us() - start <= 300000U) {
            if (raw_order_case != 64U || seq == 2U) {
              ongoing.state.state_seq()=(seq++);
              if (!send(ongoing) || !wait_for([&] {
                    std::lock_guard<std::mutex> lock(channel->mutex);
                    State observed=owned_fixture::state();dp::comm::SourceId observed_source{};
                    return ((observed=channel->observation.state),(observed_source=channel->observation.state_source),message_fields_present(observed)) == true &&
                        (*observed.state_seq()) == (*ongoing.state.state_seq()) &&
                        (accepted_state_at = channel->observation.state_received_us) != 0U;
                  })) return 358;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
          }
          std::lock_guard<std::mutex> lock(channel->mutex);
          const auto now = transport_steady_now_us();
          State observed=owned_fixture::state();dp::comm::SourceId observed_source{};
          /**
           * @brief State失鲜组合负例显式检查stale或heartbeat Stop已经发生。
           * 因此通过只证明组合安全拒绝，不声称单独隔离某一个binding年龄谓词。
           */
          if (raw_order_case == 64U) {
            if (accepted_state_at == 0U || now - accepted_state_at <= 250000U) return 359;
          } else if (((observed=channel->observation.state),(observed_source=channel->observation.state_source),message_fields_present(observed)) != true ||
              (*observed.core_state()) != (raw_order_case == 65U ? (cancel_state_stopping_notready?7U:6U) : 4U) ||
              (*observed.hold_ready()) != (raw_order_case != 66U && !cancel_state_stopping_notready) ||
              (*observed.management_epoch()) != (raw_order_case == 67U ? 1U : 0U) ||
              root_fixture::position_quality(observed) != 63U ||
              (raw_order_case != 64U &&
               now - channel->observation.state_received_us >= 100000U) ||
              (raw_order_case == 64U &&
               now - channel->observation.state_received_us <= 250000U) ||
              now - channel->observation.status_received_us <= 250000U) return 359;
          std::fprintf(stderr, "cancel stale-status witness: status_age=%llu state_age=%llu final=%u run=%llu chain=%u\n",
              (unsigned long long)(now - channel->observation.status_received_us),
              (unsigned long long)(now - channel->observation.state_received_us),
              receipt.final_state, (unsigned long long)receipt.run_id, receipt.chain_valid);
        }
        auto motion_final = valid;
        motion_final.send_state = false;
        motion_final.status.core_state()=(4U);
        motion_final.status.final_state()=(7U);
        std::uint8_t motion_digest[32]{};
        std::copy(motion_terminal.begin(), motion_terminal.end(), motion_digest);
        motion_final.status.evidence_chain_sha256().emplace().assign(motion_digest,32);
        if (!refresh && (!send(motion_final) || !wait_for([&] {
              std::lock_guard<std::mutex> lock(channel->mutex);
              Status observed=owned_fixture::status();dp::comm::SourceId observed_source{};
              return ((observed=channel->observation.status),(observed_source=channel->observation.status_source),message_fields_present(observed)) == true && (*observed.run_id()) == motion_run &&
                  (*observed.final_state()) == 7U;
            }))) return 289;
        if (raw_order_case == 64U) {
          // 组合State失鲜安全拒绝：不宣称隔离单个binding年龄谓词。
          const auto before_stale_page = transport.statistics();
          if (parameters.max_state_age_us != 250000U ||
              (before_stale_page.state_stale_stops == 0U &&
               before_stale_page.heartbeat_stops == 0U)) return 362;
        }
        const auto statuses_before_page = transport.statistics().status_messages;
        const auto release_run = raw_order_case == 23U ? 120U :
            raw_order_case == 27U ? 122U : next_run;
        const auto release_parent = raw_order_case == 21U ? 120U :
            raw_order_case == 23U ? 119U : refresh ? 0U : 121U;
        if (!publish_page(release_run, 0U,
                          raw_order_case == 28U ? 2U : refresh ? 1U : 3U,
                          raw_order_case == 29U ? 0U : 1U, 0U,
                          seed_for(release_run), &release_page,
                          refresh ? 1U : 3U, release_parent) ||
            !wait_for([&] { return transport.statistics().raw_evidence_pages_written == (raw_order_case >= 59U ? 4U : 3U) ||
                transport.statistics().raw_evidence_failure != 0U; })) return 290;
        const auto page_first = transport.statistics();
        if ((raw_order_case >= 60U && raw_order_case <= 70U) || cancel_status_bad_flags!=0U) {
          transport.stop();
          return page_first.raw_evidence_failure == 6U &&
              page_first.raw_evidence_pages_written == 3U ? 0 : 360;
        }
        if ((raw_order_case >= 20U && raw_order_case <= 23U) || raw_order_case == 27U) {
          transport.stop();
          return page_first.raw_evidence_failure == 6U &&
              page_first.raw_evidence_pages_written == 2U ? 0 : 296;
        }
        if (raw_order_case >= 28U && raw_order_case <= 31U) {
          std::fprintf(stderr, "invalid refresh first page: case=%u failure=%u pages=%llu\n",
              raw_order_case, static_cast<unsigned>(page_first.raw_evidence_failure),
              static_cast<unsigned long long>(page_first.raw_evidence_pages_written));
          transport.stop();
          return page_first.raw_evidence_failure == 6U &&
              page_first.raw_evidence_pages_written == 2U ? 0 : 299;
        }
        if (page_first.raw_evidence_failure != 0U ||
            page_first.raw_evidence_pages_written != (raw_order_case >= 59U ? 4U : 3U) ||
            page_first.status_messages != statuses_before_page) {
          std::fprintf(stderr, "page-first: failure=%u pages=%llu status_delta=%llu\n",
              static_cast<unsigned>(page_first.raw_evidence_failure),
              static_cast<unsigned long long>(page_first.raw_evidence_pages_written),
              static_cast<unsigned long long>(page_first.status_messages - statuses_before_page));
          return 291;
        }
        auto release_status = valid;
        release_status.send_state = false;
        release_status.status.run_id()=(next_run);
        release_status.status.parent_run_id()=(refresh ? 0U : 121U);
        release_status.status.attempt_kind()=(refresh ? 1U : 3U);
        release_status.status.core_state()=(refresh ? 4U : 1U);
        release_status.status.session_id()=(refresh ? 8U : 0U);
        release_status.status.reference_epoch()=(refresh ? 9U : 0U);
        release_status.status.motion_epoch()=(refresh ? 10U : 11U);
        release_status.status.management_epoch()=(refresh ? 0U : 1U);
        release_status.status.release_committed()=(!refresh);
        release_status.status.permit_remaining_us()=(0U);
        release_status.status.final_state()=(refresh ? 0U : 8U);
        if (raw_order_case == 25U || raw_order_case == 26U) {
          if (raw_order_case == 26U) release_status.status.session_id()=(9U);
          else release_status.status_source.boot_id=(8U);
          const auto statuses_before_bad_identity = transport.statistics().status_messages;
          if (!send(release_status) || !wait_for([&] {
                return transport.statistics().status_messages > statuses_before_bad_identity;
              })) return 297;
          if (raw_order_case == 25U && !wait_for([&] {
                return transport.statistics().raw_evidence_failure != 0U;
              })) return 297;
          transport.stop();
          const auto rejected = transport.statistics();
          return rejected.raw_evidence_last_terminal_written_run == motion_run &&
              (raw_order_case == 25U ? rejected.raw_evidence_failure == 6U :
                  rejected.raw_evidence_failure == 7U) ? 0 : 298;
        }
        if (!send(release_status) || !wait_for([&] {
              std::lock_guard<std::mutex> lock(channel->mutex);
              Status observed=owned_fixture::status();dp::comm::SourceId observed_source{};
              return ((observed=channel->observation.status),(observed_source=channel->observation.status_source),message_fields_present(observed)) == true && (*observed.run_id()) == next_run &&
                  (*observed.parent_run_id()) == (refresh ? 0U : 121U) &&
                  (*observed.attempt_kind()) == (refresh ? 1U : 3U) &&
                  (*observed.core_state()) == (refresh ? 4U : 1U) &&
                  (*observed.release_committed()) == !refresh;
            }) || transport.statistics().raw_evidence_failure != 0U ||
            transport.statistics().raw_evidence_last_terminal_written_run != motion_run)
          return 292;
        /**
         * @brief deliver=false先算后继终态链供Status发布，但不把终页送collector；随后补同字节终页再切下一run。
         * 验证提前Status不误判为已flush，也不在真实终页补齐后永久阻止后继。
         */
        if (raw_order_case == 33U) {
          if (!publish_page(next_run, 1U, 4U, 1U, 6U, release_page,
                            &release_terminal, 1U, 0U, false)) return 302;
          auto early_final = release_status;
          early_final.status.final_state()=(6U);
          early_final.status.motion_complete()=(true);
          std::uint8_t digest[32]{};
          std::copy(release_terminal.begin(), release_terminal.end(), digest);
          early_final.status.evidence_chain_sha256().emplace().assign(digest,32);
          if (!send(early_final) || !wait_for([&] {
                std::lock_guard<std::mutex> lock(channel->mutex);
                Status observed=owned_fixture::status();dp::comm::SourceId observed_source{};
                return ((observed=channel->observation.status),(observed_source=channel->observation.status_source),message_fields_present(observed)) == true && (*observed.run_id()) == next_run &&
                    (*observed.final_state()) == 6U;
              })) return 303;
          if (!publish_page(next_run, 1U, 4U, 1U, 6U, release_page,
                            &release_terminal, 1U, 0U) ||
              !wait_for([&] { return transport.statistics().raw_evidence_last_terminal_written_run == next_run; }))
            return 304;
          auto following = release_status;
          following.status.run_id()=(next_run + 1U);
          following.status.motion_epoch()=(11U);
          const auto before_following = transport.statistics().status_messages;
          if (!send(following) || !wait_for([&] {
                const auto stats = transport.statistics();
                return stats.status_messages > before_following || stats.raw_evidence_failure != 0U;
              })) return 305;
          const auto observed = transport.statistics();
          transport.stop();
          return observed.raw_evidence_failure == 0U &&
              observed.raw_evidence_last_terminal_written_run == next_run &&
              observed.status_messages > before_following ? 0 : 306;
        }
        if (!publish_page(next_run, 1U, 4U, 1U, refresh ? 6U : 8U,
                          release_page, &release_terminal,
                          refresh ? 1U : 3U, refresh ? 0U : 121U) ||
            !wait_for([&] { return transport.statistics().raw_evidence_last_terminal_written_run == next_run; }))
          return 293;
        auto release_final = release_status;
        if (refresh) release_final.status.final_state()=(6U);
        std::uint8_t release_digest[32]{};
        std::copy(release_terminal.begin(), release_terminal.end(), release_digest);
        if (raw_order_case == 32U) release_digest[0] ^= 1U;
        release_final.status.evidence_chain_sha256().emplace().assign(release_digest,32);
        if (raw_order_case == 32U) {
          if (!send(release_final) || !wait_for([&] {
                return transport.statistics().raw_evidence_failure != 0U;
              })) {
            const auto unchecked = transport.statistics();
            std::fprintf(stderr, "wrong refresh final chain: failure=%u pages=%llu terminal=%llu\n",
                static_cast<unsigned>(unchecked.raw_evidence_failure),
                static_cast<unsigned long long>(unchecked.raw_evidence_pages_written),
                static_cast<unsigned long long>(unchecked.raw_evidence_last_terminal_written_run));
            return 300;
          }
          transport.stop();
          return transport.statistics().raw_evidence_failure == 6U ? 0 : 301;
        }
        if (!send(release_final) || !wait_for([&] {
              std::lock_guard<std::mutex> lock(channel->mutex);
              Status observed=owned_fixture::status();dp::comm::SourceId observed_source{};
              return ((observed=channel->observation.status),(observed_source=channel->observation.status_source),message_fields_present(observed)) == true && (*observed.run_id()) == next_run &&
                  (*observed.final_state()) == (refresh ? 6U : 8U) &&
                  (*observed.core_state()) == (refresh ? 4U : 1U) &&
                  (*observed.release_committed()) == !refresh &&
                  std::equal(release_terminal.begin(), release_terminal.end(),
                             (observed.evidence_chain_sha256()->data()));
            })) return 294;
        transport.stop();
        const auto capture = transport.statistics();
        return capture.raw_evidence_failure == 0U &&
            channel->raw_evidence_failure.load(std::memory_order_acquire) == 0U &&
            capture.raw_evidence_pages_written == (raw_order_case >= 59U ? 5U : 4U) ? 0 : 295;
      }
      std::array<std::uint8_t, 32U> group_one{}, terminal_one{}, group_two{}, terminal_two{};
      if (!send(valid) || !wait_for([&] {
            std::lock_guard<std::mutex> lock(channel->mutex);
            Status observed=owned_fixture::status();dp::comm::SourceId observed_source{};
            return ((observed=channel->observation.status),(observed_source=channel->observation.status_source),message_fields_present(observed)) == true && (*observed.run_id()) == 1U;
          }) || !publish_page(1U, 0U, 2U, 1U, 0U, seed_for(1U), &group_one) ||
          !wait_for([&] { return transport.statistics().raw_evidence_pages_written == 1U; }))
        return 251;
      /**
       * @brief 前run尚无验证终态时直接来run2页，必须失败且只保留已落盘首run1页。
       */
      if (raw_order_case == 6U) {
        if (!publish_page(2U, 0U, 2U, 1U, 0U, seed_for(2U), &group_two) ||
            !wait_for([&] { return transport.statistics().raw_evidence_failure != 0U; })) return 260;
        transport.stop();
        return transport.statistics().raw_evidence_pages_written == 1U ? 0 : 261;
      }
      if (!publish_page(1U, 1U, 4U, 1U, 6U, group_one, &terminal_one) ||
          !wait_for([&] { return transport.statistics().raw_evidence_last_terminal_written_run == 1U; }))
        return 251;
      auto exported = valid;
      exported.send_state = false;
      exported.status.core_state()=(4U);
      exported.status.final_state()=(6U);
      exported.status.motion_complete()=(true);
      std::uint8_t terminal_digest[32]{};
      std::copy(terminal_one.begin(), terminal_one.end(), terminal_digest);
      exported.status.evidence_chain_sha256().emplace().assign(terminal_digest,32);
      if (!send(exported) || !wait_for([&] {
            std::lock_guard<std::mutex> lock(channel->mutex);
            Status observed=owned_fixture::status();dp::comm::SourceId observed_source{};
            return ((observed=channel->observation.status),(observed_source=channel->observation.status_source),message_fields_present(observed)) == true && (*observed.run_id()) == 1U &&
                (*observed.final_state()) == 6U && (*observed.motion_complete()) &&
                std::equal(terminal_one.begin(), terminal_one.end(), (observed.evidence_chain_sha256()->data()));
          })) return 252;
      if (!publish_page(2U, 0U, 1U, 1U, 0U, seed_for(2U), &group_two) ||
          !wait_for([&] { return transport.statistics().raw_evidence_pages_written == 3U ||
              transport.statistics().raw_evidence_failure != 0U; })) return 253;
      if (raw_order_case == 4U || raw_order_case == 5U) {
        if (!wait_for([&] { return transport.statistics().raw_evidence_failure != 0U; })) return 262;
        transport.stop();
        return transport.statistics().raw_evidence_pages_written == 2U ? 0 : 263;
      }
      if (transport.statistics().raw_evidence_failure != 0U) return 254;
      /**
       * @brief run2执行中可先收到released Idle Status，但余下disable/terminal原页仍绑定原owner session8。
       * 9完整收齐后按记录索引核先页→released Status→disable页→terminal页→final Status；10错boot、11错kind、12坏链、13缺最终Status、14最终链不符分别拒绝。
       */
      if (raw_order_case >= 9U && raw_order_case <= 14U) {
        auto released_state = valid;
        released_state.send_status = false;
        released_state.state.session_id()=(0U);
        released_state.state.reference_epoch()=(0U);
        released_state.state.motion_epoch()=(11U);
        released_state.state.management_epoch()=(1U);
        released_state.state.core_state()=(1U);
        root_fixture::set_position_quality(released_state.state,0U);
        root_fixture::set_raw_quality(released_state.state,63U);
        released_state.state.state_seq()=(2U);
        const double raw_motor_rev[6]{0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
        root_fixture::set_raw_position(released_state.state,raw_motor_rev);
        const auto state_count = transport.statistics().state_messages;
        if (!send(released_state) || !wait_for([&] {
              return transport.statistics().state_messages > state_count;
            })) return 270;
        auto released_status = valid;
        released_status.send_state = false;
        released_status.status.run_id()=(2U);
        released_status.status.session_id()=(0U);
        released_status.status.reference_epoch()=(0U);
        released_status.status.motion_epoch()=(11U);
        released_status.status.management_epoch()=(1U);
        released_status.status.core_state()=(1U);
        released_status.status.release_committed()=(true);
        released_status.status.permit_remaining_us()=(0U);
        released_status.status.final_state()=(2U);
        if (raw_order_case == 10U) released_status.status_source.boot_id=(8U);
        if (!send(released_status) || !wait_for([&] {
              if (transport.statistics().raw_evidence_failure != 0U) return true;
              std::lock_guard<std::mutex> lock(channel->mutex);
              Status observed=owned_fixture::status();dp::comm::SourceId observed_source{};
              return ((observed=channel->observation.status),(observed_source=channel->observation.status_source),message_fields_present(observed)) == true && (*observed.run_id()) == 2U &&
                  (*observed.session_id()) == 0U;
            })) return 271;
        if (raw_order_case == 10U) {
          transport.stop();
          return transport.statistics().raw_evidence_failure != 0U ? 0 : 272;
        }
        if (transport.statistics().raw_evidence_failure != 0U) return 273;
        std::array<std::uint8_t, 32U> disable_chain{};
        if (!publish_page(2U, 1U, 3U, 2U, 0U, group_two, &disable_chain) ||
            !wait_for([&] { return transport.statistics().raw_evidence_pages_written == (raw_order_case >= 59U ? 5U : 4U) ||
                transport.statistics().raw_evidence_failure != 0U; })) return 274;
        if (raw_order_case == 11U || raw_order_case == 12U) {
          transport.stop();
          return transport.statistics().raw_evidence_failure != 0U &&
              transport.statistics().raw_evidence_pages_written == 3U ? 0 : 275;
        }
        if (transport.statistics().raw_evidence_failure != 0U ||
            !publish_page(2U, 2U, 4U, 2U, 2U, disable_chain, &terminal_two) ||
            !wait_for([&] { return transport.statistics().raw_evidence_last_terminal_written_run == 2U; }))
          return 276;
        if (raw_order_case == 13U) {
          transport.stop();
          return transport.statistics().raw_evidence_failure != 0U ? 0 : 277;
        }
        auto final_status = released_status;
        final_status.status.management_epoch()=(2U);
        std::uint8_t final_digest[32]{};
        std::copy(terminal_two.begin(), terminal_two.end(), final_digest);
        if (raw_order_case == 14U) final_digest[0] ^= 1U;
        final_status.status.evidence_chain_sha256().emplace().assign(final_digest,32);
        if (raw_order_case == 14U) {
          if (!send(final_status) || !wait_for([&] {
                return transport.statistics().raw_evidence_failure != 0U;
              })) return 283;
          transport.stop();
          return transport.statistics().raw_evidence_pages_written == 5U ? 0 : 284;
        }
        if (!send(final_status) || !wait_for([&] {
              if (transport.statistics().raw_evidence_failure != 0U) return true;
              std::lock_guard<std::mutex> lock(channel->mutex);
              Status observed=owned_fixture::status();dp::comm::SourceId observed_source{};
              return ((observed=channel->observation.status),(observed_source=channel->observation.status_source),message_fields_present(observed)) == true && (*observed.run_id()) == 2U &&
                  (*observed.session_id()) == 0U && (*observed.final_state()) == 2U &&
                  std::equal(terminal_two.begin(), terminal_two.end(),
                             (observed.evidence_chain_sha256()->data()));
            })) return 278;
        transport.stop();
        const auto capture = transport.statistics();
        if (capture.raw_evidence_failure != 0U || capture.raw_evidence_pages_written != 5U)
          return 279;
        std::FILE *file = std::fopen(capture_path.c_str(), "rb");
        if (!file) return 280;
        if (::fseek(file, 8L, SEEK_SET) != 0) { std::fclose(file); return 281; }
        std::uint8_t header[kCaptureHeaderSize]{};
        int index = 0, page_indices[3]{-1, -1, -1}, released_index = -1, final_index = -1;
        bool old_identity = true;
        while (std::fread(header, 1U, sizeof(header), file) == sizeof(header)) {
          const auto size = static_cast<std::uint16_t>(header[1] | (header[2] << 8U));
          std::array<std::uint8_t, dp::protocol::canonical::Traits<Status>::kMaxEncodedSize> payload{};
          if (size > payload.size() || std::fread(payload.data(), 1U, size, file) != size) break;
          if (header[0] == 1U && size <= dp::protocol::canonical::Traits<Evidence>::kMaxEncodedSize) {
            Evidence observed=owned_fixture::evidence();dp::comm::SourceId observed_source{};
            if (((observed_source=root_fixture::capture_source(header)),dp::protocol::canonical::Traits<Evidence>::decode(payload.data(),size,&observed)) ==
                    true && (*observed.run_id()) == 2U && (*observed.evidence_seq()) < 3U) {
              page_indices[(*observed.evidence_seq())] = index;
              old_identity = old_identity && header[11] == 7U && header[19] == 8U;
            }
          } else if (header[0] == 3U && size <= dp::protocol::canonical::Traits<Status>::kMaxEncodedSize) {
            Status observed=owned_fixture::status();dp::comm::SourceId observed_source{};
            if (((observed_source=root_fixture::capture_source(header)),dp::protocol::canonical::Traits<Status>::decode(payload.data(),size,&observed)) ==
                    true && (*observed.run_id()) == 2U) {
              const bool chain_present = std::any_of(
                  observed.evidence_chain_sha256()->begin(),
                  observed.evidence_chain_sha256()->end(),
                  [](std::uint8_t byte) { return byte != 0U; });
              if (chain_present) final_index = index;
              else released_index = index;
            }
          }
          ++index;
        }
        std::fclose(file);
        return old_identity && page_indices[0] >= 0 &&
            page_indices[0] < released_index && released_index < page_indices[1] &&
            page_indices[1] < page_indices[2] && page_indices[2] < final_index ? 0 : 282;
      }
      if (raw_order_case == 8U) {
        if (!publish_page(2U, 0U, 1U, 1U, 0U, seed_for(2U), &group_two) ||
            !wait_for([&] { return transport.statistics().raw_evidence_failure != 0U; })) return 266;
        transport.stop();
        return transport.statistics().raw_evidence_pages_written == 3U ? 0 : 267;
      }
      if (raw_order_case == 7U) {
        if (!publish_page(2U, 1U, 4U, 1U, 6U, group_two, &terminal_two) ||
            !wait_for([&] { return transport.statistics().raw_evidence_last_terminal_written_run == 2U; }))
          return 268;
        transport.stop();
        return transport.statistics().raw_evidence_failure != 0U ? 0 : 269;
      }
      auto status_two = valid;
      status_two.send_state = false;
      status_two.status.run_id()=(2U);
      status_two.status.management_epoch()=(1U);
      if (raw_order_case == 2U) status_two.status_source.boot_id=(8U);
      if (raw_order_case == 3U) status_two.status.session_id()=(9U);
      if (raw_order_case == 2U || raw_order_case == 3U) {
        if (!send(status_two) || !wait_for([&] {
              return transport.statistics().raw_evidence_failure != 0U;
            })) return 264;
        transport.stop();
        return transport.statistics().raw_evidence_pages_written == 3U ? 0 : 265;
      }
      if (!send(status_two) || !wait_for([&] {
            std::lock_guard<std::mutex> lock(channel->mutex);
            Status observed=owned_fixture::status();dp::comm::SourceId observed_source{};
            return ((observed=channel->observation.status),(observed_source=channel->observation.status_source),message_fields_present(observed)) == true && (*observed.run_id()) == 2U;
          }) || !publish_page(2U, 1U, 4U, 1U, 6U, group_two, &terminal_two) ||
          !wait_for([&] { return transport.statistics().raw_evidence_last_terminal_written_run == 2U; }))
        return 255;
      transport.stop();
      const auto capture = transport.statistics();
      if (capture.raw_evidence_failure != 0U ||
          channel->raw_evidence_failure.load(std::memory_order_acquire) != 0U ||
          capture.raw_evidence_pages_written != 4U) return 256;
      std::FILE *file = std::fopen(capture_path.c_str(), "rb");
      if (!file) return 257;
      if (::fseek(file, 8L, SEEK_SET) != 0) { std::fclose(file); return 258; }
      std::uint8_t header[kCaptureHeaderSize]{};
      int index = 0, page_two_index = -1, status_two_index = -1;
      while (std::fread(header, 1U, sizeof(header), file) == sizeof(header)) {
        const auto size = static_cast<std::uint16_t>(header[1] | (header[2] << 8U));
        std::array<std::uint8_t, dp::protocol::canonical::Traits<Status>::kMaxEncodedSize> payload{};
        if (size > payload.size() || std::fread(payload.data(), 1U, size, file) != size) break;
        if (header[0] == 1U && size <= dp::protocol::canonical::Traits<Evidence>::kMaxEncodedSize) {
          Evidence observed=owned_fixture::evidence();dp::comm::SourceId observed_source{};
          if (((observed_source=root_fixture::capture_source(header)),dp::protocol::canonical::Traits<Evidence>::decode(payload.data(),size,&observed)) ==
                  true && (*observed.run_id()) == 2U && (*observed.evidence_seq()) == 0U)
            page_two_index = index;
        } else if (header[0] == 3U && size <= dp::protocol::canonical::Traits<Status>::kMaxEncodedSize) {
          Status observed=owned_fixture::status();dp::comm::SourceId observed_source{};
          if (((observed_source=root_fixture::capture_source(header)),dp::protocol::canonical::Traits<Status>::decode(payload.data(),size,&observed)) ==
                  true && (*observed.run_id()) == 2U)
            status_two_index = index;
        }
        ++index;
      }
      std::fclose(file);
      return page_two_index >= 0 && status_two_index > page_two_index ? 0 : 259;
    }
    /**
     * @brief 顺序run3..11共9run/18页，完整flush后回收有限run绑定槽。
     * run4及run11插入旧run3 Status须增加stale_run_status_rejections但不覆盖当前run，也不把可忽略旧Status升级为collector失败。
     */
    if (raw_many_mode) {
      /**
       * @par 内部publish_page（多run压力）
       * run和terminal选择本run的seq0组页/seq1终页，previous/next传递32字节链；每组固定六轴全SENT。
       * 只供run3..11闭合与回收检查，不生成真实CAN发送时间或运动完成实测。
       */
      auto publish_page = [&](std::uint64_t run, bool terminal,
                              const std::array<std::uint8_t, 32U> &previous,
                              std::array<std::uint8_t, 32U> *next) {
        auto stimulus = valid;
        stimulus.send_status = false;
        stimulus.send_state = false;
        stimulus.send_evidence = true;
        auto &message = stimulus.evidence;
        (void)0;
        (void)0;
        message.run_id()=(run);
        message.attempt_id()=(run);
        message.evidence_seq()=(terminal ? 1U : 0U);
        message.attempt_kind()=(1U);
        message.motion_epoch()=(10U);
        message.category()=(terminal ? 4U : 2U);
        message.final()=(1U);
        message.group_count()=(1U);
        if (terminal) message.final_state()=(3U);
        else {
          message.group_id()=(run);
          message.expected_mask()=(63U);
          message.submitted_mask()=(63U);
          message.sent_mask()=(63U);
        }
        std::uint8_t prior[32]{};
        std::copy(previous.begin(), previous.end(), prior);
        message.previous_chain_sha256().emplace().assign(prior,32);
        if(!dummy_ref::comm::attempt_evidence::next_chain(previous,message,next))return false;
        std::uint8_t digest[32]{};
        std::copy(next->begin(), next->end(), digest);
        message.chain_sha256().emplace().assign(digest,32);
        return send(stimulus);
      };
      for (std::uint64_t run = 3U; run <= 11U; ++run) {
        if (run != 3U) {
          auto status_only = valid;
          status_only.send_state = false;
          status_only.status.run_id()=(run);
          status_only.status.management_epoch()=(static_cast<std::uint32_t>(run - 3U));
          if (!send(status_only) || !wait_for([&] {
                std::lock_guard<std::mutex> lock(channel->mutex);
                Status observed=owned_fixture::status();dp::comm::SourceId observed_source{};
                return ((observed=channel->observation.status),(observed_source=channel->observation.status_source),message_fields_present(observed)) == true && (*observed.run_id()) == run;
              })) return 145;
        }
        std::array<std::uint8_t, 192U> seed{};
        for (unsigned i = 0U; i < 8U; ++i) seed[i] = static_cast<std::uint8_t>(run >> (8U * i));
        seed[8] = 7U;
        std::copy(parameters.client_instance_id.begin(), parameters.client_instance_id.end(), seed.begin() + 16U);
        std::copy(parameters.contract_hash.begin(), parameters.contract_hash.end(), seed.begin() + 32U);
        std::copy(parameters.model_hash.begin(), parameters.model_hash.end(), seed.begin() + 64U);
        std::copy(parameters.profile_hash.begin(), parameters.profile_hash.end(), seed.begin() + 96U);
        std::copy(parameters.firmware_hash.begin(), parameters.firmware_hash.end(), seed.begin() + 128U);
        std::array<std::uint8_t, 32U> previous{}, group_chain{}, terminal_chain{};
        ::SHA256(seed.data(), seed.size(), previous.data());
        if (!publish_page(run, false, previous, &group_chain) ||
            !wait_for([&] { return transport.statistics().raw_evidence_pages_written >=
                (run - 3U) * 2U + 1U; }) ||
            !publish_page(run, true, group_chain, &terminal_chain) ||
            !wait_for([&] { return transport.statistics().raw_evidence_last_terminal_written_run == run; }))
          return 146;
        if (run == 4U || run == 11U) {
          auto stale_status = valid;
          stale_status.send_state = false;
          stale_status.status.run_id()=(3U);
          stale_status.status.management_epoch()=(static_cast<std::uint32_t>(run - 3U));
          const auto rejected_before = transport.statistics().stale_run_status_rejections;
          if (!send(stale_status) || !wait_for([&] {
                return transport.statistics().stale_run_status_rejections > rejected_before;
              })) return 150;
          std::lock_guard<std::mutex> lock(channel->mutex);
          Status observed=owned_fixture::status();dp::comm::SourceId observed_source{};
          if (((observed=channel->observation.status),(observed_source=channel->observation.status_source),message_fields_present(observed)) != true || (*observed.run_id()) != run ||
              transport.statistics().raw_evidence_failure != 0U) return 151;
        }
      }
      transport.stop();
      const auto capture = transport.statistics();
      return capture.raw_evidence_failure == 0U &&
          channel->raw_evidence_failure.load(std::memory_order_acquire) == 0U &&
          capture.raw_evidence_pages_written == 18U ? 0 : 147;
    }
    /**
     * @brief 已从Status expect到run却没有终页即stop；collector必须锁存Incomplete并让ready=false/initialization_failed=true。
     * 这是证据采集失败状态，不能把缺页当作无动作或成功释放。
     */
    if (raw_incomplete_mode) {
      transport.stop();
      const auto capture = transport.statistics();
      return capture.raw_evidence_failure ==
                 static_cast<std::uint8_t>(RawEvidenceCollector::Failure::kIncomplete) &&
             channel->raw_evidence_failure.load(std::memory_order_acquire) ==
                 static_cast<std::uint8_t>(RawEvidenceCollector::Failure::kIncomplete) &&
             capture.initialization_failed && !capture.ready ? 0 : 144;
    }
    auto released_status = valid;
    released_status.send_state = false;
    released_status.status.session_id()=(0U);
    released_status.status.reference_epoch()=(0U);
    released_status.status.management_epoch()=(1U);
    if (raw_bad_release_mode) released_status.status.parent_run_id()=(42U);
    /**
     * @brief 将释放Status指向不相关parent42，要求采集失败及共享失败码锁存。
     * 只能证明绑定门槛触发，不说明对端真实执行了释放。
     */
    if (raw_bad_release_mode) {
      if (!send(released_status) || !wait_for([&] {
            return transport.statistics().raw_evidence_failure != 0U;
          })) return 142;
      transport.stop();
      const auto capture = transport.statistics();
      return channel->raw_evidence_failure.load(std::memory_order_acquire) != 0U &&
             capture.initialization_failed && !capture.ready ? 0 : 143;
    }
    if (!send(released_status) || !wait_for([&] {
          std::lock_guard<std::mutex> lock(channel->mutex);
          Status observed=owned_fixture::status();dp::comm::SourceId observed_source{};
          return ((observed=channel->observation.status),(observed_source=channel->observation.status_source),message_fields_present(observed)) == true && (*observed.session_id()) == 0U &&
              (*observed.run_id()) == 3U;
        })) return 141;
    auto evidence = valid;
    evidence.send_status = false;
    evidence.send_state = false;
    evidence.send_evidence = true;
    (void)0;
    (void)0;
    evidence.evidence.run_id()=(3U);
    evidence.evidence.attempt_id()=(3U);
    evidence.evidence.evidence_seq()=(0U);
    evidence.evidence.attempt_kind()=(1U);
    evidence.evidence.category()=(4U);
    evidence.evidence.final()=(1U);
    evidence.evidence.final_state()=(3U);
    evidence.evidence.terminal_reason()=(14U);
    std::array<std::uint8_t, 192U> seed{};
    seed[0] = 3U;
    seed[8] = 7U;
    std::copy(parameters.client_instance_id.begin(), parameters.client_instance_id.end(), seed.begin() + 16U);
    std::copy(parameters.contract_hash.begin(), parameters.contract_hash.end(), seed.begin() + 32U);
    std::copy(parameters.model_hash.begin(), parameters.model_hash.end(), seed.begin() + 64U);
    std::copy(parameters.profile_hash.begin(), parameters.profile_hash.end(), seed.begin() + 96U);
    std::copy(parameters.firmware_hash.begin(), parameters.firmware_hash.end(), seed.begin() + 128U);
    std::uint8_t previous[32]{};
    ::SHA256(seed.data(), seed.size(), previous);
    evidence.evidence.previous_chain_sha256().emplace().assign(previous,32);
    std::array<std::uint8_t,32> prior_array{},chain_array{};
    std::copy_n(previous,32,prior_array.begin());
    if(!dummy_ref::comm::attempt_evidence::next_chain(prior_array,evidence.evidence,&chain_array))return 140;
    std::uint8_t chain[32]{};std::copy(chain_array.begin(),chain_array.end(),chain);
    evidence.evidence.chain_sha256().emplace().assign(chain,32);
    if (!send(evidence) || !wait_for([&] {
          return transport.statistics().raw_evidence_pages_written == 1U;
        })) return 140;
    transport.stop();
    const auto capture = transport.statistics();
    std::FILE *file = std::fopen(capture_path.c_str(), "rb");
    if (!file) return 138;
    std::uint8_t header[kCaptureHeaderSize]{};
    bool found_page = false;
    if (::fseek(file, 8L, SEEK_SET) != 0) { std::fclose(file); return 139; }
    while (std::fread(header, 1U, sizeof(header), file) == sizeof(header)) {
      const auto payload_size = static_cast<std::uint16_t>(header[1] | (header[2] << 8U));
      if (header[0] == 1U && payload_size <= dp::protocol::canonical::Traits<Evidence>::kMaxEncodedSize &&
          header[11] == 7U && header[19] == 8U && header[27] == 3U) found_page = true;
      if (::fseek(file, payload_size, SEEK_CUR) != 0) break;
    }
    std::fclose(file);
    return capture.raw_evidence_failure == 0U &&
        channel->raw_evidence_failure.load(std::memory_order_acquire) == 0U && found_page ? 0 : 139;
  }
  /**
   * @brief 普通完整场景先要求连续100ms得到稳定可用快照；若绑定失效变化则升seq并重新计稳定窗。
   * 该准备过程允许异步bootstrap完成，不把一次瞬间copy成功当作长期稳定。
   */
  auto stable_sequence = (*valid.state.state_seq());
  auto observed_invalidations = transport.statistics().invalidations;
  auto stable_since = std::chrono::steady_clock::now();
  if (!wait_for([&] {
        const auto statistics = transport.statistics();
        if (statistics.invalidations != observed_invalidations) {
          observed_invalidations = statistics.invalidations;
          valid.state.state_seq()=(++stable_sequence);
          stable_since = std::chrono::steady_clock::now();
        }
        send(valid);
        if (!system.copy_state(&snapshot) || snapshot.envelope.state_seq != stable_sequence) {
          stable_since = std::chrono::steady_clock::now();
          return false;
        }
        return std::chrono::steady_clock::now() - stable_since >= std::chrono::milliseconds(100);
      })) {
    const auto stats=transport.statistics();const bool copied=system.copy_state(&snapshot);
    std::fprintf(stderr,"ROOT_STABLE_FAILED status=%llu state=%llu callbacks=%llu rejected=%llu invalid=%llu seq=%u wanted=%u copy=%u identity=%llu authority=%llu sequence=%llu ingest=%llu stop=%llu bootstrap=%llu stale=%llu skew=%llu\n",
      (unsigned long long)stats.status_messages,(unsigned long long)stats.state_messages,(unsigned long long)stats.state_callbacks,
      (unsigned long long)stats.rejected_states,(unsigned long long)stats.invalidations,snapshot.envelope.state_seq,stable_sequence,unsigned(copied),
      (unsigned long long)stats.state_identity_rejections,(unsigned long long)stats.state_authority_rejections,
      (unsigned long long)stats.state_sequence_rejections,(unsigned long long)stats.state_ingest_rejections,
      (unsigned long long)stats.stop_latched_state_rejections,(unsigned long long)stats.bootstrap_rejections,
      (unsigned long long)stats.stale_payload_rejections,(unsigned long long)stats.clock_skew_rejections);
    return 54;
  }
  if (snapshot.position[0] != 0.25 ||
      (six_axis ? snapshot.position[1] != 0.2 : !std::isnan(snapshot.position[1]))) return 13;
  /**
   * @brief 先等102ms不得Stop，之后超过250ms必须因State间隔停机。
   * 原State重放、仅补Status、即使新seq同授权State也不能绕过已锁存Stop恢复copy_state。
   */
  if (heartbeat_budget_mode) {
    const auto stops_before = transport.statistics().state_heartbeat_gap_stops;
    std::this_thread::sleep_for(std::chrono::milliseconds(102));
    if (transport.statistics().state_heartbeat_gap_stops != stops_before) return 148;
    if (!wait_for([&] {
          return transport.statistics().state_heartbeat_gap_stops > stops_before;
        }) || transport.statistics().last_state_gap_us <= 250000U) return 149;
    auto old_state = valid;
    old_state.send_status = false;
    if (!send_rejected_state(old_state) || system.copy_state(&snapshot)) return 150;
    auto rebind_status = valid;
    rebind_status.send_state = false;
    const auto status_before = transport.statistics().status_messages;
    if (!send(rebind_status) || !wait_for([&] {
          return transport.statistics().status_messages > status_before;
        })) return 151;
    auto newer_state = valid;
    newer_state.send_status = false;
    newer_state.state.state_seq()=(stable_sequence + 1U);
    const auto latched_before = transport.statistics().stop_latched_state_rejections;
    if (!send_rejected_state(newer_state) || system.copy_state(&snapshot) ||
        transport.statistics().stop_latched_state_rejections <= latched_before) return 152;
    transport.stop();
    return 0;
  }
  std::uint64_t accepted_observation_us = 0U;
  {
    std::lock_guard<std::mutex> lock(channel->mutex);
    accepted_observation_us = channel->observation.state_received_us;
  }
  if (accepted_observation_us == 0U || !send_rejected_state(valid)) return 56;
  {
    std::lock_guard<std::mutex> lock(channel->mutex);
    if (channel->observation.state_received_us != accepted_observation_us) return 57;
  }
  /**
   * @par 内部restore_valid_snapshot回调
   * 将合法seq推进后重发，直到copy_state精确看到该seq；用于普通身份负例之间恢复夹具，不清除生产Stop锁存。
   */
  auto restore_valid_snapshot = [&] {
    valid.state.state_seq()=(++stable_sequence);
    return wait_for([&] {
      send(valid);
      return system.copy_state(&snapshot) && snapshot.envelope.state_seq == stable_sequence;
    });
  };
  if (default_ack_age_mode) {
    valid.state.state_seq()=(++stable_sequence);
    valid.state.produced_at_us()=(device_now_us());
    if (!wait_for([&] { return send(valid) && system.copy_state(&snapshot) &&
        snapshot.envelope.state_seq == stable_sequence; })) return 139;
  }
  /**
   * @brief 允许未activate时由worker从全局最新目标取得一次prime；本分支没有显式transfer。
   * 核实际peer收到的wire producer/motion，再stop清身份；不证明正式生命周期之外任意目标可获准。
   */
  if (inactive_prime_mode) {
    for (unsigned fault = 0U; fault < 3U; ++fault) {
      auto invalid_floor = valid;
      invalid_floor.send_status = false;
      invalid_floor.state.state_seq()=(++stable_sequence);
      invalid_floor.state.committed_producer_seq()=(5000U);
      invalid_floor.state.admitted_producer_seq()=(5000U);
      if (fault == 0U) invalid_floor.state_source.boot_id=(999U);
      if (fault == 1U) invalid_floor.state.motion_epoch()=(999U);
      if (fault == 2U) invalid_floor.state.produced_at_us()=(1U);
      if (!send_rejected_state(invalid_floor)) return 160;
    }
    valid.state.state_seq()=(++stable_sequence);
    valid.state.produced_at_us()=(0U);
    if (!wait_for([&] { return send(valid) && system.copy_state(&snapshot) &&
        snapshot.envelope.state_seq == stable_sequence; })) return 159;
    channel->inactive_prime_allowed.store(true);
    set_command_identity({valid.status_source.boot_id, (*valid.status.session_id()),
        (*valid.status.reference_epoch()), (*valid.status.motion_epoch()), 9007199254740997ULL});
    const auto prime_generation = publish_control_update(positions, 6U, 0, nullptr);
    const bool sent = prime_generation != 0U && ::poll(&response, 1, 1500) == 1;
    Command prime_command;
    const bool valid_prime = sent && (root_fixture::read_value(input,prime_command)) &&
        (*prime_command.producer_seq()) == wire_producer_sequence(prime_generation) &&
        (*prime_command.producer_seq()) == 45U &&
        (*prime_command.motion_epoch()) == (*valid.status.motion_epoch());
    if (!valid_prime) { const auto stats=transport.statistics(); std::fprintf(stderr,"reject=%llu published=%llu invalid=%llu ready=%d\n",(unsigned long long)stats.rejected_commands,(unsigned long long)stats.commands_published,(unsigned long long)stats.invalidations,stats.ready); std::fprintf(stderr,"highwater prime sent=%d generation=%llu wire=%u motion=%u expected_motion=%u\n",sent,static_cast<unsigned long long>(prime_generation),(*prime_command.producer_seq()),(*prime_command.motion_epoch()),(*valid.status.motion_epoch())); return 154; }
    const auto before_old_ack = transport.statistics().group_sent_ack_misses;
    valid.send_status = false;
    valid.state.state_seq()=(++stable_sequence);
    valid.state.produced_at_us()=(0U);
    if (!wait_for([&] { return send(valid) && system.copy_state(&snapshot) &&
        snapshot.envelope.state_seq == stable_sequence; })) return 155;
    const auto old_ack_misses = transport.statistics().group_sent_ack_misses;
    if (old_ack_misses <= before_old_ack) return 156; // retained44 is not ACK45
    valid.state.committed_producer_seq()=((*prime_command.producer_seq()));
    valid.state.admitted_producer_seq()=((*prime_command.producer_seq()));
    valid.state.can_sent_mask()=(63U);
    valid.state.state_seq()=(++stable_sequence);
    if (!wait_for([&] { return send(valid) && system.copy_state(&snapshot) &&
        snapshot.envelope.state_seq == stable_sequence; })) return 157;
    if (transport.statistics().group_sent_ack_misses != old_ack_misses ||
        transport.statistics().group_sent_ack_timeouts != 0U) return 158;
    transport.stop();
    clear_command_identity();
    return valid_prime ? 0 : 154;
  }
  if (system.on_activate(rclcpp_lifecycle::State()) != hardware_interface::CallbackReturn::SUCCESS) return 14;
  if (ticket_mode != 0U) {
    valid.state.state_seq()=(++stable_sequence);
    valid.state.produced_at_us()=(device_now_us());
    valid.state.freshness_ticket()=(9007199254740997ULL);
    if (!send(valid) || !wait_for([&] { return system.copy_state(&snapshot) &&
        snapshot.envelope.state_seq == stable_sequence; })) return 250;
  } else {
    valid.state.state_seq()=(++stable_sequence);
    valid.state.produced_at_us()=(0U);
    if (!feedback_until(valid, [&] { return observed_state_matches(valid.state,valid.state_source) &&
        system.copy_state(&snapshot) && snapshot.envelope.state_seq == stable_sequence; })) {
      const auto stats = transport.statistics();
      const auto observation = channel->diagnostic_observation();
      State observed=owned_fixture::state();dp::comm::SourceId observed_source{};
      (void)((observed=observation.state),(observed_source=observation.state_source),message_fields_present(observed));
      std::fprintf(stderr, "ROOT_ACTIVATE_STATE_FAILED expected_seq=%u observed_seq=%u callbacks=%llu messages=%llu rejected=%llu identity=%llu authority=%llu sequence=%llu stop=%llu\n",
          stable_sequence, (*observed.state_seq()), (unsigned long long)stats.state_callbacks,
          (unsigned long long)stats.state_messages, (unsigned long long)stats.rejected_states,
          (unsigned long long)stats.state_identity_rejections, (unsigned long long)stats.state_authority_rejections,
          (unsigned long long)stats.state_sequence_rejections, (unsigned long long)stats.stop_latched_state_rejections);
      return 256;
    }
    valid.state.produced_at_us()=(0U);
  }
  set_command_identity({7U, 8U, 9U, 10U, 9007199254740997ULL});
  const auto generation = publish_control_update(positions, six_axis ? 6U : 1U, 0, nullptr);
  if (!generation || !system.transfer_control_update()) return 15;
  if (::poll(&response, 1, 3000) != 1) return 16;
  Command command;
  const auto command_bytes = root_fixture::read_value(input,command);
  if (!command_bytes) {
    std::fprintf(stderr, "first command canonical decode=%u owned_size=%zu errno=%d revents=%d\n",
                 unsigned(command_bytes), sizeof(command), errno, response.revents);
    return 17;
  }
  if ((command.domain_device_id()->data())[0] != 1U || (command.client_instance_id()->data())[0] != 2U ||
      (*command.boot_epoch()) != 7U || (*command.session_id()) != 8U ||
      (*command.reference_epoch()) != 9U || (*command.motion_epoch()) != 10U || root_fixture::configured(command) != parameters.configured_mask ||
      (*command.profile_id()) != 1U || (*command.producer_seq()) != generation ||
      (*command.freshness_ticket()) == 0U || root_fixture::position(command)[0] != 0.25 ||
      (six_axis ? root_fixture::position(command)[1] != 0.2 : root_fixture::position(command)[1] != 0.0) ||
      !std::all_of(command.joints()->begin(),command.joints()->end(),[](const auto& joint){return joint.position_rad()&&std::isfinite(*joint.position_rad());})) {
    std::fprintf(stderr,
        "first command fields: device=%u client=%u boot=%llu session=%llu reference=%u motion=%u mask=%u profile=%u seq=%u expected_seq=%llu ticket=%llu position0=%.17g position1=%.17g\n",
        (command.domain_device_id()->data())[0], (command.client_instance_id()->data())[0],
        static_cast<unsigned long long>((*command.boot_epoch())),
        static_cast<unsigned long long>((*command.session_id())),
        (*command.reference_epoch()), (*command.motion_epoch()), root_fixture::configured(command),
        (*command.profile_id()), (*command.producer_seq()),
        static_cast<unsigned long long>(generation),
        static_cast<unsigned long long>((*command.freshness_ticket())),
        root_fixture::position(command)[0], root_fixture::position(command)[1]);
    return 17;
  }
  /**
   * @brief 待ACK命令期间Status拒绝计数3→4，核rejected_ack的完整身份/序号。
   * 明确拒绝应立即触发Stop尝试且ACK timeout仍0，不能把设备拒绝拖到超时后才发现。
   */
  if (reject_during_ack_mode) {
    valid.status.command_reject_count()=(4U);
    valid.status.can_sent_mask()=(parameters.configured_mask);
    if (!send(valid) || !wait_for([&] {
          std::lock_guard<std::mutex> lock(channel->mutex);
          return channel->observation.rejected_ack_sequence == (*command.producer_seq());
        })) return 260;
    {
      std::lock_guard<std::mutex> lock(channel->mutex);
      const auto &observed=channel->observation;
      if (observed.rejected_ack_boot_epoch!=7U || observed.rejected_ack_session_id!=8U ||
          observed.rejected_ack_reference_epoch!=9U || observed.rejected_ack_motion_epoch!=10U ||
          observed.rejected_ack_sequence!=(*command.producer_seq())) return 261;
    }
    if (!wait_for([&] { const auto stats=transport.statistics();
          return stats.stop_requests_sent+stats.stop_request_failures!=0U; }) ||
        transport.statistics().group_sent_ack_timeouts!=0U) return 262;
    transport.stop();
    clear_command_identity();
    return 0;
  }
  /**
   * @brief 一个目标在途时排入第二目标，持续保鲜但暂不增加票据。
   * 75ms后ACK首目标；mode1新票据应发送第二目标并携带新ticket，mode2旧票据不发，mode3换session必须失效而不能借续票据发旧owner目标。
   */
  if (ticket_mode != 0U) {
    const auto second_position = std::array<double, 6>{0.26, 0.2, 0.3, 0.4, 0.5, 0.6};
    const auto second_generation = publish_control_update(second_position.data(), 6U, 0, nullptr);
    if (second_generation == 0U || !system.transfer_control_update()) return 251;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(75);
    while (std::chrono::steady_clock::now() < deadline) {
      valid.state.state_seq()=(++stable_sequence);
      valid.state.produced_at_us()=(device_now_us());
      if (!send(valid)) return 252;
      std::this_thread::sleep_for(std::chrono::milliseconds(8));
    }
    valid.state.state_seq()=(++stable_sequence);
    valid.state.produced_at_us()=(device_now_us());
    valid.state.committed_producer_seq()=(static_cast<std::uint32_t>(generation));
    if (ticket_mode == 1U) valid.state.freshness_ticket()=(9007199254740998ULL);
    const auto before_authority = transport.statistics().invalidations;
    if (ticket_mode == 3U) {
      valid.status.session_id()=(99U);
      valid.state.session_id()=(99U);
      valid.state.freshness_ticket()=(9007199254740998ULL);
    }
    if (!send(valid)) return 253;
    if (ticket_mode == 3U && !wait_for([&] {
          return transport.statistics().invalidations > before_authority;
        })) return 259;
    if (ticket_mode == 1U) {
      if (::poll(&response, 1, 1500) != 1 || (root_fixture::read_value(input,command)==false) ||
          (*command.producer_seq()) != second_generation || root_fixture::position(command)[0] != 0.26 ||
          (*command.freshness_ticket()) != 9007199254740998ULL) return 254;
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(80));
      if (::poll(&response, 1, 0) == 1 ||
          transport.statistics().commands_published != 1U) return 255;
    }
    transport.stop();
    clear_command_identity();
    return 0;
  }
  /**
   * @brief 两种失效原因单独安排：ACK模式继续新seq未ACK State，年龄模式完全停发State。
   * 前者只计组ACK超时，后者计State stale且未确认ACK归因总数为1；cleanup仍未收到完整Stop证明时必须unconfirmed。
   */
  if (ack_timeout_mode || default_ack_age_mode) {
    auto unacked = valid;
    unacked.send_status = false;
    const auto until = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(default_ack_age_mode ? 400 : 250);
    while (std::chrono::steady_clock::now() < until &&
           (default_ack_age_mode
                ? transport.statistics().state_stale_stops == 0U
                : transport.statistics().group_sent_ack_timeouts == 0U)) {
      if (default_ack_age_mode) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        continue;
      }
      unacked.state.state_seq()=(++stable_sequence);
      unacked.state.produced_at_us()=(device_now_us());
      if (!send(unacked)) return 134;
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    const auto statistics = transport.statistics();
    if (default_ack_age_mode) {
      if (statistics.state_stale_stops != 1U ||
          statistics.ack_unconfirmed_on_state_stale_stop +
              statistics.group_sent_ack_timeouts != 1U ||
          statistics.state_heartbeat_gap_stops != 0U) {
        std::fprintf(stderr,
            "default ack/age: stale=%llu unconfirmed=%llu ack_timeout=%llu heartbeat=%llu stop_sent=%llu\n",
            static_cast<unsigned long long>(statistics.state_stale_stops),
            static_cast<unsigned long long>(statistics.ack_unconfirmed_on_state_stale_stop),
            static_cast<unsigned long long>(statistics.group_sent_ack_timeouts),
            static_cast<unsigned long long>(statistics.state_heartbeat_gap_stops),
            static_cast<unsigned long long>(statistics.stop_requests_sent));
        return 140;
      }
      if (!wait_for([&] { return transport.statistics().stop_requests_sent == 1U; }))
        return 141;
      transport.stop();
      if (transport.statistics().stop_requests_confirmed != 0U ||
          transport.statistics().unconfirmed_stop_requests == 0U) return 143;
      return 0;
    }
    if (statistics.group_sent_ack_timeouts != 1U ||
        statistics.state_heartbeat_gap_stops != 0U ||
        statistics.group_sent_ack_misses == 0U ||
        statistics.state_callbacks < statistics.state_messages) return 135;
    transport.stop();
    return 0;
  }
  valid.status.committed_producer_seq()=(static_cast<std::uint32_t>(generation));
  valid.status.can_sent_mask()=(parameters.configured_mask);
  valid.state.state_seq()=(++stable_sequence);
  if (!wait_for([&] {
        send(valid);
        return system.copy_state(&snapshot) && snapshot.envelope.state_seq == stable_sequence;
      }) || transport.statistics().group_sent_ack_timeouts != 0U) return 67;
  // Replayed sequence, wrong device/profile and missing position evidence cannot replace feedback.
  auto invalid = valid;
  const double wrong_position[6]{0.75, NAN, NAN, NAN, NAN, NAN};
  root_fixture::set_position(invalid.state,wrong_position);
  if (!send_rejected_state(invalid) ||
      !system.copy_state(&snapshot) || snapshot.position[0] != 0.25) return 18;
  /**
   * @brief 逐项改设备/profile/mask、Source未来/过去、session/boot/reference/motion，要求拒绝且保留0.25rad旧快照。
   * 每项最多三次恢复重试是夹具调度容错，不表示生产会自动重试非法命令。
   */
  for (unsigned fault = 0U; fault < 9U; ++fault) {
    bool rejected_while_snapshot_retained = false;
    for (unsigned attempt = 0U; attempt < 3U && !rejected_while_snapshot_retained; ++attempt) {
      auto wrong = valid;
      const auto sequence = stable_sequence + 1U;
      wrong.state.state_seq()=(sequence);
      wrong.state.produced_at_us()=(device_now_us());
      if (fault == 0U) { std::uint8_t other[16]{3U}; wrong.state.domain_device_id().emplace().assign(other,16); }
      if (fault == 1U) wrong.state.profile_id()=(2U);
      if (fault == 2U) root_fixture::set_configured(wrong.state,six_axis ? 1U : 63U);
      if (fault == 3U) wrong.state.produced_at_us()=(device_now_us() + 2000000U);
      if (fault == 4U) wrong.state.produced_at_us()=(device_now_us() - 2000000U);
      if (fault == 5U) wrong.state.session_id()=(99U);
      if (fault == 6U) wrong.invalid_root_source_boot=99U;
      if (fault == 7U) wrong.state.reference_epoch()=(99U);
      if (fault == 8U) wrong.state.motion_epoch()=(99U);
      rejected_while_snapshot_retained = (fault==6U?reject_root_frame(wrong):send_rejected_state(wrong)) &&
          system.copy_state(&snapshot) && snapshot.position[0] == 0.25;
      if (!rejected_while_snapshot_retained && !restore_valid_snapshot()) return 21 + fault;
    }
    if (!rejected_while_snapshot_retained) return 21 + fault;
    valid.state.state_seq()=(++stable_sequence);
    if (!wait_for([&] { send(valid); return system.copy_state(&snapshot) &&
                                  snapshot.envelope.state_seq == stable_sequence; })) return 30 + fault;
  }
  auto invalid_format=valid;invalid_format.invalid_root_file_id=true;
  if(!reject_root_frame(invalid_format) || !system.copy_state(&snapshot) || snapshot.position[0]!=0.25)return 483;
  // A status authority transition must immediately invalidate old state and commands.
  // State carrying the previous authority cannot repopulate the snapshot.
  /**
   * @brief Status改变session/ref/motion必须立即清控制快照和全局目标身份。
   * 旧State不得补回缓存；只有与新Status完整tuple匹配的新State可重新建立观测，不能只更新一个epoch。
   */
  for (unsigned changed_field = 0U; changed_field < 3U; ++changed_field) {
    auto transition = valid;
    transition.status.management_epoch()=((*valid.status.management_epoch()) + 1U);
    transition.state.management_epoch()=((*transition.status.management_epoch()));
    if (changed_field == 0U) transition.status.session_id()=(18U);
    if (changed_field == 1U) transition.status.reference_epoch()=(19U);
    if (changed_field == 2U) transition.status.motion_epoch()=(20U);
    transition.state.state_seq()=(2U);
    const auto before = transport.statistics().invalidations;
    const auto heartbeat_before = transport.statistics().state_heartbeat_gap_stops;
    if (!feedback_until(transition, [&] {
          return observed_status_matches(transition.status,transition.status_source) && transport.statistics().invalidations > before;
        }) || transport.statistics().state_heartbeat_gap_stops != heartbeat_before ||
        system.copy_state(&snapshot) || publish_control_update(positions, six_axis ? 6U : 1U, 0, nullptr) != 0U)
      return 40 + changed_field;
    if (changed_field == 0U) {
      const auto stops_after_invalidation = transport.statistics().state_heartbeat_gap_stops;
      std::this_thread::sleep_for(std::chrono::microseconds(
          parameters.state_publish_period_us * 5U + 20000U));
      if (transport.statistics().state_heartbeat_gap_stops != stops_after_invalidation)
        return 123;
    }
    valid.status = transition.status;
    valid.state.session_id()=((*valid.status.session_id()));
    valid.state.reference_epoch()=((*valid.status.reference_epoch()));
    valid.state.motion_epoch()=((*valid.status.motion_epoch()));
    valid.state.management_epoch()=((*valid.status.management_epoch()));
    valid.state.state_seq()=(1U);
    if (!feedback_until(valid, [&] { return observed_state_matches(valid.state,valid.state_source) &&
          system.copy_state(&snapshot) && snapshot.envelope.state_seq == (*valid.state.state_seq()); })) {
      const auto stats=transport.statistics();
      std::fprintf(stderr,"transition recovery field=%u messages=%llu identity=%llu authority=%llu sequence=%llu ingest=%llu rejected=%llu\n",changed_field,
        (unsigned long long)stats.state_messages,(unsigned long long)stats.state_identity_rejections,(unsigned long long)stats.state_authority_rejections,
        (unsigned long long)stats.state_sequence_rejections,(unsigned long long)stats.state_ingest_rejections,(unsigned long long)stats.rejected_states);
      std::fprintf(stderr,"transition extra stale=%llu skew=%llu bootstrap=%llu stopped=%llu heartbeat=%llu\n",(unsigned long long)stats.stale_payload_rejections,(unsigned long long)stats.clock_skew_rejections,(unsigned long long)stats.bootstrap_rejections,(unsigned long long)stats.stop_latched_state_rejections,(unsigned long long)stats.heartbeat_stops);
      return 44+changed_field;
    }
  }
  valid.status.management_epoch()=(10U);
  valid.state.management_epoch()=(10U);
  valid.state.state_seq()=(2U);
  if (!wait_for([&] { send(valid); return system.copy_state(&snapshot); })) return 51;
  auto asynchronous_epoch = valid;
  asynchronous_epoch.status.motion_epoch()=((*valid.status.motion_epoch()) + 1U);
  asynchronous_epoch.state.motion_epoch()=((*asynchronous_epoch.status.motion_epoch()));
  asynchronous_epoch.state.state_seq()=(3U);
  const auto before_asynchronous_epoch = transport.statistics().invalidations;
  if (!wait_for([&] {
        send(asynchronous_epoch);
        return system.copy_state(&snapshot) && snapshot.envelope.state_seq == 3U;
      }) || transport.statistics().invalidations <= before_asynchronous_epoch) return 53;
  valid = asynchronous_epoch;
  /**
   * @brief 迟到management_epoch9的旧Status不应覆盖已见10或引起再次授权失效。
   * 随后用独立Committed管理回执测试Status暂缺时可接受的精确下一motion代次。
   */
  auto reordered = valid;
  reordered.status.management_epoch()=(9U);
  reordered.status.session_id()=(0U);
  reordered.state.state_seq()=(4U);
  const auto before_reordered = transport.statistics().invalidations;
  if (!wait_for([&] { send(reordered); return system.copy_state(&snapshot); }) ||
      transport.statistics().invalidations != before_reordered) return 52;
  // A committed management reply authorizes the next epoch while status is suppressed.
  const auto next_motion_epoch = (*valid.status.motion_epoch()) + 1U;
  channel->inactive_prime_allowed.store(true);
  set_command_identity({valid.status_source.boot_id, (*valid.status.session_id()),
      (*valid.status.reference_epoch()), next_motion_epoch, 9007199254740997ULL});
  const auto unauthorized_generation = publish_control_update(positions, six_axis ? 6U : 1U, 0, nullptr);
  const auto before_unauthorized = transport.statistics().rejected_commands;
  if (!unauthorized_generation || !system.transfer_control_update() ||
      !wait_for([&] {
        valid.state.state_seq()=(++stable_sequence);
        valid.state.produced_at_us()=(device_now_us());
        send(valid);
        return transport.statistics().rejected_commands > before_unauthorized;
      })) return 66;
  /**
   * @par 内部send_fresh_state回调
   * 仅发合法当前State、递增seq并让send填新Source时间，防止等待管理RPC时先触发无关心跳失效。
   */
  auto send_fresh_state = [&] {
    auto heartbeat = valid;
    heartbeat.send_status = false;
    heartbeat.state.state_seq()=(++stable_sequence);
    heartbeat.state.produced_at_us()=(0U);
    return send(heartbeat);
  };
  /**
   * @par 内部wait_for_management回调
   * 参数为可替换的shared_ptr请求引用；等complete期间每50ms发新State。只有少于100ms的prompt failed才复制同请求原文，10ms退避后排新本地句柄。
   * 失败超过100ms标failed_in_flight而不继续；这是受控fixture策略，不是任意远端超时都安全重发的保证，更不会重置生产Stop deadline。
   * 返回值同时要求心跳pipe发送、等待完成及无长时in-flight失败。
   */
  auto wait_for_management = [&](std::shared_ptr<DeviceManagementRequest> &request) {
    const auto started = std::chrono::steady_clock::now();
    auto attempt_started = started;
    auto next_heartbeat = std::chrono::steady_clock::now();
    auto next_retry = started;
    bool sent = true;
    bool failed_in_flight = false;
    const bool completed = wait_for([&] {
      if (request->complete.load()) return true;
      const auto now = std::chrono::steady_clock::now();
      if (request->failed.load()) {
        // Retry only prompt failures, as with synchronous queue-full before peer delivery.
        if (now - attempt_started >= std::chrono::milliseconds(100)) {
          failed_in_flight = true;
          return true;
        }
        if (now >= next_retry) {
          auto retry = std::make_shared<DeviceManagementRequest>();
          retry->request = request->request;
          {
            std::lock_guard<std::mutex> lock(channel->mutex);
            channel->queued = retry;
          }
          request = std::move(retry);
          attempt_started = now;
          next_retry = now + std::chrono::milliseconds(10);
        }
      }
      if (std::chrono::steady_clock::now() >= next_heartbeat) {
        sent = send_fresh_state() && sent;
        next_heartbeat = std::chrono::steady_clock::now() + std::chrono::milliseconds(50);
      }
      return false;
    });
    if (!sent || !completed || failed_in_flight) {
      Management::Request decoded=owned_fixture::request();
      ((decoded=request->request),message_fields_present(decoded));
      const auto stats = transport.statistics();
      std::fprintf(stderr,
          "management wait: request=%llu sent=%d failed=%d canceled=%d complete=%d elapsed_ms=%lld state=%llu rejected=%llu heartbeat=%llu stale=%llu stop_sent=%llu seq=%u\n",
          static_cast<unsigned long long>((*decoded.request_id())), sent, request->failed.load(),
          request->canceled.load(), request->complete.load(),
          static_cast<long long>(std::chrono::duration_cast<std::chrono::milliseconds>(
              std::chrono::steady_clock::now() - started).count()),
          static_cast<unsigned long long>(stats.state_messages),
          static_cast<unsigned long long>(stats.rejected_states),
          static_cast<unsigned long long>(stats.state_heartbeat_gap_stops),
          static_cast<unsigned long long>(stats.state_stale_stops),
          static_cast<unsigned long long>(stats.stop_requests_sent), stable_sequence);
    }
    return sent && completed && !failed_in_flight;
  };
  /**
   * @brief Begin42回执精确推进下一motion，证明该绑定可在Status暂缺时接纳新State/命令。
   * 之前用同新epoch但无回执的目标应被拒绝；本回执来自受控peer，不代表实际设备授权。
   */
  Management::Request authorization=owned_fixture::request();
  (void)0;
  (void)0;
  authorization.domain_device_id().emplace().assign(device,16);
  const std::uint8_t client[16]{2U};
  authorization.client_instance_id().emplace().assign(client,16);
  authorization.request_id()=(42U);
  authorization.request_seq()=(42U);
  authorization.operation()=(6U);
  authorization.expected_boot_epoch()=(valid.status_source.boot_id);
  authorization.expected_session_id()=((*valid.status.session_id()));
  authorization.expected_reference_epoch()=((*valid.status.reference_epoch()));
  authorization.expected_motion_epoch()=((*valid.status.motion_epoch()));
  authorization.expected_management_epoch()=((*valid.status.management_epoch()));
  authorization.expected_profile_id()=(parameters.profile_id);
  authorization.configured_mask()=(parameters.configured_mask);
  authorization.freshness_ticket()=(9007199254740997ULL);
  auto pending = std::make_shared<DeviceManagementRequest>();
  if (((pending->request=authorization),message_fields_present(authorization)) != true) return 64;
  {
    std::lock_guard<std::mutex> lock(channel->mutex);
    channel->queued = pending;
  }
  if (!wait_for_management(pending)) return 65;
  auto authorized_state = valid;
  authorized_state.send_status = false;
  authorized_state.state.motion_epoch()=(next_motion_epoch);
  authorized_state.state.state_seq()=(++stable_sequence);
  authorized_state.state.produced_at_us()=(device_now_us());
  if (!send(authorized_state) || !wait_for([&] {
        return system.copy_state(&snapshot) && snapshot.envelope.state_seq == stable_sequence;
      })) return 257;
  set_command_identity({valid.status_source.boot_id, (*valid.status.session_id()),
      (*valid.status.reference_epoch()), next_motion_epoch, 9007199254740997ULL});
  const auto next_generation = publish_control_update(positions, six_axis ? 6U : 1U, 0, nullptr);
  if (!next_generation || ::poll(&response, 1, 3000) != 1)
    return 58;
  if ((root_fixture::read_value(input,command)==false) ||
      (*command.motion_epoch()) != next_motion_epoch) return 59;
  auto status_suppressed = valid;
  status_suppressed.send_status = false;
  status_suppressed.state.produced_at_us()=(0U);
  status_suppressed.state.motion_epoch()=(next_motion_epoch);
  status_suppressed.state.state_seq()=(++stable_sequence);
  if (!wait_for([&] {
        send(status_suppressed);
        return system.copy_state(&snapshot) && snapshot.envelope.state_seq == stable_sequence;
      })) {
    const auto stats = transport.statistics();
    std::fprintf(stderr,
        "first new epoch: seq=%u expected=%u states=%llu rejected=%llu invalidations=%llu stops=%llu ready=%u\n",
        snapshot.envelope.state_seq, stable_sequence,
        static_cast<unsigned long long>(stats.state_messages),
        static_cast<unsigned long long>(stats.rejected_states),
        static_cast<unsigned long long>(stats.invalidations),
        static_cast<unsigned long long>(stats.heartbeat_stops), stats.ready);
    return 60;
  }
  status_suppressed.state.committed_producer_seq()=(static_cast<std::uint32_t>(next_generation));
  status_suppressed.state.state_seq()=(++stable_sequence);
  if (!wait_for([&] {
        send(status_suppressed);
        return system.copy_state(&snapshot) && snapshot.envelope.state_seq == stable_sequence;
      })) {
    const auto stats = transport.statistics();
    std::fprintf(stderr,
        "second committed state: seq=%u expected=%u states=%llu rejected=%llu invalidations=%llu stops=%llu\n",
        snapshot.envelope.state_seq, stable_sequence,
        static_cast<unsigned long long>(stats.state_messages),
        static_cast<unsigned long long>(stats.rejected_states),
        static_cast<unsigned long long>(stats.invalidations),
        static_cast<unsigned long long>(stats.heartbeat_stops));
    return 81;
  }
  const auto followup_generation = publish_control_update(positions, six_axis ? 6U : 1U, 0, nullptr);
  if (!followup_generation || !system.transfer_control_update() ||
      ::poll(&response, 1, 3000) != 1) return 82;
  if ((root_fixture::read_value(input,command)==false) ||
      (*command.motion_epoch()) != next_motion_epoch ||
      (*command.producer_seq()) != followup_generation) return 83;
  auto stale_epoch = status_suppressed;
  stale_epoch.state.motion_epoch()=((*valid.status.motion_epoch()));
  stale_epoch.state.state_seq()=(stable_sequence + 1U);
  if (!send_rejected_state(stale_epoch)) return 68;
  auto delayed_status = status_suppressed;
  delayed_status.send_status = true;
  delayed_status.status.committed_producer_seq()=(static_cast<std::uint32_t>(followup_generation));
  delayed_status.state.state_seq()=(++stable_sequence);
  const auto before_delayed_status = transport.statistics();
  if (!wait_for([&] {
        send(delayed_status);
        return transport.statistics().status_messages > before_delayed_status.status_messages;
      }) || transport.statistics().invalidations != before_delayed_status.invalidations ||
      !system.copy_state(&snapshot)) return 63;
  valid.status.motion_epoch()=(next_motion_epoch);
  valid.status.committed_producer_seq()=(static_cast<std::uint32_t>(followup_generation));
  valid.state.motion_epoch()=(next_motion_epoch);
  valid.state.state_seq()=(stable_sequence);
  const auto before_ack = transport.statistics().status_messages;
  if (!wait_for([&] {
        send(valid);
        return transport.statistics().status_messages > before_ack;
      })) return 62;
  /**
   * @brief peer故意回motion跳2，后继State不得借这个异常回执获得许可。
   * 之后Query0只读应答和QueryResult指向另一operation777同样不能授权新的motion。
   */
  authorization.request_id()=(43U);
  authorization.request_seq()=(43U);
  authorization.expected_motion_epoch()=(next_motion_epoch);
  auto jump_request = std::make_shared<DeviceManagementRequest>();
  if (((jump_request->request=authorization),message_fields_present(authorization)) != true) return 69;
  {
    std::lock_guard<std::mutex> lock(channel->mutex);
    channel->queued = jump_request;
  }
  if (!wait_for_management(jump_request)) return 70;
  Management::Response jump_reply=owned_fixture::response();dp::comm::SourceId jump_reply_source{};
  if (((jump_reply=jump_request->response),(jump_reply_source=jump_request->response_source),message_fields_present(jump_reply)) != true) return 127;
  auto status_only = valid;
  status_only.send_state = false;
  const auto before_jump_barrier = transport.statistics().status_messages;
  if (!send(status_only) ||
      !wait_for([&] { return transport.statistics().status_messages > before_jump_barrier; })) return 75;
  auto jumped = valid;
  jumped.send_status = false;
  jumped.state.motion_epoch()=(next_motion_epoch + 2U);
  jumped.state.state_seq()=(stable_sequence + 1U);
  jumped.state.produced_at_us()=(device_now_us());
  if (!send_rejected_state(jumped) || !system.copy_state(&snapshot) ||
      snapshot.envelope.state_seq == (*jumped.state.state_seq())) return 71;
  if (!send_fresh_state()) return 261;
  authorization.request_id()=(44U);
  authorization.request_seq()=(44U);
  authorization.operation()=(0U);
  auto status_query = std::make_shared<DeviceManagementRequest>();
  if (((status_query->request=authorization),message_fields_present(authorization)) != true) return 72;
  {
    std::lock_guard<std::mutex> lock(channel->mutex);
    channel->queued = status_query;
  }
  if (!wait_for_management(status_query)) return 73;
  const auto before_query_barrier = transport.statistics().status_messages;
  if (!send(status_only) ||
      !wait_for([&] { return transport.statistics().status_messages > before_query_barrier; })) return 76;
  auto unlicensed = valid;
  unlicensed.send_status = false;
  unlicensed.state.motion_epoch()=(next_motion_epoch + 1U);
  unlicensed.state.state_seq()=(stable_sequence + 1U);
  unlicensed.state.produced_at_us()=(device_now_us());
  if (!send_rejected_state(unlicensed) || !system.copy_state(&snapshot) ||
      snapshot.envelope.state_seq == (*unlicensed.state.state_seq())) return 74;
  if (!send_fresh_state()) return 262;
  authorization.request_id()=(45U);
  authorization.request_seq()=(45U);
  authorization.operation()=(12U);
  authorization.queried_operation_id()=(43U);
  auto unrelated_query = std::make_shared<DeviceManagementRequest>();
  if (((unrelated_query->request=authorization),message_fields_present(authorization)) != true) return 77;
  {
    std::lock_guard<std::mutex> lock(channel->mutex);
    channel->queued = unrelated_query;
  }
  if (!wait_for_management(unrelated_query)) return 78;
  const auto before_unrelated_barrier = transport.statistics().status_messages;
  if (!send(status_only) ||
      !wait_for([&] { return transport.statistics().status_messages > before_unrelated_barrier; })) return 79;
  // RPC等待会发送合法保鲜State并递增seq；负例必须在等待后分配独立seq，不能与该合法缓存碰撞。
  unlicensed.state.state_seq()=(++stable_sequence);
  unlicensed.state.produced_at_us()=(device_now_us());
  { const bool rejected=send_rejected_state(unlicensed);const bool retained=system.copy_state(&snapshot);
    if(!rejected || !retained || snapshot.envelope.state_seq==(*unlicensed.state.state_seq())) {
      const auto counts=transport.statistics();
      std::fprintf(stderr,"case80 rejected=%u retained=%u cached_seq=%u sent_seq=%u invalidations=%llu stale=%llu identity=%llu authority=%llu stopped=%llu\n",unsigned(rejected),unsigned(retained),snapshot.envelope.state_seq,(*unlicensed.state.state_seq()),(unsigned long long)counts.invalidations,(unsigned long long)counts.stale_payload_rejections,(unsigned long long)counts.state_identity_rejections,(unsigned long long)counts.state_authority_rejections,(unsigned long long)counts.stop_latched_state_rejections);return 80;
    }
  }
  if (!send_fresh_state()) return 263;
  channel->inactive_prime_allowed.store(false);
  valid.state.state_seq()=(++stable_sequence);
  valid.state.produced_at_us()=(0U);
  if (!wait_for([&] { send(valid); return system.copy_state(&snapshot) &&
                                  snapshot.envelope.state_seq == stable_sequence; })) return 108;
  /**
   * @brief 为后续超龄Stop建立最后有效授权tuple；Stop必须使用该回执的motion/management，而非旧Status缓存。
   */
  authorization.request_id()=(46U);
  authorization.request_seq()=(sequence_rebase_mode ? 2100000000U : 46U);
  authorization.operation()=(6U);
  authorization.expected_motion_epoch()=(next_motion_epoch);
  authorization.expected_management_epoch()=((*jump_reply.management_epoch()));
  auto stop_authority = std::make_shared<DeviceManagementRequest>();
  if (((stop_authority->request=authorization),message_fields_present(authorization)) != true) return 130;
  {
    std::lock_guard<std::mutex> lock(channel->mutex);
    channel->queued = stop_authority;
  }
  if (!wait_for_management(stop_authority)) return 131;
  Management::Response stop_authority_reply=owned_fixture::response();dp::comm::SourceId stop_authority_reply_source{};
  if (((stop_authority_reply=stop_authority->response),(stop_authority_reply_source=stop_authority->response_source),message_fields_present(stop_authority_reply)) != true) return 132;
  /**
   * @brief 用SelectMode请求序号2183397839提升发送序号水位。
   * 后继Stop应2183397840且独立request_id，Query再加1并保留原Stop绑定；不能在新请求类型中从旧序号重新开始。
   */
  if (sequence_rebase_mode) {
    auto select_mode = authorization;
    select_mode.request_id()=(2183397839ULL);
    select_mode.request_seq()=(2183397839U);
    select_mode.operation()=(1U);
    auto pending_select = std::make_shared<DeviceManagementRequest>();
    if (((pending_select->request=select_mode),message_fields_present(select_mode)) != true) return 264;
    { std::lock_guard<std::mutex> lock(channel->mutex); channel->queued = pending_select; }
    if (!wait_for_management(pending_select)) return 265;
  }
  /**
   * @brief 故意停止新State等待生产worker失效；保存末次Source产生时刻供Stop后重放负例使用。
   * Host now和Source produced不可直接相减，年龄判定由transport映射层负责。
   */
  const auto accepted_produced_us = snapshot.envelope.produced_at_us;
  const auto before_timeout = transport.statistics().invalidations;
  if (!wait_for([&] { return transport.statistics().invalidations > before_timeout; })) return 100;
  pollfd stop_response{stops_input, POLLIN, 0};
  if (::poll(&stop_response, 1, 3000) != 1) return 128;
  Management::Request stop_request=owned_fixture::request();
  if ((root_fixture::read_value(stops_input,stop_request)==false) ||
      (*stop_request.operation()) != 10U ||
      (*stop_request.expected_motion_epoch()) != (*stop_authority_reply.motion_epoch()) ||
      (*stop_request.expected_management_epoch()) != (*stop_authority_reply.management_epoch())) {
    std::fprintf(stderr, "stop tuple: motion=%u/%u management=%u/%u\n",
        (*stop_request.expected_motion_epoch()), (*stop_authority_reply.motion_epoch()),
        (*stop_request.expected_management_epoch()), (*stop_authority_reply.management_epoch()));
    return 129;
  }
  observed_stop_request_id = (*stop_request.request_id());
  if (sequence_rebase_mode &&
      ((*stop_request.request_seq()) != 2183397840U ||
       (*stop_request.request_id()) == 2183397839ULL ||
       !std::equal(parameters.client_instance_id.begin(), parameters.client_instance_id.end(),
                   (stop_request.client_instance_id()->data())))) return 266;
  pollfd query_response{stops_input, POLLIN, 0};
  if (::poll(&query_response, 1, 3000) != 1) return 139;
  /**
   * @brief 查询必须指向原Stop request_id，Query自己的request_id独立且configured_mask相同。
   * 首个Pending与后续终态即使到了，未见匹配Idle也不能提前stop_requests_confirmed。
   */
  Management::Request first_stop_query=owned_fixture::request();
  if ((root_fixture::read_value(stops_input,first_stop_query)==false) ||
      (*first_stop_query.operation()) != 12U ||
      (*first_stop_query.configured_mask()) != parameters.configured_mask ||
      (*first_stop_query.queried_operation_id()) != (*stop_request.request_id())) return 140;
  if (sequence_rebase_mode &&
      ((*first_stop_query.request_seq()) != (*stop_request.request_seq()) + 1U ||
       (*first_stop_query.request_id()) == (*stop_request.request_id()) ||
       (*first_stop_query.expected_session_id()) != (*stop_request.expected_session_id()) ||
       (*first_stop_query.expected_reference_epoch()) != (*stop_request.expected_reference_epoch()) ||
       (*first_stop_query.expected_boot_epoch()) != (*stop_request.expected_boot_epoch()) ||
       !std::equal(parameters.client_instance_id.begin(), parameters.client_instance_id.end(),
                   (first_stop_query.client_instance_id()->data())))) return 267;
  if (transport.statistics().stop_requests_confirmed != 0U) return 141;
  /**
   * @brief 始终Pending的Stop在transport.stop收尾后仍应未确认。
   * 把unconfirmed作为预期负例通过，不是停止成功。
   */
  if (never_complete_stop) {
    transport.stop();
    if (transport.statistics().stop_requests_confirmed != 0U ||
        transport.statistics().unconfirmed_stop_requests == 0U) return 142;
    return 0;
  }
  if (!replayed_stop_host) {
    if (::poll(&query_response, 1, 3000) != 1) return 144;
    Management::Request terminal_stop_query=owned_fixture::request();
    if ((root_fixture::read_value(stops_input,terminal_stop_query)==false) ||
        (*terminal_stop_query.operation()) != 12U ||
        (*terminal_stop_query.queried_operation_id()) != (*stop_request.request_id()) ||
        (*terminal_stop_query.request_id()) == (*first_stop_query.request_id())) return 145;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    if (transport.statistics().stop_requests_confirmed != 0U) return 146;
  }
  if (!uncommitted_stop_mode && !replayed_stop_host) {
    auto disabling = valid;
    disabling.send_state = false;
    disabling.status.motion_epoch()=((*stop_request.expected_motion_epoch()) + 1U);
    disabling.status.management_epoch()=((*stop_request.expected_management_epoch()) + 1U);
    disabling.status.core_state()=(8U);
    if (!send(disabling) || !wait_for([&] {
        std::lock_guard<std::mutex> lock(channel->mutex);
        Status observed=owned_fixture::status();dp::comm::SourceId observed_source{};
        return ((observed=channel->observation.status),(observed_source=channel->observation.status_source),message_fields_present(observed)) == true &&
            (*observed.core_state()) == 8U;
      })) return 227;
  }
  auto stop_idle = valid;
  stop_idle.status.session_id()=(0U);
  stop_idle.status.reference_epoch()=(0U);
  stop_idle.status.motion_epoch()=((*stop_request.expected_motion_epoch()) + 1U);
  stop_idle.status.management_epoch()=((*stop_request.expected_management_epoch()) + 1U);
  stop_idle.status.core_state()=(1U);
  stop_idle.status.release_committed()=(true);
  stop_idle.status.operation_id()=(exec_expected_stop_operation);
  stop_idle.status.transaction_phase()=(3U);
  stop_idle.state.motion_epoch()=((*stop_idle.status.motion_epoch()));
  stop_idle.state.session_id()=(0U);
  stop_idle.state.reference_epoch()=(0U);
  stop_idle.state.management_epoch()=((*stop_idle.status.management_epoch()));
  stop_idle.state.core_state()=(1U);
  root_fixture::set_position_quality(stop_idle.state,0U);
  root_fixture::set_raw_quality(stop_idle.state,parameters.configured_mask);
  stop_idle.state.state_seq()=(stable_sequence + 1U);
  const double idle_raw[6]{0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
  root_fixture::set_raw_position(stop_idle.state,idle_raw);
  /**
   * @brief 跨exec复用路径读取raw文件，证明匹配Idle确已走实际回调并落盘。
   * 即使接纳层因先前Stop失败拒绝Idle，raw可见也不能把旧operation900回执关联到当前Stop；最终confirmed必须0。
   */
  if (replayed_stop_host) {
    const auto before_idle = transport.statistics();
    const auto idle_sent_at = transport_steady_now_us();
    if (!send(stop_idle) || !wait_for([&] {
          const auto after = transport.statistics();
          return after.status_messages > before_idle.status_messages &&
              after.state_messages > before_idle.state_messages;
        })) return 356;
    // Stop失败后授权已失效，Idle可被接纳层拒绝；用真实回调落盘的wire确认消费。
    if (!wait_for([&] {
          auto *raw_file = std::fopen(capture_path.c_str(), "rb");
          if (!raw_file) return false;
          std::fseek(raw_file, 8L, SEEK_SET);
          bool idle_status_seen = false, idle_state_seen = false;
          std::uint8_t header[kCaptureHeaderSize]{};
          std::array<std::uint8_t, RawEvidenceCollector::kMaxRawSize> bytes{};
          while (std::fread(header, 1U, sizeof(header), raw_file) == sizeof(header)) {
            const auto size = static_cast<unsigned>(header[1]) |
                (static_cast<unsigned>(header[2]) << 8U);
            if (size > bytes.size() || std::fread(bytes.data(), 1U, size, raw_file) != size) break;
            std::uint64_t received_us = 0U;
            for (unsigned i = 0U; i < 8U; ++i)
              received_us |= static_cast<std::uint64_t>(header[3U + i]) << (8U * i);
            if (received_us < idle_sent_at) continue;
            Status observed_status=owned_fixture::status();dp::comm::SourceId observed_status_source{};
            State observed_state=owned_fixture::state();dp::comm::SourceId observed_state_source{};
            if (header[0] == 3U && ((observed_status_source=root_fixture::capture_source(header)),dp::protocol::canonical::Traits<Status>::decode(bytes.data(),size,&observed_status)) == true)
              idle_status_seen = idle_status_seen || ((*observed_status.core_state()) == 1U &&
                  (*observed_status.operation_id()) == 900U &&
                  (*observed_status.motion_epoch()) == (*stop_idle.status.motion_epoch()) &&
                  (*observed_status.management_epoch()) == (*stop_idle.status.management_epoch()));
            if (header[0] == 2U && ((observed_state_source=root_fixture::capture_source(header)),dp::protocol::canonical::Traits<State>::decode(bytes.data(),size,&observed_state)) == true)
              idle_state_seen = idle_state_seen || ((*observed_state.core_state()) == 1U &&
                  (*observed_state.motion_epoch()) == (*stop_idle.state.motion_epoch()) &&
                  (*observed_state.management_epoch()) == (*stop_idle.state.management_epoch()) &&
                  (*observed_state.state_seq()) == (*stop_idle.state.state_seq()));
          }
          std::fclose(raw_file);
          return idle_status_seen && idle_state_seen;
        })) return 361;
    std::fprintf(stderr, "跨exec旧Idle已由回调raw消费：op900及motion/management/State seq匹配\n");
    if (!wait_for([&] { return transport.statistics().unconfirmed_stop_requests != 0U; }))
      return 354;
    transport.stop();
    const auto stats = transport.statistics();
    return stats.stop_requests_confirmed == 0U &&
        stats.unconfirmed_stop_requests != 0U ? 0 : 355;
  }
  /**
   * @brief Idle Status故意release_committed=false并推进管理代次，避免被旧Status丢弃路径掩盖。
   * 核Status/State真实进入观测，仍须保持Stop未确认。
   */
  if (uncommitted_idle_status_mode) {
    stop_idle.status.release_committed()=(false);
    stop_idle.status.management_epoch()=((*stop_idle.status.management_epoch()) + 1U);
    stop_idle.state.management_epoch()=((*stop_idle.status.management_epoch()));
    const auto before = transport.statistics();
    if (!send(stop_idle) || !wait_for([&] {
          const auto after = transport.statistics();
          if (after.status_messages <= before.status_messages ||
              after.state_messages <= before.state_messages) return false;
          std::lock_guard<std::mutex> lock(channel->mutex);
          Status observed_status=owned_fixture::status();dp::comm::SourceId observed_status_source{};
          State observed_state=owned_fixture::state();dp::comm::SourceId observed_state_source{};
          return ((observed_status=channel->observation.status),(observed_status_source=channel->observation.status_source),message_fields_present(observed_status)) == true &&
              ((observed_state=channel->observation.state),(observed_state_source=channel->observation.state_source),message_fields_present(observed_state)) == true &&
              (*observed_status.core_state()) == 1U &&
              !(*observed_status.release_committed()) &&
              (*observed_status.management_epoch()) == (*stop_idle.status.management_epoch()) &&
              (*observed_state.core_state()) == 1U &&
              (*observed_state.state_seq()) == (*stop_idle.state.state_seq());
        })) return 273;
    transport.stop();
    const auto stats = transport.statistics();
    return stats.stop_requests_confirmed == 0U &&
        stats.unconfirmed_stop_requests != 0U ? 0 : 274;
  }
  /**
   * @brief Stop Query“终态”本身未release_committed，即使随后发送合法样式Idle也不能补足该强回执。
   */
  if (uncommitted_stop_mode) {
    if (!send(stop_idle)) return 269;
    if (!wait_for([&] { return transport.statistics().unconfirmed_stop_requests != 0U; }))
      return 275;
    transport.stop();
    const auto stats = transport.statistics();
    return stats.stop_requests_confirmed == 0U &&
        stats.unconfirmed_stop_requests != 0U ? 0 : 268;
  }
  /**
   * @brief 在已见Disabling8后发未Committed Idle Status，要求保留Disabling观测。
   * 随后补完整Idle Status/State才允许confirmed，防止仅core_state=Idle单字段越过释放证明。
   */
  auto uncommitted_idle = stop_idle;
  uncommitted_idle.send_state = false;
  uncommitted_idle.status.release_committed()=(false);
  const auto before_uncommitted = transport.statistics().status_messages;
  if (!send(uncommitted_idle) || !wait_for([&] {
      return transport.statistics().status_messages > before_uncommitted;
    })) return 228;
  {
    std::lock_guard<std::mutex> lock(channel->mutex);
    Status observed=owned_fixture::status();dp::comm::SourceId observed_source{};
    if (((observed=channel->observation.status),(observed_source=channel->observation.status_source),message_fields_present(observed)) != true ||
        (*observed.core_state()) != 8U) return 229;
  }
  if (!wait_for([&] {
        send(stop_idle);
        return transport.statistics().stop_requests_confirmed > 0U;
      }) ||
      transport.statistics().unconfirmed_stop_requests != 0U) return 133;
  if (six_axis && (transport.statistics().state_heartbeat_gap_stops == 0U ||
      transport.statistics().last_state_gap_us < parameters.state_publish_period_us * 5U)) return 122;
  /**
   * @brief Stop确认后旧Status不能回退当前motion；旧seq/旧Source时刻及旧授权State均不得复活控制快照。
   */
  auto stale_status_after_stop = valid;
  stale_status_after_stop.send_state = false;
  const auto stale_status_count = transport.statistics().status_messages;
  if (!send(stale_status_after_stop) || !wait_for([&] {
      return transport.statistics().status_messages > stale_status_count;
    })) return 137;
  {
    std::lock_guard<std::mutex> lock(channel->mutex);
    Status observed_status=owned_fixture::status();dp::comm::SourceId observed_status_source{};
    if (((observed_status=channel->observation.status),(observed_status_source=channel->observation.status_source),message_fields_present(observed_status)) != true ||
        (*observed_status.motion_epoch()) != (*stop_idle.status.motion_epoch())) return 138;
  }
  auto status_only_after_timeout = valid;
  status_only_after_timeout.send_state = false;
  status_only_after_timeout.status.session_id()=(0U);
  status_only_after_timeout.status.reference_epoch()=(0U);
  status_only_after_timeout.status.motion_epoch()=((*stop_request.expected_motion_epoch()) + 1U);
  status_only_after_timeout.status.management_epoch()=((*stop_request.expected_management_epoch()) + 1U);
  const auto before_rebind_status = transport.statistics().status_messages;
  if (!wait_for([&] {
        send(status_only_after_timeout);
        return transport.statistics().status_messages > before_rebind_status;
      })) return 101;
  auto replay = valid;
  replay.send_status = false;
  replay.state.state_seq()=(stable_sequence);
  replay.state.produced_at_us()=(device_now_us());
  if (!send_rejected_state(replay) || system.copy_state(&snapshot)) return 102;
  replay.state.state_seq()=(stable_sequence + 1U);
  replay.state.produced_at_us()=(accepted_produced_us);
  if (!send_rejected_state(replay) || system.copy_state(&snapshot)) return 107;
  replay.state.state_seq()=(stable_sequence + 1U);
  replay.state.produced_at_us()=(accepted_produced_us + 1U);
  if (!send_rejected_state(replay) || system.copy_state(&snapshot)) return 126;
  valid.state.state_seq()=(++stable_sequence);
  if (!send_rejected_state(valid) || system.copy_state(&snapshot)) return 103;
  if (six_axis) {
    auto released = valid;
    released.status = status_only_after_timeout.status;
    released.status.core_state()=(1U);
    released.state.core_state()=(1U);
    released.state.session_id()=(0U);
    released.state.reference_epoch()=(0U);
    released.state.motion_epoch()=((*stop_request.expected_motion_epoch()) + 1U);
    released.state.management_epoch()=((*stop_request.expected_management_epoch()) + 1U);
    root_fixture::set_position_quality(released.state,0U);
    root_fixture::set_velocity_quality(released.state,63U);
    root_fixture::set_raw_quality(released.state,63U);
    released.state.state_seq()=(stable_sequence + 1U);
    released.state.produced_at_us()=(device_now_us());
    const double raw_motor_rev[6]{0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
    root_fixture::set_raw_position(released.state,raw_motor_rev);
    if (!wait_for([&] {
          send(released);
          std::lock_guard<std::mutex> lock(channel->mutex);
          State observed=owned_fixture::state();dp::comm::SourceId observed_source{};
          return channel->observation.state_received_us != 0U &&
              ((observed=channel->observation.state),(observed_source=channel->observation.state_source),message_fields_present(observed)) == true &&
              (*observed.core_state()) == 1U && (*observed.state_seq()) == (*released.state.state_seq());
        }) || system.copy_state(&snapshot)) return 97;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  (void)system.copy_state(&snapshot);
  /**
   * @brief 保存首拒再注入新boot；最终要求首拒时间/reason/observed boot未变、拒绝总数继续增。
   * bootstrap前的expected身份尚未知，不能伪造全零valid身份充当已绑定上下文。
   */
  const auto first_before_reboot=transport.diagnostic_snapshot();
  // 首拒发生在bootstrap之前时expected身份尚不可用，必须保留valid=false，不能伪造0身份。
  if(!first_before_reboot.first_present || first_before_reboot.first.host_us==0U ||
     !first_before_reboot.first.observed.valid) {
    std::fprintf(stderr,"first rejection present=%u observed=%u reason=%u detail=%s\n",unsigned(first_before_reboot.first_present),unsigned(first_before_reboot.first.observed.valid),unsigned(first_before_reboot.first.reason),first_before_reboot.first.detail.data());return 480;
  }
  /**
   * @brief 新boot使旧状态/目标身份立即失效；旧boot后续到达也不能重建缓存。
   * Stop已锁存时即使新boot State seq1仍拒绝，不能把重启看成自动重新授权。
   */
  auto rebooted = valid;
  rebooted.root_generation=11U;
  rebooted.status_source.boot_id=(11U);
  rebooted.state_source.boot_id=(11U);
  root_fixture::set_position_quality(rebooted.state,0U);
  const auto invalidations = transport.statistics().invalidations;
  send(rebooted);
  const auto reboot_deadline=std::chrono::steady_clock::now()+std::chrono::seconds(12);
  bool reboot_observed=false;
  while(std::chrono::steady_clock::now()<reboot_deadline && !reboot_observed) {
    // 新实例订阅由正式SYSTEM异步建立；持续提供该代次heartbeat，直到目标样本真实接纳。
    if(!send(rebooted))return 19;
    reboot_observed=[&] {
        if (transport.statistics().invalidations <= invalidations) return false;
        std::lock_guard<std::mutex> lock(channel->mutex);
        Status observed_status=owned_fixture::status();dp::comm::SourceId observed_status_source{};
        return ((observed_status=channel->observation.status),(observed_status_source=channel->observation.status_source),message_fields_present(observed_status)) == true && observed_status_source.boot_id == 11U;
      }();
    if(!reboot_observed)std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  if(!reboot_observed || system.copy_state(&snapshot) || publish_control_update(positions,1U,0,nullptr)!=0U) {
    const auto stats=transport.statistics();const auto observation=channel->diagnostic_observation();Status observed=owned_fixture::status();dp::comm::SourceId observed_source{};
    (void)((observed=observation.status),(observed_source=observation.status_source),message_fields_present(observed));
    std::fprintf(stderr,"ROOT_REBOOT_FAILED observed=%u boot=%llu invalid=%llu before=%llu status=%llu\n",
      unsigned(reboot_observed),(unsigned long long)observed_source.boot_id,(unsigned long long)stats.invalidations,
      (unsigned long long)invalidations,(unsigned long long)stats.status_messages);
    return 19;
  }
  auto late_old_source=valid;late_old_source.invalid_root_source_boot=7U;
  late_old_source.status_source.boot_id=11U; // 处理屏障须为新owner合法Status，坏State仍明确携旧Source7。
  late_old_source.state_source.boot_id=11U;
  if(!reject_root_frame(late_old_source) || system.copy_state(&snapshot))return 20;
  auto post_stop = valid;
  post_stop.status_source.boot_id=(11U);
  post_stop.state_source.boot_id=(11U);
  post_stop.state.state_seq()=(1U);
  post_stop.state.produced_at_us()=(device_now_us());
  if (!send_rejected_state(post_stop) || system.copy_state(&snapshot)) return 104;
  const auto diagnostics = transport.statistics();
  final_worker_diagnostic_snapshot=diagnostics.diagnostics;
  if(!diagnostics.diagnostics.first_present ||
     diagnostics.diagnostics.first.host_us!=first_before_reboot.first.host_us ||
     diagnostics.diagnostics.first.reason!=first_before_reboot.first.reason ||
     diagnostics.diagnostics.first.observed.boot!=first_before_reboot.first.observed.boot ||
     diagnostics.diagnostics.rejections<=first_before_reboot.rejections) return 481;
  if (diagnostics.state_callbacks < diagnostics.state_messages ||
      diagnostics.state_identity_rejections == 0U ||
      diagnostics.state_authority_rejections == 0U ||
      diagnostics.state_sequence_rejections == 0U ||
      diagnostics.stop_latched_state_rejections == 0U) {
    std::fprintf(stderr,"ROOT_FINAL_COUNTS callbacks=%llu messages=%llu identity=%llu authority=%llu sequence=%llu stop=%llu\n",
      (unsigned long long)diagnostics.state_callbacks,(unsigned long long)diagnostics.state_messages,
      (unsigned long long)diagnostics.state_identity_rejections,(unsigned long long)diagnostics.state_authority_rejections,
      (unsigned long long)diagnostics.state_sequence_rejections,(unsigned long long)diagnostics.stop_latched_state_rejections);
    return 136;
  }
  transport.stop();
  std::printf("component_only: synthetic_peer axes=%u full identity, device/profile/mask/age/sequence rejection, authority/boot invalidation PASS\n",
              six_axis ? 6U : 1U);
  return 0;
}
}  // namespace
/**
 * @brief 创建独立PTY桥/peer子进程并运行一个Host场景。
 * @param six_axis true使用mask63/六位置，false使用J1 mask1。
 * @param ack_timeout_mode 缩短组ACK预算为30000µs并持续提供未ACK新State。
 * @param default_ack_age_mode 停发State，区分State超龄与ACK未确认/超时。
 * @param never_complete_stop peer一直Pending，收尾必须保留unconfirmed。
 * @param revoke_owner_before_stop_receipt peer Pending时先清owner。
 * @param heartbeat_budget_mode 使用250ms显式State间隔预算。
 * @param inactive_prime_mode 未激活硬件下允许测试首个保持目标发布。
 * @param cold_idle_gap_mode 冷Idle断流重观测，不能伪造已授权参考反馈。
 * @param cold_fault_gap_mode 同为冷态但Status Fault，断流不得走无害Idle例外。
 * @param raw_capture_mode 开启本地raw文件采集。
 * @param raw_bad_release_mode 注入不相关parent的释放Status。
 * @param raw_incomplete_mode 已expect但无终态即stop，要求Incomplete。
 * @param raw_many_mode 顺序采集run3..11并拒绝旧run Status。
 * @param raw_order_case raw/ACK/cleanup场景编号，见main模式表与各分支。
 * @param complete_order_mode Complete顺序/保持排空/刷新场景编号。
 * @param release_deactivate_mode 显式释放与生命周期停机顺序编号。
 * @param ticket_mode 1续票据后发pending，2旧票据不发，3授权变化不得借续票据发目标。
 * @param reject_during_ack_mode 待ACK时收到明确新拒绝，须停机而非等ACK超时。
 * @param sequence_rebase_mode Stop/Query从最近真实管理序号后继续。
 * @param uncommitted_stop_mode Stop Query终态缺release_committed。
 * @param uncommitted_idle_status_mode Idle Status缺release_committed。
 * @return 原样返回run_parent业务检查码；PTY/pipe/fork失败为30/31/32。
 * @details 子进程只留peer需要的pipe端；父进程启桥后调用run_parent。完成或失败均SIGTERM并waitpid回收peer，再关父端pipe。
 * 这里没有按peer退出码额外判成功，主要由Host的期限、观测、消息/统计断言决定；不得声称已完整验证所有peer内部错误路径。
 */
int run_component_case(bool six_axis, bool ack_timeout_mode, bool default_ack_age_mode,
                       bool never_complete_stop = false,
                       bool heartbeat_budget_mode = false,
                       bool revoke_owner_before_stop_receipt = false,
                       bool inactive_prime_mode = false,
                       bool cold_idle_gap_mode = false,
                       bool cold_fault_gap_mode = false,
                       bool raw_capture_mode = false,
                       bool raw_bad_release_mode = false,
                       bool raw_incomplete_mode = false,
                       bool raw_many_mode = false,
                       unsigned raw_order_case = 0U,
                       unsigned complete_order_mode = 0U,
                       unsigned release_deactivate_mode = 0U,
                       unsigned ticket_mode = 0U,
                       bool reject_during_ack_mode = false,
                       bool sequence_rebase_mode = false,
                       bool uncommitted_stop_mode = false,
                       bool uncommitted_idle_status_mode = false) {
  SerialBridge bridge;
  if (!root_tcp_boot_case && !bridge.open_pair()) return 30;
  const char* router=std::getenv("DUMMY_R11_TCP_ROUTER");
  if(root_tcp_boot_case && (!router || std::strncmp(router,"tcp/127.0.0.1:",14)!=0))return 35;
  const std::string peer_endpoint=root_tcp_boot_case?router:bridge.endpoint(0);
  const std::string host_endpoint=root_tcp_boot_case?router:bridge.endpoint(1);
  int requests[2]{}, responses[2]{};
  int stops[2]{};
  if (::pipe(requests) != 0 || ::pipe(responses) != 0 || ::pipe(stops) != 0) return 31;
  const auto child = ::fork();
  if (child < 0) return 32;
  if (child == 0) {
    if(!root_tcp_boot_case)bridge.close_child_fds();
    ::close(requests[1]); ::close(responses[0]); ::close(stops[0]);
#ifdef DUMMY_REF_ROOT_COMPONENT_PEER_PATH
    char input_fd[16],output_fd[16],stop_fd[16],fault[16];
    std::snprintf(input_fd,sizeof(input_fd),"%d",requests[0]);std::snprintf(output_fd,sizeof(output_fd),"%d",responses[1]);
    std::snprintf(stop_fd,sizeof(stop_fd),"%d",stops[1]);std::snprintf(fault,sizeof(fault),"%u",release_receipt_fault);
    ::execl(DUMMY_REF_ROOT_COMPONENT_PEER_PATH,DUMMY_REF_ROOT_COMPONENT_PEER_PATH,peer_endpoint.c_str(),input_fd,output_fd,stop_fd,
      never_complete_stop?"1":"0",revoke_owner_before_stop_receipt?"1":"0",release_deactivate_mode==7U?"1":"0",
      sequence_rebase_mode?"1":"0",uncommitted_stop_mode?"1":"0",source_cancel_fault?source_cancel_fault:"",fault,
      inject_peer_state_queue_full_once?"1":"0",static_cast<char*>(nullptr));
    ::_exit(33);
#else
    ::_exit(33); // 独立测试peer路径由本测试target提供；不在PIC角色内启动另一个资源ABI。
#endif
  }
  if(!root_tcp_boot_case)bridge.start();
  ::close(requests[0]); ::close(responses[1]); ::close(stops[1]);
  const auto result = run_parent(host_endpoint, responses[0], requests[1], stops[0],
                                 six_axis, ack_timeout_mode, default_ack_age_mode,
                                 never_complete_stop, heartbeat_budget_mode, inactive_prime_mode,
                                 cold_idle_gap_mode, cold_fault_gap_mode,
                                 raw_capture_mode, raw_bad_release_mode,
                                 raw_incomplete_mode, raw_many_mode, raw_order_case, complete_order_mode,
                                 release_deactivate_mode, ticket_mode, reject_during_ack_mode,
                                 sequence_rebase_mode, uncommitted_stop_mode,
                                 uncommitted_idle_status_mode);
  if (result != 0) std::fprintf(stderr, "component_only assertion failed: %d\n", result);
  ::kill(child, SIGTERM);
  ::waitpid(child, nullptr, 0);
  ::close(requests[1]); ::close(responses[0]); ::close(stops[0]);
  return result;
}
/**
 * @brief 解析普通组件、Complete、释放、raw顺序及源码Cancel协调场景。
 * @param argc 0附加参数为J1，至少1附加参数为六轴；场景通常在argv[2]。
 * @param argv 参数数组，Cancel协调模式会把argv[2]重定向为running_deactivate。
 * @return 所选场景检查码；普通完整六轴成功后还跑default_ack_age负例。
 * @note 本入口不是严格枚举parser，未识别第三参数可落到默认ACK超时选择；应按已注册测试的确切参数调用。
 */
static int run_transport_test_cases(int argc, char **argv) {
  // 正式Root重启场景固定六轴/mask63；普通无参J1场景仍保留独立用途。
  if(argc==2 && std::strcmp(argv[1],"protocol_r11_tcp_boot")==0){root_tcp_boot_case=true;return run_component_case(true,false,false);}
  if(argc==2 && std::strcmp(argv[1],"protocol_r11_presence")==0){root_presence_case=true;return run_component_case(false,false,false);}
  if(argc==2 && std::strcmp(argv[1],"owner_safety_stop")==0) {
    owner_safety_stop_case=true;
    return run_component_case(true,false,false,false,false,false,false,false,false,true);
  }

  // 仅新增固定Source测试模式；普通registered场景保原路径。
  if(argc==3 && std::strncmp(argv[2],"cancel_coordination_",20U)==0) {
    source_cancel_coordination=true;
    const char* mode=argv[2]+20U;
    source_cancel_fault=std::strcmp(mode,"positive")==0 ? nullptr : mode;
    argv[2]=const_cast<char*>("running_deactivate");
  }

  /**
   * @brief 选择Hold Status变体71..82后映射raw59..70，复用同一真实断言分支。
   * terminal7命令还可叠加final4/5/6/9、错/零链、failure字段与Stopping不ready；不是另创一个未经检查的默认case。
   */
  cancel_hold_case=0U;
  if(argc==3 && std::strncmp(argv[2],"raw_order_cancel_origin",23U)==0) {
    const char* suffix=argv[2]+23U;
    if(std::strcmp(suffix,"_wrong_terminal")==0)cancel_origin_fault=1U;
    else if(std::strcmp(suffix,"_first_can_terminal")==0)cancel_origin_fault=2U;
    else if(std::strcmp(suffix,"_late_rebase")==0)cancel_origin_fault=3U;
    else if(std::strcmp(suffix,"_nonzero_groups")==0)cancel_origin_fault=4U;
    else if(std::strcmp(suffix,"_wrong_attempt")==0)cancel_origin_fault=5U;
    else if(std::strcmp(suffix,"_future_origin")==0)cancel_origin_fault=6U;
    else if(*suffix!='\0')return 415;
    cancel_origin_case=true;argv[2]=const_cast<char*>("raw_order_cancel_stale_status_fresh_state");
  }
  if(argc==3 && std::strncmp(argv[2],"raw_order_cancel_pending_flush",30U)==0) {
    const char* suffix=argv[2]+30U;
    if(std::strcmp(suffix,"_bad_safety")==0)cancel_pending_flush_fault=1U;
    else if(std::strcmp(suffix,"_non_hold")==0)cancel_pending_flush_fault=2U;
    else if(std::strcmp(suffix,"_bad_mask")==0)cancel_pending_flush_fault=3U;
    else if(std::strcmp(suffix,"_bad_motion")==0)cancel_pending_flush_fault=4U;
    else if(std::strcmp(suffix,"_wrong_chain")==0)cancel_pending_flush_fault=5U;
    else if(std::strcmp(suffix,"_io_failure")==0)cancel_pending_flush_fault=6U;
    else if(std::strcmp(suffix,"_consumer_snapshot")==0)cancel_pending_flush_fault=7U;
    else if(*suffix!='\0')return 405;
    cancel_pending_flush_case=true;argv[2]=const_cast<char*>("raw_order_cancel_stale_status_fresh_state");
  }
  if(argc==3 && std::strcmp(argv[2],"raw_order_cancel_hold_status_fresh_state")==0) cancel_hold_case=71U;
  if(argc==3 && std::strcmp(argv[2],"raw_order_cancel_hold_status_bad_receipt_boot")==0) cancel_hold_case=72U;
  if(argc==3 && std::strcmp(argv[2],"raw_order_cancel_hold_status_bad_receipt_session")==0) cancel_hold_case=73U;
  if(argc==3 && std::strcmp(argv[2],"raw_order_cancel_hold_status_bad_receipt_run")==0) cancel_hold_case=74U;
  if(argc==3 && std::strcmp(argv[2],"raw_order_cancel_hold_status_bad_receipt_chain")==0) cancel_hold_case=75U;
  if(argc==3 && std::strcmp(argv[2],"raw_order_cancel_hold_status_stale_state")==0) cancel_hold_case=76U;
  if(argc==3 && std::strcmp(argv[2],"raw_order_cancel_hold_status_non_hold_state")==0) cancel_hold_case=77U;
  if(argc==3 && std::strcmp(argv[2],"raw_order_cancel_hold_status_not_ready_state")==0) cancel_hold_case=78U;
  if(argc==3 && std::strcmp(argv[2],"raw_order_cancel_hold_status_management_mismatch")==0) cancel_hold_case=79U;
  if(argc==3 && std::strcmp(argv[2],"raw_order_cancel_hold_status_no_safety_hold")==0) cancel_hold_case=80U;
  if(argc==3 && std::strcmp(argv[2],"raw_order_cancel_hold_status_bad_safety_motion")==0) cancel_hold_case=81U;
  if(argc==3 && std::strcmp(argv[2],"raw_order_cancel_hold_status_bad_successor_motion")==0) cancel_hold_case=82U;
  cancel_hold_status_variant=cancel_hold_case>=71U && cancel_hold_case<=82U;
  if(argc==3 && std::strcmp(argv[2],"raw_order_cancel_hold_terminal7_fresh_state")==0){cancel_hold_case=71U;cancel_hold_status_variant=true;cancel_status_terminal7_variant=true;cancel_status_bad_flags=0U;cancel_state_stopping_notready=false;}
  if(argc==3 && std::strcmp(argv[2],"raw_order_cancel_hold_terminal7_old_final0")==0){cancel_hold_case=71U;cancel_hold_status_variant=true;cancel_status_terminal7_variant=false;cancel_status_bad_flags=0U;cancel_state_stopping_notready=false;}
  if(argc==3 && std::strcmp(argv[2],"raw_order_cancel_hold_terminal7_stopping_state_not_ready")==0){cancel_hold_case=77U;cancel_hold_status_variant=true;cancel_status_terminal7_variant=true;cancel_status_bad_flags=0U;cancel_state_stopping_notready=true;}
  if(argc==3 && std::strcmp(argv[2],"raw_order_cancel_hold_terminal7_final4")==0){cancel_hold_case=71U;cancel_hold_status_variant=true;cancel_status_terminal7_variant=true;cancel_status_bad_flags=1U;cancel_state_stopping_notready=false;}
  if(argc==3 && std::strcmp(argv[2],"raw_order_cancel_hold_terminal7_final5")==0){cancel_hold_case=71U;cancel_hold_status_variant=true;cancel_status_terminal7_variant=true;cancel_status_bad_flags=2U;cancel_state_stopping_notready=false;}
  if(argc==3 && std::strcmp(argv[2],"raw_order_cancel_hold_terminal7_final6")==0){cancel_hold_case=71U;cancel_hold_status_variant=true;cancel_status_terminal7_variant=true;cancel_status_bad_flags=3U;cancel_state_stopping_notready=false;}
  if(argc==3 && std::strcmp(argv[2],"raw_order_cancel_hold_terminal7_final9")==0){cancel_hold_case=71U;cancel_hold_status_variant=true;cancel_status_terminal7_variant=true;cancel_status_bad_flags=4U;cancel_state_stopping_notready=false;}
  if(argc==3 && std::strcmp(argv[2],"raw_order_cancel_hold_terminal7_wrong_status_chain")==0){cancel_hold_case=71U;cancel_hold_status_variant=true;cancel_status_terminal7_variant=true;cancel_status_bad_flags=5U;cancel_state_stopping_notready=false;}
  if(argc==3 && std::strcmp(argv[2],"raw_order_cancel_hold_terminal7_zero_status_chain")==0){cancel_hold_case=71U;cancel_hold_status_variant=true;cancel_status_terminal7_variant=true;cancel_status_bad_flags=6U;cancel_state_stopping_notready=false;}
  if(argc==3 && std::strcmp(argv[2],"raw_order_cancel_hold_terminal7_failure_source")==0){cancel_hold_case=71U;cancel_hold_status_variant=true;cancel_status_terminal7_variant=true;cancel_status_bad_flags=7U;cancel_state_stopping_notready=false;}
  if(argc==3 && std::strcmp(argv[2],"raw_order_cancel_hold_terminal7_failure_axis")==0){cancel_hold_case=71U;cancel_hold_status_variant=true;cancel_status_terminal7_variant=true;cancel_status_bad_flags=8U;cancel_state_stopping_notready=false;}
  if(argc==3 && std::strcmp(argv[2],"raw_order_cancel_hold_terminal7_failure_reason")==0){cancel_hold_case=71U;cancel_hold_status_variant=true;cancel_status_terminal7_variant=true;cancel_status_bad_flags=9U;cancel_state_stopping_notready=false;}
  if(argc==3 && std::strcmp(argv[2],"raw_order_cancel_hold_terminal7_bad_receipt_boot")==0){cancel_hold_case=72U;cancel_hold_status_variant=true;cancel_status_terminal7_variant=true;cancel_status_bad_flags=0U;cancel_state_stopping_notready=false;}
  if(argc==3 && std::strcmp(argv[2],"raw_order_cancel_hold_terminal7_bad_receipt_session")==0){cancel_hold_case=73U;cancel_hold_status_variant=true;cancel_status_terminal7_variant=true;cancel_status_bad_flags=0U;cancel_state_stopping_notready=false;}
  if(argc==3 && std::strcmp(argv[2],"raw_order_cancel_hold_terminal7_bad_receipt_run")==0){cancel_hold_case=74U;cancel_hold_status_variant=true;cancel_status_terminal7_variant=true;cancel_status_bad_flags=0U;cancel_state_stopping_notready=false;}
  if(argc==3 && std::strcmp(argv[2],"raw_order_cancel_hold_terminal7_bad_receipt_chain")==0){cancel_hold_case=75U;cancel_hold_status_variant=true;cancel_status_terminal7_variant=true;cancel_status_bad_flags=0U;cancel_state_stopping_notready=false;}
  if(argc==3 && std::strcmp(argv[2],"raw_order_cancel_hold_terminal7_stale_state")==0){cancel_hold_case=76U;cancel_hold_status_variant=true;cancel_status_terminal7_variant=true;cancel_status_bad_flags=0U;cancel_state_stopping_notready=false;}
  if(argc==3 && std::strcmp(argv[2],"raw_order_cancel_hold_terminal7_non_hold_state")==0){cancel_hold_case=77U;cancel_hold_status_variant=true;cancel_status_terminal7_variant=true;cancel_status_bad_flags=0U;cancel_state_stopping_notready=false;}
  if(argc==3 && std::strcmp(argv[2],"raw_order_cancel_hold_terminal7_not_ready_state")==0){cancel_hold_case=78U;cancel_hold_status_variant=true;cancel_status_terminal7_variant=true;cancel_status_bad_flags=0U;cancel_state_stopping_notready=false;}
  if(argc==3 && std::strcmp(argv[2],"raw_order_cancel_hold_terminal7_management_mismatch")==0){cancel_hold_case=79U;cancel_hold_status_variant=true;cancel_status_terminal7_variant=true;cancel_status_bad_flags=0U;cancel_state_stopping_notready=false;}
  if(argc==3 && std::strcmp(argv[2],"raw_order_cancel_hold_terminal7_no_safety_hold")==0){cancel_hold_case=80U;cancel_hold_status_variant=true;cancel_status_terminal7_variant=true;cancel_status_bad_flags=0U;cancel_state_stopping_notready=false;}
  if(argc==3 && std::strcmp(argv[2],"raw_order_cancel_hold_terminal7_bad_safety_motion")==0){cancel_hold_case=81U;cancel_hold_status_variant=true;cancel_status_terminal7_variant=true;cancel_status_bad_flags=0U;cancel_state_stopping_notready=false;}
  if(argc==3 && std::strcmp(argv[2],"raw_order_cancel_hold_terminal7_bad_successor_motion")==0){cancel_hold_case=82U;cancel_hold_status_variant=true;cancel_status_terminal7_variant=true;cancel_status_bad_flags=0U;cancel_state_stopping_notready=false;}
  if(cancel_hold_status_variant)std::fprintf(stderr,"CANCEL_HOLD_VARIANT_MODE=%u ORIGINAL_BASE_MODE=%u STATUS=Hold4_final0\n",cancel_hold_case,cancel_hold_case-12U);

  if (argc == 2 && std::strcmp(argv[1], "peer_publish_backpressure") == 0)
    return test_peer_publish_backpressure();
  if (argc == 2 && std::strcmp(argv[1], "peer_state_queue_full_once") == 0) {
    inject_peer_state_queue_full_once = true;
    return run_component_case(true, false, false);
  }
  if (argc == 2 && std::strcmp(argv[1], "manual_fault_diagnostic") == 0) {
    manual_fault_diagnostic_mode = true;
    return run_component_case(true, false, false, false, false, false, false,
                              true, false, true);
  }
  if (argc == 2 && std::strcmp(argv[1], "manual_retained_release") == 0) {
    manual_release_mode = true;
    return run_component_case(true, false, false, false, false, false, false,
                              false, false, false, false, false, false, 0U, 0U,
                              8U);
  }
  if (argc == 2 && std::strcmp(argv[1], "manual_historical_idle") == 0) {
    manual_history_mode = true;
    return run_component_case(true, false, false, false, false, false, false,
                              true, false, true);
  }
  if (argc == 3 && std::strcmp(argv[1], "released_child") == 0) {
    released_child_case =
        static_cast<unsigned>(std::strtoul(argv[2], nullptr, 10));
    if (released_child_case < 1U || released_child_case > 6U)
      return 875;
    return run_component_case(true, false, false, false, false, false, false,
                              false, false, true);
  }
  if (argc >= 2 && std::strcmp(argv[1], "manual_recovered_release_idle") == 0) {
    manual_recovered_release_mode = true;
    manual_recovered_idle_mode = true;
    if (argc == 3) {
      const char *faults[]{"current_reason", "fault",           "source",
                           "not_sent",       "reference_drift", "not_committed",
                           "other_kind"};
      bool found = false;
      for (unsigned i = 0; i < 7; ++i)
        if (std::strcmp(argv[2], faults[i]) == 0) {
          manual_recovered_idle_fault = i + 1;
          found = true;
        }
      if (!found)
        return 864;
    }
    return run_component_case(true, false, false, false, false, false, false,
                              true, false, true);
  }
  if (argc == 2 && std::strcmp(argv[1], "manual_recovered_idle") == 0) {
    manual_recovered_idle_mode = true;
    return run_component_case(true, false, false, false, false, false, false,
                              true, false, true);
  }
  if (argc == 3 && std::strcmp(argv[1], "manual_recovered_idle") == 0) {
    const char *faults[]{"current_reason", "fault", "source", "not_sent",
                         "reference_drift"};
    for (unsigned i = 0; i < 5; ++i)
      if (std::strcmp(argv[2], faults[i]) == 0) {
        manual_recovered_idle_mode = true;
        manual_recovered_idle_fault = i + 1;
        return run_component_case(true, false, false, false, false, false,
                                  false, true, false, true);
      }
    return 864;
  }
  if (argc == 2 && std::strcmp(argv[1], "historical_idle_baseline") == 0)
    return test_released_idle_startup_baseline();
  if (argc == 3 && std::strcmp(argv[2], "stop_identity_restart") == 0) {
    if (run_component_case(true, false, false) != 0) return 270;
    const auto first = observed_stop_request_id;
    if (run_component_case(true, false, false) != 0) return 271;
    return first != 0U && observed_stop_request_id != first ? 0 : 272;
  }
  if (!test_group_sent_ack()) return 124;
  if (!test_hardware_state_age_heartbeat()) return 147;
  if (!test_heartbeat_budget_parameter()) return 153;
  const bool six_axis = argc >= 2;
  const bool revoked_stop = argc == 3 && std::strcmp(argv[2], "revoked_stop") == 0;
  const bool never_complete_stop = argc == 3 &&
      (std::strcmp(argv[2], "pending_stop") == 0 || revoked_stop);
  const bool heartbeat_budget_mode = argc == 3 && std::strcmp(argv[2], "heartbeat_budget") == 0;
  const bool inactive_prime_mode = argc == 3 && std::strcmp(argv[2], "inactive_prime") == 0;
  const bool cold_idle_gap_mode = argc == 3 && std::strcmp(argv[2], "cold_idle_gap") == 0;
  const bool cold_fault_gap_mode = argc == 3 && std::strcmp(argv[2], "cold_fault_gap") == 0;
  const bool raw_bad_release_mode = argc == 3 && std::strcmp(argv[2], "raw_bad_release") == 0;
  const bool raw_incomplete_mode = argc == 3 && std::strcmp(argv[2], "raw_incomplete") == 0;
  const bool raw_many_mode = argc == 3 && std::strcmp(argv[2], "raw_many") == 0;
  /**
   * @brief CLI raw/cleanup/ACK编号索引；每个模式只执行一个对应分支。
   * @details 71..82是Hold Status选择器，先减12映射59..70；terminal7变体再叠加final/chain/failure字段故障。22及部分间隙无CLI映射，不能当作已执行case。
   * 编号1：后继run2首页先于Status，可落第3页；文件索引要求首页在Status之前。
   * 编号2：run2 Status错boot，已落3页后锁存采集失败。
   * 编号3：run2 Status错session，已落3页后锁存采集失败。
   * 编号4：run2首页改kind2，前run完整仍不能借其资格接纳，页数停2。
   * 编号5：run2首页previous链被破坏，不能落第3页。
   * 编号6：run1只有首组页无终态，run2不得开始，页数停1。
   * 编号7：run2有页/终态但始终缺其Status，stop后必须失败。
   * 编号8：重复run2 seq0，页数停3并采集失败。
   * 编号9：释放中Status清owner后，后续disable/terminal仍归原session；读文件核五阶段索引。
   * 编号10：释放中Status错boot，拒绝并采集失败。
   * 编号11：释放中disable页改kind，不能续上已绑定run2。
   * 编号12：释放中disable页前链错误，页数停3。
   * 编号13：释放终页已落但缺最终带链Status，stop报采集失败。
   * 编号14：最终Status摘要与已落终页不符，页数虽5仍失败。
   * 编号15：新采集见合法历史released Idle，迟到该历史原页不落盘也不误失败。
   * 编号16：历史Idle未release_committed，不能建立无害历史基线。
   * 编号17：历史基线之后换boot使采集失败。
   * 编号18：历史基线之后陌生session Status不得覆盖原session0，不立即误报采集失败。
   * 编号19：历史基线之后变活动Hold/非终态使采集失败。
   * 编号20：取消后独立release首页早到：本fixture要求失败关闭，不能将未获准后继当合法release。
   * 编号21：独立release首页parent错120，失败关闭。
   * 编号23：release run退到120且parent119，旧run拒绝。
   * 编号24：同owner新motion124首页早于Status，完整前run终态允许后继。
   * 编号25：首页已落后后继Status错boot，锁存Identity失败。
   * 编号26：后继Status错session，原run终态水位保留，stop后Incomplete。
   * 编号27：后继motion run不递进，首页拒绝。
   * 编号28：刷新首页category2替代要求的motion1，拒绝。
   * 编号29：刷新首页group_count0，拒绝。
   * 编号30：刷新首页sent_mask31非全六轴，拒绝。
   * 编号31：刷新首页motion_epoch0，拒绝。
   * 编号32：后继终页已落，最终Status链翻位，失败。
   * 编号33：先只算终页摘要而不发送，发布终态Status后再补原页，后继run仍可开始。
   * 编号34：完成后Hold刷新首页先于Status，前run1 flush成立则允许run2。
   * 编号35：前run flush回执boot错误，后继首页拒绝。
   * 编号36：前run flush回执session错误，后继首页拒绝。
   * 编号37：前run缺终页，后继首页拒绝。
   * 编号38：后继首页motion0，拒绝。
   * 编号39：直接跳run3，拒绝。
   * 编号40：后继首页sent_mask31，拒绝。
   * 编号41：前终态没有motion_complete，后继首页拒绝。
   * 编号42：前run flush共享回执清空，不能以统计水位代替强回执。
   * 编号43：连续三个run均靠前run终态flush建立后继，不要求Status立即跟上。
   * 编号44：发布旧run1终态回执，run3不得借旧回执前进。
   * 编号45：run2回执boot错误，run3拒绝。
   * 编号46：run2回执session错误，run3拒绝。
   * 编号47：run2回执标run3，实际后继绑定仍拒绝。
   * 编号48：run2未终态，run3拒绝。
   * 编号49：run3首页到后才来run2错误链终态Status，failure6且总页5。
   * 编号50：同样迟到但run2链精确匹配，可继续run3最终共6页。
   * 编号51：Status陈旧>250ms而Hold State持续新鲜，已有完成flush仍允许后继首页。
   * 编号52：同51但回执boot错，拒绝。
   * 编号53：同51但回执session错，拒绝。
   * 编号54：同51但回执run错，拒绝。
   * 编号55：同51但首页previous链错，Chain失败。
   * 编号56：Status和State都陈旧，不能以完成flush放宽当前反馈年龄。
   * 编号57：State改非Hold6且不ready，后继拒绝。
   * 编号58：回执chain_valid=false，后继拒绝。
   * 编号59：取消final7已flush，安全Hold页epoch10且当前Hold State新鲜，即使Status陈旧也可接后继。
   * 编号60：取消flush回执boot错，后继拒绝。
   * 编号61：取消flush回执session错，后继拒绝。
   * 编号62：取消flush回执run122不符123，后继拒绝。
   * 编号63：取消flush回执chain_valid=false，后继拒绝。
   * 编号64：取消后State不再保鲜，须已有stale/heartbeat停机；属于组合拒绝。
   * 编号65：取消后State非Hold6（特选变体为Stopping7），后继拒绝。
   * 编号66：Hold但hold_ready=false，后继拒绝。
   * 编号67：State management_epoch1与所需0不符，后继拒绝。
   * 编号68：取消终态回执safety_hold_valid=false，拒绝。
   * 编号69：安全hold epoch被改11，拒绝。
   * 编号70：后继首页motion_epoch11与安全hold10不匹配，拒绝。
   * 编号83：Run锚已建立后public Stop失效，当前同boot新Fault可供raw cleanup，普通控制State仍不可用。
   * 编号84：仅Hold未有Run锚，Fault不能填raw cleanup。
   * 编号85：Fault换boot17，不能填raw cleanup。
   * 编号86：Fault复用旧produced_at_us，不能填raw cleanup。
   * 编号87：精确State ACK先到，再Status Hold，记录该generation quiescent排空。
   * 编号88：State无committed ACK，后Status Hold不得造ACK。
   * 编号89：State部分SENT且committed0，后Status Hold不得造ACK。
   * 编号90：ACK State错motion，不能造排空。
   * 编号91：ACK State处Run5，不能当Hold排空。
   * 编号92：ACK State错session，拒绝。
   * 编号93：ACK State Source旧2s，拒绝。
   * 编号94：ACK State错reference，拒绝。
   * 编号95：ACK State错boot，拒绝。
   * 编号96：Status Hold前public Stop，已失效上下文不能重建quiescent。
   */
  const unsigned raw_order_case = argc == 3 && std::strcmp(argv[2],"raw_complete_status_before_flush")==0 ? (complete_status_before_flush_case=true,43U) : argc == 3 && std::strcmp(argv[2],"hold_state_ack_before_status")==0 ?87U : argc == 3 && std::strcmp(argv[2],"hold_no_ACK")==0 ?88U : argc == 3 && std::strcmp(argv[2],"hold_partial_SENT")==0 ?89U : argc == 3 && std::strcmp(argv[2],"hold_wrong_epoch")==0 ?90U : argc == 3 && std::strcmp(argv[2],"hold_Run_ACK")==0 ?91U : argc == 3 && std::strcmp(argv[2],"hold_wrong_session")==0 ?92U : argc == 3 && std::strcmp(argv[2],"hold_stale_ACK")==0 ?93U : argc == 3 && std::strcmp(argv[2],"hold_wrong_reference")==0 ?94U : argc == 3 && std::strcmp(argv[2],"hold_wrong_boot")==0 ?95U : argc == 3 && std::strcmp(argv[2],"hold_Stop_before_status")==0 ?96U : argc == 3 && std::strcmp(argv[2],"raw_fault_no_RUN_anchor")==0 ?84U :
      argc == 3 && std::strcmp(argv[2],"raw_fault_wrong_boot")==0 ?85U :
      argc == 3 && std::strcmp(argv[2],"raw_fault_stale_produced")==0 ?86U : argc == 3 && std::strcmp(argv[2],"raw_fault_after_public_invalidate")==0 ? 83U : cancel_hold_status_variant ? cancel_hold_case-12U : argc == 3 && std::strcmp(argv[2], "raw_order") == 0 ? 1U :
      argc == 3 && std::strcmp(argv[2], "raw_order_bad_boot") == 0 ? 2U :
      argc == 3 && std::strcmp(argv[2], "raw_order_bad_session") == 0 ? 3U :
      argc == 3 && std::strcmp(argv[2], "raw_order_bad_kind") == 0 ? 4U :
      argc == 3 && std::strcmp(argv[2], "raw_order_bad_chain") == 0 ? 5U :
      argc == 3 && std::strcmp(argv[2], "raw_order_unverified_terminal") == 0 ? 6U :
      argc == 3 && std::strcmp(argv[2], "raw_order_missing_status") == 0 ? 7U :
      argc == 3 && std::strcmp(argv[2], "raw_order_duplicate_seq") == 0 ? 8U :
      argc == 3 && std::strcmp(argv[2], "raw_order_release") == 0 ? 9U :
      argc == 3 && std::strcmp(argv[2], "raw_order_release_bad_boot") == 0 ? 10U :
      argc == 3 && std::strcmp(argv[2], "raw_order_release_bad_kind") == 0 ? 11U :
      argc == 3 && std::strcmp(argv[2], "raw_order_release_bad_chain") == 0 ? 12U :
      argc == 3 && std::strcmp(argv[2], "raw_order_release_missing_final") == 0 ? 13U :
      argc == 3 && std::strcmp(argv[2], "raw_order_release_wrong_final_chain") == 0 ? 14U :
      argc == 3 && std::strcmp(argv[2], "raw_historical_idle") == 0 ? 15U :
      argc == 3 && std::strcmp(argv[2], "raw_historical_idle_unreleased") == 0 ? 16U :
      argc == 3 && std::strcmp(argv[2], "raw_historical_idle_wrong_boot") == 0 ? 17U :
      argc == 3 && std::strcmp(argv[2], "raw_historical_idle_wrong_session") == 0 ? 18U :
      argc == 3 && std::strcmp(argv[2], "raw_historical_idle_active") == 0 ? 19U :
      argc == 3 && std::strcmp(argv[2], "raw_order_release_page_first") == 0 ? 20U :
      argc == 3 && std::strcmp(argv[2], "raw_order_release_page_first_bad_parent") == 0 ? 21U :
      argc == 3 && std::strcmp(argv[2], "raw_order_release_page_first_old_run") == 0 ? 23U :
      argc == 3 && std::strcmp(argv[2], "raw_order_refresh_page_first") == 0 ? 24U :
      argc == 3 && std::strcmp(argv[2], "raw_order_refresh_page_first_bad_boot") == 0 ? 25U :
      argc == 3 && std::strcmp(argv[2], "raw_order_refresh_page_first_bad_session") == 0 ? 26U :
      argc == 3 && std::strcmp(argv[2], "raw_order_refresh_page_first_old_run") == 0 ? 27U :
      argc == 3 && std::strcmp(argv[2], "raw_order_refresh_page_first_bad_category") == 0 ? 28U :
      argc == 3 && std::strcmp(argv[2], "raw_order_refresh_page_first_bad_group_count") == 0 ? 29U :
      argc == 3 && std::strcmp(argv[2], "raw_order_refresh_page_first_bad_mask") == 0 ? 30U :
      argc == 3 && std::strcmp(argv[2], "raw_order_refresh_page_first_zero_motion_epoch") == 0 ? 31U :
      argc == 3 && std::strcmp(argv[2], "raw_order_refresh_page_first_wrong_final_chain") == 0 ? 32U :
      argc == 3 && std::strcmp(argv[2], "raw_order_refresh_final_status_before_terminal") == 0 ? 33U :
      argc == 3 && std::strcmp(argv[2], "raw_order_refresh_hold_page_first") == 0 ? 34U :
      argc == 3 && std::strcmp(argv[2], "raw_order_refresh_hold_bad_boot") == 0 ? 35U :
      argc == 3 && std::strcmp(argv[2], "raw_order_refresh_hold_bad_session") == 0 ? 36U :
      argc == 3 && std::strcmp(argv[2], "raw_order_refresh_hold_no_terminal") == 0 ? 37U :
      argc == 3 && std::strcmp(argv[2], "raw_order_refresh_hold_bad_motion") == 0 ? 38U :
      argc == 3 && std::strcmp(argv[2], "raw_order_refresh_hold_bad_run") == 0 ? 39U :
      argc == 3 && std::strcmp(argv[2], "raw_order_refresh_hold_bad_mask") == 0 ? 40U :
      argc == 3 && std::strcmp(argv[2], "raw_order_refresh_hold_no_motion_complete") == 0 ? 41U :
      argc == 3 && std::strcmp(argv[2], "raw_order_refresh_hold_no_receipt") == 0 ? 42U :
      argc == 3 && std::strcmp(argv[2], "raw_order_multi_successor") == 0 ? 43U :
      argc == 3 && std::strcmp(argv[2], "raw_order_multi_successor_stale_terminal") == 0 ? 44U :
      argc == 3 && std::strcmp(argv[2], "raw_order_multi_successor_bad_boot") == 0 ? 45U :
      argc == 3 && std::strcmp(argv[2], "raw_order_multi_successor_bad_session") == 0 ? 46U :
      argc == 3 && std::strcmp(argv[2], "raw_order_multi_successor_bad_run") == 0 ? 47U :
      argc == 3 && std::strcmp(argv[2], "raw_order_multi_successor_no_terminal") == 0 ? 48U :
      argc == 3 && std::strcmp(argv[2], "raw_order_multi_successor_late_wrong_chain") == 0 ? 49U :
      argc == 3 && std::strcmp(argv[2], "raw_order_multi_successor_late_correct_chain") == 0 ? 50U :
      argc == 3 && std::strcmp(argv[2], "raw_order_refresh_hold_stale_status_fresh_state") == 0 ? 51U :
      argc == 3 && std::strcmp(argv[2], "raw_order_refresh_hold_stale_status_bad_boot") == 0 ? 52U :
      argc == 3 && std::strcmp(argv[2], "raw_order_refresh_hold_stale_status_bad_session") == 0 ? 53U :
      argc == 3 && std::strcmp(argv[2], "raw_order_refresh_hold_stale_status_bad_run") == 0 ? 54U :
      argc == 3 && std::strcmp(argv[2], "raw_order_refresh_hold_stale_status_bad_chain") == 0 ? 55U :
      argc == 3 && std::strcmp(argv[2], "raw_order_refresh_hold_stale_state") == 0 ? 56U :
      argc == 3 && std::strcmp(argv[2], "raw_order_refresh_hold_stale_status_non_hold_state") == 0 ? 57U :
      argc == 3 && std::strcmp(argv[2], "raw_order_refresh_hold_stale_status_bad_receipt_chain") == 0 ? 58U :
      argc == 3 && std::strcmp(argv[2], "raw_order_cancel_stale_status_fresh_state") == 0 ? 59U :
      argc == 3 && std::strcmp(argv[2], "raw_order_cancel_stale_status_bad_receipt_boot") == 0 ? 60U :
      argc == 3 && std::strcmp(argv[2], "raw_order_cancel_stale_status_bad_receipt_session") == 0 ? 61U :
      argc == 3 && std::strcmp(argv[2], "raw_order_cancel_stale_status_bad_receipt_run") == 0 ? 62U :
      argc == 3 && std::strcmp(argv[2], "raw_order_cancel_stale_status_bad_receipt_chain") == 0 ? 63U :
      argc == 3 && std::strcmp(argv[2], "raw_order_cancel_stale_status_stale_state") == 0 ? 64U :
      argc == 3 && std::strcmp(argv[2], "raw_order_cancel_stale_status_non_hold_state") == 0 ? 65U :
      argc == 3 && std::strcmp(argv[2], "raw_order_cancel_stale_status_not_ready_state") == 0 ? 66U :
      argc == 3 && std::strcmp(argv[2], "raw_order_cancel_stale_status_management_mismatch") == 0 ? 67U :
      argc == 3 && std::strcmp(argv[2], "raw_order_cancel_stale_status_no_safety_hold") == 0 ? 68U :
      argc == 3 && std::strcmp(argv[2], "raw_order_cancel_stale_status_bad_safety_motion") == 0 ? 69U :
      argc == 3 && std::strcmp(argv[2], "raw_order_cancel_stale_status_bad_successor_motion") == 0 ? 70U :
      0U;
  const bool raw_capture_mode = raw_bad_release_mode || raw_incomplete_mode || raw_many_mode || raw_order_case != 0U ||
      (argc == 3 && std::strcmp(argv[2], "raw_release_receipt") == 0) ||
      (argc == 3 && std::strcmp(argv[2], "raw_direct_receipt") == 0) ||
      (argc == 3 && std::strcmp(argv[2], "raw_capture") == 0);
  /**
   * @brief Complete编号索引；16没有当前CLI映射。
   * 编号1：Status先、State后、Query终态最后，允许精确下一motion。
   * 编号2：State先到不靠它建立新epoch，Status到后再给新seq并Query。
   * 编号3：Query终态先，再State与Status，保持身份不误失效。
   * 编号4：未许可Status跳motion12，必须增加invalidations。
   * 编号5：只有State提前而没有Status授权，随后State间隔超时Stop。
   * 编号6：完成后同值保持目标不重复发、不生ACK超时。
   * 编号7：目标偏0.02rad超容差，应Stop且不发新命令。
   * 编号8：Complete仍Pending时同值保持目标抑制。
   * 编号9：下一epoch Status为Fault9，必须失效。
   * 编号10：Hold先于Query，抑制同值目标，之后再Query可收尾。
   * 编号11：终末目标已ACK后发送Complete并冻结旧epoch后续三代命令。
   * 编号12：终末目标未ACK前Complete不应发，补ACK后才发并冻结。
   * 编号13：0.0008rad小差保持目标在容差内抑制。
   * 编号14：Pending持续约2.2s并持续供新State，不能把小差目标误当新运动。
   * 编号15：Complete预期management错，本地failed且不发operation8。
   * 编号17：下一motion Enabling先发一个prime，第二个小变目标不应在首ACK后偷偷多发。
   * 编号18：正确run/非零链/final6建立导出资格，可发一次刷新并ACK。
   * 编号19：无非零终链，不建立资格且不刷新。
   * 编号20：ACK后生成hold_drain，冷却同值抑制，后续可周期刷新；被拒generation不得推进水位。
   * 编号21：终态Status错run，不建立刷新资格。
   * 编号22：evidence_lost使刷新资格关闭。
   * 编号23：原本可刷新后又lost，必须撤销资格并抑制目标。
   * 编号24：原本可刷新后final回退0/链清零，必须撤销资格。
   * 编号25：原本可刷新后改attempt_kind2/parent1，必须撤销资格。
   * 编号26：已排刷新目标被clear身份拒绝；新State和新身份后新generation可恢复。
   * 编号27：刷新发出但始终无ACK，必须组ACK超时。
   * 编号28：终末producer0的重试Complete，Status先路径仍可正确处理。
   * 编号29：同28但下一Status错motion，必须失效。
   * 编号30：本地Complete句柄canceled后新鲜Hold State仍可抑制同值prime，不误当完整事务终态。
   * 编号31：同30但Hold State错boot，保留原seq3且不发布。
   * 编号32：同30但错motion，保留旧快照且不发布。
   * 编号33：同30但目标偏0.02rad，增加rejected_commands，不发布。
   * 编号34：同30但Hold Source旧2s，拒绝并保留seq3。
   * 编号35：Status先给Hold+精确ACK+全SENT，同值prime抑制且不计拒绝/Stop。
   * 编号36：同35但目标偏0.02rad，拒绝不发布。
   * 编号37：Status Hold无SENT，不能充当完整组确认，拒绝prime。
   * 编号38：Status错motion导致授权失效，本地transfer拒绝也可满足负例。
   * 编号39：Hold Status不更新、State续鲜2.1s超过pending期限，prime拒绝。
   * 编号40：Status committed_seq错为first+1，不得当本组ACK。
   * 编号41：Status sent_mask缺J1，不得当全组SENT。
   */
  const unsigned complete_order_mode = argc == 3 && std::strcmp(argv[2],"complete_hold_status_before_state_wrong_ack_seq")==0 ? 40U :
      argc == 3 && std::strcmp(argv[2],"complete_hold_status_before_state_partial_sent")==0 ? 41U :
      argc == 3 && std::strcmp(argv[2],"complete_hold_status_before_state_unconfirmed_sent")==0 ? 37U :
      argc == 3 && std::strcmp(argv[2],"complete_hold_status_before_state_wrong_epoch")==0 ? 38U :
      argc == 3 && std::strcmp(argv[2],"complete_hold_status_before_state_expired_pending")==0 ? 39U :
      argc == 3 && std::strcmp(argv[2],"complete_hold_status_before_state_same_prime")==0 ? 35U :
      argc == 3 && std::strcmp(argv[2],"complete_hold_status_before_state_divergent_prime")==0 ? 36U :
      argc == 3 && std::strcmp(argv[2],"complete_retired_pending_wrong_hold_boot")==0 ? 31U :
      argc == 3 && std::strcmp(argv[2],"complete_retired_pending_wrong_hold_epoch")==0 ? 32U :
      argc == 3 && std::strcmp(argv[2],"complete_retired_pending_divergent_prime")==0 ? 33U :
      argc == 3 && std::strcmp(argv[2],"complete_retired_pending_stale_hold_payload")==0 ? 34U :
      argc == 3 && std::strcmp(argv[2],"complete_same_prime_fresh_state_pending_cleanup")==0 ? 30U :
      argc == 3 && std::strcmp(argv[2],"complete_retry_status_first")==0 ? 28U :
      argc == 3 && std::strcmp(argv[2],"complete_retry_wrong_status_epoch")==0 ? 29U :
      argc == 3 && std::strcmp(argv[2],"complete_status_first")==0 ? 1U :
      argc == 3 && std::strcmp(argv[2],"complete_state_first")==0 ? 2U :
      argc == 3 && std::strcmp(argv[2],"complete_response_first")==0 ? 3U :
      argc == 3 && std::strcmp(argv[2],"complete_unsolicited_jump")==0 ? 4U :
      argc == 3 && std::strcmp(argv[2],"complete_state_timeout")==0 ? 5U :
      argc == 3 && std::strcmp(argv[2],"complete_hold_ack")==0 ? 6U :
      argc == 3 && std::strcmp(argv[2],"complete_hold_ack_mismatch")==0 ? 7U :
      argc == 3 && std::strcmp(argv[2],"complete_pending_hold_suppressed")==0 ? 8U :
      argc == 3 && std::strcmp(argv[2],"complete_fault_status")==0 ? 9U :
      argc == 3 && std::strcmp(argv[2],"complete_hold_before_response")==0 ? 10U :
      argc == 3 && std::strcmp(argv[2],"complete_freeze_old_epoch")==0 ? 11U :
      argc == 3 && std::strcmp(argv[2],"complete_wait_terminal_ack")==0 ? 12U :
      argc == 3 && std::strcmp(argv[2],"complete_hold_small_delta")==0 ? 13U :
      argc == 3 && std::strcmp(argv[2],"complete_hold_expired_pending")==0 ? 14U :
      argc == 3 && std::strcmp(argv[2],"complete_invalid_binding")==0 ? 15U :
      argc == 3 && std::strcmp(argv[2],"complete_one_prime")==0 ? 17U :
      argc == 3 && std::strcmp(argv[2],"complete_export_ready_refresh")==0 ? 18U :
      argc == 3 && std::strcmp(argv[2],"complete_no_export_refresh")==0 ? 19U :
      argc == 3 && std::strcmp(argv[2],"complete_periodic_refresh")==0 ? 20U :
      argc == 3 && std::strcmp(argv[2],"complete_wrong_run_chain")==0 ? 21U :
      argc == 3 && std::strcmp(argv[2],"complete_lost_chain")==0 ? 22U :
      argc == 3 && std::strcmp(argv[2],"complete_export_then_lost")==0 ? 23U :
      argc == 3 && std::strcmp(argv[2],"complete_export_then_regressed")==0 ? 24U :
      argc == 3 && std::strcmp(argv[2],"complete_export_then_wrong_kind")==0 ? 25U :
      argc == 3 && std::strcmp(argv[2],"complete_refresh_stale_authority")==0 ? 26U :
      argc == 3 && std::strcmp(argv[2],"complete_export_refresh_ack_unconfirmed")==0 ? 27U : 0U;
  const bool default_ack_age_mode = argc == 3 && std::strcmp(argv[2], "default_ack_age") == 0;
  /**
   * @brief 显式释放/生命周期编号索引，字段Complete指RPC收包不等于事务Committed。
   * 编号1：Pending release80→Query81 Committed→Idle，deactivate不得再发Stop。
   * 编号2：虽有回执但Idle Status未release_committed，仍需Stop。
   * 编号3：已Released Idle后on_error仍走错误停机路径。
   * 编号4：运行中deactivate触发Stop；source协调模式在此切入各源专属Cancel场景。
   * 编号5：没有gate强回执，Idle样式不能使deactivate省略Stop。
   * 编号6：强回执后Idle State先于旧owner Status，持续续鲜不误Stop。
   * 编号7：release80直接Committed无Query，raw管理记录只两条。
   * 编号8：合法Released Idle无新State超过250ms，不应按活动State断流重复Stop。
   * 编号9：Disabling8+无许可可临时采用Idle State，Stopping7/有permit/错tuple不行。
   * 编号10：始终缺最终Idle必须State间隔停机，不能无限释放豁免。
   * 编号11：Stopping时早Idle暂存，Query强Committed后复用原接收时间及同op候选。
   * 编号12：Query强回执op43与早候选op42不匹配，先不采用；随后正确op43新Idle可接纳。
   * 编号13：Query回执motion错误，不采用早Idle。
   * 编号14：Query回执management错误，不采用早Idle。
   * 编号15：Query仍Pending，不采用早Idle。
   * 编号16：Query未release_committed，不采用早Idle。
   * 编号17：早Idle Source已陈旧，后Query强回执不能令其返鲜。
   * 编号18：已Committed后Idle State持续新鲜而旧Stopping Status落后500ms，不误心跳Stop。
   */
  const unsigned release_deactivate_mode = argc == 3 && std::strcmp(argv[2],"release_committed_idle_status_gap")==0 ? 18U :
      argc == 3 && std::strcmp(argv[2],"release_stopping_combined_stale_candidate")==0 ? 17U :
      argc == 3 && std::strcmp(argv[2],"release_stopping_wrong_receipt_op")==0 ? 12U :
      argc == 3 && std::strcmp(argv[2],"release_stopping_wrong_receipt_epoch")==0 ? 13U :
      argc == 3 && std::strcmp(argv[2],"release_stopping_wrong_receipt_management")==0 ? 14U :
      argc == 3 && std::strcmp(argv[2],"release_stopping_pending_receipt")==0 ? 15U :
      argc == 3 && std::strcmp(argv[2],"release_stopping_uncommitted_receipt")==0 ? 16U :
      argc == 3 && std::strcmp(argv[2],"release_stopping_committed_idle_proof")==0 ? 11U :
      argc == 3 &&
      (std::strcmp(argv[2], "released_idle_deactivate") == 0 ||
       std::strcmp(argv[2], "raw_release_receipt") == 0) ? 1U :
      argc == 3 && std::strcmp(argv[2], "raw_direct_receipt") == 0 ? 7U :
      argc == 3 && std::strcmp(argv[2], "released_idle_unconfirmed") == 0 ? 2U :
      argc == 3 && std::strcmp(argv[2], "released_idle_on_error") == 0 ? 3U :
      argc == 3 && std::strcmp(argv[2], "running_deactivate") == 0 ? 4U :
      argc == 3 && std::strcmp(argv[2], "released_idle_no_gate_reply") == 0 ? 5U :
      argc == 3 && std::strcmp(argv[2], "released_state_before_status") == 0 ? 6U :
      argc == 3 && std::strcmp(argv[2], "released_idle_heartbeat_gap") == 0 ? 8U :
      argc == 3 && std::strcmp(argv[2], "released_idle_before_reply") == 0 ? 9U :
      argc == 3 && std::strcmp(argv[2], "released_no_final_idle") == 0 ? 10U : 0U;
  release_receipt_fault=release_deactivate_mode>=12U && release_deactivate_mode<=16U
      ? release_deactivate_mode-11U : 0U;
  /**
   * @brief 票据编号只控制第二代目标何时可发：1新票据续发；2票据不变不发；3session变化使旧owner目标失效。
   * 首目标ACK和票据新鲜性各自独立，不能只因有ACK就复用旧票据发pending。
   */
  const unsigned ticket_mode = argc == 3 && std::strcmp(argv[2], "ticket_refresh") == 0 ? 1U :
      argc == 3 && std::strcmp(argv[2], "ticket_stale") == 0 ? 2U :
      argc == 3 && std::strcmp(argv[2], "ticket_authority") == 0 ? 3U : 0U;
  const bool reject_during_ack_mode = argc == 3 &&
      std::strcmp(argv[2], "reject_during_ack") == 0;
  const bool sequence_rebase_mode = argc == 3 &&
      std::strcmp(argv[2], "stop_sequence_rebase") == 0;
  const bool uncommitted_stop_mode = argc == 3 &&
      std::strcmp(argv[2], "stop_uncommitted_terminal") == 0;
  const bool uncommitted_idle_status_mode = argc == 3 &&
      std::strcmp(argv[2], "stop_uncommitted_idle_status") == 0;
  // 只在原完整six_axis场景核原Worker诊断；不把仅sent的running_deactivate冒充confirmed。
  const bool verify_stop_policy_diag = six_axis && argc == 2;
  const auto result = run_component_case(six_axis,
      argc == 3 && ticket_mode == 0U && !sequence_rebase_mode && !uncommitted_stop_mode && !uncommitted_idle_status_mode && !default_ack_age_mode && !never_complete_stop && !heartbeat_budget_mode &&
          !inactive_prime_mode && !cold_idle_gap_mode && !cold_fault_gap_mode && !raw_capture_mode &&
          release_deactivate_mode == 0U && !reject_during_ack_mode &&
          complete_order_mode==0U,
      default_ack_age_mode, never_complete_stop, heartbeat_budget_mode || complete_order_mode!=0U, revoked_stop,
      inactive_prime_mode, cold_idle_gap_mode, cold_fault_gap_mode,
      raw_capture_mode, raw_bad_release_mode, raw_incomplete_mode, raw_many_mode, raw_order_case,
      complete_order_mode, release_deactivate_mode, ticket_mode, reject_during_ack_mode,
      sequence_rebase_mode, uncommitted_stop_mode, uncommitted_idle_status_mode);
  /**
   * @brief 仅完整普通六轴场景核结构化Stop策略快照：唯一事件、原1s期限、confirmed在预算内及精确身份。
   * 不会把running_deactivate只sent的场景冒充确认；其它提前return子场景没有执行此组断言。
   */
  if (verify_stop_policy_diag) {
    if(result!=0)return result;
    const auto& snapshot=final_worker_diagnostic_snapshot;
    const auto& record=snapshot.stop_policy;
    if(!snapshot.stop_policy_present || snapshot.stop_policy_count!=1U || !record.start_known ||
       record.started_us==0U || record.confirmed_us<record.started_us ||
       record.deadline_us-record.started_us!=record.timeout_us || record.timeout_us!=1000000U ||
       record.confirmed_us-record.started_us>record.timeout_us || record.request_id==0U ||
       record.operation_id!=exec_expected_stop_operation || !record.identity.valid ||
       record.identity.boot!=7U || record.identity.session!=18U || record.identity.reference==0U ||
       record.identity.motion==0U || record.identity.management==0U || record.profile!=1U || record.mask!=63U)
      return 702;
  }
  if (result != 0 || !six_axis || argc != 2) return result;
  return run_component_case(true, false, true);
}

/**
 * @brief 将完整断言编号转换为可靠的进程退出状态，并输出唯一终态标记。
 * @details POSIX 只保留退出码低八位，直接返回 256 会被误判成功。
 * 原始断言编号仍完整输出；只有所有所选场景通过才返回零。
 * @param argc 场景参数数量，原样交给场景分发器。
 * @param argv 场景参数数组，分发器按原约定处理。
 * @return 0 表示全部通过，1 表示任意断言失败。
 */
int main(int argc, char **argv) {
  ::signal(SIGPIPE,SIG_IGN); // peer提前失败须报告场景断言，不能让测试因管道信号静默终止。
  const int result = run_transport_test_cases(argc, argv);
  std::fprintf(stderr, "TRANSPORT_COMPONENT_RESULT=%s assertion=%d\n",
               result == 0 ? "PASS" : "FAIL", result);
  return result == 0 ? 0 : 1;
}

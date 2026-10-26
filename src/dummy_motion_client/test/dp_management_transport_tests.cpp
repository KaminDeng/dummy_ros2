/**
 * @file dp_management_transport_tests.cpp
 * @brief dp_comm管理传输、观测连续性、释放过渡及raw首页快照测试。
 *
 * 部分模式是纯内存，默认serial模式会fork SIL peer并启动PTY转发线程，tcp模式会创建本机TCP端点/子进程；它们不是实物USB/CAN，但仍有实际OS与通信副作用。
 * raw_first_page只核内存快照一致性，不能推导真实raw文件落盘/设备终态。所有超时为Host单调时间；人工观测年龄参数单位µs。
 */
#include "dp_comm/dp_comm.hpp"
#include "dummy_motion_client/dp_management_transport.hpp"
#include "dummy_ref/wire_contract.hpp"
#include "dummy_ros_control/device_management_channel.hpp"
#include "dummy_ros_control/device_transport.hpp"
#include "dummy_ros_control/guarded_zenbedded_system.hpp"
#include <atomic>
#include <cerrno>
#include <chrono>
#include <fcntl.h>
#include <iostream>
#include <memory>
#include <poll.h>
#include <pty.h>
#include <signal.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
using namespace dummy_motion_client;
namespace wire = dp::iface;
using Service = dp::protocol::services::DummyManageV2;

namespace {
using Clock = std::chrono::steady_clock;
std::atomic<unsigned> reply_calls{0};
unsigned evidence_test_mode = 0;

/**
 * @brief 向子进程同步管道写单字节标记。
 * @param fd 可写fd，所有权不转移。
 * @param value 协议标记。
 * @return 恰好写1字节；不重试短写/EINTR。
 */
bool signal_peer(int fd, char value) { return write(fd, &value, 1) == 1; }

/**
 * @brief 在有限poll等待后读一字节并比较。
 * @param fd 可读fd。
 * @param expected 期望标记。
 * @param timeout_ms 最大poll等待毫秒。
 * @return 可读且读值匹配，否则false。
 */
bool read_peer_signal(int fd, char expected, int timeout_ms) {
  pollfd ready{fd, POLLIN, 0};
  if (poll(&ready, 1, timeout_ms) != 1 || !(ready.revents & POLLIN)) return false;
  char actual = 0;
  return read(fd, &actual, 1) == 1 && actual == expected;
}

/**
 * @brief 拥有fork子进程的RAII守卫；禁拷贝以免重复终止。
 */
class PeerProcess {
 public:
  /**
   * @brief 接管子进程回收责任。
   * @param pid 父进程保存的子PID，不负责创建。
   */
  explicit PeerProcess(pid_t pid) : pid_(pid) {}
  /**
   * @brief 析构对有效PID发SIGTERM并waitpid回收，EINTR重试。
   * @return 无；是OS进程清理，不是设备安全停止。
   */
  ~PeerProcess() {
    if (pid_ <= 0) return;
    kill(pid_, SIGTERM);
    while (waitpid(pid_, nullptr, 0) < 0 && errno == EINTR) {}
  }
  /**
   * @brief 禁止复制子进程拥有者，避免两个析构对象同时kill/wait同一PID。
   * @par 未命名参数
   * 拷贝来源不会被接受；此声明为delete，没有可执行实现。
   */
  PeerProcess(const PeerProcess &) = delete;
  /**
   * @brief 禁止复制赋值，子进程回收责任不能被隐式转移或共享。
   * @par 未命名参数
   * 来源对象不会被接受；delete声明不产生实际返回。
   */
  PeerProcess &operator=(const PeerProcess &) = delete;

 private:
  pid_t pid_;
};

/**
 * @brief 拥有同步管道读fd，作用域退出关闭。
 */
struct SignalReader {
  int fd;
  /**
   * @brief 关闭fd，不终止peer。
   * @return 无。
   */
  ~SignalReader() { close(fd); }
};

/**
 * @brief 两对PTY的双向字节桥；endpoint串口115200，master由本对象独占。
 */
class PtyBridge {
 public:
  /**
   * @brief 创建两对PTY、设置master非阻塞并组成serial端点。
   * @return 是否全部成功；错误由调用方返回失败。
   */
  bool open_pair() {
    char left_name[128]{};
    char right_name[128]{};
    if (openpty(&left_master_, &left_slave_, left_name, nullptr, nullptr) != 0 ||
        openpty(&right_master_, &right_slave_, right_name, nullptr, nullptr) != 0) return false;
    if (fcntl(left_master_, F_SETFL, O_NONBLOCK) < 0 ||
        fcntl(right_master_, F_SETFL, O_NONBLOCK) < 0) return false;
    close(left_slave_); left_slave_ = -1;
    close(right_slave_); right_slave_ = -1;
    left_endpoint_ = std::string("serial/") + left_name + "#baudrate=115200";
    right_endpoint_ = std::string("serial/") + right_name + "#baudrate=115200";
    return true;
  }

  /**
   * @brief fork子进程关闭继承的bridge master副本。
   * @return 无；不改变父进程fd表。
   */
  void close_child_fds() const {
    close(left_master_); close(right_master_);
  }

  /**
   * @brief 启动poll20ms的转发线程，暂未打开slave时遇HUP/ERR等待重试。
   * @return 无；线程捕获this，对象须活到join。
   */
  void start() {
    running_ = true;
    worker_ = std::thread([this] {
      while (running_) {
        pollfd fds[]{{left_master_, POLLIN, 0}, {right_master_, POLLIN, 0}};
        if (poll(fds, 2, 20) <= 0) continue;
        if (!(fds[0].revents & POLLIN) && !(fds[1].revents & POLLIN) &&
            ((fds[0].revents | fds[1].revents) & (POLLHUP | POLLERR))) {
          std::this_thread::sleep_for(std::chrono::milliseconds(20));
          continue;
        }
        if (fds[0].revents & POLLIN) forward(left_master_, right_master_);
        if (fds[1].revents & POLLIN) forward(right_master_, left_master_);
      }
    });
  }

  /**
   * @brief 撤running并join，再关闭masters置−1。
   * @return 无；可重复调用。
   */
  void stop() {
    running_ = false;
    if (worker_.joinable()) worker_.join();
    if (left_master_ >= 0) { close(left_master_); left_master_ = -1; }
    if (right_master_ >= 0) { close(right_master_); right_master_ = -1; }
  }
  /**
   * @brief 先stop再关闭尚存slave fd。
   * @return 无。
   */
  ~PtyBridge() {
    stop();
    if (left_slave_ >= 0) close(left_slave_);
    if (right_slave_ >= 0) close(right_slave_);
  }

  /**
   * @brief 借用左侧PTY serial端点字符串。
   * @return const引用，生存期不超过PtyBridge。
   */
  const std::string &left_endpoint() const { return left_endpoint_; }
  /**
   * @brief 借用右侧PTY serial端点字符串。
   * @return const引用，生存期不超过PtyBridge。
   */
  const std::string &right_endpoint() const { return right_endpoint_; }

 private:
  /**
   * @brief 搬运最多512字节，处理短写/EINTR/EAGAIN。
   * @param source 来源master fd。
   * @param target 目的master fd。
   * @return 无；受running标志控制，非可靠消息重传器。
   */
  void forward(int source, int target) {
    std::uint8_t bytes[512];
    const ssize_t size = read(source, bytes, sizeof(bytes));
    if (size <= 0) return;
    for (ssize_t offset = 0; running_ && offset < size;) {
      const ssize_t written = write(target, bytes + offset, static_cast<size_t>(size - offset));
      if (written > 0) offset += written;
      else if (written < 0 && errno == EINTR) continue;
      else if (written < 0 && errno == EAGAIN) {
        pollfd writable{target, POLLOUT, 0};
        poll(&writable, 1, 20);
      } else return;
    }
  }

  /**
   * @brief master/slave为本对象持有fd，−1表示已关闭；worker借用本对象且stop必须join后才释放master。
   */
  int left_master_ = -1, right_master_ = -1;
  int left_slave_ = -1, right_slave_ = -1;
  std::string left_endpoint_, right_endpoint_;
  /**
   * @brief 主线程与转发线程间停止标志；join保证转发结束，不提供业务消息完成确认。
   */
  std::atomic<bool> running_{false};
  std::thread worker_;
};

/**
 * @brief 析构顺序守卫：在transport析构前停PTY桥，以解除底层serial读取阻塞。
 * 只借用bridge，必须在transport后构造以反序析构。
 */
struct StopBridgeBeforeTransport {
  /**
   * @brief 借用桥，析构守卫只负责提前停止，不删除桥。
   */
  PtyBridge &bridge;
  /**
   * @brief 先停止桥，不接管bridge对象生存期。
   * @return 无。
   */
  ~StopBridgeBeforeTransport() { bridge.stop(); }
};

/**
 * @brief 以当前schema编码最小管理请求。
 * @param request_id 64位ID，默认值特意大于32位。
 * @return 固定wire字节，codec失败返回空；未代表有效设备管理授权。
 */
std::vector<std::uint8_t> request_bytes(std::uint64_t request_id = 0x20000000000001ULL) {
  ManageRequest request;
  request.set_schema_major(2); request.set_schema_minor(dummy_ref::protocol::kSchemaMinor);
  request.set_request_id(request_id);
  std::vector<std::uint8_t> bytes(wire::DummyManageV2RequestCodec::kEncodedSize);
  if (wire::DummyManageV2RequestCodec::encode(request, bytes.data(), bytes.size()) != wire::CodecResult::kOk)
    return {};
  return bytes;
}

/**
 * @brief 每2ms检查standalone传输ready，最多3s。
 * @param transport 已启动管理传输。
 * @return 期限内ready；不表示设备身份/permit已建立。
 */
bool wait_ready(DpManagementTransport &transport) {
  const auto deadline = Clock::now() + std::chrono::seconds(3);
  while (!transport.ready() && Clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  return transport.ready();
}

/**
 * @brief 用人工时间覆盖短缺流Wait后恢复、长期缺流Lost及活跃运动时Status节流例外；冷启动仍要求Status。还核State authority只允许当前/特定下一motion而不能任意跨代。
 * @return 0所述检查通过，非零是具体失败检查码。
 */
int observation_continuity_tests() {
  DeviceObservation observation;
  observation.state_received_us = 1000000U;
  observation.status_received_us = 1000000U;
  ObservationContinuity hold_gap(1000000U, 2000000U, 500000U);
  if (hold_gap.evaluate(observation, 1000000U, true, true) != ObservationContinuity::Result::kReady ||
      hold_gap.evaluate(observation, 2053000U, true, true) != ObservationContinuity::Result::kWait)
    return 81;
  observation.state_received_us = 2053001U;
  if (hold_gap.evaluate(observation, 2053001U, true, true) != ObservationContinuity::Result::kReady)
    return 82;
  observation.state_received_us = 1000000U;
  ObservationContinuity continuity(250000U, 2000000U, 500000U);
  if (continuity.evaluate(observation, 1000000U, false, true) != ObservationContinuity::Result::kReady)
    return 41;
  observation.state_received_us = 2200000U;
  if (continuity.evaluate(observation, 2200000U, true, true) != ObservationContinuity::Result::kReady)
    return 42;  // Suppressed status cannot stop a live action.
  if (continuity.evaluate(observation, 2500000U, true, true) != ObservationContinuity::Result::kWait)
    return 43;  // A short state gap pauses admission without latching failure.
  observation.state_received_us = 2510000U;
  if (continuity.evaluate(observation, 2510000U, true, true) != ObservationContinuity::Result::kReady)
    return 44;
  if (continuity.evaluate(observation, 4900000U, true, true) != ObservationContinuity::Result::kWait ||
      continuity.evaluate(observation, 5400001U, true, true) != ObservationContinuity::Result::kLost)
    return 45;  // Sustained state loss remains fail closed.

  ObservationContinuity bootstrap(250000U, 2000000U, 500000U);
  observation.state_received_us = 1000000U;
  observation.status_received_us = 1000000U;
  if (bootstrap.evaluate(observation, 1000000U, false, false) != ObservationContinuity::Result::kWait ||
      bootstrap.evaluate(observation, 1100000U, false, true) != ObservationContinuity::Result::kReady)
    return 46;  // A transient status/state transition does not poison bootstrap.
  if (bootstrap.evaluate(observation, 1200000U, false, false) != ObservationContinuity::Result::kWait ||
      bootstrap.evaluate(observation, 3200001U, false, false) != ObservationContinuity::Result::kLost)
    return 47;
  ObservationContinuity cold(250000U, 2000000U, 500000U);
  observation.state_received_us = 3000000U;
  if (cold.evaluate(observation, 3000000U, false, true) != ObservationContinuity::Result::kWait ||
      cold.evaluate(observation, 5000001U, false, true) != ObservationContinuity::Result::kLost)
    return 48;  // Cold status must keep arriving.
  const StateAuthority expected{7U, 42U, 3U, 9U};
  if (!state_authority_is_valid(expected, expected, false, false) ||
      state_authority_is_valid(expected, expected, true, false)) return 49;
  if (state_authority_is_valid({8U,42U,3U,9U}, expected, false, false) ||
      state_authority_is_valid({7U,43U,3U,9U}, expected, false, false) ||
      state_authority_is_valid({7U,42U,4U,9U}, expected, false, false)) return 50;
  if (!state_authority_is_valid({7U,42U,3U,10U}, expected, false, false) ||
      state_authority_is_valid({7U,42U,3U,10U}, expected, false, true) ||
      state_authority_is_valid({7U,42U,3U,11U}, expected, false, false) ||
      state_authority_is_valid({7U,42U,3U,8U}, expected, false, false)) return 51;
  return 0;
}

/**
 * @brief 以原ReleaseTransitionOrigin163/253核Stopping→UnreferencedIdle164/254过渡；stage2可容许较旧Stopping Status，但不改写原锚，最终released仍需两流Idle。
 * @return 0所述检查通过，非零是具体失败检查码。
 */
int release_transition_tests() {
  DeviceObservation observation;
  observation.state_received_us = 1500000U;
  observation.status_received_us = 1000000U;
  ReleaseTransitionObservation transition{};
  transition.release_stage = 2;
  transition.original = {7U, 42U, 3U, 163U};
  transition.status = {7U, 42U, 3U, 164U};
  transition.state = {7U, 0U, 0U, 164U};
  transition.status_management_epoch = 254U;
  transition.state_management_epoch = 254U;
  transition.original_management_epoch = 253U;
  transition.status_core_state = 7U;
  transition.state_core_state = 1U;
  if (!release_idle_transition_pending(transition, observation, 1500000U, 250000U))
    return 52;  // The 500ms Status publication gap remains pending.
  if (release_idle_transition_pending(transition, observation, 1750001U, 250000U))
    return 53;  // An expired State must not extend the transition.
  if (release_idle_transition_pending(transition, observation, 2000001U, 250000U))
    return 60;  // A stale Status cannot hold the transition open.
  transition.release_stage = 1;
  if (release_idle_transition_pending(transition, observation, 1500000U, 250000U)) return 54;
  transition.release_stage = 2;
  transition.state.boot_epoch = 8U;
  if (release_idle_transition_pending(transition, observation, 1500000U, 250000U)) return 55;
  transition.state.boot_epoch = 7U;
  transition.state_management_epoch = 255U;
  if (release_idle_transition_pending(transition, observation, 1500000U, 250000U)) return 56;
  transition.state_management_epoch = 254U;
  transition.status.session_id = 43U;
  if (release_idle_transition_pending(transition, observation, 1500000U, 250000U)) return 57;
  transition.status.session_id = 42U;
  transition.state_core_state = 9U;
  if (release_idle_transition_pending(transition, observation, 1500000U, 250000U)) return 58;
  transition.state_core_state = 1U;
  transition.status_core_state = 4U;
  if (release_idle_transition_pending(transition, observation, 1500000U, 250000U)) return 59;
  transition.status_core_state = 7U;
  transition.status.motion_epoch = transition.state.motion_epoch = 165U;
  if (release_idle_transition_pending(transition, observation, 1500000U, 250000U)) return 61;
  transition.status.motion_epoch = transition.state.motion_epoch = 164U;
  transition.status_management_epoch = transition.state_management_epoch = 255U;
  if (release_idle_transition_pending(transition, observation, 1500000U, 250000U)) return 62;
  ReleaseTransitionOrigin origin;
  origin.capture({7U, 42U, 3U, 163U}, 253U);
  const StateAuthority coherent_disabling{7U, 42U, 3U, 164U};
  const std::uint32_t updated_bootstrap_management_epoch = 254U;
  const auto delayed_idle = origin.observation(
      coherent_disabling, {7U, 0U, 0U, 164U},
      updated_bootstrap_management_epoch, 254U, 7U, 1U, 2);
  if (!release_idle_transition_pending(delayed_idle, observation, 1500000U, 250000U))
    return 63;  // Coherent Disabling must not rebase the release-start epoch.

  wire::DummyDeviceStatusV2 final_status;
  wire::DummyJointStateV2 final_state;
  final_status.set_boot_epoch(7U);
  final_status.set_session_id(0U);
  final_status.set_reference_epoch(0U);
  final_status.set_core_state(1U);
  final_status.set_motion_epoch(164U);
  final_status.set_management_epoch(254U);
  final_state.set_boot_epoch(7U);
  final_state.set_session_id(0U);
  final_state.set_reference_epoch(0U);
  final_state.set_core_state(1U);
  final_state.set_motion_epoch(164U);
  final_state.set_management_epoch(254U);
  if (released_idle_status_confirmed(final_status, final_state))
    return 64;  // A matching Idle Status without release_commit is not a terminal observation.
  final_status.set_release_committed(true);
  if (!released_idle_status_confirmed(final_status, final_state)) return 65;
  final_state.set_motion_epoch(165U);
  if (released_idle_status_confirmed(final_status, final_state)) return 66;
  final_state.set_motion_epoch(164U);
  final_status.set_permit_remaining_us(1U);
  if (released_idle_status_confirmed(final_status, final_state)) return 67;
  return 0;
}

/**
 * @brief 无channel/standalone读取应为空；实际channel中人工发布首页receipt，再用10000次并发写核快照字段整体相干。不会写raw文件或校验页链。
 * @return 0所述检查通过，非零是具体失败检查码。
 */
int raw_first_page_snapshot_tests() {
  const DpManagementParameters facade_parameters{"tcp/127.0.0.1:29998", {1}, {2}, 2, 63};
  DpManagementTransport no_channel(facade_parameters,
      DpManagementTransport::Connection::kDeviceWorker);
  if (no_channel.raw_first_page_written().run_id != 0U) return 70;
  DpManagementTransport standalone(facade_parameters);
  if (standalone.raw_first_page_written().run_id != 0U) return 71;

  dummy_ros_control::TransportParameters parameters{
      facade_parameters.endpoint, facade_parameters.device, facade_parameters.client,
      facade_parameters.profile, facade_parameters.mask};
  dummy_ros_control::GuardedZenbeddedSystem system;
  auto provider = std::make_unique<dummy_ros_control::DeviceTransport>(system, parameters);
  auto channel = dummy_ros_control::DeviceTransport::management_channel(parameters);
  if (!channel) return 72;
  if (no_channel.raw_first_page_written().run_id != 0U) return 76;
  dummy_ros_control::RawEvidenceFirstPageWritten written{};
  written.boot_epoch = 7U;
  written.session_id = 8U;
  written.run_id = 9U;
  written.motion_epoch = 10U;
  written.evidence_seq = 0U;
  written.group_count = 1U;
  written.attempt_kind = 1U;
  written.category = 2U;
  written.expected_mask = 63U;
  written.submitted_mask = 63U;
  written.sent_mask = 63U;
  written.final = true;
  written.chain_valid = true;
  written.full_six = true;
  written.strict_refresh_successor = true;
  channel->publish_raw_evidence_first_page_written(written);
  const auto receipt = no_channel.raw_first_page_written();
  if (receipt.boot_epoch != 7U || receipt.session_id != 8U || receipt.run_id != 9U ||
      receipt.motion_epoch != 10U || receipt.evidence_seq != 0U ||
      receipt.group_count != 1U || receipt.attempt_kind != 1U || receipt.category != 2U ||
      receipt.expected_mask != 63U || receipt.submitted_mask != 63U ||
      receipt.sent_mask != 63U || receipt.failed_mask != 0U ||
      !receipt.final || receipt.evidence_lost || !receipt.chain_valid || !receipt.full_six ||
      !receipt.strict_refresh_successor)
    return 73;
  if (!raw_first_page_matches(receipt, 7U, 8U, 9U, 10U) ||
      raw_first_page_matches(receipt, 7U, 8U, 11U, 10U) ||
      raw_first_page_matches(receipt, 7U, 8U, 9U, 11U) ||
      raw_first_page_matches(receipt, 8U, 8U, 9U, 10U)) return 74;
  std::atomic<bool> done{false};
  std::thread writer([&] {
    for (std::uint64_t run = 10U; run < 10010U; ++run) {
      auto next = written;
      next.boot_epoch = run;
      next.session_id = run + 100U;
      next.run_id = run + 200U;
      next.motion_epoch = static_cast<std::uint32_t>(run);
      channel->publish_raw_evidence_first_page_written(next);
    }
    done.store(true, std::memory_order_release);
  });
  bool coherent = true;
  while (!done.load(std::memory_order_acquire)) {
    const auto current = no_channel.raw_first_page_written();
    if (current.run_id > 9U &&
        (current.session_id != current.boot_epoch + 100U ||
         current.run_id != current.boot_epoch + 200U ||
         current.motion_epoch != current.boot_epoch)) coherent = false;
  }
  writer.join();
  return coherent && no_channel.raw_first_page_written().run_id == 10209U ? 0 : 75;
}
}  // namespace

/**
 * @brief 测试服务直接回显身份并返回Queued+Committed成功、固定operation88。
 * @param request 借用管理请求。
 * @param output 输出响应。
 * @param context 可选管道fd指针；特殊prime RID时发P。
 * @par 未命名参数
 * RequestInfo不参与本fixture业务判断。
 * @return kOk；此人为响应不等于生产事务实现。
 */
dp::comm::Result reply(const Service::Request &request,
                       Service::Response *output,
                       const dp::comm::RequestInfo &info, void *context) {
  // 真SDK服务回调核wire显式键传播；不据此声称跨进程缓存清除或实际管理完成。
  if (info.correlation_id !=
          dummy_ros_control::management_rpc_correlation_id(request) ||
      info.idempotency_key !=
          dummy_ros_control::management_rpc_idempotency_key(request))
    return dp::comm::Result::kInvalidArgument;
  reply_calls.fetch_add(1, std::memory_order_relaxed);
  Service::Response response;
  response.set_schema_major(2); response.set_schema_minor(dummy_ref::protocol::kSchemaMinor); response.set_request_id(request.request_id());
  response.set_device_id(request.device_id()); response.set_client_instance_id(request.client_instance_id());
  response.set_boot_epoch(request.expected_boot_epoch());
  response.set_session_id(request.expected_session_id());
  response.set_profile_id(request.expected_profile_id());
  response.set_receipt(1); response.set_transaction_phase(3); response.set_result_code(0);
  response.set_operation_id(88);
  *output=response;
  if (context != nullptr && request.request_id() == 0x20000000000002ULL)
    signal_peer(*static_cast<int *>(context), 'P');
  return dp::comm::Result::kOk;
}
/**
 * @brief 运行测试服务executor并通过管道同步父进程。
 * @param endpoint TCP或PTY serial地址。
 * @param signal_fd 写同步标记的fd。
 * @param expect_rejection true时观察drop/服务调用并发D或A。
 * @return 初始化/spin/同步失败非零；普通成功peer持续运行至父RAII终止。
 */
int peer(const std::string &endpoint, int signal_fd, bool expect_rejection) {
  dp::comm::StaticStorage storage(
      {1, 1, 0, 0, 8,
       DP_PROTOCOL_PRODUCT_POSIX_DUMMY_REF_ROS_SDK_RUNTIME_MESSAGE_BYTES, 96, 1,
       0, 0, 1});
  dp::comm::TransportConfig config{"peer","",endpoint.c_str(),"management_test"};
  auto init=dp::comm::Context::init(storage,{dp::comm::MemoryMode::kStaticOnly,&config,nullptr});
  if (!init.has_value()) return 10;
  auto context=std::move(init).take_value();
  dp::comm::NodeOptions options{}; options.product_name="dummy_robot";
  options.device_id.bytes[0]=1; options.node_name="ref_control";
  auto made=context.create_node(options); if(!made.has_value()) return 11;
  auto node=std::move(made).take_value();
  auto result=node.create_service<Service>("manage_v2",reply,expect_rejection ? nullptr : &signal_fd);
  if(!result.has_value()) return 12;
  auto service=std::move(result).take_value();
  auto made_evidence = node.create_publisher<wire::DummyAttemptEvidenceV2>(
      "attempt_evidence_v2", {});
  if (!made_evidence.has_value())
    return 14;
  auto evidence = std::move(made_evidence).take_value();
  unsigned published_evidence = 0;
  dp::comm::StaticExecutor executor;
  if (executor.add_node(node) != dp::comm::Result::kOk || node.activate() != dp::comm::Result::kOk ||
      !signal_peer(signal_fd, 'R')) return 13;
  for (;;) {
    executor.spin_some(dp::comm::Duration::from_milliseconds(2));
    if (evidence_test_mode && reply_calls.load() &&
        published_evidence < (evidence_test_mode == 2 ? 22U : 5U)) {
      wire::DummyAttemptEvidenceV2 page;
      page.set_schema_major(2);
      page.set_schema_minor(dummy_ref::protocol::kSchemaMinor);
      if (published_evidence == 0)
        page.set_schema_major(3);
      if (published_evidence == 1)
        page.set_schema_minor(0);
      page.set_run_id(59);
      page.set_attempt_id(59);
      page.set_attempt_kind(1);
      page.set_category(3);
      page.set_group_id(100U + published_evidence);
      page.set_group_count(1);
      page.set_final(1);
      page.set_evidence_seq(published_evidence >= 2 ? published_evidence - 2
                                                    : 0);
      page.set_expected_mask(63);
      page.set_sent_mask(63);
      const auto published = evidence.publish(page);
      if (published_evidence < 2U && published != dp::comm::Result::kOk)
        std::cerr << "evidence publisher schema guard result="
                  << static_cast<unsigned>(published) << '\n';
      if (published == dp::comm::Result::kOk || published_evidence < 2U) {
        ++published_evidence;
        if (published_evidence == (evidence_test_mode == 2 ? 22U : 5U))
          signal_peer(signal_fd, 'E');
      }
    }
    if (expect_rejection && (context.transport_drop_stats().protocol > 0 ||
                             reply_calls.load(std::memory_order_relaxed) > 0)) {
      const bool fail_closed = context.transport_drop_stats().protocol > 0 &&
          reply_calls.load(std::memory_order_relaxed) == 0;
      signal_peer(signal_fd, fail_closed ? 'D' : 'A');
      break;
    }
  }
  return 0;
}
/**
 * @brief fork本机TCP peer，发未认证管理wire请求；要求协议drop且服务回调零次，不能仅凭peer已启动计通过。
 * @return 0所述检查通过，非零是具体失败检查码。
 */
int tcp_rejects_unauthenticated_request() {
  const std::string endpoint="tcp/127.0.0.1:"+std::to_string(24000+getpid()%10000);
  int signals[2];
  if (pipe(signals) != 0) return 20;
  const pid_t child = fork();
  if (child == 0) {
    close(signals[0]);
    const int result = peer(endpoint, signals[1], true);
    _exit(result);
  }
  close(signals[1]);
  if (child < 0) { close(signals[0]); return 21; }
  PeerProcess peer_guard(child);
  const bool peer_ready = read_peer_signal(signals[0], 'R', 3000);
  if (!peer_ready) { close(signals[0]); return 22; }
  DpManagementParameters config{endpoint,{1},{2},1,1};
  DpManagementTransport transport(config);
  if (!transport.start() || !wait_ready(transport)) { close(signals[0]); return 23; }
  const auto bytes = request_bytes();
  if (bytes.empty()) { close(signals[0]); return 26; }
  auto handle = transport.send(bytes);
  if (!handle) { close(signals[0]); return 24; }
  const bool rejected = read_peer_signal(signals[0], 'D', 3000);
  close(signals[0]);
  return rejected ? 0 : 25;
}

/**
 * @brief fork PTY peer并建转发线程，先prime再发大ID请求，核operation88、单端点第二owner kBusy及句柄取消/关停顺序；poll≤20ms只为本次样本约束，不是WCET证明。
 * @return 0所述检查通过，非零是具体失败检查码。
 */
int serial_roundtrip() {
  PtyBridge bridge;
  if (!bridge.open_pair()) return 30;
  int signals[2];
  if (pipe(signals) != 0) return 31;
  const pid_t child = fork();
  if (child == 0) {
    close(signals[0]);
    bridge.close_child_fds();
    const int result = peer(bridge.left_endpoint(), signals[1], false);
    _exit(result);
  }
  close(signals[1]);
  if (child < 0) { close(signals[0]); return 32; }
  SignalReader signal_reader{signals[0]};
  PeerProcess peer_guard(child);
  bridge.start();
  if (!read_peer_signal(signal_reader.fd, 'R', 3000)) {
    std::cerr << "组件失败 phase=peer_ready code=33 timeout_ms=3000\n";
    return 33;
  }
  DpManagementParameters config{bridge.right_endpoint(),{1},{2},1,1};
  DpManagementTransport transport(config);
  StopBridgeBeforeTransport stop_bridge{bridge};
  if(!transport.start()) return 1;
  if(!wait_ready(transport)) return 2;
  const auto prime_bytes = request_bytes(0x20000000000002ULL);
  if (prime_bytes.empty()) return 37;
  auto prime = transport.send(prime_bytes);
  if (!prime) return 38;
  if (!read_peer_signal(signal_reader.fd, 'P', 5000)) {
    std::cerr << "组件失败 phase=prime_receipt code=39 timeout_ms=5000\n";
    return 39;
  }
  prime.reset();
  const auto bytes = request_bytes();
  if (bytes.empty()) return 34;
  auto handle=transport.send(bytes); if(!handle) return 3;
  std::optional<std::vector<std::uint8_t>> response;
  const auto deadline = Clock::now() + std::chrono::seconds(4);
  while (!response && Clock::now() < deadline) {
    const auto before=std::chrono::steady_clock::now(); response=handle->poll();
    const auto poll_elapsed = std::chrono::steady_clock::now() - before;
    if(poll_elapsed>std::chrono::milliseconds(20)) {
      std::cerr << "组件失败 phase=nonblocking_poll code=4 elapsed_ns="
                << std::chrono::duration_cast<std::chrono::nanoseconds>(poll_elapsed).count()
                << " limit_ms=20\n";
      return 4;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  if(!response) {
    std::cerr << "组件失败 phase=response_wait code=5 timeout_ms=4000\n";
    return 5;
  }
  Service::Response decoded;
  if(wire::DummyManageV2ResponseCodec::decode(response->data(),response->size(),&decoded)!=wire::CodecResult::kOk ||
     decoded.request_id()!=0x20000000000001ULL || decoded.operation_id()!=88) return 6;
  dp::comm::StaticStorage second_storage({1,0,0,0,8,618,96});
  const dp::comm::TransportConfig second_config{"client",bridge.right_endpoint().c_str(),"","second_context"};
  const auto second=dp::comm::Context::init(second_storage,{dp::comm::MemoryMode::kStaticOnly,&second_config,nullptr});
  if(second.result()!=dp::comm::Result::kBusy) return 8;
  if (evidence_test_mode) {
    if (!read_peer_signal(signal_reader.fd, 'E', 3000))
      return 80;
    const auto until = Clock::now() + std::chrono::seconds(2);
    if (evidence_test_mode == 2) {
      while (!transport.attempt_evidence_overflowed() && Clock::now() < until)
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      if (!transport.attempt_evidence_overflowed())
        return 81;
    }
    unsigned count = 0;
    while (Clock::now() < until &&
           count < (evidence_test_mode == 2 ? 16U : 3U)) {
      auto sample = transport.take_attempt_evidence();
      if (!sample) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        continue;
      }
      wire::DummyAttemptEvidenceV2 page;
      if (!sample->received_us ||
          wire::DummyAttemptEvidenceV2Codec::decode(
              sample->bytes.data(), sample->bytes.size(), &page) !=
              wire::CodecResult::kOk ||
          page.schema_major() != 2 ||
          page.schema_minor() != dummy_ref::protocol::kSchemaMinor ||
          page.run_id() != 59 || page.evidence_seq() != count)
        return 82;
      ++count;
    }
    if (count != (evidence_test_mode == 2 ? 16U : 3U) ||
        transport.take_attempt_evidence() ||
        (evidence_test_mode == 1 && transport.attempt_evidence_overflowed()))
      return 83;
    transport.stop();
    if (transport.take_attempt_evidence())
      return 84;
    bridge.stop();
    return 0;
  }
  // Destroying an outstanding facade cancels on the worker, never on a caller thread.
  handle=transport.send(bytes); if (!handle) return 35;
  handle.reset();
  handle=transport.send(bytes); if (!handle) return 36;
  bridge.stop(); transport.stop(); handle.reset();
  return 0;
}

/**
 * @brief 选择纯helper、TCP拒绝或默认PTY往返模式。
 * @param argc 进程参数数量。
 * @param argv 模式/输入路径见下述说明。
 * @return 0表示所选断言通过；非零、assert或检查异常表示失败。
 * raw_first_page、continuity、release_transition是内存模式；tcp会fork并开本机端口；无参数运行serial_roundtrip，未知模式返回40。
 */
int main(int argc, char **argv) {
  if (argc == 2 && (std::string(argv[1]) == "attempt_evidence" ||
                    std::string(argv[1]) == "attempt_evidence_overflow")) {
    evidence_test_mode = std::string(argv[1]) == "attempt_evidence" ? 1 : 2;
    return serial_roundtrip();
  }
  if (argc == 2 && std::string(argv[1]) == "raw_first_page")
    return raw_first_page_snapshot_tests();
  if (argc == 2 && std::string(argv[1]) == "continuity") return observation_continuity_tests();
  if (argc == 2 && std::string(argv[1]) == "release_transition") return release_transition_tests();
  if (argc == 2 && std::string(argv[1]) == "tcp") {
    const int tcp_result = tcp_rejects_unauthenticated_request();
    if (tcp_result != 0) return tcp_result;
    std::cout << "{\"evidence_scope\":\"component_only\",\"tcp_untrusted_rpc_fail_closed\":true}\n";
    return 0;
  }
  if (argc != 1) return 40;
  const int serial_result = serial_roundtrip();
  if (serial_result != 0) {
    std::cerr << "组件失败 phase=serial_roundtrip code=" << serial_result << '\n';
    return serial_result;
  }
  std::cout << "{\"evidence_scope\":\"component_only\",\"cross_process_manage_v2\":true,\"poll_nonblocking\":true}\n";
}

// 旧46b361固件会拒绝活动Release故障owner D10；此模式不声明旧固件可恢复。
// 仅已知REST故障恢复；新模式仅原owner
// D10→原终页→Clear，无Grant/Permit/Enable/轨迹。
#include "dummy_motion_client/manual_panel_profile.hpp"
#include "dummy_motion_client/manual_ros_recovery.hpp"
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>
using namespace dummy_motion_client;
namespace {
std::uint64_t now_us() {
  return std::chrono::duration_cast<std::chrono::microseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}
template <class Bytes> std::string hex(const Bytes &b) {
  std::ostringstream s;
  for (auto v : b)
    s << std::hex << std::setfill('0') << std::setw(2) << unsigned(v);
  return s.str();
}
struct Log {
  std::FILE *file = nullptr;
  explicit Log(const std::string &path) {
    file = std::fopen(path.c_str(), "wx");
    if (!file)
      throw std::runtime_error("report必须为尚不存在且可写路径");
  }
  ~Log() {
    if (file)
      std::fclose(file);
  }
  void write(const std::string &text) {
    if (std::fprintf(file, "%llu %s\n",
                     static_cast<unsigned long long>(now_us()),
                     text.c_str()) < 0 ||
        std::fflush(file))
      throw std::runtime_error("报告写失败");
  }
};
// 包装已有transport，仅记录实际mutation/Query字节及回复，不另写RPC实现。
struct RecordedTransport final : ManagementTransport {
  DpManagementTransport &port;
  Log &log;
  bool allow_release = false, allow_clear = true, release_sent = false,
       clear_sent = false;
  std::uint32_t next_sequence = 0;
  std::vector<std::uint64_t> prior_ids;
  std::function<bool(const ManageRequest &)> release_validator;
  struct Handle final : RequestHandle {
    std::unique_ptr<RequestHandle> inner;
    Log &log;
    Handle(std::unique_ptr<RequestHandle> p, Log &l)
        : inner(std::move(p)), log(l) {}
    std::optional<std::vector<std::uint8_t>> poll() override {
      auto data = inner->poll();
      if (data)
        log.write("RPC_RX hex=" + hex(*data));
      return data;
    }
  };
  RecordedTransport(DpManagementTransport &p, Log &l) : port(p), log(l) {}
  std::unique_ptr<RequestHandle>
  send(const std::vector<std::uint8_t> &bytes) override {
    ManageRequest r;
    if (dummy_wire::DummyManageV2RequestCodec::decode(
            bytes.data(), bytes.size(), &r) != dummy_wire::CodecResult::kOk)
      throw std::runtime_error("request decode");
    if (std::find(prior_ids.begin(), prior_ids.end(), r.request_id()) !=
        prior_ids.end())
      throw std::runtime_error("新恢复RPC root碰撞原件ID");
    if (r.operation() != 12) {
      if (r.operation() == 10 && allow_release && !release_sent &&
          release_validator && release_validator(r))
        release_sent = true;
      else if (r.operation() == 11 && allow_clear && !clear_sent)
        clear_sent = true;
      else
        throw std::runtime_error(
            "恢复请求不属于唯一受证D10/原终页后Clear/Query路径");
    }
    log.write("RPC_TX operation=" + std::to_string(r.operation()) +
              " request=" + std::to_string(r.request_id()) +
              " query_operation=" + std::to_string(r.queried_operation_id()) +
              " hex=" + hex(bytes));
    auto h = port.send(bytes);
    return h ? std::make_unique<Handle>(std::move(h), log) : nullptr;
  }
  std::uint32_t reserve_sequence(std::uint32_t n) noexcept override {
    return next_sequence ? next_sequence++ : port.reserve_sequence(n);
  }
};
template <std::size_t N>
std::array<std::uint8_t, N> unhex(const std::string &text) {
  if (text.size() != N * 2)
    throw std::runtime_error("hex长度错误");
  std::array<std::uint8_t, N> out{};
  for (std::size_t i = 0; i < N; ++i) {
    auto digit = [](char c) -> unsigned {
      if (c >= '0' && c <= '9')
        return c - '0';
      if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
      if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
      throw std::runtime_error("无效hex");
    };
    out[i] = std::uint8_t(digit(text[2 * i]) * 16 + digit(text[2 * i + 1]));
  }
  return out;
}
std::vector<std::uint8_t> frozen_raw(const std::string &path) {
  const int fd = ::open(path.c_str(), O_RDONLY | O_NOFOLLOW);
  if (fd < 0)
    throw std::runtime_error("原件open失败");
  struct Guard {
    int fd;
    ~Guard() { ::close(fd); }
  } guard{fd};
  struct stat before {
  }, after{};
  if (::fstat(fd, &before) || !S_ISREG(before.st_mode) || before.st_size < 8 ||
      before.st_size > 128 * 1024 * 1024)
    throw std::runtime_error("原件必须为8..128MiB普通文件");
  std::vector<std::uint8_t> bytes(static_cast<std::size_t>(before.st_size));
  std::size_t at = 0;
  while (at < bytes.size()) {
    const auto n = ::read(fd, bytes.data() + at, bytes.size() - at);
    if (n <= 0)
      throw std::runtime_error("原件读取失败");
    at += n;
  }
  if (::fstat(fd, &after) || before.st_dev != after.st_dev ||
      before.st_ino != after.st_ino || before.st_size != after.st_size ||
      before.st_mtim.tv_sec != after.st_mtim.tv_sec ||
      before.st_mtim.tv_nsec != after.st_mtim.tv_nsec ||
      before.st_ctim.tv_sec != after.st_ctim.tv_sec ||
      before.st_ctim.tv_nsec != after.st_ctim.tv_nsec)
    throw std::runtime_error("原件读取期间改变");
  return bytes;
}
std::string serial_path(const std::string &endpoint) {
  if (endpoint.rfind("serial/", 0) != 0)
    throw std::runtime_error("恢复仅允许已绑定serial端点");
  auto path = endpoint.substr(7);
  path = path.substr(0, path.find('#'));
  if (path.empty() || path[0] != '/')
    throw std::runtime_error("无效serial路径");
  return std::filesystem::canonical(path).string();
}
void ensure_no_other_owner(const std::string &path) {
  struct stat target {};
  if (::stat(path.c_str(), &target) || !S_ISCHR(target.st_mode))
    throw std::runtime_error("串口不是字符设备");
  for (const auto &process : std::filesystem::directory_iterator("/proc")) {
    const auto pid = process.path().filename().string();
    if (pid.empty() || !std::all_of(pid.begin(), pid.end(), ::isdigit) ||
        pid == std::to_string(::getpid()))
      continue;
    std::error_code ec;
    std::filesystem::directory_iterator fd(process.path() / "fd", ec), end;
    for (; !ec && fd != end; fd.increment(ec)) {
      struct stat value {};
      if (!::stat(fd->path().c_str(), &value) && S_ISCHR(value.st_mode) &&
          value.st_rdev == target.st_rdev)
        throw std::runtime_error("串口已被PID " + pid + "持有，禁止并发");
    }
  }
}
void exclusive_worker_fd(const std::string &path) {
  struct stat target {};
  if (::stat(path.c_str(), &target))
    throw std::runtime_error("串口stat失败");
  for (const auto &fd : std::filesystem::directory_iterator("/proc/self/fd")) {
    struct stat value {};
    if (::stat(fd.path().c_str(), &value) || !S_ISCHR(value.st_mode) ||
        value.st_rdev != target.st_rdev)
      continue;
    if (::ioctl(std::stoi(fd.path().filename().string()), TIOCEXCL) < 0)
      throw std::runtime_error("TIOCEXCL失败");
    return;
  }
  throw std::runtime_error("未找到本工具worker串口FD");
}
struct Samples {
  dummy_wire::DummyDeviceStatusV2 status;
  dummy_wire::DummyJointStateV2 state;
  std::uint64_t state_at{}, status_at{}, first_at{}, last_source_at{};
  std::uint32_t seq{};
  std::uint64_t produced{};
  unsigned count{};
  std::array<double, 6> minimum{}, maximum{};
  std::array<std::uint32_t, 6> axes{};
  bool observe(const DeviceObservation &raw, Log &log) {
    if (raw.state_received_us == state_at &&
        raw.status_received_us == status_at)
      return false;
    if (dummy_wire::DummyJointStateV2Codec::decode(raw.state.data(),
                                                   raw.state.size(), &state) !=
            dummy_wire::CodecResult::kOk ||
        dummy_wire::DummyDeviceStatusV2Codec::decode(
            raw.status.data(), raw.status.size(), &status) !=
            dummy_wire::CodecResult::kOk)
      return false;
    log.write("OBS state_received_us=" + std::to_string(raw.state_received_us) +
              " status_received_us=" + std::to_string(raw.status_received_us) +
              " state_hex=" + hex(raw.state) +
              " status_hex=" + hex(raw.status));
    state_at = raw.state_received_us;
    status_at = raw.status_received_us;
    bool progress = count == 0 || (std::int32_t(state.state_seq() - seq) > 0 &&
                                   state.produced_at_us() > produced);
    for (unsigned i = 0; i < 6; ++i)
      if (count && std::int32_t(state.feedback_seq()[i] - axes[i]) <= 0)
        progress = false;
    if (progress) {
      seq = state.state_seq();
      produced = state.produced_at_us();
      std::copy(std::begin(state.feedback_seq()),
                std::end(state.feedback_seq()), axes.begin());
      if (!count) {
        first_at = state_at;
        std::copy(std::begin(state.position_rad()),
                  std::end(state.position_rad()), minimum.begin());
        maximum = minimum;
      }
      for (unsigned i = 0; i < 6; ++i) {
        minimum[i] = std::min(minimum[i], state.position_rad()[i]);
        maximum[i] = std::max(maximum[i], state.position_rad()[i]);
      }
      last_source_at = state_at;
      ++count;
    }
    return true;
  }
  bool stationary() const {
    for (unsigned i = 0; i < 6; ++i)
      if (!std::isfinite(state.position_rad()[i]) ||
          !std::isfinite(state.estimated_velocity_rad_s()[i]) ||
          std::abs(state.estimated_velocity_rad_s()[i]) > 0.001 ||
          maximum[i] - minimum[i] > 0.00017453292519943296)
        return false;
    return true;
  }
  bool independent(std::uint64_t now) const {
    return count >= 3 && last_source_at >= first_at + 200000 &&
           now >= last_source_at && now - last_source_at <= 250000;
  }
};
} // namespace
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto logger = rclcpp::get_logger("manual_ros_recovery");
  std::unique_ptr<Log> output;
  try {
    std::string root, pin, bundle, report, evidence_path, evidence_sha;
    bool clear = false, interrupted = false;
    std::uint64_t expected_boot = 0;
    for (int i = 1; i < argc; ++i) {
      std::string a = argv[i];
      if (a == "--verify-disabled")
        continue;
      if (a == "--clear-known-interrupted-rest") {
        if (clear)
          throw std::runtime_error("清错模式冲突");
        clear = interrupted = true;
        continue;
      }
      if (a == "--clear-known-rest-timeout") {
        if (clear)
          throw std::runtime_error("清错模式冲突");
        clear = true;
        continue;
      }
      if (i + 1 >= argc)
        throw std::runtime_error("参数缺值");
      std::string v = argv[++i];
      if (a == "--deployment-root")
        root = v;
      else if (a == "--deployment-manifest-sha256")
        pin = v;
      else if (a == "--sdk-bundle-root")
        bundle = v;
      else if (a == "--report")
        report = v;
      else if (a == "--disable-evidence")
        evidence_path = v;
      else if (a == "--disable-evidence-sha256")
        evidence_sha = v;
      else if (a == "--expected-boot")
        expected_boot = std::stoull(v);
      else
        throw std::runtime_error("未知参数 " + a);
    }
    if (root.empty() || pin.empty() || bundle.empty() || report.empty() ||
        (clear && !expected_boot))
      throw std::runtime_error(
          "必须明确deployment-root/manifest "
          "pin/sdk-bundle-root/report；清错另需expected-boot");
    if (interrupted && (evidence_path.empty() || evidence_sha.empty()))
      throw std::runtime_error("中断REST恢复必须明确原DPRAW路径与SHA256");
    if (!interrupted && (!evidence_path.empty() || !evidence_sha.empty()))
      throw std::runtime_error("原页参数只适用于中断REST模式");
    const auto deployed = load_manual_panel_deployment(root, pin, bundle);
    ManualInterruptedRestProof proof;
    if (interrupted) {
      ManualRecoveryIdentity identity{deployed.transport.device_id,
                                      expected_boot, deployed.profile_id,
                                      deployed.profile};
      std::string why;
      if (!validate_interrupted_rest_raw(frozen_raw(evidence_path),
                                         unhex<32>(evidence_sha), identity,
                                         proof, why))
        throw std::runtime_error("原件证明拒绝: " + why);
    }
    output = std::make_unique<Log>(report);
    auto &log = *output;
    const auto device_path = serial_path(deployed.endpoint);
    ensure_no_other_owner(device_path);
    DpManagementParameters params;
    params.endpoint = deployed.endpoint;
    params.device = deployed.transport.device_id;
    params.profile = deployed.profile_id;
    params.mask = deployed.joint_mask;
    std::random_device random;
    for (auto &b : params.client)
      b = std::uint8_t(random());
    params.client[0] |= 1;
    if (interrupted)
      params.client = proof.client;
    log.write("START mode=" +
              std::string(interrupted ? "clear-known-interrupted-rest"
                                      : (clear ? "clear-known-rest-timeout"
                                               : "verify-disabled")) +
              " client=" + hex(params.client) + " deployment=" + root +
              " manifest=" + pin +
              " firmware=" + hex(deployed.profile.firmware_hash) +
              " model=" + hex(deployed.profile.model_hash) +
              " policy=" + hex(deployed.profile.policy_hash));
    if (interrupted)
      log.write("ORIGINAL_PROOF path=" + evidence_path + " sha256=" +
                hex(proof.file_sha) + " original_client=" + hex(proof.client) +
                " original_session=" + std::to_string(proof.session) +
                " run=" + std::to_string(proof.run) +
                " parent=" + std::to_string(proof.parent) +
                " group=" + std::to_string(proof.group) +
                " motion=" + std::to_string(proof.motion) + " management=" +
                std::to_string(proof.management) + " next_request_seq=" +
                std::to_string(proof.next_management_sequence));
    DpManagementTransport transport(
        params, DpManagementTransport::Connection::kStandaloneWorker);
    if (!transport.start())
      throw std::runtime_error("Standalone transport start失败");
    const auto begin = now_us();
    bool exclusive = false;
    Samples samples;
    ManualRecoveryIdentity expected;
    expected.device = params.device;
    expected.profile = params.profile;
    expected.bundle = deployed.profile;
    expected.boot = expected_boot;
    std::string why = "NO_OBSERVATION";
    while (rclcpp::ok() && now_us() - begin < 8000000) {
      if (transport.ready() && !exclusive) {
        ensure_no_other_owner(device_path);
        exclusive_worker_fd(device_path);
        ensure_no_other_owner(device_path);
        exclusive = true;
        log.write("USB_EXCLUSIVE pid=" + std::to_string(::getpid()));
      }
      auto raw = transport.observe();
      if (raw && samples.observe(*raw, log)) {
        if (!expected.boot && samples.status.boot_epoch())
          expected.boot = samples.status.boot_epoch();
        const auto shape =
            interrupted ? ManualRecoveryShape::kInterruptedRestFault
            : clear     ? ManualRecoveryShape::kKnownRestTimeoutFault
                        : (samples.status.core_state() == 9
                               ? ManualRecoveryShape::kKnownRestTimeoutFault
                               : ManualRecoveryShape::kRetainedDisabledIdle);
        const bool valid =
            exclusive &&
            validate_manual_recovery(samples.status, samples.state, expected,
                                     now_us(), samples.state_at,
                                     samples.status_at, shape, why) &&
            (!interrupted || interrupted_rest_live_matches(
                                 proof, samples.status, samples.state));
        if (!valid)
          samples.count = 0;
        else if (samples.independent(now_us()) &&
                 (!interrupted || samples.stationary()))
          break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (!exclusive || !samples.independent(now_us()) ||
        !validate_manual_recovery(
            samples.status, samples.state, expected, now_us(), samples.state_at,
            samples.status_at,
            interrupted ? ManualRecoveryShape::kInterruptedRestFault
            : clear     ? ManualRecoveryShape::kKnownRestTimeoutFault
                        : (samples.status.core_state() == 9
                               ? ManualRecoveryShape::kKnownRestTimeoutFault
                               : ManualRecoveryShape::kRetainedDisabledIdle),
            why))
      throw std::runtime_error("只读前置不满足: " + why);
    if (interrupted &&
        (!interrupted_rest_live_matches(proof, samples.status, samples.state) ||
         !samples.stationary()))
      throw std::runtime_error("原件与当前独立静止Fault身份不匹配");
    auto before = samples.state;
    log.write("DISABLED_BEFORE boot=" + std::to_string(expected.boot) +
              " ref=" + std::to_string(before.reference_epoch()) +
              " source_seq=" + std::to_string(before.state_seq()) +
              " motion=" + std::to_string(before.motion_epoch()) +
              " management=" + std::to_string(before.management_epoch()) +
              " state_received_us=" + std::to_string(samples.state_at) +
              " status_received_us=" + std::to_string(samples.status_at) +
              " formal_release_committed=" +
              std::to_string(samples.status.release_committed()) +
              " 不证明独立motor_error_mask为0");
    if (!clear) {
      RCLCPP_INFO(logger,
                  "VERIFY_DISABLED；无mutation；不证明独立电机错误读回");
      transport.stop();
      rclcpp::shutdown();
      return 0;
    }
    RecordedTransport recorded(transport, log);
    if (interrupted) {
      recorded.allow_release = true;
      recorded.allow_clear = false;
      recorded.next_sequence = proof.next_management_sequence;
      recorded.prior_ids = proof.request_ids;
    }

    WireManagementClient client(recorded, 0, now_us());
    ManageRequest request;
    std::uint8_t device[16], client_id[16], policy[32];
    std::copy(params.device.begin(), params.device.end(), device);
    std::copy(params.client.begin(), params.client.end(), client_id);
    std::copy(deployed.profile.policy_hash.begin(),
              deployed.profile.policy_hash.end(), policy);
    request.set_device_id(device);
    request.set_client_instance_id(client_id);
    request.set_operation(11);
    request.set_operator_confirmed(true);
    request.set_expected_boot_epoch(expected.boot);
    request.set_expected_session_id(0);
    request.set_expected_reference_epoch(before.reference_epoch());
    request.set_expected_motion_epoch(before.motion_epoch());
    request.set_expected_management_epoch(before.management_epoch());
    request.set_expected_profile_id(params.profile);
    request.set_configured_mask(63);
    request.set_freshness_ticket(before.freshness_ticket());
    request.set_compiled_policy_sha256(policy);
    if (interrupted) {
      const auto origin_state = samples.state;
      const auto origin_status = samples.status;
      recorded.release_validator = [proof, origin_status, origin_state](
                                       const ManageRequest &candidate) {
        return interrupted_release_request_matches(candidate, proof,
                                                   origin_status, origin_state);
      };
      // 这里expected_session是原安全账本owner，当前公开State/Status仍必须为session0。
      request.set_operation(10);
      request.set_expected_session_id(proof.session);
      InterruptedRestReleaseEvidence fresh_evidence(proof);
      while (auto old = transport.take_attempt_evidence()) {
        dummy_wire::DummyAttemptEvidenceV2 page;
        if (dummy_wire::DummyAttemptEvidenceV2Codec::decode(
                old->bytes.data(), old->bytes.size(), &page) !=
                dummy_wire::CodecResult::kOk ||
            page.run_id() != proof.run || page.evidence_seq() != 0 ||
            !std::equal(proof.chain.begin(), proof.chain.end(),
                        page.chain_sha256()))
          throw std::runtime_error("D10前出现未绑定的新原页");
        log.write("ORIGINAL_PAGE_REPLAY received_us=" +
                  std::to_string(old->received_us) + " hex=" + hex(old->bytes));
      }
      if (transport.attempt_evidence_overflowed())
        throw std::runtime_error("原页FIFO溢出");
      const auto release_at = now_us();
      const auto release_root = client.submit(request, release_at);
      if (!release_root)
        throw std::runtime_error("D10未提交");
      log.write("RELEASE_ROOT request=" + std::to_string(release_root));
      bool release_committed = false, release_confirmed = false;
      std::uint64_t release_reply_at = 0, release_operation = 0;
      const auto old_sequence = samples.state.state_seq();
      samples.count = 0;
      while (rclcpp::ok() && now_us() - release_at < 8000000) {
        if (auto raw = transport.observe())
          samples.observe(*raw, log);
        while (auto raw = transport.take_attempt_evidence()) {
          log.write(
              "NEW_EVIDENCE received_us=" + std::to_string(raw->received_us) +
              " hex=" + hex(raw->bytes));
          if (!fresh_evidence.consume(raw->bytes, raw->received_us, why))
            throw std::runtime_error("新D10原页拒绝: " + why);
        }
        if (transport.attempt_evidence_overflowed())
          throw std::runtime_error("新D10原页FIFO溢出");
        if (!release_committed)
          if (auto reply = client.poll(release_root)) {
            log.write(
                "RELEASE_REPLY root=" + std::to_string(reply->request_id) +
                " operation=" + std::to_string(reply->operation_id) +
                " phase=" + std::to_string(reply->transaction_phase) +
                " result=" + std::to_string(reply->result_code));
            if (reply->rejected || (reply->result_valid && !reply->succeeded))
              throw std::runtime_error("D10被拒绝/失败；绝不Clear");
            if (reply->result_valid && reply->succeeded &&
                reply->request_id == release_root && reply->operation_id &&
                reply->transaction_phase == 3 && reply->result_code == 0 &&
                reply->release_committed &&
                reply->reference_epoch == proof.reference &&
                reply->session_id == 0 &&
                reply->motion_epoch == proof.motion + 1 &&
                reply->management_epoch == proof.management + 1) {
              release_committed = true;
              release_reply_at = now_us();
              release_operation = reply->operation_id;
            }
          }
        if (release_committed && fresh_evidence.terminal_seen &&
            samples.state_at >= std::max(release_reply_at,
                                         fresh_evidence.terminal_received_us) &&
            samples.status_at >=
                std::max(release_reply_at,
                         fresh_evidence.terminal_received_us) &&
            std::int32_t(samples.state.state_seq() - old_sequence) > 0 &&
            samples.state.reference_epoch() == proof.reference &&
            samples.state.motion_epoch() == proof.motion + 1 &&
            samples.state.management_epoch() == proof.management + 1 &&
            samples.status.run_id() == proof.run &&
            samples.status.parent_run_id() == proof.parent &&
            samples.status.group_count() == 2 &&
            validate_manual_recovery(
                samples.status, samples.state, expected, now_us(),
                samples.state_at, samples.status_at,
                ManualRecoveryShape::kInterruptedReleasedFault, why) &&
            samples.independent(now_us()) && samples.stationary()) {
          release_confirmed = true;
          break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      if (!release_confirmed)
        throw std::runtime_error(
            "D10未取得COMMITTED+新Disable SENT+原终页+fresh双Fault释放: " +
            why);
      log.write("RELEASE_PROVEN root=" + std::to_string(release_root) +
                " operation=" + std::to_string(release_operation) +
                " new_group=" + std::to_string(fresh_evidence.new_group) +
                " terminal_chain=" + hex(fresh_evidence.proof.chain) +
                " terminal_received_us=" +
                std::to_string(fresh_evidence.terminal_received_us) +
                " state_received_us=" + std::to_string(samples.state_at) +
                " status_received_us=" + std::to_string(samples.status_at));
      recorded.allow_clear = true;
      before = samples.state;
      request.set_operation(11);
      request.set_expected_session_id(0);
      request.set_expected_motion_epoch(before.motion_epoch());
      request.set_expected_management_epoch(before.management_epoch());
      request.set_freshness_ticket(before.freshness_ticket());
    }
    const auto submitted = now_us();
    const auto root_id = client.submit(request, submitted);
    if (!root_id)
      throw std::runtime_error("ClearFault未提交");
    log.write("CLEAR_ROOT request=" + std::to_string(root_id));
    bool committed = false;
    std::uint64_t operation = 0, committed_at = 0;
    std::uint32_t confirmed_motion = 0, confirmed_management = 0;
    while (rclcpp::ok() && now_us() - submitted < 5000000) {
      if (auto raw = transport.observe())
        samples.observe(*raw, log);
      if (auto reply = client.poll(root_id)) {
        log.write("CLEAR_REPLY root=" + std::to_string(reply->request_id) +
                  " operation=" + std::to_string(reply->operation_id) +
                  " phase=" + std::to_string(reply->transaction_phase) +
                  " result=" + std::to_string(reply->result_code));
        if (reply->rejected || (reply->result_valid && !reply->succeeded))
          throw std::runtime_error("ClearFault被拒绝/失败");
        if (reply->result_valid && reply->succeeded &&
            reply->request_id == root_id && reply->operation_id &&
            reply->transaction_phase == 3 && reply->result_code == 0) {
          if (reply->reference_epoch != before.reference_epoch() ||
              reply->session_id != 0)
            throw std::runtime_error("ClearFault结果改变参考或session");
          committed = true;
          operation = reply->operation_id;
          committed_at = now_us();
          confirmed_motion = reply->motion_epoch;
          confirmed_management = reply->management_epoch;
          break;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (!committed)
      throw std::runtime_error(
          "ClearFault有界Query未确认COMMITTED；不重发mutation");
    const auto prior_sequence = samples.state.state_seq();
    samples.count = 0;
    bool confirmed = false;
    while (rclcpp::ok() && now_us() - submitted < 8000000) {
      if (auto raw = transport.observe(); raw && samples.observe(*raw, log)) {
        if (samples.state_at >= committed_at &&
            samples.status_at >= committed_at &&
            std::int32_t(samples.state.state_seq() - prior_sequence) > 0 &&
            samples.state.reference_epoch() == before.reference_epoch() &&
            samples.state.motion_epoch() == confirmed_motion &&
            samples.state.management_epoch() == confirmed_management &&
            validate_manual_recovery(
                samples.status, samples.state, expected, now_us(),
                samples.state_at, samples.status_at,
                ManualRecoveryShape::kRetainedDisabledIdle, why) &&
            samples.independent(now_us())) {
          confirmed = true;
          break;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (!confirmed)
      throw std::runtime_error(
          "ClearFault COMMITTED后双fresh同ref Idle未确认: " + why);
    log.write("RECOVERED_DISABLED root=" + std::to_string(root_id) +
              " operation=" + std::to_string(operation) +
              " boot=" + std::to_string(expected.boot) +
              " ref=" + std::to_string(before.reference_epoch()) +
              " state_received_us=" + std::to_string(samples.state_at) +
              " status_received_us=" + std::to_string(samples.status_at) +
              " 不证明独立motor_error_mask为0");
    transport.stop();
    RCLCPP_INFO(logger,
                "RECOVERED_DISABLED；%s；未Grant/Permit/Enable/"
                "运动，独立motor_error_mask未测",
                interrupted ? "已验证原owner D10新Disable及终页后ClearFault"
                            : "仅ClearFault");
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception &e) {
    if (output)
      try {
        output->write(std::string("FAILED disabled_after_unconfirmed reason=") +
                      e.what());
      } catch (...) {
      }
    RCLCPP_ERROR(logger, "恢复失败: %s；不自动重试mutation，不授予任何运动许可",
                 e.what());
    rclcpp::shutdown();
    return 2;
  }
}

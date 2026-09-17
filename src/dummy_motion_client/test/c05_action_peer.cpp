#include <algorithm>
#include <array>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <string>

#include "dp_comm/dp_comm.hpp"
#include "dp_dummy_robot_v2_dp_comm.hpp"
#include "dummy_motion_client/device_profile.hpp"

namespace c05_peer {
namespace wire = dp::protocol::dummy_robot_v2;
using Command = wire::DummyJointCommandV2;
using State = wire::DummyJointStateV2;
using Status = wire::DummyDeviceStatusV2;
using Service = dp::protocol::services::DummyManageV2RawFixed;

constexpr std::uint16_t kSetPermit = 5U, kEnable = 6U, kBegin = 7U, kComplete = 8U,
    kCancel = 9U, kDisableAndRelease = 10U, kQuery = 12U;
constexpr std::uint16_t kIdle = 2U, kEnablingHold = 3U, kHold = 4U, kRunning = 5U,
    kCompleting = 6U, kStopping = 7U, kDisabling = 8U, kFault = 9U;
constexpr std::uint8_t kQueued = 1U;
constexpr std::uint16_t kPending = 8U;

std::uint64_t now_us() noexcept {
  return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

struct Options { std::string endpoint, contract_bundle, profile_name; std::uint8_t device[16]{}, client[16]{}; std::uint32_t profile = 1U; std::uint8_t mask = 1U; bool motion_supported = true; };
struct Operation { std::uint64_t request = 0U, id = 0U; std::uint16_t kind = 0U; };

bool parse_id(const std::string &text, std::uint8_t (&out)[16]) {
  if (text.size() != 32U) return false;
  const auto hex = [](char c) -> int { if (c >= '0' && c <= '9') return c - '0'; if (c >= 'a' && c <= 'f') return c - 'a' + 10; if (c >= 'A' && c <= 'F') return c - 'A' + 10; return -1; };
  for (std::size_t i = 0; i < 16U; ++i) { const int high = hex(text[i * 2U]), low = hex(text[i * 2U + 1U]); if (high < 0 || low < 0) return false; out[i] = static_cast<std::uint8_t>((high << 4U) | low); }
  return true;
}
bool number(const std::string &text, std::uint32_t *value) {
  char *end = nullptr; const unsigned long parsed = std::strtoul(text.c_str(), &end, 0);
  if (text.empty() || end == text.c_str() || *end != '\0' || parsed > UINT32_MAX) return false;
  *value = static_cast<std::uint32_t>(parsed); return true;
}
bool parse(int argc, char **argv, Options *options, bool *self_test) {
  *self_test = false;
  if (!parse_id("01000000000000000000000000000000", options->device) || !parse_id("02000000000000000000000000000000", options->client)) return false;
  for (int i = 1; i < argc; ++i) {
    const std::string flag(argv[i]); if (flag == "--self-test") { *self_test = true; continue; }
    if (++i == argc) return false; const std::string value(argv[i]);
    if (flag == "--endpoint") options->endpoint = value;
    else if (flag == "--device-id") { if (!parse_id(value, options->device)) return false; }
    else if (flag == "--client-id") { if (!parse_id(value, options->client)) return false; }
    else if (flag == "--profile-id") { if (!number(value, &options->profile) || !options->profile) return false; }
    else if (flag == "--configured-mask") { std::uint32_t mask = 0; if (!number(value, &mask) || mask == 0 || mask > 63) return false; options->mask = static_cast<std::uint8_t>(mask); }
    else if (flag == "--contract-bundle") options->contract_bundle = value;
    else if (flag == "--profile-name") options->profile_name = value;
    else if (flag == "--motion-supported") {
      if (value == "true") options->motion_supported = true;
      else if (value == "false") options->motion_supported = false;
      else return false;
    }
    else return false;
  }
  return *self_test || (!options->endpoint.empty() && (options->contract_bundle.empty() == options->profile_name.empty()));
}

class Model {
 public:
  bool enable() { if (state != kHold || epoch != 0) return false; ++epoch; state = kEnablingHold; state = kHold; return true; }
  bool begin() { if (state != kHold) return false; ++epoch; state = kRunning; return true; }
  bool complete() { if (state != kRunning || !seq) return false; state = kCompleting; ++epoch; state = kHold; return true; }
  bool cancel() { if (state != kRunning) return false; ++epoch; state = kStopping; state = kHold; return true; }
  std::uint16_t state = kHold; std::uint32_t epoch = 0, seq = 1;
};

class Peer {
 public:
  explicit Peer(Options options) : options_(std::move(options)) {
    if (!options_.contract_bundle.empty()) {
      const dummy_motion_client::DeviceProfile loaded = dummy_motion_client::load_device_profile(
          options_.contract_bundle, options_.profile_name, options_.profile, options_.mask);
      std::copy(loaded.contract_hash.begin(), loaded.contract_hash.end(), contract_);
      std::copy(loaded.model_hash.begin(), loaded.model_hash.end(), model_);
      std::copy(loaded.profile_hash.begin(), loaded.profile_hash.end(), profile_hash_);
    }
  }
  static void stop() noexcept { stopping_ = true; }
  static dp::comm::Result service(const Service::Request &request, Service::Response *response, const dp::comm::RequestInfo &, void *context) {
    return context == nullptr || response == nullptr ? dp::comm::Result::kInvalidArgument : static_cast<Peer *>(context)->manage(request, response);
  }
  static void command(dp::comm::BorrowedMessage<Command> message, void *context) noexcept {
    Command command{}; if (context != nullptr && message.copy_to(&command) == dp::comm::Result::kOk) static_cast<Peer *>(context)->receive(command);
  }
  int run() {
    dp::comm::StaticStorage storage({1U, 2U, 1U, 0U, 8U, 301U, 96U, 1U, 0U, 0U, 1U});
    const dp::comm::TransportConfig transport{"peer", "", options_.endpoint.c_str(), "c05_action_peer"};
    auto initialized = dp::comm::Context::init(storage, {dp::comm::MemoryMode::kStaticOnly, &transport, nullptr}); if (!initialized.has_value()) return 10;
    auto context = std::move(initialized).take_value(); dp::comm::NodeOptions node_options{}; node_options.product_name = "dummy_robot"; node_options.node_name = "ref_control";
    std::copy(std::begin(options_.device), std::end(options_.device), node_options.device_id.bytes);
    auto made = context.create_node(node_options); if (!made.has_value()) return 11; auto node = std::move(made).take_value();
    auto states = node.create_publisher<State>("state_v2", {}); auto statuses = node.create_publisher<Status>("status_v2", {});
    auto commands = node.create_subscription<Command>("command_v2", {}, command, this); auto managed = node.create_service<Service>("manage_v2", service, this);
    if (!states.has_value() || !statuses.has_value() || !commands.has_value() || !managed.has_value()) return 12;
    auto state_publisher = std::move(states).take_value(); auto status_publisher = std::move(statuses).take_value(); auto subscription = std::move(commands).take_value(); auto server = std::move(managed).take_value();
    dp::comm::StaticExecutor executor; if (executor.add_node(node) != dp::comm::Result::kOk || node.activate() != dp::comm::Result::kOk) return 13;
    std::cout << "{\"event\":\"ready\",\"evidence_scope\":\"component_only\",\"boot_epoch\":" << boot_ << ",\"session_id\":" << session_ << ",\"reference_epoch\":" << reference_ << ",\"motion_epoch\":" << motion_ << "}" << std::endl;
    auto next_publish = std::chrono::steady_clock::now();
    while (!stopping_) {
      const auto spun = executor.spin_some(dp::comm::Duration::from_milliseconds(2U));
      if (spun != dp::comm::Result::kOk && spun != dp::comm::Result::kBusy) return 14;
      const auto now = std::chrono::steady_clock::now();
      if (now < next_publish) continue;
      publish(status_publisher, state_publisher);
      advance();
      do { next_publish += std::chrono::milliseconds(10U); } while (next_publish <= now);
    }
    return 0;
  }
 private:
  bool valid(const Service::Request &r) const noexcept {
    return r.schema_major() == 2U && r.schema_minor() == 0U && r.request_id() != 0U && valid_ticket(r.freshness_ticket()) &&
        std::equal(std::begin(r.device_id()), std::end(r.device_id()), std::begin(options_.device)) && std::equal(std::begin(r.client_instance_id()), std::end(r.client_instance_id()), std::begin(options_.client)) &&
        r.expected_boot_epoch() == boot_ && r.expected_session_id() == session_ && r.expected_reference_epoch() == reference_ && r.expected_motion_epoch() == motion_ && r.expected_management_epoch() == management_ && r.expected_profile_id() == options_.profile && r.configured_mask() == options_.mask;
  }
  bool query_identity_valid(const Service::Request &r) const noexcept {
    return r.schema_major() == 2U && r.schema_minor() == 0U && r.request_id() != 0U &&
        std::equal(std::begin(r.device_id()), std::end(r.device_id()), std::begin(options_.device)) && std::equal(std::begin(r.client_instance_id()), std::end(r.client_instance_id()), std::begin(options_.client)) &&
        r.expected_boot_epoch() == boot_ && r.expected_session_id() == session_ && r.expected_reference_epoch() == reference_ && r.expected_profile_id() == options_.profile && r.configured_mask() == options_.mask;
  }
  void base(const Service::Request &r, Service::Response *out) const {
    out->set_schema_major(2U); out->set_schema_minor(0U); out->set_device_id(options_.device);
    out->set_client_instance_id(options_.client); out->set_request_id(r.request_id());
    out->set_boot_epoch(boot_); out->set_session_id(session_); out->set_reference_epoch(reference_);
    out->set_motion_epoch(motion_); out->set_management_epoch(management_);
    out->set_profile_id(options_.profile); out->set_committed_producer_seq(accepted_);
  }
  void reject(const Service::Request &r, Service::Response *out) const {
    base(r, out); out->set_receipt(4U); out->set_result_code(1U); out->set_transaction_phase(4U);
  }
  Operation *find(std::uint64_t request, std::uint64_t id) noexcept { for (auto &item : operations_) if ((!request || item.request == request) && (!id || item.id == id)) return &item; return nullptr; }
  dp::comm::Result manage(const Service::Request &r, Service::Response *out) {
    if (r.operation() == kQuery) {
      if (!query_identity_valid(r)) { reject(r, out); return dp::comm::Result::kOk; }
      Operation *item = find(0U, r.queried_operation_id());
      if (!item) { reject(r, out); return dp::comm::Result::kOk; }
      base(r, out); out->set_receipt(kQueued); out->set_operation_id(item->id);
      const bool done = core_ == kIdle || core_ == kHold || core_ == kRunning;
      out->set_transaction_phase(done ? 3U : 2U); out->set_result_code(done ? 0U : kPending);
      std::cout << "{\"event\":\"query\",\"request_id\":" << r.request_id()
                << ",\"operation_id\":" << item->id << "}" << std::endl;
      return dp::comm::Result::kOk;
    }
    if (!valid(r)) { reject(r, out); return dp::comm::Result::kOk; }
    bool terminal_position = true;
    for (std::size_t axis = 0U; axis < 6U; ++axis)
      if ((options_.mask & (1U << axis)) != 0U && (!std::isfinite(r.terminal_position_rad()[axis]) || std::abs(r.terminal_position_rad()[axis] - position_[axis]) > 1e-9)) terminal_position = false;
    const bool terminal = r.operation() == kComplete && core_ == kRunning && accepted_ != 0U && r.terminal_producer_seq() == accepted_ && terminal_position;
    const bool allowed = (r.operation() == kEnable && core_ == kHold && motion_ == 0U) ||
        (r.operation() == kSetPermit && core_ == kHold) ||
        (r.operation() == kBegin && core_ == kHold && motion_ != 0U) ||
        (r.operation() == kCancel && (core_ == kRunning || core_ == kCompleting)) ||
        (r.operation() == kDisableAndRelease && (core_ == kHold || core_ == kFault)) || terminal;
    if (!allowed || (r.operation() != kEnable && r.operation() != kBegin && r.operation() != kCancel &&
        r.operation() != kDisableAndRelease && r.operation() != kComplete && r.operation() != kSetPermit)) {
      reject(r, out); return dp::comm::Result::kOk;
    }
    if (r.operation() == kComplete || r.operation() == kCancel) capture_hold();
    Operation &item = operations_[cursor_++ % operations_.size()]; item = {r.request_id(), next_operation_++, r.operation()}; ++management_;
    if (r.operation() == kEnable) { ++motion_; core_ = kEnablingHold; settle_ = 1U; }
    else if (r.operation() == kSetPermit) {}
    else if (r.operation() == kBegin) { ++motion_; core_ = kRunning; }
    else if (r.operation() == kComplete) { core_ = kCompleting; settle_ = 2U; }
    else if (r.operation() == kDisableAndRelease) { ++motion_; core_ = kDisabling; settle_ = 1U; }
    else { ++motion_; core_ = kStopping; settle_ = 1U; }
    base(r, out); out->set_receipt(kQueued); out->set_result_code(kPending);
    out->set_operation_id(item.id); out->set_transaction_phase(1U);
    std::cout << "{\"event\":\"manage\",\"operation\":" << r.operation() << ",\"request_id\":" << r.request_id() << ",\"operation_id\":" << item.id << ",\"core_state\":" << core_ << ",\"motion_epoch\":" << motion_ << "}" << std::endl; return dp::comm::Result::kOk;
  }
  void receive(const Command &c) noexcept {
    const bool identity = c.schema_major() == 2U && c.schema_minor() == 0U && valid_ticket(c.freshness_ticket()) && c.producer_seq() > accepted_ && c.boot_epoch() == boot_ && c.session_id() == session_ && c.reference_epoch() == reference_ && c.motion_epoch() == motion_ && c.profile_id() == options_.profile && c.configured_mask() == options_.mask && std::equal(std::begin(c.device_id()), std::end(c.device_id()), std::begin(options_.device)) && std::equal(std::begin(c.client_instance_id()), std::end(c.client_instance_id()), std::begin(options_.client));
    if (!identity || (core_ != kEnablingHold && core_ != kHold && core_ != kRunning)) { ++rejected_commands_; return; }
    for (std::size_t i = 0; i < 6U; ++i) if ((options_.mask & (1U << i)) && !std::isfinite(c.position_rad()[i])) { ++rejected_commands_; return; }
    accepted_ = c.producer_seq(); std::copy(std::begin(c.position_rad()), std::end(c.position_rad()), std::begin(position_)); if (core_ != kRunning) std::copy(std::begin(position_), std::end(position_), std::begin(hold_));
    std::cout << "{\"event\":\"command\",\"producer_seq\":" << accepted_ << ",\"motion_epoch\":" << motion_ << ",\"position\":[";
    for (std::size_t axis = 0U; axis < 6U; ++axis) {
      if (axis != 0U) std::cout << ',';
      if ((options_.mask & (1U << axis)) == 0U) std::cout << "null";
      else std::cout << position_[axis];
    }
    std::cout << "],\"freshness_ticket\":" << c.freshness_ticket() << "}" << std::endl;
  }
  void capture_hold() noexcept {
    for (std::size_t axis = 0U; axis < 6U; ++axis)
      if ((options_.mask & (1U << axis)) != 0U) hold_[axis] = position_[axis];
  }
  void advance() noexcept { if (!settle_ || --settle_) return; if (core_ == kCompleting) { ++motion_; core_ = kEnablingHold; settle_ = 1U; } else if (core_ == kDisabling) { core_ = kIdle; session_ = 0U; } else if (core_ == kEnablingHold || core_ == kStopping) core_ = kHold; }
  void publish(dp::comm::Publisher<Status> &statuses, dp::comm::Publisher<State> &states) noexcept {
    ++ticket_; ticket_times_[ticket_%ticket_times_.size()]={ticket_,now_us()};
    Status status{}; status.set_schema_major(2U); status.set_schema_minor(0U);
    status.set_device_id(options_.device); status.set_boot_epoch(boot_); status.set_session_id(session_);
    status.set_reference_epoch(reference_); status.set_motion_epoch(motion_);
    status.set_management_epoch(management_); status.set_profile_id(options_.profile);
    status.set_core_state(core_); status.set_configured_mask(options_.mask);
    status.set_control_mode(session_ == 0U ? 0U : 1U); status.set_reference_kind(2U);
    status.set_real_motion_permitted(session_ != 0U); status.set_permit_remaining_us(session_ != 0U ? 60000000U : 0U);
    status.set_motion_supported(options_.motion_supported); status.set_committed_producer_seq(accepted_);
    status.set_command_reject_count(rejected_commands_); status.set_contract_sha256(contract_);
    status.set_model_sha256(model_); status.set_profile_sha256(profile_hash_); (void)statuses.publish(status);
    State state{}; state.set_schema_major(2U); state.set_schema_minor(0U);
    state.set_device_id(options_.device); state.set_boot_epoch(boot_); state.set_session_id(session_);
    state.set_reference_epoch(reference_); state.set_motion_epoch(motion_);
    state.set_management_epoch(management_); state.set_profile_id(options_.profile);
    state.set_state_seq(++state_seq_); state.set_produced_at_us(now_us()); state.set_freshness_ticket(ticket_);
    state.set_configured_mask(options_.mask); state.set_position_valid_mask(options_.mask);
    state.set_velocity_valid_mask(options_.mask); state.set_raw_position_valid_mask(options_.mask);
    state.set_committed_producer_seq(accepted_); state.set_admitted_producer_seq(accepted_);
    state.set_position_rad(position_); state.set_core_state(core_); state.set_hold_ready(core_ == kHold);
    (void)states.publish(state);
  }
  Options options_; std::array<Operation, 16U> operations_{}; std::size_t cursor_ = 0U; std::uint64_t boot_ = 101U, session_ = 201U, ticket_ = 701U, next_operation_ = 1001U; std::uint32_t reference_ = 301U, motion_ = 0U, management_ = 401U, state_seq_ = 0U, accepted_ = 0U, rejected_commands_ = 0U; std::uint16_t core_ = kHold; std::uint8_t settle_ = 0U; double position_[6]{}, hold_[6]{};
  std::array<std::pair<std::uint64_t,std::uint64_t>,64> ticket_times_{};
  bool valid_ticket(std::uint64_t ticket) const noexcept {
    const auto &issued=ticket_times_[ticket%ticket_times_.size()];
    return ticket!=0 && issued.first==ticket && now_us()-issued.second<=50000;
  }
  std::uint8_t contract_[32] = {0x91U,0x40U,0x74U,0xd3U,0x13U,0x91U,0xb4U,0x55U,0xd9U,0x0dU,0x8eU,0xdcU,0x83U,0x3bU,0xfeU,0xbbU,0xe0U,0x82U,0xd1U,0xefU,0x1fU,0x41U,0x1dU,0xf2U,0x1eU,0x5eU,0x9fU,0xe2U,0x3fU,0x9bU,0x28U,0xf8U};
  std::uint8_t model_[32]{}, profile_hash_[32]{};
  static std::atomic<bool> stopping_;
};
std::atomic<bool> Peer::stopping_{false};
void signal_handler(int) { Peer::stop(); }
int self_test() { Model model; assert(model.enable()); assert(model.begin()); assert(model.complete()); assert(model.begin()); assert(model.cancel()); std::cout << "{\"event\":\"self_test\",\"c05_lifecycle\":true}" << std::endl; return 0; }
}  // namespace c05_peer

int main(int argc, char **argv) {
  c05_peer::Options options; bool self_test = false; if (!c05_peer::parse(argc, argv, &options, &self_test)) return 2;
  if (self_test) return c05_peer::self_test(); std::signal(SIGINT, c05_peer::signal_handler); std::signal(SIGTERM, c05_peer::signal_handler); return c05_peer::Peer(std::move(options)).run();
}

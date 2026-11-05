/** @file protocol_r11_real_plugin_probe.cpp
 * @brief 显式URDF/部署输入下装载真实六轴插件，仅configure并观测NoRef与GetInfo。
 * @details 默认执行会连接显式端点；--validate-config只核输入。绝不activate/write/reference/enable，
 * 管理请求通过该插件已有DeviceTransport的单槽channel，不创建第二通信owner。
 */
#include "dummy_ros_control/command_channel.hpp"
#include "dummy_ros_control/device_management_channel.hpp"
#include "dummy_ros_control/device_transport.hpp"
#include "dummy_ros_control/guarded_zenbedded_system.hpp"
#include "hardware_interface/component_parser.hpp"
#include "pluginlib/class_loader.hpp"
#include <openssl/evp.h>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <sstream>
#include <thread>
#include <stdexcept>

namespace {
namespace wire = dp::iface;
/** @brief 读取显式现有文件，失败/空文件抛出并在连接设备之前终止。 */
std::string read_file(const char* path) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) throw std::runtime_error("cannot read explicit input");
  std::string bytes{std::istreambuf_iterator<char>(stream), {}};
  if (bytes.empty()) throw std::runtime_error("empty explicit input");
  return bytes;
}
/** @brief 为实际输入字节计算SHA256；仅用于证据身份，不赋予设备权限。 */
std::string digest(const std::string& bytes) {
  unsigned char output[EVP_MAX_MD_SIZE]{};
  unsigned length = 0;
  if (EVP_Digest(bytes.data(), bytes.size(), output, &length, EVP_sha256(), nullptr) != 1 || length != 32U)
    throw std::runtime_error("SHA256 unavailable");
  std::ostringstream result;
  for (unsigned i = 0; i < length; ++i)
    result << std::hex << std::setfill('0') << std::setw(2) << unsigned(output[i]);
  return result.str();
}
/** @brief 复用生产能力predicate，逐项核实际Status与显式部署，不回填观测。 */
bool status_contract_matches(const wire::DummyDeviceStatusV2& status,
                             const dummy_ros_control::TransportParameters& parameters) {
  dummy_ros_control::PositionOnlyCapability expected{}, observed{};
  expected.contract_hash = parameters.contract_hash;
  expected.model_hash = parameters.model_hash;
  expected.profile_hash = parameters.profile_hash;
  expected.firmware_hash = parameters.firmware_hash;
  expected.policy_hash = parameters.policy_hash;
  expected.configured_mask = parameters.configured_mask;
  expected.capabilities = dummy_ros_control::kPositionOnlyCapability;
  expected.control_timebase_us = parameters.control_period_us;
  expected.state_publish_period_us = parameters.state_publish_period_us;
  std::copy_n(status.contract_sha256(), 32, observed.contract_hash.begin());
  std::copy_n(status.model_sha256(), 32, observed.model_hash.begin());
  std::copy_n(status.profile_sha256(), 32, observed.profile_hash.begin());
  std::copy_n(status.firmware_sha256(), 32, observed.firmware_hash.begin());
  std::copy_n(status.compiled_policy_sha256(), 32, observed.policy_hash.begin());
  observed.configured_mask = status.configured_mask();
  observed.capabilities = status.capabilities();
  observed.control_timebase_us = status.control_period_us();
  observed.state_publish_period_us = status.state_publish_period_us();
  for (std::size_t axis = 0; axis < 6U; ++axis) {
    expected.limits[axis] = {parameters.position_min[axis], parameters.position_max[axis]};
    expected.velocity_max[axis] = parameters.velocity_max[axis];
    expected.acceleration_max[axis] = parameters.acceleration_max[axis];
    observed.limits[axis] = {status.position_min_rad()[axis], status.position_max_rad()[axis]};
    observed.velocity_max[axis] = status.velocity_max_rad_s()[axis];
    observed.acceleration_max[axis] = status.acceleration_max_rad_s()[axis];
  }
  return dummy_ros_control::position_only_capability_matches(expected, observed);
}
/** @brief 验证新鲜NoRef、失能及六轴原始位置；不把raw电机位置当参考后控制位置。 */
bool no_ref(const wire::DummyJointStateV2& state, const wire::DummyDeviceStatusV2& status,
            const dummy_ros_control::TransportParameters& parameters) {
  return status_contract_matches(status, parameters) && state.boot_epoch() != 0U && state.boot_epoch() == status.boot_epoch() &&
      state.session_id() == 0U && status.session_id() == 0U &&
      state.reference_epoch() == 0U && status.reference_epoch() == 0U &&
      state.core_state() == 1U && status.core_state() == 1U &&
      !status.real_motion_permitted() && status.permit_remaining_us() == 0U &&
      state.configured_mask() == 63U && status.configured_mask() == 63U &&
      state.profile_id() == parameters.profile_id && status.profile_id() == parameters.profile_id &&
      (state.raw_position_valid_mask() & 63U) == 63U && state.position_valid_mask() == 0U &&
      std::equal(std::begin(state.device_id()), std::end(state.device_id()), parameters.device_id.begin()) &&
      std::equal(std::begin(status.device_id()), std::end(status.device_id()), parameters.device_id.begin());
}
/**
 * @brief 保存最近一次周期反馈检查的诊断，不逐帧输出、不改变资格或等待期限。
 * @details decode标志仅在fresh成立后按原短路顺序更新；age仅在对应接收时刻有效且不超前时可用。
 * 此对象仅属于Host测试主线程，退出10时输出一次；不增加设备端状态或通信owner。
 */
struct PeriodicDiagnostic {
  bool fresh = false, state_decoded = false, status_decoded = false, no_ref_ok = false;
  bool state_age_valid = false, status_age_valid = false;
  uint64_t state_age_us = 0U, status_age_us = 0U;
  wire::DummyJointStateV2 state;
  wire::DummyDeviceStatusV2 status;
};
/** @brief 仅在原周期检查失败退出时输出最后观测，未解码字段不作为设备有效值使用。 */
void print_periodic_failure(const PeriodicDiagnostic& last, bool channel, bool active,
                            unsigned samples, uint32_t sequence, uint64_t boot,
                            const dummy_ros_control::TransportParameters& parameters) {
  std::cerr << "PLUGIN_PERIODIC_FAILURE channel=" << channel << " commands_active=" << active
            << " valid_samples=" << samples << " accepted_state_seq=" << sequence << " accepted_boot=" << boot
            << " last_fresh=" << last.fresh << " state_decode=" << last.state_decoded
            << " status_decode=" << last.status_decoded << " no_ref=" << last.no_ref_ok
            << " state_age_valid=" << last.state_age_valid << " state_age_us=" << last.state_age_us
            << " status_age_valid=" << last.status_age_valid << " status_age_us=" << last.status_age_us;
  if (last.state_decoded) {
    const auto& value = last.state;
    std::cerr << " state_seq=" << value.state_seq() << " state_boot=" << value.boot_epoch()
              << " state_session=" << value.session_id() << " state_reference=" << value.reference_epoch()
              << " state_core=" << unsigned(value.core_state()) << " state_mask=" << unsigned(value.configured_mask())
              << " state_raw_mask=" << unsigned(value.raw_position_valid_mask())
              << " state_position_mask=" << unsigned(value.position_valid_mask())
              << " state_profile=" << value.profile_id()
              << " state_domain_match=" << std::equal(std::begin(value.device_id()), std::end(value.device_id()), parameters.device_id.begin());
  }
  if (last.status_decoded) {
    const auto& value = last.status;
    std::cerr << " status_boot=" << value.boot_epoch() << " status_session=" << value.session_id()
              << " status_reference=" << value.reference_epoch() << " status_core=" << unsigned(value.core_state())
              << " status_mask=" << unsigned(value.configured_mask()) << " status_profile=" << value.profile_id()
              << " permit=" << value.real_motion_permitted() << " permit_remaining_us=" << value.permit_remaining_us()
              << " status_contract_match=" << status_contract_matches(value, parameters)
              << " status_domain_match=" << std::equal(std::begin(value.device_id()), std::end(value.device_id()), parameters.device_id.begin());
  }
  std::cerr << '\n';
}
/** @brief 已configure插件退出时只cleanup已有worker，不激活硬件或另建连接。 */
struct Cleanup {
  dummy_ros_control::GuardedZenbeddedSystem& system;
  ~Cleanup() { (void)system.on_cleanup(rclcpp_lifecycle::State()); }
};
}

/** @brief 调用方式：probe URDF ENDPOINT MANIFEST [--validate-config]；执行前须由唯一设备owner授权。 */
int main(int argc, char** argv) {
  if (argc != 4 && argc != 5) return 2;
  const bool validate_only = argc == 5 && std::string(argv[4]) == "--validate-config";
  if (argc == 5 && !validate_only) return 2;
  try {
    const auto urdf = read_file(argv[1]);
    const auto manifest = read_file(argv[3]);
    auto controls = hardware_interface::parse_control_resources_from_urdf(urdf);
    const auto selected = std::find_if(controls.begin(), controls.end(), [](const auto& info) {
      return info.hardware_plugin_name == "dummy_ros_control/GuardedZenbeddedSystem";
    });
    if (selected == controls.end() || selected->joints.size() != 6U ||
        std::count_if(controls.begin(), controls.end(), [](const auto& info) {
          return info.hardware_plugin_name == "dummy_ros_control/GuardedZenbeddedSystem";
        }) != 1) return 3;
    selected->hardware_parameters["endpoint"] = argv[2];
    dummy_ros_control::TransportParameters parameters{};
    if (!dummy_ros_control::parse_transport_parameters(selected->hardware_parameters, 6U, &parameters) ||
        parameters.configured_mask != 63U || parameters.endpoint.empty()) return 4;
    std::cout << "PLUGIN_INPUTS urdf_sha=" << digest(urdf) << " manifest_sha=" << digest(manifest)
              << " endpoint=" << std::quoted(parameters.endpoint) << " axes=6 mask=63\n";
    if (validate_only) return 0;
    rclcpp::init(0, nullptr);
    pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
        "hardware_interface", "hardware_interface::SystemInterface");
    auto base = loader.createSharedInstance("dummy_ros_control/GuardedZenbeddedSystem");
    auto system = std::dynamic_pointer_cast<dummy_ros_control::GuardedZenbeddedSystem>(base);
    if (!system) return 5;
    hardware_interface::HardwareComponentParams init{};
    init.hardware_info = *selected;
    init.clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
    init.logger = rclcpp::get_logger("protocol_r11_real_plugin_probe");
    if (system->init(init) != hardware_interface::CallbackReturn::SUCCESS) return 6;
    const auto states = system->on_export_state_interfaces();
    const auto commands = system->on_export_command_interfaces();
    if (states.size() != 12U || commands.size() != 6U || system->commands_active()) return 7;
    /** @brief 启动worker前打开既有固定8槽诊断；仅此Host测试启用，不增加通信端点或改变重连。 */
    if (const auto diagnostics = dummy_ros_control::DeviceTransport::management_channel(parameters))
      diagnostics->diagnostics.enable_details(true);
    if (system->on_configure(rclcpp_lifecycle::State()) != hardware_interface::CallbackReturn::SUCCESS) return 8;
    Cleanup cleanup{*system};
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    std::shared_ptr<dummy_ros_control::DeviceManagementChannel> channel;
    unsigned samples = 0U;
    uint32_t sequence = 0U;
    uint64_t boot = 0U;
    PeriodicDiagnostic last;
    while (std::chrono::steady_clock::now() < deadline && samples < 2U) {
      if (!channel) channel = dummy_ros_control::DeviceTransport::management_channel(parameters);
      if (channel) {
        dummy_ros_control::DeviceManagementObservation observation;
        { std::lock_guard<std::mutex> lock(channel->mutex); observation = channel->observation; }
        wire::DummyJointStateV2 state;
        wire::DummyDeviceStatusV2 status;
        const auto now = dummy_ros_control::transport_steady_now_us();
        const bool fresh = observation.state_received_us && observation.status_received_us &&
            now >= observation.state_received_us && now >= observation.status_received_us &&
            now - observation.state_received_us <= parameters.max_state_age_us &&
            now - observation.status_received_us <= parameters.max_state_age_us;
        last.fresh = fresh;
        last.state_age_valid = observation.state_received_us != 0U && now >= observation.state_received_us;
        last.status_age_valid = observation.status_received_us != 0U && now >= observation.status_received_us;
        last.state_age_us = last.state_age_valid ? now - observation.state_received_us : 0U;
        last.status_age_us = last.status_age_valid ? now - observation.status_received_us : 0U;
        last.state_decoded = fresh && wire::DummyJointStateV2Codec::decode(observation.state.data(), observation.state.size(), &state) == wire::CodecResult::kOk;
        last.status_decoded = last.state_decoded && wire::DummyDeviceStatusV2Codec::decode(observation.status.data(), observation.status.size(), &status) == wire::CodecResult::kOk;
        last.no_ref_ok = last.status_decoded && no_ref(state, status, parameters);
        if (last.state_decoded) last.state = state;
        if (last.status_decoded) last.status = status;
        if (last.no_ref_ok && (samples == 0U || state.state_seq() != sequence)) {
          if (boot && boot != state.boot_epoch()) return 9;
          boot = state.boot_epoch(); sequence = state.state_seq(); ++samples;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    if (samples < 2U || !channel || system->commands_active()) {
      print_periodic_failure(last, bool(channel), system->commands_active(), samples, sequence, boot, parameters);
      /**
       * @brief 失败退出时复制既有worker统计和首拒，区分未建立连接与接收后拒绝。
       * @details 这些计数独立采样，不是跨字段事务；不改变worker、订阅、重连或5秒阈值。
       * ready说明端点曾激活；failed在一次run_connected返回后更新，后续成功但尚未返回时可保留旧失败值，
       * 另含collector失败，因此不是当前初始化阶段。first是对象生命周期首拒，不随重连清空。
       * 首拒detail仅为现有固定原因标签；附加详情只取Context-init的数值Result，不输出端点或身份文本。
       */
      const auto statistics = system->transport_statistics();
      std::cerr << "PLUGIN_WORKER_FAILURE channel_ready=" << (channel && channel->ready.load(std::memory_order_acquire))
                << " transport_ready=" << statistics.ready << " initialization_failed_lifecycle_observation=" << statistics.initialization_failed
                << " state_callbacks=" << statistics.state_callbacks << " state_messages=" << statistics.state_messages
                << " status_messages=" << statistics.status_messages << " rejected_states=" << statistics.rejected_states
                << " state_decode_rejections=" << statistics.state_decode_rejections
                << " state_identity_rejections=" << statistics.state_identity_rejections
                << " state_authority_rejections=" << statistics.state_authority_rejections
                << " state_sequence_rejections=" << statistics.state_sequence_rejections
                << " state_ingest_rejections=" << statistics.state_ingest_rejections
                << " bootstrap_rejections=" << statistics.bootstrap_rejections
                << " invalidations=" << statistics.invalidations << " first_present=" << statistics.diagnostics.first_present;
      if (statistics.diagnostics.first_present) {
        const auto& first = statistics.diagnostics.first;
        std::cerr << " first_reason=" << unsigned(first.reason) << " first_host_us=" << first.host_us
                  << " first_detail=" << std::quoted(std::string(first.detail.begin(), std::find(first.detail.begin(), first.detail.end(), '\0')));
      }
      std::cerr << '\n';
      if (channel) {
        for (std::size_t index = 0U; index < dummy_ros_control::DiagnosticJournal::kDetailCapacity; ++index) {
          const auto detail = channel->diagnostics.detail_record(index);
          if (!detail || detail->argument_count == 0U) continue;
          const std::string format(detail->format.begin(), std::find(detail->format.begin(), detail->format.end(), '\0'));
          if (format.find("device transport context init failed result=%u") == std::string::npos ||
              detail->arguments[0].kind != dummy_ros_control::DiagnosticArgument::Kind::kUnsigned) continue;
          std::cerr << "PLUGIN_CONTEXT_INIT_FAILURE lifecycle_detail_slot=" << index
                    << " result=" << detail->arguments[0].bits << '\n';
        }
      }
      return 10;
    }
    wire::DummyManageV2Request query;
    query.set_schema_major(dummy_ref::protocol::kSchemaMajor);
    query.set_schema_minor(dummy_ref::protocol::kSchemaMinor);
    uint8_t domain_device[16]{}, domain_client[16]{};
    std::copy(parameters.device_id.begin(), parameters.device_id.end(), domain_device);
    std::copy(parameters.client_instance_id.begin(), parameters.client_instance_id.end(), domain_client);
    query.set_device_id(domain_device);
    query.set_client_instance_id(domain_client);
    query.set_request_id(1U); query.set_request_seq(1U); query.set_operation(0U);
    query.set_expected_profile_id(parameters.profile_id); query.set_configured_mask(63U);
    auto pending = std::make_shared<dummy_ros_control::DeviceManagementRequest>();
    if (wire::DummyManageV2RequestCodec::encode(query, pending->request.data(), pending->request.size()) != wire::CodecResult::kOk) return 11;
    { std::lock_guard<std::mutex> lock(channel->mutex); if (channel->queued) return 12; channel->queued = pending; }
    const auto query_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (!pending->complete.load(std::memory_order_acquire) && !pending->failed.load() &&
           std::chrono::steady_clock::now() < query_deadline)
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    if (!pending->complete.load(std::memory_order_acquire) || pending->failed.load()) {
      pending->canceled.store(true); return 13;
    }
    // 响应合同不含configured_mask；同Boot的真实周期Status已独立核全6轴/63。
    wire::DummyManageV2Response reply;
    if (wire::DummyManageV2ResponseCodec::decode(pending->response.data(), pending->response.size(), &reply) != wire::CodecResult::kOk ||
        reply.request_id() != query.request_id() || reply.boot_epoch() != boot || reply.session_id() != 0U ||
        reply.profile_id() != parameters.profile_id || reply.receipt() != 1U ||
        (reply.result_code() != 0U && reply.result_code() != 23U) ||
        !std::equal(std::begin(reply.device_id()), std::end(reply.device_id()), parameters.device_id.begin()) ||
        !std::equal(std::begin(reply.client_instance_id()), std::end(reply.client_instance_id()), parameters.client_instance_id.begin()) ||
        system->commands_active()) return 14;
    std::cout << "PLUGIN_INACTIVE_RESULT=PASS axes=6 mask=63 periodic_samples=" << samples
              << " query_receipt=" << reply.receipt() << " boot=" << boot
              << " active=false motion=false\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "PLUGIN_INACTIVE_RESULT=FAIL error=" << error.what() << '\n';
    return 20;
  }
}

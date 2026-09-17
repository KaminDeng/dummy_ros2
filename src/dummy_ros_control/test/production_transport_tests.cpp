#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>

#include "dummy_ros_control/device_transport.hpp"
#include "dummy_ros_control/guarded_zenbedded_system.hpp"

int main(int argc, char **argv) {
  if (argc != 2) return 10;
  using namespace dummy_ros_control;
  if (saturating_state_deadline(UINT64_MAX - 1U, 10U) != UINT64_MAX ||
      saturating_state_deadline(100U, 10U) != 110U) return 14;
  TransportParameters parameters{};
  const std::unordered_map<std::string, std::string> valid{
      {"endpoint", argv[1]},
      {"device_id", "01000000000000000000000000000000"},
      {"client_instance_id", "02000000000000000000000000000000"},
      {"profile_id", "1"}, {"configured_mask", "1"}};
  if (!parse_transport_parameters(valid, 1U, &parameters)) return 1;
  if (parse_transport_parameters(valid, 6U, &parameters)) return 2;
  auto invalid = valid;
  invalid.erase("endpoint");
  if (parse_transport_parameters(invalid, 1U, &parameters)) return 3;
  invalid = valid;
  invalid["profile_id"] = "1garbage";
  if (parse_transport_parameters(invalid, 1U, &parameters)) return 4;
  invalid = valid;
  invalid["device_id"] = "00000000000000000000000000000000";
  if (parse_transport_parameters(invalid, 1U, &parameters)) return 5;
  parse_transport_parameters(valid, 1U, &parameters);
  GuardedZenbeddedSystem system;
  hardware_interface::HardwareComponentParams hardware{};
  hardware.hardware_info.name = "d06_hardware";
  hardware.hardware_info.type = "system";
  hardware.hardware_info.hardware_parameters = valid;
  hardware_interface::ComponentInfo joint{};
  joint.name = "Joint1";
  joint.type = "joint";
  hardware_interface::InterfaceInfo position{};
  position.name = "position";
  hardware_interface::InterfaceInfo velocity{};
  velocity.name = "velocity";
  joint.command_interfaces = {position};
  joint.state_interfaces = {position, velocity};
  hardware.hardware_info.joints = {joint};
  hardware.clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  hardware.logger = rclcpp::get_logger("production_transport_tests");
  if (system.init(hardware) != hardware_interface::CallbackReturn::SUCCESS) return 15;
  system.on_export_state_interfaces();
  system.on_export_command_interfaces();
  auto statistics = [&] { return system.transport_statistics(); };
  if (statistics().state_messages != 0U) return 6;
  if (system.on_configure(rclcpp_lifecycle::State()) != hardware_interface::CallbackReturn::SUCCESS) return 7;
  const auto feedback_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while ((statistics().state_messages < 3U || statistics().status_messages < 3U) &&
         std::chrono::steady_clock::now() < feedback_deadline) std::this_thread::sleep_for(std::chrono::milliseconds(10));
  if (statistics().state_messages < 3U || statistics().status_messages < 3U ||
      statistics().rejected_states == 0U) return 11;
  JointStateSnapshot snapshot{};
  if (system.copy_state(&snapshot)) return 8;
  if (system.read(rclcpp::Time(), rclcpp::Duration::from_nanoseconds(1000000)) !=
      hardware_interface::return_type::OK) return 16;
  system.submit_control_update({1U, 2U, 3U, 4U, 5U}, {0.25, NAN, NAN, NAN, NAN, NAN});
  const bool transferred = system.transfer_control_update();
  const auto command_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (transferred && statistics().rejected_commands == 0U &&
         std::chrono::steady_clock::now() < command_deadline) std::this_thread::yield();
  if (transferred && statistics().rejected_commands == 0U) return 13;
  system.on_error(rclcpp_lifecycle::State());
  system.on_cleanup(rclcpp_lifecycle::State());
  std::fprintf(stderr, "stop sent=%llu failed=%llu commands=%llu\n",
      static_cast<unsigned long long>(statistics().stop_requests_sent),
      static_cast<unsigned long long>(statistics().stop_request_failures),
      static_cast<unsigned long long>(statistics().commands_published));
  if (statistics().stop_requests_sent != 1U || statistics().stop_request_failures != 0U ||
      statistics().commands_published != 0U) return 9;
  std::printf("D06 real_peer negative_path (no authorized motion): state=%llu status=%llu invalid=%llu command=%llu stop_sent=%llu\n",
      static_cast<unsigned long long>(statistics().state_messages),
      static_cast<unsigned long long>(statistics().status_messages),
      static_cast<unsigned long long>(statistics().rejected_states),
      static_cast<unsigned long long>(statistics().commands_published),
      static_cast<unsigned long long>(statistics().stop_requests_sent));
  return 0;
}

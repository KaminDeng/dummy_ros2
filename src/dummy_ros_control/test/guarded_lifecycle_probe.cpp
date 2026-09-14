#include "dummy_ros_control/guarded_zenbedded_system.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

int main() {
  dummy_ros_control::GuardedZenbeddedSystem system;
  hardware_interface::HardwareComponentParams params{};
  params.hardware_info.name = "dummy";
  params.hardware_info.type = "system";
  hardware_interface::ComponentInfo joint{};
  joint.name = "Joint1";
  joint.type = "joint";
  hardware_interface::InterfaceInfo command{};
  command.name = hardware_interface::HW_IF_POSITION;
  command.initial_value = "0.0";
  joint.command_interfaces.push_back(command);
  for (const char *name : {hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY}) {
    hardware_interface::InterfaceInfo state{};
    state.name = name;
    state.initial_value = "0.0";
    joint.state_interfaces.push_back(state);
  }
  params.hardware_info.joints.push_back(joint);
  params.clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  params.logger = rclcpp::get_logger("guarded_lifecycle_probe");
  if (system.init(params) != hardware_interface::CallbackReturn::SUCCESS) return 1;
  const auto state_interfaces = system.on_export_state_interfaces();
  const auto command_interfaces = system.on_export_command_interfaces();
  if (state_interfaces.size() != 2U || command_interfaces.size() != 1U) return 6;
  dummy_ros_control::JointStateSnapshot snapshot{};
  snapshot.envelope = {7U, 1U, 1U, 100U, 1U, 1U, 1U};
  snapshot.position[0] = 0.25;
  snapshot.velocity[0] = 0.0;
  const dummy_ros_control::StateExpectation expected{7U, 1U, 0U, true, 1U, 90U, 110U, 20U};
  if (!system.ingest_state(snapshot, expected)) return 2;
  if (system.on_activate(rclcpp_lifecycle::State()) != hardware_interface::CallbackReturn::SUCCESS) return 3;
  if (system.write(rclcpp::Time(), rclcpp::Duration::from_nanoseconds(1000000)) !=
      hardware_interface::return_type::OK) return 4;
  dummy_ros_control::CommandGenerationRecord record{};
  if (system.take_publish_record(&record) || system.take_stop_request()) return 5;
  return 0;
}

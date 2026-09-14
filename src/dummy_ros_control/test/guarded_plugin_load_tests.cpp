#include <exception>
#include <iostream>
#include <memory>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "pluginlib/class_loader.hpp"

int main() {
  try {
    pluginlib::ClassLoader<controller_interface::ControllerInterface> loader(
        "controller_interface", "controller_interface::ControllerInterface");
    const auto instance = loader.createSharedInstance(
        "dummy_ros_control/GuardedJointTrajectoryController");
    if (!instance) return 2;
    pluginlib::ClassLoader<hardware_interface::SystemInterface> hardware_loader(
        "hardware_interface", "hardware_interface::SystemInterface");
    const auto hardware = hardware_loader.createSharedInstance(
        "dummy_ros_control/GuardedZenbeddedSystem");
    return hardware ? 0 : 3;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}

#include <exception>
#include <iostream>
#include <memory>

#include "controller_interface/controller_interface.hpp"
#include "pluginlib/class_loader.hpp"

int main() {
  try {
    pluginlib::ClassLoader<controller_interface::ControllerInterface> loader(
        "controller_interface", "controller_interface::ControllerInterface");
    const auto instance = loader.createSharedInstance(
        "dummy_ros_control/GuardedJointTrajectoryController");
    return instance ? 0 : 2;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}

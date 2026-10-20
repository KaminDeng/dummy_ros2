/**
 * @file guarded_plugin_load_tests.cpp
 * @brief 验证控制器/硬件类可由pluginlib按导出名实例化。
 *
 * 仅检查类注册和类型加载；不configure、activate或建立设备连接，不证明运行时生命周期/运动正常。
 */
#include <exception>
#include <iostream>
#include <memory>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "pluginlib/class_loader.hpp"

/**
 * @brief 分别加载Guarded控制器和硬件插件。
 *
 * 空实例和插件异常分别转成不同失败退出；异常文本写stderr便于定位注册/动态库问题。
 * @return 0 加载成功；1 捕获异常；2 控制器实例为空；3 硬件实例为空。
 */
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

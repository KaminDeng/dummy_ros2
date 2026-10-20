/**
 * @file guarded_lifecycle_probe.cpp
 * @brief 探查硬件插件生命周期、本地State准入及选定read/write段的分配。
 *
 * configure会创建指向127.0.0.1:17449的生产transport worker；此源不能仅因有合成State就称为纯内存测试。注释维护阶段不运行它。
 * 这里的零分配证据只覆盖计数开关打开的1000次循环，不代表整个生命周期无分配或满足硬实时上限。
 */
#include "dummy_ros_control/guarded_zenbedded_system.hpp"
#include "dummy_ros_control/command_channel.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_loader.hpp"
#include <atomic>
#include <cstdlib>
#include <new>

namespace {
/**
 * @brief 只在被测循环打开计数；插件加载、容器构造和生命周期准备不纳入统计。
 */
std::atomic<bool> count_allocations{false};
std::atomic<unsigned> allocations{0};
}
/**
 * @brief 覆盖本测试进程普通new以累计被选中阶段的调用。
 * @param size 请求字节数，原样传给malloc。
 * @return 已分配内存；失败抛std::bad_alloc。
 * @note 不是全部可能分配API的通用拦截器。
 */
void *operator new(std::size_t size) {
  if (count_allocations.load(std::memory_order_relaxed)) ++allocations;
  if (void *pointer = std::malloc(size)) return pointer;
  throw std::bad_alloc();
}
/**
 * @brief 释放普通delete对应的测试内存。
 * @param pointer 待释放指针，交给free处理（可为空）。
 * @return 无。
 */
void operator delete(void *pointer) noexcept { std::free(pointer); }
/**
 * @brief 为带大小的delete提供同一free出口。
 * @param pointer 待释放指针。
 * @par 未命名大小参数
 * std::size_t用于匹配带大小delete签名；测试不使用它。
 * @return 无。
 */
void operator delete(void *pointer, std::size_t) noexcept { std::free(pointer); }

/**
 * @brief 检查未取得有效State不可激活，再注入J1反馈验证活动态。
 *
 * 参数按J1掩码1配置，数值界限数组仍固定六元素；位置rad、速度rad/s、周期µs。硬件导出2个状态接口与1个位置命令接口。
 * @return 0 表示本文件全部检查通过；非零定位失败检查。
 */
int main() {
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
      "hardware_interface", "hardware_interface::SystemInterface");
  auto base = loader.createSharedInstance("dummy_ros_control/GuardedZenbeddedSystem");
  auto system = std::dynamic_pointer_cast<dummy_ros_control::GuardedZenbeddedSystem>(base);
  if (!system) return 7;
  hardware_interface::HardwareComponentParams params{};
  params.hardware_info.name = "dummy";
  params.hardware_info.type = "system";
  params.hardware_info.hardware_parameters = {
      {"endpoint", "tcp/127.0.0.1:17449"},
      {"device_id", "01000000000000000000000000000000"},
      {"client_instance_id", "02000000000000000000000000000000"},
      {"profile_id", "1"}, {"configured_mask", "1"},
      {"contract_sha256", "0101010101010101010101010101010101010101010101010101010101010101"},
      {"model_sha256", "0202020202020202020202020202020202020202020202020202020202020202"},
      {"profile_sha256", "0303030303030303030303030303030303030303030303030303030303030303"},
      {"firmware_sha256", "0404040404040404040404040404040404040404040404040404040404040404"},
      {"compiled_policy_sha256", "0505050505050505050505050505050505050505050505050505050505050505"},
      {"position_min_rad", "-1,-1,-1,-1,-1,-1"}, {"position_max_rad", "1,1,1,1,1,1"},
      {"velocity_max_rad_s", "1,1,1,1,1,1"}, {"acceleration_max_rad_s", "1,1,1,1,1,1"},
      {"control_period_us", "10000"}, {"state_publish_period_us", "10000"}};
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
  if (system->init(params) != hardware_interface::CallbackReturn::SUCCESS) return 1;
  /**
   * @brief 导出接口对象的生存期覆盖后续读写；其数量必须与J1的position/velocity反馈及position目标一致。
   */
  const auto state_interfaces = system->on_export_state_interfaces();
  const auto command_interfaces = system->on_export_command_interfaces();
  /**
   * @brief configure成功只表示worker配置/启动成功，不能替代有效反馈门槛。
   * 未激活read允许返回OK；缺少State的activate应失败，随后cleanup回收worker。
   */
  if (system->on_configure(rclcpp_lifecycle::State()) != hardware_interface::CallbackReturn::SUCCESS) return 8;
  if (system->read(rclcpp::Time(), rclcpp::Duration::from_nanoseconds(1000000)) !=
      hardware_interface::return_type::OK) return 15;
  if (system->on_activate(rclcpp_lifecycle::State()) != hardware_interface::CallbackReturn::FAILURE) return 10;
  if (system->on_cleanup(rclcpp_lifecycle::State()) != hardware_interface::CallbackReturn::SUCCESS) return 11;
  if (state_interfaces.size() != 2U || command_interfaces.size() != 1U) return 6;
  /**
   * @brief 注入0.25rad的人工J1实测缓存，Source时间100µs、检查时刻110µs、最大年龄20µs。
   * 该夹具使后续activate通过，不是从真实轴读到的位置。
   */
  dummy_ros_control::JointStateSnapshot snapshot{};
  snapshot.envelope = {7U, 1U, 1U, 100U, 1U, 1U, 1U};
  snapshot.position[0] = 0.25;
  snapshot.velocity[0] = 0.0;
  const dummy_ros_control::StateExpectation expected{7U, 1U, 0U, true, 1U, 90U, 110U, 20U};
  if (!system->ingest_state(snapshot, expected)) return 2;
  if (system->on_activate(rclcpp_lifecycle::State()) != hardware_interface::CallbackReturn::SUCCESS) return 3;
  if (system->read(rclcpp::Time(), rclcpp::Duration::from_nanoseconds(1000000)) !=
      hardware_interface::return_type::OK) return 9;
  if (system->write(rclcpp::Time(), rclcpp::Duration::from_nanoseconds(1000000)) !=
      hardware_interface::return_type::OK) return 4;
  dummy_ros_control::CommandGenerationRecord record{};
  if (system->take_publish_record(&record) || system->take_stop_request()) return 5;
  /**
   * @brief 此后恰好1000组read/write必须不触发普通new；返回错误同样使测试失败。
   * write不应无缘无故生成命令或Stop；此前已检查发布队列和Stop标志为空。
   */
  count_allocations = true;
  for (unsigned iteration = 0U; iteration < 1000U; ++iteration) {
    if (system->read(rclcpp::Time(), rclcpp::Duration::from_nanoseconds(1000000)) !=
            hardware_interface::return_type::OK ||
        system->write(rclcpp::Time(), rclcpp::Duration::from_nanoseconds(1000000)) !=
            hardware_interface::return_type::OK) return 12;
  }
  count_allocations = false;
  if (allocations != 0U) return 13;
  /**
   * @brief 退休旧motion身份不能撤销替换身份；invalidate_authority_state后活动态标志应保留。
   * 保留commands_active不等于向设备重新取得执行许可。
   */
  const dummy_ros_control::CommandIdentity retired{1U, 2U, 3U, 1U, 5U};
  const dummy_ros_control::CommandIdentity replacement{1U, 2U, 3U, 2U, 5U};
  dummy_ros_control::set_command_identity(replacement);
  system->invalidate_authority_state(retired);
  if (!system->commands_active()) return 14;
  return 0;
}

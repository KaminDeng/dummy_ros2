/**
 * @file production_transport_tests.cpp
 * @brief 以外部指定peer验证生产transport的未授权负向路径。
 *
 * 运行需要argv[1]端点，configure会启动生产通信worker；本轮只静态维护，不运行peer/真机。
 * 收到State/Status不等于被准入为可执行反馈；最终检查没有发布控制命令。Stop发送计数也不等于设备Idle确认、release committed或物理停止。
 */
#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>

#include "dummy_ros_control/device_transport.hpp"
#include "dummy_ros_control/guarded_zenbedded_system.hpp"

/**
 * @brief 先验证参数解析，再检查未授权State/命令被拒绝和cleanup发Stop。
 *
 * argv[1]是通信端点；等待反馈最多5s，等待已转移命令被拒最多2s（steady_clock）。配置仅J1，未使用位置以NaN填充。
 * @param argc 命令行参数个数（含程序名）。
 * @param argv 命令行字符串数组，具体位置见上述调用约定。
 * @return 0 表示本文件全部检查通过；非零定位失败检查。
 */
int main(int argc, char **argv) {
  if (argc != 2) return 10;
  using namespace dummy_ros_control;
  if (saturating_state_deadline(UINT64_MAX - 1U, 10U) != UINT64_MAX ||
      saturating_state_deadline(100U, 10U) != 110U) return 14;
  TransportParameters parameters{};
  /**
   * @brief 固定J1画像：位置rad、速度rad/s、加速度rad/s²，控制/State周期10,000µs。
   * 解析负例覆盖轴数与mask矛盾、缺端点、带尾缀数字和全零device_id。hash仅测试夹具。
   */
  const std::unordered_map<std::string, std::string> valid{
      {"endpoint", argv[1]},
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
  /**
   * @brief 接收计数至少各3条且出现拒绝记录，但copy_state必须仍失败。
   * 这条负向路径没有合法会话/授权，原始收到消息不能绕过反馈准入。
   */
  const auto feedback_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while ((statistics().state_messages < 3U || statistics().status_messages < 3U) &&
         std::chrono::steady_clock::now() < feedback_deadline) std::this_thread::sleep_for(std::chrono::milliseconds(10));
  if (statistics().state_messages < 3U || statistics().status_messages < 3U ||
      statistics().rejected_states == 0U) return 11;
  JointStateSnapshot snapshot{};
  if (system.copy_state(&snapshot)) return 8;
  if (system.read(rclcpp::Time(), rclcpp::Duration::from_nanoseconds(1000000)) !=
      hardware_interface::return_type::OK) return 16;
  /**
   * @brief 以任意本地身份尝试J1目标：即使成功转入worker队列，也必须在授权检查中被拒绝。
   * 这不是给真实设备的允许运动测试。
   */
  system.submit_control_update({1U, 2U, 3U, 4U, 5U}, {0.25, NAN, NAN, NAN, NAN, NAN});
  const bool transferred = system.transfer_control_update();
  const auto command_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (transferred && statistics().rejected_commands == 0U &&
         std::chrono::steady_clock::now() < command_deadline) std::this_thread::yield();
  if (transferred && statistics().rejected_commands == 0U) return 13;
  /**
   * @brief 错误生命周期置停机请求，cleanup等待worker收尾。
   * 只断言stop_requests_sent=1、发送失败=0、commands_published=0；不把发送成功升级为远端Stop完成。
   */
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

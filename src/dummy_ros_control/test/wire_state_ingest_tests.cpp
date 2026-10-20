/**
 * @file wire_state_ingest_tests.cpp
 * @brief 从wire State到受保护缓存验证解码、身份和有限值边界。
 *
 * 输入为当前schema合成State；不经过真实transport时钟映射。位置rad，速度rad/s；无效轴速度在缓存中用NaN表示不可用。
 */
#include <array>
#include <cmath>
#include <cstdint>

#include "dp_dummy_robot_v2.hpp"
#include "dummy_ros_control/guarded_zenbedded_system.hpp"

/**
 * @brief 检查合法State入库、坏包保留旧缓存和本地Stop标志。
 *
 * 截断、错误boot、过旧时间或标记有效但数值NaN应拒绝；失败后原位置/包络不能被半更新。J1配置允许其余未配置轴为NaN。
 * on_error/on_deactivate仅验证Stop标志可消费一次，不等于Stop请求已发出、远端确认或设备物理停止。
 * @return 0 表示本文件全部检查通过；非零定位失败检查。
 */
int main() {
  namespace wire = dp::protocol::dummy_robot_v2;
  wire::DummyJointStateV2 state;
  state.set_schema_major(2U);
  state.set_schema_minor(dp::protocol::dummy_robot_v2::kSchemaMinor);
  state.set_boot_epoch(9U);
  state.set_profile_id(7U);
  state.set_state_seq(101U);
  state.set_produced_at_us(1001U);
  state.set_configured_mask(0x3FU);
  state.set_position_valid_mask(0x3FU);
  state.set_velocity_valid_mask(0x01U);
  const double position[6]{0.1, 0.2, 0.7, 0.4, 0.5, 9.42477796076938};
  const double velocity[6]{0.01, 0.0, 0.0, 0.0, 0.0, 0.0};
  state.set_position_rad(position);
  state.set_estimated_velocity_rad_s(velocity);
  std::array<std::uint8_t, wire::DummyJointStateV2Codec::kEncodedSize> bytes{};
  if (wire::DummyJointStateV2Codec::encode(state, bytes.data(), bytes.size()) != wire::CodecResult::kOk)
    return 1;
  dummy_ros_control::GuardedZenbeddedSystem system;
  /**
   * @brief 预期boot9/profile7，要求全六轴位置且seq>100；请求锚1000µs、当前1100µs、最大年龄150µs。
   * 只有velocity位0有效，其他速度即使wire为0也应在缓存中标NaN，不能误作测得静止。
   */
  const dummy_ros_control::StateExpectation expected{9U, 7U, 100U, true, 0x3FU, 1000U, 1100U, 150U};
  if (!system.ingest_wire_state(bytes.data(), bytes.size(), expected)) return 2;
  dummy_ros_control::JointStateSnapshot snapshot{};
  if (!system.copy_state(&snapshot) || snapshot.position[5] != position[5] ||
      snapshot.velocity[0] != velocity[0] || !std::isnan(snapshot.velocity[1])) return 3;
  /**
   * @brief 同一合法字节分别以截断长度及错误boot期望送入，随后核对旧缓存仍完整。
   * 这段unchanged断言具体覆盖前两项拒绝；后续另检查超龄和有效位内NaN必须返回false。
   */
  if (system.ingest_wire_state(bytes.data(), bytes.size() - 1U, expected)) return 4;
  auto wrong = expected;
  wrong.boot = 10U;
  if (system.ingest_wire_state(bytes.data(), bytes.size(), wrong)) return 5;
  dummy_ros_control::JointStateSnapshot unchanged{};
  if (!system.copy_state(&unchanged) || unchanged.envelope.boot != 9U || unchanged.position[5] != position[5])
    return 6;
  auto stale = expected;
  stale.request_started_at_us = 0U;
  stale.now_us = 1200U;
  if (system.ingest_wire_state(bytes.data(), bytes.size(), stale)) return 7;
  /**
   * @brief velocity_valid_mask仍声明J1有效，所以J1速度NaN必须拒绝。
   * 未配置/无效位中的NaN与有效位中的NaN语义不同，不能一概当0或一概拒绝。
   */
  const double invalid_velocity[6]{NAN, 0.0, 0.0, 0.0, 0.0, 0.0};
  state.set_estimated_velocity_rad_s(invalid_velocity);
  if (wire::DummyJointStateV2Codec::encode(state, bytes.data(), bytes.size()) != wire::CodecResult::kOk)
    return 8;
  if (system.ingest_wire_state(bytes.data(), bytes.size(), expected)) return 9;
  if (!dummy_ros_control::GuardedZenbeddedSystem::active_joint_count_supported(1U) ||
      !dummy_ros_control::GuardedZenbeddedSystem::active_joint_count_supported(6U) ||
      dummy_ros_control::GuardedZenbeddedSystem::active_joint_count_supported(0U) ||
      dummy_ros_control::GuardedZenbeddedSystem::active_joint_count_supported(2U)) return 10;
  if (system.on_error(rclcpp_lifecycle::State()) != hardware_interface::CallbackReturn::SUCCESS ||
      !system.take_stop_request() || system.take_stop_request()) return 11;
  if (system.on_deactivate(rclcpp_lifecycle::State()) != hardware_interface::CallbackReturn::SUCCESS ||
      !system.take_stop_request() || system.take_stop_request()) return 13;
  /**
   * @brief J1画像mask=1只要求第0轴位置0.25rad；其余五轴NaN是未配置而非反馈损坏。
   * 此夹具直接走ingest_state，和上方wire解码路径一起限定允许的1/6轴形状。
   */
  dummy_ros_control::JointStateSnapshot j1{};
  j1.envelope = {7U, 1U, 1U, 100U, 1U, 1U, 0U};
  j1.position.fill(NAN);
  j1.position[0] = 0.25;
  const dummy_ros_control::StateExpectation j1_expected{7U, 1U, 0U, false, 1U, 90U, 110U, 20U};
  if (!system.ingest_state(j1, j1_expected)) return 12;
  return 0;
}

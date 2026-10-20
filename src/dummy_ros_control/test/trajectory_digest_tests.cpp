/**
 * @file trajectory_digest_tests.cpp
 * @brief 检查轨迹规范摘要的黄金向量、字段敏感性与非法输入。
 *
 * 只验证JointTrajectory规范化/哈希，不检查机器人限位、时间可执行性、碰撞、轨迹执行或硬件到位。
 */
#include "dummy_ros_control/guarded_joint_trajectory_controller.hpp"

#include <array>
#include <cmath>
#include <cstdio>
#include <stdexcept>

namespace {
/**
 * @brief 构造固定双轴黄金向量，包含header、位置/速度/effort及点时间，初始加速度数组为空。
 * @return 独立轨迹消息副本；stamp为42s+123ns，点time_from_start为2s+500ns。
 * @note JointTrajectory中的角位置/角速度等沿用rad/rad/s约定；effort单位由接口类型定义。
 */
trajectory_msgs::msg::JointTrajectory vector() {
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.header.stamp.sec = 42;
  trajectory.header.stamp.nanosec = 123;
  trajectory.header.frame_id = "base";
  trajectory.joint_names = {"J1", "J2"};
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = {1.0, -2.0};
  point.velocities = {0.5, -0.5};
  point.effort = {0.0, 3.0};
  point.time_from_start.sec = 2;
  point.time_from_start.nanosec = 500;
  trajectory.points.push_back(point);
  return trajectory;
}

/**
 * @brief 比较输入的规范SHA-256与预置32字节黄金摘要。
 * @param trajectory 待序列化轨迹，不在此函数中做物理可执行性验证。
 * @return 摘要逐字节相同为true；非法数值异常由调用层负例捕获。
 */
bool matches_golden(const trajectory_msgs::msg::JointTrajectory &trajectory) {
  constexpr std::array<std::uint8_t, 32> golden = {
      0xca, 0x08, 0x3d, 0x6d, 0x43, 0x41, 0xd3, 0xde,
      0x12, 0x15, 0x51, 0x59, 0xdc, 0x07, 0x55, 0x59,
      0x3f, 0x65, 0x69, 0xca, 0x76, 0xad, 0xd9, 0x01,
      0x5c, 0x22, 0x7b, 0x07, 0x0c, 0x3e, 0x27, 0x1c};
  return dummy_ros_control::trajectory_digest(trajectory) == golden;
}
}  // namespace

/**
 * @brief 核对-0规范化、各字段变化和NaN/纳秒越界异常。
 *
 * 每次从新黄金向量改一项，避免前一变更影响后一结论；关节名顺序、header和可选数组均进入摘要。
 * @return 0 表示本文件全部检查通过；非零定位失败检查。
 */
int main() {
  auto trajectory = vector();
  if (!matches_golden(trajectory)) return 1;
  trajectory.points[0].effort[0] = -0.0;
  if (!matches_golden(trajectory)) return 2;
  trajectory = vector();
  trajectory.header.stamp.sec++;
  if (matches_golden(trajectory)) return 3;
  trajectory = vector();
  trajectory.header.stamp.nanosec++;
  if (matches_golden(trajectory)) return 4;
  trajectory = vector();
  trajectory.header.frame_id = "tool";
  if (matches_golden(trajectory)) return 5;
  trajectory = vector();
  trajectory.joint_names.push_back("J3");
  if (matches_golden(trajectory)) return 6;
  trajectory = vector();
  trajectory.points.push_back(trajectory.points[0]);
  if (matches_golden(trajectory)) return 7;
  trajectory = vector();
  trajectory.points[0].positions[0] += 0.25;
  if (matches_golden(trajectory)) return 8;
  trajectory = vector();
  trajectory.points[0].velocities[0] += 0.25;
  if (matches_golden(trajectory)) return 9;
  trajectory = vector();
  trajectory.points[0].accelerations.push_back(0.1);
  if (matches_golden(trajectory)) return 10;
  trajectory = vector();
  trajectory.points[0].effort[0] = 0.1;
  if (matches_golden(trajectory)) return 11;
  trajectory = vector();
  trajectory.points[0].time_from_start.sec++;
  if (matches_golden(trajectory)) return 12;
  trajectory = vector();
  trajectory.points[0].time_from_start.nanosec++;
  if (matches_golden(trajectory)) return 13;
  trajectory = vector();
  /**
   * @brief 非有限浮点数必须抛invalid_argument；不能把NaN字节平台差异写进规范摘要。
   * 后一个负例独立检查nanosec必须小于10^9。
   */
  trajectory.points[0].effort[0] = NAN;
  try { (void)dummy_ros_control::trajectory_digest(trajectory); return 14; }
  catch (const std::invalid_argument &) {}
  trajectory = vector();
  trajectory.header.stamp.nanosec = 1000000000U;
  try { (void)dummy_ros_control::trajectory_digest(trajectory); return 15; }
  catch (const std::invalid_argument &) {}
  return 0;
}

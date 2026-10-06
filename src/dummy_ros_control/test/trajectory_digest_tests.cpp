#include "dummy_ros_control/guarded_joint_trajectory_controller.hpp"

#include <array>
#include <cmath>
#include <cstdio>
#include <stdexcept>

namespace {
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

bool matches_golden(const trajectory_msgs::msg::JointTrajectory &trajectory) {
  constexpr std::array<std::uint8_t, 32> golden = {
      0xca, 0x08, 0x3d, 0x6d, 0x43, 0x41, 0xd3, 0xde,
      0x12, 0x15, 0x51, 0x59, 0xdc, 0x07, 0x55, 0x59,
      0x3f, 0x65, 0x69, 0xca, 0x76, 0xad, 0xd9, 0x01,
      0x5c, 0x22, 0x7b, 0x07, 0x0c, 0x3e, 0x27, 0x1c};
  return dummy_ros_control::trajectory_digest(trajectory) == golden;
}
}  // namespace

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
  trajectory.points[0].effort[0] = NAN;
  try { (void)dummy_ros_control::trajectory_digest(trajectory); return 14; }
  catch (const std::invalid_argument &) {}
  trajectory = vector();
  trajectory.header.stamp.nanosec = 1000000000U;
  try { (void)dummy_ros_control::trajectory_digest(trajectory); return 15; }
  catch (const std::invalid_argument &) {}
  return 0;
}

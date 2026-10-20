/**
 * @file device_goal_validation_tests.cpp
 * @brief 核设备Gate可承诺的轨迹与容差形状边界。
 *
 * 纯消息转换/校验；不规划碰撞、不建ROS节点、不提供实机许可。位置rad，时间秒，末速rad/s。
 */
#include "dummy_motion_client/native_goal_validation.hpp"
#include <limits>
using namespace dummy_motion_client;
/**
 * @brief 接受合法J1，拒绝NaN/不支持导数容差/非零header及非静止末速。
 * @return 0表示所选检查通过；非零或断言/require异常表示失败。
 * 末速4e−19rad/s作为数值近零可接受，1e−4拒绝；路径velocity tolerance0.1不能被静默接受，因为设备Gate不承诺该导数容差。
 */
int main() {
  JointProfile profile{{"Joint1"},{-1},{1},{2},{4},{0.15},{0.1}};
  NativeGoal goal; goal.trajectory.joint_names={"Joint1"};
  trajectory_msgs::msg::JointTrajectoryPoint point; point.positions={0.1}; point.time_from_start.sec=1;
  goal.trajectory.points={point};
  if(!validated_device_trajectory(goal,profile)) return 1;
  goal.trajectory.points[0].positions[0]=std::numeric_limits<double>::quiet_NaN();
  if(validated_device_trajectory(goal,profile)) return 2;
  goal.trajectory.points[0].positions[0]=0.1;
  control_msgs::msg::JointTolerance tolerance; tolerance.name="Joint1"; tolerance.velocity=0.1;
  goal.path_tolerance={tolerance};
  // Device gate cannot silently accept derivative tolerances it does not enforce.
  if(validated_device_trajectory(goal,profile)) return 3;
  goal.path_tolerance.clear(); goal.trajectory.header.stamp.sec=1;
  if(validated_device_trajectory(goal,profile)) return 4;
  goal.trajectory.header.stamp.sec=0;
  goal.trajectory.points[0].velocities={4e-19};
  if(!validated_device_trajectory(goal,profile)) return 5;
  goal.trajectory.points[0].velocities={1e-4};
  if(validated_device_trajectory(goal,profile)) return 6;
}

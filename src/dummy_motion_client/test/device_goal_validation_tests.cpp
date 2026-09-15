#include "dummy_motion_client/native_goal_validation.hpp"
#include <limits>
using namespace dummy_motion_client;
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
}

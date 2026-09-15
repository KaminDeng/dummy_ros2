#include "dummy_motion_client/native_goal_validation.hpp"
#include <iostream>
#include <limits>
int main() {
  using namespace dummy_motion_client;
  NativeGoal goal;
  goal.trajectory.joint_names={"Joint1","Joint2","Joint3","Joint4","Joint5","Joint6"};
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions={0,0,1.57,0,0,0}; point.time_from_start.sec=1;
  goal.trajectory.points={point};
  if (!valid_native_goal(goal)) return 1;
  auto invalid=goal; invalid.trajectory.joint_names.pop_back();
  const bool partial=!valid_native_goal(invalid); if (!partial) return 2;
  invalid=goal; invalid.trajectory.points[0].positions[0]=std::numeric_limits<double>::quiet_NaN();
  const bool nonfinite=!valid_native_goal(invalid); if (!nonfinite) return 3;
  invalid=goal; invalid.trajectory.points[0].positions[0]=100;
  const bool bounds=!valid_native_goal(invalid); if (!bounds) return 4;
  invalid=goal; invalid.trajectory.points[0].velocities={0.1,0,0,0,0,0};
  const bool velocity=!valid_native_goal(invalid); if (!velocity) return 5;
  invalid=goal; control_msgs::msg::JointTolerance tolerance;
  tolerance.name="Joint1"; tolerance.position=-1; invalid.path_tolerance={tolerance};
  const bool weakened=!valid_native_goal(invalid); if (!weakened) return 6;
  invalid=goal; invalid.trajectory.points[0].time_from_start.nanosec=1000000000;
  const bool bad_time=!valid_native_goal(invalid); if (!bad_time) return 7;
  Trajectory finite_time{{"Joint1"},{{{0},{},{},std::numeric_limits<double>::quiet_NaN()}},{},{},0};
  const JointProfile profile{{"Joint1"},{-1},{1},{1},{1},{0.1},{0.1}};
  const bool nan_time=!validate_trajectory(finite_time,profile); if (!nan_time) return 8;
  auto strict=goal; tolerance.position=0.005; strict.goal_tolerance={tolerance};
  tolerance.position=0.03; strict.path_tolerance={tolerance};
  strict.goal_time_tolerance.sec=2; strict.goal_time_tolerance.nanosec=300000000;
  const auto converted=validated_native_trajectory(strict);
  if (!converted || converted->goal_tolerance[0]!=0.005 || converted->path_tolerance[0]!=0.03 ||
      std::abs(converted->goal_time_tolerance-2.3)>1e-9) return 9;
  std::cout << "{\"partial_joints_rejected\":" << partial << ",\"nan_position_rejected\":" << nonfinite
    << ",\"bounds_rejected\":" << bounds << ",\"terminal_velocity_rejected\":" << velocity
    << ",\"minus_one_tolerance_rejected\":" << weakened << ",\"malformed_duration_rejected\":" << bad_time
    << ",\"nan_time_rejected\":" << nan_time << "}\n";
}

#include "dummy_motion_client/native_goal_validation.hpp"
#include <algorithm>
#include <cmath>
#include <unordered_set>
namespace dummy_motion_client {
namespace {
bool duration_valid(const builtin_interfaces::msg::Duration &duration) {
  return duration.sec>=0 && duration.nanosec<1000000000U;
}
bool tolerances_valid(const std::vector<control_msgs::msg::JointTolerance> &values,
                      const std::vector<std::string> &names, double position_limit) {
  std::unordered_set<std::string> seen;
  for (const auto &value:values) {
    if (std::find(names.begin(),names.end(),value.name)==names.end() || !seen.insert(value.name).second ||
        !std::isfinite(value.position) || !std::isfinite(value.velocity) || !std::isfinite(value.acceleration) ||
        value.position<0 || value.position>position_limit || value.velocity<0 || value.acceleration<0)
      return false;
  }
  return true;
}
}
std::optional<Trajectory> validated_native_trajectory(const NativeGoal &goal) {
  const JointProfile profile{
    {"Joint1","Joint2","Joint3","Joint4","Joint5","Joint6"},
    {-2.9670597283903604,-1.2740903539558606,0.6108652381980153,-3.141592653589793,-2.0943951023931953,-12.566370614359172},
    {2.9670597283903604,1.5707963267948966,3.141592653589793,3.141592653589793,2.0943951023931953,12.566370614359172},
    {2,2,2,2,2,2},{4,4,4,4,4,4},{0.15,0.15,0.15,0.15,0.15,0.15},{0.1,0.1,0.1,0.1,0.1,0.1}};
  if (!duration_valid(goal.goal_time_tolerance) || !goal.multi_dof_trajectory.points.empty() ||
      !goal.component_path_tolerance.empty() || !goal.component_goal_tolerance.empty() ||
      !tolerances_valid(goal.path_tolerance,profile.joints,0.15) ||
      !tolerances_valid(goal.goal_tolerance,profile.joints,0.1)) return {};
  Trajectory trajectory; trajectory.joints=goal.trajectory.joint_names;
  trajectory.goal_time_tolerance=static_cast<double>(goal.goal_time_tolerance.sec)+
      goal.goal_time_tolerance.nanosec*1e-9;
  for (const auto &name:trajectory.joints) {
    const auto path=std::find_if(goal.path_tolerance.begin(),goal.path_tolerance.end(),
        [&name](const auto &value) { return value.name==name; });
    const auto terminal=std::find_if(goal.goal_tolerance.begin(),goal.goal_tolerance.end(),
        [&name](const auto &value) { return value.name==name; });
    trajectory.path_tolerance.push_back(path==goal.path_tolerance.end()?0:path->position);
    trajectory.goal_tolerance.push_back(terminal==goal.goal_tolerance.end()?0:terminal->position);
  }
  for (const auto &point:goal.trajectory.points) {
    if (!duration_valid(point.time_from_start) || !point.effort.empty()) return {};
    trajectory.points.push_back({point.positions,point.velocities,point.accelerations,
        static_cast<double>(point.time_from_start.sec)+point.time_from_start.nanosec*1e-9});
  }
  return validate_trajectory(trajectory,profile);
}
bool valid_native_goal(const NativeGoal &goal) { return validated_native_trajectory(goal).has_value(); }
}  // namespace dummy_motion_client

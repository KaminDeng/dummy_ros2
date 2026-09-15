#pragma once
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "dummy_motion_client/action_gate.hpp"
namespace dummy_motion_client {
using NativeGoal=control_msgs::action::FollowJointTrajectory::Goal;
bool valid_native_goal(const NativeGoal &);
std::optional<Trajectory> validated_native_trajectory(const NativeGoal &);
std::optional<Trajectory> validated_device_trajectory(const NativeGoal &, const JointProfile &);
}  // namespace dummy_motion_client

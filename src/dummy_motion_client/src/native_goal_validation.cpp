/**
 * @file native_goal_validation.cpp
 * @brief 实现ROS FollowJointTrajectory支持字段检查与内部轨迹转换。
 * 位置rad、速度rad/s、加速度rad/s²，Duration转相对秒。验证不做碰撞规划，不比较实时实物姿态，也不发控制命令；Native固定profile和设备明确profile的支持语义不同，不能将返回有值解释为已获设备许可。
 */
#include "dummy_motion_client/native_goal_validation.hpp"
#include <algorithm>
#include <cmath>
#include <unordered_set>
namespace dummy_motion_client {
namespace {
/**
 * @brief 核对ROS Duration的非负秒与规范纳秒域。
 * @param duration 输入Duration；此处不把它当绝对墙钟。
 * @return sec≥0且nanosec<10^9为true；零时长在本层允许，轨迹递增检查由下层完成。
 */
bool duration_valid(const builtin_interfaces::msg::Duration &duration) {
  return duration.sec>=0 && duration.nanosec<1000000000U;
}
/**
 * @brief 检查容差轴名唯一/存在及有限非负数值。
 * 没有-1哨兵语义，负值直接拒绝；位置0留给profile默认映射，设备入口另要求速度/加速度容差为0。
 * @param values JointTolerance数组，位置rad、速度rad/s、加速度rad/s²。
 * @param names 允许关节名集合。
 * @param position_limit 位置容差最大值rad。
 * @return bool；只为position设上限，velocity/acceleration在此仅要求有限非负。
 */
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
/**
 * @brief 转换受支持Goal字段并交通用profile轨迹验证。
 * 禁止multi_dof路点、component容差及effort。只复制位置容差，速度/加速度容差虽经数值检查但不进入内部Trajectory；头stamp也不在此处理，设备入口另加即时语义限制。完整路点维度/有限性/时间递增/范围/终速由validate_trajectory负责，不证明避碰或全部插值动态。
 * @param goal 原生Goal，完整轴名和路点。
 * @param profile 明确轴序、范围、动态和容差限制。
 * @return 规范化Trajectory或nullopt，原Goal不修改。
 */
std::optional<Trajectory> convert_goal(const NativeGoal &goal,const JointProfile &profile) {
  if (!duration_valid(goal.goal_time_tolerance) || !goal.multi_dof_trajectory.points.empty() ||
      !goal.component_path_tolerance.empty() || !goal.component_goal_tolerance.empty() ||
      !tolerances_valid(goal.path_tolerance,profile.joints,0.15) ||
      !tolerances_valid(goal.goal_tolerance,profile.joints,0.1)) return {};
  /**
   * @brief Goal相对时间从sec+nanosec×1e-9转为double秒，绝不与宿主steady µs或设备produced epoch直接混算。
   */
  Trajectory trajectory; trajectory.joints=goal.trajectory.joint_names;
  trajectory.goal_time_tolerance=static_cast<double>(goal.goal_time_tolerance.sec)+
      goal.goal_time_tolerance.nanosec*1e-9;
  /**
   * @brief 按输入轨迹轴名查位置容差，缺失先写0；下层按profile轴序重排并把0替换为默认，不把0解释为必须数学精确零误差。
   */
  for (const auto &name:trajectory.joints) {
    const auto path=std::find_if(goal.path_tolerance.begin(),goal.path_tolerance.end(),
        [&name](const auto &value) { return value.name==name; });
    const auto terminal=std::find_if(goal.goal_tolerance.begin(),goal.goal_tolerance.end(),
        [&name](const auto &value) { return value.name==name; });
    trajectory.path_tolerance.push_back(path==goal.path_tolerance.end()?0:path->position);
    trajectory.goal_tolerance.push_back(terminal==goal.goal_tolerance.end()?0:terminal->position);
  }
  /**
   * @brief 保留所有原路点及可选速度/加速度，不只取末点；effort不支持且不能静默丢弃。
   */
  for (const auto &point:goal.trajectory.points) {
    if (!duration_valid(point.time_from_start) || !point.effort.empty()) return {};
    trajectory.points.push_back({point.positions,point.velocities,point.accelerations,
        static_cast<double>(point.time_from_start.sec)+point.time_from_start.nanosec*1e-9});
  }
  return validate_trajectory(trajectory,profile);
}
/**
 * @brief 对明确设备profile只接受当前实现支持的即时位置轨迹语义。
 * header stamp必须全0，拒绝未实现的定时起步；path/goal速度与加速度容差必须0，避免接纳却不执行这些跟踪要求。其余转换/范围检查复用convert_goal，无碰撞或设备状态证明。
 * @param goal 设备入口原生Goal。
 * @param profile 已绑定设备profile与工作范围。
 * @return 规范化Trajectory或nullopt。
 */
std::optional<Trajectory> validated_device_trajectory(const NativeGoal &goal,const JointProfile &profile) {
  if(goal.trajectory.header.stamp.sec!=0 || goal.trajectory.header.stamp.nanosec!=0) return {};
  for(const auto &value:goal.path_tolerance) if(value.velocity!=0 || value.acceleration!=0) return {};
  for(const auto &value:goal.goal_tolerance) if(value.velocity!=0 || value.acceleration!=0) return {};
  return convert_goal(goal,profile);
}
/**
 * @brief 按固定Native六轴限制校验并转换Goal。
 * Joint1..6固定原弧度范围，速度2rad/s、加速度4rad/s²、path0.15rad和goal0.1rad。此分支不额外拒绝非零头stamp；非零速度/加速度容差只经有限/非负检查但不会保存，不能当支持全部FJT语义。
 * @param goal 输入Goal。
 * @return Trajectory或nullopt。
 */
std::optional<Trajectory> validated_native_trajectory(const NativeGoal &goal) {
  /**
   * @brief 这份固定Native profile只属于此入口；真实设备入口必须使用已加载且身份绑定的profile，不能借默认值扩大实机工作域。
   */
  const JointProfile profile{
    {"Joint1","Joint2","Joint3","Joint4","Joint5","Joint6"},
    {-2.9670597283903604,-1.2740903539558606,0.6108652381980153,-3.141592653589793,-2.0943951023931953,-12.566370614359172},
    {2.9670597283903604,1.5707963267948966,3.141592653589793,3.141592653589793,2.0943951023931953,12.566370614359172},
    {2,2,2,2,2,2},{4,4,4,4,4,4},{0.15,0.15,0.15,0.15,0.15,0.15},{0.1,0.1,0.1,0.1,0.1,0.1}};
  return convert_goal(goal,profile);
}
/**
 * @brief 用Native规范转换是否成功实现布尔验证。
 * @param goal 待验证原生Goal。
 * @return 等价validated_native_trajectory(goal).has_value()，不执行运动。
 */
bool valid_native_goal(const NativeGoal &goal) { return validated_native_trajectory(goal).has_value(); }
}  // namespace dummy_motion_client

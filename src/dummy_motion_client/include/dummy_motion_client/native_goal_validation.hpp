/**
 * @file native_goal_validation.hpp
 * @brief 声明ROS FollowJointTrajectory到内部轨迹的有限输入验证。
 * 只做消息形状、支持字段、单位和profile动态/范围校验，不请求MoveIt避碰，也不发送轨迹。Native默认profile与明确设备profile是两个入口，验证返回不能代替设备授权、场景有效性或运行结果。
 */
#pragma once
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "dummy_motion_client/action_gate.hpp"
namespace dummy_motion_client {
/**
 * @brief 原生control_msgs FollowJointTrajectory::Goal别名；内部Trajectory保位置rad、速度rad/s、加速度rad/s²和相对秒。
 */
using NativeGoal=control_msgs::action::FollowJointTrajectory::Goal;
/**
 * @brief 用内置六轴Native profile判断Goal可转换。
 * 匿名NativeGoal为输入；不启动动作，不检查实时实物状态。
 * @return bool，等价validated_native_trajectory有值。
 */
bool valid_native_goal(const NativeGoal &);
/**
 * @brief 校验支持的原生字段并按固定六轴Native profile规范化。
 * 匿名NativeGoal输入不允许multi_dof点、component容差、effort；duration秒非负且纳秒<10^9。位置容差需非负且path≤0.15rad、goal≤0.1rad，按Joint1..6默认范围及2rad/s、4rad/s²验证；此入口不额外要求头时间戳为0；只保留位置容差，速度/加速度容差虽做有限/非负检查但不进入内部Trajectory，设备入口则严格要求它们为0。
 * @return 内部完整Trajectory或nullopt。
 */
std::optional<Trajectory> validated_native_trajectory(const NativeGoal &);
/**
 * @brief 在通用转换检查外限制设备入口只支持即时位置轨迹语义。
 * 匿名参数依次NativeGoal和JointProfile。header stamp必须sec/nanosec全0；path/goal的速度与加速度容差必须0，不接受未实现的跟踪语义。通用转换仍禁止multi_dof、component容差和effort，并校验完整路点/终速；不含避碰。
 * @return 按调用方profile重排/校验的Trajectory或nullopt。
 */
std::optional<Trajectory> validated_device_trajectory(const NativeGoal &, const JointProfile &);
}  // namespace dummy_motion_client

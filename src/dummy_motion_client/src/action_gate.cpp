/**
 * @file action_gate.cpp
 * @brief 把管理事务、可信State与真实控制update合成单目标Action阶段。
 * 本对象借用client/port，由外层控制executor串行推进；now/deadline是宿主steady微秒，
 * 轨迹时间是相对秒、位置rad、速度rad/s、加速度rad/s²。Ready/本地入队/内部JTC成功
 * 均不等于设备使能、CAN SENT或机械到位；源设备时钟和State新鲜度须由上层transport先验证。
 */
#include "dummy_motion_client/action_gate.hpp"
#include "dummy_motion_client/deadline.hpp"
#include "dummy_ros_control/diagnostic_scope.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <unordered_set>
namespace dummy_motion_client {
namespace {
/**
 * @brief 显式终点速度的零容差1e-9 rad/s；未提供速度的纯位置路径另按分段差分检查。
 */
constexpr double kTerminalVelocityZeroToleranceRadS=1e-9;
/**
 * @brief 按32位串行算术判断严格后继。
 * @param candidate 候选线序号。
 * @param current 已接纳序号。
 * @return 模差非零且小于2³¹为true；不用于宿主64位generation。
 */
bool sequence_newer(std::uint32_t candidate,std::uint32_t current) {
  const auto distance=static_cast<std::uint32_t>(candidate-current);
  return distance!=0U && distance<0x80000000U;
}
/**
 * @brief 检查同一或后继线序号，允许相等。
 * @param candidate 当前观测commit。
 * @param current 需要覆盖的水位。
 * @return 模差小于2³¹时true，要求比较跨度在半周期内。
 */
bool sequence_at_or_after(std::uint32_t candidate,std::uint32_t current) {
  return static_cast<std::uint32_t>(candidate-current)<0x80000000U;
}
/**
 * @brief 检查一个浮点向量的所有现有元素有限，空向量按all_of语义为true。
 * @param values 位置/速度/加速度或容差，单位由调用处决定。
 * @return 无NaN/Inf时true；不在此检查长度或数值范围。
 */
bool finite(const std::vector<double> &values) {
  return std::all_of(values.begin(),values.end(),[](double v) { return std::isfinite(v); });
}
/**
 * @brief 预验与运行采样共用的边界退入检查；起点限于名义范围和既有完成容差带。
 * @param origin 原始起点；previous 是上一个已接受的位置，均不裁剪到工作边界。
 * @param value 本次待验位置；一旦 previous 已进入工作区，不允许再次越界。
 * @return 工作区内位置或同侧单调向内的位置为 true；不授予运动权限。
 */
bool inward_boundary_step(const JointProfile& profile,std::size_t axis,
                          double origin,double previous,double value) {
  if(value>=profile.lower[axis] && value<=profile.upper[axis])return true;
  if(!profile.manual_inward_start || profile.nominal_lower.size()!=profile.joints.size() ||
     profile.nominal_upper.size()!=profile.joints.size() || !std::isfinite(origin) ||
     !std::isfinite(previous) || !std::isfinite(value) ||
     origin<profile.nominal_lower[axis] || origin>profile.nominal_upper[axis] ||
     value<profile.nominal_lower[axis] || value>profile.nominal_upper[axis])return false;
  if(origin>profile.upper[axis])
    return origin-profile.upper[axis]<=profile.goal_tolerance[axis] &&
      previous>profile.upper[axis] && value>profile.upper[axis] && value<=previous;
  if(origin<profile.lower[axis])
    return profile.lower[axis]-origin<=profile.goal_tolerance[axis] &&
      previous<profile.lower[axis] && value<profile.lower[axis] && value>=previous;
  return false;
}
/**
 * @brief 校验用户容差形状和最大值，零值后续映射到 profile 默认。
 * @param values 可为空的容差；limits 为同轴序上限。
 * @param allow_unspecified_limit 为 true 时，路径默认零不设置附加 Host 上限。
 */
bool tolerance(const std::vector<double> &values, const std::vector<double> &limits,
               bool allow_unspecified_limit = false) {
  if (values.empty()) return true;
  if (values.size()!=limits.size() || !finite(values)) return false;
  for (std::size_t i=0;i<values.size();++i)
    if (values[i]<0 || ((!allow_unspecified_limit || limits[i]>0) && values[i]>limits[i])) return false;
  return true;
}
}
/**
 * @brief 复制、按profile轴序重排并检查全部路点和容差。
 * @param input 用户轨迹，路点时间为相对秒，位置rad、速度rad/s、加速度rad/s²。
 * @param profile 1..6轴完整范围/动态/默认容差，轴名必须与输入唯一集合一致。
 * @param measured_start 可空六轴实测起点，非空时将纯位置轨迹的首段也纳入差分校验。
 * @return 规范化副本或nullopt，不修改输入；分配失败仍可能抛异常。
 * @note 校验路点及纯位置段差分，不做避碰，也不证明JTC完整插值曲线所有时刻满足动态上限。
 */
std::optional<Trajectory> validate_trajectory(const Trajectory &input, const JointProfile &profile,
                                              const Position *measured_start) {
  const auto count=profile.joints.size();
  if (count==0 || count>6 || input.joints.size()!=count || input.points.empty() ||
      profile.lower.size()!=count || profile.upper.size()!=count || profile.velocity.size()!=count ||
      profile.acceleration.size()!=count || profile.path_tolerance.size()!=count || profile.goal_tolerance.size()!=count ||
      !finite(profile.lower) || !finite(profile.upper) || !finite(profile.velocity) ||
      !finite(profile.acceleration) || !finite(profile.path_tolerance) || !finite(profile.goal_tolerance) ||
      !std::isfinite(input.goal_time_tolerance) || input.goal_time_tolerance<0) return {};
  const std::unordered_set<std::string> names(input.joints.begin(),input.joints.end());
  if (names.size()!=count || std::unordered_set<std::string>(profile.joints.begin(),profile.joints.end())!=names)
    return {};
  auto output=input; output.joints=profile.joints;
  std::vector<std::size_t> order;
  for (const auto &name:profile.joints)
    order.push_back(std::find(input.joints.begin(),input.joints.end(),name)-input.joints.begin());
  /**
   * @brief 将一个已检查维度的输入向量按关节名映射到profile轴序。
   * @param values 输入轴序的数值向量。
   * @return 重排后的独立向量，发生在非实时目标准入路径。
   */
  auto reorder=[&order](const std::vector<double> &values) {
    std::vector<double> result; for (auto index:order) result.push_back(values[index]); return result;
  };
  if ((!input.path_tolerance.empty() && input.path_tolerance.size()!=count) ||
      (!input.goal_tolerance.empty() && input.goal_tolerance.size()!=count)) return {};
  if (!input.path_tolerance.empty()) output.path_tolerance=reorder(input.path_tolerance);
  if (!input.goal_tolerance.empty()) output.goal_tolerance=reorder(input.goal_tolerance);
  if (!tolerance(output.path_tolerance,profile.path_tolerance,true) ||
      !tolerance(output.goal_tolerance,profile.goal_tolerance)) return {};
  if (output.path_tolerance.empty()) output.path_tolerance=profile.path_tolerance;
  if (output.goal_tolerance.empty()) output.goal_tolerance=profile.goal_tolerance;
  for (std::size_t i=0;i<count;++i) {
    if (output.path_tolerance[i]==0) output.path_tolerance[i]=profile.path_tolerance[i];
    if (output.goal_tolerance[i]==0) output.goal_tolerance[i]=profile.goal_tolerance[i];
  }
  /**
   * @brief 首点允许t=0，后续时间必须严格递增；每点先重排再核本轴位置及显式速度/加速度。
   */
  double previous=-1;
  for (auto &point:output.points) {
    if (point.positions.size()!=count || (!point.velocities.empty() && point.velocities.size()!=count) ||
        (!point.accelerations.empty() && point.accelerations.size()!=count) || !finite(point.positions) ||
        !finite(point.velocities) || !finite(point.accelerations) || !std::isfinite(point.time) ||
        point.time<0 || point.time<=previous) return {};
    previous=point.time; point.positions=reorder(point.positions);
    if (!point.velocities.empty()) point.velocities=reorder(point.velocities);
    if (!point.accelerations.empty()) point.accelerations=reorder(point.accelerations);
    for (std::size_t i=0;i<count;++i) {
      if (profile.lower[i]>profile.upper[i] || profile.velocity[i]<=0 || profile.acceleration[i]<=0 ||
          profile.path_tolerance[i]<0 || profile.goal_tolerance[i]<=0 ||
          (!point.velocities.empty() && std::abs(point.velocities[i])>profile.velocity[i]) ||
          (!point.accelerations.empty() && std::abs(point.accelerations[i])>profile.acceleration[i])) return {};
    }
  }
  for (double velocity:output.points.back().velocities)
    if (std::abs(velocity)>kTerminalVelocityZeroToleranceRadS) return {};
  /**
   * @brief 只有所有点都未提供速度和加速度才走纯位置差分门；混合显式字段不在此做同样的整段差分证明。
   */
  const bool position_only=std::all_of(output.points.begin(),output.points.end(),[](const auto &point) {
    return point.velocities.empty() && point.accelerations.empty();
  });
  // 仅显式手动策略可保留工作区外、名义范围内的真实起点；每轴一旦进入工作区不得再离开。
  if(profile.manual_inward_start &&
     (profile.nominal_lower.size()!=count || profile.nominal_upper.size()!=count ||
      !finite(profile.nominal_lower) || !finite(profile.nominal_upper)))return {};
  for(std::size_t axis=0;axis<count;++axis) {
    if(profile.manual_inward_start &&
       (profile.nominal_lower[axis]>profile.lower[axis] ||
        profile.nominal_upper[axis]<profile.upper[axis]))return {};
    const double origin=measured_start ? (*measured_start)[axis] : output.points.front().positions[axis];
    if(!std::isfinite(origin))return {};
    int side=origin<profile.lower[axis] ? -1 : origin>profile.upper[axis] ? 1 : 0;
    if(side && (!profile.manual_inward_start || !position_only ||
       origin<profile.nominal_lower[axis] || origin>profile.nominal_upper[axis]))return {};
    // 与设备既有完成容差带一致；名义范围并非任意越过工作边界的许可。
    if(side && (side<0 ? profile.lower[axis]-origin : origin-profile.upper[axis])>
       profile.goal_tolerance[axis])return {};
    double previous=origin;
    if(measured_start && side) {
      const double first=output.points.front().positions[axis];
      const bool first_outside=first<profile.lower[axis] || first>profile.upper[axis];
      if(first_outside) {
        // 对照Core首采样连续性例外：保留新鲜测量与原首点的微小差异，后续仍从原首点严格向内。
        if(std::abs(first-origin)>profile.goal_tolerance[axis] ||
           (side<0 ? first>=profile.lower[axis] : first<=profile.upper[axis]) ||
           first<profile.nominal_lower[axis] || first>profile.nominal_upper[axis] ||
           (side<0 ? profile.lower[axis]-first : first-profile.upper[axis])>
             profile.goal_tolerance[axis])return {};
        previous=first;
      }
    }
    for(const auto& point:output.points) {
      const double value=point.positions[axis];
      if(value>=profile.lower[axis] && value<=profile.upper[axis])side=0;
      else if(!side || !inward_boundary_step(profile,axis,origin,previous,value))return {};
      previous=value;
    }
    if(side)return {};
  }
  if (position_only) {
    std::vector<double> previous_position;
    double previous_time=0, previous_duration=0;
    std::vector<double> previous_velocity;
    if (measured_start) {
      previous_position.assign(measured_start->begin(),measured_start->begin()+count);
      // 实际Hold的命令速度为零；首槽也须证明从Hold到首点的加速度。
      previous_velocity.assign(count,0.0);
      previous_duration=output.points.front().time;
      for (std::size_t i=0;i<count;++i)
        if (!std::isfinite(previous_position[i])) return {};
    }
    /**
     * @brief 有实测起点时从t=0到第0点也须有正时长且起点在profile内；无起点时以第0点为已知锚。
     * 相邻段加速度按速度差/两段较短时长检查；实测Hold入口首段从已知静止命令速度0开始。
     */
    const std::size_t first_segment=measured_start ? 0 : 1;
    if (!measured_start) {
      previous_position=output.points.front().positions;
      previous_time=output.points.front().time;
    }
    for (std::size_t point_index=first_segment;point_index<output.points.size();++point_index) {
      const auto &point=output.points[point_index];
      const double duration=point.time-previous_time;
      if (duration<=0) return {};
      std::vector<double> velocity(count);
      for (std::size_t i=0;i<count;++i) {
        velocity[i]=(point.positions[i]-previous_position[i])/duration;
        if (std::abs(velocity[i])>profile.velocity[i] ||
            (!previous_velocity.empty() &&
             std::abs(velocity[i]-previous_velocity[i])/std::min(duration,previous_duration)>profile.acceleration[i]))
          return {};
      }
      previous_position=point.positions; previous_time=point.time;
      previous_duration=duration; previous_velocity=std::move(velocity);
    }
  }
  return output;
}
bool PreparedTrajectory::matches(const JointProfile& p) const noexcept {
 return p.joints==profile_.joints && p.lower==profile_.lower && p.upper==profile_.upper &&
  p.velocity==profile_.velocity && p.acceleration==profile_.acceleration &&
  p.path_tolerance==profile_.path_tolerance && p.goal_tolerance==profile_.goal_tolerance &&
  p.manual_inward_start==profile_.manual_inward_start && p.nominal_lower==profile_.nominal_lower && p.nominal_upper==profile_.nominal_upper;
}
std::shared_ptr<const PreparedTrajectory> prepare_trajectory(const Trajectory& input,const JointProfile& profile) {
 auto value=validate_trajectory(input,profile);if(!value)return {};
 const bool positions_only=std::all_of(value->points.begin(),value->points.end(),[](const auto& point){return point.velocities.empty()&&point.accelerations.empty();});
 return std::shared_ptr<const PreparedTrajectory>(new PreparedTrajectory(std::move(*value),profile,positions_only));
}
// 实测起点只影响首槽及下一段的速度差，完整后续路径已经不可变地验证。
bool validate_prepared_start(const PreparedTrajectory& prepared,const Position& actual) noexcept {
 const auto& path=prepared.trajectory_;const auto& p=prepared.profile_;const auto& first=path.points.front();
 for(std::size_t axis=0;axis<p.joints.size();++axis) {
  const double origin=actual[axis],q=first.positions[axis];
  if(!std::isfinite(origin))return false;
  const bool outside=origin<p.lower[axis] || origin>p.upper[axis];
  if(outside && (!prepared.positions_only_ || !p.manual_inward_start ||
      origin<p.nominal_lower[axis] || origin>p.nominal_upper[axis] ||
      (origin<p.lower[axis]?p.lower[axis]-origin:origin-p.upper[axis])>p.goal_tolerance[axis]))return false;
  if(q<p.lower[axis] || q>p.upper[axis]) {
   if(!outside || std::abs(q-origin)>p.goal_tolerance[axis] || !inward_boundary_step(p,axis,origin,q,q))return false;
  }
  if(!prepared.positions_only_)continue;
  const double dt=first.time;if(dt<=0)return false;
  const double velocity=(q-origin)/dt;
  if(std::abs(velocity)>p.velocity[axis] || std::abs(velocity)/dt>p.acceleration[axis])return false;
  if(path.points.size()>1) {
   const auto& second=path.points[1];const double next_dt=second.time-dt;
   const double next_velocity=(second.positions[axis]-q)/next_dt;
   if(std::abs(next_velocity-velocity)/std::min(dt,next_dt)>p.acceleration[axis])return false;
  }
 }
 return true;
}
/**
 * @brief 保存借用依赖、profile值副本与握手预算，不创建会话或控制线程。
 * @param client 串行管理端口，寿命覆盖gate。
 * @param port 实际JTC/控制update适配器，寿命覆盖gate。
 * @param profile 已绑定的轴序/范围/动态/容差。
 * @param timeout 单次管理/prime/排空预算，宿主µs。
 */
ActionGate::ActionGate(ManagementClient &client, ActionPort &port, JointProfile profile, std::uint64_t timeout)
    : client_(client),port_(port),profile_(std::move(profile)),timeout_(timeout) {}
/**
 * @brief 构造当前授权下的管理根，固定或复用总截止后提交一次。
 * @param operation Enable/Begin/Complete/Cancel等操作，Complete附冻结终seq和rad终点。
 * @param now 宿主steady µs提交时刻。
 * @param existing_deadline 非零复用原截止，0才计算now+timeout的饱和值。
 * @return 成功取得request或可因精确同身份Renew暂延时true，否则转失败并false。
 * @note 仅Complete和无错误的正常用户Cancel可等待占槽续租；不是任意Busy请求都可延期。
 */
bool ActionGate::issue(ManageOperation operation,std::uint64_t now,std::uint64_t existing_deadline) {
  if (operation==ManageOperation::kBegin && !valid_begin_trajectory(*prepared_trajectory_,now)) {
    // Begin尚未提交：不发Stop/Cancel、不撤现有Hold，不制造新的设备事务。
    terminal_=Terminal::kAborted; request_=0; deadline_=0; hold_drain_pending_=false;
    return false;
  }
  auto request=authorization_.identity; request.set_expected_motion_epoch(epoch_);
  request.set_operation(static_cast<std::uint16_t>(operation));
  if (operation==ManageOperation::kComplete) {
    request.set_terminal_producer_seq(frozen_seq_);
    double positions[6]{};
    std::copy_n(frozen_q_.begin(),profile_.joints.size(),positions);
    request.set_terminal_position_rad(positions);
  }
  operation_=operation;
  deadline_=existing_deadline ? existing_deadline : saturating_deadline(now,timeout_);
  request_=client_.submit(request,now);
  management_evidence_={}; state_evidence_={};
  cancel_request_id_=(operation==ManageOperation::kCancel && cancel_terminal_) ? request_ : 0U;
  /**
   * @brief 尚未提交的Complete可保留原根等待同tuple续租；不重启deadline，不把返回true当设备已经收到。
   */
  if (!request_ && operation==ManageOperation::kComplete) {
    completing_after_renewal_=client_.matching_inflight_renewal(request);
    if(completing_after_renewal_!=0U) { deferred_complete_request_=request; return true; }
  }
  if (!request_ && operation==ManageOperation::kCancel && cancel_terminal_ && error_.empty()) {
    canceling_after_renewal_=client_.matching_inflight_renewal(request);
    if(canceling_after_renewal_!=0U) { deferred_cancel_request_=request; return true; }
  }
  if (!request_) { fail(operation==ManageOperation::kCancel ? "STOP_FAILED" : "TRANSPORT_UNAVAILABLE"); return false; }
  return true;
}
/**
 * @brief 导出已COMMITTED并回到Hold的正常用户取消ID对。
 * @return 所有终态、operation、request/operation ID、成功阶段和身份/epoch均匹配时有值。
 * @note 此取消凭据不是D10释放凭据，raw终页和当前保持验收仍由集成层负责。
 */
std::optional<CancelReceipt> ActionGate::confirmed_cancel_receipt() const noexcept {
  const auto &reply=management_evidence_;
  if(terminal_!=Terminal::kCanceled || phase_!=Phase::kHold || request_!=0U ||
     operation_!=ManageOperation::kCancel || !cancel_terminal_ || cancel_request_id_==0U ||
     reply.request_id!=cancel_request_id_ || reply.operation_id==0U ||
     !reply.accepted || !reply.result_valid || !reply.succeeded ||
     reply.transaction_phase!=wire_value(WireTransactionPhase::kCommitted) ||
     reply.result_code!=wire_value(WireResult::kOk) || reply.motion_epoch!=epoch_ ||
     reply.session_id!=authorization_.identity.expected_session_id() ||
     reply.reference_epoch!=authorization_.identity.expected_reference_epoch())
    return std::nullopt;
  return CancelReceipt{cancel_request_id_,reply.operation_id};
}
/**
 * @brief 仅从Idle且session/reference/permit三个本地资格齐备时请求PrepareHold。
 * @param authorization 上层按真实观测建立的身份/资格快照。
 * @param now 宿主µs。
 * @return 已提交本地管理根时true；进入Hold还需结果、State和prime控制更新合证。
 */
bool ActionGate::enable(const Authorization &authorization,std::uint64_t now) {
  if (phase_!=Phase::kIdle || request_ || !authorization.session || !authorization.reference ||
      !authorization.permit || authorization.identity.expected_session_id()==0) return false;
  authorization_=authorization;
  epoch_=authorization.identity.expected_motion_epoch();
  return issue(ManageOperation::kEnable,now);
}
/**
 * @brief 空闲Hold中替换同设备/会话/参考的已验证permit快照。
 * @param authorization 外部刚完成许可流程的身份和三资格。
 * @return 本地绑定检查通过时true，不发送SetPermit，也不续session租期。
 */
bool ActionGate::renew_permit(const Authorization &authorization) {
  const auto &current=authorization_.identity;
  const auto &next=authorization.identity;
  if (phase_!=Phase::kHold || request_!=0U || !authorization.session ||
      !authorization.reference || !authorization.permit ||
      current.expected_boot_epoch()!=next.expected_boot_epoch() ||
      current.expected_session_id()!=next.expected_session_id() ||
      current.expected_reference_epoch()!=next.expected_reference_epoch() ||
      current.expected_profile_id()!=next.expected_profile_id() ||
      current.configured_mask()!=next.configured_mask() ||
      !std::equal(std::begin(current.device_id()),std::end(current.device_id()),std::begin(next.device_id())) ||
      !std::equal(std::begin(current.client_instance_id()),std::end(current.client_instance_id()),
                  std::begin(next.client_instance_id()))) return false;
  authorization_=authorization;
  return true;
}
/**
 * @brief 接纳已验证轨迹，并请求Begin或先冻结Hold等待worker排空。
 * @param trajectory 完整用户路径，不只检查末点。
 * @param now 宿主µs，排空与随后Begin共用同一截止。
 * @return 本地受理时true；设备Running和内部轨迹启动均尚需后继握手。
 */
bool ActionGate::valid_begin_trajectory(const PreparedTrajectory &prepared,std::uint64_t now) {
  if(!prepared.positions_only())return true;
  // 上层transport另核源序号/CAN年龄；本层仍不准以旧Hold位置启动新的纯位置路径。
  if (!hold_state_observed_us_ || now<hold_state_observed_us_ ||
      now-hold_state_observed_us_>250000U ||
      hold_submitted_mask_!=authorization_.identity.configured_mask() ||
      hold_sent_mask_!=authorization_.identity.configured_mask()) {
    error_="FRESH_HOLD_START_REQUIRED";
    return false;
  }
  if (!validate_prepared_start(prepared,measured_hold_q_)) {
    error_="TRAJECTORY_LIMIT_VIOLATION";
    return false;
  }
  return true;
}
std::shared_ptr<const PreparedTrajectoryTail> prepare_trajectory_tail(
    std::shared_ptr<const PreparedTrajectory> base,std::shared_ptr<const PreparedTrajectory> candidate,
    std::uint64_t through) {
 if(!base || !candidate || !through)return {};
 const auto& a=base->trajectory();const auto& b=candidate->trajectory();
 if(a.joints!=b.joints || a.path_tolerance!=b.path_tolerance || a.goal_tolerance!=b.goal_tolerance ||
    a.goal_time_tolerance!=b.goal_time_tolerance || b.points.back().time>630. ||
    b.points.back().time*1e9<=static_cast<double>(through))return {};
 bool matched=false;
 for(std::size_t i=0;i<a.points.size();++i) {
   const auto& x=a.points[i];const auto stamp=static_cast<std::uint64_t>(std::llround(x.time*1e9));
   if(stamp>through)break;
   if(i>=b.points.size())return {};const auto& y=b.points[i];
   if(x.time!=y.time || x.positions!=y.positions || x.velocities!=y.velocities || x.accelerations!=y.accelerations)return {};
   if(stamp==through)matched=true;
 }
 if(!matched)return {};
 auto value=std::make_shared<PreparedTrajectoryTail>();value->base_=std::move(base);
 value->candidate_=std::move(candidate);value->through_ns_=through;return value;
}
bool ActionGate::can_extend_prepared(const std::shared_ptr<const PreparedTrajectoryTail>& tail,std::uint64_t phase_ns) const noexcept {
 return tail && phase_==Phase::kRunning && terminal_==Terminal::kNone && !frozen_ &&
   authorization_.session && authorization_.reference && tail->base_==prepared_trajectory_ &&
   tail->candidate_->matches(profile_) && tail->through_ns_>=phase_ns &&
   tail->through_ns_-phase_ns>=400000000ULL;
}
bool ActionGate::extend_prepared(const std::shared_ptr<const PreparedTrajectoryTail>& tail,std::uint64_t phase_ns) noexcept {
 if(!can_extend_prepared(tail,phase_ns))return false;
 prepared_trajectory_=tail->candidate_;trajectory_=&prepared_trajectory_->trajectory();
 positions_only_trajectory_=prepared_trajectory_->positions_only();return true;
}
bool ActionGate::submit(const Trajectory& trajectory,std::uint64_t now) {
 return submit_prepared(prepare_trajectory(trajectory,profile_),now);
}
bool ActionGate::submit_prepared(std::shared_ptr<const PreparedTrajectory> prepared,std::uint64_t now) {
 if(!ready() || !authorization_.session || !authorization_.reference || !authorization_.permit)return false;
 if(!prepared || !prepared->matches(profile_)){error_="TRAJECTORY_LIMIT_VIOLATION";return false;}
 if(!valid_begin_trajectory(*prepared,now))return false;
 prepared_trajectory_=std::move(prepared);trajectory_=&prepared_trajectory_->trajectory();
 positions_only_trajectory_=prepared_trajectory_->positions_only();
 update_rejection_={};terminal_=Terminal::kNone;error_.clear();frozen_=false;frozen_seq_=0;
  /**
   * @brief 端口不提供冻结时直接提交Begin；支持冻结则保存宿主代/线序号，等待worker处理水位和新State的提交/SENT水位。
   */
  if (!port_.freeze_hold()) return issue(ManageOperation::kBegin,now);
  frozen_seq_=last_seq_; frozen_generation_=last_producer_generation_;
  drained_generation_=0; drained_committed_seq_=0;
  drain_worker_ready_=false; drain_state_fresh_=false; hold_drain_pending_=true;
  if(auto* journal=dummy_ros_control::current_host_diagnostics) {
        journal->detail(
               "GATE_HOLD_DRAIN_FREEZE_DIAG frozen=%u generation=%llu committed=%u state=%u epoch=%u submitted=%u sent=%u\n",
               frozen_seq_,static_cast<unsigned long long>(frozen_generation_),
               hold_committed_seq_,state_seq_,epoch_,hold_submitted_mask_,hold_sent_mask_);
  } else std::fprintf(stderr,
               "GATE_HOLD_DRAIN_FREEZE_DIAG frozen=%u generation=%llu committed=%u state=%u epoch=%u submitted=%u sent=%u\n",
               frozen_seq_,static_cast<unsigned long long>(frozen_generation_),
               hold_committed_seq_,state_seq_,epoch_,hold_submitted_mask_,hold_sent_mask_);
  deadline_=saturating_deadline(now,timeout_);
  return true;
}
/**
 * @brief 切换内部generation/motion epoch并要求端口产生真实保持采样。
 * @param reply 已合并的设备接受/实测或保持目标证据。
 * @param phase 本次PrimeHold/Motion/CompletionHold/StopHold阶段。
 * @param now 宿主µs，运动首段重验证失败时用于受控取消。
 * @return prime请求已发给port为true；不表示已经有update、网络publish或SENT。
 */
bool ActionGate::prime(const ManageReply &reply,Phase phase,std::uint64_t now) {
  port_.disable(); ++generation_; epoch_=reply.motion_epoch; phase_=phase;
  sampled_=false; last_seq_=0; last_producer_generation_=0;
  prime_seq_=0; frozen_=false;
  prime_producer_generation_=0; hold_state_observed_us_=0;
  hold_committed_seq_=0; hold_submitted_mask_=hold_sent_mask_=0;
  prime_q_=phase==Phase::kPrimeMotion ? reply.position : reply.hold_position;
  /**
   * @brief 进入运动时用刚取得的实测rad起点重验纯位置首段，防止初始验证后实际起点变化造成动态越限。
   */
  if (phase==Phase::kPrimeMotion) {
    if(!prepared_trajectory_ || !validate_prepared_start(*prepared_trajectory_,prime_q_)) {
      stop(false,"TRAJECTORY_LIMIT_VIOLATION",now);return false;
    }
  }
  if(auto* journal=dummy_ros_control::current_host_diagnostics) {
        journal->detail( "GATE_PRIME_DIAG phase=%u epoch=%u generation=%llu q0=%.9f\n",
               static_cast<unsigned>(phase), epoch_,
               static_cast<unsigned long long>(generation_), prime_q_[0]);
  } else std::fprintf(stderr, "GATE_PRIME_DIAG phase=%u epoch=%u generation=%llu q0=%.9f\n",
               static_cast<unsigned>(phase), epoch_,
               static_cast<unsigned long long>(generation_), prime_q_[0]);
  port_.prime(prime_q_,epoch_,generation_);
  return true;
}
/**
 * @brief 检查管理成功、同epoch实际快照及真实prime更新的设备commit覆盖。
 * @param reply State与管理结果的合成证据。
 * @return snapshot_valid、sampled、非零commit覆盖prime_seq且result成功时true；不是独立六轴原始SENT证明。
 */
bool ActionGate::handshake(const ManageReply &reply) const {
  return reply.snapshot_valid && reply.motion_epoch==epoch_ && sampled_ &&
      reply.accepted_seq!=0U && sequence_at_or_after(reply.accepted_seq,prime_seq_) &&
      reply.result_valid && reply.succeeded;
}
/**
 * @brief 合并当前request的管理结果和已到State，再推进阶段。
 * @param reply 解码后的分层管理证据；普通wire RPC不带实际位置快照。
 * @param now 宿主µs。
 * @note 管理先到或State先到均可缓存；不能只因queued或accepted就启动轨迹。
 */
void ActionGate::observe(const ManageReply &reply,std::uint64_t now) {
  if (reply.request_id!=request_) return;
  management_evidence_=reply;
  if (state_evidence_.snapshot_valid && !reply.snapshot_valid) {
    auto combined=state_evidence_;
    combined.result_valid=reply.result_valid; combined.succeeded=reply.succeeded;
    combined.rejected=reply.rejected; combined.queued=false;
    advance(combined,now);
  } else advance(reply,now);
  try_completion_hold(now);
}
/**
 * @brief 解码并检查基本身份、严格序号和配置轴有限位置后消费可信State。
 * @param bytes 调用期有效的wire State字节。
 * @param size 固定codec载荷长度，由decode核对。
 * @param now 上层提供的宿主新鲜接收/观察µs。
 * @return false拒帧；true表示消费，可能引发故障或停止。
 * @note 本层不估算设备produced时钟或原始年龄；调用方须先保证流的新鲜性/源时序，不能把此返回值当全部transport校验。
 */
bool ActionGate::observe_state(const std::uint8_t *bytes,std::size_t size,std::uint64_t now) {
  namespace wire=dp::iface;
  wire::DummyJointStateV2 state;
  if (wire::DummyJointStateV2Codec::decode(bytes,size,&state)!=wire::CodecResult::kOk ||
      state.schema_major()!=2 ||
      (!state_seen_ && state.state_seq()==0U) ||
      (state_seen_ && !sequence_newer(state.state_seq(),state_seq_)) ||
      state.boot_epoch()!=authorization_.identity.expected_boot_epoch() ||
      state.session_id()!=authorization_.identity.expected_session_id() ||
      state.reference_epoch()!=authorization_.identity.expected_reference_epoch() ||
      state.profile_id()!=authorization_.identity.expected_profile_id() ||
      state.configured_mask()!=authorization_.identity.configured_mask() ||
      (state.position_valid_mask() & state.configured_mask())!=state.configured_mask() ||
      !std::equal(std::begin(state.device_id()),std::end(state.device_id()),std::begin(authorization_.identity.device_id()))) {
    return false;
  }
  const bool hold_valid=state.hold_ready() && state.configured_mask()!=0 &&
      (state.position_valid_mask() & state.configured_mask())==state.configured_mask();
  for (std::size_t axis=0;axis<6;++axis) {
    if (!(state.configured_mask() & (1U<<axis))) continue;
    if (!std::isfinite(state.position_rad()[axis])) return false;
  }
  state_seq_=state.state_seq(); state_seen_=true;
  hold_state_observed_us_=0;
  /**
   * @brief 接纳帧后更新票据及管理epoch；hold证据只在当前motion且core Hold/位置有效时记录，不把任意State当排空确认。
   */
  authorization_.identity.set_expected_management_epoch(state.management_epoch());
  authorization_.identity.set_freshness_ticket(state.freshness_ticket());
  if (state.motion_epoch()==epoch_ &&
      state.core_state()==wire_value(WireCoreState::kHold) && hold_valid) {
    hold_committed_seq_=state.committed_producer_seq();
    hold_submitted_mask_=state.can_submitted_mask();
    hold_sent_mask_=state.can_sent_mask();
    hold_state_observed_us_=now;
    std::copy(std::begin(state.position_rad()),std::end(state.position_rad()),measured_hold_q_.begin());
    if (hold_drain_pending_) drain_state_fresh_=true;
  }
  if (hold_drain_pending_ && state.core_state()==wire_value(WireCoreState::kFault)) {
    fail("DEVICE_FAULT");
    return true;
  }
  if (hold_drain_pending_ && deadline_ && now>=deadline_) {
    stop(false,"HOLD_DRAIN_TIMEOUT",now);
    return true;
  }
  /**
   * @brief Begin排空同时要求worker水位和新到Hold State完整submitted/sent掩码及commit覆盖；复用原截止提交Begin，不能因等待结束续预算。
   */
  if (hold_drain_pending_ && drain_worker_ready_ && drain_state_fresh_ &&
      state.motion_epoch()==epoch_ &&
      state.core_state()==wire_value(WireCoreState::kHold) && hold_valid &&
      state.can_submitted_mask()==state.configured_mask() &&
      state.can_sent_mask()==state.configured_mask() &&
      state.committed_producer_seq()!=0U &&
      sequence_at_or_after(state.committed_producer_seq(),drained_committed_seq_)) {
    const auto begin_deadline=deadline_;
    hold_drain_pending_=false;
    issue(ManageOperation::kBegin,now,begin_deadline);
    return true;
  }
  if (!request_) return true;
  if (state.core_state()==wire_value(WireCoreState::kFault)) { fail("DEVICE_FAULT"); return true; }
  /**
   * @brief State提供实测位置与core接受现场，RPC提供result_valid/succeeded；两来源保留独立性，Queued本身不提供实际状态。
   */
  ManageReply evidence; evidence.request_id=request_; evidence.snapshot_valid=true;
  evidence.motion_epoch=state.motion_epoch(); evidence.accepted_seq=state.committed_producer_seq();
  const bool enabling_hold=state.core_state()==wire_value(WireCoreState::kEnablingHold);
  evidence.hold_ready=state.hold_ready() && hold_valid;
  evidence.hold_available=enabling_hold ||
      (hold_valid && state.core_state()==wire_value(WireCoreState::kHold));
  // A fresh, identity-checked state transition is core acceptance evidence; QUEUED alone is not.
  evidence.accepted=(evidence.motion_epoch>epoch_ &&
      ((operation_==ManageOperation::kBegin && state.core_state()==wire_value(WireCoreState::kRunning)) ||
       (operation_!=ManageOperation::kBegin && evidence.hold_available))) ||
      (evidence.motion_epoch==epoch_ && phase_!=Phase::kIdle && phase_!=Phase::kHold);
  std::copy(std::begin(state.position_rad()),std::end(state.position_rad()),evidence.position.begin());
  evidence.hold_position=evidence.position;
  evidence.result_valid=management_evidence_.result_valid; evidence.succeeded=management_evidence_.succeeded;
  state_evidence_=evidence; advance(evidence,now);
  // prime() may have advanced the epoch while handling this very State.
  if(state.motion_epoch()==epoch_ && state.core_state()==wire_value(WireCoreState::kHold) && hold_valid) {
    hold_committed_seq_=state.committed_producer_seq();
    hold_submitted_mask_=state.can_submitted_mask(); hold_sent_mask_=state.can_sent_mask();
    hold_state_observed_us_=now;
  }
  try_completion_hold(now); return true;
}
/**
 * @brief 记录worker排空快照，并在同身份/代数覆盖时建立Begin排空水位。
 * @param evidence 宿主processed_generation、线commit和实际Hold位置/容差。
 * @param now 宿主µs，原截止内才推进。
 * @note Complete后保持使用更严格try_completion_hold；这里的Begin排空标志仍要后继State核全轴SENT。
 */
void ActionGate::observe_hold_drain(const HoldDrainEvidence &evidence,std::uint64_t now) {
  completion_worker_=evidence;
  try_completion_hold(now);
  if (!hold_drain_pending_ || !evidence.processed_generation || !evidence.committed_seq ||
      evidence.processed_generation<frozen_generation_ ||
      evidence.boot_epoch!=authorization_.identity.expected_boot_epoch() ||
      evidence.session_id!=authorization_.identity.expected_session_id() ||
      evidence.reference_epoch!=authorization_.identity.expected_reference_epoch() ||
      evidence.motion_epoch!=epoch_ || (deadline_ && now>=deadline_)) return;
  if (!drain_worker_ready_)
    if(auto* journal=dummy_ros_control::current_host_diagnostics) {
        journal->detail(
                 "GATE_HOLD_DRAIN_WORKER_DIAG frozen_generation=%llu processed=%llu target=%u epoch=%u\n",
                 static_cast<unsigned long long>(frozen_generation_),
                 static_cast<unsigned long long>(evidence.processed_generation),
                 evidence.committed_seq,epoch_);
  } else std::fprintf(stderr,
                 "GATE_HOLD_DRAIN_WORKER_DIAG frozen_generation=%llu processed=%llu target=%u epoch=%u\n",
                 static_cast<unsigned long long>(frozen_generation_),
                 static_cast<unsigned long long>(evidence.processed_generation),
                 evidence.committed_seq,epoch_);
  drained_generation_=evidence.processed_generation;
  drained_committed_seq_=evidence.committed_seq;
  drain_worker_ready_=true;
}
/**
 * @brief 严格核管理Complete、worker排空和双新鲜实际Hold，再发布外部Succeeded。
 * @param now 宿主µs，核worker/State原接收锚及原截止。
 * @note 要求worker quiescent/合法disposition、代号覆盖、State全轴submitted/sent和commit覆盖；
 * 逐轴rad值须在worker容差内。该分支不代替raw首/终页或机械资格验收。
 */
void ActionGate::try_completion_hold(std::uint64_t now) {
  const auto &worker=completion_worker_;
  const auto mask=authorization_.identity.configured_mask();
  if(phase_!=Phase::kPrimeCompletionHold || operation_!=ManageOperation::kComplete ||
     !request_ || !sampled_ || !prime_producer_generation_ ||
     (deadline_ && now>=deadline_) || !management_evidence_.result_valid ||
     !management_evidence_.succeeded || management_evidence_.request_id!=request_ ||
     management_evidence_.motion_epoch!=epoch_ ||
     !worker.quiescent || (worker.disposition!=1U && worker.disposition!=2U) ||
     worker.processed_generation<prime_producer_generation_ || !worker.committed_seq ||
     worker.boot_epoch!=authorization_.identity.expected_boot_epoch() ||
     worker.session_id!=authorization_.identity.expected_session_id() ||
     worker.reference_epoch!=authorization_.identity.expected_reference_epoch() ||
     worker.motion_epoch!=epoch_ || !worker.observed_us || !worker.max_state_age_us ||
     now<worker.observed_us || now-worker.observed_us>worker.max_state_age_us ||
     !hold_state_observed_us_ || now<hold_state_observed_us_ ||
     now-hold_state_observed_us_>worker.max_state_age_us ||
     !state_evidence_.snapshot_valid || state_evidence_.motion_epoch!=epoch_ ||
     !state_evidence_.hold_ready || !mask || hold_submitted_mask_!=mask || hold_sent_mask_!=mask ||
     !sequence_at_or_after(hold_committed_seq_,worker.committed_seq)) return;
  for(std::size_t axis=0;axis<prime_q_.size();++axis) {
    if(!(mask & (1U<<axis))) continue;
    const auto tolerance=worker.tolerance[axis];
    if(!std::isfinite(tolerance) || tolerance<0.0 || !std::isfinite(worker.position[axis]) ||
       std::abs(worker.position[axis]-prime_q_[axis])>tolerance ||
       std::abs(state_evidence_.hold_position[axis]-prime_q_[axis])>tolerance) return;
  }
  terminal_=Terminal::kSucceeded; phase_=Phase::kHold; request_=0; deadline_=0;
}
/**
 * @brief 根据当前管理根、State快照与prime更新推进Enable/Begin/Complete/Cancel。
 * @param reply 已合并分层证据，request必须匹配。
 * @param now 宿主µs，失败时决定取消或Fault。
 * @note PrimeMotion握手后才调用port.start；普通首样本握手和特殊Complete排空路径各自保留当前实现，不把内部结果直接当外部终态。
 */
void ActionGate::advance(const ManageReply &reply,std::uint64_t now) {
  if (!request_ || reply.request_id!=request_ || phase_==Phase::kFault) return;
  if (reply.rejected || (reply.result_valid && !reply.succeeded)) {
    if (phase_==Phase::kStopping || phase_==Phase::kPrimeStopHold) fail("STOP_FAILED");
    else if (operation_==ManageOperation::kEnable) fail("ENABLE_FAILED");
    else stop(false,"OPERATION_FAILED",now);
    return;
  }
  if (reply.queued || !reply.accepted || !reply.snapshot_valid) return;
  if (phase_==Phase::kCompleting && frozen_ && operation_==ManageOperation::kBegin) {
    if (reply.motion_epoch==epoch_ && reply.accepted_seq==frozen_seq_) issue(ManageOperation::kComplete,now);
    return;
  }
  if (operation_==ManageOperation::kEnable && phase_==Phase::kIdle && reply.motion_epoch>epoch_ && reply.hold_available) {
    if (!prime(reply,Phase::kPrimeHold,now)) return;
  } else if (operation_==ManageOperation::kBegin && phase_==Phase::kHold && reply.motion_epoch>epoch_) {
    if (!prime(reply,Phase::kPrimeMotion,now)) return;
    authorization_.permit=false;
  } else if (operation_==ManageOperation::kComplete && phase_==Phase::kCompleting && reply.motion_epoch>epoch_ && reply.hold_available) {
    if (!prime(reply,Phase::kPrimeCompletionHold,now)) return;
  } else if (operation_==ManageOperation::kCancel && phase_==Phase::kStopping && reply.motion_epoch>epoch_ && reply.hold_available) {
    if (!prime(reply,Phase::kPrimeStopHold,now)) return;
  }
  if (!handshake(reply)) return;
  if (phase_==Phase::kPrimeMotion) {
    phase_=Phase::kRunning; port_.start(*trajectory_,generation_); deadline_=0;
  } else if ((phase_==Phase::kPrimeHold || phase_==Phase::kPrimeCompletionHold || phase_==Phase::kPrimeStopHold) && reply.hold_ready) {
    if (phase_==Phase::kPrimeCompletionHold) terminal_=Terminal::kSucceeded;
    if (phase_==Phase::kPrimeStopHold) terminal_=cancel_terminal_ ? Terminal::kCanceled : Terminal::kAborted;
    phase_=Phase::kHold; request_=0; deadline_=0;
  }
}
/**
 * @brief 核延期Complete与当前授权的完整根绑定，ticket稍后可更新。
 * @return session/reference可用且device/client/boot/session/ref/motion/management/profile/mask一致时true。
 */
bool ActionGate::deferred_complete_matches_authority() const noexcept {
  const auto &r=deferred_complete_request_; const auto &a=authorization_.identity;
  return authorization_.session && authorization_.reference &&
      r.expected_boot_epoch()==a.expected_boot_epoch() &&
      r.expected_session_id()==a.expected_session_id() &&
      r.expected_reference_epoch()==a.expected_reference_epoch() &&
      r.expected_motion_epoch()==epoch_ && r.expected_management_epoch()==a.expected_management_epoch() &&
      r.expected_profile_id()==a.expected_profile_id() && r.configured_mask()==a.configured_mask() &&
      std::equal(std::begin(r.device_id()),std::end(r.device_id()),std::begin(a.device_id())) &&
      std::equal(std::begin(r.client_instance_id()),std::end(r.client_instance_id()),std::begin(a.client_instance_id()));
}
/**
 * @brief 识别本目标已冻结且设备commit覆盖终序号的有限完成意图。
 * @param live 当前完整管理身份，不能跨设备/会话/参考/epoch复用。
 * @param committed_seq 当前设备commit，按32位串行水位比较。
 * @param now 宿主µs，须严格早于原deadline。
 * @return 冻结目标等于最后真实采样且在轨迹终容差内、阶段及身份均满足时true。
 * @note 仅支持撤回未发的RUN续租预约，不授权新采样或延长许可。
 */
bool ActionGate::bounded_completion_intent(const ManageRequest &live, std::uint32_t committed_seq,
                                          std::uint64_t now) const noexcept {
  // 只证明本goal已冻结且设备确认的完成意图，不授权新采样或续租。
  const auto &identity=authorization_.identity;
  if(phase_!=Phase::kCompleting || terminal_!=Terminal::kNone || !frozen_ ||
     !frozen_seq_ || frozen_seq_!=last_seq_ || !committed_seq ||
     !sequence_at_or_after(committed_seq,frozen_seq_) || !deadline_ || now>=deadline_ ||
     !authorization_.session || !authorization_.reference || !trajectory_ || trajectory_->points.empty() ||
     (operation_!=ManageOperation::kBegin && operation_!=ManageOperation::kComplete) ||
     (operation_==ManageOperation::kBegin && !request_) ||
     (operation_==ManageOperation::kComplete && !request_ && !completing_after_renewal_) ||
     !live.expected_boot_epoch() || !live.expected_session_id() ||
     !live.expected_reference_epoch() || !live.expected_profile_id() || !live.configured_mask() ||
     live.expected_boot_epoch()!=identity.expected_boot_epoch() ||
     live.expected_session_id()!=identity.expected_session_id() ||
     live.expected_reference_epoch()!=identity.expected_reference_epoch() ||
     live.expected_motion_epoch()!=epoch_ ||
     live.expected_management_epoch()!=identity.expected_management_epoch() ||
     live.expected_profile_id()!=identity.expected_profile_id() ||
     live.configured_mask()!=identity.configured_mask() ||
     !std::equal(std::begin(live.device_id()),std::end(live.device_id()),std::begin(identity.device_id())) ||
     !std::equal(std::begin(live.client_instance_id()),std::end(live.client_instance_id()),std::begin(identity.client_instance_id())))
    return false;
  const auto &target=trajectory_->points.back().positions;
  if(target.size()!=profile_.joints.size() || trajectory_->goal_tolerance.size()!=target.size()) return false;
  for(std::size_t axis=0;axis<target.size();++axis) {
    if(!std::isfinite(frozen_q_[axis]) || !std::isfinite(last_q_[axis]) ||
       !std::isfinite(target[axis]) || !std::isfinite(trajectory_->goal_tolerance[axis]) ||
       trajectory_->goal_tolerance[axis]<0.0 || frozen_q_[axis]!=last_q_[axis] ||
       std::abs(frozen_q_[axis]-target[axis])>trajectory_->goal_tolerance[axis]) return false;
  }
  return true;
}
/**
 * @brief 识别当前已提交且仍在期限内的正常用户Cancel。
 * @param live 外层当前Cancel意图身份。
 * @param now 宿主µs。
 * @return 同根已提交取消且身份不漂移时true，供协调撤回未发送续租；不包括异常中止或仅延期未发取消。
 */
bool ActionGate::bounded_cancel_intent(const ManageRequest &live, std::uint64_t now) const noexcept {
  // 只识别当前已提交、仍在原期限内的取消事务，供撤回尚未发送的
  // RUN续租预约；不授权新的续租或延长停止期限。
  const auto &identity=authorization_.identity;
  return phase_==Phase::kStopping && terminal_==Terminal::kNone && frozen_ &&
      cancel_terminal_ && error_.empty() && operation_==ManageOperation::kCancel &&
      request_!=0U && cancel_request_id_==request_ && deadline_!=0U && now<deadline_ &&
      authorization_.session && authorization_.reference &&
      live.operation()==wire_value(ManageOperation::kCancel) &&
      live.expected_boot_epoch()!=0U && live.expected_session_id()!=0U &&
      live.expected_reference_epoch()!=0U && live.expected_profile_id()!=0U &&
      live.configured_mask()!=0U &&
      live.expected_boot_epoch()==identity.expected_boot_epoch() &&
      live.expected_session_id()==identity.expected_session_id() &&
      live.expected_reference_epoch()==identity.expected_reference_epoch() &&
      live.expected_motion_epoch()==epoch_ &&
      live.expected_management_epoch()==identity.expected_management_epoch() &&
      live.expected_profile_id()==identity.expected_profile_id() &&
      live.configured_mask()==identity.configured_mask() &&
      std::equal(std::begin(live.device_id()),std::end(live.device_id()),std::begin(identity.device_id())) &&
      std::equal(std::begin(live.client_instance_id()),std::end(live.client_instance_id()),std::begin(identity.client_instance_id()));
}
/**
 * @brief 查询尚未提交的冻结Complete是否被精确同根在途Renew暂占槽。
 * @param now 宿主µs。
 * @return 原截止内、延期根仍匹配且client报告同续租在途时true，不恢复目标生产。
 */
bool ActionGate::completion_waiting_for_renewal(std::uint64_t now) const noexcept {
  return phase_==Phase::kCompleting && terminal_==Terminal::kNone &&
      operation_==ManageOperation::kComplete && request_==0U && frozen_ && frozen_seq_!=0U &&
      completing_after_renewal_!=0U && deadline_!=0U && now<deadline_ &&
      deferred_complete_matches_authority() &&
      client_.matching_inflight_renewal(deferred_complete_request_)==completing_after_renewal_;
}
/**
 * @brief 核延期Cancel操作号与当前完整session/reference授权根。
 * @return 身份和epoch全部一致时true，ticket允许提交前单独刷新。
 */
bool ActionGate::deferred_cancel_matches_authority() const noexcept {
  const auto &r=deferred_cancel_request_; const auto &a=authorization_.identity;
  return r.operation()==wire_value(ManageOperation::kCancel) && authorization_.session && authorization_.reference &&
      r.expected_boot_epoch()==a.expected_boot_epoch() &&
      r.expected_session_id()==a.expected_session_id() &&
      r.expected_reference_epoch()==a.expected_reference_epoch() &&
      r.expected_motion_epoch()==epoch_ && r.expected_management_epoch()==a.expected_management_epoch() &&
      r.expected_profile_id()==a.expected_profile_id() && r.configured_mask()==a.configured_mask() &&
      std::equal(std::begin(r.device_id()),std::end(r.device_id()),std::begin(a.device_id())) &&
      std::equal(std::begin(r.client_instance_id()),std::end(r.client_instance_id()),std::begin(a.client_instance_id()));
}
/**
 * @brief 核正常用户Cancel尚未发出且仅等待同身份Renew。
 * @param live 当前外层Cancel根。
 * @param now 宿主µs，须在原deadline内。
 * @return 冻结/无错误/未建取消ID且延期请求及client占槽根均一致时true。
 */
bool ActionGate::cancel_waiting_for_renewal(const ManageRequest &live, std::uint64_t now) const noexcept {
  return phase_==Phase::kStopping && terminal_==Terminal::kNone && frozen_ &&
      cancel_terminal_ && error_.empty() && operation_==ManageOperation::kCancel &&
      request_==0U && cancel_request_id_==0U && canceling_after_renewal_!=0U &&
      deadline_!=0U && now<deadline_ && authorization_.session && authorization_.reference &&
      deferred_cancel_matches_authority() &&
      live.operation()==wire_value(ManageOperation::kCancel) &&
      live.expected_boot_epoch()==deferred_cancel_request_.expected_boot_epoch() &&
      live.expected_session_id()==deferred_cancel_request_.expected_session_id() &&
      live.expected_reference_epoch()==deferred_cancel_request_.expected_reference_epoch() &&
      live.expected_motion_epoch()==deferred_cancel_request_.expected_motion_epoch() &&
      live.expected_management_epoch()==deferred_cancel_request_.expected_management_epoch() &&
      live.expected_profile_id()==deferred_cancel_request_.expected_profile_id() &&
      live.configured_mask()==deferred_cancel_request_.configured_mask() &&
      std::equal(std::begin(live.device_id()),std::end(live.device_id()),std::begin(deferred_cancel_request_.device_id())) &&
      std::equal(std::begin(live.client_instance_id()),std::end(live.client_instance_id()),std::begin(deferred_cancel_request_.client_instance_id())) &&
      client_.matching_inflight_renewal(deferred_cancel_request_)==canceling_after_renewal_;
}
/**
 * @brief 推进有界续租延期、管理轮询、prime及Hold排空监督。
 * @param now 同一宿主steady µs。
 * @note 到截止即失败；续租延期只等同tuple COMMITTED，随后刷新ticket并提交原意图，不重发已发送mutation。
 * 超时根据阶段转STOP_FAILED、FIRST_SAMPLE_TIMEOUT或受控取消；无诊断scope时错误路径会同步fprintf，不是所有用法都无I/O。
 */
void ActionGate::tick(std::uint64_t now) {
  /**
   * @brief 正常取消的延期必须仍冻结、未结束、无错误且根身份不变；续租失败/超时直接STOP_FAILED，不能无限等待抢占单槽。
   */
  if(canceling_after_renewal_!=0U) {
    if(phase_!=Phase::kStopping || terminal_!=Terminal::kNone || !frozen_ || !cancel_terminal_ ||
       !error_.empty() || operation_!=ManageOperation::kCancel || request_!=0U || cancel_request_id_!=0U ||
       deadline_==0U || !deferred_cancel_matches_authority()) {
      canceling_after_renewal_=0U; fail("STOP_FAILED"); return;
    }

    if(deadline_ && now>=deadline_) { canceling_after_renewal_=0U; fail("STOP_FAILED"); return; }
    const auto renewal=client_.renewal_completion(canceling_after_renewal_);
    if(renewal==RenewalCompletion::kPending) return;
    canceling_after_renewal_=0U;
    if(renewal!=RenewalCompletion::kCommitted || !deferred_cancel_matches_authority()) { fail("STOP_FAILED"); return; }
    deferred_cancel_request_.set_freshness_ticket(authorization_.identity.freshness_ticket());
    request_=client_.submit(deferred_cancel_request_,now);cancel_request_id_=request_;
    if(!request_) { fail("STOP_FAILED"); return; }
  }
  /**
   * @brief Complete等待续租时原deadline仍运行；失败先转取消，成功只刷新原请求票据，不改变冻结终点/终seq。
   */
  if(completing_after_renewal_!=0U) {
    if(deadline_ && now>=deadline_) {
      completing_after_renewal_=0U; stop(false,"RESULT_UNCONFIRMED",now); return;
    }
    const auto renewal=client_.renewal_completion(completing_after_renewal_);
    if(renewal==RenewalCompletion::kPending) return;
    completing_after_renewal_=0U;
    if(renewal!=RenewalCompletion::kCommitted || !deferred_complete_matches_authority()) {
      stop(false,"RESULT_UNCONFIRMED",now); return;
    }
    deferred_complete_request_.set_freshness_ticket(authorization_.identity.freshness_ticket());
    request_=client_.submit(deferred_complete_request_,now);
    if(!request_) { fail("TRANSPORT_UNAVAILABLE"); return; }
  }
  try_completion_hold(now);
  if (hold_drain_pending_ && deadline_ && now>=deadline_) {
    if(auto* journal=dummy_ros_control::current_host_diagnostics) {
    const auto observed=journal->context().first;
    const auto& identity=authorization_.identity;
    const dummy_ros_control::DiagnosticIdentity expected{identity.expected_boot_epoch(),identity.expected_session_id(),identity.expected_reference_epoch(),epoch_,last_seq_,identity.expected_management_epoch(),authorization_.session && authorization_.reference};
    journal->reject(dummy_ros_control::DiagnosticReason::kActionHoldDrainTimeout,observed,expected,now,"HOLD_DRAIN_TIMEOUT",0U,{frozen_seq_,frozen_generation_,last_seq_,drained_generation_,drained_committed_seq_,hold_committed_seq_,state_seq_,epoch_,hold_submitted_mask_,hold_sent_mask_});
    journal->detail(
                 "GATE_HOLD_DRAIN_TIMEOUT_DIAG frozen=%u generation=%llu local=%u processed=%llu target=%u committed=%u state=%u epoch=%u submitted=%u sent=%u\n",
                 frozen_seq_,static_cast<unsigned long long>(frozen_generation_),last_seq_,
                 static_cast<unsigned long long>(drained_generation_),drained_committed_seq_,
                 hold_committed_seq_,state_seq_,epoch_,
                 hold_submitted_mask_,hold_sent_mask_);
  } else std::fprintf(stderr,
                 "GATE_HOLD_DRAIN_TIMEOUT_DIAG frozen=%u generation=%llu local=%u processed=%llu target=%u committed=%u state=%u epoch=%u submitted=%u sent=%u\n",
                 frozen_seq_,static_cast<unsigned long long>(frozen_generation_),last_seq_,
                 static_cast<unsigned long long>(drained_generation_),drained_committed_seq_,
                 hold_committed_seq_,state_seq_,epoch_,
                 hold_submitted_mask_,hold_sent_mask_);
    hold_drain_pending_=false;
    stop(false,"HOLD_DRAIN_TIMEOUT",now);
    return;
  }
  if (request_ && deadline_ && now>=deadline_) {
    if (phase_==Phase::kStopping || phase_==Phase::kPrimeStopHold) fail("STOP_FAILED");
    else if (operation_==ManageOperation::kEnable) fail("FIRST_SAMPLE_TIMEOUT");
    else stop(false,"RESULT_UNCONFIRMED",now);
    return;
  }
  if (!request_) return;
  try { if (const auto reply=client_.poll(request_)) observe(*reply,now); }
  catch (const std::exception &) {
    if (phase_==Phase::kStopping || phase_==Phase::kPrimeStopHold) fail("STOP_FAILED");
    else if (phase_==Phase::kRunning || phase_==Phase::kCompleting || phase_==Phase::kPrimeMotion)
      stop(false,"RESULT_UNCONFIRMED",now);
    else if (operation_==ManageOperation::kEnable) fail("ENABLE_FAILED");
    else fail("RESULT_UNCONFIRMED");
  }
}
/**
 * @brief 登记真实update产生的本代目标，并绑定宿主generation和32位producer序号。
 * @param generation 必须等gate内部执行代。
 * @param epoch 必须等设备motion epoch。
 * @param seq 非零且严格较last_seq新的线序号。
 * @param q 六轴容器，配置轴位置rad。
 * @param producer_generation 非零时为真实宿主控制代；0兼容回退为seq。
 * @return 通过阶段/身份/数值门并保存时true。Running核工作边界及显式手动纯位置退入特例；其他阶段每个样本及Running首样本须在1e-9 rad内等prime_q。
 * @note 登记不证明队列、网络、设备commit或CAN SENT。
 */
bool ActionGate::record_update(std::uint64_t generation,std::uint32_t epoch,std::uint32_t seq,
                               const Position &q,std::uint64_t producer_generation) {
  const auto reject=[&](UpdateRejectReason reason,unsigned axis=6U) {
    if(update_rejection_.reason==UpdateRejectReason::kNone) {
      update_rejection_={reason,phase_,axis,generation,generation_,epoch,epoch_,seq,last_seq_,q,prime_q_,last_q_};
      std::fprintf(stderr,"CONTROL_UPDATE_REJECT_DIAG reason=%u phase=%u axis=%u generation=%llu expected_generation=%llu epoch=%u expected_epoch=%u seq=%u previous_seq=%u frozen=%u sampled=%u candidate=[%.17g,%.17g,%.17g,%.17g,%.17g,%.17g] prime=[%.17g,%.17g,%.17g,%.17g,%.17g,%.17g] previous=[%.17g,%.17g,%.17g,%.17g,%.17g,%.17g]\n",
        static_cast<unsigned>(reason),static_cast<unsigned>(phase_),axis,
        static_cast<unsigned long long>(generation),static_cast<unsigned long long>(generation_),epoch,epoch_,seq,last_seq_,frozen_,sampled_,
        q[0],q[1],q[2],q[3],q[4],q[5],prime_q_[0],prime_q_[1],prime_q_[2],prime_q_[3],prime_q_[4],prime_q_[5],
        last_q_[0],last_q_[1],last_q_[2],last_q_[3],last_q_[4],last_q_[5]);
    }
    return false;
  };
  if(generation!=generation_)return reject(UpdateRejectReason::kGeneration);
  if(epoch!=epoch_)return reject(UpdateRejectReason::kEpoch);
  if(!seq || (last_seq_ && !sequence_newer(seq,last_seq_)))return reject(UpdateRejectReason::kSequence);
  if(frozen_)return reject(UpdateRejectReason::kFrozen);
  if(phase_==Phase::kStopping || phase_==Phase::kFault || phase_==Phase::kIdle || phase_==Phase::kCompleting)
    return reject(UpdateRejectReason::kPhase);
  for(std::size_t i=0;i<profile_.joints.size();++i) {
    if(!std::isfinite(q[i]))return reject(UpdateRejectReason::kNonfinite,i);
    if(phase_==Phase::kRunning && (q[i]<profile_.lower[i] || q[i]>profile_.upper[i])) {
      if(!positions_only_trajectory_ || !profile_.manual_inward_start)return reject(UpdateRejectReason::kRange,i);
      if(!inward_boundary_step(profile_,i,prime_q_[i],sampled_?last_q_[i]:prime_q_[i],q[i]))return reject(UpdateRejectReason::kInward,i);
    }
    if((phase_!=Phase::kRunning || !sampled_) && std::abs(q[i]-prime_q_[i])>1e-9)
      return reject(UpdateRejectReason::kPrimeContinuity,i);
  }
  if (!sampled_) {
    prime_seq_=seq;
    prime_producer_generation_=producer_generation ? producer_generation : seq;
  }
  last_q_=q; last_seq_=seq;
  last_producer_generation_=producer_generation ? producer_generation : seq;
  sampled_=true; return true;
}
/**
 * @brief 接收当前Running代的内部轨迹结果，成功也先冻结终采样。
 * @param generation 当前内部执行代，迟到旧代忽略。
 * @param result 内部Terminal，非Succeeded转异常取消。
 * @param error 内部错误文本，空时使用默认原因。
 * @param now 宿主µs。
 * @note 最后真实update须在目标终容差内；随后关闭sink并进入Completing，仍未报外部Succeeded。
 */
void ActionGate::inner_result(std::uint64_t generation,Terminal result,const std::string &error,std::uint64_t now) {
  if (generation!=generation_ || phase_!=Phase::kRunning || terminal_!=Terminal::kNone) return;
  if (result!=Terminal::kSucceeded) { stop(false,error.empty()?"INNER_ABORT":error,now); return; }
  if (!last_seq_) { stop(false,"NO_TERMINAL_SAMPLE",now); return; }
  for (std::size_t i=0;i<profile_.joints.size();++i)
    if (std::abs(last_q_[i]-trajectory_->points.back().positions[i])>trajectory_->goal_tolerance[i]) {
      stop(false,"BAD_TERMINAL_POSITION",now); return;
    }
  port_.disable(); frozen_=true; frozen_seq_=last_seq_; frozen_q_=last_q_;
  phase_=Phase::kCompleting; deadline_=saturating_deadline(now,timeout_);
}
/**
 * @brief 冻结旧采样、停内部执行并请求设备Cancel，区分正常取消与异常中止。
 * @param canceled true为用户正常取消，后继Hold可给Canceled；false为异常，后继Hold给Aborted。
 * @param error 原因文本，正常取消可空。
 * @param now 宿主µs，建立本次停止管理预算。
 * @note port.disable/cancel只控制宿主生产与JTC，不是设备失能；失败仍须外部Stop/释放监督。
 */
void ActionGate::stop(bool canceled,const std::string &error,std::uint64_t now) {
  canceling_after_renewal_=0U; deferred_cancel_request_={};
  completing_after_renewal_=0U;
  if (phase_==Phase::kFault || phase_==Phase::kStopping || phase_==Phase::kPrimeStopHold ||
      (ready() && terminal_!=Terminal::kNone)) return;
  if(auto* journal=dummy_ros_control::current_host_diagnostics) {
    const auto observed=journal->context().first;
    const auto& identity=authorization_.identity;
    const dummy_ros_control::DiagnosticIdentity expected{identity.expected_boot_epoch(),identity.expected_session_id(),identity.expected_reference_epoch(),epoch_,last_seq_,identity.expected_management_epoch(),authorization_.session && authorization_.reference};
    journal->reject(dummy_ros_control::DiagnosticReason::kActionStop,observed,expected,now,error.c_str(),0U,{static_cast<unsigned>(phase_),static_cast<unsigned>(operation_),request_,now},!canceled || !error.empty());
    journal->detail("GATE_STOP_DIAG phase=%u operation=%u reason=%s request=%llu now=%llu\n",
               static_cast<unsigned>(phase_),static_cast<unsigned>(operation_),error.c_str(),
               static_cast<unsigned long long>(request_),static_cast<unsigned long long>(now));
  } else std::fprintf(stderr,"GATE_STOP_DIAG phase=%u operation=%u reason=%s request=%llu now=%llu\n",
               static_cast<unsigned>(phase_),static_cast<unsigned>(operation_),error.c_str(),
               static_cast<unsigned long long>(request_),static_cast<unsigned long long>(now));
  port_.disable(); ++generation_; port_.cancel(); cancel_terminal_=canceled;
  error_=error; phase_=Phase::kStopping; frozen_=true; hold_drain_pending_=false;
  issue(ManageOperation::kCancel,now);
}
/**
 * @brief 对当前管理/排空/延期完成请求发正常用户取消，已在停止态不重启流程。
 * @param now 宿主µs。
 */
void ActionGate::cancel(std::uint64_t now) { if (request_ || hold_drain_pending_ || completing_after_renewal_!=0U) stop(true,"",now); }
/**
 * @brief 因异常取消或故障收尾，延期未发Cancel时直接Fault而不继续等续租。
 * @param error 异常原因，可空。
 * @param now 宿主µs。
 */
void ActionGate::abort(const std::string &error,std::uint64_t now) {
  if(canceling_after_renewal_!=0U && request_==0U && cancel_request_id_==0U && phase_==Phase::kStopping) { fail(error.empty()?"RESULT_UNCONFIRMED":error); return; }
  if (request_ || hold_drain_pending_ || completing_after_renewal_!=0U) stop(false,error,now);
}
/**
 * @brief 撤销session本地资格、淘汰旧generation并停止生产。
 * @param now 宿主µs；存在活动等待时先尝试取消，随后Fault。
 * @note 本地失效不等于设备会话已释放。
 */
void ActionGate::invalidate_authorization(std::uint64_t now) {
  canceling_after_renewal_=0U; deferred_cancel_request_={};
  if (phase_==Phase::kFault && !authorization_.session) return;
  if (request_ || hold_drain_pending_ || completing_after_renewal_!=0U) stop(false,"SESSION_CHANGED",now);
  authorization_.session=false; port_.disable(); ++generation_;
  fail("SESSION_CHANGED");
}
/**
 * @brief 清待办、关闭sink/取消内部轨迹并锁Fault/Aborted。
 * @param error 失败原因；通常保留首错，STOP_FAILED可覆盖旧原因。
 * @note 不在此新建网络Stop或D10；未确认设备停止仍由外部生命周期监督处理。
 */
void ActionGate::fail(const std::string &error) {
  canceling_after_renewal_=0U; deferred_cancel_request_={};
  completing_after_renewal_=0U;
  if(phase_!=Phase::kFault || error_!=error)
    if(auto* journal=dummy_ros_control::current_host_diagnostics) {
    const auto observed=journal->context().first;
    const auto& identity=authorization_.identity;
    const dummy_ros_control::DiagnosticIdentity expected{identity.expected_boot_epoch(),identity.expected_session_id(),identity.expected_reference_epoch(),epoch_,last_seq_,identity.expected_management_epoch(),authorization_.session && authorization_.reference};
    journal->reject(dummy_ros_control::DiagnosticReason::kActionFail,observed,expected,dummy_ros_control::host_diagnostic_now_us(),error.c_str(),0U,{static_cast<unsigned>(phase_),static_cast<unsigned>(operation_),request_});
    journal->detail("GATE_FAIL_DIAG phase=%u operation=%u reason=%s request=%llu\n",
                 static_cast<unsigned>(phase_),static_cast<unsigned>(operation_),error.c_str(),
                 static_cast<unsigned long long>(request_));
  } else std::fprintf(stderr,"GATE_FAIL_DIAG phase=%u operation=%u reason=%s request=%llu\n",
                 static_cast<unsigned>(phase_),static_cast<unsigned>(operation_),error.c_str(),
                 static_cast<unsigned long long>(request_));
  port_.disable(); port_.cancel(); ++generation_; phase_=Phase::kFault; terminal_=Terminal::kAborted;
  if (error_.empty() || error=="STOP_FAILED") error_=error;
  request_=0; deadline_=0; hold_drain_pending_=false;
}
}  // namespace dummy_motion_client

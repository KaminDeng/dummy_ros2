/**
 * @file guarded_joint_trajectory_controller.cpp
 * @brief 实现完整轨迹摘要、登记后提交和实时保持/采样转发。
 * 非实时register/submit采用mutex和分配，实时update不获取登记锁；基类直接订阅/action在configure时移除。此controller只产生宿主期望位置，设备发送、授权与完成证明由外部gate/transport处理。
 */
#include "dummy_ros_control/guarded_joint_trajectory_controller.hpp"
#include "dummy_ros_control/command_channel.hpp"

#include "pluginlib/class_list_macros.hpp"
#include <openssl/sha.h>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace dummy_ros_control {
namespace {
/**
 * @brief 把uint32按小端追加到摘要输入。
 * @param bytes 非实时可增长字节向量。
 * @param value 要序列化的整数位型。
 */
void append_u32(std::vector<std::uint8_t> &bytes, std::uint32_t value) {
  for (int shift = 0; shift < 32; shift += 8) bytes.push_back(static_cast<std::uint8_t>(value >> shift));
}

/**
 * @brief 校验纳秒域并保存秒/纳秒两个32位字段。
 * nanosec非法抛invalid_argument；该规范化编码不把ROS时间转换成设备/steady时钟。
 * @param bytes 摘要输入字节向量。
 * @param sec 有符号秒数，保留其32位位型。
 * @param nanosec 纳秒部分，必须小于10^9。
 */
void append_stamp(std::vector<std::uint8_t> &bytes, std::int32_t sec, std::uint32_t nanosec) {
  if (nanosec >= 1000000000U) throw std::invalid_argument("trajectory nanosec outside range");
  append_u32(bytes, static_cast<std::uint32_t>(sec));
  append_u32(bytes, nanosec);
}

/**
 * @brief 将字段长度限定为uint32后写入。
 * 超过UINT32_MAX抛invalid_argument，防长度截断碰撞。
 * @param bytes 摘要输入。
 * @param length 元素数或字节数，由调用者决定。
 */
void append_length(std::vector<std::uint8_t> &bytes, std::size_t length) {
  if (length > std::numeric_limits<std::uint32_t>::max())
    throw std::invalid_argument("trajectory sequence outside uint32 range");
  append_u32(bytes, static_cast<std::uint32_t>(length));
}

/**
 * @brief 追加长度前缀和完整字符串字节。
 * @param bytes 摘要输入。
 * @param value 坐标系/关节名等原始字符串。
 */
void append_string(std::vector<std::uint8_t> &bytes, const std::string &value) {
  append_length(bytes, value.size());
  bytes.insert(bytes.end(), value.begin(), value.end());
}

/**
 * @brief 校验有限浮点并按统一位型小端追加。
 * 先写元素数，负零规范为正零；NaN/Inf拒绝。memcpy保double位模式，不使用依平台文本格式。
 * @param bytes 摘要输入。
 * @param values 位置/速度/加速度/effort字段，各自单位保持消息语义。
 */
void append_doubles(std::vector<std::uint8_t> &bytes, const std::vector<double> &values) {
  append_length(bytes, values.size());
  for (double value : values) {
    if (!std::isfinite(value)) throw std::invalid_argument("trajectory values must be finite");
    const double normalized = value == 0.0 ? 0.0 : value;
    std::uint64_t bits = 0;
    std::memcpy(&bits, &normalized, sizeof(bits));
    for (int shift = 0; shift < 64; shift += 8)
      bytes.push_back(static_cast<std::uint8_t>(bits >> shift));
  }
}
}  // namespace

/**
 * @brief 为完整轨迹规范编码计算SHA-256。
 * 有域分隔前缀，整数/浮点按小端编码，负零规范为正零；不是仅摘要末点。
 * @param trajectory 包含头时间/坐标系、轴名、所有点位置/速度/加速度/effort/相对时间的消息。
 * @return 32字节摘要；非法时间/非有限数/超长字段抛invalid_argument。
 */
TrajectoryDigest trajectory_digest(const trajectory_msgs::msg::JointTrajectory &trajectory) {
  std::vector<std::uint8_t> bytes{'D','P','S','D','K','-','J','T','-','1',0};
  append_stamp(bytes, trajectory.header.stamp.sec, trajectory.header.stamp.nanosec);
  append_string(bytes, trajectory.header.frame_id);
  append_length(bytes, trajectory.joint_names.size());
  for (const auto &name : trajectory.joint_names) append_string(bytes, name);
  append_length(bytes, trajectory.points.size());
  for (const auto &point : trajectory.points) {
    append_doubles(bytes, point.positions);
    append_doubles(bytes, point.velocities);
    append_doubles(bytes, point.accelerations);
    append_doubles(bytes, point.effort);
    append_stamp(bytes, point.time_from_start.sec, point.time_from_start.nanosec);
  }
  TrajectoryDigest digest{};
  SHA256(bytes.data(), bytes.size(), digest.data());
  return digest;
}

/**
 * @brief 在实时控制拍处理prime或JTC轨迹并发布本地采样。
 * @param time ROS控制时刻，传基类和sink，不与设备时间直接相减。
 * @param period 控制拍时长。
 * @return 基类结果或接口/轴数错误；只有OK且desired形状匹配才调用sink。
 */
controller_interface::return_type GuardedJointTrajectoryController::update(
    const rclcpp::Time &time, const rclcpp::Duration &period) {
  /**
   * @brief prime存在时直接输出保持位置并读取actual，不推进JTC轨迹；无prime才调用基类update。该分支仍只写命令接口，不表示设备已到位。
   */
  const auto prime=*prime_buffer_.readFromRT();
  auto result=controller_interface::return_type::OK;
  if(prime) {
    if(dof_!=1 && dof_!=6) return controller_interface::return_type::ERROR;
    for(std::size_t index=0;index<dof_;++index) {
      state_desired_.positions[index]=(*prime)[index];
      state_current_.positions[index]=joint_state_interface_[0][index].get().get_optional().value_or(NAN);
      if(!joint_command_interface_[0][index].get().set_value((*prime)[index]))
        return controller_interface::return_type::ERROR;
    }
  } else result = JointTrajectoryController::update(time, period);
  /**
   * @brief 奇数序号标记采样写入中，写六轴原子actual/desired后再发布偶数；execution_sample只读一次，不忙等。
   */
  sample_sequence_.fetch_add(1,std::memory_order_acq_rel);
  for(std::size_t index=0;index<dof_ && index<6;++index) {
    sample_actual_[index].store(state_current_.positions[index],std::memory_order_relaxed);
    sample_desired_[index].store(state_desired_.positions[index],std::memory_order_relaxed);
  }
  sample_sequence_.fetch_add(1,std::memory_order_release);
  const auto sink = update_sink_.load(std::memory_order_acquire);
  if (result == controller_interface::return_type::OK && sink != nullptr &&
      state_desired_.positions.size() == dof_) {
    sink(state_desired_.positions.data(), state_desired_.positions.size(),
         time.nanoseconds(), update_sink_context_.load(std::memory_order_relaxed));
  }
  return result;
}

/**
 * @brief 配置基类后关闭其直达订阅/action并取消sink。
 * @param previous_state 框架生命周期先前状态。
 * @return 基类失败原样返回，成功返回SUCCESS。
 */
controller_interface::CallbackReturn GuardedJointTrajectoryController::on_configure(
    const rclcpp_lifecycle::State &previous_state) {
  const auto result = JointTrajectoryController::on_configure(previous_state);
  if (result != controller_interface::CallbackReturn::SUCCESS) return result;
  joint_command_subscriber_.reset();
  action_server_.reset();
  subscriber_is_active_.store(false);
  // The action gate arms this sink only after it primes the hold target.
  set_update_sink(nullptr, nullptr);
  return result;
}

/**
 * @brief 先停sink及其拥有的命令身份，再停基类。
 * @param previous_state 框架先前状态。
 * @return 基类deactivate结果；不是设备Stop确认。
 */
controller_interface::CallbackReturn GuardedJointTrajectoryController::on_deactivate(
    const rclcpp_lifecycle::State &previous_state) {
  const bool owned_command_sink=update_sink_.load(std::memory_order_acquire)!=nullptr;
  set_update_sink(nullptr, nullptr);
  if(owned_command_sink) clear_command_identity();
  return JointTrajectoryController::on_deactivate(previous_state);
}

/**
 * @brief 锁住登记槽并保存单个凭据。
 * @param credential 外部gate核对的UUID+轨迹摘要。
 * @return false表示登记槽忙；不在此执行轨迹。
 */
bool GuardedJointTrajectoryController::register_goal(const GoalCredential &credential) {
  std::lock_guard<std::mutex> lock(gate_mutex_);
  const auto registered = gate_.register_goal(credential);
  if (!registered.has_value()) return false;
  gate_ = registered.value();
  return true;
}

/**
 * @brief 核对登记/实际摘要及JTC消息有效性后移交轨迹。
 * 成功表示控制器接纳，不是设备已发送/到位。失败保留现有登记，调用方负责clear或处理错误。
 * @param credential 必须与已登记相同的UUID+摘要。
 * @param trajectory 非空共享消息；本方法复制并按本地轴序整理。
 * @return bool；成功消费登记、取消prime并放入基类轨迹缓冲。
 */
bool GuardedJointTrajectoryController::submit_registered_goal(
    const GoalCredential &credential,
    const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> &trajectory) {
  std::lock_guard<std::mutex> lock(gate_mutex_);
  if (!gate_.permits(credential) || !trajectory) return false;
  try {
    /**
     * @brief 摘要覆盖原消息全内容，先对登记值验证再按本地关节序重排，避免同UUID替换轨迹。
     */
    if (trajectory_digest(*trajectory) != credential.digest) return false;
  } catch (const std::invalid_argument &) {
    return false;
  }
  if (!validate_trajectory_msg(*trajectory)) return false;
  auto ordered = std::make_shared<trajectory_msgs::msg::JointTrajectory>(*trajectory);
  if (ordered->joint_names.size() != command_joint_names_.size()) return false;
  sort_to_local_joint_order(ordered);
  add_new_trajectory_msg(ordered);
  prime_buffer_.writeFromNonRT(nullptr);
  gate_ = gate_.clear();
  return true;
}

/**
 * @brief 在非实时端分配保持目标并写RT缓冲。
 * @param position 六轴容器中的前dof项rad，dof只允许1或6。
 * @return bool；实际使用项必须有限；不要求该值已经到位。
 */
bool GuardedJointTrajectoryController::prime_position(const std::array<double,6> &position) {
  if(dof_!=1 && dof_!=6) return false;
  for(std::size_t i=0;i<dof_;++i) if(!std::isfinite(position[i])) return false;
  prime_buffer_.writeFromNonRT(std::make_shared<std::array<double,6>>(position));
  return true;
}
/**
 * @brief 清登记并用一致样本的actual建立保持prime。
 * 若采样正在更新可无新prime；本地保持不是设备受控停车/释放回执，外部gate仍负责Stop流程。
 */
void GuardedJointTrajectoryController::cancel_execution() {
  clear_registration();
  if(const auto sample=execution_sample()) prime_position(sample->actual);
}
/**
 * @brief 无等待读取一个首尾序号一致的执行样本。
 * @return optional快照；未采样、奇数更新中或读取期间变化则空。
 */
std::optional<GuardedExecutionSample> GuardedJointTrajectoryController::execution_sample() const noexcept {
  const auto sequence=sample_sequence_.load(std::memory_order_acquire);
  if(sequence==0 || sequence%2) return {};
  GuardedExecutionSample sample; sample.sequence=sequence;
  for(std::size_t i=0;i<6;++i) {
    sample.actual[i]=sample_actual_[i].load(std::memory_order_relaxed);
    sample.desired[i]=sample_desired_[i].load(std::memory_order_relaxed);
  }
  if(sample_sequence_.load(std::memory_order_acquire)!=sequence) return {};
  return sample;
}

/**
 * @brief 在非实时mutex下清空未提交登记。
 * 不单独发送设备取消；实际运行路径的保持/停止另由cancel/gate协调。
 */
void GuardedJointTrajectoryController::clear_registration() {
  std::lock_guard<std::mutex> lock(gate_mutex_);
  gate_ = gate_.clear();
}

/**
 * @brief 先解除旧sink，发布context后装入新sink。
 * 本方法不等待已开始的回调结束，调用方须按控制器生命周期管理上下文。
 * @param sink 可空回调；nullptr停止生产本地控制更新。
 * @param context 借用上下文，设置方保证回调期间存活。
 */
void GuardedJointTrajectoryController::set_update_sink(
    UpdateSink sink, void *context) noexcept {
  update_sink_.store(nullptr, std::memory_order_release);
  update_sink_context_.store(context, std::memory_order_relaxed);
  update_sink_.store(sink, std::memory_order_release);
}

}  // namespace dummy_ros_control

PLUGINLIB_EXPORT_CLASS(
    dummy_ros_control::GuardedJointTrajectoryController,
    controller_interface::ControllerInterface)

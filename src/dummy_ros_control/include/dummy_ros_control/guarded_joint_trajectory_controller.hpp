/**
 * @file guarded_joint_trajectory_controller.hpp
 * @brief 声明只接受已登记凭据轨迹的JTC派生控制器。
 * configure移除基类公开订阅和action服务，外部gate必须通过UUID+摘要登记再提交。实时update仍复用JTC插值或显式prime保持目标，并把最新desired rad快照送本地sink；不能绕过设备会话或原始状态校验。
 */
#pragma once

#include <atomic>
#include <cstddef>
#include <memory>
#include <mutex>

#include "dummy_ros_control/goal_registration_gate.hpp"
#include "joint_trajectory_controller/joint_trajectory_controller.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace dummy_ros_control {
/**
 * @brief 为完整轨迹规范编码计算SHA-256。
 * 有域分隔前缀，整数/浮点按小端编码，负零规范为正零；不是仅摘要末点。
 * @param trajectory 包含头时间/坐标系、轴名、所有点位置/速度/加速度/effort/相对时间的消息。
 * @return 32字节摘要；非法时间/非有限数/超长字段抛invalid_argument。
 */
TrajectoryDigest trajectory_digest(const trajectory_msgs::msg::JointTrajectory &trajectory);
/**
 * @brief 六轴actual与desired位置rad及宿主样本序号；读取成功不等于设备动作完成。
 */
struct GuardedExecutionSample {
  std::array<double,6> actual{}, desired{};
  std::uint64_t sequence=0, phase_ns=0;
  std::int64_t trajectory_start_ns=0;
};

/**
 * @brief 封装一次性登记、保持prime和执行样本的控制器插件。
 */
class GuardedJointTrajectoryController
    : public joint_trajectory_controller::JointTrajectoryController {
 public:
  /**
   * @brief sink借用positions指针只在回调期间有效，count为轴数，update_time_ns是ROS时钟ns，context生命周期由设置方保证。
   */
  using UpdateSink = void (*)(const double *positions, std::size_t count,
                              std::int64_t update_time_ns, void *context);
  /**
   * @brief 在实时控制拍处理prime或JTC轨迹并发布本地采样。
   * @param time ROS控制时刻，传基类和sink，不与设备时间直接相减。
   * @param period 控制拍时长。
   * @return 基类结果或接口/轴数错误；只有OK且desired形状匹配才调用sink。
   */
  controller_interface::return_type update(
      const rclcpp::Time &time, const rclcpp::Duration &period) override;
  /**
   * @brief 配置基类后关闭其直达订阅/action并取消sink。
   * @param previous_state 框架生命周期先前状态。
   * @return 基类失败原样返回，成功返回SUCCESS。
   */
  controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State &previous_state) override;
  /**
   * @brief 先停sink及其拥有的命令身份，再停基类。
   * @param previous_state 框架先前状态。
   * @return 基类deactivate结果；不是设备Stop确认。
   */
  controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State &previous_state) override;

  /**
   * @brief 锁住登记槽并保存单个凭据。
   * @param credential 外部gate核对的UUID+轨迹摘要。
   * @return false表示登记槽忙；不在此执行轨迹。
   */
  bool register_goal(const GoalCredential &credential);
  // 消息验证、重排、摘要与基类Trajectory构造只在非控制线程执行。
  class PreparedGoal {
    friend class GuardedJointTrajectoryController;
    const GuardedJointTrajectoryController* owner=nullptr;
    GoalCredential credential{};
    mutable std::atomic<bool> installed{false};
    bool tail=false;
    std::uint64_t prepared_phase_ns=0;
    std::int64_t start_ns=0;
    std::shared_ptr<trajectory_msgs::msg::JointTrajectory> message;
    std::shared_ptr<joint_trajectory_controller::Trajectory> trajectory;
  };
  std::shared_ptr<const PreparedGoal> prepare_goal(const GoalCredential&,
      const std::shared_ptr<trajectory_msgs::msg::JointTrajectory>&);
  bool install_prepared_goal(const GoalCredential&,const std::shared_ptr<const PreparedGoal>&);

  // 非实时预热JTC索引；实时切换不重置母轨迹相位或生产序号。
  std::shared_ptr<const PreparedGoal> prepare_tail(const GoalCredential&,
      const std::shared_ptr<trajectory_msgs::msg::JointTrajectory>&);
  bool install_prepared_tail(const GoalCredential&,const std::shared_ptr<const PreparedGoal>&);

  /**
   * @brief 核对登记/实际摘要及JTC消息有效性后移交轨迹。
   * 成功表示控制器接纳，不是设备已发送/到位。失败保留现有登记，调用方负责clear或处理错误。
   * @param credential 必须与已登记相同的UUID+摘要。
   * @param trajectory 非空共享消息；本方法复制并按本地轴序整理。
   * @return bool；成功消费登记、取消prime并放入基类轨迹缓冲。
   */
  bool submit_registered_goal(
      const GoalCredential &credential,
      const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> &trajectory);
  /**
   * @brief 在非实时mutex下清空未提交登记。
   * 不单独发送设备取消；实际运行路径的保持/停止另由cancel/gate协调。
   */
  void clear_registration();
  /**
   * @brief 先解除旧sink，发布context后装入新sink。
   * 本方法不等待已开始的回调结束，调用方须按控制器生命周期管理上下文。
   * @param sink 可空回调；nullptr停止生产本地控制更新。
   * @param context 借用上下文，设置方保证回调期间存活。
   */
  void set_update_sink(UpdateSink sink, void *context) noexcept;
  /** @brief 非控制线程一次性输出已冻结的首错现场；控制拍不执行日志I/O。 */
  void log_update_failure();
  /**
   * @brief 在非实时端分配保持目标并写RT缓冲。
   * @param position 六轴容器中的前dof项rad，dof只允许1或6。
   * @return bool；实际使用项必须有限；不要求该值已经到位。
   */
  bool prime_position(const std::array<double,6> &position);
  /**
   * @brief 清登记并用一致样本的actual建立保持prime。
   * 若采样正在更新可无新prime；本地保持不是设备受控停车/释放回执，外部gate仍负责Stop流程。
   */
  void cancel_execution();
  /**
   * @brief 无等待读取一个首尾序号一致的执行样本。
   * @return optional快照；未采样、奇数更新中或读取期间变化则空。
   */
  std::optional<GuardedExecutionSample> execution_sample() const noexcept;

 private:
  // 控制拍只写固定八槽；故障后才输出，避免逐拍日志干扰正常调度。
  struct UpdateDiagnostic {
    std::uint64_t sequence=0;
    std::int64_t time_ns=0, period_ns=0, clock_delta_ns=0, steady_gap_ns=0;
    int clock_type=0;
  };
  std::array<UpdateDiagnostic,8> update_diagnostics_{};
  std::uint64_t diagnostic_updates_=0;
  std::int64_t diagnostic_last_steady_ns_=0;
  bool diagnostic_failure_reported_=false;
  std::atomic<bool> diagnostic_pending_{false};
  const char* diagnostic_reason_=nullptr;
  std::array<char,33> diagnostic_uuid_{};
  unsigned diagnostic_axis_mask_=0;
  std::int64_t diagnostic_nominal_ns_=0;
  std::uint64_t diagnostic_phase_ns_=0, diagnostic_sample_seq_=0;
  bool diagnostic_sink_enabled_=false;
  controller_interface::return_type report_update_failure(
      const char* reason, std::size_t axis=6);
  /**
   * @brief 登记mutex仅用于非实时register/submit/clear；update不获取此锁。
   */
  // 采样相位固定20ms；真实回调时钟仅用于倒退和完整丢拍检查。
  bool timing_failed_=false, update_clock_known_=false;
  std::int64_t last_update_clock_ns_=0;
  rcl_clock_type_t last_update_clock_type_=RCL_ROS_TIME;
  std::mutex gate_mutex_;
  GoalRegistrationGate gate_;
  GoalUuid active_goal_uuid_{};
  std::shared_ptr<const std::array<double,6>> entry_prime_;
  std::atomic<UpdateSink> update_sink_{nullptr};
  std::atomic<void *> update_sink_context_{nullptr};
  /**
   * @brief 非实时写共享prime，RT只读；sample_sequence奇数写入中、偶数完成，actual/desired均rad原子字段。
   */
  realtime_tools::RealtimeBuffer<std::shared_ptr<std::array<double,6>>> prime_buffer_;
  std::atomic<std::uint64_t> sample_sequence_{0}, sample_phase_ns_{0};
  std::atomic<std::int64_t> sample_start_ns_{0};
  std::array<std::atomic<double>,6> sample_actual_{}, sample_desired_{};
};

}  // namespace dummy_ros_control

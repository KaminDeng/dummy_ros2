/**
 * @file guarded_zenbedded_system.hpp
 * @brief 声明ros2_control硬件系统与通信worker间的状态/命令边界。
 * 支持Joint1或Joint1..6的position命令和position/velocity状态。实时read只try_lock复制缓存，write只尝试把新控制代放4槽队列；网络、管理准入、设备时钟及停止确认由DeviceTransport worker处理，不把active标志等同设备许可。
 */
#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <memory>

#include "dummy_ros_control/bounded_publish_queue.hpp"
#include "dummy_ros_control/command_producer.hpp"
#include "dummy_ros_control/state_validator.hpp"
#include "hardware_interface/system_interface.hpp"

namespace dummy_ros_control {
class DeviceTransport;
struct TransportStatistics;

/**
 * @brief 原始状态包络及六轴位置rad/速度rad/s值副本；无效速度可为NaN，位置有效性由掩码和ingest检查。
 */
struct JointStateSnapshot {
  StateEnvelope envelope{};
  std::array<double, 6> position{};
  std::array<double, 6> velocity{};
};

/**
 * @brief 以独占transport和有限共享缓存实现硬件插件。
 */
class GuardedZenbeddedSystem final : public hardware_interface::SystemInterface {
 public:
  /**
   * @brief 待worker消费的停止请求：None、生命周期Deactivation或Safety；只是宿主意图，不是设备Stop结果。
   */
  enum class StopRequestKind : std::uint8_t { kNone, kDeactivation, kSafety };
  /**
   * @brief 默认构造空硬件状态，不建立传输或授权。
   */
  GuardedZenbeddedSystem();
  /**
   * @brief 默认析构拥有的transport，其析构会停止并等待worker。
   */
  ~GuardedZenbeddedSystem() override;
  /**
   * @brief 校验轴序、position接口和传输配置并创建transport。
   * 仅1或6轴，名称必须Joint1起连续；每轴必须恰有一个position命令接口。参数来自URDF并非在线身份认证。
   * @param params ros2_control硬件描述及框架初始化参数。
   * @return SUCCESS或ERROR；不在此启动线程。
   */
  CallbackReturn on_init(
      const hardware_interface::HardwareComponentInterfaceParams &params) override;
  /**
   * @brief 取得状态/命令接口句柄并启动transport worker。
   * 配置后硬件仍未active；原始状态和管理bootstrap可由worker建立。
   * @param previous_state 框架传入的先前生命周期状态，当前实现未使用其内容。
   * @return SUCCESS或ERROR；线程启动不等于设备ready。
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
  /**
   * @brief 结束worker并清测量/授权缓存。
   * stop为非实时等待；清缓存不构成设备已释放的独立证明。
   * @param previous_state 框架传入的先前生命周期状态，当前实现未使用其内容。
   * @return SUCCESS。
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
  /**
   * @brief 用新鲜有效缓存填初始目标，避免激活时跳向旧命令。
   * 成功置本地active，不自行创建设备session；设备端许可由gate/transport另行确认。
   * @param previous_state 框架传入的先前生命周期状态，当前实现未使用其内容。
   * @return SUCCESS或FAILURE；锁忙、无有效状态、截止已过或接口写失败均拒绝。
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
  /**
   * @brief 撤销本地active和命令身份并请求worker停用处理。
   * 仅当没有更高优先级请求时设置Deactivation，不覆盖Safety。
   * @param previous_state 框架传入的先前生命周期状态，当前实现未使用其内容。
   * @return SUCCESS；不等待设备停止事务。
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
  /**
   * @brief 撤销active并无条件锁存Safety停止请求。
   * 清命令身份；实际管理Stop和确认由worker消费，不在实时路径同步网络。
   * @param previous_state 框架传入的先前生命周期状态，当前实现未使用其内容。
   * @return SUCCESS表示已记录宿主停止意图。
   */
  CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;
  /**
   * @brief 激活时导出仍新鲜测量，不在实时循环等待接收锁。
   * 接收正在写缓存导致try_lock失败时保留上次导出值并返回OK，但不更新时间戳/截止；因此锁忙不会伪造新鲜反馈。
   * @param time ROS控制时刻，当前未用于传输新鲜度。
   * @param period 控制周期，当前不用于外推状态。
   * @return OK或ERROR；inactive返回OK，失效/过期或句柄写失败ERROR。
   */
  hardware_interface::return_type read(
      const rclcpp::Time &time, const rclcpp::Duration &period) override;
  /**
   * @brief 激活时尝试转交最新控制记录到有限队列。
   * @param time ROS时刻，本实现不使用。
   * @param period 控制周期，本实现不使用。
   * @return OK；暂时无更新、队满/转交忙留待后续，不在此返回网络接纳状态。
   */
  hardware_interface::return_type write(
      const rclcpp::Time &time, const rclcpp::Duration &period) override;

  /**
   * @brief 校验包络及配置轴有限值后try_lock发布缓存。
   * 只对velocity_valid_mask标记的速度强制有限；成功同时开放授权状态缓存与测量读取缓存，但active仍须生命周期操作。
   * @param snapshot 六轴rad/rad/s状态及源时间包络。
   * @param expectation transport已对齐时间/身份的期望。
   * @return bool；非法状态或锁忙拒绝，不部分写入。
   */
  bool ingest_state(const JointStateSnapshot &snapshot,
                    const StateExpectation &expectation) noexcept;
  /**
   * @brief 按当前wire codec解码State并生成数值快照。
   * @param bytes 借用原始载荷，调用期间有效。
   * @param size 字节数，codec精确验证。
   * @param expectation 已经建立源时钟域关系的期望。
   * @return 解码与ingest都成功为true；无效速度置NaN。
   */
  bool ingest_wire_state(const std::uint8_t *bytes, std::size_t size,
                         const StateExpectation &expectation) noexcept;
  /**
   * @brief 非阻塞复制可用于控制/激活的有效状态。
   * 与read_state_valid分开：epoch切换可留测量给read，却禁止copy_state继续作为控制准入依据。
   * @param snapshot 非空输出指针。
   * @return bool；锁忙、state_valid=false或宿主deadline过期则false。
   */
  bool copy_state(JointStateSnapshot *snapshot) const noexcept;
  /**
   * @brief 显式发布身份后提交六轴目标快照。
   * set_command_identity可能获取writer mutex，此便捷入口不是纯无锁RT钩子。
   * @param identity 应由调用方验证的设备授权身份。
   * @param position 六轴rad目标。
   * @return 宿主控制generation或0；没有网络发送或设备commit保证。
   */
  std::uint64_t submit_control_update(
      const CommandIdentity &identity,
      const std::array<double, 6> &position) noexcept;
  /**
   * @brief 由唯一worker尝试取一条已转交记录。
   * @param record 非空输出。
   * @return bool队首存在且已复制；该层尚未检查ticket或网络。
   */
  bool take_publish_record(CommandGenerationRecord *record) noexcept;
  /**
   * @brief 在worker/ControllerManager之间竞争一次有限转交所有权。
   * atomic<bool> CAS失败立即返回；RAII归还转交所有权。只有入队成功才推进consumed_generation，队满可原代重试。
   * @return true新代成功入4槽队列；忙/无更新/队满false。
   */
  bool transfer_control_update() noexcept;
  /**
   * @brief 读取有界发布队列的累计满槽拒绝。
   * @return 宿主次数，不是CAN丢帧数。
   */
  std::uint64_t publish_drops() const noexcept;
  /**
   * @brief 取走任意停止请求并清槽。
   * @return 是否取到非None；丢失具体类型时只宜用于清理/统计。
   */
  bool take_stop_request() noexcept;
  /**
   * @brief 原子exchange消费一次停止意图。
   * @return 此前类型；之后槽为None。
   */
  StopRequestKind take_stop_request_kind() noexcept;
  /**
   * @brief 撤销本地命令并给worker设置Safety停止。
   * clear_command_identity抬高授权世代，禁止旧控制记录再生效；设备是否已停止须另看管理回执。
   */
  void request_transport_stop() noexcept;
  /**
   * @brief 查询尚未消费的停止意图。
   * @return bool；无阻塞，不清标记。
   */
  bool stop_request_pending() const noexcept {
    return stop_requested_.load(std::memory_order_acquire) != StopRequestKind::kNone;
  }
  /**
   * @brief 读取当前transport统计或默认空值。
   * @return TransportStatistics值副本。
   */
  TransportStatistics transport_statistics() const noexcept;
  /**
   * @brief 同时撤销测量、控制缓存和active。
   * @param retired_authority 可选旧身份；非空时只条件撤销该命令身份，避免覆盖新会话。
   */
  void invalidate_transport_state(const CommandIdentity *retired_authority = nullptr) noexcept;
  /**
   * @brief 仅撤销控制准入缓存及对应旧命令身份。
   * 保留read_state_valid和仍未过期的实际测量，read可继续显示；这不开放旧命令或延长deadline。
   * @param retired_authority 发生epoch变化的旧身份。
   */
  void invalidate_authority_state(const CommandIdentity &retired_authority) noexcept;
  /**
   * @brief 更新实际状态可用的宿主失鲜截止。
   * @param deadline_us HOST_TRANSPORT_STEADY_US绝对截止；0保留当前不检查截止的语义。
   */
  void set_state_deadline(std::uint64_t deadline_us) noexcept;
  /**
   * @brief 读取ROS硬件本地active标记。
   * @return bool；不能据此推断设备已使能或所有命令已发送。
   */
  bool commands_active() const noexcept { return active_.load(std::memory_order_acquire); }
  /**
   * @brief 限定支持轴数。
   * @param count 硬件描述关节数。
   * @return 仅1或6为true。
   */
  static constexpr bool active_joint_count_supported(std::size_t count) noexcept {
    return count == 1U || count == 6U;
  }

 private:
  static constexpr std::array<const char *, 6> kJointNames{
      "Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"};
  /**
   * @brief state_mutex保护值快照；RT端只try_lock，state_valid供控制准入，read_state_valid供测量导出。
   */
  mutable std::mutex state_mutex_;
  JointStateSnapshot state_{};
  std::atomic<bool> state_valid_{false};
  // Epoch changes suspend command authorization but retain still-fresh measured positions for read().
  std::atomic<bool> read_state_valid_{false};
  std::atomic<bool> active_{false};
  /**
   * @brief deadline为宿主steady µs；stop_requested存未消费意图；active_joint_count只在初始化设置。
   */
  std::atomic<std::uint64_t> state_deadline_us_{0U};
  std::atomic<StopRequestKind> stop_requested_{StopRequestKind::kNone};
  std::size_t active_joint_count_ = 0;
  static_assert(std::atomic<bool>::is_always_lock_free, "transfer owner gate必须无等待");
  /**
   * @brief 无等待转交所有权门；consumed_generation仅由持有者写。4槽队列面向唯一worker消费者。
   */
  std::atomic<bool> transfer_busy_{false};
  std::uint64_t consumed_generation_ = 0;
  BoundedPublishQueue<4> publish_queue_;
  /**
   * @brief 唯一拥有transport；接口shared_ptr由ros2_control管理并在configure取得，位置rad/速度rad/s。
   */
  std::unique_ptr<DeviceTransport> transport_;
  std::array<hardware_interface::StateInterface::SharedPtr, 6> position_handles_{};
  std::array<hardware_interface::StateInterface::SharedPtr, 6> velocity_handles_{};
  std::array<hardware_interface::CommandInterface::SharedPtr, 6> command_handles_{};
};

}  // namespace dummy_ros_control

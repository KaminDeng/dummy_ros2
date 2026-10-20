/**
 * @file guarded_zenbedded_system.cpp
 * @brief 实现ros2_control生命周期和有界状态/命令转交。
 * 本层不在read/write执行dp_comm或文件I/O。on_configure启动独占transport；on_activate先用当前新鲜测量prime接口，deactivate/error只记录停止意图，实际Stop/QueryResult/Idle确认在worker。
 */
#include "dummy_ros_control/guarded_zenbedded_system.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include "dp_dummy_robot_v2.hpp"
#include "dummy_ros_control/command_channel.hpp"
#include "dummy_ros_control/device_transport.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace dummy_ros_control {
/**
 * @brief 默认构造空硬件状态，不建立传输或授权。
 */
GuardedZenbeddedSystem::GuardedZenbeddedSystem() = default;
/**
 * @brief 默认析构拥有的transport，其析构会停止并等待worker。
 */
GuardedZenbeddedSystem::~GuardedZenbeddedSystem() = default;

/**
 * @brief 校验轴序、position接口和传输配置并创建transport。
 * 仅1或6轴，名称必须Joint1起连续；每轴必须恰有一个position命令接口。参数来自URDF并非在线身份认证。
 * @param params ros2_control硬件描述及框架初始化参数。
 * @return SUCCESS或ERROR；不在此启动线程。
 */
hardware_interface::CallbackReturn GuardedZenbeddedSystem::on_init(
    const hardware_interface::HardwareComponentInterfaceParams &params) {
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS ||
      !active_joint_count_supported(params.hardware_info.joints.size())) return CallbackReturn::ERROR;
  active_joint_count_ = params.hardware_info.joints.size();
  for (std::size_t index = 0; index < active_joint_count_; ++index) {
    const auto &joint = params.hardware_info.joints[index];
    if (joint.name != kJointNames[index] || joint.command_interfaces.size() != 1U ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      return CallbackReturn::ERROR;
    }
  }
  TransportParameters transport_parameters{};
  if (!parse_transport_parameters(params.hardware_info.hardware_parameters,
                                  active_joint_count_, &transport_parameters)) return CallbackReturn::ERROR;
  transport_ = std::make_unique<DeviceTransport>(*this, transport_parameters);
  return CallbackReturn::SUCCESS;
}

/**
 * @brief 取得状态/命令接口句柄并启动transport worker。
 * 配置后硬件仍未active；原始状态和管理bootstrap可由worker建立。
 * @param previous_state 框架传入的先前生命周期状态，当前实现未使用其内容。
 * @return SUCCESS或ERROR；线程启动不等于设备ready。
 */
hardware_interface::CallbackReturn GuardedZenbeddedSystem::on_configure(
    const rclcpp_lifecycle::State &) {
  for (std::size_t index = 0; index < active_joint_count_; ++index) {
    position_handles_[index] = get_state_interface_handle(std::string(kJointNames[index]) + "/position");
    velocity_handles_[index] = get_state_interface_handle(std::string(kJointNames[index]) + "/velocity");
    command_handles_[index] = get_command_interface_handle(std::string(kJointNames[index]) + "/position");
  }
  return transport_ && transport_->start() ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
}

/**
 * @brief 结束worker并清测量/授权缓存。
 * stop为非实时等待；清缓存不构成设备已释放的独立证明。
 * @param previous_state 框架传入的先前生命周期状态，当前实现未使用其内容。
 * @return SUCCESS。
 */
hardware_interface::CallbackReturn GuardedZenbeddedSystem::on_cleanup(
    const rclcpp_lifecycle::State &) {
  if (transport_) transport_->stop();
  invalidate_transport_state();
  return CallbackReturn::SUCCESS;
}

/**
 * @brief 用新鲜有效缓存填初始目标，避免激活时跳向旧命令。
 * 成功置本地active，不自行创建设备session；设备端许可由gate/transport另行确认。
 * @param previous_state 框架传入的先前生命周期状态，当前实现未使用其内容。
 * @return SUCCESS或FAILURE；锁忙、无有效状态、截止已过或接口写失败均拒绝。
 */
hardware_interface::CallbackReturn GuardedZenbeddedSystem::on_activate(
    const rclcpp_lifecycle::State &) {
  std::unique_lock<std::mutex> lock(state_mutex_, std::try_to_lock);
  if (!lock.owns_lock() || !state_valid_ ||
      (state_deadline_us_ != 0U && transport_steady_now_us() > state_deadline_us_)) return CallbackReturn::FAILURE;
  for (std::size_t index = 0; index < active_joint_count_; ++index) {
    if (!set_command<double>(command_handles_[index], state_.position[index], false)) return CallbackReturn::FAILURE;
  }
  active_ = true;
  return CallbackReturn::SUCCESS;
}

/**
 * @brief 撤销本地active和命令身份并请求worker停用处理。
 * 仅当没有更高优先级请求时设置Deactivation，不覆盖Safety。
 * @param previous_state 框架传入的先前生命周期状态，当前实现未使用其内容。
 * @return SUCCESS；不等待设备停止事务。
 */
hardware_interface::CallbackReturn GuardedZenbeddedSystem::on_deactivate(
    const rclcpp_lifecycle::State &) {
  active_ = false;
  auto none = StopRequestKind::kNone;
  stop_requested_.compare_exchange_strong(none, StopRequestKind::kDeactivation,
                                          std::memory_order_acq_rel);
  clear_command_identity();
  return CallbackReturn::SUCCESS;
}

/**
 * @brief 撤销active并无条件锁存Safety停止请求。
 * 清命令身份；实际管理Stop和确认由worker消费，不在实时路径同步网络。
 * @param previous_state 框架传入的先前生命周期状态，当前实现未使用其内容。
 * @return SUCCESS表示已记录宿主停止意图。
 */
hardware_interface::CallbackReturn GuardedZenbeddedSystem::on_error(
    const rclcpp_lifecycle::State &) {
  active_ = false;
  stop_requested_.store(StopRequestKind::kSafety, std::memory_order_release);
  clear_command_identity();
  return CallbackReturn::SUCCESS;
}

/**
 * @brief 激活时导出仍新鲜测量，不在实时循环等待接收锁。
 * 接收正在写缓存导致try_lock失败时保留上次导出值并返回OK，但不更新时间戳/截止；因此锁忙不会伪造新鲜反馈。
 * @param time ROS控制时刻，当前未用于传输新鲜度。
 * @param period 控制周期，当前不用于外推状态。
 * @return OK或ERROR；inactive返回OK，失效/过期或句柄写失败ERROR。
 */
hardware_interface::return_type GuardedZenbeddedSystem::read(
    const rclcpp::Time &, const rclcpp::Duration &) {
  // controller_manager polls inactive hardware before activation. Fresh state
  // remains mandatory in on_activate(); an inactive poll must not force ERROR.
  if (!active_) return hardware_interface::return_type::OK;
  std::unique_lock<std::mutex> lock(state_mutex_, std::try_to_lock);
  if (!read_state_valid_ ||
      (state_deadline_us_ != 0U && transport_steady_now_us() > state_deadline_us_)) return hardware_interface::return_type::ERROR;
  // A receive in progress is not stale state. Keep the last exported handles;
  // neither the payload timestamp nor its deadline is advanced by this path.
  /**
   * @brief 锁忙时仅沿用已导出handle值，前面仍检查read_state_valid及原deadline；不能把本轮返回OK当成收到新State。
   */
  if(!lock.owns_lock()) return hardware_interface::return_type::OK;
  for (std::size_t index = 0; index < active_joint_count_; ++index) {
    if (!set_state<double>(position_handles_[index], state_.position[index], false) ||
        !set_state<double>(velocity_handles_[index], state_.velocity[index], false))
      return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

/**
 * @brief 激活时尝试转交最新控制记录到有限队列。
 * @param time ROS时刻，本实现不使用。
 * @param period 控制周期，本实现不使用。
 * @return OK；暂时无更新、队满/转交忙留待后续，不在此返回网络接纳状态。
 */
hardware_interface::return_type GuardedZenbeddedSystem::write(
    const rclcpp::Time &, const rclcpp::Duration &) {
  if (!active_) return hardware_interface::return_type::OK;
  transfer_control_update();
  return hardware_interface::return_type::OK;
}

/**
 * @brief 在worker/ControllerManager之间竞争一次有限转交所有权。
 * atomic<bool> CAS失败立即返回；RAII归还转交所有权。只有入队成功才推进consumed_generation，队满可原代重试。
 * @return true新代成功入4槽队列；忙/无更新/队满false。
 */
bool GuardedZenbeddedSystem::transfer_control_update() noexcept {
  // worker与ControllerManager共用一个有界producer区间；Busy留待原下轮重试。
  bool expected = false;
  if (!transfer_busy_.compare_exchange_strong(expected, true,
          std::memory_order_acquire, std::memory_order_relaxed)) return false;
  /**
   * @brief 局部RAII归还转交所有权；无论无记录、队满或成功早返回，都清transfer_busy。
   */
  struct ReleaseTransfer {
    std::atomic<bool> &busy;
    /**
     * @brief 离开转交区间时以release归还生产者资格。
     */
    ~ReleaseTransfer() { busy.store(false, std::memory_order_release); }
  } release_transfer{transfer_busy_};
  const auto record = consume_control_update(consumed_generation_);
  if (!record.has_value()) return false;
  if (!publish_queue_.try_push(record.value())) return false;
  /**
   * @brief 只有try_push成功后推进宿主消费水位；队满不丢失该代的再次转交机会。
   */
  consumed_generation_ = record->generation;
  return true;
}

/**
 * @brief 校验包络及配置轴有限值后try_lock发布缓存。
 * 只对velocity_valid_mask标记的速度强制有限；成功同时开放授权状态缓存与测量读取缓存，但active仍须生命周期操作。
 * @param snapshot 六轴rad/rad/s状态及源时间包络。
 * @param expectation transport已对齐时间/身份的期望。
 * @return bool；非法状态或锁忙拒绝，不部分写入。
 */
bool GuardedZenbeddedSystem::ingest_state(
    const JointStateSnapshot &snapshot, const StateExpectation &expectation) noexcept {
  if (validate_state(snapshot.envelope, expectation) != StateValidity::kValid) return false;
  for (std::size_t index = 0; index < snapshot.velocity.size(); ++index) {
    if ((snapshot.envelope.configured_mask & (1U << index)) != 0U &&
        !std::isfinite(snapshot.position[index])) return false;
    if ((snapshot.envelope.velocity_valid_mask & (1U << index)) != 0U &&
        !std::isfinite(snapshot.velocity[index])) return false;
  }
  std::unique_lock<std::mutex> lock(state_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) return false;
  state_ = snapshot;
  state_valid_ = true;
  read_state_valid_ = true;
  return true;
}

/**
 * @brief 按当前wire codec解码State并生成数值快照。
 * @param bytes 借用原始载荷，调用期间有效。
 * @param size 字节数，codec精确验证。
 * @param expectation 已经建立源时钟域关系的期望。
 * @return 解码与ingest都成功为true；无效速度置NaN。
 */
bool GuardedZenbeddedSystem::ingest_wire_state(
    const std::uint8_t *bytes, std::size_t size,
    const StateExpectation &expectation) noexcept {
  namespace wire = dp::protocol::dummy_robot_v2;
  wire::DummyJointStateV2 decoded;
  if (wire::DummyJointStateV2Codec::decode(bytes, size, &decoded) != wire::CodecResult::kOk)
    return false;
  JointStateSnapshot snapshot{};
  snapshot.envelope.boot = decoded.boot_epoch();
  snapshot.envelope.profile = decoded.profile_id();
  snapshot.envelope.state_seq = decoded.state_seq();
  snapshot.envelope.produced_at_us = decoded.produced_at_us();
  snapshot.envelope.configured_mask = decoded.configured_mask();
  snapshot.envelope.position_valid_mask = decoded.position_valid_mask();
  snapshot.envelope.velocity_valid_mask = decoded.velocity_valid_mask();
  for (std::size_t index = 0; index < snapshot.position.size(); ++index) {
    snapshot.position[index] = decoded.position_rad()[index];
    snapshot.velocity[index] =
        (decoded.velocity_valid_mask() & (1U << index)) != 0U
            ? decoded.estimated_velocity_rad_s()[index]
            : std::numeric_limits<double>::quiet_NaN();
  }
  return ingest_state(snapshot, expectation);
}

/**
 * @brief 非阻塞复制可用于控制/激活的有效状态。
 * 与read_state_valid分开：epoch切换可留测量给read，却禁止copy_state继续作为控制准入依据。
 * @param snapshot 非空输出指针。
 * @return bool；锁忙、state_valid=false或宿主deadline过期则false。
 */
bool GuardedZenbeddedSystem::copy_state(JointStateSnapshot *snapshot) const noexcept {
  if (!snapshot) return false;
  std::unique_lock<std::mutex> lock(state_mutex_, std::try_to_lock);
  if (!lock.owns_lock() || !state_valid_ ||
      (state_deadline_us_ != 0U && transport_steady_now_us() > state_deadline_us_)) return false;
  *snapshot = state_;
  return true;
}

/**
 * @brief 显式发布身份后提交六轴目标快照。
 * set_command_identity可能获取writer mutex，此便捷入口不是纯无锁RT钩子。
 * @param identity 应由调用方验证的设备授权身份。
 * @param position 六轴rad目标。
 * @return 宿主控制generation或0；没有网络发送或设备commit保证。
 */
std::uint64_t GuardedZenbeddedSystem::submit_control_update(
    const CommandIdentity &identity, const std::array<double, 6> &position) noexcept {
  set_command_identity(identity);
  return publish_control_update(position.data(), position.size(), 0, nullptr);
}

/**
 * @brief 由唯一worker尝试取一条已转交记录。
 * @param record 非空输出。
 * @return bool队首存在且已复制；该层尚未检查ticket或网络。
 */
bool GuardedZenbeddedSystem::take_publish_record(CommandGenerationRecord *record) noexcept {
  return publish_queue_.try_pop(record);
}

/**
 * @brief 读取有界发布队列的累计满槽拒绝。
 * @return 宿主次数，不是CAN丢帧数。
 */
std::uint64_t GuardedZenbeddedSystem::publish_drops() const noexcept {
  return publish_queue_.drops();
}

/**
 * @brief 取走任意停止请求并清槽。
 * @return 是否取到非None；丢失具体类型时只宜用于清理/统计。
 */
bool GuardedZenbeddedSystem::take_stop_request() noexcept {
  return take_stop_request_kind() != StopRequestKind::kNone;
}

/**
 * @brief 原子exchange消费一次停止意图。
 * @return 此前类型；之后槽为None。
 */
GuardedZenbeddedSystem::StopRequestKind GuardedZenbeddedSystem::take_stop_request_kind() noexcept {
  return stop_requested_.exchange(StopRequestKind::kNone, std::memory_order_acq_rel);
}

/**
 * @brief 撤销本地命令并给worker设置Safety停止。
 * clear_command_identity抬高授权世代，禁止旧控制记录再生效；设备是否已停止须另看管理回执。
 */
void GuardedZenbeddedSystem::request_transport_stop() noexcept {
  active_.store(false, std::memory_order_release);
  stop_requested_.store(StopRequestKind::kSafety, std::memory_order_release);
  clear_command_identity();
}

/**
 * @brief 读取当前transport统计或默认空值。
 * @return TransportStatistics值副本。
 */
TransportStatistics GuardedZenbeddedSystem::transport_statistics() const noexcept {
  return transport_ ? transport_->statistics() : TransportStatistics{};
}

/**
 * @brief 同时撤销测量、控制缓存和active。
 * @param retired_authority 可选旧身份；非空时只条件撤销该命令身份，避免覆盖新会话。
 */
void GuardedZenbeddedSystem::invalidate_transport_state(
    const CommandIdentity *retired_authority) noexcept {
  /**
   * @brief 全部传输失效时同时撤销控制和测量资格；下面条件清身份只对退役会话生效，保护已切换的新授权。
   */
  state_valid_.store(false, std::memory_order_release);
  read_state_valid_.store(false, std::memory_order_release);
  active_.store(false, std::memory_order_release);
  if (retired_authority != nullptr) clear_command_identity_if(*retired_authority);
  else clear_command_identity();
}

/**
 * @brief 仅撤销控制准入缓存及对应旧命令身份。
 * 保留read_state_valid和仍未过期的实际测量，read可继续显示；这不开放旧命令或延长deadline。
 * @param retired_authority 发生epoch变化的旧身份。
 */
void GuardedZenbeddedSystem::invalidate_authority_state(
    const CommandIdentity &retired_authority) noexcept {
  state_valid_.store(false, std::memory_order_release);
  clear_command_identity_if(retired_authority);
}

/**
 * @brief 更新实际状态可用的宿主失鲜截止。
 * @param deadline_us HOST_TRANSPORT_STEADY_US绝对截止；0保留当前不检查截止的语义。
 */
void GuardedZenbeddedSystem::set_state_deadline(std::uint64_t deadline_us) noexcept {
  state_deadline_us_.store(deadline_us, std::memory_order_release);
}

}  // namespace dummy_ros_control

PLUGINLIB_EXPORT_CLASS(
    dummy_ros_control::GuardedZenbeddedSystem,
    hardware_interface::SystemInterface)

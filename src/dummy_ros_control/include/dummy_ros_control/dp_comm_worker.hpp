/**
 * @file dp_comm_worker.hpp
 * @brief 提供通用dp_comm命令发布与State借用回调适配。
 * 这些适配不拥有system/publisher，调用方保证其寿命。完整生产DeviceTransport还有身份bootstrap、票据、时钟、ACK和停止流程，不能把此简化publish_one的返回直接等同整套设备授权或CAN SENT。
 */
#pragma once

#include <cstdint>

#include "dp_comm/dp_comm.hpp"
#include "dp_dummy_robot_v2_dp_comm.hpp"
#include "dummy_ros_control/guarded_zenbedded_system.hpp"

namespace dummy_ros_control {

/**
 * @brief 从硬件发布队列取一条记录并编码位置消息的借用适配器。
 */
class DpCommCommandWorker {
 public:
  using WireCommand = dp::protocol::dummy_robot_v2::DummyJointCommandV2;
  using Publisher = dp::comm::Publisher<WireCommand>;

  /**
   * @brief 保存借用对象和固定profile/轴掩码。
   * @param system 待取队列的硬件系统，须存活。
   * @param publisher 已建立的dp_comm发布器，须存活。
   * @param profile wire profile ID。
   * @param configured_mask 配置轴位掩码。
   */
  DpCommCommandWorker(GuardedZenbeddedSystem *system, Publisher *publisher,
                      std::uint32_t profile, std::uint8_t configured_mask)
      : system_(system), publisher_(publisher), profile_(profile), configured_mask_(configured_mask) {}

  /**
   * @brief 取队首、填wire身份和rad目标后调用publisher。
   * 取出后不在此重排回队；此适配不核对票据时效、commit或设备ID完整bootstrap，生产worker有更完整路径。
   * @return 本地dp_comm结果；空依赖InvalidArgument，无记录NotReady。
   */
  dp::comm::Result publish_one() noexcept {
    if (!system_ || !publisher_) return dp::comm::Result::kInvalidArgument;
    CommandGenerationRecord record{};
    if (!system_->take_publish_record(&record)) return dp::comm::Result::kNotReady;
    WireCommand message;
    message.set_schema_major(2U);
    message.set_schema_minor(dp::protocol::dummy_robot_v2::kSchemaMinor);
    message.set_boot_epoch(record.identity.boot);
    message.set_session_id(record.identity.session);
    message.set_reference_epoch(record.identity.reference_epoch);
    message.set_motion_epoch(record.identity.motion_epoch);
    message.set_profile_id(profile_);
    message.set_producer_seq(static_cast<std::uint32_t>(record.generation));
    message.set_freshness_ticket(record.identity.ticket);
    message.set_configured_mask(configured_mask_);
    const double positions[6]{record.position[0], record.position[1], record.position[2],
                              record.position[3], record.position[4], record.position[5]};
    message.set_position_rad(positions);
    return publisher_->publish(message);
  }

 private:
  GuardedZenbeddedSystem *system_;
  /**
   * @brief 借用system与publisher指针，无所有权；profile/mask为消息元数据。
   */
  Publisher *publisher_;
  std::uint32_t profile_;
  std::uint8_t configured_mask_;
};

/**
 * @brief 将借用wire消息复制编码后交给硬件State校验。
 */
class DpCommStateReceiver {
 public:
  using WireState = dp::protocol::dummy_robot_v2::DummyJointStateV2;
  /**
   * @brief 固定借用目标系统和本次期望条件。
   * @param system 长于订阅回调的硬件对象。
   * @param expectation 调用方已对齐的身份/源时间域期望。
   */
  DpCommStateReceiver(GuardedZenbeddedSystem *system, StateExpectation expectation)
      : system_(system), expectation_(expectation) {}
  /**
   * @brief 在回调借用期内复制载荷，避免保存借用指针。
   * 空context/拷贝/编码失败直接返回；accepted_只在ingest发生后更新，不是所有到达帧的独立计数。
   * @param borrowed dp_comm借用State，本回调结束后不可继续使用。
   * @param context DpCommStateReceiver实例地址。
   */
  static void receive(dp::comm::BorrowedMessage<WireState> borrowed, void *context) noexcept {
    auto *self = static_cast<DpCommStateReceiver *>(context);
    if (!self || !self->system_) return;
    WireState state;
    if (borrowed.copy_to(&state) != dp::comm::Result::kOk) return;
    std::uint8_t bytes[dp::protocol::dummy_robot_v2::DummyJointStateV2Codec::kEncodedSize]{};
    if (dp::protocol::dummy_robot_v2::DummyJointStateV2Codec::encode(
            state, bytes, sizeof(bytes)) != dp::protocol::dummy_robot_v2::CodecResult::kOk) return;
    self->accepted_ = self->system_->ingest_wire_state(bytes, sizeof(bytes), self->expectation_);
  }
  /**
   * @brief 读取最近一次实际ingest结果。
   * @return bool；调用方负责与回调同上下文读取，字段非原子。
   */
  bool accepted() const noexcept { return accepted_; }
 private:
  GuardedZenbeddedSystem *system_;
  /**
   * @brief 本适配不自动刷新时钟或序号期望；生产transport按每帧情况构造新expectation。
   */
  StateExpectation expectation_;
  bool accepted_ = false;
};
}  // namespace dummy_ros_control

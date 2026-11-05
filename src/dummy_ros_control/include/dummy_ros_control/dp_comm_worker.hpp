/**
 * @file dp_comm_worker.hpp
 * @brief 提供通用dp_comm命令发布与State借用回调适配。
 * 这些适配不拥有system/publisher，调用方保证其寿命。完整生产DeviceTransport还有身份bootstrap、票据、时钟、ACK和停止流程，不能把此简化publish_one的返回直接等同整套设备授权或CAN SENT。
 */
#pragma once

#include <cstdint>

#include "dp_comm/dp_comm.hpp"
#include "dummy_ref/message_contract.hpp"
#include "dp_comm/typed_sdk.hpp"
#include "dummy_ros_control/guarded_zenbedded_system.hpp"

namespace dummy_ros_control {

/**
 * @brief 从硬件发布队列取一条记录并编码位置消息的借用适配器。
 */
class DpCommCommandWorker {
 public:
  using Command = dummy_ref::messages::Command;
  using Publisher = dp::comm::RootPublisher<Command>;

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
    Command message;
    message.boot_epoch()=record.identity.boot;
    message.session_id()=record.identity.session;
    message.operation_id()=0U;
    message.reference_epoch()=record.identity.reference_epoch;
    message.motion_epoch()=record.identity.motion_epoch;
    message.profile_id()=profile_;
    message.producer_seq()=static_cast<std::uint32_t>(record.generation);
    message.freshness_ticket()=record.identity.ticket;
    std::uint8_t zero_identity[16]{};
    message.client_instance_id().emplace().assign(zero_identity,16U);
    message.domain_device_id().emplace().assign(zero_identity,16U);
    auto& joints=message.joints().emplace();
    for(std::size_t axis=0;axis<6;++axis){
      auto* joint=joints.emplace_back();
      if(!joint)return dp::comm::Result::kNoMemory;
      joint->joint_id()=static_cast<std::uint8_t>(axis);
      joint->configured()=(configured_mask_&(1U<<axis))!=0U;
      joint->position_rad()=record.position[axis];
    }
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
 * @brief 将SDK交付的实际Owned与Source直接交给硬件State校验。
 */
class DpCommStateReceiver {
 public:
  using State = dummy_ref::messages::State;
  /**
   * @brief 固定借用目标系统和本次期望条件。
   * @param system 长于订阅回调的硬件对象。
   * @param expectation 调用方已对齐的身份/源时间域期望。
   */
  DpCommStateReceiver(GuardedZenbeddedSystem *system, StateExpectation expectation)
      : system_(system), expectation_(expectation) {}
  /**
   * @brief 在回调借用期内复制载荷，避免保存借用指针。
   * 空context直接返回；accepted_只在ingest发生后更新，不是所有到达帧的独立计数。
   * @param state SDK回调借用的实际State，不在回调外保存引用。
   * @param info 仅回调期有效的SDK接收信息；只复制SourceId。
   * @param context DpCommStateReceiver实例地址。
   */
  static void receive(const State& state,const dp::comm::ReceiveInfo& info,void* context) noexcept {
    auto* self=static_cast<DpCommStateReceiver*>(context);
    if(!self || !self->system_)return;
    self->accepted_=self->system_->ingest_message_state(state,info.source(),self->expectation_);
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

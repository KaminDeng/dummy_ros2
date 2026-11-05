/** @file typed_management_fixture.hpp
 * @brief 内存ManagementTransport直接交接生成Owned与独立Source，不运行网络或设备。
 */
#pragma once
#include "dummy_motion_client/management_client.hpp"
#include "../../dummy_ros_control/test/owned_message_fixtures.hpp"
#include <deque>
#include "dummy_ros_control/message_presence.hpp"
#include <stdexcept>
#include "canonical_records.h"
namespace typed_management_fixture {
inline dp::comm::SourceId source(std::uint64_t boot=7U){dp::comm::SourceId value{};value.device.bytes[0]=0x91;value.boot_id=boot;value.context_id=1U;return value;}
/** @brief 测试刺激直接修改实际关节质量字段，不恢复旧数组/掩码消息接口。 */
inline void configure(dummy_ref::messages::State& state,std::uint8_t mask){for(auto& joint:*state.joints())joint.configured_quality()=(mask&(1U<<*joint.joint_id()))!=0U;}
inline void position_quality(dummy_ref::messages::State& state,std::uint8_t mask){for(auto& joint:*state.joints())joint.position_valid_quality()=(mask&(1U<<*joint.joint_id()))!=0U;}
/** @brief 用中央规范记录比较完整实际Owned，测试不维护字段投影或编码表。 */
template<class T>std::vector<std::uint8_t> canonical_record(const T& value){
 using Traits=dp::protocol::canonical::Traits<T>;std::vector<std::uint8_t> bytes(Traits::kMaxEncodedSize);std::size_t written=0;
 if(!Traits::encode(value,bytes.data(),bytes.size(),&written))throw std::runtime_error("test canonical encode failed");bytes.resize(written);return bytes;
}
struct Transport:dummy_motion_client::ManagementTransport {
  bool timeout_next=false,reject_send=false;
  std::vector<dummy_motion_client::ManageRequest> requests;
  std::deque<dummy_motion_client::ManagementResponse> responses;
  struct Handle:RequestHandle {
    Transport& transport;
    explicit Handle(Transport& value):transport(value){}
    std::optional<dummy_motion_client::ManagementResponse> poll() override {
      if(transport.timeout_next){transport.timeout_next=false;throw std::runtime_error("simulated query timeout");}
      if(transport.responses.empty())return {};
      auto response=std::move(transport.responses.front());transport.responses.pop_front();return response;
    }
  };
  std::unique_ptr<RequestHandle> send(const dummy_motion_client::ManageRequest& request) override {
    if(reject_send)return {};
    requests.push_back(request);return std::make_unique<Handle>(*this);
  }
  void respond(dummy_motion_client::ManagementResponse response){responses.push_back(std::move(response));}
};
}

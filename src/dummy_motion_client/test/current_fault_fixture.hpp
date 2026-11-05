/** @file current_fault_fixture.hpp
 * @brief 当前中央Owned故障输入及规范链；不读取历史抓取，不维护业务编码。 */
#pragma once
#include "../../dummy_ros_control/test/owned_message_fixtures.hpp"
#include "canonical_records.h"
#include <algorithm>
#include <array>
#include <cstdlib>
#include <vector>
#include <openssl/sha.h>
namespace current_fixture {
template<std::size_t N>std::array<std::uint8_t,N> tag(std::uint8_t value){std::array<std::uint8_t,N> result{};result[0]=value;return result;}
inline const auto client_id=tag<16>(2),device=tag<16>(1);
inline const auto contract_hash=tag<32>(3),model_hash=tag<32>(4),profile_hash=tag<32>(5),firmware_hash=tag<32>(6);
inline const dp::comm::SourceId source=[] {dp::comm::SourceId value{};value.device.bytes[0]=0x91;value.boot_id=3498049537ULL;value.context_id=1;return value;}();
inline const std::array<std::uint64_t,5> page_when={1000,2000,3000,4000,5000};
/** @brief 链材料只调用中央规范Hash purpose，返回实际Owned的新chain。 */
inline std::array<std::uint8_t,32> chain_page(dummy_ref::messages::Evidence& value,const std::array<std::uint8_t,32>& previous){
 value.previous_chain_sha256().emplace().assign(previous.data(),previous.size());
 using Traits=dp::protocol::canonical::Traits<dummy_ref::messages::Evidence>;
 std::vector<std::uint8_t> material(32+Traits::kMaxEncodedSize);std::copy(previous.begin(),previous.end(),material.begin());std::size_t written=0;
 if(!Traits::encode(value,material.data()+32,material.size()-32,&written,dp::protocol::canonical::Purpose::kEvidenceHash))std::abort();
 std::array<std::uint8_t,32> next{};SHA256(material.data(),32+written,next.data());value.chain_sha256().emplace().assign(next.data(),next.size());return next;
}
inline const auto pages=[] {
 std::array<dummy_ref::messages::Evidence,5> result{};
 std::array<std::uint8_t,192> seed{};seed[0]=1;
 for(unsigned i=0;i<8;++i)seed[8+i]=static_cast<std::uint8_t>(source.boot_id>>(i*8));
 std::copy(client_id.begin(),client_id.end(),seed.begin()+16);
 const std::array<std::array<std::uint8_t,32>,4> hashes={contract_hash,model_hash,profile_hash,firmware_hash};
 for(unsigned i=0;i<4;++i)std::copy(hashes[i].begin(),hashes[i].end(),seed.begin()+32+i*32);
 std::array<std::uint8_t,32> previous{};SHA256(seed.data(),seed.size(),previous.data());
 for(unsigned i=0;i<result.size();++i){auto value=owned_fixture::evidence();
  value.run_id()=1;value.attempt_id()=1;value.attempt_kind()=1;value.evidence_seq()=i;value.final()=true;
  value.category()=i<2?1:i==2?2:i==3?3:4;value.motion_epoch()=i==0||i==4?1:i==1?2:3;value.group_count()=i==4?4:i+1;
  if(i!=4){value.group_id()=i+1;value.expected_mask()=63;value.submitted_mask()=63;value.sent_mask()=63;
   const std::array<std::uint64_t,6> times{100,100,100,100,100,100};
   value.queued_at_us().emplace().assign(times.data(),6);value.taken_at_us().emplace().assign(times.data(),6);value.submitted_at_us().emplace().assign(times.data(),6);value.sent_at_us().emplace().assign(times.data(),6);
  }else{value.final_state()=4;value.release_committed()=true;}
  if(i>=2){value.first_failure_source()=0x1003;value.first_failure_axis_mask()=63;}
  previous=chain_page(value,previous);result[i]=value;
 }return result;
}();
inline const auto fault_state=[] {auto value=owned_fixture::state();
 value.motion_epoch()=3;value.management_epoch()=6;value.core_state()=9;value.profile_id()=1;value.domain_device_id().emplace().assign(device.data(),device.size());
 for(auto& joint:*value.joints()){joint.position_valid_quality()=false;joint.raw_position_valid_quality()=false;}
 return value;
}();
inline const auto fault_status=[] {auto value=owned_fixture::status();
 value.motion_epoch()=3;value.management_epoch()=6;value.core_state()=9;value.profile_id()=1;value.configured_mask()=63;value.domain_device_id().emplace().assign(device.data(),device.size());
 value.run_id()=1;value.attempt_id()=1;value.attempt_kind()=1;value.final_state()=4;value.release_committed()=true;value.failure_source()=0x1003;value.failure_axis_mask()=63;value.group_count()=4;
 value.contract_sha256().emplace().assign(contract_hash.data(),32);value.model_sha256().emplace().assign(model_hash.data(),32);value.profile_sha256().emplace().assign(profile_hash.data(),32);value.firmware_sha256().emplace().assign(firmware_hash.data(),32);
 return value;
}();
constexpr std::uint64_t fault_state_when=6000,fault_status_when=6000;
}

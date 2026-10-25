/** @brief 当前SDK合成故障输入；完全不读取历史抓取或复用历史wire字节。 */
#pragma once
#include "dummy_ref/wire_validation.hpp"
#include <algorithm>
#include <array>
#include <cstdlib>
#include <openssl/sha.h>
namespace current_fixture {
namespace wire=dp::iface;
template<std::size_t N> struct CBytes { std::uint8_t value[N]; explicit CBytes(const std::array<std::uint8_t,N>& input) {std::copy(input.begin(),input.end(),value);} };
template<std::size_t N> std::array<std::uint8_t,N> tag(std::uint8_t value) {
 std::array<std::uint8_t,N> result{};result[0]=value;return result;
}
inline const auto client_id=tag<16>(2);
inline const auto contract_hash=tag<32>(3);
inline const auto model_hash=tag<32>(4);
inline const auto profile_hash=tag<32>(5);
inline const auto firmware_hash=tag<32>(6);
inline const std::array<std::uint64_t,5> page_when={1000,2000,3000,4000,5000};
inline const auto pages=[] {
 std::array<std::array<std::uint8_t,wire::DummyAttemptEvidenceV2Codec::kEncodedSize>,5> result{};
 std::array<std::uint8_t,192> seed{};seed[0]=1;
 const std::uint64_t boot=3498049537ULL;
 for(unsigned i=0;i<8;++i)seed[8+i]=static_cast<std::uint8_t>(boot>>(i*8));
 std::copy(client_id.begin(),client_id.end(),seed.begin()+16);
 const std::array<std::array<std::uint8_t,32>,4> hashes={contract_hash,model_hash,profile_hash,firmware_hash};
 for(unsigned i=0;i<4;++i)std::copy(hashes[i].begin(),hashes[i].end(),seed.begin()+32+i*32);
 std::array<std::uint8_t,32> previous{};SHA256(seed.data(),seed.size(),previous.data());
 for(unsigned i=0;i<result.size();++i) {
  wire::DummyAttemptEvidenceV2 f{};f.set_schema_major(dummy_ref::protocol::kSchemaMajor);f.set_schema_minor(dummy_ref::protocol::kSchemaMinor);
  f.set_run_id(1);f.set_attempt_id(1);f.set_attempt_kind(1);f.set_evidence_seq(i);f.set_final(true);
  f.set_category(i<2?1:i==2?2:i==3?3:4);f.set_motion_epoch(i==0||i==4?1:i==1?2:3);
  f.set_group_count(i==4?4:i+1);
  if(i!=4) {
   f.set_group_id(i+1);f.set_expected_mask(63);f.set_submitted_mask(63);f.set_sent_mask(63);
   std::uint64_t times[6]={100,100,100,100,100,100};f.set_queued_at_us(times);f.set_taken_at_us(times);f.set_submitted_at_us(times);f.set_sent_at_us(times);
  } else {f.set_final_state(4);f.set_release_committed(true);}
  if(i>=2){f.set_first_failure_source(0x1003);f.set_first_failure_axis_mask(63);}
  f.set_previous_chain_sha256(CBytes<32>(previous).value);
  if(wire::DummyAttemptEvidenceV2Codec::encode(f,result[i].data(),result[i].size())!=wire::CodecResult::kOk)std::abort();
  constexpr auto prefix=wire::DummyAttemptEvidenceV2Codec::kOffset_previous_chain_sha256;
  std::array<std::uint8_t,32+prefix+1> input{};std::copy(previous.begin(),previous.end(),input.begin());std::copy_n(result[i].begin(),prefix,input.begin()+32);input.back()=result[i][wire::DummyAttemptEvidenceV2Codec::kOffset_evidence_lost];
  SHA256(input.data(),input.size(),previous.data());f.set_chain_sha256(CBytes<32>(previous).value);
  if(wire::DummyAttemptEvidenceV2Codec::encode(f,result[i].data(),result[i].size())!=wire::CodecResult::kOk)std::abort();
 }
 return result;
}();
inline const auto fault_state=[] {
 wire::DummyJointStateV2 f{};f.set_schema_major(dummy_ref::protocol::kSchemaMajor);f.set_schema_minor(dummy_ref::protocol::kSchemaMinor);
 f.set_boot_epoch(3498049537ULL);f.set_motion_epoch(3);f.set_management_epoch(6);f.set_core_state(9);f.set_profile_id(1);f.set_configured_mask(63);const auto device=tag<16>(1);f.set_device_id(CBytes<16>(device).value);
 std::array<std::uint8_t,wire::DummyJointStateV2Codec::kEncodedSize> result{};
 if(wire::DummyJointStateV2Codec::encode(f,result.data(),result.size())!=wire::CodecResult::kOk)std::abort();return result;
}();
inline const auto fault_status=[] {
 wire::DummyDeviceStatusV2 f{};f.set_schema_major(dummy_ref::protocol::kSchemaMajor);f.set_schema_minor(dummy_ref::protocol::kSchemaMinor);
 f.set_boot_epoch(3498049537ULL);f.set_motion_epoch(3);f.set_management_epoch(6);f.set_core_state(9);f.set_profile_id(1);f.set_configured_mask(63);const auto device=tag<16>(1);f.set_device_id(CBytes<16>(device).value);
 f.set_run_id(1);f.set_attempt_id(1);f.set_attempt_kind(1);f.set_final_state(4);f.set_release_committed(true);f.set_failure_source(0x1003);f.set_failure_axis_mask(63);f.set_group_count(4);
 f.set_contract_sha256(CBytes<32>(contract_hash).value);f.set_model_sha256(CBytes<32>(model_hash).value);f.set_profile_sha256(CBytes<32>(profile_hash).value);f.set_firmware_sha256(CBytes<32>(firmware_hash).value);
 std::array<std::uint8_t,wire::DummyDeviceStatusV2Codec::kEncodedSize> result{};
 if(wire::DummyDeviceStatusV2Codec::encode(f,result.data(),result.size())!=wire::CodecResult::kOk)std::abort();return result;
}();
constexpr std::uint64_t fault_state_when=6000,fault_status_when=6000;
}

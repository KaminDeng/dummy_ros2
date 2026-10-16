#include "fixtures/ros05_fault146/original-once-decoded-146-payloads-fixture.hpp"
#include "fixtures/ros05_fault146/original-once-decoded-current-fault-observations.hpp"
#include "dummy_motion_client/dp_management_transport.hpp"
#include "dummy_ros_control/raw_evidence_collector.hpp"
#include "dummy_ros_control/device_management_channel.hpp"
#include "dp_dummy_robot_v2.hpp"
#include <algorithm>
#include <cstdio>
#include <chrono>
#include <thread>
#include <unistd.h>
#include <openssl/sha.h>
static bool canonical_bad_tail_never_confirms(const char* base,unsigned mode) {
 using namespace dummy_ros_control;namespace wire=dp::protocol::dummy_robot_v2;
 RawEvidenceIdentity trust{original_fixture::client_id,original_fixture::contract_hash,original_fixture::model_hash,original_fixture::profile_hash,original_fixture::firmware_hash};
 DeviceManagementChannel channel(TransportParameters{});RawEvidenceCollector collector(trust,&channel);
 const std::string path=std::string(base)+"-negative-"+std::to_string(mode);
 if(!collector.start(path)||!collector.expect_run(1))return false;
 std::array<std::uint8_t,32> previous{};
 std::copy_n(original_fixture::pages[0].begin()+307,32,previous.begin());
 for(std::size_t i=0;i<original_fixture::pages.size();++i) {
  auto raw=original_fixture::pages[i];wire::DummyAttemptEvidenceV2 f{};wire::DummyAttemptEvidenceV2Codec::decode(raw.data(),raw.size(),&f);
  if(i==143 && mode==0)f.set_sent_mask(31);
  if(i==144 && mode==1)f.set_sent_mask(31);
  if(i==143 && mode==2)f.set_motion_epoch(2);
  if(i==145 && mode==3)f.set_release_committed(false);
  if(i==145 && mode==4)f.set_motion_epoch(2);
  if(i==145 && mode==5)f.set_final_state(5);
  std::uint8_t prev_bytes[32];std::copy(previous.begin(),previous.end(),prev_bytes);f.set_previous_chain_sha256(prev_bytes);wire::DummyAttemptEvidenceV2Codec::encode(f,raw.data(),raw.size());
  std::array<std::uint8_t,340> input{};std::copy(previous.begin(),previous.end(),input.begin());std::copy_n(raw.begin(),307,input.begin()+32);input.back()=raw[371];SHA256(input.data(),input.size(),previous.data());std::uint8_t next_bytes[32];std::copy(previous.begin(),previous.end(),next_bytes);f.set_chain_sha256(next_bytes);wire::DummyAttemptEvidenceV2Codec::encode(f,raw.data(),raw.size());
  RawEvidencePage p{};p.raw=raw;p.boot_epoch=3498049537ULL;p.session_id=71;p.run_id=1;p.attempt_kind=1;p.evidence_seq=f.evidence_seq();p.group_count=f.group_count();p.category=f.category();p.final_state=f.final_state();p.terminal_reason=f.terminal_reason();p.final=f.final();p.evidence_lost=f.evidence_lost();std::copy(std::begin(f.previous_chain_sha256()),std::end(f.previous_chain_sha256()),p.previous_chain_sha256.begin());p.chain_sha256=previous;
  if(!collector.offer_page(p,original_fixture::page_when[i])){collector.stop();return collector.failure()!=RawEvidenceCollector::Failure::kNone;}
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(1);while(collector.pages_written()<i+1&&std::chrono::steady_clock::now()<deadline)std::this_thread::yield();if(collector.pages_written()<i+1)return false;
 }
 collector.stop();return !channel.raw_evidence_terminal_flushed().full_fault_groups_valid;
}
int main(int argc,char**argv) {
 char temp[]="/tmp/dummy-fault-receipt-XXXXXX";std::string permanent_path;
 if(argc==1) {char* dir=mkdtemp(temp);if(!dir)return 1;permanent_path=std::string(dir)+"/component.raw";}
 const char* output_path=argc==1?permanent_path.c_str():argv[1];
 using namespace dummy_ros_control;
 namespace wire=dp::protocol::dummy_robot_v2;
 if(argc!=1 && argc!=2)return 1;
 RawEvidenceIdentity trust{};
 trust.client_id=original_fixture::client_id;trust.contract_hash=original_fixture::contract_hash;
 trust.model_hash=original_fixture::model_hash;trust.profile_hash=original_fixture::profile_hash;trust.firmware_hash=original_fixture::firmware_hash;
 DeviceManagementChannel channel(TransportParameters{});RawEvidenceCollector collector(trust,&channel);
 if(!collector.start(output_path)||!collector.expect_run(1))return 2;
 for(const auto& raw:original_fixture::pages) {
  wire::DummyAttemptEvidenceV2 f{};
  if(wire::DummyAttemptEvidenceV2Codec::decode(raw.data(),raw.size(),&f)!=wire::CodecResult::kOk)return 3;
  RawEvidencePage p{};p.raw=raw;p.boot_epoch=3498049537ULL;p.session_id=71;p.run_id=f.run_id();p.parent_run_id=f.parent_run_id();p.attempt_kind=f.attempt_kind();p.evidence_seq=f.evidence_seq();p.group_count=f.group_count();p.category=f.category();p.final_state=f.final_state();p.terminal_reason=f.terminal_reason();p.final=f.final();p.evidence_lost=f.evidence_lost();
  std::copy(std::begin(f.previous_chain_sha256()),std::end(f.previous_chain_sha256()),p.previous_chain_sha256.begin());
  std::copy(std::begin(f.chain_sha256()),std::end(f.chain_sha256()),p.chain_sha256.begin());
  if(!collector.offer_page(p,original_fixture::page_when[f.evidence_seq()])) {std::printf("offer_failed=%u seq=%u\n",static_cast<unsigned>(collector.failure()),f.evidence_seq());return 4;}
  // 原有4096字节队列容量不改；等真正异步writer，单页观察后继续。
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(1);
  while(collector.pages_written()<f.evidence_seq()+1 && std::chrono::steady_clock::now()<deadline)std::this_thread::yield();
  if(collector.pages_written()<f.evidence_seq()+1)return 5;
 }
 collector.stop();const auto receipt=channel.raw_evidence_terminal_flushed();
 std::printf("actualCollectorFailure=%u pages=%llu run=%llu final=%u release=%u chainValid=%u firstFailureSource=%u expectedOriginal4099 safetyValid=%u safetyEpoch=%u\n",static_cast<unsigned>(collector.failure()),static_cast<unsigned long long>(collector.pages_written()),static_cast<unsigned long long>(receipt.run_id),receipt.final_state,receipt.release_committed,receipt.chain_valid,receipt.first_failure_source,receipt.safety_hold_valid,receipt.safety_hold_motion_epoch);
 if(collector.failure()!=RawEvidenceCollector::Failure::kNone || receipt.run_id!=1 || receipt.final_state!=4 || !receipt.release_committed || !receipt.chain_valid)return 6;
 if(receipt.first_failure_source!=0x1003U)return 36;
 if(!receipt.full_fault_groups_valid || receipt.attempt_motion_epoch!=1 || receipt.fault_stop_motion_epoch!=3)return 38;
 dummy_motion_client::RawTerminalIdentity proof{receipt.boot_epoch,receipt.session_id,receipt.run_id,receipt.parent_run_id,receipt.attempt_kind,receipt.final_state,receipt.first_failure_source,receipt.first_failure_axis_mask,receipt.evidence_lost,receipt.motion_complete,receipt.release_committed,receipt.chain_valid,receipt.safety_hold_motion_epoch,receipt.safety_hold_valid,receipt.attempt_motion_epoch,receipt.fault_stop_motion_epoch,receipt.terminal_received_us,receipt.terminal_chain,receipt.full_fault_groups_valid,receipt.identity_hashes,receipt.terminal_group_count};
 const auto fp=channel.raw_evidence_first_page_written();
 dummy_motion_client::RawFirstPageWritten first{fp.boot_epoch,fp.session_id,fp.run_id,fp.motion_epoch,fp.evidence_seq,fp.group_count,fp.attempt_kind,fp.category,fp.expected_mask,fp.submitted_mask,fp.sent_mask,fp.failed_mask,fp.final,fp.evidence_lost,fp.chain_valid,fp.full_six,fp.strict_refresh_successor};
 dummy_motion_client::DeviceObservation observation;observation.raw_cleanup_producer_valid=true;
 observation.state.assign(original_fixture::fault_state.begin(),original_fixture::fault_state.end());observation.status.assign(original_fixture::fault_status.begin(),original_fixture::fault_status.end());observation.state_received_us=original_fixture::fault_state_when;observation.status_received_us=original_fixture::fault_status_when;

 std::array<std::uint8_t,16> device{};device[0]=1;
 const auto now=std::max(observation.state_received_us,observation.status_received_us);
 if(!dummy_motion_client::raw_fault_cleanup_matches(proof,first,observation,3498049537ULL,71,1,2,1,63,device,now))return 39;
 for(unsigned n=0;n<26;++n) {
  auto r=proof;auto ffirst=first;auto o=observation;auto expected_boot=3498049537ULL;std::uint64_t expected_session=71,expected_run=1;auto sample_now=now;
  switch(n) {
   case 0:expected_boot++;break;case 1:expected_session++;break;case 2:expected_run++;break;
   case 3:r.attempt_motion_epoch=2;break;case 4:r.fault_stop_motion_epoch=4;break;
   case 5:r.first_failure_source=3;break;case 6:r.first_failure_source=0x7777;break;
   case 7:r.first_failure_axis_mask=31;break;case 8:r.full_fault_groups_valid=false;break;
   case 9:r.release_committed=false;break;case 10:r.chain_valid=false;break;
   case 11:r.terminal_chain={};break;case 12:r.evidence_lost=true;break;
   case 13:r.motion_complete=true;break;case 14:r.parent_run_id=9;break;
   case 15:sample_now+=250001;break;case 16:o.state_received_us=sample_now+1;break;
   case 17:o.state={};break;case 18:r.identity_hashes[0][0]^=1;break;
   case 19:r.terminal_received_us=sample_now+1;break;
   case 20:o.raw_cleanup_producer_valid=false;break;case 21:ffirst.chain_valid=false;break;case 22:ffirst.motion_epoch=2;break;
   case 23:{wire::DummyDeviceStatusV2 f{};wire::DummyDeviceStatusV2Codec::decode(o.status.data(),o.status.size(),&f);std::uint8_t bad[32]{};bad[0]=1;f.set_evidence_chain_sha256(bad);wire::DummyDeviceStatusV2Codec::encode(f,o.status.data(),o.status.size());break;}
   case 24:{wire::DummyJointStateV2 f{};wire::DummyJointStateV2Codec::decode(o.state.data(),o.state.size(),&f);f.set_core_state(4);wire::DummyJointStateV2Codec::encode(f,o.state.data(),o.state.size());break;}
   case 25:{wire::DummyDeviceStatusV2 f{};wire::DummyDeviceStatusV2Codec::decode(o.status.data(),o.status.size(),&f);f.set_profile_id(201);wire::DummyDeviceStatusV2Codec::encode(f,o.status.data(),o.status.size());break;}
  }
  if(dummy_motion_client::raw_fault_cleanup_matches(r,ffirst,o,expected_boot,expected_session,expected_run,2,1,63,device,sample_now))return 40+n;
 }
 for(unsigned n=0;n<6;++n)if(!canonical_bad_tail_never_confirms(output_path,n))return 80+n;
 std::puts("strong_original_fault146_and_actual_current_observation_match + 26pairednegatives+6canonical-valid-chain-bad-tail PASS");
 return 0;
}

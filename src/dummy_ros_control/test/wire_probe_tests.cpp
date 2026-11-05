/** @file wire_probe_tests.cpp
 * @brief 真实Owned的中央Root/canonical往返、64位业务身份、展开角度及显式Source夹具。
 * @details 纯内存测试；不调用已退役raw service bridge，不把合成Source称为认证或远端接纳。 */
#include "owned_message_fixtures.hpp"
#include "dummy_ref/root_transport.hpp"
#include "canonical_records.h"
#include <array>
#include <cmath>
#include <cstring>
namespace {
namespace selected=dp::protocol::selection;
namespace messages=dummy_ref::messages;
using Result=flatbuffers::bounded::Status;
/** @brief 通过正式Root traits验证正文和独立来源，所有位布局由SDK处理。 */
template<class Descriptor>
bool root_roundtrip(const typename Descriptor::Owned& input,typename Descriptor::Owned& output,dp::comm::SourceId* source_out=nullptr){
 alignas(8) std::array<std::uint8_t,Descriptor::kWorkspaceSize> workspace{};
 dp::comm::SourceId source{};source.device.bytes[0]=0xB2;source.boot_id=9007199254740993ULL;source.context_id=7;
 flatbuffers::bounded::SdkMetadata metadata{},decoded{};
 const auto policy=Descriptor::policy();
 const auto role=policy.mode==flatbuffers::bounded::SdkMode::sample?dp::comm::detail::Role::Sample:
  policy.mode==flatbuffers::bounded::SdkMode::request?dp::comm::detail::Role::Request:dp::comm::detail::Role::Response;
 std::uint32_t token=0;if(!dp::comm::detail::encode_token(role,17,token))return false;
 dp::comm::detail::RootMetadata input_metadata{};input_metadata.token=token;
 if(policy.mode==flatbuffers::bounded::SdkMode::request)input_metadata.requester=dp::comm::detail::RootMetadata::Requester{source,9007199254740995ULL};
 else input_metadata.source=source;
 if(Descriptor::kTargetRequired)input_metadata.target=source;
 metadata=dp::comm::detail::to_wire(input_metadata);
 flatbuffers::bounded::BufferView bytes{};
 if(Descriptor::Traits::encode(input,policy,metadata,workspace.data(),workspace.size(),bytes)!=Result::ok ||
    Descriptor::Traits::decode(bytes.data,bytes.size,policy,output,decoded)!=Result::ok)return false;
 const auto decoded_metadata=dp::comm::detail::from_wire(decoded);
 if(!decoded_metadata.source&&!decoded_metadata.requester)return false;
 const auto restored=decoded_metadata.source?*decoded_metadata.source:decoded_metadata.requester->source;
 if(decoded_metadata.requester&&decoded_metadata.requester->client_instance!=9007199254740995ULL)return false;
 if(source_out)*source_out=restored;
 return restored.device.bytes[0]==0xB2&&restored.boot_id==source.boot_id&&restored.context_id==7;
}
/** @brief 本地记录只用中央canonical；实际长度不等于最大容量。 */
template<class T>bool record_roundtrip(const T& value,T& output){
 using Traits=dp::protocol::canonical::Traits<T>;std::array<std::uint8_t,Traits::kMaxEncodedSize> bytes{};std::size_t written=0;
 return Traits::encode(value,bytes.data(),bytes.size(),&written)&&Traits::decode(bytes.data(),written,&output);
}
}
int main(){
 messages::Command command;
 command.boot_epoch()=9007199254740993ULL;command.session_id()=9007199254740995ULL;
 command.profile_id()=0;command.reference_epoch()=9;command.motion_epoch()=10;command.producer_seq()=11;command.operation_id()=0;
 command.freshness_ticket()=9007199254740997ULL;
 std::array<std::uint8_t,16> identity{};identity[0]=1;command.domain_device_id().emplace().assign(identity.data(),16);
 identity[0]=2;command.client_instance_id().emplace().assign(identity.data(),16);
 const double positions[6]{0.1,-0.2,0.7,-0.4,0.5,9.42477796076938};
 auto& joints=command.joints().emplace();
 for(unsigned a=0;a<6;++a){auto* joint=joints.emplace_back();joint->joint_id()=a;joint->configured()=true;joint->position_rad()=positions[a];}
 messages::Command decoded;
 if(!root_roundtrip<selected::dp_protocol_dummy_JointCommand>(command,decoded)||
    !record_roundtrip(command,decoded))return 1;
 if(*decoded.boot_epoch()!=9007199254740993ULL||*decoded.session_id()!=9007199254740995ULL||
    *decoded.freshness_ticket()!=9007199254740997ULL||*(*decoded.joints())[5].position_rad()!=positions[5])return 3;
 auto status=owned_fixture::status();status.configured_mask()=63;status.capabilities()=1ULL<<2;
 status.control_period_us()=10000;status.state_publish_period_us()=10000;
 std::array<std::uint8_t,32> hash{};hash[0]=1;status.compiled_policy_sha256().emplace().assign(hash.data(),32);
 std::array<double,6> values{};values.fill(-1);status.position_min_rad().emplace().assign(values.data(),6);
 values.fill(1);status.position_max_rad().emplace().assign(values.data(),6);status.velocity_max_rad_s().emplace().assign(values.data(),6);
 values.fill(2);status.acceleration_max_rad_s().emplace().assign(values.data(),6);
 messages::Status restored_status;
 if(!root_roundtrip<selected::dp_protocol_dummy_DeviceStatus>(status,restored_status)||!record_roundtrip(status,restored_status)||
    (*restored_status.compiled_policy_sha256())[0]!=1||(*restored_status.position_min_rad())[0]!=-1||
    (*restored_status.position_max_rad())[5]!=1||(*restored_status.velocity_max_rad_s())[2]!=1||
    (*restored_status.acceleration_max_rad_s())[4]!=2||*restored_status.control_period_us()!=10000||
    *restored_status.state_publish_period_us()!=10000)return 13;
 auto queued=owned_fixture::response();queued.request_id()=9007199254740999ULL;
 queued.receipt()=1;queued.result_code()=8;queued.transaction_phase()=1;
 messages::ManageResponse decoded_response;
 if(!root_roundtrip<selected::dp_protocol_dummy_ManageResponse>(queued,decoded_response)||!record_roundtrip(queued,decoded_response))return 4;
 if(*decoded_response.request_id()!=9007199254740999ULL||*decoded_response.receipt()!=1||*decoded_response.transaction_phase()!=1)return 6;
 auto evidence=owned_fixture::evidence();evidence.category()=1;evidence.run_id()=9007199254740993ULL;
 evidence.evidence_seq()=17;evidence.expected_mask()=1;evidence.evidence_lost()=true;
 messages::Evidence decoded_evidence;
 if(!root_roundtrip<selected::dp_protocol_dummy_AttemptEvidence>(evidence,decoded_evidence)||!record_roundtrip(evidence,decoded_evidence)||
    *decoded_evidence.run_id()!=*evidence.run_id()||*decoded_evidence.evidence_seq()!=17||!*decoded_evidence.evidence_lost())return 15;
 // 原monitor响应业务断言仍保留；实际RPC来源/回调另由真实Root PTY往返覆盖。
 auto monitor=messages::make_manage_request();monitor.operation()=0;messages::ManageRequest decoded_request;dp::comm::SourceId caller{};
 if(!root_roundtrip<selected::dp_protocol_dummy_ManageRequest>(monitor,decoded_request,&caller))return 8;
 auto snapshot=owned_fixture::response();snapshot.client_instance_id().emplace().assign(caller.device.bytes,16);
 snapshot.receipt()=1;snapshot.result_code()=0;snapshot.operation_id()=9;snapshot.transaction_phase()=3;snapshot.session_id()=9007199254740995ULL;
 if(!root_roundtrip<selected::dp_protocol_dummy_ManageResponse>(snapshot,decoded_response)||
    (*decoded_response.client_instance_id())[0]!=0xB2||*decoded_response.operation_id()!=9||
    *decoded_response.transaction_phase()!=3||*decoded_response.result_code()!=0||*decoded_response.session_id()!=9007199254740995ULL)return 11;
 using CommandRecord=dp::protocol::canonical::Traits<messages::Command>;
 std::array<std::uint8_t,CommandRecord::kMaxEncodedSize> bytes{};std::size_t written=0;
 if(!CommandRecord::encode(command,bytes.data(),bytes.size(),&written))return 12;
 decoded.boot_epoch()=42;
 if(CommandRecord::decode(bytes.data(),written-1,&decoded)||*decoded.boot_epoch()!=42)return 12;
 // 退役业务schema号拒绝改为实际中央记录跨类型拒绝，同样不得部分改写输出。
 std::array<std::uint8_t,dp::protocol::canonical::Traits<messages::Evidence>::kMaxEncodedSize> other{};
 if(!dp::protocol::canonical::Traits<messages::Evidence>::encode(evidence,other.data(),other.size(),&written)||
    CommandRecord::decode(other.data(),written,&decoded)||*decoded.boot_epoch()!=42)return 14;
 return 0;
}

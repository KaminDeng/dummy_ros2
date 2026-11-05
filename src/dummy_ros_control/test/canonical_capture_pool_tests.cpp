/** @file canonical_capture_pool_tests.cpp
 * @brief 中央记录的真实writer/flush/容量边界回归，不使用旧业务offset。
 */
#include "dummy_ros_control/raw_evidence_collector.hpp"
#include "dummy_ros_control/device_management_channel.hpp"
#include "owned_message_fixtures.hpp"
#include <openssl/sha.h>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <vector>
#include <iostream>
#include <thread>
#include <unistd.h>
using namespace dummy_ros_control;
#define CHECK(value) do{if(!(value)){std::cerr<<"CHECK failed line "<<__LINE__<<": "#value<<'\n';std::abort();}}while(false)
namespace {
struct Directory {
  std::string path;
  Directory(){char name[]="/tmp/dummy-canonical-capture-XXXXXX";const auto* made=::mkdtemp(name);CHECK(made);path=made;}
  ~Directory(){::unlink((path+"/capture.raw").c_str());::rmdir(path.c_str());}
};
struct Barrier {std::atomic<bool> entered{false},release{false};};
bool terminal_barrier(void* context) noexcept {
  auto& barrier=*static_cast<Barrier*>(context);barrier.entered=true;
  while(!barrier.release.load())std::this_thread::yield();return true;
}
int flush_barrier(std::FILE* file,void* context) noexcept {terminal_barrier(context);return std::fflush(file);}
template<class Predicate>void wait_for(Predicate predicate){
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(3);
  while(!predicate()){CHECK(std::chrono::steady_clock::now()<deadline);std::this_thread::sleep_for(std::chrono::milliseconds(1));}
}
dp::comm::SourceId source(){dp::comm::SourceId value{};value.device.bytes[0]=0xa5;value.device.bytes[15]=0x6c;value.boot_id=7U;value.context_id=93U;return value;}
std::array<std::uint8_t,32> seed(const RawEvidenceIdentity& identity){
  std::array<std::uint8_t,192> bytes{};capture_detail::put(bytes.data(),1U,8U);capture_detail::put(bytes.data()+8U,7U,8U);
  std::copy(identity.client_id.begin(),identity.client_id.end(),bytes.begin()+16U);
  std::copy(identity.contract_hash.begin(),identity.contract_hash.end(),bytes.begin()+32U);
  std::copy(identity.model_hash.begin(),identity.model_hash.end(),bytes.begin()+64U);
  std::copy(identity.profile_hash.begin(),identity.profile_hash.end(),bytes.begin()+96U);
  std::copy(identity.firmware_hash.begin(),identity.firmware_hash.end(),bytes.begin()+128U);
  std::array<std::uint8_t,32> hash{};SHA256(bytes.data(),bytes.size(),hash.data());return hash;
}
void seal(dummy_ref::messages::Evidence& page,const std::array<std::uint8_t,32>& previous){
  (void)page.previous_chain_sha256().emplace().assign(previous.data(),previous.size());
  using Traits=dp::protocol::canonical::Traits<dummy_ref::messages::Evidence>;
  std::array<std::uint8_t,32U+Traits::kMaxEncodedSize> bytes{};std::copy(previous.begin(),previous.end(),bytes.begin());std::size_t written=0U;
  CHECK(Traits::encode(page,bytes.data()+32U,bytes.size()-32U,&written,dp::protocol::canonical::Purpose::kEvidenceHash));
  std::array<std::uint8_t,32> hash{};SHA256(bytes.data(),32U+written,hash.data());(void)page.chain_sha256().emplace().assign(hash.data(),hash.size());
}
dummy_ref::messages::Evidence first(const RawEvidenceIdentity& identity){
  auto page=owned_fixture::evidence();page.run_id()=1U;page.attempt_id()=1U;page.attempt_kind()=1U;page.motion_epoch()=1U;
  page.group_id()=1U;page.group_count()=1U;page.category()=1U;page.final()=1U;
  page.expected_mask()=63U;page.submitted_mask()=63U;page.sent_mask()=63U;
  const std::array<std::uint64_t,6> queued{1,1,1,1,1,1},taken{2,2,2,2,2,2},submitted{3,3,3,3,3,3},sent{4,4,4,4,4,4};
  (void)page.queued_at_us().emplace().assign(queued.data(),6);(void)page.taken_at_us().emplace().assign(taken.data(),6);
  (void)page.submitted_at_us().emplace().assign(submitted.data(),6);(void)page.sent_at_us().emplace().assign(sent.data(),6);
  seal(page,seed(identity));return page;
}
dummy_ref::messages::Evidence terminal(const dummy_ref::messages::Evidence& prior){
  auto page=owned_fixture::evidence();page.run_id()=1U;page.attempt_id()=1U;page.attempt_kind()=1U;page.motion_epoch()=1U;
  page.evidence_seq()=1U;page.group_count()=1U;page.category()=4U;page.final()=1U;page.final_state()=6U;page.motion_complete()=true;
  std::array<std::uint8_t,32> previous{};std::copy(prior.chain_sha256()->begin(),prior.chain_sha256()->end(),previous.begin());seal(page,previous);return page;
}
std::vector<std::uint8_t> read(const std::string& path){std::ifstream input(path,std::ios::binary);return {std::istreambuf_iterator<char>(input),std::istreambuf_iterator<char>()};}
void completed_flush(){
  Directory directory;Barrier barrier;DeviceManagementChannel channel{TransportParameters{}};RawEvidenceIdentity identity{};
  RawEvidenceCollector collector(identity,&channel,terminal_barrier,&barrier);CHECK(collector.start(directory.path+"/capture.raw"));CHECK(collector.expect_run(1U));
  const auto page=first(identity);CHECK(collector.offer_page(page,source(),2U,100U));
  wait_for([&]{return collector.pages_written()==1U;});
  CHECK(collector.offer_page(terminal(page),source(),2U,101U));wait_for([&]{return barrier.entered.load();});
  CHECK(collector.terminal_written_run_id()==0U);CHECK(channel.raw_evidence_terminal_flushed().run_id==0U);
  auto status=owned_fixture::status();CHECK(collector.offer_observation(status,source(),102U));
  barrier.release=true;wait_for([&]{return collector.written()==3U;});collector.stop();
  CHECK(collector.failure()==RawEvidenceCollector::Failure::kNone);CHECK(collector.terminal_written_run_id()==1U);
  CHECK(channel.raw_evidence_terminal_flushed().motion_complete);
  const auto bytes=read(directory.path+"/capture.raw");CHECK(std::equal(kCaptureMagic.begin(),kCaptureMagic.end(),bytes.begin()));
  std::size_t at=8U,count=0U;
  while(at<bytes.size()){
    CaptureRecordHeader header;CHECK(decode_capture_header(bytes.data()+at,bytes.size()-at,&header));at+=kCaptureHeaderSize;
    CHECK(header.source==source());CHECK(at+header.size<=bytes.size());
    if(header.kind==CaptureRecordKind::kStatus){dummy_ref::messages::Status decoded;CHECK(dp::protocol::canonical::Traits<dummy_ref::messages::Status>::decode(bytes.data()+at,header.size,&decoded));CHECK(decoded.real_motion_permitted() && !*decoded.real_motion_permitted());}
    else {dummy_ref::messages::Evidence decoded;CHECK(dp::protocol::canonical::Traits<dummy_ref::messages::Evidence>::decode(bytes.data()+at,header.size,&decoded));CHECK(*decoded.run_id()==1U);}
    at+=header.size;++count;
  }
  CHECK(count==3U);
}
void full_capacity(){
  Directory directory;Barrier barrier;RawEvidenceIdentity identity{};RawEvidenceCollector collector(identity,nullptr,nullptr,&barrier,flush_barrier);
  CHECK(collector.start(directory.path+"/capture.raw"));CHECK(collector.offer_page(first(identity),source(),2U,100U));wait_for([&]{return barrier.entered.load();});
  auto status=owned_fixture::status();std::array<std::uint8_t,dp::protocol::canonical::Traits<dummy_ref::messages::Status>::kMaxEncodedSize> encoded{};std::size_t written=0U;
  CHECK(dp::protocol::canonical::Traits<dummy_ref::messages::Status>::encode(status,encoded.data(),encoded.size(),&written));CHECK(written==RawEvidenceCollector::kMaxRawSize);
  for(std::size_t i=1U;i<RawEvidenceCollector::kCapacity;++i)CHECK(collector.offer_observation(status,source(),100U+i));
  CHECK(!collector.offer_observation(status,source(),999U));CHECK(collector.failure()==RawEvidenceCollector::Failure::kOverflow);
  barrier.release=true;wait_for([&]{return collector.written()==RawEvidenceCollector::kCapacity;});collector.stop();CHECK(collector.failure()==RawEvidenceCollector::Failure::kOverflow);
  const auto bytes=read(directory.path+"/capture.raw");std::size_t at=8U,count=0U;
  while(at<bytes.size()){CaptureRecordHeader header;CHECK(decode_capture_header(bytes.data()+at,bytes.size()-at,&header));at+=kCaptureHeaderSize+header.size;++count;}
  CHECK(at==bytes.size() && count==256U);
}
}
int main(){completed_flush();full_capacity();std::cout<<"canonical capture flush/source/presence/256-record capacity passed\n";}

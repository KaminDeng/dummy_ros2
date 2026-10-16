#define main original_wire_management_main
#include "wire_management_tests.cpp"
#undef main
#include "dummy_ros_control/diagnostic_scope.hpp"
#include "dummy_ros_control/diagnostic_json.hpp"
#include <atomic>
#include <cassert>
#include <thread>
int main() {
  dummy_ros_control::DiagnosticJournal journal;
  dummy_ros_control::HostDiagnosticScope scope(journal);
  journal.enable_details(true);
  for(unsigned i=0U;i<dummy_ros_control::DiagnosticJournal::kDetailCapacity;++i)journal.detail("full %u",i);
  std::atomic<bool> running{true};
  std::thread reader([&] {
    while(running.load(std::memory_order_relaxed)) {
      (void)dummy_ros_control::diagnostic_journal_json(journal);
      std::this_thread::yield();
    }
  });
  char name[]="wire",mode[]="release_query_backoff_guards";char* arguments[]={name,mode};
  for(unsigned i=0U;i<20U;++i)assert(original_wire_management_main(2,arguments)==0);
  DpManagementTransport absent({"tcp/127.0.0.1:29999",{1},{2},2,1},DpManagementTransport::Connection::kDeviceWorker);
  ManageRequest request;request.set_schema_major(2U);request.set_schema_minor(wire::kSchemaMinor);
  request.set_request_id(42U);request.set_request_seq(7U);request.set_operation(3U);
  request.set_expected_boot_epoch(7U);request.set_expected_session_id(9U);
  request.set_expected_reference_epoch(1U);request.set_expected_motion_epoch(10U);
  std::vector<std::uint8_t> bytes(wire::DummyManageV2RequestCodec::kEncodedSize);
  assert(wire::DummyManageV2RequestCodec::encode(request,bytes.data(),bytes.size())==wire::CodecResult::kOk);
  for(unsigned i=0U;i<100U;++i)assert(!absent.send(bytes));
  running=false;reader.join();
  const auto snapshot=journal.snapshot();
  assert(snapshot.first_present && snapshot.rejections>=140U && snapshot.detail_dropped>0U);
  assert(journal.first_record(dummy_ros_control::DiagnosticReason::kManagementRejected,true));
}

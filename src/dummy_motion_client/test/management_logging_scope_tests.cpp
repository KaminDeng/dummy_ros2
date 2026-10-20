/**
 * @file management_logging_scope_tests.cpp
 * @brief 并发读诊断JSON时复用有界Query重试负例并核饱和日志。
 *
 * 通过宏重命名wire_management_tests.cpp入口，仅调用release_query_backoff_guards模式20次；未运行其原默认main全部case。
 * 另100次DeviceWorker facade无ready发送均本地拒绝，不启动provider或真实TCP peer；线程只并发读journal，assert须启用。
 */
#define main original_wire_management_main
#include "wire_management_tests.cpp"
#undef main
#include "dummy_ros_control/diagnostic_scope.hpp"
#include "dummy_ros_control/diagnostic_json.hpp"
#include <atomic>
#include <cassert>
#include <thread>
/**
 * @brief 填满detail槽，竞争读取时仍保首拒/计数与分reason记录。
 * @return 0表示所选检查通过；非零或断言/require异常表示失败。
 * 复用分支分别测Renew普通退避与Disable快速Query；每种Query只可同字节重试2次，第三次超时抛错，不能重发原mutation。
 * reader循环输出到内存string后yield，不写外部sink；最后join再核rejections≥140/detail_dropped>0及ManagementRejected。
 */
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

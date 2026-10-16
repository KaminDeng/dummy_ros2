#include "dummy_ros_control/diagnostic_journal.hpp"
#include "dummy_ros_control/diagnostic_json.hpp"
#include <cassert>
#include <chrono>
#include <fcntl.h>
#include <thread>
#include <sys/wait.h>
#include <unistd.h>
#include <array>
#include <fstream>
using namespace dummy_ros_control;
int main(int argc,char** argv) {
    // 同时制造已满、无人读取的stderr sink；生产者绝不能碰它。
    int sink[2]; assert(::pipe(sink)==0);
    int flags=::fcntl(sink[1],F_GETFL); assert(flags>=0);
    assert(::fcntl(sink[1],F_SETFL,flags|O_NONBLOCK)==0);
    std::array<char,4096> bytes{};
    while(::write(sink[1],bytes.data(),bytes.size())>0) {}
    assert(::fcntl(sink[1],F_SETFL,flags)==0);
    const auto child=::fork(); assert(child>=0);
    if(child==0) {
        assert(::dup2(sink[1],STDERR_FILENO)>=0);
        DiagnosticJournal journal;
        DiagnosticIdentity observed{11U,22U,3U,4U,5U,6U,true};
        DiagnosticIdentity expected{12U,23U,7U,8U,9U,10U,true};
        journal.reject(DiagnosticReason::kStateRejected,observed,expected,500U,"identity",17U);
        for(unsigned n=0U;n<100000U;++n) {
            journal.detail("periodic %u",n);
            journal.reject(DiagnosticReason::kStopUnconfirmed,{}, {},501U+n,"later");
        }
        auto snapshot=journal.snapshot();
        assert(snapshot.first_present && snapshot.first.reason==DiagnosticReason::kStateRejected);
        assert(snapshot.first.observed.boot==11U && snapshot.first.expected.boot==12U);
        assert(snapshot.first.observed.sequence==5U && snapshot.first.expected.sequence==9U);
        assert(snapshot.first.host_us==500U && snapshot.first.detail_code==17U);
        assert(snapshot.detail_suppressed==100000U && snapshot.rejections==100001U);
        assert(snapshot.first_suppressed==100000U && !journal.detail_record(0U));
        journal.enable_details(true);
        for(unsigned n=0U;n<DiagnosticJournal::kDetailCapacity+3U;++n) journal.detail("prime %u",n);
        snapshot=journal.snapshot(); assert(snapshot.detail_dropped==3U);
        assert(journal.detail_record(0U) && snapshot.first.host_us==500U);
        const std::array<std::uint8_t,350> raw{};
        journal.blob("safe_prime",raw.data(),raw.size());
        const auto blob=journal.blob_record(); assert(blob && blob->valid && blob->payload_size==raw.size());
        journal.blob("later",nullptr,500U);
        assert(journal.snapshot().blob_dropped==1U && journal.blob_record()->payload_size==350U);
        DiagnosticJournal invalid_blob;
        invalid_blob.blob("unavailable",nullptr,350U);
        assert(invalid_blob.blob_record() && !invalid_blob.blob_record()->valid && invalid_blob.snapshot().blob_dropped==1U);
        DiagnosticJournal fresh;
        assert(!fresh.snapshot().first_present); // 唯一reset边界是新对象；boot切换不会清首错。
        ::_exit(0);
    }
    int status=0;
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
    while(::waitpid(child,&status,WNOHANG)==0) {
        if(std::chrono::steady_clock::now()>deadline) { ::kill(child,SIGKILL); ::waitpid(child,&status,0); assert(false && "producer blocked on slow sink"); }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    assert(WIFEXITED(status) && WEXITSTATUS(status)==0);
    ::close(sink[0]); ::close(sink[1]);
    // 并发首拒发布：一个完整winner，首错不依赖可选detail槽。
    DiagnosticJournal shared; shared.enable_details(true);
    std::array<std::thread,8> producers;
    for(unsigned i=0;i<producers.size();++i) producers[i]=std::thread([&,i] {
        DiagnosticIdentity id{100U+i,200U+i,i,i,i,i,true};
        shared.reject(DiagnosticReason::kGoalRejected,id,id,1000U+i,"concurrent");
        shared.detail("value %u",i);
    });
    for(auto& thread:producers)thread.join();
    const auto snapshot=shared.snapshot(); assert(snapshot.first_present && snapshot.rejections==8U);
    assert(snapshot.first.observed.boot-100U==snapshot.first.observed.session-200U);
    shared.publish_context(snapshot.first.observed,snapshot.first.expected);
    const auto context=shared.context(); assert(context.first.valid && context.second.valid);
    DiagnosticJournal domains;
    domains.reject(DiagnosticReason::kStateIdentity,{}, {},1U,"bootstrap");
    domains.reject(DiagnosticReason::kStateIdentity,context.first,context.second,2U,"bound_identity");
    assert(domains.snapshot().first.host_us==1U);
    assert(domains.first_record(DiagnosticReason::kStateIdentity,false)->host_us==1U);
    assert(domains.first_record(DiagnosticReason::kStateIdentity,true)->host_us==2U);
    DiagnosticJournal report;
    report.record_stop_policy({true,1000000U,1100000U,1250000U,250000U,51U,99U,
      {10U,17U,2U,3U,1U,4U,true},201U,63U,9U,4U,5U});
    if(argc==2) {
      std::ofstream output(argv[1]);
      output << "{\"schema\":\"DUMMY_ROS_DIAGNOSTIC_SNAPSHOT_V1\",\"gate\":null,\"worker\":"
             << diagnostic_journal_json(report) << '}';
      assert(output.good());
    }
    const auto document=diagnostic_journal_json(domains);
    assert(document.find("DUMMY_ROS_DIAGNOSTICS_V1")!=std::string::npos);
    assert(document.find("pre_identity")!=std::string::npos && document.find("bound_identity")!=std::string::npos);
    return 0;
}

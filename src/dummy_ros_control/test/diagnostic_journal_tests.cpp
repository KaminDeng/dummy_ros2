/**
 * @file diagnostic_journal_tests.cpp
 * @brief 验证诊断首错保留、有限详情、并发快照及离线JSON。
 *
 * 本程序使用pipe/fork和线程制造受控竞争；不连接设备。满stderr只检验这里调用的journal生产者路径不受慢sink阻塞，2s看门狗不是实时最坏耗时证明。
 * 检查主要由assert完成，验证构建必须保留断言。
 */
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
/**
 * @brief 运行诊断缓冲压力和一致性断言。
 *
 * 可选argv[1]接收Stop策略JSON快照；无该参数时只执行内存检查。所有host_us字段均为人工主机时间，不能拿来当设备Source时间。
 * @param argc 命令行参数个数（含程序名）。
 * @param argv 命令行字符串数组，具体位置见上述调用约定。
 * @return 0 表示断言均通过；断言失败终止进程，子进程超时会被父进程杀掉。
 */
int main(int argc,char** argv) {
    // 同时制造已满、无人读取的stderr sink；生产者绝不能碰它。
    /**
     * @brief 先非阻塞填满pipe，再恢复阻塞；子进程stderr指向无人读取的写端。
     * 如journal误写stderr，子进程将卡住，父进程2s期限能观察到这一回归。
     */
    int sink[2]; assert(::pipe(sink)==0);
    int flags=::fcntl(sink[1],F_GETFL); assert(flags>=0);
    assert(::fcntl(sink[1],F_SETFL,flags|O_NONBLOCK)==0);
    std::array<char,4096> bytes{};
    while(::write(sink[1],bytes.data(),bytes.size())>0) {}
    assert(::fcntl(sink[1],F_SETFL,flags)==0);
    const auto child=::fork(); assert(child>=0);
    if(child==0) {
        assert(::dup2(sink[1],STDERR_FILENO)>=0);
        /**
         * @brief 首拒固定为host=500µs、detail_code=17。
         * 之后100,000次拒绝应只增加计数，默认关闭的detail应计suppressed，不能覆盖首拒。
         */
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
        /**
         * @brief 详情容量是固定槽数，超出的3条计dropped；blob也只保留首次载荷。
         * 空指针blob会形成valid=false记录并增加丢弃计数，不能伪造有效350字节数据。
         */
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
    /**
     * @brief 8个生产者用相关字段编码各自编号。
     * join后检查首拒winner的boot/session来自同一生产者，不能拼接两个线程的字段；不要求固定哪个线程获胜。
     */
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
    /**
     * @brief 同一reason在尚未绑定身份/已绑定身份两个域各保留首拒；全局首拒仍是首次bootstrap。
     */
    DiagnosticJournal domains;
    domains.reject(DiagnosticReason::kStateIdentity,{}, {},1U,"bootstrap");
    domains.reject(DiagnosticReason::kStateIdentity,context.first,context.second,2U,"bound_identity");
    assert(domains.snapshot().first.host_us==1U);
    assert(domains.first_record(DiagnosticReason::kStateIdentity,false)->host_us==1U);
    assert(domains.first_record(DiagnosticReason::kStateIdentity,true)->host_us==2U);
    DiagnosticJournal report;
    /**
     * @brief 人工构造Stop策略时间窗：主机起点、期限及250,000µs预算随JSON输出。
     * 只检查记录和序列化，不表示实际Stop已经收到确认。
     */
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

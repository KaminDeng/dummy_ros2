/**
 * @file fault_cleanup_timeline_tests.cpp
 * @brief 核raw故障生产时钟映射边界与专用FaultCleanup时间线收尾。
 *
 * 时钟输入为人工Host/Sourceµs及序号锚；会写本地CSV，不访问设备。专用fault cleanup不改变普通success/cancel仍须Hold的规则；assert须启用。
 */
#include "dummy_ros_control/raw_evidence_collector.hpp"
#include "dummy_motion_client/goal_timeline.hpp"
#include <cassert>
#include <cstdio>
#include <string>
using namespace dummy_motion_client;
/**
 * @brief 用固定已校准锚调用raw_fault_producer_sample_valid。
 * @param received Host接收µs。
 * @param produced 设备Source产生µs。
 * @param seq State串号。
 * @param anchor 是否存在有效既有时钟锚，默认true。
 * @return 对应Source→Host映射/年龄/序号关系成立；不是直接拿两钟绝对值比较。
 */
static bool clock(std::uint64_t received,std::uint64_t produced,std::uint32_t seq,bool anchor=true) {
 dummy_ros_control::RawFaultProducerClock f{};
 return dummy_ros_control::raw_fault_producer_sample_valid(received,produced,seq,anchor,10,900000,100000,100000,1000000,f,250000);
}
/**
 * @brief 先核250ms边界，再生成13种专用清理时间线。
 * @param argc 参数个数（含程序名）。
 * @param argv 参数数组，模式/路径位置见下述说明。
 * @return 0表示所选检查通过；非零或断言/require异常表示失败。
 * 1450000µs边界有效、再加1失效；缺锚、旧seq、零/旧/未来Source及第二样本Source未前进都拒绝。
 * CSV编号0合法Aborted后FaultCleanup，1..11改boot/session/run/stop epoch/轴mask/组/release/chain/当前Fault清理字段或混release键，12把前终态改Succeeded，均不complete。argc1临时目录或argc2路径前缀。
 */
int main(int argc,char**argv) {
 assert(clock(1200000,1100000,11));assert(clock(1450000,1100000,11));
 assert(!clock(1450001,1100000,11));assert(!clock(1200000,1100000,11,false));assert(!clock(1200000,800000,11));assert(!clock(1200000,1100000,10));assert(!clock(1200000,1100000,9));assert(!clock(1200000,0,11));assert(!clock(1200000,1600000,11));
 dummy_ros_control::RawFaultProducerClock c{};
 assert(dummy_ros_control::raw_fault_producer_sample_valid(1200000,1100000,11,true,10,900000,100000,100000,1000000,c,250000));
 assert(!dummy_ros_control::raw_fault_producer_sample_valid(1210000,1100000,12,true,10,900000,100000,100000,1000000,c,250000));
 char temp[]="/tmp/dummy-fault-clock-XXXXXX";std::string permanent_path;
 if(argc==1) {char* dir=mkdtemp(temp);if(!dir)return 1;permanent_path=std::string(dir)+"/timeline";}
 const char* output_path=argc==1?permanent_path.c_str():argv[1];
 if(argc!=1 && argc!=2)return 1;
 for(unsigned n=0;n<13;++n) {
  GoalTimeline t(std::string(output_path)+"-"+std::to_string(n)+".csv");TimelineEvent e{};e.uuid[0]=1;e.axis_mask=63;e.identity={11,12,13,1,1};
  e.kind=TimelineEvent::Kind::kAccepted;e.steady_us=1;assert(t.record(e));e.kind=TimelineEvent::Kind::kBeforeBegin;e.steady_us=2;assert(t.record(e));e.kind=TimelineEvent::Kind::kAfterBegin;e.identity.motion_epoch=2;e.steady_us=3;assert(t.record(e));e.kind=TimelineEvent::Kind::kFeedback;e.steady_us=4;assert(t.record(e));e.kind=n==12?TimelineEvent::Kind::kTerminalSuccess:TimelineEvent::Kind::kTerminalAborted;e.steady_us=5;assert(t.record(e));
  e.kind=TimelineEvent::Kind::kFaultCleanup;e.steady_us=6;e.status_core_state=9;e.state_core_state=9;
  e.fault_cleanup={11,12,1,1,2,3,0x1003,63,4,{},true,true,true,false,false,true};e.fault_cleanup.terminal_chain[0]=1;
  switch(n){case 1:e.fault_cleanup.boot=22;break;case 2:e.fault_cleanup.session=22;break;case 3:e.fault_cleanup.run=22;break;case 4:e.fault_cleanup.stop_epoch=4;break;case 5:e.fault_cleanup.axis_mask=31;break;case 6:e.fault_cleanup.full_groups_valid=false;break;case 7:e.fault_cleanup.release_committed=false;break;case 8:e.fault_cleanup.terminal_chain={};break;case 9:e.fault_cleanup.current_fault_clear_verified=false;break;case 10:e.state_core_state=4;break;case 11:e.release_request_id=1;break;default:break;}
  const bool ok=t.record(e);assert(ok==(n==0));t.close();assert(t.complete()==(n==0));
 }
 // normal/cancel旧决策依旧必须Hold：fault事件没有更改通用inline。
 assert(timeline_success_decision(true,false,false,true,true,false,true)==TimelineSuccessDecision::kWait);
 std::puts("actualclockanchor/freshproducer negatives + namedfaultcleanup13modes + originalHoldgate unchanged PASS");
}

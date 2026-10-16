// 只在本测试TU取消重复plugin注册；被测方法仍逐字包含正式CPP。
#include "pluginlib/class_list_macros.hpp"
#undef PLUGINLIB_EXPORT_CLASS
#define PLUGINLIB_EXPORT_CLASS(...)
#include "../src/guarded_zenbedded_system.cpp"
#undef PLUGINLIB_EXPORT_CLASS
#include "dummy_ros_control/command_channel.hpp"
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <mutex>
#include <thread>
using namespace dummy_ros_control;
std::mutex barrier_mutex;
std::condition_variable barrier_cv;
bool first_inside=false,second_finished=false,timeout_seen=false;
std::atomic<unsigned> consume_calls{0U};
extern "C" std::optional<CommandGenerationRecord> real_consume(std::uint64_t) asm("__real__ZN17dummy_ros_control22consume_control_updateEm");
extern "C" std::optional<CommandGenerationRecord> wrapped_consume(std::uint64_t) asm("__wrap__ZN17dummy_ros_control22consume_control_updateEm");
extern "C" std::optional<CommandGenerationRecord> wrapped_consume(std::uint64_t generation) {
  auto record=real_consume(generation);
  const auto call=consume_calls.fetch_add(1U);
  if(call==0U) {
    std::unique_lock<std::mutex> lock(barrier_mutex);
    first_inside=true;barrier_cv.notify_all();
    if(!barrier_cv.wait_for(lock,std::chrono::seconds(2),[]{return second_finished;}))timeout_seen=true;
  }
  return record;
}
int main() {
  GuardedZenbeddedSystem system;
  const CommandIdentity identity{11U,22U,33U,44U,55U};
  const std::array<double,6> q{.1,.2,.3,.4,.5,.6};
  set_command_identity(identity);
  const auto generation=publish_control_update(q.data(),q.size(),0,nullptr);
  if(!generation)return 2;
  bool first=false,second=false;
  std::thread a([&]{first=system.transfer_control_update();});
  {
    std::unique_lock<std::mutex> lock(barrier_mutex);
    if(!barrier_cv.wait_for(lock,std::chrono::seconds(2),[]{return first_inside;})) {
      second_finished=true;lock.unlock();barrier_cv.notify_all();a.join();return 3;
    }
  }
  std::thread b([&]{
    second=system.transfer_control_update();
    {std::lock_guard<std::mutex> lock(barrier_mutex);second_finished=true;}
    barrier_cv.notify_all();
  });
  a.join();b.join();
  CommandGenerationRecord record{};unsigned count=0U;bool intact=true;
  while(system.take_publish_record(&record)) {
    ++count;intact=intact && record.generation==generation && record.position==q && record.identity.boot==11U && record.identity.ticket==55U;
  }
  const auto retry=system.transfer_control_update();
  std::printf("重叠transfer first=%d second=%d 实际记录数=%u 原记录一致=%d consume次数=%u 成功后重试=%d 超时=%d\n",first,second,count,intact,consume_calls.load(),retry,timeout_seen);
  bool contracts=true;
  {
    GuardedZenbeddedSystem ordinary;
    const auto older=publish_control_update(q.data(),q.size(),0,nullptr);
    const auto latest=publish_control_update(q.data(),q.size(),0,nullptr);
    const bool sent=ordinary.transfer_control_update();
    CommandGenerationRecord got{};
    const bool one=ordinary.take_publish_record(&got) && got.generation==latest && got.generation>older && !ordinary.take_publish_record(&got);
    const bool empty=!ordinary.transfer_control_update();
    contracts=contracts && sent && one && empty;
    std::printf("latest_only=%d empty_release=%d\n",sent && one,empty);
  }
  {
    GuardedZenbeddedSystem bounded;
    std::array<std::uint64_t,5> generations{};
    for(unsigned i=0;i<4;++i) {generations[i]=publish_control_update(q.data(),q.size(),0,nullptr);contracts=bounded.transfer_control_update() && contracts;}
    generations[4]=publish_control_update(q.data(),q.size(),0,nullptr);
    const bool full=!bounded.transfer_control_update();
    CommandGenerationRecord got{};
    bool order=bounded.take_publish_record(&got) && got.generation==generations[0];
    const bool retry_full=bounded.transfer_control_update();
    for(unsigned i=1;i<5;++i)order=bounded.take_publish_record(&got) && got.generation==generations[i] && order;
    const bool end=!bounded.take_publish_record(&got) && !bounded.transfer_control_update();
    contracts=contracts && full && retry_full && order && end && bounded.publish_drops()==1U;
    std::printf("capacity4_full=%d retry_original_generation=%d order=%d drops=%llu\n",full,retry_full,order,static_cast<unsigned long long>(bounded.publish_drops()));
  }
  {
    GuardedZenbeddedSystem identity_gate;
    (void)publish_control_update(q.data(),q.size(),0,nullptr);
    clear_command_identity();
    const bool retired=!identity_gate.transfer_control_update();
    const CommandIdentity next_identity{111U,222U,333U,444U,555U};
    set_command_identity(next_identity);
    const auto next=publish_control_update(q.data(),q.size(),0,nullptr);
    CommandGenerationRecord got{};
    const bool fresh=identity_gate.transfer_control_update() && identity_gate.take_publish_record(&got) && got.generation==next && got.identity.boot==111U && got.identity.ticket==555U;
    contracts=contracts && retired && fresh;
    std::printf("retired_identity_rejected=%d fresh_identity_retry=%d\n",retired,fresh);
  }
  const bool pass=first && !second && count==1U && intact && !retry && !timeout_seen && contracts && system.publish_drops()==0U;
  std::puts(pass?"通过：重叠producer立即返回false，只入队一份原generation":"失败：重叠producer重复入队原generation或破坏原队列合同");
  return pass?0:1;
}

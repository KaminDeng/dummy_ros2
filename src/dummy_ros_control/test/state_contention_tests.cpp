/**
 * @file state_contention_tests.cpp
 * @brief 在合成State写入竞争下检查未激活read与缓存失效。
 *
 * 硬件对象从未configure/activate，因此循环中的read走inactive快速返回。此用例不覆盖活动态read的try_lock竞争路径，也不是实时最大耗时测试。
 */
#include "dummy_ros_control/guarded_zenbedded_system.hpp"
#include "dummy_ros_control/device_transport.hpp"
#include <atomic>
#include <thread>
#include <iostream>
/**
 * @brief 启动State写线程、重复inactive read并验证失效后的行为。
 *
 * Source产生时刻100µs和expectation时间均为夹具；另设主机steady_clock期限200ms。读取次数最多50,000，线程启动标志只是安排并发时序。
 * @return 0 表示本文件全部检查通过；非零定位失败检查。
 */
int main() {
  using namespace dummy_ros_control;
  GuardedZenbeddedSystem system;
  JointStateSnapshot snapshot;
  snapshot.envelope={7,1,1,100,1,1,1}; snapshot.position[0]=0.25;
  const StateExpectation expectation{7,1,0,true,1,90,110,20};
  if(!system.ingest_state(snapshot,expectation)) return 1;
  const auto deadline=transport_steady_now_us()+200000;
  system.set_state_deadline(deadline);
  std::atomic<bool> running{true}, started{false};
  /**
   * @brief producer不断写入同一合法合成快照；main读循环不修改这些夹具。
   * join保证检查副本前生产者已退出；inactive read成功不能据此推论活动态一直获得了缓存锁。
   */
  std::thread producer([&] {
    started=true;
    while(running) system.ingest_state(snapshot,expectation);
  });
  while(!started) std::this_thread::yield();
  bool fresh_ok=true;
  for(unsigned i=0;i<50000 && transport_steady_now_us()<deadline;++i) {
    if(system.read(rclcpp::Time(),rclcpp::Duration::from_seconds(0.01))!=hardware_interface::return_type::OK) {
      fresh_ok=false; break;
    }
  }
  running=false; producer.join();
  if(!fresh_ok) { std::cerr<<"fresh state read failed under writer contention\n"; return 2; }
  JointStateSnapshot observed;
  if(!system.copy_state(&observed) || observed.envelope.produced_at_us!=100) return 3;
  /**
   * @brief 旧执行身份失效后copy_state应不可作为授权控制反馈使用。
   * inactive read仍返回OK供controller-manager轮询；过期截止时间也留给activate严格检查。
   */
  system.invalidate_authority_state({7,8,9,10,11});
  if(system.copy_state(&observed)) return 5;
  if(system.read(rclcpp::Time(),rclcpp::Duration::from_seconds(0.01))!=hardware_interface::return_type::OK) {
    std::cerr<<"fresh measurement was discarded during authority transition\n"; return 6;
  }
  // An expired deadline remains recorded, but inactive controller-manager polls
  // must not force an error before on_activate performs the strict fresh check.
  system.set_state_deadline(transport_steady_now_us()-1);
  if(system.read(rclcpp::Time(),rclcpp::Duration::from_seconds(0.01))!=hardware_interface::return_type::OK) return 4;
}

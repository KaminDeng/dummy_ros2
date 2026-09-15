#include "dummy_ros_control/guarded_zenbedded_system.hpp"
#include "dummy_ros_control/device_transport.hpp"
#include <atomic>
#include <thread>
#include <iostream>
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

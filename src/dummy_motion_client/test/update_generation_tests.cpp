#include "dummy_ros_control/command_channel.hpp"
#include <array>
#include <iostream>
int main() {
  const std::array<double,6> positions{0,0,1,0,0,0};
  dummy_ros_control::set_command_identity({1,2,3,4,5});
  const auto generation=dummy_ros_control::publish_control_update(positions.data(),6,100,nullptr);
  const int fresh=dummy_ros_control::consume_control_update(0).has_value()?1:0;
  int duplicates=0;
  for(int write=0;write<100;++write)
    duplicates+=dummy_ros_control::consume_control_update(generation).has_value()?1:0;
  dummy_ros_control::clear_command_identity();
  if (!generation || fresh!=1 || duplicates) return 1;
  std::cout << "{\"fresh_generations\":" << fresh << ",\"duplicate_generations\":" << duplicates << "}\n";
}

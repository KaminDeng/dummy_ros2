#include "action_test_support.hpp"
#include <iostream>
int main() {
  using namespace action_test;
  Harness harness; harness.enable(); unsigned successes=0;
  for (unsigned goal=0;goal<2;++goal) {
    harness.begin(); harness.complete();
    successes+=harness.gate.terminal()==Terminal::kSucceeded ? 1:0;
  }
  require(successes==2 && harness.port.starts==2);
  const auto old_generation=harness.gate.generation()-1;
  const bool old_accepted=harness.gate.record_update(old_generation,harness.epoch,2,Position{});
  require(!old_accepted);
  std::cout << "{\"component_successful_goals\":" << successes << ",\"component_completion_barrier_count\":" << successes
    << ",\"component_old_generation_accepted\":" << (old_accepted?"true":"false") << "}\n";
}

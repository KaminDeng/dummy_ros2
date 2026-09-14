#include <array>
#include "dummy_ros_control/command_producer.hpp"

int main() {
  dummy_ros_control::CommandProducer producer;
  const dummy_ros_control::CommandIdentity hold{1U, 2U, 3U, 4U, 5U};
  const std::array<double, 6> position{0.1, 0.2, 0.7, 0.4, 0.5, 9.42477796076938};
  const auto first_generation = producer.record_update(hold, position);
  const auto first = producer.consume_after(0U);
  if (!first || first->generation != first_generation || first->position != position) return 1;
  if (producer.consume_after(first_generation).has_value()) return 2;
  for (unsigned write = 0; write < 100; ++write) {
    if (producer.consume_after(first_generation).has_value()) return 3;
  }
  const auto second_generation = producer.record_update(hold, position);
  const auto second = producer.consume_after(first_generation);
  if (!second || second->generation != second_generation || second_generation != first_generation + 1U)
    return 4;
  if (second->identity.ticket != hold.ticket || second->position != position) return 5;
  return 0;
}

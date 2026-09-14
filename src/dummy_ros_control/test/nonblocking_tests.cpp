#include "dummy_ros_control/bounded_publish_queue.hpp"
#include "dummy_ros_control/management_access.hpp"
#include "dummy_ros_control/guarded_zenbedded_system.hpp"

int main() {
  dummy_ros_control::BoundedPublishQueue<2> queue;
  dummy_ros_control::CommandGenerationRecord first{};
  first.generation = 1U;
  dummy_ros_control::CommandGenerationRecord second{};
  second.generation = 2U;
  dummy_ros_control::CommandGenerationRecord third{};
  third.generation = 3U;
  if (!queue.try_push(first) || !queue.try_push(second)) return 1;
  if (queue.try_push(third) || queue.drops() != 1U) return 2;
  dummy_ros_control::CommandGenerationRecord output{};
  if (!queue.try_pop(&output) || output.generation != 1U) return 3;
  if (!queue.try_push(third)) return 4;
  if (!queue.try_pop(&output) || output.generation != 2U) return 5;
  if (!queue.try_pop(&output) || output.generation != 3U) return 6;
  if (queue.try_pop(&output) || queue.drops() != 1U) return 7;
  const dummy_ros_control::ManagementAccess inactive(false, true);
  if (!inactive.can_query() || !inactive.can_capture_reference()) return 8;
  const auto active = inactive.with_hardware_active(true);
  if (!active.can_query() || !active.hardware_active()) return 9;
  dummy_ros_control::GuardedZenbeddedSystem system;
  const dummy_ros_control::CommandIdentity identity{1U, 2U, 3U, 4U, 5U};
  const std::array<double, 6> hold{0.1, 0.2, 0.7, 0.4, 0.5, 0.6};
  for (unsigned update = 0; update < 1000U; ++update) {
    if (system.submit_control_update(identity, hold) == 0U) return 10;
    system.transfer_control_update();
  }
  if (system.publish_drops() != 996U) return 11;
  return 0;
}

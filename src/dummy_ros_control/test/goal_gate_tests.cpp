#include "dummy_ros_control/goal_registration_gate.hpp"

int main() {
  using dummy_ros_control::GoalCredential;
  using dummy_ros_control::GoalRegistrationGate;

  const GoalCredential registered{{1U}, {2U}};
  const GoalCredential wrong_uuid{{9U}, {2U}};
  const GoalCredential wrong_digest{{1U}, {8U}};
  const GoalRegistrationGate empty;
  if (empty.permits(registered) || empty.busy()) return 1;

  const auto active_result = empty.register_goal(registered);
  if (!active_result.has_value()) return 2;
  const auto active = active_result.value();
  if (!active.busy() || !active.permits(registered)) return 3;
  if (active.permits(wrong_uuid) || active.permits(wrong_digest)) return 4;
  if (active.register_goal(registered).has_value()) return 5;

  const auto cleared = active.clear();
  if (cleared.busy() || cleared.permits(registered)) return 6;
  return 0;
}

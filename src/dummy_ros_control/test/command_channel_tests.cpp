#include <array>
#include "dummy_ros_control/command_channel.hpp"

int main() {
  const std::array<double, 6> hold{0.1, 0.2, 0.7, 0.4, 0.5, 0.6};
  dummy_ros_control::clear_command_identity();
  if (dummy_ros_control::publish_control_update(hold.data(), hold.size(), 1, nullptr) != 0U) return 1;
  const dummy_ros_control::CommandIdentity identity{1U, 2U, 3U, 4U, 5U};
  dummy_ros_control::set_command_identity(identity);
  const auto first = dummy_ros_control::publish_control_update(hold.data(), hold.size(), 2, nullptr);
  const auto second = dummy_ros_control::publish_control_update(hold.data(), hold.size(), 3, nullptr);
  if (first == 0U || second != first + 1U) return 2;
  const auto latest = dummy_ros_control::consume_control_update(first);
  if (!latest || latest->generation != second || latest->identity.ticket != 5U) return 3;
  if (dummy_ros_control::consume_control_update(second).has_value()) return 4;
  const auto joint1 = dummy_ros_control::publish_control_update(hold.data(), 1U, 4, nullptr);
  if (joint1 == 0U) return 6;
  const auto single = dummy_ros_control::consume_control_update(second);
  if (!single || single->position[0] != hold[0]) return 7;
  if (dummy_ros_control::publish_control_update(hold.data(), 2U, 4, nullptr) != 0U) return 8;
  dummy_ros_control::clear_command_identity();
  if (dummy_ros_control::publish_control_update(hold.data(), hold.size(), 4, nullptr) != 0U) return 5;
  if (dummy_ros_control::consume_control_update(0U).has_value()) return 9;
  dummy_ros_control::set_command_identity(identity);
  if (dummy_ros_control::consume_control_update(0U).has_value()) return 10;
  if (dummy_ros_control::command_record_current(*single)) return 11;
  const dummy_ros_control::CommandIdentity old_identity{1U, 2U, 3U, 1U, 5U};
  const dummy_ros_control::CommandIdentity new_identity{1U, 2U, 3U, 2U, 5U};
  dummy_ros_control::set_command_identity(old_identity);
  const auto old_record_generation =
      dummy_ros_control::publish_control_update(hold.data(), hold.size(), 5, nullptr);
  dummy_ros_control::set_command_identity(new_identity);
  if (dummy_ros_control::clear_command_identity_if(old_identity)) return 12;
  if (dummy_ros_control::publish_control_update(hold.data(), hold.size(), 6, nullptr) == 0U) return 13;
  const auto old_record = dummy_ros_control::consume_control_update(old_record_generation - 1U);
  if (old_record && old_record->identity.motion_epoch == old_identity.motion_epoch) return 14;
  return 0;
}

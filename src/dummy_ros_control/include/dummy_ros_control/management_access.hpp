#pragma once

namespace dummy_ros_control {
class ManagementAccess {
 public:
  constexpr ManagementAccess(bool hardware_active, bool raw_state_valid)
      : hardware_active_(hardware_active), raw_state_valid_(raw_state_valid) {}
  constexpr bool hardware_active() const { return hardware_active_; }
  constexpr bool can_query() const { return raw_state_valid_; }
  constexpr bool can_capture_reference() const { return raw_state_valid_; }
  constexpr ManagementAccess with_hardware_active(bool value) const {
    return ManagementAccess(value, raw_state_valid_);
  }
 private:
  bool hardware_active_;
  bool raw_state_valid_;
};
}  // namespace dummy_ros_control

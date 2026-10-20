/**
 * @file management_access.hpp
 * @brief 表达只读管理准入与硬件active标志的两个独立条件。
 * can_query/can_capture_reference只取决于原始状态可用；硬件inactive不必拒绝只读查询或人工参考捕获。实际管理操作的身份、静止、会话和许可由更高层校验。
 */
#pragma once

namespace dummy_ros_control {
/**
 * @brief 不可变的两标志管理访问视图。
 */
class ManagementAccess {
 public:
  /**
   * @brief 保存当前硬件生命周期和原始状态可用性。
   * @param hardware_active ROS硬件组件是否active。
   * @param raw_state_valid 调用方已判定原始状态有效。
   */
  constexpr ManagementAccess(bool hardware_active, bool raw_state_valid)
      : hardware_active_(hardware_active), raw_state_valid_(raw_state_valid) {}
  /**
   * @brief 读取硬件生命周期标志。
   * @return bool，不推导设备已使能。
   */
  constexpr bool hardware_active() const { return hardware_active_; }
  /**
   * @brief 读取查询最低准入条件。
   * @return raw_state_valid，不要求active。
   */
  constexpr bool can_query() const { return raw_state_valid_; }
  /**
   * @brief 读取参考捕获最低状态条件。
   * @return raw_state_valid，非完整捕获授权。
   */
  constexpr bool can_capture_reference() const { return raw_state_valid_; }
  /**
   * @brief 复制视图并替换硬件active标志。
   * @param value 新的active值。
   * @return 新ManagementAccess；保留原始状态有效性。
   */
  constexpr ManagementAccess with_hardware_active(bool value) const {
    return ManagementAccess(value, raw_state_valid_);
  }
 private:
  /**
   * @brief 两个独立布尔值，不保存设备会话或网络句柄。
   */
  bool hardware_active_;
  bool raw_state_valid_;
};
}  // namespace dummy_ros_control

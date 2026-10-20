/**
 * @file goal_registration_gate.hpp
 * @brief 以目标UUID和轨迹摘要绑定一次性提交权限。
 * 这是进程内不可变值对象，不是设备会话许可。调用方以mutex串行替换gate；register_goal返回新gate，不能忽略返回值就认为登记完成。
 */
#pragma once

#include <array>
#include <cstdint>
#include <optional>

namespace dummy_ros_control {

/**
 * @brief GoalUuid为16字节ROS目标ID，TrajectoryDigest为32字节规范轨迹SHA-256。
 */
using GoalUuid = std::array<std::uint8_t, 16>;
using TrajectoryDigest = std::array<std::uint8_t, 32>;

/**
 * @brief 同时匹配目标UUID与全部轨迹摘要，不能只按同末点或同UUID放行。
 */
struct GoalCredential {
  GoalUuid uuid{};
  TrajectoryDigest digest{};

  /**
   * @brief 逐字节比较两份目标凭据。
   * @param left 第一凭据。
   * @param right 第二凭据。
   * @return UUID和摘要都相同为true。
   */
  friend bool operator==(const GoalCredential &left, const GoalCredential &right) {
    return left.uuid == right.uuid && left.digest == right.digest;
  }
};

/**
 * @brief 至多保存一份未消费目标凭据。
 */
class GoalRegistrationGate {
 public:
  /**
   * @brief 构造空闲门控，不登记任何目标。
   */
  constexpr GoalRegistrationGate() = default;

  /**
   * @brief 查询是否已有登记凭据。
   * @return bool，仅表示本地登记槽占用，不代表电机正在运动。
   */
  [[nodiscard]] constexpr bool busy() const { return credential_.has_value(); }

  /**
   * @brief 仅为空闲门控创建含凭据的新对象。
   * @param credential UUID与规范轨迹摘要。
   * @return 新gate或nullopt；不修改原对象。
   */
  [[nodiscard]] constexpr std::optional<GoalRegistrationGate> register_goal(
      const GoalCredential &credential) const {
    if (busy()) {
      return std::nullopt;
    }
    return GoalRegistrationGate(credential);
  }

  /**
   * @brief 检查候选凭据与已登记值相同。
   * @param candidate 提交时提供的凭据。
   * @return bool；不在此重新计算轨迹摘要。
   */
  [[nodiscard]] bool permits(const GoalCredential &candidate) const {
    return credential_.has_value() && credential_.value() == candidate;
  }

  /**
   * @brief 返回空闲gate以消费/取消本地登记。
   * @return 新空对象；原值不变。
   */
  [[nodiscard]] constexpr GoalRegistrationGate clear() const { return GoalRegistrationGate(); }

 private:
  /**
   * @brief 仅内部用于建立已登记值。
   * @param credential 要保存的凭据副本。
   */
  explicit constexpr GoalRegistrationGate(const GoalCredential &credential) : credential_(credential) {}

  /**
   * @brief 可选单槽；无动态队列，不自动驱动设备或判断动作完成。
   */
  std::optional<GoalCredential> credential_{};
};

}  // namespace dummy_ros_control

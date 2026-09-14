#pragma once

#include <array>
#include <cstdint>
#include <optional>

namespace dummy_ros_control {

using GoalUuid = std::array<std::uint8_t, 16>;
using TrajectoryDigest = std::array<std::uint8_t, 32>;

struct GoalCredential {
  GoalUuid uuid{};
  TrajectoryDigest digest{};

  friend bool operator==(const GoalCredential &left, const GoalCredential &right) {
    return left.uuid == right.uuid && left.digest == right.digest;
  }
};

class GoalRegistrationGate {
 public:
  constexpr GoalRegistrationGate() = default;

  [[nodiscard]] constexpr bool busy() const { return credential_.has_value(); }

  [[nodiscard]] constexpr std::optional<GoalRegistrationGate> register_goal(
      const GoalCredential &credential) const {
    if (busy()) {
      return std::nullopt;
    }
    return GoalRegistrationGate(credential);
  }

  [[nodiscard]] bool permits(const GoalCredential &candidate) const {
    return credential_.has_value() && credential_.value() == candidate;
  }

  [[nodiscard]] constexpr GoalRegistrationGate clear() const { return GoalRegistrationGate(); }

 private:
  explicit constexpr GoalRegistrationGate(const GoalCredential &credential) : credential_(credential) {}

  std::optional<GoalCredential> credential_{};
};

}  // namespace dummy_ros_control

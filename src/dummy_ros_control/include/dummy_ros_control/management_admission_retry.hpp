#pragma once

#include <cstdint>
#include "dp_comm/dp_comm.hpp"

namespace dummy_ros_control {

class ManagementAdmissionRetry {
 public:
  enum class Decision { kAccepted, kRetry, kFail };

  explicit ManagementAdmissionRetry(std::uint64_t started_us,
                                    bool unsent_zero_correlation = false) noexcept
      : started_us_(started_us), unsent_zero_correlation_(unsent_zero_correlation) {}

  static bool retryable(dp::comm::Result result, bool unsent_zero_correlation) noexcept {
    return result == dp::comm::Result::kQueueFull ||
        (unsent_zero_correlation && result == dp::comm::Result::kBusy);
  }

  bool ready(std::uint64_t now_us) const noexcept { return now_us >= next_attempt_us_; }
  bool expired(std::uint64_t now_us) const noexcept {
    return now_us < started_us_ || now_us - started_us_ > 100000U;
  }

  Decision on_result(dp::comm::Result result, std::uint64_t now_us,
                     bool stop_pending) noexcept {
    if (stop_pending || expired(now_us)) return Decision::kFail;
    if (result == dp::comm::Result::kOk) return Decision::kAccepted;
    if (!retryable(result, unsent_zero_correlation_) ||
        now_us - started_us_ >= 100000U ||
        ++retries_ > 50U || now_us > UINT64_MAX - 2000U)
      return Decision::kFail;
    next_attempt_us_ = now_us + 2000U;
    return Decision::kRetry;
  }

 private:
  std::uint64_t started_us_ = 0U;
  std::uint64_t next_attempt_us_ = 0U;
  unsigned retries_ = 0U;
  bool unsent_zero_correlation_ = false;
};

}  // namespace dummy_ros_control

#include <cstdint>
#include <cstdio>

#include "dp_comm/dp_comm.hpp"
#include "dummy_ros_control/management_admission_retry.hpp"

namespace {
using dummy_ros_control::ManagementAdmissionRetry;
using Decision = ManagementAdmissionRetry::Decision;
using dp::comm::Result;

struct Request {
  const std::uint64_t id;
  const std::uint32_t seq;
};

bool retry_preserves_request_and_captures_once() {
  constexpr std::uint64_t start_us = 1000000U;
  const Request request{47U, 47U};
  ManagementAdmissionRetry retry(start_us);
  unsigned attempts = 0U;
  unsigned submissions = 0U;
  unsigned captures = 0U;
  const auto attempt = [&](std::uint64_t now_us, Result result) {
    if (request.id != 47U || request.seq != 47U) return Decision::kFail;
    ++attempts;
    const auto decision = retry.on_result(result, now_us, false);
    if (decision == Decision::kAccepted) {
      ++submissions;
      ++captures;
    }
    return decision;
  };

  if (attempt(start_us, Result::kQueueFull) != Decision::kRetry ||
      retry.ready(start_us) || retry.ready(start_us + 1999U) ||
      !retry.ready(start_us + 2000U)) return false;
  if (attempt(start_us + 2000U, Result::kWouldBlock) != Decision::kRetry ||
      retry.ready(start_us + 3999U) || !retry.ready(start_us + 4000U)) return false;
  if (attempt(start_us + 4000U, Result::kOk) != Decision::kAccepted)
    return false;
  return attempts == 3U && submissions == 1U && captures == 1U &&
         request.id == 47U && request.seq == 47U;
}

bool retry_is_bounded_and_stop_has_priority() {
  constexpr std::uint64_t start_us = 1000000U;
  ManagementAdmissionRetry expired(start_us);
  if (expired.on_result(Result::kQueueFull, start_us, false) != Decision::kRetry ||
      expired.on_result(Result::kQueueFull, start_us + 100001U, false) !=
          Decision::kFail) return false;

  ManagementAdmissionRetry no_late_attempt(start_us);
  unsigned attempts = 1U;
  unsigned captures = 0U;
  if (no_late_attempt.on_result(Result::kQueueFull, start_us, false) !=
          Decision::kRetry ||
      no_late_attempt.expired(start_us + 99999U) ||
      !no_late_attempt.expired(start_us + 100001U)) return false;
  const auto now_us = start_us + 100001U;
  if (!no_late_attempt.expired(now_us) && no_late_attempt.ready(now_us)) {
    ++attempts;
    if (no_late_attempt.on_result(Result::kOk, now_us, false) ==
        Decision::kAccepted) ++captures;
  }
  if (attempts != 1U || captures != 0U) return false;

  ManagementAdmissionRetry late_success(start_us);
  if (late_success.on_result(Result::kQueueFull, start_us, false) !=
          Decision::kRetry ||
      late_success.on_result(Result::kOk, start_us + 100001U, false) !=
          Decision::kFail) return false;

  ManagementAdmissionRetry stopped(start_us);
  if (stopped.on_result(Result::kQueueFull, start_us, false) != Decision::kRetry ||
      stopped.on_result(Result::kOk, start_us + 2000U, true) !=
          Decision::kFail) return false;

  ManagementAdmissionRetry non_retryable(start_us);
  return non_retryable.on_result(Result::kInvalidArgument, start_us, false) ==
         Decision::kFail;
}
}  // namespace

int main() {
  if (!retry_preserves_request_and_captures_once()) {
    std::fprintf(stderr, "management retry lost identity, timing, or single capture\n");
    return 1;
  }
  if (!retry_is_bounded_and_stop_has_priority()) {
    std::fprintf(stderr, "management retry ignored deadline, Stop, or hard failure\n");
    return 2;
  }
  return 0;
}

/**
 * @file management_admission_retry_tests.cpp
 * @brief 验证管理请求进入本地发送队列时的有限重试策略。
 *
 * 这里模拟dp_comm返回结果，不发送管理请求；kAccepted是本地准入决策，不能读成设备事务Accepted/Committed。
 * 所有时刻使用人工主机µs，退避2,000µs，总窗口100,000µs，重试不重建请求ID/seq。
 */
#include <cstdint>
#include <cstdio>

#include "dp_comm/dp_comm.hpp"
#include "dummy_ros_control/management_admission_retry.hpp"

namespace {
using dummy_ros_control::ManagementAdmissionRetry;
using Decision = ManagementAdmissionRetry::Decision;
using dp::comm::Result;

/**
 * @brief 不可变请求夹具：id是逻辑请求标识，seq是请求序号；退避期间都必须冻结。
 */
struct Request {
  const std::uint64_t id;
  const std::uint32_t seq;
};

/**
 * @brief 模拟QueueFull、WouldBlock、Ok，检查2ms准入节奏和单次capture。
 * @return true 表示三次尝试中只有成功准入一次计submission/capture且身份未变。
 */
bool retry_preserves_request_and_captures_once() {
  constexpr std::uint64_t start_us = 1000000U;
  const Request request{47U, 47U};
  ManagementAdmissionRetry retry(start_us);
  unsigned attempts = 0U;
  unsigned submissions = 0U;
  unsigned captures = 0U;
  /**
   * @par 内部attempt回调
   * 以now_us和result作为一次本地准入结果；只有返回kAccepted才加submissions/captures。
   * 计数器证明本地重试不重复记请求原文，不证明远端业务恰好执行一次。
   */
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

/**
 * @brief 检查超时、Stop优先级和不可重试错误。
 * @return true 表示超100ms不再尝试/接受，Stop即使伴随Ok也失败，InvalidArgument直接失败。
 */
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
/**
 * @brief Busy只有调用方明确知道尚未发送且零correlation时才可选择重试。
 * @return true 表示默认Busy失败、opt-in按2ms退避，成功只采集一次，50次后在100ms边界失败。
 * @note 夹具直接设置opt-in，不自行证明任意实际Busy一定未发送。
 */
bool busy_requires_unsent_zero_correlation_opt_in() {
  constexpr std::uint64_t start = 1000000U;
  ManagementAdmissionRetry correlated_or_dispatched(start);
  if (correlated_or_dispatched.on_result(Result::kBusy,start,false)!=Decision::kFail)
    return false;
  ManagementAdmissionRetry unsent(start,true);
  const Request frozen{47U,47U};
  unsigned sent=0U, captures=0U;
  if (unsent.on_result(Result::kBusy,start,false)!=Decision::kRetry ||
      unsent.ready(start+1999U) || !unsent.ready(start+2000U)) return false;
  if (unsent.on_result(Result::kOk,start+2000U,false)==Decision::kAccepted) {
    ++sent; ++captures;
  }
  ManagementAdmissionRetry expired(start,true), stopped(start,true);
  if (expired.on_result(Result::kBusy,start,false)!=Decision::kRetry ||
      expired.on_result(Result::kOk,start+100001U,false)!=Decision::kFail ||
      stopped.on_result(Result::kBusy,start,false)!=Decision::kRetry ||
      stopped.on_result(Result::kOk,start+2000U,true)!=Decision::kFail) return false;
  ManagementAdmissionRetry exhausted(start,true);
  for(unsigned attempt=0U;attempt<50U;++attempt)
    if(exhausted.on_result(Result::kBusy,start+attempt*2000U,false)!=Decision::kRetry)
      return false;
  return exhausted.on_result(Result::kBusy,start+100000U,false)==Decision::kFail &&
      sent==1U && captures==1U && frozen.id==47U && frozen.seq==47U;
}
}  // namespace

/**
 * @brief 汇总三个准入策略回归用例。
 *
 * 先检查Busy特殊前提，再检查请求冻结和有限窗口；stderr只在相应用例失败时输出。
 * @return 0 全通过；3 Busy前提错误；1 身份/退避/单次采集错误；2 期限/Stop/硬错误优先级错误。
 */
int main() {
  if (!busy_requires_unsent_zero_correlation_opt_in()) return 3;
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

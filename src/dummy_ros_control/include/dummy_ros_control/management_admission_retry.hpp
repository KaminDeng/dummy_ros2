/**
 * @file management_admission_retry.hpp
 * @brief 为尚未成功送出的管理请求提供有限本地准入重试。
 * 只处理本地QueueFull，以及明确零correlation未发出的Busy；不能把已发请求的超时当作可安全重发。宿主steady微秒时钟总预算100ms，重试间隔2ms且最多50次，停止请求优先失败。
 */
#pragma once

#include <cstdint>
#include "dp_comm/dp_comm.hpp"

namespace dummy_ros_control {

/**
 * @brief 保存一次管理提交的重试预算，不拥有网络请求。
 */
class ManagementAdmissionRetry {
 public:
  /**
   * @brief Accepted表示本地接纳，Retry表示稍后再试，Fail表示结束准入；Accepted不是设备管理操作成功。
   */
  enum class Decision { kAccepted, kRetry, kFail };

  /**
   * @brief 固定准入开始时刻与Busy重试资格。
   * @param started_us 宿主单调µs起点。
   * @param unsent_zero_correlation 仅在确认尚未发出且correlation为0时可为true。
   */
  explicit ManagementAdmissionRetry(std::uint64_t started_us,
                                    bool unsent_zero_correlation = false) noexcept
      : started_us_(started_us), unsent_zero_correlation_(unsent_zero_correlation) {}

  /**
   * @brief 分类是否允许本地重试。
   * @param result dp_comm提交结果。
   * @param unsent_zero_correlation Busy尚未产生远端事务的明确证据。
   * @return QueueFull，或已证未发出的Busy时true。
   */
  static bool retryable(dp::comm::Result result, bool unsent_zero_correlation) noexcept {
    return result == dp::comm::Result::kQueueFull ||
        (unsent_zero_correlation && result == dp::comm::Result::kBusy);
  }

  /**
   * @brief 查询2ms退避时刻是否已到。
   * @param now_us 宿主单调µs。
   * @return bool；调用者仍须另查expired。
   */
  bool ready(std::uint64_t now_us) const noexcept { return now_us >= next_attempt_us_; }
  /**
   * @brief 检测时钟倒退或总预算严格超100ms。
   * @param now_us 同一宿主steady时钟µs。
   * @return bool；恰100ms未由此判过期，但on_result不再安排重试。
   */
  bool expired(std::uint64_t now_us) const noexcept {
    return now_us < started_us_ || now_us - started_us_ > 100000U;
  }

  /**
   * @brief 按停止、时长、结果和次数推进准入状态。
   * @param result 本次提交返回。
   * @param now_us 当前宿主steady µs。
   * @param stop_pending 有更优先停止需求时true。
   * @return Decision；Retry时next_attempt=now+2000µs。
   */
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
  /**
   * @brief 开始/下一尝试均为宿主µs；retries计实际失败后的重试安排次数。
   */
  std::uint64_t started_us_ = 0U;
  std::uint64_t next_attempt_us_ = 0U;
  unsigned retries_ = 0U;
  bool unsent_zero_correlation_ = false;
};

}  // namespace dummy_ros_control

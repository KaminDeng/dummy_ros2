#include "dummy_motion_client/manual_ros_baseline.hpp"
#include <algorithm>
#include <cmath>
namespace dummy_motion_client {
BaselineCurveBounds baseline_quintic_bounds(double p0, double v0, double a0, double p1, double v1, double a1,
                                            double T) noexcept {
    for (double value : {p0, v0, a0, p1, v1, a1, T})
        if (!std::isfinite(value)) return {};
    if (T <= 0) return {};
    const std::array<double, 6> control{p0,
                                        p0 + v0 * T / 5,
                                        p0 + 2 * v0 * T / 5 + a0 * T * T / 20,
                                        p1 - 2 * v1 * T / 5 + a1 * T * T / 20,
                                        p1 - v1 * T / 5,
                                        p1};
    for (double value : control)
        if (!std::isfinite(value)) return {};
    BaselineCurveBounds result;
    const auto extrema = std::minmax_element(control.begin(), control.end());
    result.lower = *extrema.first;
    result.upper = *extrema.second;
    for (unsigned i = 0; i < 5; ++i) {
        const double velocity = 5 * (control[i + 1] - control[i]) / T;
        if (!std::isfinite(velocity)) return {};
        result.max_velocity = std::max(result.max_velocity, std::abs(velocity));
    }
    for (unsigned i = 0; i < 4; ++i) {
        const double acceleration = 20 * (control[i + 2] - 2 * control[i + 1] + control[i]) / T / T;
        if (!std::isfinite(acceleration)) return {};
        result.max_acceleration = std::max(result.max_acceleration, std::abs(acceleration));
    }
    result.valid = true;
    return result;
}
bool valid_baseline_rest_alignment(const std::array<double, 6> &current, const std::array<double, 6> &rest,
                                   const std::array<double, 6> &lower, const std::array<double, 6> &upper,
                                   double max_delta) {
    if (!std::isfinite(max_delta) || max_delta <= 0) return false;
    for (unsigned i = 0; i < 6; ++i) {
        if (!std::isfinite(current[i]) || !std::isfinite(rest[i]) || !std::isfinite(lower[i]) ||
            !std::isfinite(upper[i]) || lower[i] >= upper[i] || current[i] < lower[i] || current[i] > upper[i] ||
            rest[i] < lower[i] || rest[i] > upper[i] ||
            std::abs(rest[i] - current[i]) > ((i == 1 || i == 2) ? max_delta : 0.001))
            return false;
    }
    return true;
}
bool ManualRosBaseline::start(std::uint64_t now) {
    if (stage_ != BaselineStage::kReadOnly) return false;
    started_ = now;
    stage_ = BaselineStage::kCalibrating;
    return true;
}
bool ManualRosBaseline::active() const noexcept {
    return stage_ != BaselineStage::kReadOnly && stage_ != BaselineStage::kPassed && stage_ != BaselineStage::kFailed;
}
void ManualRosBaseline::complete(bool confirmed, const std::string &, std::uint64_t now) {
    tick(now);
    if (!confirmed || !active() || stage_ == BaselineStage::kRelease) return;
    switch (stage_) {
        case BaselineStage::kCalibrating:
            stage_ = BaselineStage::kHold;
            break;
        case BaselineStage::kHold:
            stage_ = BaselineStage::kSingleAxis;
            break;
        case BaselineStage::kSingleAxis:
            stage_ = BaselineStage::kMultiAxis;
            break;
        case BaselineStage::kMultiAxis:
            stage_ = BaselineStage::kPlanRestRoundTrip;
            break;
        case BaselineStage::kPlanRestRoundTrip:
            stage_ = BaselineStage::kObstacleCheck;
            break;
        case BaselineStage::kObstacleCheck:
            stage_ = BaselineStage::kCancelCheck;
            break;
        case BaselineStage::kCancelCheck:
            stage_ = BaselineStage::kRelease;
            break;
        default:
            break;
    }
}
void ManualRosBaseline::fail(const std::string &reason, std::uint64_t) {
    if (!active()) return;
    if (failure_.empty()) failure_ = reason.empty() ? "未提供原因的失败" : reason;
    stage_ = BaselineStage::kRelease;
}
void ManualRosBaseline::tick(std::uint64_t now) {
    if (active() && stage_ != BaselineStage::kRelease && now - started_ >= options_.total_timeout_us)
        fail("自动任务总期限超时", now);
}
void ManualRosBaseline::release_confirmed(bool confirmed) {
    if (stage_ != BaselineStage::kRelease) return;
    if (!confirmed && failure_.empty()) failure_ = "失能释放未确认";
    stage_ = failure_.empty() && confirmed ? BaselineStage::kPassed : BaselineStage::kFailed;
}
BaselineTarget bounded_baseline_target(const std::array<double, 6> &current, const std::array<double, 6> &lower,
                                       const std::array<double, 6> &upper, const std::array<double, 6> &velocity,
                                       const std::array<double, 6> &acceleration, int axis,
                                       const BaselineOptions &options) {
    BaselineTarget out;
    out.positions = current;
    out.velocity = std::min(options.max_velocity_rad_s, 0.03490658503988659);
    out.acceleration = std::min(options.max_acceleration_rad_s2, 0.1);
    const double delta = std::min(options.max_axis_delta_rad, 0.017453292519943295);
    if (axis < -1 || axis > 5 || !std::isfinite(delta) || delta <= 0) return out;
    for (unsigned i = 0; i < 6; ++i) {
        if (!std::isfinite(current[i]) || !std::isfinite(lower[i]) || !std::isfinite(upper[i]) ||
            !std::isfinite(velocity[i]) || !std::isfinite(acceleration[i]) || current[i] < lower[i] ||
            current[i] > upper[i] || velocity[i] <= 0 || acceleration[i] <= 0)
            return out;
        out.velocity = std::min(out.velocity, velocity[i]);
        out.acceleration = std::min(out.acceleration, acceleration[i]);
        if (axis >= 0 && unsigned(axis) != i) continue;
        const double positive = upper[i] - current[i], negative = current[i] - lower[i];
        const double displacement =
            positive >= negative ? std::min(delta, positive * 0.5) : -std::min(delta, negative * 0.5);
        if (std::abs(displacement) < 0.001) return out;
        out.positions[i] += displacement;
    }
    out.valid = out.velocity > 0 && out.acceleration > 0;
    return out;
}
const char *baseline_stage_name(BaselineStage s) {
    switch (s) {
#define STAGE(x)              \
    case BaselineStage::k##x: \
        return #x
        STAGE(ReadOnly);
        STAGE(Calibrating);
        STAGE(Hold);
        STAGE(SingleAxis);
        STAGE(MultiAxis);
        STAGE(PlanRestRoundTrip);
        STAGE(ObstacleCheck);
        STAGE(CancelCheck);
        STAGE(Release);
        STAGE(Passed);
        STAGE(Failed);
#undef STAGE
    }
    return "Unknown";
}
}  // namespace dummy_motion_client

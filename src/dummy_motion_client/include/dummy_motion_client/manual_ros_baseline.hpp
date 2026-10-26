#pragma once
#include <array>
#include <cstdint>
#include <string>
namespace dummy_motion_client {
// 单轴五次Hermite曲线的Bezier凸包保守上界；T为秒，位置rad。
// max_velocity/max_acceleration分别为rad/s、rad/s²的绝对值上界。
// 无I/O；非有限输入、T<=0或计算溢出返回valid=false。
struct BaselineCurveBounds {
    bool valid{};
    double lower{}, upper{}, max_velocity{}, max_acceleration{};
};
BaselineCurveBounds baseline_quintic_bounds(double p0, double v0, double a0, double p1, double v1, double a1,
                                            double T) noexcept;
enum class BaselineStage {
    kReadOnly,
    kCalibrating,
    kHold,
    kSingleAxis,
    kMultiAxis,
    kPlanRestRoundTrip,
    kObstacleCheck,
    kCancelCheck,
    kRelease,
    kPassed,
    kFailed
};
struct BaselineOptions {
    double max_axis_delta_rad{0.017453292519943295};
    double max_velocity_rad_s{0.03490658503988659};
    double max_acceleration_rad_s2{0.1};
    std::uint64_t total_timeout_us{900000000};
};
struct BaselineTarget {
    bool valid{};
    std::array<double, 6> positions{};
    double velocity{}, acceleration{};
};
// Native资格退让可冻结已有安全余量的J2/J3；自动基线先规划到模型REST。
// 仅允许这两轴在SDK指定余量内归位，其余轴须已在REST的0.001rad内。
bool valid_baseline_rest_alignment(const std::array<double, 6> &current, const std::array<double, 6> &rest,
                                   const std::array<double, 6> &lower, const std::array<double, 6> &upper,
                                   double max_delta);
BaselineTarget bounded_baseline_target(const std::array<double, 6> &current, const std::array<double, 6> &lower,
                                       const std::array<double, 6> &upper, const std::array<double, 6> &velocity,
                                       const std::array<double, 6> &acceleration, int axis,
                                       const BaselineOptions &options = {});
class ManualRosBaseline {
public:
    explicit ManualRosBaseline(BaselineOptions options = {}) : options_(options) {}
    bool start(std::uint64_t now);
    void complete(bool confirmed, const std::string &detail, std::uint64_t now);
    void fail(const std::string &reason, std::uint64_t now);
    void tick(std::uint64_t now);
    void release_confirmed(bool confirmed);
    BaselineStage stage() const noexcept { return stage_; }
    const std::string &failure() const noexcept { return failure_; }
    bool active() const noexcept;

private:
    BaselineOptions options_;
    BaselineStage stage_{BaselineStage::kReadOnly};
    std::uint64_t started_{};
    std::string failure_;
};
const char *baseline_stage_name(BaselineStage stage);
}  // namespace dummy_motion_client

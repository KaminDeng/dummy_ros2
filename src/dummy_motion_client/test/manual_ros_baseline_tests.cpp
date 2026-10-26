#include "dummy_motion_client/manual_ros_baseline.hpp"
#include <cassert>
#include <cmath>
#include <limits>
using namespace dummy_motion_client;
int main() {
    // 相同端点并不代表轴静止：非零中间导数会让JTC五次插值离开路点走廊。
    const auto overshoot = baseline_quintic_bounds(0, 0, 0, 0, .01, 0, 2);
    assert(overshoot.valid && overshoot.lower < -.001 && overshoot.upper == 0);
    const auto monotonic = baseline_quintic_bounds(0, 0, 0, 1, 0, 0, 2);
    assert(monotonic.valid && monotonic.lower == 0 && monotonic.upper == 1);
    assert(monotonic.max_velocity == 2.5 && monotonic.max_acceleration == 5);
    const auto slowed = baseline_quintic_bounds(0, 0, 0, 1, 0, 0, 4);
    assert(slowed.valid && slowed.max_velocity == monotonic.max_velocity / 2 &&
           slowed.max_acceleration == monotonic.max_acceleration / 4);
    // 独立Hermite幂基求值，逐样本核位置/速度/加速度都被凸包上界包住。
    const std::array<std::array<double, 7>, 4> curves{{{{0, 0, 0, 0, .01, 0, 2}},
                                                       {{0, 0, 0, 1, 0, 0, 2}},
                                                       {{.2, -.03, .02, -.4, .04, -.01, 3}},
                                                       {{-.7, .2, -.1, .5, -.15, .08, .7}}}};
    for (const auto &c : curves) {
        const auto bounds = baseline_quintic_bounds(c[0], c[1], c[2], c[3], c[4], c[5], c[6]);
        assert(bounds.valid);
        const double T = c[6], c0 = c[0], c1 = c[1] * T, c2 = c[2] * T * T / 2;
        const double D = c[3] - c0 - c1 - c2, V = c[4] * T - c1 - 2 * c2, A = c[5] * T * T - 2 * c2;
        const double c3 = 10 * D - 4 * V + A / 2, c4 = -15 * D + 7 * V - A, c5 = 6 * D - 3 * V + A / 2;
        for (unsigned i = 0; i <= 1000; ++i) {
            const double u = i / 1000.;
            const double q = ((((c5 * u + c4) * u + c3) * u + c2) * u + c1) * u + c0;
            const double v = ((((5 * c5 * u + 4 * c4) * u + 3 * c3) * u + 2 * c2) * u + c1) / T;
            const double a = (((20 * c5 * u + 12 * c4) * u + 6 * c3) * u + 2 * c2) / (T * T);
            if (c[0] == 0 && c[3] == 0 && i == 500) assert(q < -.001 && std::abs(q + .003125) < 1e-12);
            assert(q >= bounds.lower - 1e-12 && q <= bounds.upper + 1e-12);
            assert(std::abs(v) <= bounds.max_velocity + 1e-12);
            assert(std::abs(a) <= bounds.max_acceleration + 1e-12);
        }
    }
    for (double bad : {0., -1., std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()})
        assert(!baseline_quintic_bounds(0, 0, 0, 1, 0, 0, bad).valid);
    for (unsigned i = 0; i < 6; ++i) {
        auto c = curves[0];
        c[i] = std::numeric_limits<double>::quiet_NaN();
        assert(!baseline_quintic_bounds(c[0], c[1], c[2], c[3], c[4], c[5], c[6]).valid);
    }
    assert(!baseline_quintic_bounds(0, std::numeric_limits<double>::max(), 0, 1, 0, 0, 10).valid);

    // 来自run5的保留参考：J3已有13度工作余量，资格退让冻结166.33度，非模型REST。
    const double radians = 3.14159265358979323846 / 180.0;
    std::array<double, 6> retained{0, -60 * radians, 166.33 * radians, 0, 0, 0};
    const std::array<double, 6> nominal{0, -60 * radians, 163.97260273972603 * radians, 0, 0, 0};
    const std::array<double, 6> align_lower{-3, -73 * radians, 35 * radians, -3, -1, -12};
    const std::array<double, 6> align_upper{3, 90 * radians, 180 * radians, 3, 1, 12};
    assert(valid_baseline_rest_alignment(retained, nominal, align_lower, align_upper, 13 * radians));
    retained[0] = 0.01;
    assert(!valid_baseline_rest_alignment(retained, nominal, align_lower, align_upper, 13 * radians));
    retained[0] = 0;
    retained[2] = 179 * radians;
    assert(!valid_baseline_rest_alignment(retained, nominal, align_lower, align_upper, 13 * radians));
    retained = nominal;
    assert(!valid_baseline_rest_alignment(retained, nominal, align_lower, align_upper, 0));
    retained[2] = std::numeric_limits<double>::quiet_NaN();
    assert(!valid_baseline_rest_alignment(retained, nominal, align_lower, align_upper, 13 * radians));
    ManualRosBaseline flow;
    assert(flow.stage() == BaselineStage::kReadOnly);
    assert(flow.start(1));
    assert(!flow.start(2));
    assert(flow.stage() == BaselineStage::kCalibrating);
    flow.complete(false, "QUEUED", 2);
    assert(flow.stage() == BaselineStage::kCalibrating);
    flow.fail("反馈断开", 3);
    assert(flow.stage() == BaselineStage::kRelease);
    flow.release_confirmed(true);
    assert(flow.stage() == BaselineStage::kFailed);
    assert(flow.failure() == "反馈断开");
    ManualRosBaseline success;
    assert(success.start(1));
    for (int i = 0; i < 9; ++i) success.complete(true, "CONFIRMED", i + 2);
    assert(success.stage() == BaselineStage::kRelease);
    success.release_confirmed(false);
    assert(success.stage() == BaselineStage::kFailed);
    ManualRosBaseline timeout;
    timeout.start(1);
    timeout.tick(900000002);
    assert(timeout.stage() == BaselineStage::kRelease);
    std::array<double, 6> p{}, lo{}, hi{}, v{}, a{};
    lo.fill(-0.2);
    hi.fill(0.2);
    v.fill(0.01);
    a.fill(0.02);
    p[0] = hi[0];
    auto target = bounded_baseline_target(p, lo, hi, v, a, 0);
    assert(target.valid && target.positions[0] < p[0]);
    assert(target.velocity <= 0.01 && target.acceleration <= 0.02);
    hi[0] = lo[0];
    p[0] = lo[0];
    assert(!bounded_baseline_target(p, lo, hi, v, a, 0).valid);
    // 每个非清理阶段都必须有明确完成事实；queued不消费步骤。
    ManualRosBaseline passed;
    assert(passed.start(10));
    const BaselineStage stages[] = {BaselineStage::kCalibrating,       BaselineStage::kHold,
                                    BaselineStage::kSingleAxis,        BaselineStage::kMultiAxis,
                                    BaselineStage::kPlanRestRoundTrip, BaselineStage::kObstacleCheck,
                                    BaselineStage::kCancelCheck};
    for (auto stage : stages) {
        assert(passed.stage() == stage);
        passed.complete(false, "QUEUED", 11);
        assert(passed.stage() == stage);
        passed.complete(true, "CONFIRMED", 12);
    }
    assert(passed.stage() == BaselineStage::kRelease);
    passed.complete(true, "QUEUED", 13);
    assert(passed.stage() == BaselineStage::kRelease);
    passed.release_confirmed(true);
    assert(passed.stage() == BaselineStage::kPassed);
    assert(!passed.start(14));
    ManualRosBaseline original_failure;
    assert(original_failure.start(1));
    original_failure.fail("原始故障", 2);
    original_failure.fail("清理故障", 3);
    original_failure.release_confirmed(false);
    assert(original_failure.failure() == "原始故障");
    hi.fill(0.2);
    p.fill(0);
    auto multi = bounded_baseline_target(p, lo, hi, v, a, -1);
    assert(multi.valid);
    for (unsigned axis = 0; axis < 6; ++axis) assert(std::abs(multi.positions[axis] - p[axis]) <= 0.017453292519943295);
    p[0] = std::numeric_limits<double>::quiet_NaN();
    assert(!bounded_baseline_target(p, lo, hi, v, a, 0).valid);
    p.fill(0);
    v[2] = 0;
    assert(!bounded_baseline_target(p, lo, hi, v, a, 0).valid);
    ManualRosBaseline empty_failure;
    assert(empty_failure.start(1));
    empty_failure.fail("", 2);
    empty_failure.release_confirmed(true);
    assert(empty_failure.stage() == BaselineStage::kFailed);
}

#include "motioncore/multiaxis/arc_interpolator.hpp"
#include "motioncore/planning/s_curve_profile.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

using motioncore::multiaxis::ArcDirection;
using motioncore::multiaxis::ArcInterpolator;
using motioncore::multiaxis::ArcResultantLimits;
using motioncore::multiaxis::Vector3;
using motioncore::planning::MotionConstraints;

namespace {
void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

void test_curvature_constraints_are_applied_before_planning() {
    // 半径 10 mm、原始速度 50 mm/s 时法向加速度已达到 250 mm/s^2，
    // 明显超过 100 mm/s^2；修复后应自动降低标量速度而非生成超限弧。
    const ArcInterpolator arc(
        Vector3{0.01, 0.0, 0.0}, Vector3{0.0, 0.01, 0.0}, Vector3{},
        ArcDirection::CCW, MotionConstraints{0.05, 0.1, 0.5},
        motioncore::multiaxis::PlanningMode::SCurve, false,
        ArcResultantLimits{0.1, 0.5});
    const auto scalar = arc.scalar_constraints();
    require(scalar.max_velocity_mps < 0.05, "arc velocity was not conservatively reduced");
    require(arc.satisfies_resultant_limits(), "arc resultant limits were not accepted");
    double max_acceleration = 0.0;
    double max_jerk = 0.0;
    for (const auto& segment :
         static_cast<const motioncore::planning::SCurveProfile&>(arc.scalar_profile()).segments()) {
        for (int sample = 0; sample <= 20; ++sample) {
            const double time = segment.start_time_s
                + segment.duration_s * static_cast<double>(sample) / 20.0;
            const auto metrics = arc.kinematics(time);
            max_acceleration = std::max(max_acceleration,
                                        metrics.resultant_acceleration_mps2);
            max_jerk = std::max(max_jerk, metrics.resultant_jerk_mps3);
        }
    }
    require(max_acceleration <= 0.1 + 1e-8, "arc acceleration exceeds resultant limit");
    require(max_jerk <= 0.5 + 1e-7, "arc jerk exceeds resultant limit");
}

void test_invalid_resultant_limits_are_rejected() {
    bool rejected = false;
    try {
        ArcInterpolator(Vector3{0.01, 0.0, 0.0}, Vector3{0.0, 0.01, 0.0}, Vector3{},
                        ArcDirection::CCW, MotionConstraints{0.05, 0.1, 0.5},
                        motioncore::multiaxis::PlanningMode::SCurve, false,
                        ArcResultantLimits{0.0, 0.5});
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    require(rejected, "zero resultant acceleration limit was accepted");
}
}  // namespace

int main() {
    try {
        test_curvature_constraints_are_applied_before_planning();
        test_invalid_resultant_limits_are_rejected();
        std::cout << "MotionCore Phase 4.1 fix tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "MotionCore Phase 4.1 fix test failure: " << error.what() << '\n';
        return 1;
    }
}

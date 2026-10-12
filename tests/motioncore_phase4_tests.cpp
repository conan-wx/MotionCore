#include "motioncore/multiaxis/arc_interpolator.hpp"
#include "motioncore/multiaxis/coordinated_trajectory_planner.hpp"
#include "motioncore/multiaxis/line_interpolator.hpp"
#include "motioncore/multiaxis/multi_axis_servo_system.hpp"
#include "motioncore/multiaxis/vector3.hpp"
#include "motioncore/planning/s_curve_profile.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using motioncore::multiaxis::ArcDirection;
using motioncore::multiaxis::ArcInterpolator;
using motioncore::multiaxis::ArcResultantLimits;
using motioncore::multiaxis::CoordinatedTrajectoryPlanner;
using motioncore::multiaxis::LineInterpolator;
using motioncore::multiaxis::MotionSegment;
using motioncore::multiaxis::PlanningMode;
using motioncore::multiaxis::Vector3;
using motioncore::planning::MotionConstraints;

constexpr double kTol = 1e-9;
constexpr double kPi = 3.141592653589793238462643383279502884;

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void near(const std::string& name, double actual, double expected, double tolerance) {
    if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance) {
        throw std::runtime_error(name + ": actual=" + std::to_string(actual) +
                                 ", expected=" + std::to_string(expected) +
                                 ", tolerance=" + std::to_string(tolerance));
    }
}

void near_vec(const std::string& name, const Vector3& actual, const Vector3& expected,
              double tolerance) {
    near(name + ".x", actual.x, expected.x, tolerance);
    near(name + ".y", actual.y, expected.y, tolerance);
    near(name + ".z", actual.z, expected.z, tolerance);
}

MotionConstraints constraints(double vmax = 0.05, double amax = 0.1,
                              double jmax = 0.5) {
    return MotionConstraints{vmax, amax, jmax};
}

// Find the time at which an interpolator has traversed the requested scalar
// path fraction.  This deliberately uses the public sampled path progress
// rather than assuming that midpoint time equals midpoint distance.
double time_at_progress(const motioncore::multiaxis::Interpolator& interpolator,
                        double progress) {
    double lo = 0.0;
    double hi = interpolator.duration();
    for (int i = 0; i < 100; ++i) {
        const double mid = 0.5 * (lo + hi);
        if (interpolator.sample(mid).path_progress_m < progress) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    return 0.5 * (lo + hi);
}

template <typename Function>
void expect_invalid(Function&& constructor, const std::string& name) {
    bool thrown = false;
    try {
        constructor();
    } catch (const std::invalid_argument&) {
        thrown = true;
    }
    require(thrown, name + " should reject invalid arguments");
}

void test_vector3_operations() {
    const Vector3 a{1.0, 2.0, 3.0};
    const Vector3 b{-2.0, 4.0, 1.0};
    near_vec("addition", a + b, Vector3{-1.0, 6.0, 4.0}, kTol);
    near_vec("subtraction", a - b, Vector3{3.0, -2.0, 2.0}, kTol);
    near_vec("scalar multiplication", 2.0 * a, Vector3{2.0, 4.0, 6.0}, kTol);
    near_vec("scalar division", a / 2.0, Vector3{0.5, 1.0, 1.5}, kTol);
    near("dot product", a.dot(b), 9.0, kTol);
    near("norm", a.norm(), std::sqrt(14.0), kTol);
    near_vec("normalization", a.normalized(), a / std::sqrt(14.0), kTol);
    require(a.is_finite(), "finite vector reported as non-finite");
    require(!Vector3{std::numeric_limits<double>::infinity(), 0.0, 0.0}.is_finite(),
            "infinite vector reported as finite");
    expect_invalid([] { (void)Vector3{}.normalized(); }, "zero-vector normalization");
    expect_invalid([] { (void)(Vector3{1.0, 2.0, 3.0} / 0.0); }, "zero divisor");
}

void test_line_interpolation_and_branches() {
    const Vector3 start{0.0, 0.0, 0.0};
    const Vector3 target{0.03, 0.04, 0.05};
    const double expected_length = std::sqrt(0.03 * 0.03 + 0.04 * 0.04 + 0.05 * 0.05);
    const LineInterpolator trapezoid(start, target, constraints(), PlanningMode::Trapezoid);
    near("line length", trapezoid.path_length_m(), expected_length, 1e-12);
    // Independent trapezoid formula: ta=v/a=.5s, ramp distance=.025m,
    // cruise distance=L-.025m.
    const double expected_trapezoid_time = 2.0 * (0.05 / 0.1) +
                                           (expected_length - 0.05 * 0.05 / 0.1) / 0.05;
    near("line trapezoid duration", trapezoid.duration(), expected_trapezoid_time, 1e-9);
    near_vec("line start", trapezoid.sample(-0.1).position_m, start, kTol);
    near_vec("line endpoint", trapezoid.sample(trapezoid.duration()).position_m, target, 1e-12);
    near_vec("line endpoint velocity", trapezoid.sample(trapezoid.duration()).velocity_mps,
             Vector3{}, 1e-12);

    const LineInterpolator scurve(start, target, constraints(), PlanningMode::SCurve);
    near("line S-curve duration", scurve.duration(), 2.114213562373095, 1e-8);
    const double half_time = time_at_progress(scurve, expected_length * 0.5);
    const auto half = scurve.sample(half_time);
    near("line half progress", half.path_progress_m, expected_length * 0.5, 1e-10);
    near_vec("line half position", half.position_m, Vector3{0.015, 0.020, 0.025}, 1e-10);

    const LineInterpolator reverse(target, start, constraints(), PlanningMode::SCurve);
    const auto reverse_mid = reverse.sample(reverse.duration() * 0.5);
    require(reverse_mid.velocity_mps.x < 0.0 && reverse_mid.velocity_mps.y < 0.0 &&
                reverse_mid.velocity_mps.z < 0.0,
            "reverse line velocity should point toward the target");
    near_vec("reverse endpoint", reverse.sample(reverse.duration()).position_m, start, 1e-12);

    const LineInterpolator single_axis(start, Vector3{0.01, 0.0, 0.0}, constraints(),
                                       PlanningMode::SCurve);
    const auto single_mid = single_axis.sample(single_axis.duration() * 0.5);
    near("single-axis Y", single_mid.position_m.y, 0.0, kTol);
    near("single-axis Z", single_mid.position_m.z, 0.0, kTol);

    const LineInterpolator short_trapezoid(start, Vector3{0.01, 0.0, 0.0}, constraints(),
                                           PlanningMode::Trapezoid);
    near("short triangular duration", short_trapezoid.duration(),
         2.0 * std::sqrt(0.01 / 0.1), 1e-9);
    const LineInterpolator critical_trapezoid(start, Vector3{0.025, 0.0, 0.0}, constraints(),
                                              PlanningMode::Trapezoid);
    near("critical trapezoid duration", critical_trapezoid.duration(), 1.0, 1e-9);
    const LineInterpolator very_short_scurve(start, Vector3{0.001, 0.0, 0.0}, constraints(),
                                             PlanningMode::SCurve);
    near("short S-curve duration", very_short_scurve.duration(), 0.4, 1e-8);

    const LineInterpolator zero(start, start, constraints(), PlanningMode::SCurve);
    near("zero line duration", zero.duration(), 0.0, kTol);
    near_vec("zero line sample", zero.sample(12.0).position_m, start, kTol);
    near("zero line progress", zero.sample(12.0).path_progress_m, 0.0, kTol);
}

void test_line_constraints_and_continuity() {
    const LineInterpolator line(Vector3{}, Vector3{0.03, 0.04, 0.05}, constraints(),
                                PlanningMode::SCurve);
    double max_v = 0.0;
    double max_a = 0.0;
    double max_j = 0.0;
    for (int i = 0; i <= 3000; ++i) {
        const auto state = line.sample(i * 0.001);
        max_v = std::max(max_v, state.velocity_mps.norm());
        max_a = std::max(max_a, state.acceleration_mps2.norm());
        if (std::isfinite(state.jerk_mps3.norm())) {
            max_j = std::max(max_j, state.jerk_mps3.norm());
        }
    }
    require(max_v <= 0.05 + 1e-9, "line speed exceeds scalar limit");
    require(max_a <= 0.1 + 1e-9, "line acceleration exceeds scalar limit");
    require(max_j <= 0.5 + 1e-8, "line jerk exceeds scalar limit");
    const auto& segments = line.scalar_profile();
    // Explicitly check position, velocity and acceleration continuity around
    // every S-curve phase boundary exposed by the reusable planner.
    const auto* profile = dynamic_cast<const motioncore::planning::SCurveProfile*>(&segments);
    if (profile != nullptr) {
        for (const auto& segment : profile->segments()) {
            if (segment.start_time_s <= 0.0 || segment.start_time_s >= line.duration()) {
                continue;
            }
            const auto left = line.sample(segment.start_time_s - 1e-10);
            const auto right = line.sample(segment.start_time_s + 1e-10);
            near("line boundary position", left.position_m.x, right.position_m.x, 1e-8);
            near("line boundary velocity", left.velocity_mps.x, right.velocity_mps.x, 1e-8);
            near("line boundary acceleration", left.acceleration_mps2.x,
                 right.acceleration_mps2.x, 1e-8);
        }
    }
}

void test_line_invalid_constraints() {
    const Vector3 start{};
    const Vector3 target{0.01, 0.0, 0.0};
    expect_invalid([&] {
        LineInterpolator(start, target, MotionConstraints{-0.01, 0.1, 0.5}, PlanningMode::SCurve);
    }, "negative line velocity constraint");
    expect_invalid([&] {
        LineInterpolator(start, target, MotionConstraints{0.05, 0.0, 0.5}, PlanningMode::SCurve);
    }, "zero line acceleration constraint");
    expect_invalid([&] {
        LineInterpolator(start, target,
                         MotionConstraints{0.05, 0.1,
                                            std::numeric_limits<double>::quiet_NaN()},
                         PlanningMode::SCurve);
    }, "NaN line jerk constraint");
    expect_invalid([&] {
        LineInterpolator(Vector3{std::numeric_limits<double>::infinity(), 0.0, 0.0},
                         target, constraints(), PlanningMode::SCurve);
    }, "infinite line endpoint");
}

void test_arc_interpolation() {
    const Vector3 center{0.0, 0.0, 0.0};
    const Vector3 start{0.02, 0.0, 0.0};
    const Vector3 target{0.0, 0.02, 0.0};
    const MotionConstraints arc_constraints{0.02, 0.04, 0.2};
    const ArcResultantLimits generous_limits{0.1, 0.5};
    const ArcInterpolator ccw(start, target, center, ArcDirection::CCW, arc_constraints,
                              PlanningMode::SCurve, false, generous_limits);
    near("CCW radius", ccw.radius_m(), 0.02, 1e-12);
    near("CCW arc length", ccw.path_length_m(), 0.5 * kPi * 0.02, 1e-12);
    near("CCW duration", ccw.duration(), 2.2707963267948966, 1e-8);
    near_vec("CCW start", ccw.sample(0.0).position_m, start, 1e-12);
    near_vec("CCW pre-start", ccw.sample(-0.2).position_m, start, 1e-12);
    near_vec("CCW end", ccw.sample(ccw.duration()).position_m, target, 1e-12);
    near_vec("CCW post-end", ccw.sample(ccw.duration() + 0.2).position_m, target, 1e-12);
    const auto midpoint = ccw.sample(time_at_progress(ccw, ccw.path_length_m() * 0.5));
    near_vec("CCW midpoint", midpoint.position_m,
             Vector3{0.02 / std::sqrt(2.0), 0.02 / std::sqrt(2.0), 0.0}, 1e-9);
    require(ccw.satisfies_resultant_limits(), "valid CCW arc violates resultant limits");

    const ArcInterpolator cw(start, target, center, ArcDirection::CW, arc_constraints,
                             PlanningMode::SCurve);
    require(cw.path_length_m() > ccw.path_length_m(),
            "CW 90-degree command should take the long way around to the same endpoint");
    near_vec("CW end", cw.sample(cw.duration()).position_m, target, 1e-12);

    // Crossing the atan2 branch cut (350 degrees -> 10 degrees) must follow
    // the short positive CCW arc, not rotate almost a full revolution.
    const double ten_degrees = 10.0 * kPi / 180.0;
    const double three_fifty_degrees = 350.0 * kPi / 180.0;
    const Vector3 cross_zero_start{0.02 * std::cos(three_fifty_degrees),
                                   0.02 * std::sin(three_fifty_degrees), 0.0};
    const Vector3 cross_zero_target{0.02 * std::cos(ten_degrees),
                                    0.02 * std::sin(ten_degrees), 0.0};
    const ArcInterpolator cross_zero(cross_zero_start, cross_zero_target, center,
                                     ArcDirection::CCW, arc_constraints, PlanningMode::SCurve);
    near("cross-zero angle", cross_zero.path_length_m(), 20.0 * kPi / 180.0 * 0.02, 1e-12);

    const Vector3 opposite{-0.02, 0.0, 0.0};
    const ArcInterpolator half_ccw(start, opposite, center, ArcDirection::CCW,
                                   arc_constraints, PlanningMode::SCurve);
    near("180-degree arc length", half_ccw.path_length_m(), kPi * 0.02, 1e-12);

    // A CW command from 0 degrees to +90 degrees traverses 270 degrees and
    // therefore exercises angle wrapping beyond a half turn.
    near("long CW arc length", cw.path_length_m(), 1.5 * kPi * 0.02, 1e-12);

    // Equal start/end points are a zero move by default, but become an
    // explicit full circle when requested.
    const ArcInterpolator zero(start, start, center, ArcDirection::CCW, arc_constraints,
                               PlanningMode::SCurve);
    near("zero arc length", zero.path_length_m(), 0.0, kTol);
    const ArcInterpolator full(start, start, center, ArcDirection::CCW, arc_constraints,
                               PlanningMode::SCurve, true);
    near("full-circle length", full.path_length_m(), 2.0 * kPi * 0.02, 1e-12);
    near_vec("full-circle endpoint", full.sample(full.duration()).position_m, start, 1e-12);
}

void test_arc_validation() {
    const MotionConstraints c{0.02, 0.04, 0.2};
    expect_invalid([&] {
        ArcInterpolator(Vector3{0.02, 0.0, 0.0}, Vector3{0.0, 0.03, 0.0}, Vector3{},
                        ArcDirection::CCW, c);
    }, "unequal radii");
    expect_invalid([&] {
        ArcInterpolator(Vector3{0.0, 0.0, 0.0}, Vector3{0.0, 0.02, 0.0}, Vector3{},
                        ArcDirection::CCW, c);
    }, "zero radius");
    expect_invalid([&] {
        ArcInterpolator(Vector3{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0},
                        Vector3{0.0, 0.02, 0.0}, Vector3{}, ArcDirection::CCW, c);
    }, "NaN arc point");
    expect_invalid([&] {
        ArcInterpolator(Vector3{0.02, 0.0, 0.0}, Vector3{0.0, 0.02, 0.0}, Vector3{},
                        ArcDirection::CCW, MotionConstraints{0.0, 0.04, 0.2});
    }, "non-positive speed constraint");
    expect_invalid([&] {
        ArcInterpolator(Vector3{0.02, 0.0, 0.0}, Vector3{0.0, 0.02, 0.0}, Vector3{},
                        ArcDirection::CCW,
                        MotionConstraints{0.02, 0.04, std::numeric_limits<double>::infinity()});
    }, "infinite jerk constraint");
    expect_invalid([&] {
        ArcInterpolator(Vector3{0.02, 0.0, 0.0}, Vector3{0.0, 0.02, 0.0}, Vector3{},
                        static_cast<ArcDirection>(99), c);
    }, "invalid arc direction");
    expect_invalid([&] {
        ArcInterpolator(Vector3{0.02, 0.0, 0.001}, Vector3{0.0, 0.02, 0.0}, Vector3{},
                        ArcDirection::CCW, c);
    }, "non-planar arc");
    expect_invalid([&] {
        ArcInterpolator(Vector3{0.02, 0.0, 0.0}, Vector3{0.0, 0.02, 0.0}, Vector3{},
                        ArcDirection::CCW, c, PlanningMode::SCurve, false,
                        ArcResultantLimits{-1.0, 0.5});
    }, "negative resultant limit");
}

void test_segment_queue_and_corner_stop() {
    const MotionConstraints c{0.05, 0.1, 0.5};
    const LineInterpolator first(Vector3{}, Vector3{0.01, 0.0, 0.0}, c, PlanningMode::SCurve);
    const LineInterpolator second(Vector3{0.01, 0.0, 0.0}, Vector3{0.01, 0.01, 0.0}, c,
                                  PlanningMode::SCurve);
    CoordinatedTrajectoryPlanner planner;
    planner.add_line(first);
    planner.add_line(second);
    near("queued duration", planner.duration(), first.duration() + second.duration(), 1e-12);
    require(planner.segments().size() == 2, "two queued segments expected");
    const auto corner = planner.sample(first.duration());
    near_vec("corner position", corner.position_m, Vector3{0.01, 0.0, 0.0}, 1e-12);
    near("corner X velocity", corner.velocity_mps.x, 0.0, 1e-10);
    near("corner Y velocity", corner.velocity_mps.y, 0.0, 1e-10);
    const auto end = planner.sample(planner.duration());
    near_vec("queued endpoint", end.position_m, Vector3{0.01, 0.01, 0.0}, 1e-12);
    near("queued endpoint progress", end.path_progress_m,
         first.path_length_m() + second.path_length_m(), 1e-12);

    // Variant construction and explicit timeline should produce the same
    // corner behavior as add_line().
    CoordinatedTrajectoryPlanner from_segments(
        std::vector<MotionSegment>{MotionSegment(first), MotionSegment(second)});
    near("variant queued duration", from_segments.duration(), planner.duration(), 1e-12);
    near_vec("variant endpoint", from_segments.sample(from_segments.duration()).position_m,
             end.position_m, 1e-12);

    planner.clear();
    near("clear duration", planner.duration(), 0.0, kTol);
    require(planner.segments().empty(), "clear should remove all segments");
}

void test_multi_axis_servo_tracking_and_validation() {
    const MotionConstraints c{0.05, 0.1, 0.5};
    const LineInterpolator line(Vector3{}, Vector3{0.03, 0.04, 0.05}, c,
                                PlanningMode::SCurve);

    auto configs = motioncore::multiaxis::MultiAxisServoSystem::default_configs();
    // Each axis remains an independent plant.  Deliberately use different
    // masses/damping to ensure the system does not accidentally share state.
    configs[1].dynamics.mass_kg = 0.8;
    configs[1].controller.max_drive_force_n = configs[1].dynamics.max_drive_force_n;
    configs[2].dynamics.viscous_damping_n_s_per_m = 4.0;
    configs[2].controller.viscous_damping_n_s_per_m =
        configs[2].dynamics.viscous_damping_n_s_per_m;
    motioncore::multiaxis::MultiAxisServoSystem servo(configs);

    constexpr double dt = 0.001;
    const double simulation_time = line.duration() + 2.0;
    double maximum_error = 0.0;
    for (int i = 0; i <= static_cast<int>(simulation_time / dt); ++i) {
        const double time = std::min(i * dt, simulation_time);
        const auto reference = line.sample(std::min(time, line.duration()));
        const auto& state = servo.step(reference, dt);
        const Vector3 error = state.reference.position_m - state.actual_position_m;
        maximum_error = std::max(maximum_error, error.norm());
        require(std::isfinite(state.time_s), "multi-axis time became non-finite");
        require(state.actual_position_m.is_finite(), "multi-axis position became non-finite");
        require(state.actual_velocity_mps.is_finite(), "multi-axis speed became non-finite");
        near("shared axis time", state.time_s, (i + 1) * dt, 1e-12);
    }

    const auto& final_state = servo.state();
    const Vector3 target{0.03, 0.04, 0.05};
    require((target - final_state.actual_position_m).norm() < 0.002,
            "three-axis hold did not converge after two seconds");
    require(final_state.actual_velocity_mps.norm() < 0.005,
            "three-axis final speed is not settled");
    require(maximum_error < 0.01, "multi-axis closed-loop tracking error is unexpectedly large");

    expect_invalid([&] { servo.step(line.sample(0.1), 0.0); }, "zero servo dt");
    expect_invalid([&] { servo.step(line.sample(0.1), -0.001); }, "negative servo dt");
    auto nonfinite_reference = line.sample(0.1);
    nonfinite_reference.position_m.x = std::numeric_limits<double>::quiet_NaN();
    expect_invalid([&] { servo.step(nonfinite_reference, dt); }, "non-finite reference");
}

}  // namespace

int main() {
    try {
        test_vector3_operations();
        test_line_interpolation_and_branches();
        test_line_constraints_and_continuity();
        test_line_invalid_constraints();
        test_arc_interpolation();
        test_arc_validation();
        test_segment_queue_and_corner_stop();
        test_multi_axis_servo_tracking_and_validation();
        std::cout << "MotionCore Phase 4 geometry tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "MotionCore Phase 4 test failure: " << error.what() << '\n';
        return 1;
    }
}

#include "motioncore/control/cascade_servo_controller.hpp"
#include "motioncore/model/dynamic_axis.hpp"
#include "motioncore/planning/s_curve_profile.hpp"
#include "motioncore/planning/trapezoidal_profile.hpp"
#include "motioncore/planning/trajectory_profile.hpp"

#include <cmath>
#include <exception>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {

using motioncore::planning::MotionConstraints;
using motioncore::planning::SCurveProfile;
using motioncore::planning::TrajectoryRequest;
using motioncore::planning::TrajectoryState;
using motioncore::planning::TrajectorySampler;
using motioncore::planning::TrapezoidalProfile;

constexpr double kTol = 1e-9;

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void near(const std::string& name, double actual, double expected, double tolerance) {
    if (std::isnan(actual) || std::abs(actual - expected) > tolerance) {
        throw std::runtime_error(name + ": actual=" + std::to_string(actual) +
                                 ", expected=" + std::to_string(expected) +
                                 ", tolerance=" + std::to_string(tolerance));
    }
}

TrajectoryRequest standard_request(double target_m = 0.1) {
    TrajectoryRequest request;
    request.start_position_m = 0.0;
    request.target_position_m = target_m;
    request.start_velocity_mps = 0.0;
    request.end_velocity_mps = 0.0;
    request.constraints = MotionConstraints{0.05, 0.1, 0.5};
    return request;
}

void expect_invalid(const TrajectoryRequest& request, bool s_curve) {
    bool thrown = false;
    try {
        if (s_curve) {
            SCurveProfile profile(request);
            (void)profile;
        } else {
            TrapezoidalProfile profile(request);
            (void)profile;
        }
    } catch (const std::invalid_argument&) {
        thrown = true;
    }
    require(thrown, "illegal trajectory parameters were not rejected");
}

void test_trapezoid_standard() {
    const auto request = standard_request();
    const TrapezoidalProfile profile(request);

    // Independent calculation: ta = vmax/amax = .5 s, distance in both
    // ramps = vmax^2/amax = .025 m, so tc = 1.5 s and T = 2.5 s.
    near("trapezoid duration", profile.duration(), 2.5, 1e-7);
    require(profile.is_trapezoidal(), "standard move must have a cruise segment");
    near("trapezoid peak velocity", profile.peak_velocity_mps(), 0.05, 1e-10);

    const auto before = profile.sample(-0.1);
    const auto start = profile.sample(0.0);
    const auto end = profile.sample(profile.duration());
    const auto after = profile.sample(profile.duration() + 0.1);
    near("trapezoid start position", start.position_m, 0.0, kTol);
    near("trapezoid start velocity", start.velocity_mps, 0.0, kTol);
    near("trapezoid endpoint", end.position_m, 0.1, kTol);
    near("trapezoid end velocity", end.velocity_mps, 0.0, kTol);
    near("trapezoid post-end position", after.position_m, 0.1, kTol);
    near("trapezoid pre-start position", before.position_m, 0.0, kTol);
    require(std::isnan(start.jerk_mps3),
            "trapezoid jerk must be marked discontinuous (NaN)");

    const auto acceleration = profile.sample(0.25);
    const auto cruise = profile.sample(1.0);
    near("trapezoid acceleration", acceleration.acceleration_mps2, 0.1, kTol);
    near("trapezoid ramp position", acceleration.position_m, 0.003125, kTol);
    near("trapezoid cruise velocity", cruise.velocity_mps, 0.05, kTol);
}

void test_trapezoid_branches() {
    auto short_request = standard_request(0.01);
    const TrapezoidalProfile triangular(short_request);
    // Independent triangular formula T = 2*sqrt(distance/amax).
    near("triangular duration", triangular.duration(), 2.0 * std::sqrt(0.01 / 0.1), 1e-7);
    require(!triangular.is_trapezoidal(), "short move must be triangular");
    near("triangular peak velocity", triangular.peak_velocity_mps(), std::sqrt(0.01 * 0.1), 1e-9);
    const auto tri_end = triangular.sample(triangular.duration());
    near("triangular endpoint", tri_end.position_m, 0.01, kTol);
    near("triangular end speed", tri_end.velocity_mps, 0.0, kTol);

    const TrapezoidalProfile reverse(standard_request(-0.1));
    const auto reverse_mid = reverse.sample(reverse.duration() * 0.5);
    const auto reverse_end = reverse.sample(reverse.duration());
    require(reverse_mid.velocity_mps < 0.0, "reverse velocity must be negative");
    near("reverse endpoint", reverse_end.position_m, -0.1, kTol);
    near("reverse end speed", reverse_end.velocity_mps, 0.0, kTol);

    auto zero_request = standard_request(0.0);
    const TrapezoidalProfile zero(zero_request);
    near("zero duration", zero.duration(), 0.0, kTol);
    for (double time : {-1.0, 0.0, 0.123, 2.0}) {
        const auto state = zero.sample(time);
        near("zero position", state.position_m, 0.0, kTol);
        near("zero velocity", state.velocity_mps, 0.0, kTol);
        near("zero acceleration", state.acceleration_mps2, 0.0, kTol);
    }

    TrajectorySampler sampler(triangular);
    near("sampler duration", sampler.duration(), triangular.duration(), kTol);
    near("sampler arbitrary time", sampler.sample(0.123).time_s, 0.123, kTol);
}

void test_scurve_standard_and_limits() {
    const auto request = standard_request();
    const SCurveProfile profile(request);

    // For these values amax/jmax = .2 s, the constant-acceleration part is
    // .3 s, and the complete acceleration half is .7 s.  The cruise is 1.3 s.
    near("S-curve duration", profile.duration(), 2.7, 1e-7);
    near("S-curve t=.2 position", profile.sample(0.2).position_m,
         0.0006666666666666667, 1e-8);
    near("S-curve t=.2 velocity", profile.sample(0.2).velocity_mps, 0.01, 1e-9);
    near("S-curve t=.2 acceleration", profile.sample(0.2).acceleration_mps2, 0.1, 1e-9);
    // t=.2 is exactly a jerk-to-constant-acceleration boundary.  Position,
    // speed and acceleration are continuous there; jerk is intentionally
    // allowed to switch from +J to zero at the boundary.
    near("S-curve t=.7 position", profile.sample(0.7).position_m, 0.0175, 1e-8);
    near("S-curve t=.7 velocity", profile.sample(0.7).velocity_mps, 0.05, 1e-9);

    const auto start = profile.sample(0.0);
    const auto end = profile.sample(profile.duration());
    near("S-curve start position", start.position_m, 0.0, kTol);
    near("S-curve start speed", start.velocity_mps, 0.0, kTol);
    near("S-curve endpoint", end.position_m, 0.1, kTol);
    near("S-curve end speed", end.velocity_mps, 0.0, kTol);
    near("S-curve end acceleration", end.acceleration_mps2, 0.0, kTol);
    require(profile.segments().size() == 7, "standard S-curve must expose seven segments");

    double max_velocity = 0.0;
    double max_acceleration = 0.0;
    double max_jerk = 0.0;
    for (int i = 0; i <= 2700; ++i) {
        const auto state = profile.sample(i * 0.001);
        max_velocity = std::max(max_velocity, std::abs(state.velocity_mps));
        max_acceleration = std::max(max_acceleration, std::abs(state.acceleration_mps2));
        if (std::isfinite(state.jerk_mps3)) {
            max_jerk = std::max(max_jerk, std::abs(state.jerk_mps3));
        }
    }
    require(max_velocity <= 0.05 + 1e-9, "S-curve exceeds velocity limit");
    require(max_acceleration <= 0.1 + 1e-9, "S-curve exceeds acceleration limit");
    require(max_jerk <= 0.5 + 1e-9, "S-curve exceeds jerk limit");

    // Position, speed, and acceleration must agree at every exposed segment boundary.
    for (std::size_t i = 1; i < profile.segments().size(); ++i) {
        const auto& previous = profile.segments()[i - 1];
        const auto& current = profile.segments()[i];
        const auto left = profile.sample(current.start_time_s - 1e-10);
        const auto right = profile.sample(current.start_time_s + 1e-10);
        near("S-curve boundary position", left.position_m, right.position_m, 1e-8);
        near("S-curve boundary velocity", left.velocity_mps, right.velocity_mps, 1e-8);
        near("S-curve boundary acceleration", left.acceleration_mps2, right.acceleration_mps2, 1e-8);
        near("segment start position", current.position_m,
             profile.sample(current.start_time_s).position_m, 1e-8);
        (void)previous;
    }
}

void test_scurve_short_and_reverse() {
    auto very_short = standard_request(0.001);
    const SCurveProfile short_profile(very_short);
    // For 1 mm with the stated limits, all constant-acceleration and cruise
    // portions disappear; the four jerk ramps take 4*(d/(2*j))^(1/3) = .4 s.
    near("short S-curve duration", short_profile.duration(), 0.4, 1e-7);
    const auto short_end = short_profile.sample(short_profile.duration());
    near("short S-curve endpoint", short_end.position_m, 0.001, 1e-8);
    near("short S-curve end speed", short_end.velocity_mps, 0.0, 1e-8);
    require(short_profile.segments().size() == 4 || short_profile.segments().size() == 7,
            "short S-curve should expose a jerk-limited degenerate profile");

    const SCurveProfile reverse(standard_request(-0.1));
    const auto middle = reverse.sample(reverse.duration() * 0.5);
    const auto end = reverse.sample(reverse.duration());
    require(middle.velocity_mps < 0.0, "reverse S-curve velocity must be negative");
    near("reverse S-curve endpoint", end.position_m, -0.1, 1e-8);
    near("reverse S-curve end speed", end.velocity_mps, 0.0, 1e-8);

    auto zero_request = standard_request(0.0);
    const SCurveProfile zero(zero_request);
    near("zero S-curve duration", zero.duration(), 0.0, kTol);
    near("zero S-curve position", zero.sample(5.0).position_m, 0.0, kTol);
}

void test_invalid_requests() {
    auto request = standard_request();
    for (auto invalid : {
             MotionConstraints{0.0, 0.1, 0.5},
             MotionConstraints{0.05, 0.0, 0.5},
             MotionConstraints{-0.05, 0.1, 0.5},
         }) {
        request.constraints = invalid;
        expect_invalid(request, false);
        expect_invalid(request, true);
    }
    request.constraints.max_jerk_mps3 = 0.0;
    // A trapezoid has mathematically discontinuous acceleration and therefore
    // does not consume a finite jerk limit; SCurve must reject it.
    expect_invalid(request, true);
    request = standard_request();
    request.constraints.max_velocity_mps = std::numeric_limits<double>::quiet_NaN();
    expect_invalid(request, false);
    expect_invalid(request, true);
    request = standard_request();
    request.constraints.max_acceleration_mps2 = std::numeric_limits<double>::infinity();
    expect_invalid(request, false);
    expect_invalid(request, true);
    request = standard_request();
    request.start_velocity_mps = 0.01;
    expect_invalid(request, false);
    expect_invalid(request, true);
}

void test_closed_loop_tracking() {
    const SCurveProfile profile(standard_request());
    motioncore::model::DynamicAxisConfig axis_config;
    axis_config.mass_kg = 1.0;
    axis_config.viscous_damping_n_s_per_m = 3.0;
    axis_config.max_drive_force_n = 20.0;
    motioncore::model::DynamicAxisModel axis(axis_config);
    motioncore::control::CascadeServoConfig controller_config;
    controller_config.position_kp_per_s = 8.0;
    controller_config.max_target_velocity_mps = 0.1;
    controller_config.max_target_acceleration_mps2 = 0.8;
    controller_config.speed_kp = 20.0;
    controller_config.speed_ki = 80.0;
    controller_config.max_drive_force_n = 20.0;
    motioncore::control::CascadedServoController controller(controller_config);

    const double dt = 0.001;
    const int count = static_cast<int>(std::ceil(profile.duration() / dt));
    double maximum_error = 0.0;
    for (int i = 0; i < count; ++i) {
        const auto reference = profile.sample(std::min(i * dt, profile.duration()));
        const auto output = controller.update(reference.position_m, axis.state(), dt);
        require(std::isfinite(output.drive_force_n), "closed-loop drive became non-finite");
        axis.step(output.drive_force_n, dt);
        maximum_error = std::max(maximum_error,
                                 std::abs(reference.position_m - axis.state().position_m));
    }
    // This measures the physical closed-loop error separately from the planner's
    // 1e-6 mm numerical target.  The plant is intentionally a simple dynamic model.
    require(maximum_error < 0.02, "closed-loop trajectory error is unexpectedly large");
    const auto final_reference = profile.sample(profile.duration());
    const auto final_state = axis.state();
    require(std::abs(final_reference.position_m - final_state.position_m) < 0.02,
            "closed-loop endpoint tracking failed");
    require(std::isfinite(final_state.velocity_mps), "closed-loop final velocity is non-finite");
}

}  // namespace

int main() {
    try {
        test_trapezoid_standard();
        test_trapezoid_branches();
        test_scurve_standard_and_limits();
        test_scurve_short_and_reverse();
        test_invalid_requests();
        test_closed_loop_tracking();
        std::cout << "MotionCore Phase 3 tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "MotionCore Phase 3 test failure: " << error.what() << '\n';
        return 1;
    }
}

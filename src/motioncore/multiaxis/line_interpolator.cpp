#include "motioncore/multiaxis/line_interpolator.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

#include "motioncore/planning/s_curve_profile.hpp"
#include "motioncore/planning/trapezoidal_profile.hpp"

namespace motioncore::multiaxis {

LineInterpolator::LineInterpolator(
    const Vector3& start_m, const Vector3& target_m,
    const planning::MotionConstraints& constraints, PlanningMode mode)
    : start_m_(start_m), target_m_(target_m), mode_(mode) {
    if (!start_m_.is_finite() || !target_m_.is_finite()) {
        throw std::invalid_argument("line endpoints must be finite");
    }
    const Vector3 delta = target_m_ - start_m_;
    length_m_ = delta.norm();
    if (length_m_ > 1e-15) {
        unit_direction_ = delta / length_m_;
    }
    const auto request = make_scalar_request(length_m_, constraints);
    if (mode_ == PlanningMode::Trapezoid) {
        profile_ = std::make_shared<planning::TrapezoidalProfile>(request);
    } else {
        profile_ = std::make_shared<planning::SCurveProfile>(request);
    }
}

MultiAxisTrajectoryState LineInterpolator::sample(double time_s) const {
    const planning::TrajectoryState scalar = profile_->sample(time_s);
    MultiAxisTrajectoryState state;
    state.time_s = scalar.time_s;
    state.path_progress_m = scalar.position_m;
    state.position = start_m_ + unit_direction_ * scalar.position_m;
    state.velocity = unit_direction_ * scalar.velocity_mps;
    state.acceleration = unit_direction_ * scalar.acceleration_mps2;
    state.position_m = state.position;
    state.velocity_mps = state.velocity;
    state.acceleration_mps2 = state.acceleration;
    if (std::isfinite(scalar.jerk_mps3)) {
        state.jerk = unit_direction_ * scalar.jerk_mps3;
    } else {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        state.jerk = {nan, nan, nan};
    }
    state.jerk_mps3 = state.jerk;
    // The scalar profiles force the exact target at their final sample. Keep
    // this assignment explicit so geometry and numerical error cannot drift.
    if (state.time_s >= duration()) {
        state.position = target_m_;
        state.velocity = {};
        state.acceleration = {};
        state.position_m = state.position;
        state.velocity_mps = state.velocity;
        state.acceleration_mps2 = state.acceleration;
        state.path_progress_m = length_m_;
        if (mode_ == PlanningMode::SCurve) state.jerk = {};
        state.jerk_mps3 = state.jerk;
    }
    return state;
}

double LineInterpolator::duration() const noexcept { return profile_->duration(); }

}  // namespace motioncore::multiaxis

#include "motioncore/planning/trajectory_profile.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace motioncore::planning {

TrajectoryState TrajectorySampler::sample(double time_s) const {
    if (profile_ == nullptr) {
        throw std::logic_error("TrajectorySampler has no profile");
    }
    return profile_->sample(time_s);
}

namespace {

void validate_request(const TrajectoryRequest& request, bool check_jerk) {
    const auto finite = [](double value) { return std::isfinite(value); };
    if (!finite(request.start_position_m) || !finite(request.target_position_m)
        || !finite(request.start_velocity_mps) || !finite(request.end_velocity_mps)) {
        throw std::invalid_argument("trajectory positions and boundary velocities must be finite");
    }
    if (std::abs(request.start_velocity_mps) > 1e-12
        || std::abs(request.end_velocity_mps) > 1e-12) {
        throw std::invalid_argument("only zero start and end velocities are supported");
    }
    const auto& c = request.constraints;
    if (!finite(c.max_velocity_mps) || c.max_velocity_mps <= 0.0) {
        throw std::invalid_argument("max_velocity_mps must be finite and positive");
    }
    if (!finite(c.max_acceleration_mps2) || c.max_acceleration_mps2 <= 0.0) {
        throw std::invalid_argument("max_acceleration_mps2 must be finite and positive");
    }
    if (check_jerk && (!finite(c.max_jerk_mps3) || c.max_jerk_mps3 <= 0.0)) {
        throw std::invalid_argument("max_jerk_mps3 must be finite and positive");
    }
}

double clamp_time(double time_s, double duration_s) {
    if (!std::isfinite(time_s)) {
        throw std::invalid_argument("sample time must be finite");
    }
    return std::clamp(time_s, 0.0, duration_s);
}

}  // namespace

// Keep validation in one translation unit so both profiles make exactly the same
// decision for boundary velocity and NaN/Inf inputs.
void validate_trajectory_request(const TrajectoryRequest& request, bool check_jerk) {
    validate_request(request, check_jerk);
}

double clamp_trajectory_time(double time_s, double duration_s) {
    return clamp_time(time_s, duration_s);
}

}  // namespace motioncore::planning

#include "motioncore/multiaxis/interpolator.hpp"

#include <cmath>
#include <stdexcept>

namespace motioncore::multiaxis {

planning::TrajectoryRequest make_scalar_request(
    double length_m, const planning::MotionConstraints& constraints) {
    if (!std::isfinite(length_m) || length_m < 0.0) {
        throw std::invalid_argument("trajectory path length must be finite and non-negative");
    }
    planning::TrajectoryRequest request;
    request.start_position_m = 0.0;
    request.target_position_m = length_m;
    request.start_velocity_mps = 0.0;
    request.end_velocity_mps = 0.0;
    request.constraints = constraints;
    return request;
}

}  // namespace motioncore::multiaxis

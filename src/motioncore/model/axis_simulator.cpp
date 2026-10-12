#include "motioncore/model/axis_simulator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace motioncore::model {

AxisSimulator::AxisSimulator(AxisLimits limits) : limits_(limits) {
    if (!std::isfinite(limits_.max_velocity) || limits_.max_velocity <= 0.0) {
        throw std::invalid_argument("max_velocity must be finite and positive");
    }
    if (!std::isfinite(limits_.max_acceleration) || limits_.max_acceleration <= 0.0) {
        throw std::invalid_argument("max_acceleration must be finite and positive");
    }
}

void AxisSimulator::reset(AxisState state) {
    state_ = state;
    state_.velocity = std::clamp(
        state_.velocity, -limits_.max_velocity, limits_.max_velocity);
    state_.acceleration = std::clamp(
        state_.acceleration, -limits_.max_acceleration, limits_.max_acceleration);
}

const AxisState& AxisSimulator::step(double acceleration_command, double dt_seconds) {
    if (!std::isfinite(dt_seconds) || dt_seconds <= 0.0) {
        throw std::invalid_argument("dt_seconds must be finite and positive");
    }

    const double limited_acceleration = std::clamp(
        acceleration_command,
        -limits_.max_acceleration,
        limits_.max_acceleration);
    const double old_velocity = state_.velocity;
    const double requested_velocity = old_velocity + limited_acceleration * dt_seconds;
    const double new_velocity = std::clamp(
        requested_velocity, -limits_.max_velocity, limits_.max_velocity);

    // Trapezoidal integration remains well behaved when the velocity limit is hit.
    state_.position += 0.5 * (old_velocity + new_velocity) * dt_seconds;
    state_.velocity = new_velocity;
    state_.acceleration = (new_velocity - old_velocity) / dt_seconds;
    state_.time_s += dt_seconds;
    return state_;
}

}  // namespace motioncore::model

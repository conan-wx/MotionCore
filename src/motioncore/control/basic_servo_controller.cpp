#include "motioncore/control/basic_servo_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace motioncore::control {

BasicServoController::BasicServoController(BasicServoConfig config) : config_(config) {
    if (!std::isfinite(config_.position_gain) || config_.position_gain <= 0.0) {
        throw std::invalid_argument("position_gain must be finite and positive");
    }
    if (!std::isfinite(config_.velocity_gain) || config_.velocity_gain <= 0.0) {
        throw std::invalid_argument("velocity_gain must be finite and positive");
    }
    if (!std::isfinite(config_.max_velocity) || config_.max_velocity <= 0.0) {
        throw std::invalid_argument("max_velocity must be finite and positive");
    }
    if (!std::isfinite(config_.max_acceleration) || config_.max_acceleration <= 0.0) {
        throw std::invalid_argument("max_acceleration must be finite and positive");
    }
}

double BasicServoController::update(double target_position,
                                    const model::AxisState& state,
                                    double dt_seconds) {
    if (!std::isfinite(dt_seconds) || dt_seconds <= 0.0) {
        throw std::invalid_argument("dt_seconds must be finite and positive");
    }

    position_error_ = target_position - state.position;
    desired_velocity_ = std::clamp(
        config_.position_gain * position_error_,
        -config_.max_velocity,
        config_.max_velocity);
    const double acceleration_command =
        config_.velocity_gain * (desired_velocity_ - state.velocity);
    return std::clamp(
        acceleration_command,
        -config_.max_acceleration,
        config_.max_acceleration);
}

}  // namespace motioncore::control

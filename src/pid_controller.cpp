#include "motion_control/pid_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace motion_control {

PIDController::PIDController(PIDConfig config) : config_(config) {
    if (config_.output_min > config_.output_max) {
        throw std::invalid_argument("output_min must not exceed output_max");
    }
    if (config_.integral_min > config_.integral_max) {
        throw std::invalid_argument("integral_min must not exceed integral_max");
    }
}

void PIDController::reset() {
    integral_ = 0.0;
    previous_error_ = 0.0;
    has_previous_error_ = false;
}

double PIDController::update(double setpoint, double measurement, double dt_seconds) {
    if (!std::isfinite(dt_seconds) || dt_seconds <= 0.0) {
        throw std::invalid_argument("dt_seconds must be finite and positive");
    }

    const double error = setpoint - measurement;
    integral_ = std::clamp(
        integral_ + error * dt_seconds,
        config_.integral_min,
        config_.integral_max);

    double derivative = 0.0;
    if (has_previous_error_) {
        derivative = (error - previous_error_) / dt_seconds;
    }

    previous_error_ = error;
    has_previous_error_ = true;

    const double command = config_.kp * error
        + config_.ki * integral_
        + config_.kd * derivative;
    return std::clamp(command, config_.output_min, config_.output_max);
}

}  // namespace motion_control

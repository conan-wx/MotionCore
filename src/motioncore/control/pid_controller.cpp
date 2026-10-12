#include "motioncore/control/pid_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace motioncore::control {

PIDController::PIDController(PIDControllerConfig config) : config_(config) {
    if (!std::isfinite(config_.kp) || !std::isfinite(config_.ki)
        || !std::isfinite(config_.kd) || !std::isfinite(config_.output_min)
        || !std::isfinite(config_.output_max) || !std::isfinite(config_.integral_min)
        || !std::isfinite(config_.integral_max)
        || config_.kp < 0.0 || config_.ki < 0.0 || config_.kd < 0.0
        || config_.output_min > config_.output_max
        || config_.integral_min > config_.integral_max
        || config_.derivative_filter_alpha < 0.0
        || config_.derivative_filter_alpha >= 1.0) {
        throw std::invalid_argument("invalid PID controller configuration");
    }
}

void PIDController::reset() noexcept {
    integral_state_ = 0.0;
    previous_measurement_ = 0.0;
    filtered_derivative_ = 0.0;
    last_output_ = 0.0;
    has_previous_measurement_ = false;
}

double PIDController::update(double setpoint, double measurement, double dt_seconds) {
    if (!std::isfinite(setpoint) || !std::isfinite(measurement)) {
        throw std::invalid_argument("PID setpoint and measurement must be finite");
    }
    if (!std::isfinite(dt_seconds) || dt_seconds <= 0.0) {
        throw std::invalid_argument("dt_seconds must be finite and positive");
    }

    const double error = setpoint - measurement;
    const double candidate_integral = std::clamp(
        integral_state_ + error * dt_seconds,
        config_.integral_min,
        config_.integral_max);

    // 对测量值求导而不是对设定值求导，避免位置指令阶跃造成微分冲击。
    double derivative_on_measurement = 0.0;
    if (has_previous_measurement_) {
        derivative_on_measurement = (measurement - previous_measurement_) / dt_seconds;
    }
    // 一阶低通滤波抑制测量噪声。alpha 越大，平滑越强、响应越慢。
    filtered_derivative_ = config_.derivative_filter_alpha * filtered_derivative_
        + (1.0 - config_.derivative_filter_alpha) * (-derivative_on_measurement);
    previous_measurement_ = measurement;
    has_previous_measurement_ = true;

    const double candidate_output = config_.kp * error
        + config_.ki * candidate_integral
        + config_.kd * filtered_derivative_;
    const double limited_candidate_output = std::clamp(
        candidate_output, config_.output_min, config_.output_max);
    const bool saturated_high = candidate_output > config_.output_max && error > 0.0;
    const bool saturated_low = candidate_output < config_.output_min && error < 0.0;
    if (!config_.anti_windup || (!saturated_high && !saturated_low)) {
        integral_state_ = candidate_integral;
    }

    last_output_ = limited_candidate_output;
    return last_output_;
}

}  // namespace motioncore::control

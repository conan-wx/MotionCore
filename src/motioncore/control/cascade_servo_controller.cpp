#include "motioncore/control/cascade_servo_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace motioncore::control {

CascadedServoController::CascadedServoController(CascadeServoConfig config)
    : config_(config),
      speed_p_({config.speed_kp, -config.max_drive_force_n, config.max_drive_force_n}),
      speed_pi_({config.speed_kp, config.speed_ki,
                 -config.max_drive_force_n, config.max_drive_force_n,
                 config.integral_min, config.integral_max, config.anti_windup}) {
    if (!std::isfinite(config_.position_kp_per_s)
        || !std::isfinite(config_.max_target_velocity_mps)
        || !std::isfinite(config_.max_target_acceleration_mps2)
        || !std::isfinite(config_.speed_kp) || !std::isfinite(config_.speed_ki)
        || !std::isfinite(config_.max_drive_force_n)
        || !std::isfinite(config_.integral_min)
        || !std::isfinite(config_.integral_max)
        || !std::isfinite(config_.viscous_damping_n_s_per_m)
        || config_.position_kp_per_s < 0.0
        || config_.max_target_velocity_mps <= 0.0
        || config_.max_target_acceleration_mps2 <= 0.0
        || config_.speed_kp < 0.0 || config_.speed_ki < 0.0
        || config_.max_drive_force_n <= 0.0
        || config_.integral_min > config_.integral_max
        || config_.viscous_damping_n_s_per_m < 0.0) {
        throw std::invalid_argument("invalid cascaded servo configuration");
    }
}

void CascadedServoController::reset() noexcept {
    previous_target_velocity_mps_ = 0.0;
    has_previous_target_velocity_ = false;
    speed_p_.reset();
    speed_pi_.reset();
}

CascadeServoOutput CascadedServoController::update(
    double target_position_m,
    const model::DynamicAxisState& measured_state,
    double dt_seconds) {
    if (!std::isfinite(target_position_m)
        || !std::isfinite(measured_state.position_m)
        || !std::isfinite(measured_state.velocity_mps)
        || !std::isfinite(dt_seconds) || dt_seconds <= 0.0) {
        throw std::invalid_argument("cascade input must be finite and dt positive");
    }

    return update_with_velocity_reference(target_position_m, 0.0,
                                          measured_state, dt_seconds);
}

CascadeServoOutput CascadedServoController::update_with_velocity_reference(
    double target_position_m,
    double reference_velocity_mps,
    const model::DynamicAxisState& measured_state,
    double dt_seconds) {
    if (!std::isfinite(target_position_m)
        || !std::isfinite(reference_velocity_mps)
        || !std::isfinite(measured_state.position_m)
        || !std::isfinite(measured_state.velocity_mps)
        || !std::isfinite(dt_seconds) || dt_seconds <= 0.0) {
        throw std::invalid_argument("cascade input must be finite and dt positive");
    }

    const double position_error = target_position_m - measured_state.position_m;
    const double unconstrained_velocity = reference_velocity_mps
        + config_.position_kp_per_s * position_error;
    const double velocity_limit = config_.max_target_velocity_mps;
    const double limited_velocity = std::clamp(
        unconstrained_velocity, -velocity_limit, velocity_limit);

    // 对速度指令做斜率限制，避免位置误差阶跃直接造成无限加速度。
    if (!has_previous_target_velocity_) {
        previous_target_velocity_mps_ = 0.0;
        has_previous_target_velocity_ = true;
    }
    const double max_delta = config_.max_target_acceleration_mps2 * dt_seconds;
    const double target_velocity = std::clamp(
        limited_velocity,
        previous_target_velocity_mps_ - max_delta,
        previous_target_velocity_mps_ + max_delta);
    previous_target_velocity_mps_ = target_velocity;

    const double speed_error = target_velocity - measured_state.velocity_mps;
    double drive_force = config_.speed_mode == SpeedControllerMode::P
        ? speed_p_.update(speed_error)
        : speed_pi_.update(speed_error, dt_seconds);
    if (config_.damping_feedforward) {
        // 粘性阻尼力约为 b*v，使用期望速度提前补偿它。
        drive_force += config_.viscous_damping_n_s_per_m * target_velocity;
    }
    const bool drive_saturated = std::abs(drive_force) >= config_.max_drive_force_n;
    drive_force = std::clamp(
        drive_force, -config_.max_drive_force_n, config_.max_drive_force_n);

    return {position_error, target_velocity, speed_error, drive_force, drive_saturated};
}

}  // namespace motioncore::control

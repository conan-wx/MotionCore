#include "motioncore/model/dynamic_axis.hpp"

#include <algorithm>
#include <stdexcept>

namespace motioncore::model {

double LoadProfile::force_at(double time_s) const noexcept {
    // 脉冲采用左闭右开区间，避免边界时间同时属于两段。
    const bool in_pulse = pulse_duration_s > 0.0
        && time_s >= pulse_start_s
        && time_s < pulse_start_s + pulse_duration_s;
    return constant_force_n + (in_pulse ? pulse_force_n : 0.0);
}

DynamicAxisModel::DynamicAxisModel(DynamicAxisConfig config) : config_(config) {
    if (!std::isfinite(config_.mass_kg) || config_.mass_kg <= 0.0) {
        throw std::invalid_argument("mass_kg must be finite and positive");
    }
    if (!std::isfinite(config_.viscous_damping_n_s_per_m)
        || config_.viscous_damping_n_s_per_m < 0.0) {
        throw std::invalid_argument("viscous damping must be finite and non-negative");
    }
    if (!std::isfinite(config_.max_drive_force_n) || config_.max_drive_force_n <= 0.0) {
        throw std::invalid_argument("max_drive_force_n must be finite and positive");
    }
    if (!std::isfinite(config_.initial_position_m)
        || !std::isfinite(config_.initial_velocity_mps)) {
        throw std::invalid_argument("initial position and velocity must be finite");
    }
    if (!std::isfinite(config_.load_profile.constant_force_n)
        || !std::isfinite(config_.load_profile.pulse_force_n)
        || !std::isfinite(config_.load_profile.pulse_start_s)
        || !std::isfinite(config_.load_profile.pulse_duration_s)
        || config_.load_profile.pulse_start_s < 0.0
        || config_.load_profile.pulse_duration_s < 0.0) {
        throw std::invalid_argument("load profile contains an invalid value");
    }
    reset();
}

void DynamicAxisModel::reset() {
    state_ = {};
    state_.position_m = config_.initial_position_m;
    state_.velocity_mps = config_.initial_velocity_mps;
    state_.load_force_n = config_.load_profile.force_at(0.0);
}

const DynamicAxisState& DynamicAxisModel::step(double requested_drive_force_n,
                                               double dt_seconds) {
    if (!std::isfinite(requested_drive_force_n)) {
        throw std::invalid_argument("requested drive force must be finite");
    }
    if (!std::isfinite(dt_seconds) || dt_seconds <= 0.0) {
        throw std::invalid_argument("dt_seconds must be finite and positive");
    }

    const double drive_force = std::clamp(
        requested_drive_force_n,
        -config_.max_drive_force_n,
        config_.max_drive_force_n);
    const double load_force = config_.load_profile.force_at(state_.time_s);
    const double acceleration = (
        drive_force
        - config_.viscous_damping_n_s_per_m * state_.velocity_mps
        - load_force) / config_.mass_kg;

    // 采用“当前拍加速度保持不变”的常加速度积分。这样既比直接给速度赋值
    // 更接近真实电机，又能让初学者清楚看到每个状态量是怎样更新的：
    // x[k+1] = x[k] + v[k]T + 0.5*a[k]T^2，v[k+1] = v[k] + a[k]T。
    state_.position_m += state_.velocity_mps * dt_seconds
        + 0.5 * acceleration * dt_seconds * dt_seconds;
    state_.velocity_mps += acceleration * dt_seconds;
    state_.acceleration_mps2 = acceleration;
    state_.drive_force_n = drive_force;
    state_.load_force_n = load_force;
    state_.time_s += dt_seconds;
    return state_;
}

}  // namespace motioncore::model

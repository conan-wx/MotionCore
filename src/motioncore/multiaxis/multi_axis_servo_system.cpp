#include "motioncore/multiaxis/multi_axis_servo_system.hpp"

#include <cmath>
#include <stdexcept>

namespace motioncore::multiaxis {

std::array<MultiAxisServoAxisConfig, 3> MultiAxisServoSystem::default_configs() {
    std::array<MultiAxisServoAxisConfig, 3> configs{};
    for (auto& config : configs) {
        config.controller.position_kp_per_s = 8.0;
        config.controller.max_target_velocity_mps = 0.1;
        config.controller.max_target_acceleration_mps2 = 0.8;
        config.controller.speed_mode = control::SpeedControllerMode::PI;
        config.controller.speed_kp = 20.0;
        config.controller.speed_ki = 80.0;
        config.controller.max_drive_force_n = config.dynamics.max_drive_force_n;
        config.controller.integral_min = -0.25;
        config.controller.integral_max = 0.25;
        config.controller.anti_windup = true;
        config.controller.viscous_damping_n_s_per_m =
            config.dynamics.viscous_damping_n_s_per_m;
    }
    return configs;
}

MultiAxisServoSystem::MultiAxisServoSystem(
    std::array<MultiAxisServoAxisConfig, 3> configs)
    : axes_{model::DynamicAxisModel(configs[0].dynamics),
            model::DynamicAxisModel(configs[1].dynamics),
            model::DynamicAxisModel(configs[2].dynamics)},
      controllers_{control::CascadedServoController(configs[0].controller),
                   control::CascadedServoController(configs[1].controller),
                   control::CascadedServoController(configs[2].controller)} {
    reset();
}

void MultiAxisServoSystem::reset() {
    for (auto& axis : axes_) {
        axis.reset();
    }
    for (auto& controller : controllers_) {
        controller.reset();
    }
    state_ = {};
    for (std::size_t index = 0; index < axes_.size(); ++index) {
        const auto& axis = axes_[index].state();
        if (index == 0) {
            state_.time_s = axis.time_s;
        }
        state_.actual_position_m.x = index == 0 ? axis.position_m : state_.actual_position_m.x;
        state_.actual_position_m.y = index == 1 ? axis.position_m : state_.actual_position_m.y;
        state_.actual_position_m.z = index == 2 ? axis.position_m : state_.actual_position_m.z;
        state_.actual_velocity_mps.x = index == 0 ? axis.velocity_mps : state_.actual_velocity_mps.x;
        state_.actual_velocity_mps.y = index == 1 ? axis.velocity_mps : state_.actual_velocity_mps.y;
        state_.actual_velocity_mps.z = index == 2 ? axis.velocity_mps : state_.actual_velocity_mps.z;
        state_.actual_acceleration_mps2.x = index == 0 ? axis.acceleration_mps2 : state_.actual_acceleration_mps2.x;
        state_.actual_acceleration_mps2.y = index == 1 ? axis.acceleration_mps2 : state_.actual_acceleration_mps2.y;
        state_.actual_acceleration_mps2.z = index == 2 ? axis.acceleration_mps2 : state_.actual_acceleration_mps2.z;
        state_.load_force_n.x = index == 0 ? axis.load_force_n : state_.load_force_n.x;
        state_.load_force_n.y = index == 1 ? axis.load_force_n : state_.load_force_n.y;
        state_.load_force_n.z = index == 2 ? axis.load_force_n : state_.load_force_n.z;
    }
}

void MultiAxisServoSystem::adopt_physical_state(const MultiAxisServoSystem& other) {
    axes_ = other.axes_;
    state_ = other.state_;
    // 积分器属于控制器内部历史，不应把回零阶段的速度指令带入第一拍普通运动。
    for (auto& controller : controllers_) controller.reset();
}

const MultiAxisServoState& MultiAxisServoSystem::step(
    const MultiAxisTrajectoryState& reference, double dt_seconds) {
    if (!std::isfinite(dt_seconds) || dt_seconds <= 0.0
        || !reference.position_m.is_finite()
        || !reference.velocity_mps.is_finite()) {
        throw std::invalid_argument("multi-axis reference and dt must be finite");
    }

    const std::array<double, 3> target_positions{
        reference.position_m.x, reference.position_m.y, reference.position_m.z};
    const std::array<double, 3> target_velocities{
        reference.velocity_mps.x, reference.velocity_mps.y, reference.velocity_mps.z};
    std::array<double, 3> drive_forces{};
    for (std::size_t index = 0; index < axes_.size(); ++index) {
        const auto command = controllers_[index].update_with_velocity_reference(
            target_positions[index], target_velocities[index], axes_[index].state(), dt_seconds);
        drive_forces[index] = command.drive_force_n;
        state_.drive_saturated[index] = command.drive_saturated;
    }
    // 所有控制器都先算出本拍指令，再统一推进三个模型，保证共享仿真时钟。
    for (std::size_t index = 0; index < axes_.size(); ++index) {
        axes_[index].step(drive_forces[index], dt_seconds);
    }

    state_.time_s += dt_seconds;
    state_.reference = reference;
    state_.reference.time_s = state_.time_s;
    state_.drive_force_n = {drive_forces[0], drive_forces[1], drive_forces[2]};
    for (std::size_t index = 0; index < axes_.size(); ++index) {
        const auto& axis = axes_[index].state();
        if (index == 0) {
            state_.actual_position_m.x = axis.position_m;
            state_.actual_velocity_mps.x = axis.velocity_mps;
            state_.actual_acceleration_mps2.x = axis.acceleration_mps2;
            state_.load_force_n.x = axis.load_force_n;
        } else if (index == 1) {
            state_.actual_position_m.y = axis.position_m;
            state_.actual_velocity_mps.y = axis.velocity_mps;
            state_.actual_acceleration_mps2.y = axis.acceleration_mps2;
            state_.load_force_n.y = axis.load_force_n;
        } else {
            state_.actual_position_m.z = axis.position_m;
            state_.actual_velocity_mps.z = axis.velocity_mps;
            state_.actual_acceleration_mps2.z = axis.acceleration_mps2;
            state_.load_force_n.z = axis.load_force_n;
        }
    }
    return state_;
}

const MultiAxisServoState& MultiAxisServoSystem::step_drive_disabled(double dt_seconds) {
    if (!std::isfinite(dt_seconds) || dt_seconds <= 0.0) {
        throw std::invalid_argument("disabled drive dt must be finite and positive");
    }
    for (std::size_t index = 0; index < axes_.size(); ++index) {
        axes_[index].step(0.0, dt_seconds);
        state_.drive_saturated[index] = false;
    }
    state_.time_s += dt_seconds;
    state_.drive_force_n = {};
    for (std::size_t index = 0; index < axes_.size(); ++index) {
        const auto& axis = axes_[index].state();
        if (index == 0) {
            state_.actual_position_m.x = axis.position_m;
            state_.actual_velocity_mps.x = axis.velocity_mps;
            state_.actual_acceleration_mps2.x = axis.acceleration_mps2;
            state_.load_force_n.x = axis.load_force_n;
        } else if (index == 1) {
            state_.actual_position_m.y = axis.position_m;
            state_.actual_velocity_mps.y = axis.velocity_mps;
            state_.actual_acceleration_mps2.y = axis.acceleration_mps2;
            state_.load_force_n.y = axis.load_force_n;
        } else {
            state_.actual_position_m.z = axis.position_m;
            state_.actual_velocity_mps.z = axis.velocity_mps;
            state_.actual_acceleration_mps2.z = axis.acceleration_mps2;
            state_.load_force_n.z = axis.load_force_n;
        }
    }
    return state_;
}

}  // namespace motioncore::multiaxis

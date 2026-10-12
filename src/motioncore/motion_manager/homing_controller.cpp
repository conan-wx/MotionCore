#include "motioncore/motion_manager/homing_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace motioncore::manager {
namespace {

constexpr double kAxisEpsilon = 1e-12;

int sign_of(HomeDirection direction) {
    return direction == HomeDirection::Positive ? 1 : -1;
}

bool finite_positive(double value) {
    return std::isfinite(value) && value > 0.0;
}

}  // namespace

HomingController::HomingController(
    HomingConfig config,
    std::array<multiaxis::MultiAxisServoAxisConfig, 3> servo_configs)
    : config_(std::move(config)), servo_(std::move(servo_configs)) {
    validate_config(config_);
    reset();
}

void HomingController::validate_config(const HomingConfig& config) {
    std::array<bool, 3> seen{{false, false, false}};
    for (const std::size_t axis : config.order) {
        if (axis >= 3U || seen[axis]) {
            throw std::invalid_argument("homing order must contain each axis exactly once");
        }
        seen[axis] = true;
    }
    for (const auto& axis : config.axes) {
        const int direction = sign_of(axis.sensor.direction);
        (void)direction;
        if (!std::isfinite(axis.sensor.trigger_position_m)
            || !finite_positive(axis.fast_speed_mps)
            || !finite_positive(axis.slow_speed_mps)
            || !finite_positive(axis.backoff_distance_m)
            || !finite_positive(axis.search_range_m)
            || !finite_positive(axis.timeout_s)
            || !std::isfinite(axis.offset_m)
            || !finite_positive(axis.position_tolerance_m)
            || !std::isfinite(axis.sensor.debounce_time_s)
            || axis.sensor.debounce_time_s < 0.0
            || (axis.sensor.simulate_stuck_active && axis.sensor.simulate_stuck_inactive)
            || axis.slow_speed_mps > axis.fast_speed_mps) {
            throw std::invalid_argument("invalid homing axis configuration");
        }
    }
}

void HomingController::reset() {
    servo_.reset();
    status_ = {};
    command_position_m_ = servo_.state().actual_position_m;
    machine_offsets_m_ = {};
    debounce_candidate_ = {{false, false, false}};
    debounce_elapsed_ = {{0.0, 0.0, 0.0}};
    backoff_target_m_ = {{0.0, 0.0, 0.0}};
    completion_settle_elapsed_s_ = 0.0;
    for (std::size_t axis = 0; axis < 3U; ++axis) {
        auto& axis_status = status_.axes[axis];
        axis_status.phase = HomingPhase::Idle;
        axis_status.search_start_position_m = axis_component(command_position_m_, axis);
        axis_status.trigger_position_m = config_.axes[axis].sensor.trigger_position_m;
        axis_status.machine_position_m = axis_status.search_start_position_m;
        axis_status.sensor_active = false;
        axis_status.sensor_level = !config_.axes[axis].sensor.active_high;
    }
}

bool HomingController::start() {
    if (status_.running || status_.fault || status_.emergency_stopped) {
        return false;
    }
    // 即使之前已经回零，显式 Home 也应重新执行搜索，而不是复用旧的完成标志。
    status_.time_s = 0.0;
    status_.active_axis = config_.order[0];
    status_.running = true;
    status_.completed = false;
    completion_settle_elapsed_s_ = 0.0;
    const auto& state = servo_.state();
    command_position_m_ = state.actual_position_m;
    for (auto& axis : status_.axes) {
        axis.phase = HomingPhase::Idle;
        axis.homed = false;
        axis.elapsed_s = 0.0;
        axis.message.clear();
    }
    auto& active = status_.axes[status_.active_axis];
    active.phase = HomingPhase::SearchFast;
    active.search_start_position_m = axis_component(command_position_m_, status_.active_axis);
    debounce_candidate_[status_.active_axis] = false;
    debounce_elapsed_[status_.active_axis] = 0.0;
    update_machine_positions();
    return true;
}

void HomingController::stop() {
    if (!status_.running) {
        return;
    }
    status_.running = false;
    for (auto& axis : status_.axes) {
        if (!axis.homed) {
            axis.phase = HomingPhase::Stopped;
        }
    }
}

void HomingController::emergency_stop() {
    status_.running = false;
    status_.emergency_stopped = true;
    status_.completed = false;
    for (auto& axis : status_.axes) {
        if (!axis.homed) {
            axis.phase = HomingPhase::EmergencyStopped;
        }
    }
}

bool HomingController::clear_stop() {
    if (status_.fault) return false;
    if (status_.emergency_stopped) {
        // 显式解除急停锁存；保留动力学中的当前位置和速度，下一步仍必须重新回零。
        status_.emergency_stopped = false;
        status_.running = false;
        status_.completed = false;
        for (auto& axis : status_.axes) {
            if (!axis.homed) axis.phase = HomingPhase::Idle;
            axis.message.clear();
        }
        return true;
    }
    bool was_stopped = false;
    for (auto& axis : status_.axes) {
        if (axis.phase == HomingPhase::Stopped || axis.phase == HomingPhase::EmergencyStopped) {
            was_stopped = true;
            axis.phase = HomingPhase::Idle;
            axis.message.clear();
        }
    }
    if (was_stopped) {
        status_.running = false;
        status_.completed = false;
    }
    return was_stopped;
}

bool HomingController::clear_fault() {
    if (!status_.fault) return false;
    status_.fault = false;
    status_.fault_message.clear();
    status_.running = false;
    status_.completed = false;
    for (auto& axis : status_.axes) {
        if (!axis.homed) axis.phase = HomingPhase::Idle;
        axis.message.clear();
    }
    return true;
}

double HomingController::axis_component(const multiaxis::Vector3& value, std::size_t axis) {
    if (axis == 0U) return value.x;
    if (axis == 1U) return value.y;
    return value.z;
}

void HomingController::set_axis_component(multiaxis::Vector3& value,
                                           std::size_t axis,
                                           double x) {
    if (axis == 0U) value.x = x;
    else if (axis == 1U) value.y = x;
    else value.z = x;
}

bool HomingController::update_sensor(std::size_t axis, double dt_seconds) {
    const auto& cfg = config_.axes[axis];
    const double physical_position =
        axis_component(servo_.state().actual_position_m, axis);
    const double trigger = cfg.sensor.trigger_position_m;
    const int direction = sign_of(cfg.sensor.direction);
    // 开关在搜索方向一侧触发，反向离开触发点后恢复无效。
    const bool geometric_active = direction < 0 ? physical_position <= trigger
                                                 : physical_position >= trigger;
    const bool active = cfg.sensor.simulate_stuck_active
        ? true
        : (cfg.sensor.simulate_stuck_inactive ? false : geometric_active);
    const bool level = cfg.sensor.active_high ? active : !active;
    auto& axis_status = status_.axes[axis];
    axis_status.sensor_level = level;
    if (active != debounce_candidate_[axis]) {
        debounce_candidate_[axis] = active;
        debounce_elapsed_[axis] = 0.0;
    } else {
        debounce_elapsed_[axis] += dt_seconds;
    }
    if (debounce_elapsed_[axis] + kAxisEpsilon >= cfg.sensor.debounce_time_s) {
        axis_status.sensor_active = debounce_candidate_[axis];
    }
    return axis_status.sensor_active;
}

void HomingController::set_fault(std::size_t axis, const std::string& message) {
    status_.running = false;
    status_.fault = true;
    status_.completed = false;
    status_.fault_message = message;
    if (axis < 3U) {
        status_.axes[axis].phase = HomingPhase::Fault;
        status_.axes[axis].message = message;
    }
}

bool HomingController::reached_position(std::size_t axis, double target) const {
    const auto& cfg = config_.axes[axis];
    const double position = axis_component(servo_.state().actual_position_m, axis);
    const double velocity = axis_component(servo_.state().actual_velocity_mps, axis);
    return std::abs(position - target) <= cfg.position_tolerance_m
        && std::abs(velocity) <= cfg.position_tolerance_m;
}

bool HomingController::advance_axis(std::size_t axis, double dt_seconds) {
    auto& axis_status = status_.axes[axis];
    const auto& cfg = config_.axes[axis];
    axis_status.elapsed_s += dt_seconds;
    const double position = axis_component(servo_.state().actual_position_m, axis);
    if (axis_status.elapsed_s > cfg.timeout_s) {
        set_fault(axis, "homing timeout");
        return false;
    }
    if (std::abs(position - axis_status.search_start_position_m) > cfg.search_range_m) {
        set_fault(axis, axis_status.phase == HomingPhase::Backoff
            ? "home sensor did not release during backoff"
            : "homing search range exceeded");
        return false;
    }

    const int direction = sign_of(cfg.sensor.direction);
    double speed = 0.0;
    if (axis_status.phase == HomingPhase::SearchFast) {
        speed = static_cast<double>(direction) * cfg.fast_speed_mps;
        if (update_sensor(axis, dt_seconds)) {
            axis_status.trigger_position_m = position;
            backoff_target_m_[axis] = position - static_cast<double>(direction)
                * cfg.backoff_distance_m;
            // 重新设置参考目标，避免搜索阶段累积的前向目标把回退指令抵消。
            set_axis_component(command_position_m_, axis, backoff_target_m_[axis]);
            axis_status.phase = HomingPhase::Backoff;
            debounce_candidate_[axis] = true;
            debounce_elapsed_[axis] = 0.0;
            return true;
        }
    } else if (axis_status.phase == HomingPhase::Backoff) {
        const bool active = update_sensor(axis, dt_seconds);
        const double velocity = axis_component(servo_.state().actual_velocity_mps, axis);
        const bool crossed_target = direction < 0
            ? position >= backoff_target_m_[axis]
            : position <= backoff_target_m_[axis];
        if (!active && crossed_target) {
            // 回退目标已到达后先给控制器一个零速参考，等待真实动力学速度降到
            // 很小的范围，再开始慢速二次搜索。不能把实际速度直接写成零。
            set_axis_component(command_position_m_, axis, backoff_target_m_[axis]);
            if (std::abs(velocity) <= cfg.position_tolerance_m) {
                axis_status.phase = HomingPhase::SearchSlow;
                axis_status.elapsed_s = 0.0;
                axis_status.search_start_position_m = position;
                debounce_candidate_[axis] = false;
                debounce_elapsed_[axis] = 0.0;
            }
            return true;
        }
        speed = -static_cast<double>(direction) * cfg.fast_speed_mps;
    } else if (axis_status.phase == HomingPhase::SearchSlow) {
        speed = static_cast<double>(direction) * cfg.slow_speed_mps;
        if (update_sensor(axis, dt_seconds)) {
            axis_status.trigger_position_m = position;
            axis_status.phase = HomingPhase::ApplyOffset;
            return true;
        }
    } else if (axis_status.phase == HomingPhase::ApplyOffset) {
        // 不修改物理位置，只建立一个从传感器触发点到机床坐标的偏移。
        set_axis_component(machine_offsets_m_, axis,
                           cfg.offset_m - axis_status.trigger_position_m);
        axis_status.homed = true;
        axis_status.phase = HomingPhase::Homed;
        axis_status.message = "home complete";
        return true;
    } else {
        return true;
    }

    set_axis_component(command_position_m_, axis,
                       axis_component(command_position_m_, axis) + speed * dt_seconds);
    return true;
}

bool HomingController::step(double dt_seconds) {
    if (!std::isfinite(dt_seconds) || dt_seconds <= 0.0) {
        throw std::invalid_argument("homing dt must be finite and positive");
    }
    if (!status_.running) {
        update_machine_positions();
        return !status_.fault && !status_.emergency_stopped;
    }
    const std::size_t axis = status_.active_axis;
    if (!advance_axis(axis, dt_seconds)) {
        update_machine_positions();
        return false;
    }

    // 当前轴的 command_position 已更新；非活动轴保持当前位置目标。
    const auto physical = servo_.state().actual_position_m;
    for (std::size_t index = 0; index < 3U; ++index) {
        if (index != axis && !status_.axes[index].homed) {
            set_axis_component(command_position_m_, index, axis_component(physical, index));
        }
    }
    multiaxis::MultiAxisTrajectoryState reference{};
    reference.position_m = command_position_m_;
    reference.position = command_position_m_;
    reference.velocity_mps = {};
    reference.velocity = {};
    const auto& phase = status_.axes[axis].phase;
    if (phase == HomingPhase::SearchFast || phase == HomingPhase::SearchSlow
        || phase == HomingPhase::Backoff) {
        const int direction = sign_of(config_.axes[axis].sensor.direction);
        const double speed = phase == HomingPhase::SearchSlow
            ? config_.axes[axis].slow_speed_mps
            : config_.axes[axis].fast_speed_mps;
        const double backoff_position = axis_component(
            servo_.state().actual_position_m, axis);
        const bool backoff_target_reached = phase == HomingPhase::Backoff
            && !status_.axes[axis].sensor_active
            && (direction < 0 ? backoff_position >= backoff_target_m_[axis]
                              : backoff_position <= backoff_target_m_[axis]);
        const double signed_speed = backoff_target_reached
            ? 0.0
            : (phase == HomingPhase::Backoff
                ? -static_cast<double>(direction) * speed
                : static_cast<double>(direction) * speed);
        set_axis_component(reference.velocity_mps, axis, signed_speed);
        set_axis_component(reference.velocity, axis, signed_speed);
    }
    servo_.step(reference, dt_seconds);
    status_.time_s += dt_seconds;

    // ApplyOffset intentionally occupies one full sample. The next call to
    // step() applies the coordinate offset, making this event visible to logs
    // and tests instead of collapsing it into the preceding sensor sample.
    if (status_.axes[axis].homed) {
        bool all_homed = true;
        for (const auto& item : status_.axes) all_homed = all_homed && item.homed;
        if (all_homed) {
            // 传感器触发本身不是“瞬时停止”命令。保持零速参考，等
            // 动力学模型的真实速度衰减后再报告回零完成。
            if (servo_.state().actual_velocity_mps.norm() <= 1e-4) {
                completion_settle_elapsed_s_ += dt_seconds;
            } else {
                completion_settle_elapsed_s_ = 0.0;
            }
            if (completion_settle_elapsed_s_ >= 0.05) {
                status_.running = false;
                status_.completed = true;
            }
        } else {
            std::size_t next = 3U;
            for (const std::size_t candidate : config_.order) {
                if (!status_.axes[candidate].homed) { next = candidate; break; }
            }
            if (next == 3U) {
                status_.running = false;
                status_.completed = true;
            } else {
                status_.active_axis = next;
                auto& next_status = status_.axes[next];
                next_status.phase = HomingPhase::SearchFast;
                next_status.elapsed_s = 0.0;
                next_status.search_start_position_m = axis_component(
                    servo_.state().actual_position_m, next);
                debounce_candidate_[next] = false;
                debounce_elapsed_[next] = 0.0;
            }
        }
    }
    update_machine_positions();
    return !status_.fault && !status_.emergency_stopped;
}

void HomingController::update_machine_positions() {
    const auto& physical = servo_.state().actual_position_m;
    const auto set_status_position = [&](std::size_t axis, double value) {
        status_.axes[axis].machine_position_m = value;
    };
    set_status_position(0U, physical.x + machine_offsets_m_.x);
    set_status_position(1U, physical.y + machine_offsets_m_.y);
    set_status_position(2U, physical.z + machine_offsets_m_.z);
}

multiaxis::Vector3 HomingController::physical_position_m() const noexcept {
    return servo_.state().actual_position_m;
}

multiaxis::Vector3 HomingController::machine_position_m() const noexcept {
    return {servo_.state().actual_position_m.x + machine_offsets_m_.x,
            servo_.state().actual_position_m.y + machine_offsets_m_.y,
            servo_.state().actual_position_m.z + machine_offsets_m_.z};
}

}  // namespace motioncore::manager


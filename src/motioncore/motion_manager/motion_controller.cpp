#include "motioncore/motion_manager/motion_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace motioncore::manager {
namespace {

bool finite_positive(double x) { return std::isfinite(x) && x > 0.0; }

multiaxis::ArcDirection arc_direction(ArcCommandDirection direction) {
    return direction == ArcCommandDirection::CW ? multiaxis::ArcDirection::CW
                                                 : multiaxis::ArcDirection::CCW;
}

}  // namespace

MotionController::MotionController(MotionControllerConfig config)
    : config_(std::move(config)),
      queue_(config_.queue_capacity),
      soft_limits_(config_.soft_limits),
      homing_(config_.homing, config_.servo_configs),
      servo_(config_.servo_configs) {
    for (std::size_t axis = 0; axis < 3; ++axis) {
        const auto& limit = soft_limits_.limits()[axis];
        if (!std::isfinite(limit.max_braking_deceleration_mps2) ||
            limit.max_braking_deceleration_mps2 <= 0.0 ||
            limit.min_position_m > limit.max_position_m) {
            (void)soft_limits_.set_axis_limit(axis, AxisSoftLimit{-1.0, 1.0, 1.0, 1.0, 1.0, 0.0});
        }
    }
    if (!finite_positive(config_.settle_time_s) ||
        !finite_positive(config_.position_tolerance_m) ||
        !finite_positive(config_.velocity_tolerance_mps) ||
        !finite_positive(config_.following_error_limit_m) ||
        !finite_positive(config_.following_error_hold_s) ||
        !finite_positive(config_.motion_timeout_factor)) {
        throw std::invalid_argument("invalid motion controller configuration");
    }
    status_.state = MotionState::Disabled;
    update_status_from_servo();
}

void MotionController::log_event(const std::string& event, std::uint64_t id,
                                 FaultCode code, const std::string& message) {
    events_.push_back({status_.time_s, event, state_machine_.state(), id, code, message});
}

bool MotionController::enable() {
    const auto result = state_machine_.dispatch(MotionEvent::Enable);
    if (!result.accepted) return false;
    status_.state = state_machine_.state();
    log_event("enable", 0, FaultCode::None, result.message);
    return true;
}

bool MotionController::disable() {
    const auto result = state_machine_.dispatch(MotionEvent::Disable);
    if (!result.accepted) return false;
    if (has_active_command_) {
        queue_.cancel(active_command_.id, "controller disabled");
        log_event("command_canceled", active_command_.id, FaultCode::None,
                  "controller disabled");
    }
    active_profile_.reset();
    has_active_command_ = false;
    queue_.cancel_queued("controller disabled");
    status_.state = state_machine_.state();
    log_event("disable");
    return true;
}

bool MotionController::home() {
    if (state_machine_.state() != MotionState::NotHomed) return false;
    if (!state_machine_.dispatch(MotionEvent::StartHoming).accepted) return false;
    if (!homing_.start()) return false;
    status_.state = state_machine_.state();
    log_event("homing_start");
    return true;
}

QueueResult MotionController::enqueue_move_line(
    const multiaxis::Vector3& start_m, const multiaxis::Vector3& target_m,
    const planning::MotionConstraints& constraints, multiaxis::PlanningMode mode) {
    QueueResult rejected{};
    rejected.error = QueueError::InvalidCommand;
    if (state_machine_.state() == MotionState::Fault ||
        state_machine_.state() == MotionState::EmergencyStopped ||
        !state_machine_.homed()) {
        rejected.message = "motion requires enabled and homed controller";
        return rejected;
    }
    // 当已有运动正在执行或等待时，调用方传入的 start_m 往往是发令
    // 瞬间的实际位置，而不是前一条路径的终点。把起点解析到队列尾端，
    // 让核心控制器负责绝对坐标指令的几何衔接，GUI 无需猜测执行时刻。
    const auto planned_start = queue_.planned_endpoint();
    const auto effective_start = planned_start.value_or(start_m);
    auto check = soft_limits_.check_line(effective_start, target_m);
    if (!check.valid) {
        rejected.message = check.reason;
        faults_.raise_warning(FaultCode::InvalidMotionCommand, status_.time_s,
                              "queue", check.reason);
        return rejected;
    }
    auto result = queue_.enqueue_line(effective_start, target_m, constraints, mode);
    if (!result.accepted) log_event("command_rejected", result.command_id,
                                    FaultCode::InvalidMotionCommand, result.message);
    else log_event("command_queued", result.command_id, FaultCode::None,
                   planned_start ? "line start resolved from planned queue endpoint" : "");
    return result;
}

QueueResult MotionController::enqueue_move_arc(
    const multiaxis::Vector3& start_m, const multiaxis::Vector3& target_m,
    const multiaxis::Vector3& center_m, ArcCommandDirection direction,
    const planning::MotionConstraints& constraints, multiaxis::PlanningMode mode,
    bool full_circle) {
    QueueResult rejected{};
    rejected.error = QueueError::InvalidCommand;
    if (state_machine_.state() == MotionState::Fault ||
        state_machine_.state() == MotionState::EmergencyStopped ||
        !state_machine_.homed()) {
        rejected.message = "motion requires enabled and homed controller";
        return rejected;
    }
    const auto planned_start = queue_.planned_endpoint();
    const auto effective_start = planned_start.value_or(start_m);
    auto check = soft_limits_.check_arc(effective_start, target_m, center_m,
                                        arc_direction(direction), full_circle);
    if (!check.valid) { rejected.message = check.reason; return rejected; }
    auto result = queue_.enqueue_arc(effective_start, target_m, center_m, direction,
                                     constraints, full_circle, mode);
    if (!result.accepted) log_event("command_rejected", result.command_id,
                                    FaultCode::InvalidMotionCommand, result.message);
    else log_event("command_queued", result.command_id, FaultCode::None,
                   planned_start ? "arc start resolved from planned queue endpoint" : "");
    return result;
}

QueueResult MotionController::enqueue_dwell(double seconds) {
    QueueResult rejected{};
    rejected.error = QueueError::InvalidCommand;
    if (state_machine_.state() != MotionState::Ready &&
        state_machine_.state() != MotionState::Running) {
        rejected.message = "dwell requires Ready or Running state";
        return rejected;
    }
    auto result = queue_.enqueue_dwell(seconds);
    if (result.accepted) log_event("command_queued", result.command_id);
    return result;
}

bool MotionController::stop() {
    const auto result = queue_.enqueue_stop();
    if (!result.accepted) return false;
    log_event("stop_requested", result.command_id);
    return true;
}

bool MotionController::emergency_stop() {
    const auto result = queue_.enqueue_emergency_stop();
    if (!result.accepted) return false;
    log_event("emergency_stop_requested", result.command_id);
    return true;
}

bool MotionController::process_control_events() {
    // This is deliberately separate from step(): it gives a paused GUI a
    // zero-time control path for Stop/EmergencyStop without pretending that
    // the mechanism itself stopped or advancing the simulation clock.
    if (const auto next = queue_.peek_next(); next && next->is_high_priority()) {
        auto priority = queue_.pop_next();
        if (priority) (void)handle_priority_event(*priority);
    }
    update_status_from_servo();
    return state_machine_.state() != MotionState::Fault;
}

bool MotionController::reset_fault() {
    if (state_machine_.state() != MotionState::Fault || !faults_.active()) return false;
    // Reset is an acknowledgement, not a way to bypass an active cause.  A
    // soft-limit fault must be physically back inside the configured range;
    // homing faults require the sensor fault to have cleared; numerical faults
    // require a finite snapshot before the latch can be released.
    const auto code = faults_.code();
    if (code == FaultCode::SoftLimitViolation &&
        !soft_limits_.check_position(status_.actual_position_m).valid) return false;
    if ((code == FaultCode::HomingTimeout || code == FaultCode::HomeSensorFault) &&
        homing_.status().fault) return false;
    if (code == FaultCode::NumericalError && !finite_status()) return false;
    // 清除锁存故障前，机械本体必须已经停止到控制器定义的速度阈值内。
    // Fault 分支仍在推进自由滑行，因此不能只检查一个布尔复位输入，
    // 否则复位和重新回零可能与仍在运动的轴同时发生。
    if (!finite_status() || status_.actual_velocity_mps.norm() >
                                config_.velocity_tolerance_mps) return false;
    if (!faults_.reset(true, status_.time_s, "MotionController")) return false;
    if (!state_machine_.dispatch(MotionEvent::ResetFault).accepted) return false;
    (void)homing_.clear_fault();
    status_.fault_code = FaultCode::None;
    status_.fault_active = false;
    status_.state = state_machine_.state();
    log_event("fault_reset");
    return true;
}

bool MotionController::reset_emergency() {
    if (state_machine_.state() != MotionState::EmergencyStopped) return false;
    if (emergency_input_active_ || !finite_status() ||
        status_.actual_velocity_mps.norm() > config_.velocity_tolerance_mps) return false;
    if (!state_machine_.dispatch(MotionEvent::ResetEmergency).accepted) return false;
    if (faults_.active()) (void)faults_.reset(true, status_.time_s, "MotionController");
    (void)homing_.clear_stop();
    status_.fault_code = FaultCode::None;
    status_.fault_active = false;
    status_.state = state_machine_.state();
    log_event("emergency_reset");
    return true;
}

void MotionController::release_emergency_input() {
    // Releasing the simulated input does not reset the latch or start motion.
    emergency_input_active_ = false;
    log_event("emergency_input_released");
}

bool MotionController::build_active_profile(const MotionCommand& command) {
    try {
        if (command.type == MotionCommandType::MoveLine) {
            active_profile_ = std::make_unique<multiaxis::LineInterpolator>(
                command.start_m, command.target_m, command.constraints,
                command.planning_mode);
        } else if (command.type == MotionCommandType::MoveArc) {
            // 取与软件限位相同的合成约束作为圆弧的保守上限，避免多段路径
            // 绕过 Phase 4.1 的曲率检查。
            double resultant_a = std::numeric_limits<double>::infinity();
            double resultant_j = std::numeric_limits<double>::infinity();
            for (const auto& limit : config_.soft_limits) {
                resultant_a = std::min(resultant_a, limit.max_acceleration_mps2);
                resultant_j = std::min(resultant_j, limit.max_acceleration_mps2 * 5.0);
            }
            active_profile_ = std::make_unique<multiaxis::ArcInterpolator>(
                command.start_m, command.target_m, command.center_m,
                arc_direction(command.direction), command.constraints,
                command.planning_mode, command.full_circle,
                multiaxis::ArcResultantLimits{resultant_a, resultant_j});
        } else {
            active_profile_.reset();
        }
    } catch (const std::exception& exception) {
        active_profile_.reset();
        faults_.raise_warning(FaultCode::InvalidMotionCommand, status_.time_s,
                              "planner", exception.what());
        return false;
    }
    return true;
}

bool MotionController::start_next_command() {
    if (has_active_command_ || state_machine_.state() != MotionState::Ready) return false;
    auto next = queue_.pop_next();
    if (!next) return false;
    active_command_ = *next;
    has_active_command_ = true;
    if (active_command_.type == MotionCommandType::MoveLine ||
        active_command_.type == MotionCommandType::MoveArc) {
        const auto actual = status_.actual_position_m;
        if ((actual - active_command_.start_m).norm() > 2e-3 ||
            !build_active_profile(active_command_)) {
            queue_.mark_rejected(active_command_.id, "start point or trajectory is invalid");
            has_active_command_ = false;
            active_profile_.reset();
            log_event("command_rejected", active_command_.id,
                      FaultCode::InvalidMotionCommand, "start point or trajectory is invalid");
            return false;
        }
    } else if (active_command_.type == MotionCommandType::Dwell) {
        if (!std::isfinite(active_command_.dwell_seconds) || active_command_.dwell_seconds < 0.0) {
            queue_.mark_rejected(active_command_.id, "invalid dwell duration");
            has_active_command_ = false;
            return false;
        }
    }
    if (!state_machine_.dispatch(MotionEvent::MotionStarted).accepted) {
        queue_.mark_rejected(active_command_.id, "state changed before execution");
        has_active_command_ = false;
        active_profile_.reset();
        return false;
    }
    queue_.mark_running(active_command_.id, status_.time_s);
    active_command_.status = CommandStatus::Running;
    active_elapsed_s_ = 0.0;
    settle_elapsed_s_ = 0.0;
    following_error_elapsed_s_ = 0.0;
    have_reference_ = false;
    status_.state = state_machine_.state();
    log_event("command_started", active_command_.id);
    return true;
}

void MotionController::update_status_from_servo() {
    const auto& state = servo_.state();
    status_.actual_position_m = state.actual_position_m + machine_offset_m_;
    status_.actual_velocity_mps = state.actual_velocity_mps;
    status_.actual_acceleration_mps2 = state.actual_acceleration_mps2;
    status_.drive_force_n = state.drive_force_n;
    status_.soft_limit_margin_m = soft_limits_.margins(status_.actual_position_m);
    status_.following_error_m =
        (status_.reference_position_m - status_.actual_position_m).norm();
    status_.homed = state_machine_.homed_axes();
    status_.fault_code = faults_.code();
    status_.fault_active = faults_.active();
    status_.state = state_machine_.state();
    if (has_active_command_) {
        status_.command_id = active_command_.id;
        status_.command_status = active_command_.status;
    } else {
        status_.command_id = 0;
        status_.command_status = CommandStatus::Queued;
    }
}

void MotionController::begin_controlled_stop() {
    if (state_machine_.state() != MotionState::Running &&
        state_machine_.state() != MotionState::Homing) return;
    (void)state_machine_.dispatch(MotionEvent::Stop);
    if (has_active_command_) {
        queue_.cancel(active_command_.id, "controlled stop");
        has_active_command_ = false;
        active_profile_.reset();
    }
    homing_.stop();
    // Conservative stop policy: cancel the unexecuted path as well as the
    // current segment.  It must never resume implicitly after braking.
    queue_.cancel_queued("controlled stop");
    settle_elapsed_s_ = 0.0;
    log_event("controlled_stop");
}

bool MotionController::handle_priority_event(const MotionCommand& command) {
    if (command.type == MotionCommandType::EmergencyStop) {
        emergency_input_active_ = true;
        (void)state_machine_.dispatch(MotionEvent::EmergencyStop);
        (void)faults_.raise(FaultCode::EmergencyStopTriggered, status_.time_s,
                            "MotionController", "emergency stop latched");
        queue_.cancel_queued("emergency stop");
        if (has_active_command_) {
            queue_.cancel(active_command_.id, "emergency stop");
            has_active_command_ = false;
            active_profile_.reset();
        }
        homing_.emergency_stop();
        queue_.mark_completed(command.id, status_.time_s, "emergency stop latched");
        log_event("emergency_stop", command.id, FaultCode::EmergencyStopTriggered);
        status_.state = state_machine_.state();
        return true;
    }
    if (command.type == MotionCommandType::Stop) {
        queue_.mark_completed(command.id, status_.time_s, "controlled stop requested");
        begin_controlled_stop();
        return true;
    }
    return false;
}

void MotionController::finish_command(const std::string& message) {
    if (!has_active_command_) return;
    queue_.mark_completed(active_command_.id, status_.time_s, message);
    log_event("command_completed", active_command_.id, FaultCode::None, message);
    has_active_command_ = false;
    active_profile_.reset();
    if (state_machine_.state() == MotionState::Running) {
        (void)state_machine_.dispatch(MotionEvent::MotionComplete);
    }
    status_.state = state_machine_.state();
}

void MotionController::latch_fault(FaultCode code, const std::string& message) {
    faults_.raise(code, status_.time_s, "MotionController", message);
    (void)state_machine_.dispatch(MotionEvent::FaultDetected, message);
    queue_.cancel_queued("fault latched");
    if (has_active_command_) {
        queue_.cancel(active_command_.id, "fault latched");
        has_active_command_ = false;
        active_profile_.reset();
    }
    // latch_fault can be called immediately after a servo update.  Keep the
    // public snapshot synchronized even though no subsequent update cycle may
    // run while the controller is faulted.
    status_.fault_code = faults_.code();
    status_.fault_active = faults_.active();
    status_.state = state_machine_.state();
    log_event("fault", 0, code, message);
}

bool MotionController::monitor_dynamic_limits(double dt_seconds) {
    // 回零尚未建立机床坐标时，搜索范围由 HomingController 单独保护；
    // 这里只监控已经建立坐标系后的普通运动和受控停止。
    if (!state_machine_.homed()) return true;
    const double values[3] = {status_.actual_position_m.x,
                              status_.actual_position_m.y,
                              status_.actual_position_m.z};
    // 5e-6 m (0.005 mm) is below the requested positioning tolerances and
    // filters only numerical ripple at an idle axis boundary.
    constexpr double kPositionNoiseToleranceM = 5e-6;
    std::size_t violating_axis = 3;
    for (std::size_t axis = 0; axis < 3; ++axis) {
        const auto& limit = soft_limits_.limits()[axis];
        if (values[axis] < limit.min_position_m + limit.safety_margin_m -
                              kPositionNoiseToleranceM ||
            values[axis] > limit.max_position_m - limit.safety_margin_m +
                              kPositionNoiseToleranceM) {
            violating_axis = axis;
            break;
        }
    }
    if (violating_axis < 3) {
        const auto& limit = soft_limits_.limits()[violating_axis];
        std::ostringstream message;
        message << "axis " << violating_axis << " actual position "
                << values[violating_axis]
                << " m is outside [" << limit.min_position_m << ", "
                << limit.max_position_m << "] m";
        latch_fault(FaultCode::SoftLimitViolation, message.str());
        return false;
    }

    // 预测制动检查使用实际速度和配置的最大可实现减速度。它在越界前
    // 锁存故障，避免把“已经来不及停下”误报为安全完成。
    const double positions[3] = {status_.actual_position_m.x,
                                 status_.actual_position_m.y,
                                 status_.actual_position_m.z};
    const double velocities[3] = {status_.actual_velocity_mps.x,
                                  status_.actual_velocity_mps.y,
                                  status_.actual_velocity_mps.z};
    for (std::size_t axis = 0; axis < 3; ++axis) {
        if (std::abs(velocities[axis]) <= 1e-9) continue;
        const auto& limit = soft_limits_.limits()[axis];
        const double lower = limit.min_position_m + limit.safety_margin_m;
        const double upper = limit.max_position_m - limit.safety_margin_m;
        const double available = velocities[axis] > 0.0
            ? upper - positions[axis] : positions[axis] - lower;
        const double stop_distance = soft_limits_.stop_distance(
            velocities[axis], axis, dt_seconds);
        // A few nanometres of velocity/position ripple can be produced by the
        // independent axes while they are nominally stationary at a boundary.
        // Leave that numerical noise to the explicit position check; a real
        // predicted violation must exceed the tolerance by a meaningful amount.
        if (available > kPositionNoiseToleranceM &&
            stop_distance > available + kPositionNoiseToleranceM) {
            std::ostringstream message;
            message << "axis " << axis << " cannot stop before software limit: position "
                    << positions[axis] << " m, velocity " << velocities[axis]
                    << " m/s, predicted stop distance " << stop_distance
                    << " m, limits [" << limit.min_position_m << ", "
                    << limit.max_position_m << "] m";
            latch_fault(FaultCode::SoftLimitViolation, message.str());
            return false;
        }
    }
    return true;
}

bool MotionController::finite_status() const noexcept {
    return status_.actual_position_m.is_finite() && status_.actual_velocity_mps.is_finite() &&
           status_.actual_acceleration_mps2.is_finite() && status_.drive_force_n.is_finite();
}

bool MotionController::step(double dt_seconds) {
    if (!std::isfinite(dt_seconds) || dt_seconds <= 0.0) {
        // 已经锁存的 Fault/EmergencyStop 不应因为后续一个坏 dt 覆盖原始
        // 故障码与来源；在其它状态下才把非法时间步升级为 NumericalError。
        if (state_machine_.state() != MotionState::Fault &&
            state_machine_.state() != MotionState::EmergencyStopped) {
            latch_fault(FaultCode::NumericalError, "dt must be finite and positive");
        }
        return false;
    }
    status_.time_s += dt_seconds;

    // 急停/停止优先级通道先于普通 FIFO。
    if (const auto next = queue_.peek_next(); next && next->is_high_priority()) {
        auto priority = queue_.pop_next();
        if (priority) handle_priority_event(*priority);
    }

    if (state_machine_.state() == MotionState::Homing) {
        homing_.step(dt_seconds);
        servo_.adopt_physical_state(homing_.servo());
        status_.actual_position_m = homing_.machine_position_m();
        status_.actual_velocity_mps = servo_.state().actual_velocity_mps;
        status_.actual_acceleration_mps2 = servo_.state().actual_acceleration_mps2;
        status_.drive_force_n = servo_.state().drive_force_n;
        if (homing_.status().fault) {
            const auto& text = homing_.status().fault_message;
            const auto code = text.find("sensor") != std::string::npos
                ? FaultCode::HomeSensorFault : FaultCode::HomingTimeout;
            latch_fault(code, text);
        } else if (homing_.status().completed) {
            for (std::size_t i = 0; i < 3; ++i) (void)state_machine_.set_axis_homed(i, true);
            (void)state_machine_.dispatch(MotionEvent::HomingComplete);
            machine_offset_m_ = homing_.machine_offset_m();
            status_.reference_position_m = homing_.machine_position_m();
            status_.reference_velocity_mps = {};
            status_.following_error_m = 0.0;
            log_event("homing_complete");
        }
        update_status_from_servo();
        // machine coordinates include homing offsets; keep reference in the same frame.
        status_.actual_position_m = homing_.machine_position_m();
        status_.homed = state_machine_.homed_axes();
        if (homing_.status().completed && !has_active_command_) {
            status_.reference_position_m = status_.actual_position_m;
            status_.reference_velocity_mps = {};
            status_.following_error_m = 0.0;
        }
        return !faults_.active();
    }

    if (state_machine_.state() == MotionState::EmergencyStopped) {
        // 驱动禁能的仿真仍让机械轴按当前速度和阻尼滑行，绝不瞬时清零位置/速度。
        servo_.step_drive_disabled(dt_seconds);
        update_status_from_servo();
        return false;
    }

    if (state_machine_.state() == MotionState::Fault) {
        // 故障锁存只禁止新的控制指令，不冻结机械本体。驱动器被视为
        // 禁能，DynamicAxisModel 仍按 F_drive=0、阻尼和外负载推进，
        // 因而故障时的惯性滑行、限位裕量和最终停距都可以被真实记录。
        // 这里不能调用 monitor_dynamic_limits：它的职责是发现并锁存新
        // 故障，而当前 Fault 已经锁存，重复 raise 会覆盖原始故障上下文。
        servo_.step_drive_disabled(dt_seconds);
        update_status_from_servo();
        status_.state = MotionState::Fault;
        status_.fault_code = faults_.code();
        status_.fault_active = faults_.active();
        if (!finite_status()) {
            // 保留既有故障锁存，只更新公开快照，绝不把 NaN 静默写成零。
            status_.fault_active = true;
            status_.fault_code = faults_.code() == FaultCode::None
                ? FaultCode::NumericalError : faults_.code();
        }
        return false;
    }

    if (state_machine_.state() == MotionState::ControlledStopping) {
        multiaxis::MultiAxisTrajectoryState reference{};
        reference.position_m = servo_.state().actual_position_m;
        reference.position = reference.position_m;
        reference.velocity_mps = {};
        reference.velocity = {};
        servo_.step(reference, dt_seconds);
        update_status_from_servo();
        if (!monitor_dynamic_limits(dt_seconds)) return false;
        if (status_.actual_velocity_mps.norm() <= config_.velocity_tolerance_mps) {
            settle_elapsed_s_ += dt_seconds;
            if (settle_elapsed_s_ >= config_.settle_time_s) {
                (void)state_machine_.dispatch(MotionEvent::ControlledStopComplete);
                status_.reference_position_m = status_.actual_position_m;
                status_.reference_velocity_mps = {};
                status_.following_error_m = 0.0;
                status_.state = state_machine_.state();
                log_event("controlled_stop_complete");
            }
        } else settle_elapsed_s_ = 0.0;
        return true;
    }

    if (state_machine_.state() == MotionState::Ready) start_next_command();
    if (state_machine_.state() != MotionState::Running || !has_active_command_) {
        if (state_machine_.state() == MotionState::Disabled ||
            state_machine_.state() == MotionState::NotHomed) {
            // Disabling a drive cannot remove the mechanism's kinetic energy.
            servo_.step_drive_disabled(dt_seconds);
        } else if (state_machine_.state() == MotionState::Ready) {
            // Ready is a position hold, not a frozen final sample.  Residual
            // velocity is allowed to decay under the same physical servo.
            multiaxis::MultiAxisTrajectoryState hold{};
            hold.position_m = status_.reference_position_m - machine_offset_m_;
            hold.position = hold.position_m;
            servo_.step(hold, dt_seconds);
        }
        update_status_from_servo();
        if (state_machine_.state() == MotionState::Ready &&
            !monitor_dynamic_limits(dt_seconds)) return false;
        return !faults_.active();
    }

    active_elapsed_s_ += dt_seconds;
    multiaxis::MultiAxisTrajectoryState reference{};
    if (active_command_.type == MotionCommandType::Dwell) {
        // Dwell has no geometric target, but the reference still uses machine
        // coordinates like every other command.  Re-add the homing offset
        // before the common conversion below; using physical coordinates here
        // would make the servo chase the offset during a dwell.
        reference.position_m = servo_.state().actual_position_m + machine_offset_m_;
        reference.position = reference.position_m;
        reference.velocity_mps = {};
        reference.velocity = {};
    } else {
        reference = active_profile_->sample(active_elapsed_s_);
    }
    const auto machine_reference = reference;
    last_reference_ = machine_reference;
    have_reference_ = true;
    status_.reference_position_m = machine_reference.position_m;
    status_.reference_velocity_mps = machine_reference.velocity_mps;
    status_.reference_acceleration_mps2 = machine_reference.acceleration_mps2;
    status_.reference_jerk_mps3 = machine_reference.jerk_mps3;
    // 规划器在机床坐标系中工作；动力学轴保留传感器触发时的物理坐标，
    // 因此送入伺服前只减去回零建立的固定坐标偏移。
    reference.position_m -= machine_offset_m_;
    reference.position = reference.position_m;
    servo_.step(reference, dt_seconds);
    update_status_from_servo();
    status_.reference_position_m = machine_reference.position_m;
    status_.reference_velocity_mps = machine_reference.velocity_mps;
    status_.following_error_m = (machine_reference.position_m - status_.actual_position_m).norm();

    if (!finite_status()) {
        latch_fault(FaultCode::NumericalError, "non-finite servo state");
        return false;
    }
    if (!monitor_dynamic_limits(dt_seconds)) return false;
    if (status_.following_error_m > config_.following_error_limit_m) {
        following_error_elapsed_s_ += dt_seconds;
        if (following_error_elapsed_s_ >= config_.following_error_hold_s) {
            latch_fault(FaultCode::FollowingError, "following error persisted");
            return false;
        }
    } else following_error_elapsed_s_ = 0.0;

    const double duration = active_command_.type == MotionCommandType::Dwell
        ? active_command_.dwell_seconds : active_profile_->duration();
    if (active_elapsed_s_ >= duration) {
        const bool settled = status_.following_error_m <= config_.position_tolerance_m &&
                             status_.actual_velocity_mps.norm() <= config_.velocity_tolerance_mps;
        if (settled) settle_elapsed_s_ += dt_seconds;
        else settle_elapsed_s_ = 0.0;
        if (settle_elapsed_s_ >= config_.settle_time_s) finish_command("motion settled");
        if (active_elapsed_s_ > duration * config_.motion_timeout_factor + config_.settle_time_s) {
            latch_fault(FaultCode::MotionTimeout, "motion did not settle before timeout");
            return false;
        }
    }
    return true;
}

bool MotionController::inject_fault(FaultCode code, const std::string& message) {
    if (state_machine_.state() == MotionState::Disabled) return false;
    latch_fault(code, message);
    return true;
}

}  // namespace motioncore::manager

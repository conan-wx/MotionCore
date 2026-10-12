#include "motioncore/motion_manager/motion_controller.hpp"

#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>

using motioncore::manager::FaultCode;
using motioncore::manager::MotionController;
using motioncore::manager::MotionControllerConfig;
using motioncore::manager::MotionState;
using motioncore::manager::CommandStatus;
using motioncore::multiaxis::Vector3;
using motioncore::planning::MotionConstraints;

namespace {

void require(bool value, const std::string& message) {
    if (!value) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}

MotionControllerConfig config() {
    MotionControllerConfig c;
    c.queue_capacity = 16;
    c.servo_configs = motioncore::multiaxis::MultiAxisServoSystem::default_configs();
    c.servo_configs[0].dynamics.initial_position_m = .006;
    c.servo_configs[1].dynamics.initial_position_m = .005;
    c.servo_configs[2].dynamics.initial_position_m = .004;
    for (auto& h : c.homing.axes) {
        h.sensor.trigger_position_m = 0.0;
        h.sensor.direction = motioncore::manager::HomeDirection::Negative;
        h.fast_speed_mps = .015;
        h.slow_speed_mps = .004;
        h.backoff_distance_m = .002;
        h.search_range_m = .05;
        h.timeout_s = 10.0;
        h.sensor.debounce_time_s = .002;
    }
    for (auto& limit : c.soft_limits) {
        limit.min_position_m = 0.0;
        limit.max_position_m = .1;
        limit.max_velocity_mps = .1;
        limit.max_acceleration_mps2 = .8;
        limit.max_braking_deceleration_mps2 = .5;
    }
    c.following_error_limit_m = .05;
    c.following_error_hold_s = .02;
    c.motion_timeout_factor = 6.0;
    return c;
}

void home(MotionController& controller) {
    require(controller.enable(), "enable before homing");
    require(controller.home(), "homing command accepted");
    for (int i = 0; i < 30000 && controller.state() == MotionState::Homing; ++i) {
        controller.step(.001);
    }
    require(controller.state() == MotionState::Ready, "homing reaches Ready");
}

void run_until_ready(MotionController& controller, int max_steps = 80000) {
    for (int i = 0; i < max_steps; ++i) {
        controller.step(.001);
        if (controller.state() == MotionState::Ready &&
            controller.status().command_id == 0 && controller.queue().empty()) return;
    }
    require(false, "queued trajectory did not finish within deterministic timeout");
}

void fault_preserves_inertia() {
    MotionController controller(config());
    home(controller);
    const MotionConstraints constraints{.05, .1, .5};
    const auto move = controller.enqueue_move_line({}, {.06, 0.0, 0.0}, constraints);
    require(move.accepted, "fault test motion accepted");

    // Wait until the real dynamic model has non-zero velocity.  The test then
    // injects a latched fault without touching the axis state.
    bool moving = false;
    for (int i = 0; i < 5000; ++i) {
        controller.step(.001);
        if (controller.state() == MotionState::Running &&
            std::abs(controller.status().actual_velocity_mps.x) > 1e-3) {
            moving = true;
            break;
        }
    }
    require(moving, "real axis reached non-zero velocity before fault");
    const auto before = controller.status();
    require(controller.inject_fault(FaultCode::FollowingError, "regression fault"),
            "fault injection accepted");
    require(controller.state() == MotionState::Fault, "fault is latched");
    require(!controller.reset_fault(),
            "fault reset is rejected while the inertial axis is still moving");
    require(!controller.step(std::numeric_limits<double>::quiet_NaN()) &&
            controller.status().fault_code == FaultCode::FollowingError,
            "invalid dt cannot overwrite the original latched fault");

    for (int i = 0; i < 1000; ++i) {
        require(!controller.step(.001), "fault remains latched while physical axis evolves");
    }
    const auto after = controller.status();
    require(after.time_s >= before.time_s + .999,
            "controller clock advances while fault is latched");
    require(std::abs(after.actual_position_m.x - before.actual_position_m.x) > 1e-6,
            "faulted axis position continues to move by inertia");
    require(std::abs(after.actual_velocity_mps.x - before.actual_velocity_mps.x) > 1e-6,
            "faulted axis velocity follows freewheel dynamics");
    require(after.fault_active && after.fault_code == FaultCode::FollowingError,
            "original fault remains visible after freewheel evolution");
    std::cout << std::setprecision(12)
              << "fault_freewheel before_x_mm=" << before.actual_position_m.x * 1000.0
              << " after_x_mm=" << after.actual_position_m.x * 1000.0
              << " before_v_mm_s=" << before.actual_velocity_mps.x * 1000.0
              << " after_v_mm_s=" << after.actual_velocity_mps.x * 1000.0
              << " elapsed_s=" << (after.time_s - before.time_s) << '\n';
}

void two_lines_rebase_at_core() {
    MotionController controller(config());
    home(controller);
    const MotionConstraints constraints{.05, .1, .5};
    // The second command deliberately supplies the stale GUI-time start (zero).
    // Core queue planning must resolve it to the first target before execution.
    const auto first = controller.enqueue_move_line({}, {.03, 0.0, 0.0}, constraints);
    const auto second = controller.enqueue_move_line({}, {.06, 0.0, 0.0}, constraints);
    require(first.accepted && second.accepted, "two absolute lines accepted");
    run_until_ready(controller);
    const auto first_record = controller.queue().find(first.command_id);
    const auto second_record = controller.queue().find(second.command_id);
    require(first_record && second_record &&
            first_record->status == CommandStatus::Completed &&
            second_record->status == CommandStatus::Completed,
            "two continuous lines complete in FIFO order");
    require(std::abs(controller.status().actual_position_m.x - .06) < .002,
            "two-line endpoint reaches second absolute target");
    std::cout << std::setprecision(12)
              << "two_line final_x_mm=" << controller.status().actual_position_m.x * 1000.0
              << " first_status=" << motioncore::manager::to_string(first_record->status)
              << " second_status=" << motioncore::manager::to_string(second_record->status)
              << '\n';
}

void line_arc_line_rebase_at_core() {
    MotionController controller(config());
    home(controller);
    const MotionConstraints constraints{.05, .1, .5};
    // All starts after the first command are intentionally stale.  The arc's
    // center and target still define a valid circle once its queued start is
    // resolved to the prior segment endpoint.
    const auto line_a = controller.enqueue_move_line({}, {.05, .03, 0.0}, constraints);
    const auto arc = controller.enqueue_move_arc(
        {}, {.03, .05, 0.0}, {.03, .03, 0.0},
        motioncore::manager::ArcCommandDirection::CCW, constraints);
    const auto line_b = controller.enqueue_move_line({}, {.01, .01, 0.0}, constraints);
    require(line_a.accepted && arc.accepted && line_b.accepted,
            "line-arc-line queue accepts all commands");
    run_until_ready(controller, 120000);
    for (const auto id : {line_a.command_id, arc.command_id, line_b.command_id}) {
        const auto record = controller.queue().find(id);
        require(record && record->status == CommandStatus::Completed,
                "line-arc-line command completed once");
    }
    const auto final = controller.status().actual_position_m;
    require((final - Vector3{.01, .01, 0.0}).norm() < .003,
            "line-arc-line reaches final target");
    std::cout << std::setprecision(12)
              << "line_arc_line final_mm=" << final.x * 1000.0 << ','
              << final.y * 1000.0 << ',' << final.z * 1000.0 << '\n';
}

}  // namespace

int main() {
    fault_preserves_inertia();
    two_lines_rebase_at_core();
    line_arc_line_rebase_at_core();
    std::cout << "phase5 final core regression tests passed\n";
    return 0;
}

#include "motioncore/motion_manager/motion_controller.hpp"

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

using namespace motioncore;
using namespace motioncore::manager;
using motioncore::multiaxis::Vector3;
using motioncore::planning::MotionConstraints;

namespace {

void require(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}

MotionControllerConfig test_config() {
    MotionControllerConfig config;
    config.queue_capacity = 16;
    config.servo_configs = multiaxis::MultiAxisServoSystem::default_configs();
    config.servo_configs[0].dynamics.initial_position_m = .006;
    config.servo_configs[1].dynamics.initial_position_m = .005;
    config.servo_configs[2].dynamics.initial_position_m = .004;
    for (auto& homing : config.homing.axes) {
        homing.sensor.trigger_position_m = 0.0;
        homing.sensor.direction = HomeDirection::Negative;
        homing.fast_speed_mps = .015;
        homing.slow_speed_mps = .004;
        homing.backoff_distance_m = .002;
        homing.search_range_m = .05;
        homing.timeout_s = 10.0;
        homing.sensor.debounce_time_s = .002;
    }
    for (auto& limit : config.soft_limits) {
        limit.min_position_m = 0.0;
        limit.max_position_m = .1;
        limit.max_velocity_mps = .1;
        limit.max_acceleration_mps2 = .8;
        limit.max_braking_deceleration_mps2 = .5;
    }
    config.following_error_limit_m = .05;
    config.following_error_hold_s = .02;
    config.motion_timeout_factor = 6.0;
    return config;
}

void home(MotionController& controller) {
    if (controller.state() == MotionState::Disabled) {
        require(controller.enable(), "enable controller");
    }
    require(controller.home(), "start homing");
    for (int i = 0; i < 25000 && controller.state() == MotionState::Homing; ++i) {
        require(controller.step(.001), "homing step");
    }
    require(controller.state() == MotionState::Ready, "homing reaches Ready");
}

void queue_terminal_states_are_not_runnable() {
    MotionCommandQueue queue(8);
    const MotionConstraints limits{.05, .1, .5};
    const auto first = queue.enqueue_line({}, {.04, 0.0, 0.0}, limits);
    require(first.accepted, "first command queued");
    require(queue.cancel(first.command_id, "test cancellation"), "queued command canceled");
    const auto record = queue.find(first.command_id);
    require(record && record->status == CommandStatus::Canceled,
            "canceled command remains auditable");
    require(!queue.mark_running(first.command_id, 0.0),
            "canceled command cannot become running");
    require(!queue.mark_completed(first.command_id, 0.0),
            "canceled command cannot become completed");
    require(!queue.pop_next(), "canceled command is never returned for execution");

    const auto stop = queue.enqueue_stop();
    require(stop.accepted, "stop command queued");
    auto popped_stop = queue.pop_next();
    require(popped_stop && popped_stop->id == stop.command_id,
            "priority stop can be popped");
    require(queue.mark_completed(stop.command_id, .001),
            "priority stop can complete without normal motion phase");
    require(!queue.mark_completed(stop.command_id, .002),
            "completed priority event cannot complete twice");
}

void emergency_reset_does_not_restart_canceled_commands() {
    MotionController controller(test_config());
    home(controller);

    const auto first = controller.enqueue_move_line({}, {.08, 0.0, 0.0},
                                                     {.05, .1, .5});
    const auto second = controller.enqueue_move_line({.08, 0.0, 0.0}, {.02, .02, 0.0},
                                                      {.05, .1, .5});
    require(first.accepted && second.accepted, "two normal commands queued");

    // Let the first command start before injecting the higher-priority event.
    for (int i = 0; i < 250; ++i) require(controller.step(.001), "pre-emergency step");
    require(controller.state() == MotionState::Running, "first command is running");
    require(controller.emergency_stop(), "emergency stop accepted");
    require(!controller.step(.001), "emergency stop latches controller");
    require(controller.state() == MotionState::EmergencyStopped,
            "controller enters EmergencyStopped");

    auto canceled = controller.queue().find(second.command_id);
    require(canceled && canceled->status == CommandStatus::Canceled,
            "queued command is canceled by emergency stop");

    require(!controller.reset_emergency(), "active emergency input blocks reset");
    controller.release_emergency_input();
    require(!controller.reset_emergency(), "moving mechanics block emergency reset");
    for (int i = 0; i < 5000 && controller.status().actual_velocity_mps.norm() >
                                  controller.config().velocity_tolerance_mps; ++i)
        (void)controller.step(.001);
    require(controller.reset_emergency(), "released and stationary emergency can reset explicitly");
    require(controller.state() == MotionState::NotHomed,
            "reset requires homing before motion");
    home(controller);
    require(controller.state() == MotionState::Ready, "rehome reaches Ready");

    // If pop_next ignored status, this step would transition to Running and
    // start the old second command. It must remain canceled forever.
    for (int i = 0; i < 1000; ++i) {
        require(controller.step(.001), "post-rehome idle step");
        require(controller.state() != MotionState::Running,
                "canceled command is not restarted after rehome");
    }
    canceled = controller.queue().find(second.command_id);
    require(canceled && canceled->status == CommandStatus::Canceled,
            "canceled command remains terminal after reset and rehome");
}

}  // namespace

int main() {
    queue_terminal_states_are_not_runnable();
    emergency_reset_does_not_restart_canceled_commands();
    std::cout << "phase5.1 queue lifecycle regression passed\n";
    return 0;
}


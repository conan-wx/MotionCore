#include "motioncore/motion_manager/motion_controller.hpp"

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

using namespace motioncore;
using namespace motioncore::manager;

namespace {

void require(bool value, const std::string& message) {
    if (!value) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}

MotionControllerConfig make_config(bool reverse) {
    MotionControllerConfig config;
    config.queue_capacity = 8;
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
        limit.safety_margin_m = 0.0;
    }
    if (reverse) {
        config.soft_limits[0].min_position_m = -.1;
        // Homing leaves a small, physically simulated positive residual.  A
        // 1 mm upper allowance keeps that real sensor/backoff residual inside
        // the test range while the negative target still exercises the lower
        // boundary.
        config.soft_limits[0].max_position_m = .001;
    }
    config.following_error_limit_m = 1.0;
    config.following_error_hold_s = .1;
    config.motion_timeout_factor = 20.0;
    return config;
}

void home(MotionController& controller) {
    require(controller.enable(), "enable");
    require(controller.home(), "home");
    for (int i = 0; i < 30000 && controller.state() == MotionState::Homing; ++i) {
        require(controller.step(.001), "homing remains numerically valid");
    }
    require(controller.state() == MotionState::Ready, "homing reaches Ready");
}

void run_case(bool reverse) {
    MotionController controller(make_config(reverse));
    home(controller);
    const auto target = reverse ? multiaxis::Vector3{-.0999, 0.0, 0.0}
                                : multiaxis::Vector3{.0999, 0.0, 0.0};
    const auto result = controller.enqueue_move_line({}, target, {.1, .8, 4.0});
    require(result.accepted, "near-limit trajectory accepted for dynamic check");

    for (int i = 0; i < 30000; ++i) {
        (void)controller.step(.001);
        const auto command = controller.queue().find(result.command_id);
        if (controller.state() == MotionState::Fault ||
            (command && (command->status == CommandStatus::Completed ||
                         command->status == CommandStatus::Canceled ||
                         command->status == CommandStatus::Rejected))) break;
    }
    require(controller.state() == MotionState::Fault,
            reverse ? "negative near-limit motion faults before unsafe completion"
                    : "positive near-limit motion faults before unsafe completion");
    require(controller.status().fault_code == FaultCode::SoftLimitViolation,
            "dynamic limit reports SoftLimitViolation");
    require(controller.status().fault_active, "soft-limit fault is latched");
    require(controller.events().back().message.find("axis 0") != std::string::npos,
            "fault message identifies the offending axis");
}

}  // namespace

int main() {
    run_case(false);
    run_case(true);
    std::cout << "phase5.1 dynamic soft-limit regression passed\n";
    return 0;
}


#include "motioncore/motion_manager/motion_manager.hpp"
#include "motioncore/motion_manager/motion_controller.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

using namespace motioncore;
using manager::MotionController;
using manager::MotionControllerConfig;
using manager::MotionState;
using manager::MotionEvent;
using manager::FaultCode;
using manager::CommandStatus;
using multiaxis::Vector3;
using planning::MotionConstraints;

namespace {

void check(bool condition, const std::string& message) {
    if (!condition) { std::cerr << "FAIL: " << message << '\n'; std::exit(1); }
}

MotionControllerConfig config() {
    MotionControllerConfig c;
    c.queue_capacity = 8;
    c.servo_configs = multiaxis::MultiAxisServoSystem::default_configs();
    c.servo_configs[0].dynamics.initial_position_m = .006;
    c.servo_configs[1].dynamics.initial_position_m = .005;
    c.servo_configs[2].dynamics.initial_position_m = .004;
    for (auto& h : c.homing.axes) {
        h.sensor.trigger_position_m = 0.0;
        h.sensor.direction = manager::HomeDirection::Negative;
        h.fast_speed_mps = .015; h.slow_speed_mps = .004;
        h.backoff_distance_m = .002; h.search_range_m = .05;
        h.timeout_s = 10.0; h.sensor.debounce_time_s = .002; h.offset_m = 0.0;
    }
    for (auto& limit : c.soft_limits) {
        limit.min_position_m = 0.0; limit.max_position_m = .1;
        limit.max_velocity_mps = .1; limit.max_acceleration_mps2 = .8;
        limit.max_braking_deceleration_mps2 = .5; limit.safety_margin_m = 0.0;
    }
    c.following_error_limit_m = .05;
    c.following_error_hold_s = .02;
    c.motion_timeout_factor = 6.0;
    return c;
}

bool home(MotionController& c) {
    check(c.enable(), "enable"); check(c.home(), "home start");
    for (int i = 0; i < 25000 && c.state() == MotionState::Homing; ++i) c.step(.001);
    return c.state() == MotionState::Ready;
}

int state_case() {
    manager::MotionStateMachine sm;
    check(!sm.dispatch(MotionEvent::MotionStarted).accepted, "motion disabled rejected");
    check(sm.enable() && sm.state() == MotionState::NotHomed, "enable -> not homed");
    check(!sm.motion_started(), "motion before home rejected");
    check(sm.start_homing(), "start homing");
    (void)sm.set_axis_homed(0, true); (void)sm.set_axis_homed(1, true); (void)sm.set_axis_homed(2, true);
    check(sm.homing_complete() && sm.state() == MotionState::Ready, "home complete");
    check(sm.motion_started() && sm.state() == MotionState::Running, "motion start");
    check(sm.stop() && sm.state() == MotionState::ControlledStopping, "stop transition");
    check(sm.controlled_stop_complete() && sm.state() == MotionState::Ready, "stop complete");
    check(sm.emergency_stop() && sm.state() == MotionState::EmergencyStopped, "emergency latch");
    check(!sm.motion_started(), "motion blocked by emergency");
    check(sm.dispatch(MotionEvent::ResetEmergency).accepted && sm.state() == MotionState::NotHomed,
          "emergency reset requires rehome");
    return 0;
}

int queue_case() {
    manager::MotionCommandQueue q(4);
    const MotionConstraints c{.1, .2, 1.0};
    auto a = q.enqueue_line({}, {1, 0, 0}, c);
    auto b = q.enqueue_dwell(.1);
    auto e = q.enqueue_emergency_stop();
    check(a.accepted && b.accepted && e.accepted, "queue accepts commands");
    check(q.pop_next()->type == manager::MotionCommandType::EmergencyStop, "priority first");
    check(q.pop_next()->id == a.command_id, "FIFO normal order");
    check(q.mark_running(a.command_id, 1.0), "running lifecycle");
    check(q.mark_completed(a.command_id, 2.0), "complete once");
    check(!q.mark_completed(a.command_id, 3.0), "terminal command cannot complete twice");
    return 0;
}

int homing_case() {
    auto hcfg = config().homing;
    std::array<multiaxis::MultiAxisServoAxisConfig, 3> axes = config().servo_configs;
    manager::HomingController h(hcfg, axes);
    // Standalone HomingController can be started directly; MotionController adds
    // the higher-level Enable/NotHomed state-machine rule.
    check(h.start(), "standalone homing start");
    for (int i = 0; i < 25000 && !h.all_homed() && !h.status().fault; ++i) h.step(.001);
    check(h.all_homed() && !h.status().fault, "homing completes");
    check(h.machine_position_m().norm() < 1e-3, "machine coordinate near zero");
    auto bad = config().homing;
    bad.axes[0].sensor.simulate_stuck_inactive = true;
    manager::HomingController stuck(bad, config().servo_configs);
    check(stuck.start(), "stuck sensor homing start");
    for (int i = 0; i < 20000 && !stuck.status().fault; ++i) stuck.step(.001);
    check(stuck.status().fault, "stuck sensor eventually faults");
    return 0;
}

int soft_limit_case() {
    std::array<manager::AxisSoftLimit, 3> limits{};
    for (auto& l : limits) { l.min_position_m=0; l.max_position_m=.1; l.max_velocity_mps=1;
        l.max_acceleration_mps2=1; l.max_braking_deceleration_mps2=.5; }
    manager::SoftLimitManager m(limits);
    check(m.check_line({0,0,0},{.09,.09,0}).valid, "line inside limit");
    check(!m.check_line({0,0,0},{.11,0,0}).valid, "line outside limit");
    check(!m.check_arc({.05,-.06,0},{.05,.06,0},{.05,0,0},multiaxis::ArcDirection::CCW).valid,
          "arc cardinal extremum limit");
    check(std::abs(m.stop_distance(1.0, 0) - 1.0) < 1e-12, "stop distance formula");
    check(!m.can_stop_before_limit(.09, 1.0, 0), "predicted stop rejects danger");
    return 0;
}

int stop_case() {
    MotionController c(config()); check(home(c), "controller homing for stop");
    check(c.enqueue_move_line({}, {.08,0,0}, {.05,.1,.5}).accepted, "enqueue stop motion");
    for (int i=0; i<8000; ++i) { if (i==500) check(c.stop(), "stop request"); c.step(.001); }
    check(c.state() == MotionState::Ready, "controlled stop returns ready");
    check(c.status().actual_velocity_mps.norm() < .001, "controlled stop velocity is small");
    MotionController e(config()); check(home(e), "controller homing for emergency");
    e.enqueue_move_line({}, {.08,0,0}, {.05,.1,.5}); e.enqueue_move_line({.08,0,0},{.02,.02,0},{.05,.1,.5});
    for (int i=0; i<1000; ++i) { if (i==200) e.emergency_stop(); e.step(.001); }
    check(e.state() == MotionState::EmergencyStopped, "emergency latched");
    check(e.status().fault_code == FaultCode::EmergencyStopTriggered, "emergency fault code");
    check(!e.enqueue_dwell(.1).accepted, "emergency rejects new command");
    return 0;
}

int fault_case() {
    MotionController c(config()); check(c.enable(), "fault enable");
    check(c.inject_fault(FaultCode::NumericalError), "inject fault");
    check(c.state() == MotionState::Fault && c.faults().latched(), "fault latched");
    check(c.reset_fault() && c.state() == MotionState::NotHomed, "explicit fault reset");
    check(!c.enqueue_dwell(.1).accepted, "fault reset requires rehome");
    return 0;
}

int integration_case() {
    MotionController c(config()); check(home(c), "integration homing");
    check(c.enqueue_move_line({}, {.03,.04,.05}, {.05,.1,.5}).accepted, "integration enqueue");
    for (int i=0; i<20000; ++i) {
        c.step(.001);
        if (i > 100 && c.state() == MotionState::Ready && c.status().command_id == 0) break;
    }
    check(c.state() == MotionState::Ready, "integration reaches ready");
    check((c.status().actual_position_m - Vector3{.03,.04,.05}).norm() < .002,
          "three axis endpoint tracking");
    check(c.status().actual_velocity_mps.norm() < .002, "endpoint velocity");
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    const std::string which = argc > 1 ? argv[1] : "all";
    if (which == "state" || which == "all") state_case();
    if (which == "queue" || which == "all") queue_case();
    if (which == "homing" || which == "all") homing_case();
    if (which == "soft" || which == "all") soft_limit_case();
    if (which == "stop" || which == "all") stop_case();
    if (which == "fault" || which == "all") fault_case();
    if (which == "integration" || which == "all") integration_case();
    std::cout << "phase5 " << which << " tests passed\n";
    return 0;
}


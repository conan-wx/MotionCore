#include "motioncore/motion_manager/motion_controller.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

using motioncore::manager::CommandStatus;
using motioncore::manager::FaultCode;
using motioncore::manager::MotionController;
using motioncore::manager::MotionControllerConfig;
using motioncore::manager::MotionState;
using motioncore::manager::ArcCommandDirection;
using motioncore::multiaxis::Vector3;
using motioncore::planning::MotionConstraints;

namespace {

struct Options { std::string scenario{"nominal"}; std::filesystem::path output{"results/phase5"}; };

Options parse(int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg(argv[i]);
        auto value = [&](const char* name) -> std::string {
            if (arg == name && i + 1 < argc) return argv[++i];
            return {};
        };
        if (const auto v = value("--scenario"); !v.empty()) options.scenario = v;
        else if (const auto v = value("--output"); !v.empty()) options.output = v;
        else if (arg == "--help") {
            std::cout << "--scenario homing|nominal|queue|soft-limit|controlled-stop|emergency-stop|fault-recovery\n"
                         "--output DIRECTORY\n";
            std::exit(0);
        }
    }
    return options;
}

MotionControllerConfig make_config() {
    MotionControllerConfig config;
    config.queue_capacity = 16;
    config.servo_configs = motioncore::multiaxis::MultiAxisServoSystem::default_configs();
    config.servo_configs[0].dynamics.initial_position_m = 0.020;
    config.servo_configs[1].dynamics.initial_position_m = 0.015;
    config.servo_configs[2].dynamics.initial_position_m = 0.025;
    for (auto& axis : config.homing.axes) {
        axis.sensor.trigger_position_m = 0.0;
        axis.sensor.direction = motioncore::manager::HomeDirection::Negative;
        axis.sensor.debounce_time_s = 0.003;
        axis.fast_speed_mps = 0.020;
        axis.slow_speed_mps = 0.005;
        axis.backoff_distance_m = 0.003;
        axis.search_range_m = 0.10;
        axis.timeout_s = 15.0;
        axis.offset_m = 0.0;
    }
    config.soft_limits = {};
    for (auto& limit : config.soft_limits) {
        limit.min_position_m = 0.0;
        limit.max_position_m = 0.100;
        limit.max_velocity_mps = 0.100;
        limit.max_acceleration_mps2 = 0.8;
        limit.max_braking_deceleration_mps2 = 0.5;
        // 演示从物理边界 0 mm 起步；预测停止测试单独验证安全余量。
        limit.safety_margin_m = 0.0;
    }
    config.settle_time_s = 0.05;
    config.position_tolerance_m = 2e-4;
    config.velocity_tolerance_mps = 5e-4;
    config.following_error_limit_m = 0.02;
    config.following_error_hold_s = 0.05;
    config.motion_timeout_factor = 5.0;
    return config;
}

void write_header(std::ofstream& csv) {
    csv << "time_s,motion_state,command_id,command_status,x_ref_mm,y_ref_mm,z_ref_mm,"
           "x_actual_mm,y_actual_mm,z_actual_mm,vx_actual_mm_s,vy_actual_mm_s,vz_actual_mm_s,"
           "drive_force_x_n,drive_force_y_n,drive_force_z_n,homed_x,homed_y,homed_z,"
           "following_error_mm,fault_code,fault_active,soft_limit_margin_x_mm,"
           "soft_limit_margin_y_mm,soft_limit_margin_z_mm\n";
}

void write_row(std::ofstream& csv, const MotionController& controller) {
    const auto& s = controller.status();
    csv << std::setprecision(12) << s.time_s << ',' << motioncore::manager::to_string(s.state)
        << ',' << s.command_id << ',' << motioncore::manager::to_string(s.command_status) << ','
        << s.reference_position_m.x * 1000.0 << ',' << s.reference_position_m.y * 1000.0 << ','
        << s.reference_position_m.z * 1000.0 << ',' << s.actual_position_m.x * 1000.0 << ','
        << s.actual_position_m.y * 1000.0 << ',' << s.actual_position_m.z * 1000.0 << ','
        << s.actual_velocity_mps.x * 1000.0 << ',' << s.actual_velocity_mps.y * 1000.0 << ','
        << s.actual_velocity_mps.z * 1000.0 << ',' << s.drive_force_n.x << ',' << s.drive_force_n.y << ','
        << s.drive_force_n.z << ',' << s.homed[0] << ',' << s.homed[1] << ',' << s.homed[2] << ','
        << s.following_error_m * 1000.0 << ',' << motioncore::manager::to_string(s.fault_code) << ','
        << s.fault_active << ',' << s.soft_limit_margin_m.x * 1000.0 << ','
        << s.soft_limit_margin_m.y * 1000.0 << ',' << s.soft_limit_margin_m.z * 1000.0 << '\n';
}

void write_events(const std::filesystem::path& path, const MotionController& controller) {
    std::ofstream out(path);
    out << "time_s,event,state,command_id,fault_code,message\n";
    for (const auto& event : controller.events()) {
        out << std::setprecision(12) << event.time_s << ',' << event.event << ','
            << motioncore::manager::to_string(event.state) << ',' << event.command_id << ','
            << motioncore::manager::to_string(event.fault_code) << ",\"" << event.message << "\"\n";
    }
}

bool run_homing(MotionController& controller, std::ofstream& csv, double max_time = 30.0) {
    if (!controller.enable() || !controller.home()) return false;
    write_row(csv, controller);
    for (double elapsed = 0.0; elapsed < max_time && controller.state() == MotionState::Homing;
         elapsed += 0.001) {
        if (!controller.step(0.001)) break;
        write_row(csv, controller);
    }
    return controller.state() == MotionState::Ready;
}

int run_scenario(const Options& options) {
    std::filesystem::create_directories(options.output);
    const auto csv_path = options.output / (options.scenario + ".csv");
    std::ofstream csv(csv_path);
    write_header(csv);
    MotionController controller(make_config());

    if (options.scenario == "homing") {
        const bool ok = run_homing(controller, csv);
        write_events(options.output / "homing_events.csv", controller);
        const auto p = controller.status().actual_position_m;
        std::cout << "scenario=homing completed=" << ok << " state="
                  << motioncore::manager::to_string(controller.state()) << " position_mm="
                  << p.x * 1000.0 << ',' << p.y * 1000.0 << ',' << p.z * 1000.0 << '\n';
        return ok ? 0 : 1;
    }

    if (!run_homing(controller, csv)) {
        std::cerr << "homing failed before scenario\n";
        write_events(options.output / (options.scenario + "_events.csv"), controller);
        return 2;
    }
    const MotionConstraints line{0.05, 0.10, 0.50};
    if (options.scenario == "nominal") {
        auto result = controller.enqueue_move_line({0, 0, 0}, {0.03, 0.04, 0.05}, line);
        std::cout << "line_command_id=" << result.command_id << " accepted=" << result.accepted << '\n';
    } else if (options.scenario == "queue") {
        // Keep the demonstration arc away from the hard lower boundary.  The
        // runtime limit monitor intentionally treats a boundary arc with
        // insufficient braking distance as unsafe rather than hiding servo
        // overshoot in the demo output.
        controller.enqueue_move_line({0, 0, 0}, {0.05, 0.03, 0}, line);
        controller.enqueue_move_arc({0.05, 0.03, 0}, {0.03, 0.05, 0}, {0.03, 0.03, 0},
                                     ArcCommandDirection::CCW, {0.02, 0.04, 0.20});
        controller.enqueue_dwell(0.05);
        controller.enqueue_move_line({0.03, 0.05, 0}, {0.01, 0.01, 0}, line);
    } else if (options.scenario == "soft-limit") {
        auto result = controller.enqueue_move_line({0, 0, 0}, {0.15, 0, 0}, line);
        std::cout << "out_of_limit_accepted=" << result.accepted << " reason=" << result.message << '\n';
        write_events(options.output / "soft-limit_events.csv", controller);
        return result.accepted ? 1 : 0;
    } else if (options.scenario == "controlled-stop") {
        controller.enqueue_move_line({0, 0, 0}, {0.08, 0, 0}, line);
    } else if (options.scenario == "emergency-stop") {
        controller.enqueue_move_line({0, 0, 0}, {0.08, 0, 0}, line);
        controller.enqueue_move_line({0.08, 0, 0}, {0.02, 0.02, 0}, line);
    } else if (options.scenario == "fault-recovery") {
        controller.enqueue_move_line({0, 0, 0}, {0.08, 0, 0}, line);
    } else {
        std::cerr << "unknown scenario: " << options.scenario << '\n';
        return 2;
    }

    bool injected = false;
    for (int k = 0; k < 30000; ++k) {
        const double t = k * 0.001;
        if (options.scenario == "controlled-stop" && t >= 0.50 && !injected) {
            controller.stop(); injected = true;
        }
        if (options.scenario == "emergency-stop" && t >= 0.30 && !injected) {
            controller.emergency_stop(); injected = true;
        }
        if (options.scenario == "fault-recovery" && t >= 0.30 && !injected) {
            controller.inject_fault(FaultCode::FollowingError, "deterministic demo fault"); injected = true;
        }
        if (options.scenario == "fault-recovery" && injected && t >= 0.50 &&
            controller.state() == MotionState::Fault) {
            // 故障原因已由演示注入器停止，显式复位后必须重新回零。
            controller.reset_fault();
            controller.home();
        }
        controller.step(0.001);
        write_row(csv, controller);
        if ((options.scenario == "nominal" || options.scenario == "queue") &&
            controller.state() == MotionState::Ready && controller.queue().empty() &&
            controller.status().command_id == 0 && k > 3000) break;
    }
    write_events(options.output / (options.scenario + "_events.csv"), controller);
    const auto& s = controller.status();
    std::cout << "scenario=" << options.scenario << " state=" << motioncore::manager::to_string(s.state)
              << " final_position_mm=" << s.actual_position_m.x * 1000.0 << ','
              << s.actual_position_m.y * 1000.0 << ',' << s.actual_position_m.z * 1000.0
              << " final_velocity_mm_s=" << s.actual_velocity_mps.norm() * 1000.0
              << " following_error_mm=" << s.following_error_m * 1000.0
              << " fault=" << motioncore::manager::to_string(s.fault_code) << '\n';
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    try { return run_scenario(parse(argc, argv)); }
    catch (const std::exception& exception) {
        std::cerr << "motioncore_phase5 error: " << exception.what() << '\n';
        return 1;
    }
}


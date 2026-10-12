#include "motioncore/control/cascade_servo_controller.hpp"
#include "motioncore/model/dynamic_axis.hpp"
#include "motioncore/planning/s_curve_profile.hpp"
#include "motioncore/planning/trapezoidal_profile.hpp"
#include "motioncore/planning/trajectory_profile.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using motioncore::planning::MotionConstraints;
using motioncore::planning::SCurveProfile;
using motioncore::planning::TrajectoryProfile;
using motioncore::planning::TrajectoryRequest;
using motioncore::planning::TrajectoryState;
using motioncore::planning::TrapezoidalProfile;

struct Options {
    std::string profile{"scurve"};
    std::string mode{"planner"};
    double start_mm{0.0};
    double target_mm{100.0};
    double vmax_mm_s{50.0};
    double amax_mm_s2{100.0};
    double jmax_mm_s3{500.0};
    double dt_ms{1.0};
    double hold_s{0.0};
    std::filesystem::path output{};
};

void print_usage() {
    std::cout << "Usage: motioncore_phase3 [options]\n"
                 "  --profile trapezoid|scurve\n"
                 "  --mode planner|closed-loop\n"
                 "  --start-mm VALUE --target-mm VALUE\n"
                 "  --vmax-mm-s VALUE --amax-mm-s2 VALUE --jmax-mm-s3 VALUE\n"
                 "  --dt-ms VALUE --hold-s VALUE --output PATH\n";
}

double parse_number(const std::string& text, const std::string& option) {
    std::size_t consumed = 0;
    const double value = std::stod(text, &consumed);
    if (consumed != text.size() || !std::isfinite(value)) {
        throw std::invalid_argument(option + " requires a finite number");
    }
    return value;
}

Options parse_options(int argc, char* argv[]) {
    Options options;
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "--help" || argument == "-h") {
            print_usage();
            std::exit(0);
        }
        if (index + 1 >= argc) {
            throw std::invalid_argument("missing value for " + argument);
        }
        const std::string value = argv[++index];
        if (argument == "--profile") {
            options.profile = value;
        } else if (argument == "--mode") {
            options.mode = value;
        } else if (argument == "--start-mm") {
            options.start_mm = parse_number(value, argument);
        } else if (argument == "--target-mm") {
            options.target_mm = parse_number(value, argument);
        } else if (argument == "--vmax-mm-s") {
            options.vmax_mm_s = parse_number(value, argument);
        } else if (argument == "--amax-mm-s2") {
            options.amax_mm_s2 = parse_number(value, argument);
        } else if (argument == "--jmax-mm-s3") {
            options.jmax_mm_s3 = parse_number(value, argument);
        } else if (argument == "--dt-ms") {
            options.dt_ms = parse_number(value, argument);
        } else if (argument == "--hold-s") {
            options.hold_s = parse_number(value, argument);
        } else if (argument == "--output") {
            options.output = value;
        } else {
            throw std::invalid_argument("unknown option: " + argument);
        }
    }
    if (options.profile != "trapezoid" && options.profile != "scurve") {
        throw std::invalid_argument("--profile must be trapezoid or scurve");
    }
    if (options.mode != "planner" && options.mode != "closed-loop") {
        throw std::invalid_argument("--mode must be planner or closed-loop");
    }
    if (options.dt_ms <= 0.0) {
        throw std::invalid_argument("--dt-ms must be positive");
    }
    if (options.hold_s < 0.0) {
        throw std::invalid_argument("--hold-s must be non-negative");
    }
    if (options.output.empty()) {
        options.output = std::filesystem::path("results/phase3")
            / (options.profile + "_" + options.mode + ".csv");
    }
    return options;
}

TrajectoryRequest make_request(const Options& options) {
    TrajectoryRequest request;
    request.start_position_m = options.start_mm / 1000.0;
    request.target_position_m = options.target_mm / 1000.0;
    request.start_velocity_mps = 0.0;
    request.end_velocity_mps = 0.0;
    request.constraints = {
        options.vmax_mm_s / 1000.0,
        options.amax_mm_s2 / 1000.0,
        options.jmax_mm_s3 / 1000.0,
    };
    return request;
}

std::unique_ptr<TrajectoryProfile> make_profile(const Options& options,
                                                const TrajectoryRequest& request) {
    if (options.profile == "trapezoid") {
        return std::make_unique<TrapezoidalProfile>(request);
    }
    return std::make_unique<SCurveProfile>(request);
}

struct CsvRow {
    double time_s;
    TrajectoryState reference;
    bool has_actual{false};
    motioncore::model::DynamicAxisState actual{};
    double position_error_m{0.0};
    double drive_force_n{0.0};
};

void write_csv(const std::filesystem::path& path,
               const std::vector<CsvRow>& rows,
               bool closed_loop) {
    if (!path.parent_path().empty()) {
        std::filesystem::create_directories(path.parent_path());
    }
    std::ofstream output(path);
    if (!output) {
        throw std::runtime_error("cannot open output: " + path.string());
    }
    output << std::setprecision(12);
    output << "time_s,reference_position_mm,reference_velocity_mm_s,"
              "reference_acceleration_mm_s2,reference_jerk_mm_s3";
    if (closed_loop) {
        output << ",actual_position_mm,actual_velocity_mm_s,"
                  "actual_acceleration_mm_s2,position_error_mm,drive_force_N";
    }
    output << '\n';
    for (const auto& row : rows) {
        output << row.time_s << ','
               << row.reference.position_m * 1000.0 << ','
               << row.reference.velocity_mps * 1000.0 << ','
               << row.reference.acceleration_mps2 * 1000.0 << ','
               << row.reference.jerk_mps3 * 1000.0;
        if (closed_loop) {
            output << ',' << row.actual.position_m * 1000.0
                   << ',' << row.actual.velocity_mps * 1000.0
                   << ',' << row.actual.acceleration_mps2 * 1000.0
                   << ',' << row.position_error_m * 1000.0
                   << ',' << row.drive_force_n;
        }
        output << '\n';
    }
}

std::vector<CsvRow> run_planner(const TrajectoryProfile& profile,
                                const Options& options) {
    const double dt = options.dt_ms / 1000.0;
    const double duration = profile.duration();
    std::vector<CsvRow> rows;
    for (double time = 0.0; time < duration - 1e-12; time += dt) {
        rows.push_back({time, profile.sample(time)});
    }
    rows.push_back({duration, profile.sample(duration)});
    return rows;
}

std::vector<CsvRow> run_closed_loop(const TrajectoryProfile& profile,
                                    const TrajectoryRequest& request,
                                    const Options& options) {
    const double nominal_dt = options.dt_ms / 1000.0;
    motioncore::model::DynamicAxisConfig axis_config;
    axis_config.initial_position_m = request.start_position_m;
    axis_config.initial_velocity_mps = 0.0;
    motioncore::model::DynamicAxisModel axis(axis_config);

    motioncore::control::CascadeServoConfig servo_config;
    servo_config.position_kp_per_s = 8.0;
    servo_config.max_target_velocity_mps = request.constraints.max_velocity_mps;
    servo_config.max_target_acceleration_mps2 = request.constraints.max_acceleration_mps2;
    servo_config.speed_mode = motioncore::control::SpeedControllerMode::PI;
    servo_config.speed_kp = 20.0;
    servo_config.speed_ki = 80.0;
    servo_config.max_drive_force_n = 20.0;
    servo_config.integral_min = -0.25;
    servo_config.integral_max = 0.25;
    servo_config.anti_windup = true;
    motioncore::control::CascadedServoController controller(servo_config);

    std::vector<CsvRow> rows;
    double time = 0.0;
    const auto initial_reference = profile.sample(0.0);
    rows.push_back({0.0, initial_reference, true, axis.state(),
                    initial_reference.position_m - axis.state().position_m, 0.0});
    while (time < profile.duration() - 1e-12) {
        const auto reference = profile.sample(time);
        const auto command = controller.update_with_velocity_reference(
            reference.position_m, reference.velocity_mps, axis.state(), nominal_dt);
        const double step = std::min(nominal_dt, profile.duration() - time);
        axis.step(command.drive_force_n, step);
        time += step;
        const auto next_reference = profile.sample(time);
        rows.push_back({time, next_reference, true, axis.state(),
                        next_reference.position_m - axis.state().position_m,
                        command.drive_force_n});
    }
    // 轨迹结束后继续以终点为位置目标、零速度为速度前馈，观察伺服
    // 是否能把结束瞬间的动态误差收敛掉。这个保持阶段不会修改实际状态。
    const double hold_end = profile.duration() + options.hold_s;
    while (time < hold_end - 1e-12) {
        const auto endpoint = profile.sample(profile.duration());
        const auto command = controller.update_with_velocity_reference(
            endpoint.position_m, 0.0, axis.state(), nominal_dt);
        const double step = std::min(nominal_dt, hold_end - time);
        axis.step(command.drive_force_n, step);
        time += step;
        auto hold_reference = endpoint;
        hold_reference.time_s = time;
        rows.push_back({time, hold_reference, true, axis.state(),
                        endpoint.position_m - axis.state().position_m,
                        command.drive_force_n});
    }
    return rows;
}

void print_result(const Options& options, const TrajectoryProfile& profile,
                  const std::vector<CsvRow>& rows) {
    const auto& last = rows.back();
    double max_error_mm = 0.0;
    double max_drive_force_n = 0.0;
    for (const auto& row : rows) {
        if (row.has_actual) {
            max_error_mm = std::max(max_error_mm,
                                    std::abs(row.position_error_m) * 1000.0);
            max_drive_force_n = std::max(max_drive_force_n,
                                         std::abs(row.drive_force_n));
        }
    }
    std::cout << "profile=" << options.profile << ", mode=" << options.mode
              << ", duration_s=" << std::setprecision(12) << profile.duration()
              << ", final_reference_mm=" << last.reference.position_m * 1000.0;
    if (last.has_actual) {
        std::cout << ", final_actual_mm=" << last.actual.position_m * 1000.0
                  << ", final_actual_velocity_mm_s="
                  << last.actual.velocity_mps * 1000.0
                  << ", peak_drive_force_N=" << max_drive_force_n
                  << ", max_tracking_error_mm=" << max_error_mm;
    }
    std::cout << ", csv=" << options.output.string() << '\n';
}

}  // namespace

int main(int argc, char* argv[]) {
    try {
        const Options options = parse_options(argc, argv);
        const TrajectoryRequest request = make_request(options);
        const auto profile = make_profile(options, request);
        const auto rows = options.mode == "planner"
            ? run_planner(*profile, options)
            : run_closed_loop(*profile, request, options);
        write_csv(options.output, rows, options.mode == "closed-loop");
        print_result(options, *profile, rows);
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Phase 3 failed: " << error.what() << '\n';
        print_usage();
        return 1;
    }
}

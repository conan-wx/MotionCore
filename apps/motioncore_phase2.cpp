#include "motioncore/control/cascade_servo_controller.hpp"
#include "motioncore/io/csv_logger.hpp"
#include "motioncore/metrics/metrics.hpp"
#include "motioncore/model/dynamic_axis.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr double kDt = 0.001;
constexpr double kDuration = 8.0;
constexpr double kTargetPosition = 0.1;
constexpr double kErrorBand = 0.0002;
constexpr double kHoldTime = 0.2;
const std::filesystem::path kResultDirectory = "results/phase2";

using motioncore::control::CascadeServoConfig;
using motioncore::control::CascadedServoController;
using motioncore::control::SpeedControllerMode;
using motioncore::metrics::SimulationMetrics;
using motioncore::metrics::SimulationSample;
using motioncore::model::DynamicAxisConfig;
using motioncore::model::DynamicAxisModel;

struct RunResult {
    std::string name;
    std::vector<SimulationSample> samples;
    SimulationMetrics metrics;
};

CascadeServoConfig make_controller_config(SpeedControllerMode mode,
                                          bool feedforward) {
    CascadeServoConfig config;
    config.position_kp_per_s = 8.0;
    config.max_target_velocity_mps = 0.1;
    config.max_target_acceleration_mps2 = 0.8;
    config.speed_mode = mode;
    config.speed_kp = 20.0;
    config.speed_ki = 80.0;
    config.max_drive_force_n = 20.0;
    config.integral_min = -0.25;
    config.integral_max = 0.25;
    config.anti_windup = true;
    config.damping_feedforward = feedforward;
    config.viscous_damping_n_s_per_m = 3.0;
    return config;
}

RunResult run_closed_loop(const std::string& name,
                          DynamicAxisConfig axis_config,
                          CascadeServoConfig controller_config) {
    DynamicAxisModel axis(axis_config);
    CascadedServoController controller(controller_config);
    std::vector<SimulationSample> samples;
    samples.reserve(static_cast<std::size_t>(kDuration / kDt) + 1U);

    const auto append_sample = [&](double target_velocity_mps,
                                   double drive_force_n) {
        const auto& state = axis.state();
        samples.push_back({state.time_s,
                           kTargetPosition,
                           state.position_m,
                           target_velocity_mps,
                           state.velocity_mps,
                           state.acceleration_mps2,
                           drive_force_n,
                           state.load_force_n,
                           kTargetPosition - state.position_m});
    };

    append_sample(0.0, 0.0);
    const auto steps = static_cast<int>(std::llround(kDuration / kDt));
    for (int i = 0; i < steps; ++i) {
        const auto command = controller.update(kTargetPosition, axis.state(), kDt);
        axis.step(command.drive_force_n, kDt);
        append_sample(command.target_velocity_mps, command.drive_force_n);
    }

    const auto metrics = motioncore::metrics::compute_metrics(
        samples, {kErrorBand, kHoldTime});
    motioncore::io::write_simulation_csv(
        kResultDirectory / (name + ".csv"), samples);
    return {name, std::move(samples), metrics};
}

double recovery_time_after(const std::vector<SimulationSample>& samples,
                           double start_time_s) {
    if (samples.size() < 2U) {
        return -1.0;
    }
    const double dt = samples[1].time_s - samples[0].time_s;
    const auto hold_samples = static_cast<std::size_t>(std::ceil(kHoldTime / dt));
    for (std::size_t i = 0; i + hold_samples < samples.size(); ++i) {
        if (samples[i].time_s < start_time_s) {
            continue;
        }
        bool in_band = true;
        for (std::size_t j = i; j <= i + hold_samples; ++j) {
            if (std::abs(samples[j].position_error_m) > kErrorBand) {
                in_band = false;
                break;
            }
        }
        if (in_band) {
            return samples[i].time_s - start_time_s;
        }
    }
    return -1.0;
}

void print_metrics(const RunResult& result) {
    const auto& m = result.metrics;
    std::cout << "[" << result.name << "] "
              << "max_overshoot_mm=" << m.max_overshoot_mm
              << ", final_position_error_mm=" << m.final_position_error_mm
              << ", final_velocity_mm_s=" << m.final_velocity_mm_s
              << ", settling_time_s=" << m.settling_time_s
              << ", max_velocity_mm_s=" << m.max_actual_velocity_mm_s
              << ", max_acceleration_mm_s2=" << m.max_actual_acceleration_mm_s2
              << ", settled=" << (m.settled ? "true" : "false") << '\n';
}

void write_summary(const std::vector<RunResult>& results,
                  const std::string& scenario) {
    std::ofstream output(kResultDirectory / (scenario + "_metrics.csv"));
    if (!output) {
        throw std::runtime_error("cannot create metrics summary");
    }
    output << "scenario,max_overshoot_mm,final_position_error_mm,final_velocity_mm_s,"
              "settling_time_s,max_actual_velocity_mm_s,max_actual_acceleration_mm_s2\n";
    output << std::setprecision(12);
    for (const auto& result : results) {
        const auto& m = result.metrics;
        output << result.name << ',' << m.max_overshoot_mm << ','
               << m.final_position_error_mm << ',' << m.final_velocity_mm_s << ','
               << m.settling_time_s << ',' << m.max_actual_velocity_mm_s << ','
               << m.max_actual_acceleration_mm_s2 << '\n';
    }
}

void print_usage() {
    std::cout << "Usage: motioncore_phase2 --scenario "
                 "nominal|disturbance|comparison|feedforward\n";
}

int run_scenario(const std::string& scenario) {
    std::filesystem::create_directories(kResultDirectory);
    std::vector<RunResult> results;
    DynamicAxisConfig base_axis;
    base_axis.mass_kg = 1.0;
    base_axis.viscous_damping_n_s_per_m = 3.0;
    base_axis.max_drive_force_n = 20.0;

    if (scenario == "nominal") {
        results.push_back(run_closed_loop(
            "nominal", base_axis,
            make_controller_config(SpeedControllerMode::PI, false)));
        print_metrics(results.back());
    } else if (scenario == "disturbance") {
        base_axis.load_profile.pulse_force_n = 2.0;
        base_axis.load_profile.pulse_start_s = 2.0;
        base_axis.load_profile.pulse_duration_s = 0.5;
        results.push_back(run_closed_loop(
            "disturbance", base_axis,
            make_controller_config(SpeedControllerMode::PI, false)));
        print_metrics(results.back());
        const double recovery = recovery_time_after(results.back().samples, 2.5);
        std::cout << "[disturbance] recovery_time_after_pulse_s=" << recovery
                  << " (error band +/-0.2 mm held for 0.2 s)\n";
    } else if (scenario == "comparison") {
        base_axis.load_profile.constant_force_n = 1.0;
        results.push_back(run_closed_loop(
            "comparison_p", base_axis,
            make_controller_config(SpeedControllerMode::P, false)));
        results.push_back(run_closed_loop(
            "comparison_pi", base_axis,
            make_controller_config(SpeedControllerMode::PI, false)));
        for (const auto& result : results) {
            print_metrics(result);
        }
        const auto mean_speed_error = [](const RunResult& result) {
            double sum = 0.0;
            std::size_t count = 0;
            for (auto it = result.samples.rbegin(); it != result.samples.rend()
                 && (*it).time_s >= kDuration - 0.5; ++it) {
                sum += std::abs((*it).target_velocity_mps - (*it).actual_velocity_mps);
                ++count;
            }
            return count == 0 ? 0.0 : sum / static_cast<double>(count);
        };
        const auto& p = results[0].samples.back();
        const auto& pi = results[1].samples.back();
        std::cout << "[comparison] final_speed_error_p_mm_s="
                  << (p.target_velocity_mps - p.actual_velocity_mps) * 1000.0
                  << ", final_speed_error_pi_mm_s="
                  << (pi.target_velocity_mps - pi.actual_velocity_mps) * 1000.0
                  << ", mean_last_0.5s_speed_error_p_mm_s="
                  << mean_speed_error(results[0]) * 1000.0
                  << ", mean_last_0.5s_speed_error_pi_mm_s="
                  << mean_speed_error(results[1]) * 1000.0 << '\n';
    } else if (scenario == "feedforward") {
        results.push_back(run_closed_loop(
            "feedforward_off", base_axis,
            make_controller_config(SpeedControllerMode::PI, false)));
        results.push_back(run_closed_loop(
            "feedforward_on", base_axis,
            make_controller_config(SpeedControllerMode::PI, true)));
        for (const auto& result : results) {
            print_metrics(result);
        }
        double peak_off = 0.0;
        double peak_on = 0.0;
        for (const auto& sample : results[0].samples) {
            peak_off = std::max(peak_off, std::abs(sample.drive_force_n));
        }
        for (const auto& sample : results[1].samples) {
            peak_on = std::max(peak_on, std::abs(sample.drive_force_n));
        }
        std::cout << "[feedforward] peak_drive_force_off_N=" << peak_off
                  << ", peak_drive_force_on_N=" << peak_on << '\n';
    } else {
        print_usage();
        return 2;
    }

    write_summary(results, scenario);
    return 0;
}

}  // namespace

int main(int argc, char* argv[]) {
    std::string scenario = "nominal";
    for (int i = 1; i < argc; ++i) {
        const std::string argument = argv[i];
        if (argument == "--scenario" && i + 1 < argc) {
            scenario = argv[++i];
        } else if (argument == "--help" || argument == "-h") {
            print_usage();
            return 0;
        } else {
            print_usage();
            return 2;
        }
    }

    try {
        return run_scenario(scenario);
    } catch (const std::exception& error) {
        std::cerr << "Phase 2 simulation failed: " << error.what() << '\n';
        return 1;
    }
}

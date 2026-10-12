#include "motioncore/control/cascade_servo_controller.hpp"
#include "motioncore/control/p_controller.hpp"
#include "motioncore/control/pid_controller.hpp"
#include "motioncore/control/pi_controller.hpp"
#include "motioncore/io/csv_logger.hpp"
#include "motioncore/metrics/metrics.hpp"
#include "motioncore/model/dynamic_axis.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

constexpr double kDt = 0.001;
constexpr double kTargetPositionM = 0.1;

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void require_near(double actual, double expected, double tolerance,
                  const std::string& message) {
    if (std::abs(actual - expected) > tolerance) {
        std::ostringstream stream;
        stream << message << " (actual=" << actual << ", expected=" << expected
               << ", tolerance=" << tolerance << ')';
        throw std::runtime_error(stream.str());
    }
}

template <typename Function>
void require_invalid_argument(Function&& function, const std::string& message) {
    bool threw = false;
    try {
        function();
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    require(threw, message);
}

motioncore::control::CascadeServoConfig make_cascade_config(
    motioncore::control::SpeedControllerMode mode =
        motioncore::control::SpeedControllerMode::PI) {
    motioncore::control::CascadeServoConfig config;
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
    config.damping_feedforward = false;
    config.viscous_damping_n_s_per_m = 3.0;
    return config;
}

motioncore::model::DynamicAxisConfig make_axis_config() {
    motioncore::model::DynamicAxisConfig config;
    config.mass_kg = 1.0;
    config.viscous_damping_n_s_per_m = 3.0;
    config.max_drive_force_n = 20.0;
    return config;
}

void test_dynamics_update_formula() {
    motioncore::model::DynamicAxisConfig config;
    config.mass_kg = 2.0;
    config.viscous_damping_n_s_per_m = 1.0;
    config.max_drive_force_n = 5.0;
    config.initial_position_m = 0.1;
    config.initial_velocity_mps = 0.2;
    config.load_profile.constant_force_n = 0.5;

    motioncore::model::DynamicAxisModel axis(config);
    const auto& state = axis.step(10.0, 0.1);  // 请求力应被限制到 5 N。

    // a=(5-1*0.2-0.5)/2=2.15 m/s²，采用常加速度积分。
    require_near(state.acceleration_mps2, 2.15, 1e-12,
                 "dynamic acceleration formula is incorrect");
    require_near(state.velocity_mps, 0.415, 1e-12,
                 "dynamic velocity integration is incorrect");
    require_near(state.position_m, 0.13075, 1e-12,
                 "dynamic position integration is incorrect");
    require_near(state.drive_force_n, 5.0, 1e-12,
                 "dynamic drive-force saturation is incorrect");
    require_near(state.load_force_n, 0.5, 1e-12,
                 "dynamic load force is incorrect");
    require_near(state.time_s, 0.1, 1e-12, "dynamic time update is incorrect");
}

void test_dynamics_forward_and_reverse_motion() {
    motioncore::model::DynamicAxisConfig config;
    config.mass_kg = 1.0;
    config.viscous_damping_n_s_per_m = 0.0;
    config.max_drive_force_n = 10.0;

    motioncore::model::DynamicAxisModel forward(config);
    forward.step(2.0, 0.1);
    require(forward.state().position_m > 0.0 && forward.state().velocity_mps > 0.0,
            "positive drive force should produce positive motion");

    motioncore::model::DynamicAxisModel reverse(config);
    reverse.step(-2.0, 0.1);
    require(reverse.state().position_m < 0.0 && reverse.state().velocity_mps < 0.0,
            "negative drive force should produce negative motion");
}

void test_p_controller_limits_and_reset() {
    motioncore::control::PController controller({2.0, -3.0, 3.0});
    require_near(controller.update(10.0), 3.0, 1e-12,
                 "P controller positive output limit failed");
    require_near(controller.update(-10.0), -3.0, 1e-12,
                 "P controller negative output limit failed");
    controller.reset();
    require_near(controller.last_output(), 0.0, 1e-12,
                 "P controller reset failed");
}

void test_pi_anti_windup_and_limits() {
    motioncore::control::PIController anti_windup({0.0, 1.0, -1.0, 1.0,
                                                    -100.0, 100.0, true});
    require_near(anti_windup.update(0.5, 0.1), 0.05, 1e-12,
                 "PI pre-saturation integration failed");
    for (int i = 0; i < 20; ++i) {
        require_near(anti_windup.update(10.0, 0.2), 1.0, 1e-12,
                     "PI output limit failed while saturated");
    }
    // The first saturated sample must not be allowed to accumulate an
    // ever-growing integral state.
    require_near(anti_windup.integral_state(), 0.05, 1e-12,
            "PI anti-windup did not hold the integral state");
    require(anti_windup.update(-1.0, 0.1) < 0.0,
            "PI output did not recover when the error reversed");
    anti_windup.reset();
    require_near(anti_windup.integral_state(), 0.0, 1e-12,
                 "PI reset did not clear integral state");

    motioncore::control::PIController without_anti_windup({0.0, 10.0, -1.0, 1.0,
                                                           -100.0, 100.0, false});
    for (int i = 0; i < 5; ++i) {
        without_anti_windup.update(1.0, 0.2);
    }
    require(without_anti_windup.integral_state() > 0.5,
            "PI non-anti-windup reference did not accumulate");
}

void test_pid_limits_anti_windup_and_measurement_derivative() {
    motioncore::control::PIDController controller({0.0, 1.0, 0.0, -1.0, 1.0,
                                                    -100.0, 100.0, 0.0, true});
    require_near(controller.update(0.5, 0.0, 0.1), 0.05, 1e-12,
                 "PID pre-saturation integration failed");
    for (int i = 0; i < 10; ++i) {
        require_near(controller.update(10.0, 0.0, 0.2), 1.0, 1e-12,
                     "PID output limit failed while saturated");
    }
    require_near(controller.integral_state(), 0.05, 1e-12,
                 "PID anti-windup did not hold the integral state");
    require(controller.update(-1.0, 0.0, 0.1) < 0.0,
            "PID output did not recover when the error reversed");

    // A setpoint step alone must not create a derivative kick because the
    // derivative is calculated from the measured position.
    motioncore::control::PIDController derivative({0.0, 0.0, 1.0, -100.0, 100.0,
                                                   -100.0, 100.0, 0.0, true});
    require_near(derivative.update(0.0, 0.0, 0.001), 0.0, 1e-12,
                 "PID first derivative sample should be zero");
    require_near(derivative.update(1.0, 0.0, 0.001), 0.0, 1e-12,
                 "PID setpoint step caused derivative kick");
    require_near(derivative.update(1.0, 0.1, 0.1), -1.0, 1e-12,
                 "PID measurement derivative sign is incorrect");

    motioncore::control::PIDController filtered({0.0, 0.0, 1.0, -100.0, 100.0,
                                                  -100.0, 100.0, 0.9, true});
    filtered.update(0.0, 0.0, 0.1);
    const double first_filtered_output = filtered.update(0.0, 0.1, 0.1);
    const double second_filtered_output = filtered.update(0.0, 0.2, 0.1);
    require(std::abs(first_filtered_output) < 0.2
                && std::abs(second_filtered_output) < 0.3,
            "PID derivative filter did not attenuate measurement changes");
}

void test_cascade_closed_loop_to_position() {
    motioncore::model::DynamicAxisModel axis(make_axis_config());
    motioncore::control::CascadedServoController controller(make_cascade_config());
    for (int i = 0; i < 8000; ++i) {
        const auto command = controller.update(kTargetPositionM, axis.state(), kDt);
        axis.step(command.drive_force_n, kDt);
        require(std::isfinite(axis.state().position_m)
                    && std::isfinite(axis.state().velocity_mps),
                "closed-loop state became non-finite");
    }
    require(std::abs(axis.state().position_m - kTargetPositionM) <= 0.0002,
            "cascade servo final position error exceeds 0.2 mm");
    require(std::abs(axis.state().velocity_mps) <= 0.0005,
            "cascade servo final velocity exceeds 0.5 mm/s");
}

void test_cascade_velocity_limit_and_slew_rate() {
    motioncore::model::DynamicAxisModel axis(make_axis_config());
    motioncore::control::CascadedServoController controller(make_cascade_config());
    const auto first = controller.update(kTargetPositionM, axis.state(), kDt);
    const auto second = controller.update(kTargetPositionM, axis.state(), kDt);
    require_near(first.target_velocity_mps, 0.0008, 1e-12,
                 "first target-velocity step does not respect acceleration limit");
    require(second.target_velocity_mps - first.target_velocity_mps <= 0.0008 + 1e-12,
            "target velocity slew rate exceeded acceleration limit");
    require_near(axis.state().velocity_mps, 0.0, 1e-12,
                 "controller update must not overwrite actual model velocity");
}

void test_disturbance_recovery() {
    auto config = make_axis_config();
    config.load_profile.pulse_force_n = 2.0;
    config.load_profile.pulse_start_s = 2.0;
    config.load_profile.pulse_duration_s = 0.5;
    motioncore::model::DynamicAxisModel axis(config);
    motioncore::control::CascadedServoController controller(make_cascade_config());

    bool recovered_after_disturbance = false;
    for (int i = 0; i < 8000; ++i) {
        const auto command = controller.update(kTargetPositionM, axis.state(), kDt);
        axis.step(command.drive_force_n, kDt);
        const double time_s = axis.state().time_s;
        require(std::isfinite(axis.state().position_m)
                    && std::isfinite(axis.state().velocity_mps),
                "disturbance response diverged numerically");
        if (time_s >= 3.0
            && std::abs(axis.state().position_m - kTargetPositionM) <= 0.001) {
            recovered_after_disturbance = true;
        }
    }
    require(recovered_after_disturbance,
            "disturbance response did not return to a 1 mm position band");
    require(std::abs(axis.state().position_m - kTargetPositionM) <= 0.0002,
            "disturbance final position error exceeds 0.2 mm");
    require(std::abs(axis.state().velocity_mps) <= 0.0005,
            "disturbance final velocity exceeds 0.5 mm/s");
}

void test_invalid_parameters() {
    auto axis_config = make_axis_config();
    axis_config.mass_kg = 0.0;
    require_invalid_argument([&] { motioncore::model::DynamicAxisModel axis(axis_config); },
                             "zero mass should be rejected");

    axis_config = make_axis_config();
    axis_config.viscous_damping_n_s_per_m = -1.0;
    require_invalid_argument([&] { motioncore::model::DynamicAxisModel axis(axis_config); },
                             "negative damping should be rejected");

    axis_config = make_axis_config();
    axis_config.max_drive_force_n = 0.0;
    require_invalid_argument([&] { motioncore::model::DynamicAxisModel axis(axis_config); },
                             "non-positive maximum drive force should be rejected");

    motioncore::model::DynamicAxisModel axis(make_axis_config());
    require_invalid_argument([&] { axis.step(0.0, 0.0); },
                             "non-positive integration step should be rejected");
    require_invalid_argument([&] {
        axis.step(std::numeric_limits<double>::quiet_NaN(), kDt);
    }, "non-finite drive force should be rejected");

    require_invalid_argument(
        [] { motioncore::control::PController controller({-1.0, -1.0, 1.0}); },
        "negative P gain should be rejected");
    require_invalid_argument(
        [] { motioncore::control::PIController controller({1.0, 1.0, 1.0, -1.0,
                                                            -1.0, 1.0, true}); },
        "reversed PI output limits should be rejected");
    require_invalid_argument(
        [] { motioncore::control::PIDController controller({1.0, 1.0, 1.0, -1.0,
                                                             1.0, -1.0, 1.0, 1.0,
                                                             true}); },
        "invalid PID derivative filter should be rejected");
    require_invalid_argument(
        [] { motioncore::control::PIController controller({1.0, 1.0, -1.0, 1.0,
                                                            -1.0, 1.0, true});
             controller.update(1.0, 0.0); },
        "non-positive PI time step should be rejected");
    require_invalid_argument(
        [] { motioncore::control::CascadedServoController controller(
                [] {
                    auto config = make_cascade_config();
                    config.max_target_velocity_mps = 0.0;
                    return config;
                }()); },
        "non-positive cascade velocity limit should be rejected");
}

void test_csv_data_completeness() {
    using motioncore::metrics::SimulationSample;
    std::vector<SimulationSample> samples;
    samples.push_back({0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1});
    samples.push_back({kDt, 0.1, 0.000001, 0.001, 0.001, 1.0, 1.5, 0.0,
                      0.099999});

    const auto path = std::filesystem::temp_directory_path()
        / "motioncore_phase2_csv_completeness.csv";
    motioncore::io::write_csv(path, samples);

    std::ifstream input(path);
    require(input.good(), "CSV logger did not create its output file");
    std::string line;
    require(static_cast<bool>(std::getline(input, line)), "CSV header is missing");
    require(line
                == "time_s,target_position_mm,actual_position_mm,target_velocity_mm_s,"
                   "actual_velocity_mm_s,actual_acceleration_mm_s2,drive_force_N,"
                   "load_force_N,position_error_mm",
            "CSV header does not contain the required fields");

    int data_rows = 0;
    std::vector<double> first_data_row;
    while (std::getline(input, line)) {
        if (line.empty()) {
            continue;
        }
        ++data_rows;
        std::stringstream row(line);
        std::string field;
        int fields = 0;
        std::vector<double> values;
        while (std::getline(row, field, ',')) {
            ++fields;
            values.push_back(std::stod(field));
        }
        require(fields == 9, "CSV row does not contain nine numeric fields");
        if (data_rows == 1) {
            first_data_row = std::move(values);
        }
    }
    require(data_rows == 2, "CSV row count does not match simulation sample count");
    require_near(first_data_row.at(1), 100.0, 1e-12,
                 "CSV position was not converted from metres to millimetres");
    require_near(first_data_row.at(8), 100.0, 1e-12,
                 "CSV position error was not converted from metres to millimetres");
    input.close();
    std::filesystem::remove(path);
}

}  // namespace

int main() {
    try {
        test_dynamics_update_formula();
        test_dynamics_forward_and_reverse_motion();
        test_p_controller_limits_and_reset();
        test_pi_anti_windup_and_limits();
        test_pid_limits_anti_windup_and_measurement_derivative();
        test_cascade_velocity_limit_and_slew_rate();
        test_cascade_closed_loop_to_position();
        test_disturbance_recovery();
        test_invalid_parameters();
        test_csv_data_completeness();
        std::cout << "MotionCore 第二阶段测试全部通过\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Phase 2 测试失败: " << error.what() << '\n';
        return 1;
    }
}

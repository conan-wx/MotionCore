#include "motioncore/control/basic_servo_controller.hpp"
#include "motioncore/model/axis_simulator.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>

int main() {
    using motioncore::control::BasicServoConfig;
    using motioncore::control::BasicServoController;
    using motioncore::model::AxisLimits;
    using motioncore::model::AxisSimulator;

    constexpr double dt_seconds = 0.001;
    constexpr double duration_seconds = 8.0;
    constexpr double target_position = 100.0;  // mm

    const AxisLimits axis_limits{80.0, 160.0};
    const BasicServoConfig controller_config{2.0, 4.0, 80.0, 160.0};
    AxisSimulator axis(axis_limits);
    BasicServoController controller(controller_config);

    std::filesystem::create_directories("build/logs");
    std::ofstream log("build/logs/motioncore_phase1_axis.csv");
    if (!log) {
        std::cerr << "无法创建日志文件 build/logs/motioncore_phase1_axis.csv\n";
        return 1;
    }

    log << "time_s,target_position_mm,position_mm,velocity_mm_s,"
           "acceleration_mm_s2,position_error_mm,command_acceleration_mm_s2\n";

    double max_abs_velocity = 0.0;
    double max_abs_acceleration = 0.0;
    const int steps = static_cast<int>(duration_seconds / dt_seconds);
    for (int step = 0; step < steps; ++step) {
        const auto& state = axis.state();
        const double command = controller.update(
            target_position, state, dt_seconds);
        axis.step(command, dt_seconds);
        const auto& next_state = axis.state();

        max_abs_velocity = std::max(max_abs_velocity, std::abs(next_state.velocity));
        max_abs_acceleration = std::max(
            max_abs_acceleration, std::abs(next_state.acceleration));

        log << std::fixed << std::setprecision(6)
            << next_state.time_s << ','
            << target_position << ','
            << next_state.position << ','
            << next_state.velocity << ','
            << next_state.acceleration << ','
            << (target_position - next_state.position) << ','
            << command << '\n';
    }

    const double final_error = target_position - axis.state().position;
    std::cout << std::fixed << std::setprecision(4)
              << "MotionCore 第一阶段单轴仿真完成\n"
              << "目标位置: " << target_position << " mm\n"
              << "最终位置: " << axis.state().position << " mm\n"
              << "最终误差: " << final_error << " mm\n"
              << "最大速度: " << max_abs_velocity << " mm/s\n"
              << "最大加速度: " << max_abs_acceleration << " mm/s^2\n"
              << "日志文件: build/logs/motioncore_phase1_axis.csv\n";

    return std::abs(final_error) < 0.5 ? 0 : 2;
}

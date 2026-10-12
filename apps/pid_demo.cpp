#include "motion_control/pid_controller.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>

namespace {

struct MotorModel {
    double position{0.0};
    double velocity{0.0};

    void step(double command, double dt_seconds) {
        // A small, deterministic motor-like model: command produces
        // acceleration and viscous friction opposes velocity.
        constexpr double viscous_friction = 4.0;
        velocity += (command - viscous_friction * velocity) * dt_seconds;
        position += velocity * dt_seconds;
    }
};

}  // namespace

int main() {
    using motion_control::PIDConfig;
    using motion_control::PIDController;

    constexpr double dt_seconds = 0.01;
    constexpr double duration_seconds = 8.0;
    constexpr double target_position = 1.0;

    PIDConfig pid_config;
    pid_config.kp = 24.0;
    pid_config.ki = 6.0;
    pid_config.kd = 3.0;
    pid_config.output_min = -12.0;
    pid_config.output_max = 12.0;
    pid_config.integral_min = -2.0;
    pid_config.integral_max = 2.0;
    PIDController controller(pid_config);
    MotorModel motor;

    std::filesystem::create_directories("build/logs");
    std::ofstream log("build/logs/pid_demo.csv");
    if (!log) {
        std::cerr << "无法创建日志文件 build/logs/pid_demo.csv\n";
        return 1;
    }

    log << "time_s,target,position,velocity,error,command\n";
    const int steps = static_cast<int>(duration_seconds / dt_seconds);
    for (int step = 0; step <= steps; ++step) {
        const double time_seconds = step * dt_seconds;
        const double command = controller.update(
            target_position, motor.position, dt_seconds);
        motor.step(command, dt_seconds);

        log << std::fixed << std::setprecision(6)
            << time_seconds << ','
            << target_position << ','
            << motor.position << ','
            << motor.velocity << ','
            << (target_position - motor.position) << ','
            << command << '\n';
    }

    const double final_error = std::abs(target_position - motor.position);
    std::cout << std::fixed << std::setprecision(4)
              << "PID 仿真完成\n"
              << "目标位置: " << target_position << "\n"
              << "最终位置: " << motor.position << "\n"
              << "最终误差: " << final_error << "\n"
              << "日志文件: build/logs/pid_demo.csv\n";

    return final_error < 0.02 ? 0 : 2;
}

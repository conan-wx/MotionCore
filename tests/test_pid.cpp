#include "motion_control/pid_controller.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void test_positive_step() {
    motion_control::PIDConfig config;
    config.kp = 2.0;
    config.output_min = -10.0;
    config.output_max = 10.0;
    motion_control::PIDController controller(config);
    require(controller.update(1.0, 0.0, 0.1) > 0.0,
            "positive position error must produce positive command");
}

void test_output_saturation() {
    motion_control::PIDConfig config;
    config.kp = 100.0;
    config.output_min = -1.0;
    config.output_max = 1.0;
    motion_control::PIDController controller(config);
    require(std::abs(controller.update(1.0, 0.0, 0.1) - 1.0) < 1e-12,
            "command must respect the upper output limit");
}

void test_reset() {
    motion_control::PIDConfig config;
    config.ki = 1.0;
    config.integral_min = -10.0;
    config.integral_max = 10.0;
    motion_control::PIDController controller(config);
    controller.update(1.0, 0.0, 1.0);
    require(controller.integral() > 0.0, "integral should accumulate before reset");
    controller.reset();
    require(std::abs(controller.integral()) < 1e-12,
            "reset must clear the integral state");
}

void test_invalid_dt() {
    motion_control::PIDController controller(motion_control::PIDConfig{});
    bool threw = false;
    try {
        controller.update(0.0, 0.0, 0.0);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    require(threw, "non-positive dt must be rejected");
}

}  // namespace

int main() {
    try {
        test_positive_step();
        test_output_saturation();
        test_reset();
        test_invalid_dt();
        std::cout << "所有 PID 单元测试通过\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "测试失败: " << error.what() << '\n';
        return 1;
    }
}

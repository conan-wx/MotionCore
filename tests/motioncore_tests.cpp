#include "motioncore/control/basic_servo_controller.hpp"
#include "motioncore/model/axis_simulator.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void test_discrete_kinematics() {
    motioncore::model::AxisSimulator axis({10.0, 4.0});
    axis.step(2.0, 0.5);
    require(std::abs(axis.state().velocity - 1.0) < 1e-12,
            "velocity integration is incorrect");
    require(std::abs(axis.state().position - 0.25) < 1e-12,
            "position integration is incorrect");
    require(std::abs(axis.state().acceleration - 2.0) < 1e-12,
            "acceleration state is incorrect");
}

void test_limits() {
    motioncore::model::AxisSimulator axis({1.0, 2.0});
    axis.step(100.0, 1.0);
    require(std::abs(axis.state().velocity - 1.0) < 1e-12,
            "velocity limit is not enforced");
    require(std::abs(axis.state().position - 0.5) < 1e-12,
            "limited velocity integration is incorrect");
}

void test_controller_direction_and_limits() {
    motioncore::control::BasicServoController controller({2.0, 4.0, 3.0, 5.0});
    motioncore::model::AxisState state;
    const double command = controller.update(10.0, state, 0.01);
    require(std::abs(command - 5.0) < 1e-12,
            "controller acceleration limit is not enforced");
    require(controller.position_error() > 0.0,
            "positive target error should be reported");
    require(std::abs(controller.desired_velocity() - 3.0) < 1e-12,
            "controller velocity limit is not enforced");
}

void test_closed_loop_convergence() {
    motioncore::model::AxisSimulator axis({80.0, 160.0});
    motioncore::control::BasicServoController controller({2.0, 4.0, 80.0, 160.0});
    constexpr double dt_seconds = 0.001;
    for (int step = 0; step < 8000; ++step) {
        axis.step(controller.update(100.0, axis.state(), dt_seconds), dt_seconds);
    }
    require(std::abs(100.0 - axis.state().position) < 0.5,
            "basic servo did not converge within the phase-one tolerance");
}

void test_negative_target_convergence() {
    motioncore::model::AxisSimulator axis({80.0, 160.0});
    motioncore::control::BasicServoController controller({2.0, 4.0, 80.0, 160.0});
    constexpr double dt_seconds = 0.001;
    for (int step = 0; step < 8000; ++step) {
        axis.step(controller.update(-50.0, axis.state(), dt_seconds), dt_seconds);
    }
    require(std::abs(-50.0 - axis.state().position) < 0.5,
            "basic servo did not converge for a negative target");
}

}  // namespace

int main() {
    try {
        test_discrete_kinematics();
        test_limits();
        test_controller_direction_and_limits();
        test_closed_loop_convergence();
        test_negative_target_convergence();
        std::cout << "MotionCore 第一阶段测试全部通过\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "测试失败: " << error.what() << '\n';
        return 1;
    }
}

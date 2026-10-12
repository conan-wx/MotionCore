#pragma once

#include "motioncore/model/axis_simulator.hpp"

namespace motioncore::control {

struct BasicServoConfig {
    double position_gain{2.0};
    double velocity_gain{4.0};
    double max_velocity{100.0};
    double max_acceleration{200.0};
};

class BasicServoController {
public:
    explicit BasicServoController(BasicServoConfig config = {});

    // A simple cascaded P/P controller for the first phase:
    // position error -> desired velocity -> acceleration command.
    double update(double target_position,
                  const model::AxisState& state,
                  double dt_seconds);

    [[nodiscard]] double position_error() const noexcept { return position_error_; }
    [[nodiscard]] double desired_velocity() const noexcept { return desired_velocity_; }

private:
    BasicServoConfig config_;
    double position_error_{0.0};
    double desired_velocity_{0.0};
};

}  // namespace motioncore::control

#pragma once

#include <limits>

namespace motioncore::model {

struct AxisLimits {
    double max_velocity{100.0};       // position unit / s
    double max_acceleration{200.0};   // position unit / s^2
};

struct AxisState {
    double time_s{0.0};
    double position{0.0};
    double velocity{0.0};
    double acceleration{0.0};
};

class AxisSimulator {
public:
    explicit AxisSimulator(AxisLimits limits = {});

    void reset(AxisState state = {});

    // Advances the axis by one fixed simulation step. The command is an
    // acceleration request and is limited by the configured axis limits.
    const AxisState& step(double acceleration_command, double dt_seconds);

    [[nodiscard]] const AxisState& state() const noexcept { return state_; }
    [[nodiscard]] const AxisLimits& limits() const noexcept { return limits_; }

private:
    AxisLimits limits_;
    AxisState state_;
};

}  // namespace motioncore::model

#pragma once

#include <limits>

namespace motion_control {

struct PIDConfig {
    double kp{0.0};
    double ki{0.0};
    double kd{0.0};
    double output_min{-std::numeric_limits<double>::infinity()};
    double output_max{std::numeric_limits<double>::infinity()};
    double integral_min{-std::numeric_limits<double>::infinity()};
    double integral_max{std::numeric_limits<double>::infinity()};
};

class PIDController {
public:
    explicit PIDController(PIDConfig config);

    void reset();

    // Computes one discrete-time control command.
    // setpoint and measurement use the same physical unit.
    double update(double setpoint, double measurement, double dt_seconds);

    [[nodiscard]] double integral() const noexcept { return integral_; }
    [[nodiscard]] double previous_error() const noexcept { return previous_error_; }

private:
    PIDConfig config_;
    double integral_{0.0};
    double previous_error_{0.0};
    bool has_previous_error_{false};
};

}  // namespace motion_control

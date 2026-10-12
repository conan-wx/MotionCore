#pragma once

namespace motioncore::control {

struct PIDControllerConfig {
    double kp{0.0};
    double ki{0.0};
    double kd{0.0};
    double output_min{-1.0e9};
    double output_max{1.0e9};
    double integral_min{-1.0e9};
    double integral_max{1.0e9};
    // 0 means no history, values near 1.0 smooth measurement noise strongly.
    double derivative_filter_alpha{0.9};
    bool anti_windup{true};
};

class PIDController {
public:
    explicit PIDController(PIDControllerConfig config = {});

    // The derivative is taken from the measurement, not the setpoint. This
    // avoids a derivative kick when the target position changes suddenly.
    double update(double setpoint, double measurement, double dt_seconds);
    void reset() noexcept;

    [[nodiscard]] double integral_state() const noexcept { return integral_state_; }
    [[nodiscard]] double last_output() const noexcept { return last_output_; }

private:
    PIDControllerConfig config_;
    double integral_state_{0.0};
    double previous_measurement_{0.0};
    double filtered_derivative_{0.0};
    double last_output_{0.0};
    bool has_previous_measurement_{false};
};

}  // namespace motioncore::control

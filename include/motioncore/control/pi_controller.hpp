#pragma once

namespace motioncore::control {

struct PIControllerConfig {
    double kp{0.0};
    double ki{0.0};
    double output_min{-1.0e9};
    double output_max{1.0e9};
    double integral_min{-1.0e9};
    double integral_max{1.0e9};
    bool anti_windup{true};
};

class PIController {
public:
    explicit PIController(PIControllerConfig config = {});

    double update(double error, double dt_seconds);
    void reset() noexcept;

    [[nodiscard]] double integral_state() const noexcept { return integral_state_; }
    [[nodiscard]] double last_output() const noexcept { return last_output_; }

private:
    PIControllerConfig config_;
    double integral_state_{0.0};
    double last_output_{0.0};
};

}  // namespace motioncore::control

#pragma once

namespace motioncore::control {

struct PControllerConfig {
    double kp{0.0};
    double output_min{-1.0e9};
    double output_max{1.0e9};
};

class PController {
public:
    explicit PController(PControllerConfig config = {});

    double update(double error);
    void reset() noexcept { last_output_ = 0.0; }

    [[nodiscard]] double last_output() const noexcept { return last_output_; }

private:
    PControllerConfig config_;
    double last_output_{0.0};
};

}  // namespace motioncore::control

#pragma once

#include <vector>

namespace motioncore::metrics {

struct SimulationSample {
    double time_s{0.0};
    double target_position_m{0.0};
    double actual_position_m{0.0};
    double target_velocity_mps{0.0};
    double actual_velocity_mps{0.0};
    double actual_acceleration_mps2{0.0};
    double drive_force_n{0.0};
    double load_force_n{0.0};
    double position_error_m{0.0};
};

struct MetricsConfig {
    double position_error_band_m{0.0002};
    double hold_time_s{0.2};
};

struct SimulationMetrics {
    double max_overshoot_mm{0.0};
    double final_position_error_mm{0.0};
    double final_velocity_mm_s{0.0};
    double settling_time_s{-1.0};
    double max_actual_velocity_mm_s{0.0};
    double max_actual_acceleration_mm_s2{0.0};
    bool settled{false};
};

SimulationMetrics compute_metrics(const std::vector<SimulationSample>& samples,
                                  MetricsConfig config = {});

}  // namespace motioncore::metrics

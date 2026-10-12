#include "motioncore/metrics/metrics.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace motioncore::metrics {

SimulationMetrics compute_metrics(const std::vector<SimulationSample>& samples,
                                  MetricsConfig config) {
    if (samples.empty()) {
        throw std::invalid_argument("cannot compute metrics for empty samples");
    }
    if (!std::isfinite(config.position_error_band_m)
        || !std::isfinite(config.hold_time_s)
        || config.position_error_band_m <= 0.0 || config.hold_time_s < 0.0) {
        throw std::invalid_argument("invalid metrics configuration");
    }

    const double target = samples.back().target_position_m;
    const double direction = target >= samples.front().actual_position_m ? 1.0 : -1.0;
    double max_overshoot_m = 0.0;
    double max_speed = 0.0;
    double max_acceleration = 0.0;
    for (std::size_t index = 0; index < samples.size(); ++index) {
        const auto& sample = samples[index];
        if (!std::isfinite(sample.time_s) || !std::isfinite(sample.target_position_m)
            || !std::isfinite(sample.actual_position_m)
            || !std::isfinite(sample.target_velocity_mps)
            || !std::isfinite(sample.actual_velocity_mps)
            || !std::isfinite(sample.actual_acceleration_mps2)
            || !std::isfinite(sample.drive_force_n)
            || !std::isfinite(sample.load_force_n)
            || !std::isfinite(sample.position_error_m)) {
            throw std::invalid_argument("simulation samples must be finite");
        }
        if (index > 0 && sample.time_s <= samples[index - 1].time_s) {
            throw std::invalid_argument("sample timestamps must strictly increase");
        }
        max_overshoot_m = std::max(
            max_overshoot_m,
            direction * (sample.actual_position_m - target));
        max_speed = std::max(max_speed, std::abs(sample.actual_velocity_mps));
        max_acceleration = std::max(max_acceleration,
                                    std::abs(sample.actual_acceleration_mps2));
    }

    SimulationMetrics result;
    result.max_overshoot_mm = std::max(0.0, max_overshoot_m) * 1000.0;
    result.final_position_error_mm = std::abs(samples.back().position_error_m) * 1000.0;
    result.final_velocity_mm_s = samples.back().actual_velocity_mps * 1000.0;
    result.max_actual_velocity_mm_s = max_speed * 1000.0;
    result.max_actual_acceleration_mm_s2 = max_acceleration * 1000.0;

    double dt = 0.0;
    if (samples.size() > 1) {
        dt = samples[1].time_s - samples[0].time_s;
    }
    if (dt <= 0.0 || !std::isfinite(dt)) {
        throw std::invalid_argument("sample timestamps must increase");
    }
    const auto hold_samples = static_cast<std::size_t>(
        std::ceil(config.hold_time_s / dt));
    if (samples.size() > hold_samples) {
        for (std::size_t i = 0; i + hold_samples < samples.size(); ++i) {
            bool in_band = true;
            for (std::size_t j = i; j <= i + hold_samples; ++j) {
                if (std::abs(samples[j].position_error_m) > config.position_error_band_m) {
                    in_band = false;
                    break;
                }
            }
            if (in_band) {
                result.settling_time_s = samples[i].time_s;
                result.settled = true;
                break;
            }
        }
    }
    return result;
}

}  // namespace motioncore::metrics

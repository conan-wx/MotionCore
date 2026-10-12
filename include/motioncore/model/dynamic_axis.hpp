#pragma once

#include <cmath>

namespace motioncore::model {

// A deterministic rectangular load pulse is enough for Phase 2 experiments.
// Positive load force opposes positive motion because it is subtracted in the model.
struct LoadProfile {
    double constant_force_n{0.0};
    double pulse_force_n{0.0};
    double pulse_start_s{0.0};
    double pulse_duration_s{0.0};

    [[nodiscard]] double force_at(double time_s) const noexcept;
};

struct DynamicAxisConfig {
    double mass_kg{1.0};
    double viscous_damping_n_s_per_m{3.0};
    double max_drive_force_n{20.0};
    double initial_position_m{0.0};
    double initial_velocity_mps{0.0};
    LoadProfile load_profile{};
};

struct DynamicAxisState {
    double time_s{0.0};
    double position_m{0.0};
    double velocity_mps{0.0};
    double acceleration_mps2{0.0};
    double drive_force_n{0.0};
    double load_force_n{0.0};
};

class DynamicAxisModel {
public:
    explicit DynamicAxisModel(DynamicAxisConfig config = {});

    void reset();

    // Integrates one fixed interval with a zero-order-held drive force.
    // The force is saturated before applying m*a = F_drive - b*v - F_load.
    const DynamicAxisState& step(double requested_drive_force_n,
                                 double dt_seconds);

    [[nodiscard]] const DynamicAxisState& state() const noexcept { return state_; }
    [[nodiscard]] const DynamicAxisConfig& config() const noexcept { return config_; }

private:
    DynamicAxisConfig config_;
    DynamicAxisState state_;
};

}  // namespace motioncore::model

#pragma once

#include "motioncore/planning/trajectory_profile.hpp"

namespace motioncore::planning {

class TrapezoidalProfile final : public TrajectoryProfile {
public:
    explicit TrapezoidalProfile(const TrajectoryRequest& request);

    [[nodiscard]] TrajectoryState sample(double time_s) const override;
    [[nodiscard]] double duration() const noexcept override { return total_time_s_; }
    [[nodiscard]] bool is_trapezoidal() const noexcept { return has_cruise_; }
    [[nodiscard]] double peak_velocity_mps() const noexcept { return peak_velocity_mps_; }
    [[nodiscard]] const TrajectoryRequest& request() const noexcept { return request_; }
    [[nodiscard]] const std::vector<ProfileSegment>& segments() const noexcept { return segments_; }

private:
    TrajectoryRequest request_{};
    double direction_{1.0};
    double distance_m_{0.0};
    double acceleration_time_s_{0.0};
    double cruise_time_s_{0.0};
    double peak_velocity_mps_{0.0};
    double total_time_s_{0.0};
    bool has_cruise_{false};
    std::vector<ProfileSegment> segments_;
};

}  // namespace motioncore::planning

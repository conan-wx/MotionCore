#pragma once

#include "motioncore/planning/trajectory_profile.hpp"

namespace motioncore::planning {

class SCurveProfile final : public TrajectoryProfile {
public:
    explicit SCurveProfile(const TrajectoryRequest& request);

    [[nodiscard]] TrajectoryState sample(double time_s) const override;
    [[nodiscard]] double duration() const noexcept override { return total_time_s_; }
    [[nodiscard]] double peak_velocity_mps() const noexcept { return peak_velocity_mps_; }
    [[nodiscard]] double peak_acceleration_mps2() const noexcept { return peak_acceleration_mps2_; }
    [[nodiscard]] const TrajectoryRequest& request() const noexcept { return request_; }
    [[nodiscard]] const std::vector<ProfileSegment>& segments() const noexcept { return segments_; }

private:
    TrajectoryRequest request_{};
    double direction_{1.0};
    double distance_m_{0.0};
    double total_time_s_{0.0};
    double peak_velocity_mps_{0.0};
    double peak_acceleration_mps2_{0.0};
    std::vector<ProfileSegment> segments_;
};

}  // namespace motioncore::planning

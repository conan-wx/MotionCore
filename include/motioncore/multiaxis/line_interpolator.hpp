#pragma once

#include <memory>

#include "motioncore/multiaxis/interpolator.hpp"

namespace motioncore::multiaxis {

class LineInterpolator final : public Interpolator {
public:
    LineInterpolator(const Vector3& start_m,
                     const Vector3& target_m,
                     const planning::MotionConstraints& constraints,
                     PlanningMode mode = PlanningMode::SCurve);

    [[nodiscard]] MultiAxisTrajectoryState sample(double time_s) const override;
    [[nodiscard]] double duration() const noexcept override;
    [[nodiscard]] double path_length_m() const noexcept override { return length_m_; }
    [[nodiscard]] double length_m() const noexcept { return length_m_; }
    [[nodiscard]] double length() const noexcept { return length_m_; }
    [[nodiscard]] const Vector3& start_m() const noexcept { return start_m_; }
    [[nodiscard]] const Vector3& target_m() const noexcept { return target_m_; }
    [[nodiscard]] const Vector3& unit_direction() const noexcept { return unit_direction_; }
    [[nodiscard]] const planning::TrajectoryProfile& scalar_profile() const noexcept { return *profile_; }
    [[nodiscard]] PlanningMode mode() const noexcept { return mode_; }

private:
    Vector3 start_m_{};
    Vector3 target_m_{};
    Vector3 unit_direction_{};
    double length_m_{0.0};
    PlanningMode mode_{PlanningMode::SCurve};
    std::shared_ptr<planning::TrajectoryProfile> profile_;
};

}  // namespace motioncore::multiaxis

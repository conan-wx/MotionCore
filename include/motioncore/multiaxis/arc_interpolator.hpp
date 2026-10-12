#pragma once

#include <limits>
#include <memory>

#include "motioncore/multiaxis/interpolator.hpp"

namespace motioncore::multiaxis {

enum class ArcDirection {
    CW = -1,
    CCW = 1,
    Clockwise = CW,
    CounterClockwise = CCW
};

// 圆弧的合成约束可以独立于切向标量约束给出。
// 例如标量轨迹使用 40 mm/s^2，但允许圆弧合成加速度达到 100 mm/s^2。
struct ArcResultantLimits {
    double max_acceleration_mps2{std::numeric_limits<double>::infinity()};
    double max_jerk_mps3{std::numeric_limits<double>::infinity()};
};

struct ArcKinematicMetrics {
    double tangential_acceleration_mps2{0.0};
    double normal_acceleration_mps2{0.0};
    double resultant_acceleration_mps2{0.0};
    double tangential_jerk_mps3{0.0};
    double normal_jerk_mps3{0.0};
    double resultant_jerk_mps3{0.0};
};

class ArcInterpolator final : public Interpolator {
public:
    ArcInterpolator(const Vector3& start_m,
                    const Vector3& target_m,
                    const Vector3& center_m,
                    ArcDirection direction,
                    const planning::MotionConstraints& constraints,
                    PlanningMode mode = PlanningMode::SCurve,
                    bool full_circle = false,
                    ArcResultantLimits resultant_limits = {});

    [[nodiscard]] MultiAxisTrajectoryState sample(double time_s) const override;
    [[nodiscard]] double duration() const noexcept override;
    [[nodiscard]] double path_length_m() const noexcept override { return arc_length_m_; }
    [[nodiscard]] double length_m() const noexcept { return arc_length_m_; }
    [[nodiscard]] double length() const noexcept { return arc_length_m_; }
    [[nodiscard]] double radius_m() const noexcept { return radius_m_; }
    [[nodiscard]] double start_angle_rad() const noexcept { return start_angle_rad_; }
    [[nodiscard]] double delta_angle_rad() const noexcept { return delta_angle_rad_; }
    [[nodiscard]] const Vector3& start_m() const noexcept { return start_m_; }
    [[nodiscard]] const Vector3& target_m() const noexcept { return target_m_; }
    [[nodiscard]] const Vector3& center_m() const noexcept { return center_m_; }
    [[nodiscard]] ArcDirection direction() const noexcept { return direction_; }
    [[nodiscard]] bool is_full_circle() const noexcept { return full_circle_; }
    [[nodiscard]] const planning::TrajectoryProfile& scalar_profile() const noexcept { return *profile_; }
    [[nodiscard]] ArcResultantLimits resultant_limits() const noexcept { return resultant_limits_; }
    // 返回圆弧曲率约束后实际交给标量规划器的限制。合成约束有限时，
    // 规划器会保守地降低速度、加速度或 Jerk，而不是先生成超限轨迹再裁剪。
    [[nodiscard]] planning::MotionConstraints scalar_constraints() const noexcept {
        return scalar_constraints_;
    }
    [[nodiscard]] ArcKinematicMetrics kinematics(double time_s) const;
    // 直接检查轨迹全程的合成加速度/Jerk；NaN Jerk 表示梯形规划的理论跳变。
    [[nodiscard]] bool satisfies_resultant_limits() const noexcept { return resultant_limits_satisfied_; }

private:
    void validate_and_build(const planning::MotionConstraints& constraints,
                            PlanningMode mode,
                            bool full_circle);
    [[nodiscard]] double normalized_angle_delta() const;

    Vector3 start_m_{};
    Vector3 target_m_{};
    Vector3 center_m_{};
    ArcDirection direction_{ArcDirection::CCW};
    double radius_m_{0.0};
    double start_angle_rad_{0.0};
    double delta_angle_rad_{0.0};
    double arc_length_m_{0.0};
    PlanningMode mode_{PlanningMode::SCurve};
    bool full_circle_{false};
    ArcResultantLimits resultant_limits_{};
    planning::MotionConstraints scalar_constraints_{};
    bool resultant_limits_satisfied_{true};
    std::shared_ptr<planning::TrajectoryProfile> profile_;
};

}  // namespace motioncore::multiaxis

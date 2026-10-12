#pragma once

#include "motioncore/multiaxis/vector3.hpp"
#include "motioncore/multiaxis/arc_interpolator.hpp"

#include <array>
#include <cstddef>
#include <string>

namespace motioncore::manager {

struct AxisSoftLimit {
    double min_position_m{0.0};
    double max_position_m{1.0};
    double max_velocity_mps{1.0};
    double max_acceleration_mps2{1.0};
    double max_braking_deceleration_mps2{1.0};
    double safety_margin_m{0.0};
};

struct LimitCheckResult {
    bool valid{false};
    std::string reason;
    multiaxis::Vector3 closest_margin_m{};
};

// 对目标、有限直线和有限圆弧进行几何限位检查，并提供预测制动距离。
// 所有位置/速度均采用 SI 单位；调用者负责把 mm 转成 m。
class SoftLimitManager {
public:
    SoftLimitManager();
    explicit SoftLimitManager(const std::array<AxisSoftLimit, 3>& limits);

    [[nodiscard]] const std::array<AxisSoftLimit, 3>& limits() const noexcept { return limits_; }
    [[nodiscard]] bool set_axis_limit(std::size_t axis, const AxisSoftLimit& limit) noexcept;
    [[nodiscard]] LimitCheckResult check_position(const multiaxis::Vector3& position_m) const;
    [[nodiscard]] LimitCheckResult check_line(const multiaxis::Vector3& start_m,
                                              const multiaxis::Vector3& target_m) const;
    [[nodiscard]] LimitCheckResult check_arc(const multiaxis::Vector3& start_m,
                                             const multiaxis::Vector3& target_m,
                                             const multiaxis::Vector3& center_m,
                                             multiaxis::ArcDirection direction,
                                             bool full_circle = false) const;

    [[nodiscard]] double stop_distance(double velocity_mps,
                                       std::size_t axis,
                                       double update_delay_s = 0.0) const noexcept;
    [[nodiscard]] bool can_stop_before_limit(double position_m, double velocity_mps,
                                             std::size_t axis,
                                             double update_delay_s = 0.0) const noexcept;
    [[nodiscard]] multiaxis::Vector3 margins(const multiaxis::Vector3& position_m) const noexcept;

private:
    [[nodiscard]] bool valid_limit(const AxisSoftLimit& limit) const noexcept;
    [[nodiscard]] LimitCheckResult check_points(const std::array<multiaxis::Vector3, 8>& points,
                                                std::size_t count) const;
    std::array<AxisSoftLimit, 3> limits_{};
};

}  // namespace motioncore::manager

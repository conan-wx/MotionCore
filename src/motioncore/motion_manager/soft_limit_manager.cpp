#include "motioncore/motion_manager/soft_limit_manager.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace motioncore::manager {
namespace {
constexpr double kPi = 3.1415926535897932384626433832795;
constexpr double kTwoPi = 2.0 * kPi;

double wrap_positive(double angle) {
    angle = std::fmod(angle, kTwoPi);
    if (angle < 0.0) angle += kTwoPi;
    return angle;
}

bool angle_on_arc(double start, double delta, double candidate) {
    const double magnitude = std::abs(delta);
    const double relative = delta >= 0.0 ? wrap_positive(candidate - start)
                                         : wrap_positive(start - candidate);
    return relative <= magnitude + 1e-12;
}
}  // namespace

SoftLimitManager::SoftLimitManager() {
    for (auto& limit : limits_) {
        limit.min_position_m = -1.0;
        limit.max_position_m = 1.0;
        limit.max_velocity_mps = 1.0;
        limit.max_acceleration_mps2 = 1.0;
        limit.max_braking_deceleration_mps2 = 1.0;
        limit.safety_margin_m = 0.0;
    }
}

SoftLimitManager::SoftLimitManager(const std::array<AxisSoftLimit, 3>& limits)
    : limits_(limits) {}

bool SoftLimitManager::valid_limit(const AxisSoftLimit& limit) const noexcept {
    return std::isfinite(limit.min_position_m) && std::isfinite(limit.max_position_m) &&
           std::isfinite(limit.max_velocity_mps) && std::isfinite(limit.max_acceleration_mps2) &&
           std::isfinite(limit.max_braking_deceleration_mps2) &&
           std::isfinite(limit.safety_margin_m) && limit.min_position_m <= limit.max_position_m &&
           limit.max_velocity_mps >= 0.0 && limit.max_acceleration_mps2 >= 0.0 &&
           limit.max_braking_deceleration_mps2 > 0.0 && limit.safety_margin_m >= 0.0 &&
           limit.min_position_m + limit.safety_margin_m <= limit.max_position_m - limit.safety_margin_m;
}

bool SoftLimitManager::set_axis_limit(std::size_t axis, const AxisSoftLimit& limit) noexcept {
    if (axis >= limits_.size() || !valid_limit(limit)) return false;
    limits_[axis] = limit;
    return true;
}

LimitCheckResult SoftLimitManager::check_position(const multiaxis::Vector3& position_m) const {
    std::array<multiaxis::Vector3, 8> points{};
    points[0] = position_m;
    return check_points(points, 1);
}

LimitCheckResult SoftLimitManager::check_points(const std::array<multiaxis::Vector3, 8>& points,
                                                std::size_t count) const {
    LimitCheckResult result{};
    result.valid = true;
    result.closest_margin_m = {std::numeric_limits<double>::infinity(),
                               std::numeric_limits<double>::infinity(),
                               std::numeric_limits<double>::infinity()};
    if (count == 0 || count > points.size()) {
        result.valid = false; result.reason = "no points to check"; return result;
    }
    for (std::size_t p = 0; p < count; ++p) {
        if (!points[p].is_finite()) {
            result.valid = false; result.reason = "non-finite trajectory point"; return result;
        }
        const double values[3] = {points[p].x, points[p].y, points[p].z};
        double* margins[3] = {&result.closest_margin_m.x, &result.closest_margin_m.y,
                              &result.closest_margin_m.z};
        for (std::size_t axis = 0; axis < 3; ++axis) {
            if (!valid_limit(limits_[axis])) {
                result.valid = false; result.reason = "invalid soft limit configuration"; return result;
            }
            const auto& limit = limits_[axis];
            const double lower = limit.min_position_m + limit.safety_margin_m;
            const double upper = limit.max_position_m - limit.safety_margin_m;
            const double margin = std::min(values[axis] - lower, upper - values[axis]);
            *margins[axis] = std::min(*margins[axis], margin);
            if (margin < -1e-12) {
                result.valid = false;
                result.reason = "trajectory exceeds software position limit";
            }
        }
    }
    return result;
}

LimitCheckResult SoftLimitManager::check_line(const multiaxis::Vector3& start_m,
                                              const multiaxis::Vector3& target_m) const {
    // 每个轴在线段上是线性函数，端点通过则整条线段通过。
    std::array<multiaxis::Vector3, 8> points{};
    points[0] = start_m; points[1] = target_m;
    return check_points(points, 2);
}

LimitCheckResult SoftLimitManager::check_arc(const multiaxis::Vector3& start_m,
                                             const multiaxis::Vector3& target_m,
                                             const multiaxis::Vector3& center_m,
                                             multiaxis::ArcDirection direction,
                                             bool full_circle) const {
    LimitCheckResult bad{};
    if (!start_m.is_finite() || !target_m.is_finite() || !center_m.is_finite()) {
        bad.reason = "non-finite arc geometry"; return bad;
    }
    const double r0 = std::hypot(start_m.x - center_m.x, start_m.y - center_m.y);
    const double r1 = std::hypot(target_m.x - center_m.x, target_m.y - center_m.y);
    if (!(r0 > 1e-12) || std::abs(r0 - r1) > 1e-9 || std::abs(start_m.z - center_m.z) > 1e-9 ||
        std::abs(target_m.z - center_m.z) > 1e-9 ||
        (direction != multiaxis::ArcDirection::CW && direction != multiaxis::ArcDirection::CCW)) {
        bad.reason = "invalid arc geometry"; return bad;
    }
    const double start = std::atan2(start_m.y - center_m.y, start_m.x - center_m.x);
    const double target = std::atan2(target_m.y - center_m.y, target_m.x - center_m.x);
    double delta = direction == multiaxis::ArcDirection::CCW ? wrap_positive(target - start)
                                                              : -wrap_positive(start - target);
    if (full_circle) delta = direction == multiaxis::ArcDirection::CCW ? kTwoPi : -kTwoPi;
    if (!full_circle && std::abs(delta) <= 1e-12) {
        // 相同起终点默认是零位移，不隐式当成整圆。
        std::array<multiaxis::Vector3, 8> points{}; points[0] = start_m;
        return check_points(points, 1);
    }
    std::array<multiaxis::Vector3, 8> points{};
    std::size_t count = 0;
    points[count++] = start_m; points[count++] = target_m;
    for (const double cardinal : {0.0, kPi / 2.0, kPi, 3.0 * kPi / 2.0}) {
        if (angle_on_arc(start, delta, cardinal)) {
            points[count++] = {center_m.x + r0 * std::cos(cardinal),
                               center_m.y + r0 * std::sin(cardinal), start_m.z};
        }
    }
    return check_points(points, count);
}

double SoftLimitManager::stop_distance(double velocity_mps, std::size_t axis,
                                       double update_delay_s) const noexcept {
    if (axis >= limits_.size() || !std::isfinite(velocity_mps) || !std::isfinite(update_delay_s) ||
        update_delay_s < 0.0) return std::numeric_limits<double>::infinity();
    const auto& limit = limits_[axis];
    if (!valid_limit(limit)) return std::numeric_limits<double>::infinity();
    const double speed = std::abs(velocity_mps);
    return speed * update_delay_s + speed * speed / (2.0 * limit.max_braking_deceleration_mps2);
}

bool SoftLimitManager::can_stop_before_limit(double position_m, double velocity_mps,
                                             std::size_t axis, double update_delay_s) const noexcept {
    if (axis >= limits_.size() || !std::isfinite(position_m) || !std::isfinite(velocity_mps)) return false;
    const auto& limit = limits_[axis];
    if (!valid_limit(limit)) return false;
    const double distance = stop_distance(velocity_mps, axis, update_delay_s);
    if (velocity_mps > 0.0) return position_m + distance <= limit.max_position_m - limit.safety_margin_m + 1e-12;
    if (velocity_mps < 0.0) return position_m - distance >= limit.min_position_m + limit.safety_margin_m - 1e-12;
    return position_m >= limit.min_position_m + limit.safety_margin_m - 1e-12 &&
           position_m <= limit.max_position_m - limit.safety_margin_m + 1e-12;
}

multiaxis::Vector3 SoftLimitManager::margins(const multiaxis::Vector3& position_m) const noexcept {
    const double p[3] = {position_m.x, position_m.y, position_m.z};
    double* out[3] = {nullptr, nullptr, nullptr};
    multiaxis::Vector3 result{}; out[0] = &result.x; out[1] = &result.y; out[2] = &result.z;
    for (std::size_t axis = 0; axis < 3; ++axis) {
        if (!std::isfinite(p[axis]) || !valid_limit(limits_[axis])) *out[axis] = -std::numeric_limits<double>::infinity();
        else *out[axis] = std::min(p[axis] - limits_[axis].min_position_m - limits_[axis].safety_margin_m,
                                   limits_[axis].max_position_m - limits_[axis].safety_margin_m - p[axis]);
    }
    return result;
}

}  // namespace motioncore::manager

#include "motioncore/multiaxis/arc_interpolator.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "motioncore/planning/s_curve_profile.hpp"
#include "motioncore/planning/trapezoidal_profile.hpp"

namespace motioncore::multiaxis {

namespace {
constexpr double kTwoPi = 2.0 * 3.141592653589793238462643383279502884;
constexpr double kGeometryTolerance = 1e-10;
constexpr double kSqrtTwo = 1.414213562373095048801688724209698079;

double positive_modulo(double value, double period) {
    double result = std::fmod(value, period);
    if (result < 0.0) result += period;
    return result;
}

// 圆弧的法向加速度和 Jerk 会随速度增加。为了避免先生成超限轨迹，
// 在构造标量规划器之前把切向约束保守地收紧。这样得到的轨迹仍然是
// 解析 S 曲线，只是可能比用户请求的轨迹更慢；若无法建立正约束，
// 构造函数会明确拒绝指令。
planning::MotionConstraints conservative_constraints(
    planning::MotionConstraints requested, const ArcResultantLimits& limits,
    double radius_m) {
    const bool acceleration_ok = std::isinf(limits.max_acceleration_mps2)
        || std::hypot(requested.max_acceleration_mps2,
                      requested.max_velocity_mps * requested.max_velocity_mps / radius_m)
            <= limits.max_acceleration_mps2 + 1e-12;
    const bool jerk_ok = std::isinf(limits.max_jerk_mps3)
        || std::hypot(requested.max_jerk_mps3
                          + requested.max_velocity_mps * requested.max_velocity_mps
                              * requested.max_velocity_mps / (radius_m * radius_m),
                      3.0 * requested.max_velocity_mps
                          * requested.max_acceleration_mps2 / radius_m)
            <= limits.max_jerk_mps3 + 1e-12;
    if (acceleration_ok && jerk_ok) return requested;

    if (std::isfinite(limits.max_acceleration_mps2)) {
        const double component_limit = limits.max_acceleration_mps2 / kSqrtTwo;
        requested.max_acceleration_mps2 =
            std::min(requested.max_acceleration_mps2, component_limit);
        requested.max_velocity_mps = std::min(
            requested.max_velocity_mps, std::sqrt(component_limit * radius_m));
    }
    if (std::isfinite(limits.max_jerk_mps3)) {
        const double component_limit = limits.max_jerk_mps3 / kSqrtTwo;
        // |j - v^3/R^2| <= |j| + v^3/R^2.  Allocate half of the
        // tangent budget to each term and the other half to 3*v*a/R.
        const double tangent_budget = limits.max_jerk_mps3 / (2.0 * kSqrtTwo);
        requested.max_jerk_mps3 = std::min(requested.max_jerk_mps3, tangent_budget);
        requested.max_velocity_mps = std::min(
            requested.max_velocity_mps,
            std::cbrt(tangent_budget * radius_m * radius_m));
        if (requested.max_velocity_mps > 0.0) {
            requested.max_acceleration_mps2 = std::min(
                requested.max_acceleration_mps2,
                component_limit * radius_m
                    / (3.0 * requested.max_velocity_mps));
        }
    }
    if (!std::isfinite(requested.max_velocity_mps)
        || !std::isfinite(requested.max_acceleration_mps2)
        || !std::isfinite(requested.max_jerk_mps3)
        || requested.max_velocity_mps <= 0.0
        || requested.max_acceleration_mps2 <= 0.0
        || requested.max_jerk_mps3 <= 0.0) {
        throw std::invalid_argument("arc resultant limits leave no positive scalar constraints");
    }
    return requested;
}
}

ArcInterpolator::ArcInterpolator(
    const Vector3& start_m, const Vector3& target_m, const Vector3& center_m,
    ArcDirection direction, const planning::MotionConstraints& constraints,
    PlanningMode mode, bool full_circle, ArcResultantLimits resultant_limits)
    : start_m_(start_m), target_m_(target_m), center_m_(center_m),
      direction_(direction), mode_(mode), full_circle_(full_circle),
      resultant_limits_(resultant_limits) {
    if (!start_m_.is_finite() || !target_m_.is_finite() || !center_m_.is_finite()) {
        throw std::invalid_argument("arc points must be finite");
    }
    if (std::isnan(resultant_limits_.max_acceleration_mps2)
        || resultant_limits_.max_acceleration_mps2 <= 0.0
        || std::isnan(resultant_limits_.max_jerk_mps3)
        || resultant_limits_.max_jerk_mps3 <= 0.0) {
        throw std::invalid_argument("arc resultant limits must be positive or infinity");
    }
    if (std::abs(start_m_.z - center_m_.z) > kGeometryTolerance
        || std::abs(target_m_.z - center_m_.z) > kGeometryTolerance
        || std::abs(start_m_.z - target_m_.z) > kGeometryTolerance) {
        throw std::invalid_argument("only XY plane arcs with constant Z are supported");
    }

    const double start_radius = (start_m_ - center_m_).norm();
    const double target_radius = (target_m_ - center_m_).norm();
    if (!std::isfinite(start_radius) || start_radius <= kGeometryTolerance) {
        throw std::invalid_argument("arc radius must be finite and positive");
    }
    const double radius_tolerance = std::max(kGeometryTolerance, start_radius * 1e-8);
    if (std::abs(start_radius - target_radius) > radius_tolerance) {
        throw std::invalid_argument("arc endpoints must have the same radius");
    }
    radius_m_ = start_radius;
    start_angle_rad_ = std::atan2(start_m_.y - center_m_.y, start_m_.x - center_m_.x);
    const double target_angle = std::atan2(target_m_.y - center_m_.y,
                                           target_m_.x - center_m_.x);
    const double raw_delta = target_angle - start_angle_rad_;
    const double positive_delta = positive_modulo(raw_delta, kTwoPi);
    if (direction_ == ArcDirection::CCW) {
        delta_angle_rad_ = positive_delta;
    } else if (direction_ == ArcDirection::CW) {
        delta_angle_rad_ = positive_delta == 0.0 ? 0.0 : positive_delta - kTwoPi;
    } else {
        throw std::invalid_argument("invalid arc direction");
    }
    if (full_circle && std::abs(delta_angle_rad_) <= kGeometryTolerance) {
        delta_angle_rad_ = direction_ == ArcDirection::CCW ? kTwoPi : -kTwoPi;
    }
    arc_length_m_ = radius_m_ * std::abs(delta_angle_rad_);

    scalar_constraints_ = conservative_constraints(
        constraints, resultant_limits_, radius_m_);
    const auto request = make_scalar_request(arc_length_m_, scalar_constraints_);
    if (mode_ == PlanningMode::Trapezoid) {
        profile_ = std::make_shared<planning::TrapezoidalProfile>(request);
    } else {
        profile_ = std::make_shared<planning::SCurveProfile>(request);
    }
    validate_and_build(constraints, mode, full_circle);
}

void ArcInterpolator::validate_and_build(
    const planning::MotionConstraints&, PlanningMode, bool) {
    // 约束检查使用每个解析分段的边界、加速度为零的速度极值以及
    // 合成加速度的一阶导数候选点。它不依赖固定数量的离散采样来宣称
    // 全局满足约束；采样网格仅用于用户导出的曲线，不参与本判定。
    resultant_limits_satisfied_ = true;
    if (std::isfinite(resultant_limits_.max_jerk_mps3)
        && mode_ == PlanningMode::Trapezoid) {
        // A trapezoid has instantaneous acceleration changes.  Its jerk is
        // deliberately represented as NaN by Phase 3, so it cannot claim to
        // satisfy a finite jerk limit.
        throw std::invalid_argument("trapezoidal arc has discontinuous jerk and cannot satisfy a finite jerk limit");
    }
    if (std::isinf(resultant_limits_.max_acceleration_mps2)
        && std::isinf(resultant_limits_.max_jerk_mps3)) {
        return;
    }
    const double duration_s = profile_->duration();
    std::vector<double> candidates{0.0, duration_s};
    const auto add_profile_candidates = [&](const auto& profile) {
        for (const auto& segment : profile.segments()) {
            const double begin = segment.start_time_s;
            const double end = begin + segment.duration_s;
            candidates.push_back(begin);
            candidates.push_back(end);
            // a(t)=a0+j*t, hence this is the only interior velocity extremum.
            if (std::abs(segment.jerk_mps3) > 1e-15) {
                const double local = -segment.acceleration_mps2 / segment.jerk_mps3;
                if (local > 0.0 && local < segment.duration_s) {
                    candidates.push_back(begin + local);
                }
            }
        }
    };
    if (const auto* profile = dynamic_cast<const planning::SCurveProfile*>(profile_.get())) {
        add_profile_candidates(*profile);
    } else if (const auto* profile = dynamic_cast<const planning::TrapezoidalProfile*>(profile_.get())) {
        add_profile_candidates(*profile);
    }

    // For |A|, d(A^2)/dt = 2*a*(j + 2*v^3/R^2).  Add roots of the
    // second factor by bisection inside each constant-jerk segment.
    const auto add_acceleration_roots = [&](const auto& profile) {
        for (const auto& segment : profile.segments()) {
            const double begin = segment.start_time_s;
            const auto root_function = [&](double local) {
                const double velocity = segment.velocity_mps
                    + segment.acceleration_mps2 * local
                    + 0.5 * segment.jerk_mps3 * local * local;
                return segment.jerk_mps3 + 2.0 * velocity * velocity * velocity
                    / (radius_m_ * radius_m_);
            };
            double left = 0.0;
            double right = segment.duration_s;
            double f_left = root_function(left);
            double f_right = root_function(right);
            if (std::isfinite(f_left) && std::isfinite(f_right)
                && f_left * f_right < 0.0) {
                for (int iteration = 0; iteration < 80; ++iteration) {
                    const double mid = 0.5 * (left + right);
                    const double f_mid = root_function(mid);
                    if (f_left * f_mid <= 0.0) {
                        right = mid;
                        f_right = f_mid;
                    } else {
                        left = mid;
                        f_left = f_mid;
                    }
                }
                candidates.push_back(begin + 0.5 * (left + right));
            }
        }
    };
    if (const auto* profile = dynamic_cast<const planning::SCurveProfile*>(profile_.get())) {
        add_acceleration_roots(*profile);
    } else if (const auto* profile = dynamic_cast<const planning::TrapezoidalProfile*>(profile_.get())) {
        add_acceleration_roots(*profile);
    }

    std::sort(candidates.begin(), candidates.end());
    candidates.erase(std::unique(candidates.begin(), candidates.end()), candidates.end());
    for (const double t : candidates) {
        const auto scalar = profile_->sample(t);
        const double v = scalar.velocity_mps;
        const double a = scalar.acceleration_mps2;
        const double normal_a = v * v / radius_m_;
        const double resultant_a = std::hypot(a, normal_a);
        if (std::isfinite(resultant_limits_.max_acceleration_mps2)
            && resultant_a > resultant_limits_.max_acceleration_mps2 + 1e-10) {
            resultant_limits_satisfied_ = false;
            break;
        }
        if (std::isfinite(resultant_limits_.max_jerk_mps3)
            && std::isfinite(scalar.jerk_mps3)) {
            const double tangent_j = scalar.jerk_mps3 - v * v * v / (radius_m_ * radius_m_);
            const double normal_j = 3.0 * v * a / radius_m_;
            if (std::hypot(tangent_j, normal_j)
                    > resultant_limits_.max_jerk_mps3 + 1e-8) {
                resultant_limits_satisfied_ = false;
                break;
            }
        }
    }
    // The exact jerk extrema are polynomial roots.  A segment-wise bound is
    // stronger than a grid check and keeps the proof simple: v and |a| are
    // bounded by their analytic segment extrema, while each term in the
    // Frenet jerk formula is bounded independently.
    if (resultant_limits_satisfied_ && std::isfinite(resultant_limits_.max_jerk_mps3)) {
        const auto check_jerk_bounds = [&](const auto& profile) {
            for (const auto& segment : profile.segments()) {
                const double begin = segment.start_time_s;
                const double end = begin + segment.duration_s;
                double max_v = 0.0;
                double max_a = 0.0;
                const std::array<double, 3> times{
                    begin, end,
                    begin + std::clamp(-segment.acceleration_mps2
                                       / (std::abs(segment.jerk_mps3) > 1e-15
                                              ? segment.jerk_mps3 : 1.0),
                                       0.0, segment.duration_s)};
                for (const double time : times) {
                    const auto value = profile_->sample(time);
                    max_v = std::max(max_v, std::abs(value.velocity_mps));
                    max_a = std::max(max_a, std::abs(value.acceleration_mps2));
                }
                const double tangent_bound = std::abs(segment.jerk_mps3)
                    + max_v * max_v * max_v / (radius_m_ * radius_m_);
                const double normal_bound = 3.0 * max_v * max_a / radius_m_;
                if (std::hypot(tangent_bound, normal_bound)
                        > resultant_limits_.max_jerk_mps3 + 1e-8) {
                    return false;
                }
            }
            return true;
        };
        bool jerk_ok = true;
        if (const auto* profile = dynamic_cast<const planning::SCurveProfile*>(profile_.get())) {
            jerk_ok = check_jerk_bounds(*profile);
        } else if (const auto* profile = dynamic_cast<const planning::TrapezoidalProfile*>(profile_.get())) {
            jerk_ok = check_jerk_bounds(*profile);
        }
        resultant_limits_satisfied_ = resultant_limits_satisfied_ && jerk_ok;
    }
    if (!resultant_limits_satisfied_) {
        throw std::invalid_argument("arc trajectory violates resultant acceleration or jerk limit");
    }
}

double ArcInterpolator::normalized_angle_delta() const { return delta_angle_rad_; }

MultiAxisTrajectoryState ArcInterpolator::sample(double time_s) const {
    const planning::TrajectoryState scalar = profile_->sample(time_s);
    MultiAxisTrajectoryState state;
    state.time_s = scalar.time_s;
    state.path_progress_m = scalar.position_m;
    if (state.time_s >= duration()) {
        state.position = target_m_;
        state.velocity = {};
        state.acceleration = {};
        state.jerk = {};
        state.position_m = state.position;
        state.velocity_mps = state.velocity;
        state.acceleration_mps2 = state.acceleration;
        state.jerk_mps3 = state.jerk;
        state.path_progress_m = arc_length_m_;
        return state;
    }
    const double sign = delta_angle_rad_ < 0.0 ? -1.0 : 1.0;
    const double theta = start_angle_rad_ + sign * scalar.position_m / radius_m_;
    const Vector3 radial{std::cos(theta), std::sin(theta), 0.0};
    const Vector3 tangent{-sign * std::sin(theta), sign * std::cos(theta), 0.0};
    state.position = center_m_ + radius_m_ * radial;
    state.position.z = start_m_.z;
    state.velocity = tangent * scalar.velocity_mps;
    state.acceleration = tangent * scalar.acceleration_mps2
        - radial * (scalar.velocity_mps * scalar.velocity_mps / radius_m_);
    if (std::isfinite(scalar.jerk_mps3)) {
        const double tangent_j = scalar.jerk_mps3
            - scalar.velocity_mps * scalar.velocity_mps * scalar.velocity_mps
                / (radius_m_ * radius_m_);
        const double normal_j = 3.0 * scalar.velocity_mps * scalar.acceleration_mps2 / radius_m_;
        state.jerk = tangent * tangent_j - radial * normal_j;
    } else {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        state.jerk = {nan, nan, nan};
    }
    state.position_m = state.position;
    state.velocity_mps = state.velocity;
    state.acceleration_mps2 = state.acceleration;
    state.jerk_mps3 = state.jerk;
    return state;
}

double ArcInterpolator::duration() const noexcept { return profile_->duration(); }

ArcKinematicMetrics ArcInterpolator::kinematics(double time_s) const {
    const auto scalar = profile_->sample(time_s);
    ArcKinematicMetrics result;
    result.tangential_acceleration_mps2 = scalar.acceleration_mps2;
    result.normal_acceleration_mps2 = scalar.velocity_mps * scalar.velocity_mps / radius_m_;
    result.resultant_acceleration_mps2 = std::hypot(
        result.tangential_acceleration_mps2, result.normal_acceleration_mps2);
    if (std::isfinite(scalar.jerk_mps3)) {
        result.tangential_jerk_mps3 = scalar.jerk_mps3
            - scalar.velocity_mps * scalar.velocity_mps * scalar.velocity_mps
                / (radius_m_ * radius_m_);
        result.normal_jerk_mps3 = 3.0 * scalar.velocity_mps
            * scalar.acceleration_mps2 / radius_m_;
        result.resultant_jerk_mps3 = std::hypot(
            result.tangential_jerk_mps3, result.normal_jerk_mps3);
    } else {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        result.tangential_jerk_mps3 = nan;
        result.normal_jerk_mps3 = nan;
        result.resultant_jerk_mps3 = nan;
    }
    return result;
}

}  // namespace motioncore::multiaxis

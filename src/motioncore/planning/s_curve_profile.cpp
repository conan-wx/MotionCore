#include "motioncore/planning/s_curve_profile.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace motioncore::planning {

void validate_trajectory_request(const TrajectoryRequest&, bool);
double clamp_trajectory_time(double, double);

namespace {

struct ScurveParameters {
    double tj{0.0};
    double ta{0.0};
    double tv{0.0};
    double peak_acceleration{0.0};
    double peak_velocity{0.0};
};

// Distance travelled by one (positive) acceleration half of a symmetric profile.
double acceleration_distance(double acceleration, double tj, double ta) {
    return acceleration * (0.5 * ta * ta + 1.5 * tj * ta + tj * tj);
}

// Find the shortest S-curve that reaches a specified velocity under A/J limits.
// The result has no cruise segment; adding a cruise later is exact because the
// acceleration and deceleration halves are mirror images.
ScurveParameters parameters_for_peak_velocity(double velocity,
                                               double amax,
                                               double jmax) {
    ScurveParameters p;
    const double velocity_at_jerk_limit = amax * amax / jmax;
    if (velocity <= velocity_at_jerk_limit) {
        p.tj = std::sqrt(velocity / jmax);
        p.ta = 0.0;
        p.peak_acceleration = jmax * p.tj;
    } else {
        p.tj = amax / jmax;
        p.ta = velocity / amax - p.tj;
        p.peak_acceleration = amax;
    }
    p.peak_velocity = velocity;
    p.tv = 0.0;
    return p;
}

}  // namespace

SCurveProfile::SCurveProfile(const TrajectoryRequest& request)
    : request_(request) {
    validate_trajectory_request(request_, true);
    const double displacement = request_.target_position_m - request_.start_position_m;
    distance_m_ = std::abs(displacement);
    direction_ = displacement < 0.0 ? -1.0 : 1.0;
    if (distance_m_ == 0.0) {
        return;
    }

    const double vmax = request_.constraints.max_velocity_mps;
    const double amax = request_.constraints.max_acceleration_mps2;
    const double jmax = request_.constraints.max_jerk_mps3;

    // 先判断能否达到最大速度；若可以，剩余距离自然由中间匀速段承担。
    ScurveParameters velocity_limited = parameters_for_peak_velocity(vmax, amax, jmax);
    const double distance_to_reach_vmax = 2.0 * acceleration_distance(
        velocity_limited.peak_acceleration, velocity_limited.tj, velocity_limited.ta);
    ScurveParameters p;
    if (distance_m_ >= distance_to_reach_vmax - 1e-15) {
        p = velocity_limited;
        p.tv = (distance_m_ - distance_to_reach_vmax) / vmax;
        // Avoid a tiny negative value from a distance exactly on the boundary.
        if (p.tv < 1e-14) {
            p.tv = 0.0;
        }
    } else {
        // 尝试使用最大加速度。由
        // D/2 = A*(0.5*ta^2 + 1.5*tj*ta + tj^2)
        // 解出正根；若 ta=0 仍超过目标距离，就退化为四段 Jerk 轨迹。
        const double tj_at_amax = amax / jmax;
        const double distance_at_amax_without_plateau = 2.0 *
            acceleration_distance(amax, tj_at_amax, 0.0);
        if (distance_m_ >= distance_at_amax_without_plateau - 1e-15) {
            p.tj = tj_at_amax;
            const double discriminant = tj_at_amax * tj_at_amax
                + 4.0 * distance_m_ / amax;
            p.ta = 0.5 * (-3.0 * tj_at_amax + std::sqrt(discriminant));
            p.ta = std::max(0.0, p.ta);
            p.tv = 0.0;
            p.peak_acceleration = amax;
            p.peak_velocity = amax * (p.ta + p.tj);
        } else {
            p.tj = std::cbrt(distance_m_ / (2.0 * jmax));
            p.ta = 0.0;
            p.tv = 0.0;
            p.peak_acceleration = jmax * p.tj;
            p.peak_velocity = jmax * p.tj * p.tj;
        }
    }

    peak_velocity_mps_ = p.peak_velocity;
    peak_acceleration_mps2_ = p.peak_acceleration;
    total_time_s_ = 2.0 * (2.0 * p.tj + p.ta) + p.tv;

    // 按七段顺序建立轨迹，零时长段会被跳过。每段保存起始 x/v/a，
    // sample() 使用常 Jerk 的三次解析式直接计算，不依赖逐步积分。
    auto append = [&](double duration, double jerk) {
        if (duration <= 0.0) {
            return;
        }
        ProfileSegment segment;
        segment.start_time_s = segments_.empty()
            ? 0.0
            : segments_.back().start_time_s + segments_.back().duration_s;
        segment.duration_s = duration;
        segment.jerk_mps3 = direction_ * jerk;
        if (segments_.empty()) {
            segment.position_m = request_.start_position_m;
            segment.velocity_mps = 0.0;
            segment.acceleration_mps2 = 0.0;
        } else {
            const ProfileSegment& previous = segments_.back();
            const double u = previous.duration_s;
            segment.position_m = previous.position_m
                + previous.velocity_mps * u
                + 0.5 * previous.acceleration_mps2 * u * u
                + previous.jerk_mps3 * u * u * u / 6.0;
            segment.velocity_mps = previous.velocity_mps
                + previous.acceleration_mps2 * u
                + 0.5 * previous.jerk_mps3 * u * u;
            segment.acceleration_mps2 = previous.acceleration_mps2
                + previous.jerk_mps3 * u;
        }
        segments_.push_back(segment);
    };

    append(p.tj, jmax);
    append(p.ta, 0.0);
    append(p.tj, -jmax);
    append(p.tv, 0.0);
    append(p.tj, -jmax);
    append(p.ta, 0.0);
    append(p.tj, jmax);
}

TrajectoryState SCurveProfile::sample(double time_s) const {
    const double t = clamp_trajectory_time(time_s, total_time_s_);
    if (distance_m_ == 0.0) {
        return {t, request_.start_position_m, 0.0, 0.0, 0.0};
    }
    if (t >= total_time_s_) {
        return {total_time_s_, request_.target_position_m, 0.0, 0.0, 0.0};
    }

    const ProfileSegment* selected = &segments_.back();
    for (const auto& segment : segments_) {
        const double end = segment.start_time_s + segment.duration_s;
        if (t < end || &segment == &segments_.back()) {
            selected = &segment;
            break;
        }
    }
    const double u = t - selected->start_time_s;
    const double j = selected->jerk_mps3;
    const double x = selected->position_m
        + selected->velocity_mps * u
        + 0.5 * selected->acceleration_mps2 * u * u
        + j * u * u * u / 6.0;
    const double v = selected->velocity_mps
        + selected->acceleration_mps2 * u
        + 0.5 * j * u * u;
    const double a = selected->acceleration_mps2 + j * u;
    return {t, x, v, a, j};
}

}  // namespace motioncore::planning

#include "motioncore/planning/trapezoidal_profile.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace motioncore::planning {

// These helpers live in trajectory_profile.cpp but are intentionally kept out of
// the public header: validation is an implementation detail of the profiles.
void validate_trajectory_request(const TrajectoryRequest&, bool);
double clamp_trajectory_time(double, double);

TrapezoidalProfile::TrapezoidalProfile(const TrajectoryRequest& request)
    : request_(request) {
    // 统一请求仍要求给出有效 Jerk 约束，虽然梯形轨迹不会使用它，
    // 这样可以尽早发现调用方传入的非法约束。
    validate_trajectory_request(request_, true);
    const double displacement = request_.target_position_m - request_.start_position_m;
    distance_m_ = std::abs(displacement);
    direction_ = displacement < 0.0 ? -1.0 : 1.0;
    if (distance_m_ == 0.0) {
        return;
    }

    const double vmax = request_.constraints.max_velocity_mps;
    const double amax = request_.constraints.max_acceleration_mps2;
    const double minimum_distance_for_vmax = vmax * vmax / amax;
    if (distance_m_ >= minimum_distance_for_vmax) {
        peak_velocity_mps_ = vmax;
        acceleration_time_s_ = vmax / amax;
        cruise_time_s_ = (distance_m_ - minimum_distance_for_vmax) / vmax;
        has_cruise_ = cruise_time_s_ > 1e-14;
    } else {
    // 短距离无法达到 vmax：加减速时间相同，峰值速度为 sqrt(a*d)。
        peak_velocity_mps_ = std::sqrt(amax * distance_m_);
        acceleration_time_s_ = peak_velocity_mps_ / amax;
        cruise_time_s_ = 0.0;
        has_cruise_ = false;
    }
    total_time_s_ = 2.0 * acceleration_time_s_ + cruise_time_s_;

    // 段状态用常加速度解析式预先计算，sample() 不做逐步积分，
    // 因此不会因为 1 ms 采样而累计位置误差。
    const double nan_jerk = std::numeric_limits<double>::quiet_NaN();
    auto append = [&](double duration, double acceleration) {
        if (duration <= 0.0) {
            return;
        }
        ProfileSegment segment;
        segment.start_time_s = segments_.empty()
            ? 0.0
            : segments_.back().start_time_s + segments_.back().duration_s;
        segment.duration_s = duration;
        segment.jerk_mps3 = nan_jerk;
        if (segments_.empty()) {
            segment.position_m = request_.start_position_m;
            segment.velocity_mps = 0.0;
            segment.acceleration_mps2 = 0.0;
        } else {
            const ProfileSegment& previous = segments_.back();
            const double t = previous.duration_s;
            segment.position_m = previous.position_m
                + previous.velocity_mps * t
                + 0.5 * previous.acceleration_mps2 * t * t;
            segment.velocity_mps = previous.velocity_mps
                + previous.acceleration_mps2 * t;
            segment.acceleration_mps2 = acceleration;
        }
        // The first segment starts with the requested acceleration.
        if (segments_.empty()) {
            segment.acceleration_mps2 = acceleration;
        }
        segments_.push_back(segment);
    };
    append(acceleration_time_s_, direction_ * amax);
    append(cruise_time_s_, 0.0);
    append(acceleration_time_s_, -direction_ * amax);
}

TrajectoryState TrapezoidalProfile::sample(double time_s) const {
    const double t = clamp_trajectory_time(time_s, total_time_s_);
    if (distance_m_ == 0.0) {
        return {t, request_.start_position_m, 0.0, 0.0,
                std::numeric_limits<double>::quiet_NaN()};
    }

    // At the exact end, force the requested target to remove round-off drift.
    if (t >= total_time_s_) {
        return {total_time_s_, request_.target_position_m, 0.0, 0.0,
                std::numeric_limits<double>::quiet_NaN()};
    }

    const double a = direction_ * request_.constraints.max_acceleration_mps2;
    const double ta = acceleration_time_s_;
    const double tc = cruise_time_s_;
    double x = request_.start_position_m;
    double v = 0.0;
    double acceleration = a;
    if (t < ta) {
        x += 0.5 * a * t * t;
        v = a * t;
    } else if (t < ta + tc) {
        const double x_acc = 0.5 * a * ta * ta;
        x += x_acc + direction_ * peak_velocity_mps_ * (t - ta);
        v = direction_ * peak_velocity_mps_;
        acceleration = 0.0;
    } else {
        const double tau = t - ta - tc;
        const double x_acc = 0.5 * a * ta * ta;
        const double x_cruise = direction_ * peak_velocity_mps_ * tc;
        x += x_acc + x_cruise
            + direction_ * peak_velocity_mps_ * tau - 0.5 * a * tau * tau;
        v = direction_ * peak_velocity_mps_ - a * tau;
        acceleration = -a;
    }
    return {t, x, v, acceleration,
            std::numeric_limits<double>::quiet_NaN()};
}

}  // namespace motioncore::planning

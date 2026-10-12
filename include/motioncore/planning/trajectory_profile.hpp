#pragma once

#include <cstddef>
#include <memory>
#include <vector>

namespace motioncore::planning {

// 轨迹状态统一使用 SI 单位：位置 m、速度 m/s、加速度 m/s^2、Jerk m/s^3。
// 梯形速度轨迹没有有限 Jerk 约束，因此其 jerk_mps3 用 NaN 表示“理论上不连续”。
struct TrajectoryState {
    double time_s{0.0};
    double position_m{0.0};
    double velocity_mps{0.0};
    double acceleration_mps2{0.0};
    double jerk_mps3{0.0};
};

struct MotionConstraints {
    double max_velocity_mps{0.0};
    double max_acceleration_mps2{0.0};
    double max_jerk_mps3{0.0};
};

struct TrajectoryRequest {
    double start_position_m{0.0};
    double target_position_m{0.0};
    double start_velocity_mps{0.0};
    double end_velocity_mps{0.0};
    MotionConstraints constraints{};
};

// 两种规划器都实现这个很小的接口，便于采样器和 Phase 3 程序统一处理。
class TrajectoryProfile {
public:
    virtual ~TrajectoryProfile() = default;
    [[nodiscard]] virtual TrajectoryState sample(double time_s) const = 0;
    [[nodiscard]] virtual double duration() const noexcept = 0;
};

// 采样器将“任意时间查询”统一成同一个接口。profile 的生命周期必须覆盖 sampler。
class TrajectorySampler {
public:
    explicit TrajectorySampler(const TrajectoryProfile& profile) noexcept
        : profile_(&profile) {}

    [[nodiscard]] TrajectoryState sample(double time_s) const;
    [[nodiscard]] double duration() const noexcept { return profile_->duration(); }

private:
    const TrajectoryProfile* profile_;
};

// 暴露给文档、测试和可视化的一个解析阶段。
struct ProfileSegment {
    double start_time_s{0.0};
    double duration_s{0.0};
    double jerk_mps3{0.0};
    double position_m{0.0};
    double velocity_mps{0.0};
    double acceleration_mps2{0.0};
};

}  // namespace motioncore::planning

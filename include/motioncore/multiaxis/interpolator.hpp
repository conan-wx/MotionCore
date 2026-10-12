#pragma once

#include "motioncore/planning/trajectory_profile.hpp"
#include "motioncore/multiaxis/vector3.hpp"

namespace motioncore::multiaxis {

struct MultiAxisTrajectoryState {
    double time_s{0.0};
    // Short field names are convenient in geometry code.  The *_m variants
    // below are kept as explicit SI aliases for callers that prefer units in
    // names; interpolators populate both sets together.
    Vector3 position{};
    Vector3 velocity{};
    Vector3 acceleration{};
    Vector3 jerk{};
    double path_progress_m{0.0};
    // Explicit SI aliases are placed after the canonical six fields so the
    // common aggregate form {time, position, velocity, acceleration, jerk,
    // progress} remains source-compatible.
    Vector3 position_m{};
    Vector3 velocity_mps{};
    Vector3 acceleration_mps2{};
    Vector3 jerk_mps3{};
};

enum class PlanningMode { Trapezoid, SCurve };

// 所有几何插补器都实现这个小接口，便于 MotionSegment 统一调度。
class Interpolator {
public:
    virtual ~Interpolator() = default;
    [[nodiscard]] virtual MultiAxisTrajectoryState sample(double time_s) const = 0;
    [[nodiscard]] virtual double duration() const noexcept = 0;
    [[nodiscard]] virtual double path_length_m() const noexcept = 0;
};

// 将 Phase 4 的输入转换成 Phase 3 的标量规划请求。
planning::TrajectoryRequest make_scalar_request(double length_m,
                                                const planning::MotionConstraints& constraints);

}  // namespace motioncore::multiaxis

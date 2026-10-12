#pragma once

#include <variant>
#include <vector>
#include <utility>

#include "motioncore/multiaxis/arc_interpolator.hpp"
#include "motioncore/multiaxis/line_interpolator.hpp"

namespace motioncore::multiaxis {

enum class MotionSegmentType { Line, Arc };

// 拐角处采用“上一段减速到零，再进入下一段”的保守策略。
struct MotionSegment {
    using Geometry = std::variant<LineInterpolator, ArcInterpolator>;
    Geometry geometry;
    double start_time_s{0.0};

    explicit MotionSegment(const LineInterpolator& line) : geometry(line) {}
    explicit MotionSegment(const ArcInterpolator& arc) : geometry(arc) {}
    explicit MotionSegment(LineInterpolator&& line) : geometry(std::move(line)) {}
    explicit MotionSegment(ArcInterpolator&& arc) : geometry(std::move(arc)) {}

    [[nodiscard]] MotionSegmentType type() const noexcept {
        return std::holds_alternative<LineInterpolator>(geometry)
            ? MotionSegmentType::Line : MotionSegmentType::Arc;
    }
    [[nodiscard]] double duration() const noexcept {
        return std::visit([](const auto& value) { return value.duration(); }, geometry);
    }
    [[nodiscard]] double path_length_m() const noexcept {
        return std::visit([](const auto& value) { return value.path_length_m(); }, geometry);
    }
    [[nodiscard]] Vector3 start_m() const noexcept {
        return std::visit([](const auto& value) { return value.start_m(); }, geometry);
    }
    [[nodiscard]] Vector3 end_m() const noexcept {
        return std::visit([](const auto& value) { return value.target_m(); }, geometry);
    }
    [[nodiscard]] MultiAxisTrajectoryState sample(double local_time_s) const {
        return std::visit([&](const auto& value) { return value.sample(local_time_s); }, geometry);
    }
};

class CoordinatedTrajectoryPlanner {
public:
    CoordinatedTrajectoryPlanner() = default;
    explicit CoordinatedTrajectoryPlanner(std::vector<MotionSegment> segments);

    void clear() noexcept;
    void add_line(const LineInterpolator& line);
    void add_arc(const ArcInterpolator& arc);
    [[nodiscard]] MultiAxisTrajectoryState sample(double time_s) const;
    [[nodiscard]] double duration() const noexcept { return total_duration_s_; }
    [[nodiscard]] const std::vector<MotionSegment>& segments() const noexcept { return segments_; }

private:
    void rebuild_timeline();
    std::vector<MotionSegment> segments_;
    double total_duration_s_{0.0};
};

}  // namespace motioncore::multiaxis

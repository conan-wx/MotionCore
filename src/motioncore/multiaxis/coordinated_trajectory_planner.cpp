#include "motioncore/multiaxis/coordinated_trajectory_planner.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace motioncore::multiaxis {

CoordinatedTrajectoryPlanner::CoordinatedTrajectoryPlanner(std::vector<MotionSegment> segments)
    : segments_(std::move(segments)) { rebuild_timeline(); }

void CoordinatedTrajectoryPlanner::clear() noexcept {
    segments_.clear();
    total_duration_s_ = 0.0;
}

void CoordinatedTrajectoryPlanner::add_line(const LineInterpolator& line) {
    segments_.emplace_back(line);
    rebuild_timeline();
}

void CoordinatedTrajectoryPlanner::add_arc(const ArcInterpolator& arc) {
    segments_.emplace_back(arc);
    rebuild_timeline();
}

void CoordinatedTrajectoryPlanner::rebuild_timeline() {
    double time = 0.0;
    bool has_previous = false;
    Vector3 previous_end{};
    for (auto& segment : segments_) {
        if (has_previous && (segment.start_m() - previous_end).norm() > 1e-9) {
            throw std::invalid_argument("motion segments must be geometrically contiguous");
        }
        segment.start_time_s = time;
        time += segment.duration();
        previous_end = segment.end_m();
        has_previous = true;
    }
    total_duration_s_ = time;
}

MultiAxisTrajectoryState CoordinatedTrajectoryPlanner::sample(double time_s) const {
    if (!std::isfinite(time_s)) {
        throw std::invalid_argument("planner sample time must be finite");
    }
    if (segments_.empty()) return {};
    const double t = std::max(0.0, std::min(time_s, total_duration_s_));
    const MotionSegment* selected = &segments_.back();
    for (const auto& segment : segments_) {
        if (t < segment.start_time_s + segment.duration() || &segment == &segments_.back()) {
            selected = &segment;
            break;
        }
    }
    const double local_time = std::max(0.0, t - selected->start_time_s);
    auto state = selected->sample(local_time);
    state.time_s = t;
    double prior_path = 0.0;
    for (const auto& segment : segments_) {
        if (&segment == selected) break;
        prior_path += segment.path_length_m();
    }
    state.path_progress_m += prior_path;
    return state;
}

}  // namespace motioncore::multiaxis

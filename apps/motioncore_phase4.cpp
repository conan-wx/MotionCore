#include "motioncore/multiaxis/arc_interpolator.hpp"
#include "motioncore/multiaxis/coordinated_trajectory_planner.hpp"
#include "motioncore/multiaxis/line_interpolator.hpp"
#include "motioncore/multiaxis/multi_axis_servo_system.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

namespace {
using motioncore::multiaxis::ArcDirection;
using motioncore::multiaxis::ArcInterpolator;
using motioncore::multiaxis::CoordinatedTrajectoryPlanner;
using motioncore::multiaxis::Interpolator;
using motioncore::multiaxis::LineInterpolator;
using motioncore::multiaxis::MultiAxisServoState;
using motioncore::multiaxis::MultiAxisServoSystem;
using motioncore::multiaxis::MultiAxisTrajectoryState;
using motioncore::multiaxis::PlanningMode;
using motioncore::multiaxis::Vector3;
using motioncore::planning::MotionConstraints;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kPi = 3.141592653589793238462643383279502884;

struct Options {
    std::string geometry{"line"};
    std::string profile{"scurve"};
    std::string mode{"planner"};
    std::string direction{"ccw"};
    Vector3 start_mm{0.0, 0.0, 0.0};
    Vector3 target_mm{30.0, 40.0, 50.0};
    Vector3 center_mm{0.0, 0.0, 0.0};
    double vmax_mm_s{50.0};
    double amax_mm_s2{100.0};
    double jmax_mm_s3{500.0};
    double dt_ms{1.0};
    double hold_s{2.0};
    std::string output{"results/phase4/phase4.csv"};
    bool start_explicit{false};
    bool target_explicit{false};
    bool center_explicit{false};
};

struct Row {
    double time_s{0.0};
    MultiAxisTrajectoryState ref{};
    Vector3 actual{ kNaN, kNaN, kNaN };
    Vector3 actual_velocity{ kNaN, kNaN, kNaN };
    Vector3 actual_acceleration{ kNaN, kNaN, kNaN };
    Vector3 drive{ kNaN, kNaN, kNaN };
    Vector3 load{ kNaN, kNaN, kNaN };
    std::array<bool, 3> drive_saturated{false, false, false};
    double tracking_error_mm{kNaN};
    double path_deviation_mm{kNaN};
    double radial_error_mm{kNaN};
    double sync_error{kNaN};
};

double parse_number(const std::string& value, const std::string& name) {
    std::size_t used = 0;
    const double result = std::stod(value, &used);
    if (used != value.size() || !std::isfinite(result)) {
        throw std::invalid_argument(name + " must be a finite number");
    }
    return result;
}

Vector3 parse_vector(const std::string& text, const std::string& name) {
    std::stringstream stream(text);
    std::string item;
    std::array<double, 3> values{};
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (!std::getline(stream, item, ',')) {
            throw std::invalid_argument(name + " must have three comma-separated values");
        }
        values[i] = parse_number(item, name);
    }
    if (std::getline(stream, item, ',')) {
        throw std::invalid_argument(name + " must have exactly three values");
    }
    return {values[0], values[1], values[2]};
}

void print_help() {
    std::cout << "MotionCore Phase 4\n"
              << "  --geometry line|arc|multi-demo\n"
              << "  --profile trapezoid|scurve\n"
              << "  --mode planner|closed-loop\n"
              << "  --start-mm x,y,z --target-mm x,y,z --center-mm x,y,z\n"
              << "  --direction cw|ccw --vmax-mm-s V --amax-mm-s2 A --jmax-mm-s3 J\n"
              << "  --dt-ms DT --hold-s S --output FILE\n";
}

Options parse_options(int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string key(argv[i]);
        if (key == "--help" || key == "-h") {
            print_help();
            std::exit(0);
        }
        if (i + 1 >= argc) throw std::invalid_argument("missing value for " + key);
        const std::string value(argv[++i]);
        if (key == "--geometry") options.geometry = value;
        else if (key == "--profile") options.profile = value;
        else if (key == "--mode") options.mode = value;
        else if (key == "--direction") options.direction = value;
        else if (key == "--start-mm") { options.start_mm = parse_vector(value, key); options.start_explicit = true; }
        else if (key == "--target-mm") { options.target_mm = parse_vector(value, key); options.target_explicit = true; }
        else if (key == "--center-mm") { options.center_mm = parse_vector(value, key); options.center_explicit = true; }
        else if (key == "--vmax-mm-s") options.vmax_mm_s = parse_number(value, key);
        else if (key == "--amax-mm-s2") options.amax_mm_s2 = parse_number(value, key);
        else if (key == "--jmax-mm-s3") options.jmax_mm_s3 = parse_number(value, key);
        else if (key == "--dt-ms") options.dt_ms = parse_number(value, key);
        else if (key == "--hold-s") options.hold_s = parse_number(value, key);
        else if (key == "--output") options.output = value;
        else throw std::invalid_argument("unknown option: " + key);
    }
    if (options.geometry == "multi") {
        throw std::invalid_argument("geometry 'multi' is ambiguous; use the fixed demo name 'multi-demo'");
    }
    if (options.geometry != "line" && options.geometry != "arc" && options.geometry != "multi-demo")
        throw std::invalid_argument("geometry must be line, arc or multi-demo");
    if (options.profile != "trapezoid" && options.profile != "scurve")
        throw std::invalid_argument("profile must be trapezoid or scurve");
    if (options.mode != "planner" && options.mode != "closed-loop")
        throw std::invalid_argument("mode must be planner or closed-loop");
    if (options.direction != "cw" && options.direction != "ccw")
        throw std::invalid_argument("direction must be cw or ccw");
    if (options.dt_ms <= 0.0 || options.hold_s < 0.0)
        throw std::invalid_argument("dt-ms must be positive and hold-s non-negative");
    if (options.geometry == "multi-demo"
        && (options.start_explicit || options.target_explicit || options.center_explicit)) {
        throw std::invalid_argument(
            "multi-demo uses fixed geometry; do not pass start-mm, target-mm or center-mm");
    }
    return options;
}

Vector3 to_m(const Vector3& value_mm) { return value_mm * 1e-3; }
Vector3 to_mm(const Vector3& value_m) { return value_m * 1e3; }

PlanningMode planning_mode(const Options& options) {
    return options.profile == "trapezoid" ? PlanningMode::Trapezoid : PlanningMode::SCurve;
}

std::unique_ptr<Interpolator> make_single_interpolator(const Options& options) {
    MotionConstraints constraints{options.vmax_mm_s * 1e-3,
                                  options.amax_mm_s2 * 1e-3,
                                  options.jmax_mm_s3 * 1e-3};
    const auto mode = planning_mode(options);
    const Vector3 start = to_m(options.start_mm);
    const Vector3 target = to_m(options.target_mm);
    if (options.geometry == "line") {
        return std::make_unique<LineInterpolator>(start, target, constraints, mode);
    }
    if (options.geometry == "arc") {
        const auto direction = options.direction == "cw" ? ArcDirection::CW : ArcDirection::CCW;
        // The standard arc uses a stricter scalar constraint and resultant limits
        // when the user leaves the line defaults in place.
        const double accel_limit = 0.1;
        const double jerk_limit = 0.5;
        return std::make_unique<ArcInterpolator>(start, target, to_m(options.center_mm),
                                                 direction, constraints, mode, false,
                                                 motioncore::multiaxis::ArcResultantLimits{
                                                     accel_limit, jerk_limit});
    }
    throw std::invalid_argument("multi-demo geometry is built as a segment queue");
}

CoordinatedTrajectoryPlanner make_multi_planner(const Options& options) {
    MotionConstraints c{options.vmax_mm_s * 1e-3, options.amax_mm_s2 * 1e-3,
                        options.jmax_mm_s3 * 1e-3};
    const auto mode = planning_mode(options);
    // A connected line -> quarter circle -> line path demonstrates the queue
    // and its conservative zero-speed corner handling.
    const Vector3 p0 = to_m(options.start_mm);
    const Vector3 p1{0.030, 0.0, 0.0};
    const Vector3 p2{0.040, 0.010, 0.0};
    const Vector3 p3{0.040, 0.020, 0.0};
    CoordinatedTrajectoryPlanner planner;
    planner.add_line(LineInterpolator(p0, p1, c, mode));
    planner.add_arc(ArcInterpolator(p1, p2, Vector3{0.030, 0.010, 0.0},
                                    ArcDirection::CCW, c, mode, false,
                                    motioncore::multiaxis::ArcResultantLimits{0.1, 0.5}));
    planner.add_line(LineInterpolator(p2, p3, c, mode));
    return planner;
}

double line_deviation_m(const Vector3& actual, const LineInterpolator& line) {
    const Vector3 d = line.target_m() - line.start_m();
    const double length = d.norm();
    if (length <= 1e-15) return (actual - line.start_m()).norm();
    const Vector3 u = d / length;
    const double projection = std::clamp((actual - line.start_m()).dot(u), 0.0, length);
    const Vector3 nearest = line.start_m() + u * projection;
    return (actual - nearest).norm();
}

double arc_radial_error_m(const Vector3& actual, const ArcInterpolator& arc) {
    const Vector3 delta = actual - arc.center_m();
    return std::abs(std::hypot(delta.x, delta.y) - arc.radius_m());
}

// 到有限圆弧（而不是完整圆周）的最近距离。角度先按 CW/CCW 方向展开，
// 再限制在 [起点,终点] 区间；因此越过端点的实际点不会被误判为在圆弧上。
double arc_deviation_m(const Vector3& actual, const ArcInterpolator& arc) {
    const Vector3 radial = actual - arc.center_m();
    const double radial_norm = std::hypot(radial.x, radial.y);
    if (radial_norm <= 1e-15) return (actual - arc.start_m()).norm();
    double travelled = std::atan2(radial.y, radial.x) - arc.start_angle_rad();
    const double direction = arc.delta_angle_rad() < 0.0 ? -1.0 : 1.0;
    travelled *= direction;
    const double total = std::abs(arc.delta_angle_rad());
    while (travelled < 0.0) travelled += 2.0 * kPi;
    while (travelled > 2.0 * kPi) travelled -= 2.0 * kPi;
    travelled = std::clamp(travelled, 0.0, total);
    const double angle = arc.start_angle_rad() + direction * travelled;
    const Vector3 nearest = arc.center_m() + arc.radius_m()
        * Vector3{std::cos(angle), std::sin(angle), 0.0};
    return (actual - nearest).norm();
}

double path_progress_for_actual(const Vector3& actual, const Interpolator& interpolator) {
    if (const auto* line = dynamic_cast<const LineInterpolator*>(&interpolator)) {
        const Vector3 d = line->target_m() - line->start_m();
        if (d.norm() <= 1e-15) return 0.0;
        return std::clamp((actual - line->start_m()).dot(d.normalized()), 0.0, d.norm());
    }
    if (const auto* arc = dynamic_cast<const ArcInterpolator*>(&interpolator)) {
        const Vector3 radial = actual - arc->center_m();
        double angle = std::atan2(radial.y, radial.x) - arc->start_angle_rad();
        const double delta = arc->delta_angle_rad();
        if (delta > 0.0) {
            while (angle < 0.0) angle += 2.0 * kPi;
            angle = std::min(angle, delta);
        } else {
            while (angle > 0.0) angle -= 2.0 * kPi;
            angle = std::max(angle, delta);
        }
        return std::abs(angle) * arc->radius_m();
    }
    return 0.0;
}

double normalized_sync_error(const Vector3& actual,
                             const MultiAxisTrajectoryState& reference,
                             const Interpolator& interpolator) {
    if (const auto* line = dynamic_cast<const LineInterpolator*>(&interpolator)) {
        const std::array<double, 3> start{line->start_m().x, line->start_m().y,
                                          line->start_m().z};
        const std::array<double, 3> target{line->target_m().x, line->target_m().y,
                                           line->target_m().z};
        const std::array<double, 3> measured{actual.x, actual.y, actual.z};
        double minimum = std::numeric_limits<double>::infinity();
        double maximum = -std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < 3; ++i) {
            if (std::abs(target[i] - start[i]) <= 1e-15) continue;
            const double progress = (measured[i] - start[i]) / (target[i] - start[i]);
            minimum = std::min(minimum, progress);
            maximum = std::max(maximum, progress);
        }
        return std::isfinite(minimum) ? maximum - minimum : 0.0;
    }
    const double length = interpolator.path_length_m();
    if (length <= 1e-15) return 0.0;
    const double actual_progress = path_progress_for_actual(actual, interpolator);
    return std::abs(actual_progress - reference.path_progress_m) / length;
}

void multi_segment_metrics(const Vector3& actual,
                           const MultiAxisTrajectoryState& reference,
                           const CoordinatedTrajectoryPlanner& planner,
                           double& path_deviation_mm,
                           double& radial_error_mm,
                           double& sync_error) {
    const auto& segments = planner.segments();
    if (segments.empty()) return;
    const motioncore::multiaxis::MotionSegment* selected = &segments.back();
    double prior_path = 0.0;
    for (const auto& segment : segments) {
        if (reference.time_s < segment.start_time_s + segment.duration()
            || &segment == &segments.back()) {
            selected = &segment;
            break;
        }
        prior_path += segment.path_length_m();
    }
    const auto local_reference = [&] {
        auto result = reference;
        result.path_progress_m -= prior_path;
        return result;
    }();
    std::visit([&](const auto& geometry) {
        using Geometry = std::decay_t<decltype(geometry)>;
        if constexpr (std::is_same_v<Geometry, LineInterpolator>) {
            path_deviation_mm = line_deviation_m(actual, geometry) * 1e3;
            // A line has no separate radial metric; retain NaN to avoid
            // conflating its perpendicular distance with an arc radius error.
        } else {
            path_deviation_mm = arc_deviation_m(actual, geometry) * 1e3;
            radial_error_mm = arc_radial_error_m(actual, geometry) * 1e3;
        }
        sync_error = normalized_sync_error(actual, local_reference, geometry);
    }, selected->geometry);
}

void write_header(std::ofstream& file) {
    file << "time_s,x_ref_mm,y_ref_mm,z_ref_mm,x_actual_mm,y_actual_mm,z_actual_mm,"
            "vx_ref_mm_s,vy_ref_mm_s,vz_ref_mm_s,vx_actual_mm_s,vy_actual_mm_s,vz_actual_mm_s,"
            "ax_ref_mm_s2,ay_ref_mm_s2,az_ref_mm_s2,ax_actual_mm_s2,ay_actual_mm_s2,az_actual_mm_s2,"
            "jx_ref_mm_s3,jy_ref_mm_s3,jz_ref_mm_s3,jerk_ref_magnitude_mm_s3,"
            "tracking_error_mm,path_deviation_mm,radial_error_mm,sync_error,"
            "drive_force_x_N,drive_force_y_N,drive_force_z_N,load_force_x_N,load_force_y_N,load_force_z_N,"
            "drive_saturated_x,drive_saturated_y,drive_saturated_z\n";
}

void write_row(std::ofstream& file, const Row& row) {
    const auto p = to_mm(row.ref.position_m);
    const auto v = to_mm(row.ref.velocity_mps);
    const auto a = to_mm(row.ref.acceleration_mps2);
    const auto j = to_mm(row.ref.jerk_mps3);
    const auto ap = to_mm(row.actual);
    const auto av = to_mm(row.actual_velocity);
    const auto aa = to_mm(row.actual_acceleration);
    // jerk_mps3 is stored internally in m/s^3.  The CSV columns are all
    // explicitly labeled mm/s^3, so the magnitude must be converted with the
    // same 1000 factor as each exported component.
    const double jerk_magnitude = row.ref.jerk_mps3.norm() * 1000.0;
    file << std::setprecision(12) << row.time_s << ',' << p.x << ',' << p.y << ',' << p.z << ','
         << ap.x << ',' << ap.y << ',' << ap.z << ',' << v.x << ',' << v.y << ',' << v.z << ','
         << av.x << ',' << av.y << ',' << av.z << ',' << a.x << ',' << a.y << ',' << a.z << ','
         << aa.x << ',' << aa.y << ',' << aa.z << ',' << j.x << ',' << j.y << ',' << j.z << ','
         << jerk_magnitude << ',' << row.tracking_error_mm << ',' << row.path_deviation_mm << ','
         << row.radial_error_mm << ',' << row.sync_error << ',' << row.drive.x << ','
         << row.drive.y << ',' << row.drive.z << ',' << row.load.x << ',' << row.load.y << ','
         << row.load.z << ',' << (row.drive_saturated[0] ? 1 : 0) << ','
         << (row.drive_saturated[1] ? 1 : 0) << ',' << (row.drive_saturated[2] ? 1 : 0) << '\n';
}

int run(const Options& options) {
    std::unique_ptr<Interpolator> single;
    CoordinatedTrajectoryPlanner queued;
    const Interpolator* interpolator = nullptr;
    if (options.geometry == "multi-demo") {
        queued = make_multi_planner(options);
    } else {
        single = make_single_interpolator(options);
        interpolator = single.get();
    }
    const double duration = interpolator != nullptr ? interpolator->duration() : queued.duration();
    const double dt = options.dt_ms * 1e-3;
    std::vector<Row> rows;
    rows.reserve(static_cast<std::size_t>((duration + options.hold_s) / dt) + 3U);

    auto sample_reference = [&](double time) {
        return interpolator != nullptr ? interpolator->sample(time) : queued.sample(time);
    };
    auto servo_configs = MultiAxisServoSystem::default_configs();
    const auto initial_reference = sample_reference(0.0);
    servo_configs[0].dynamics.initial_position_m = initial_reference.position_m.x;
    servo_configs[1].dynamics.initial_position_m = initial_reference.position_m.y;
    servo_configs[2].dynamics.initial_position_m = initial_reference.position_m.z;
    MultiAxisServoSystem servo(servo_configs);
    const bool closed_loop = options.mode == "closed-loop";
    if (closed_loop) {
        Row initial;
        initial.ref = initial_reference;
        initial.time_s = 0.0;
        const auto& s = servo.state();
        initial.actual = s.actual_position_m;
        initial.actual_velocity = s.actual_velocity_mps;
        initial.actual_acceleration = s.actual_acceleration_mps2;
        rows.push_back(initial);
    }

    double time = 0.0;
    const double end_time = duration + (closed_loop ? options.hold_s : 0.0);
    while (time < end_time - 1e-12) {
        const double next = std::min(end_time, time + dt);
        const double step_dt = next - time;
        const auto reference = sample_reference(std::min(next, duration));
        Row row;
        row.time_s = next;
        row.ref = reference;
        if (closed_loop) {
            const MultiAxisServoState& state = servo.step(reference, step_dt);
            row.actual = state.actual_position_m;
            row.actual_velocity = state.actual_velocity_mps;
            row.actual_acceleration = state.actual_acceleration_mps2;
            row.drive = state.drive_force_n;
            row.load = state.load_force_n;
            row.drive_saturated = state.drive_saturated;
        }
        const Vector3 ref_m = row.ref.position_m;
        if (closed_loop) {
            row.tracking_error_mm = (row.actual - ref_m).norm() * 1e3;
            if (interpolator != nullptr) {
                if (const auto* line = dynamic_cast<const LineInterpolator*>(interpolator))
                    row.path_deviation_mm = line_deviation_m(row.actual, *line) * 1e3;
                else if (const auto* arc = dynamic_cast<const ArcInterpolator*>(interpolator)) {
                    row.path_deviation_mm = arc_deviation_m(row.actual, *arc) * 1e3;
                    row.radial_error_mm = arc_radial_error_m(row.actual, *arc) * 1e3;
                }
                const double length = interpolator->path_length_m();
                if (length > 1e-15) {
                    row.sync_error = normalized_sync_error(row.actual, row.ref, *interpolator);
                }
            } else {
                multi_segment_metrics(row.actual, row.ref, queued,
                                      row.path_deviation_mm, row.radial_error_mm,
                                      row.sync_error);
            }
        } else {
            row.actual = {kNaN, kNaN, kNaN};
        }
        rows.push_back(row);
        time = next;
    }
    if (!closed_loop && rows.empty()) {
        Row row;
        row.time_s = duration;
        row.ref = sample_reference(duration);
        rows.push_back(row);
    }

    std::ofstream file(options.output);
    if (!file) throw std::runtime_error("cannot open output: " + options.output);
    write_header(file);
    for (const auto& row : rows) write_row(file, row);

    double sum_sq = 0.0;
    double max_error = 0.0;
    double max_path_deviation = 0.0;
    double max_sync = 0.0;
    double peak_force = 0.0;
    double max_actual_speed = 0.0;
    double max_actual_acceleration = 0.0;
    double max_actual_jerk = 0.0;
    int force_saturation = 0;
    std::size_t actual_count = 0;
    bool have_previous_acceleration = false;
    Vector3 previous_acceleration{};
    double previous_time = 0.0;
    for (const auto& row : rows) {
        if (std::isfinite(row.tracking_error_mm)) {
            sum_sq += row.tracking_error_mm * row.tracking_error_mm;
            max_error = std::max(max_error, row.tracking_error_mm);
            max_path_deviation = std::max(max_path_deviation,
                                          std::isfinite(row.path_deviation_mm) ? row.path_deviation_mm : 0.0);
            max_sync = std::max(max_sync, std::isfinite(row.sync_error) ? row.sync_error : 0.0);
            peak_force = std::max(peak_force, row.drive.norm());
            max_actual_speed = std::max(max_actual_speed, row.actual_velocity.norm() * 1e3);
            max_actual_acceleration = std::max(max_actual_acceleration,
                                               row.actual_acceleration.norm() * 1e3);
            if (have_previous_acceleration && row.time_s > previous_time) {
                max_actual_jerk = std::max(
                    max_actual_jerk,
                    ((row.actual_acceleration - previous_acceleration)
                         / (row.time_s - previous_time)).norm() * 1e3);
            }
            previous_acceleration = row.actual_acceleration;
            previous_time = row.time_s;
            have_previous_acceleration = true;
            if (row.drive_saturated[0] || row.drive_saturated[1] || row.drive_saturated[2]) {
                ++force_saturation;
            }
            ++actual_count;
        }
    }
    const Row& last = rows.back();
    const double final_error = std::isfinite(last.tracking_error_mm) ? last.tracking_error_mm : 0.0;
    const double final_speed = std::isfinite(last.actual_velocity.norm())
        ? last.actual_velocity.norm() * 1e3 : 0.0;
    const auto final_reference_mm = to_mm(last.ref.position_m);
    std::cout << std::setprecision(12)
              << "geometry=" << options.geometry << ", profile=" << options.profile
              << ", mode=" << options.mode << ", duration_s=" << duration
              << ", total_simulation_s=" << end_time
              << ", final_reference_xyz_mm=" << final_reference_mm.x << ','
              << final_reference_mm.y << ',' << final_reference_mm.z
              << ", final_reference_path_mm=" << last.ref.path_progress_m * 1e3
              << ", final_tracking_error_mm=" << final_error
              << ", final_speed_mm_s=" << final_speed
              << ", max_tracking_error_mm=" << (actual_count ? max_error : 0.0)
              << ", rmse_mm=" << (actual_count ? std::sqrt(sum_sq / actual_count) : 0.0)
              << ", max_path_deviation_mm=" << max_path_deviation
              << ", max_sync_error=" << max_sync
              << ", max_actual_speed_mm_s=" << max_actual_speed
              << ", max_actual_acceleration_mm_s2=" << max_actual_acceleration
              << ", max_actual_jerk_mm_s3=" << max_actual_jerk
              << ", peak_drive_force_N=" << peak_force
              << ", saturation_samples=" << force_saturation
              << ", csv=" << options.output << '\n';
    return 0;
}
}  // namespace

int main(int argc, char** argv) {
    try {
        return run(parse_options(argc, argv));
    } catch (const std::exception& error) {
        std::cerr << "MotionCore Phase 4 error: " << error.what() << '\n';
        return 1;
    }
}

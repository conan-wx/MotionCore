#include "simulation_worker.hpp"

#include <QFile>
#include <QFileInfo>
#include <QCoreApplication>
#include <QDir>
#include <QHash>
#include <QTextStream>
#include <QTimer>

#include <algorithm>
#include <cmath>

using motioncore::manager::CommandStatus;
using motioncore::manager::MotionControllerConfig;
using motioncore::manager::MotionState;
using motioncore::manager::to_string;
using motioncore::multiaxis::Vector3;
using motioncore::planning::MotionConstraints;

namespace {

MotionControllerConfig build_config() {
    MotionControllerConfig config;
    config.queue_capacity = 32;
    config.servo_configs = motioncore::multiaxis::MultiAxisServoSystem::default_configs();
    config.servo_configs[0].dynamics.initial_position_m = .006;
    config.servo_configs[1].dynamics.initial_position_m = .005;
    config.servo_configs[2].dynamics.initial_position_m = .004;
    for (auto& homing : config.homing.axes) {
        homing.sensor.trigger_position_m = 0.0;
        homing.sensor.direction = motioncore::manager::HomeDirection::Negative;
        homing.fast_speed_mps = .015;
        homing.slow_speed_mps = .004;
        homing.backoff_distance_m = .002;
        homing.search_range_m = .05;
        homing.timeout_s = 10.0;
        homing.sensor.debounce_time_s = .002;
    }
    for (auto& limit : config.soft_limits) {
        limit.min_position_m = 0.0;
        limit.max_position_m = .1;
        limit.max_velocity_mps = .1;
        limit.max_acceleration_mps2 = .8;
        limit.max_braking_deceleration_mps2 = .5;
    }
    config.following_error_limit_m = .05;
    config.following_error_hold_s = .02;
    config.motion_timeout_factor = 6.0;
    return config;
}

}  // namespace

SimulationWorker::SimulationWorker(QObject* parent)
    : QObject(parent), controller_(make_config()) {
    history_log_path_ = QDir(QDir::tempPath()).filePath(
        QString("motioncore_gui_history_%1.csv")
            .arg(static_cast<qulonglong>(reinterpret_cast<quintptr>(this)), 0, 16));
    // QFile is a QObject.  The worker is later moved to its dedicated
    // QThread, so open the stream from start(), after the move, rather than
    // constructing it in the GUI thread and writing it from another thread.
}

MotionControllerConfig SimulationWorker::make_config() { return build_config(); }

void SimulationWorker::start() {
    if (!history_log_.isOpen()) initialise_history_log();
    if (!timer_object_) {
        auto* timer = new QTimer(this);
        timer_object_ = timer;
        timer->setInterval(20);  // GUI refresh period; simulation remains 1 ms.
        connect(timer, &QTimer::timeout, this, &SimulationWorker::tick);
        timer->start();
    }
    emit_snapshot();
}

void SimulationWorker::set_running(bool running) {
    if (replay_mode_) replay_playing_ = running;
    else running_ = running;
}

void SimulationWorker::set_speed(double multiplier) {
    if (std::isfinite(multiplier)) speed_multiplier_ = std::clamp(multiplier, 0.1, 20.0);
}

void SimulationWorker::enable_controller() {
    if (replay_mode_) {
        emit event_message("Enable rejected: offline replay mode");
        return;
    }
    const bool ok = controller_.enable();
    emit event_message(ok ? "Enable accepted" : "Enable rejected");
    emit_snapshot();
}

void SimulationWorker::disable_controller() {
    if (replay_mode_) { emit event_message("Disable rejected: offline replay mode"); return; }
    const bool ok = controller_.disable();
    if (ok) running_ = true; // keep simulating any residual mechanical inertia
    emit event_message(ok ? "Disable accepted; drive disabled" : "Disable rejected");
    emit_snapshot();
}

void SimulationWorker::home_axes() {
    if (replay_mode_) {
        emit event_message("Home rejected: offline replay mode");
        return;
    }
    const bool ok = controller_.home();
    running_ = ok;
    emit event_message(ok ? "Homing started" : "Homing rejected");
    emit_snapshot();
}

void SimulationWorker::move_line(double x_mm, double y_mm, double z_mm,
                                 double vmax_mm_s, double amax_mm_s2,
                                 double jmax_mm_s3, bool scurve) {
    if (replay_mode_) {
        emit event_message("MoveLine rejected: offline replay mode");
        return;
    }
    // The controller resolves absolute commands against its queue's planned
    // endpoint.  The GUI supplies only the measured position as a fallback;
    // it never predicts a future endpoint itself.
    const auto actual = controller_.status().actual_position_m;
    const Vector3 target{x_mm * 1e-3, y_mm * 1e-3, z_mm * 1e-3};
    const auto result = controller_.enqueue_move_line(
        actual, target,
        {vmax_mm_s * 1e-3, amax_mm_s2 * 1e-3, jmax_mm_s3 * 1e-3},
        scurve ? motioncore::multiaxis::PlanningMode::SCurve
               : motioncore::multiaxis::PlanningMode::Trapezoid);
    emit event_message(result.accepted
                           ? QString("MoveLine queued id=%1").arg(result.command_id)
                           : QString("MoveLine rejected: %1").arg(QString::fromStdString(result.message)));
    if (result.accepted) running_ = true;
    emit_snapshot();
}

void SimulationWorker::move_arc_demo(double vmax_mm_s, double amax_mm_s2,
                                     double jmax_mm_s3, bool scurve) {
    if (replay_mode_) {
        emit event_message("Arc rejected: offline replay mode");
        return;
    }
    // A deterministic safe demonstration arc in the positive XY quadrant.
    // It can be queued after the line command that reaches its start point.
    const Vector3 arc_start = controller_.queue().planned_endpoint().value_or(
        controller_.status().actual_position_m);
    const Vector3 expected_start{0.05, 0.03, 0.01};
    if ((arc_start - expected_start).norm() > 2e-6) {
        emit event_message("Arc rejected: demo arc requires the preceding endpoint (50,30,10) mm");
        emit_snapshot();
        return;
    }
    const Vector3 arc_target{0.03, 0.05, 0.01};
    const auto result = controller_.enqueue_move_arc(
        arc_start, arc_target, {0.03, 0.03, 0.01},
        motioncore::manager::ArcCommandDirection::CCW,
        {vmax_mm_s * 1e-3, amax_mm_s2 * 1e-3, jmax_mm_s3 * 1e-3},
        scurve ? motioncore::multiaxis::PlanningMode::SCurve
               : motioncore::multiaxis::PlanningMode::Trapezoid);
    emit event_message(result.accepted
                           ? QString("Arc queued id=%1").arg(result.command_id)
                           : QString("Arc rejected: %1").arg(QString::fromStdString(result.message)));
    if (result.accepted) running_ = true;
    emit_snapshot();
}

void SimulationWorker::move_arc(double x_mm, double y_mm, double z_mm,
                                double cx_mm, double cy_mm, bool clockwise,
                                double vmax_mm_s, double amax_mm_s2,
                                double jmax_mm_s3, bool scurve) {
    if (replay_mode_) { emit event_message("Arc rejected: offline replay mode"); return; }
    const auto start = controller_.queue().planned_endpoint().value_or(
        controller_.status().actual_position_m);
    const auto result = controller_.enqueue_move_arc(start,
        {x_mm * .001, y_mm * .001, z_mm * .001}, {cx_mm * .001, cy_mm * .001, start.z},
        clockwise ? motioncore::manager::ArcCommandDirection::CW
                  : motioncore::manager::ArcCommandDirection::CCW,
        {vmax_mm_s * .001, amax_mm_s2 * .001, jmax_mm_s3 * .001},
        scurve ? motioncore::multiaxis::PlanningMode::SCurve
               : motioncore::multiaxis::PlanningMode::Trapezoid);
    if (result.accepted) running_ = true;
    emit event_message(result.accepted ? QString("Arc queued id=%1").arg(result.command_id)
        : QString("Arc rejected: %1").arg(QString::fromStdString(result.message)));
    emit_snapshot();
}

void SimulationWorker::controlled_stop() {
    if (replay_mode_) {
        emit event_message("Stop ignored: offline replay mode");
        return;
    }
    const bool ok = controller_.stop();
    // A stop request is a control event, not ordinary simulation work.  Run
    // one controller cycle immediately so it is honoured while the GUI is
    // paused; subsequent timer ticks continue the physical deceleration.
    if (ok) {
        running_ = true;
        (void)controller_.process_control_events();
    }
    emit event_message(ok ? "Controlled Stop requested" : "Controlled Stop rejected");
    emit_snapshot();
}

void SimulationWorker::emergency_stop() {
    if (replay_mode_) {
        emit event_message("EmergencyStop ignored: offline replay mode");
        return;
    }
    const bool ok = controller_.emergency_stop();
    // EmergencyStop has the highest priority.  Process the queued event even
    // when ordinary simulation advancement is paused, then keep ticking while
    // the disabled drive lets the virtual mechanics coast to rest.
    if (ok) {
        running_ = true;
        (void)controller_.process_control_events();
    }
    emit event_message(ok ? "Emergency Stop requested" : "Emergency Stop rejected");
    emit_snapshot();
}

void SimulationWorker::reset_fault_or_emergency() {
    if (replay_mode_) {
        emit event_message("Reset ignored: offline replay mode");
        return;
    }
    bool ok = false;
    if (controller_.state() == MotionState::EmergencyStopped) {
        controller_.release_emergency_input();
        ok = controller_.reset_emergency();
    }
    else if (controller_.state() == MotionState::Fault) ok = controller_.reset_fault();
    if (ok) {
        running_ = false;
    }
    emit event_message(ok ? "Reset accepted; re-home is required" : "Reset rejected: cause is active or state is not latched");
    emit_snapshot();
}

void SimulationWorker::reset_simulation() {
    controller_ = motioncore::manager::MotionController(make_config());
    samples_.clear();
    replay_samples_.clear();
    replay_index_ = 0;
    replay_mode_ = false;
    replay_playing_ = false;
    have_logged_time_ = false;
    last_logged_time_s_ = 0.0;
    derivative_time_s_ = -1.0;
    actual_jerk_est_mps3_ = {};
    initialise_history_log();
    running_ = false;
    emit event_message("Simulation reset");
    emit_snapshot();
}

void SimulationWorker::tick() {
    if (replay_mode_) {
        if (replay_playing_ && replay_index_ < replay_samples_.size()) {
            replay_clock_s_ += .02 * speed_multiplier_;
            while (replay_index_ + 1 < replay_samples_.size() &&
                   replay_samples_[replay_index_ + 1].time_s <= replay_clock_s_)
                ++replay_index_;
            emit_replay_snapshot();
            if (replay_index_ == replay_samples_.size() - 1) ++replay_index_;
            if (replay_index_ >= replay_samples_.size()) replay_playing_ = false;
        }
        return;
    }
    if (running_) {
        const int steps = std::max(1, static_cast<int>(std::lround(speed_multiplier_ * 20.0)));
        for (int i = 0; i < steps; ++i) {
            const bool normal = controller_.step(.001);
            // Log every physical 1 ms step, independently of the 20 ms display
            // refresh.  Reference/actual jerk is never computed from GUI FPS.
            append_history_sample(capture_snapshot());
            if (!normal) {
                // MotionController returns false for latched emergency/fault
                // states even though EmergencyStopped still advances the
                // disabled-drive dynamics.  Keep the worker alive for that
                // physical tail and stop only after the axis has settled.
                const auto state = controller_.state();
                if (state == MotionState::EmergencyStopped) {
                    const auto velocity = controller_.status().actual_velocity_mps.norm();
                    running_ = velocity > 1e-8;
                } else if (state == MotionState::Fault) {
                    // A latched fault blocks new commands, but it does not
                    // freeze the virtual mechanics.  Keep sampling while an
                    // axis still has kinetic energy so the GUI shows the
                    // physically meaningful free-wheel tail.
                    const auto velocity = controller_.status().actual_velocity_mps.norm();
                    running_ = velocity > 1e-8;
                } else {
                    running_ = state == MotionState::Homing;
                }
                if (!running_) break;
            }
        }
    }
    emit_snapshot();
}

void SimulationWorker::emit_snapshot() {
    const auto snapshot = capture_snapshot();
    append_history_sample(snapshot);
    emit snapshot_ready(snapshot);
}

GuiSnapshot SimulationWorker::capture_snapshot() {
    const auto& status = controller_.status();
    GuiSnapshot snapshot;
    snapshot.time_s = status.time_s;
    snapshot.state = QString::fromLatin1(to_string(status.state));
    snapshot.fault = QString::fromLatin1(to_string(status.fault_code));
    snapshot.command_id = status.command_id;
    snapshot.command_status = QString::fromLatin1(to_string(status.command_status));
    snapshot.x_ref_mm = status.reference_position_m.x * 1000.0;
    snapshot.y_ref_mm = status.reference_position_m.y * 1000.0;
    snapshot.z_ref_mm = status.reference_position_m.z * 1000.0;
    snapshot.vx_ref_mm_s = status.reference_velocity_mps.x * 1000.0;
    snapshot.vy_ref_mm_s = status.reference_velocity_mps.y * 1000.0;
    snapshot.vz_ref_mm_s = status.reference_velocity_mps.z * 1000.0;
    snapshot.ax_ref_mm_s2 = status.reference_acceleration_mps2.x * 1000.0;
    snapshot.ay_ref_mm_s2 = status.reference_acceleration_mps2.y * 1000.0;
    snapshot.az_ref_mm_s2 = status.reference_acceleration_mps2.z * 1000.0;
    snapshot.jx_ref_mm_s3 = status.reference_jerk_mps3.x * 1000.0;
    snapshot.jy_ref_mm_s3 = status.reference_jerk_mps3.y * 1000.0;
    snapshot.jz_ref_mm_s3 = status.reference_jerk_mps3.z * 1000.0;
    if (derivative_time_s_ >= 0.0 && status.time_s > derivative_time_s_ + 1e-12)
        actual_jerk_est_mps3_ = (status.actual_acceleration_mps2 - previous_acceleration_mps2_) *
                              (1.0 / (status.time_s - derivative_time_s_));
    if (status.time_s > derivative_time_s_) {
        previous_acceleration_mps2_ = status.actual_acceleration_mps2;
        derivative_time_s_ = status.time_s;
    }
    snapshot.jx_actual_est_mm_s3 = actual_jerk_est_mps3_.x * 1000.0;
    snapshot.jy_actual_est_mm_s3 = actual_jerk_est_mps3_.y * 1000.0;
    snapshot.jz_actual_est_mm_s3 = actual_jerk_est_mps3_.z * 1000.0;
    snapshot.queue_summary = QString("FIFO waiting: %1 | priority events: %2")
        .arg(controller_.queue().normal_size()).arg(controller_.queue().priority_size());
    snapshot.x_mm = status.actual_position_m.x * 1000.0;
    snapshot.y_mm = status.actual_position_m.y * 1000.0;
    snapshot.z_mm = status.actual_position_m.z * 1000.0;
    snapshot.vx_mm_s = status.actual_velocity_mps.x * 1000.0;
    snapshot.vy_mm_s = status.actual_velocity_mps.y * 1000.0;
    snapshot.vz_mm_s = status.actual_velocity_mps.z * 1000.0;
    snapshot.ax_mm_s2 = status.actual_acceleration_mps2.x * 1000.0;
    snapshot.ay_mm_s2 = status.actual_acceleration_mps2.y * 1000.0;
    snapshot.az_mm_s2 = status.actual_acceleration_mps2.z * 1000.0;
    snapshot.following_error_mm = status.following_error_m * 1000.0;
    snapshot.drive_x_n = status.drive_force_n.x;
    snapshot.drive_y_n = status.drive_force_n.y;
    snapshot.drive_z_n = status.drive_force_n.z;
    snapshot.homed_x = status.homed[0];
    snapshot.homed_y = status.homed[1];
    snapshot.homed_z = status.homed[2];
    snapshot.fault_active = status.fault_active;
    snapshot.soft_limit_margin_x_mm = status.soft_limit_margin_m.x * 1000.0;
    snapshot.soft_limit_margin_y_mm = status.soft_limit_margin_m.y * 1000.0;
    snapshot.soft_limit_margin_z_mm = status.soft_limit_margin_m.z * 1000.0;
    return snapshot;
}

void SimulationWorker::emit_replay_snapshot() {
    if (replay_index_ >= 0 && replay_index_ < replay_samples_.size()) {
        auto snapshot = replay_samples_[replay_index_];
        snapshot.offline_replay = true;
        emit snapshot_ready(snapshot);
    }
}

void SimulationWorker::load_replay_csv(const QString& path) {
    QFile file(path);
    if (path.isEmpty() || !file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        emit event_message(QString("Replay rejected: cannot open %1").arg(path));
        return;
    }
    QTextStream input(&file);
    const QString header_line = input.readLine();
    if (header_line.isEmpty()) {
        emit event_message("Replay rejected: empty CSV");
        return;
    }
    const QStringList headers = header_line.split(',');
    QHash<QString, int> column;
    for (int i = 0; i < headers.size(); ++i) {
        const auto name = headers[i].trimmed();
        if (name.isEmpty() || column.contains(name)) {
            emit event_message("Replay rejected: duplicate or empty CSV column"); return;
        }
        column.insert(name, i);
    }
    const QStringList required{"time_s", "x_actual_mm", "y_actual_mm", "z_actual_mm",
                              "vx_actual_mm_s", "vy_actual_mm_s", "vz_actual_mm_s"};
    for (const auto& name : required) {
        if (!column.contains(name)) {
            emit event_message(QString("Replay rejected: missing column %1").arg(name));
            return;
        }
    }
    QVector<GuiSnapshot> parsed;
    double previous_time = -1.0;
    int line_number = 1;
    while (!input.atEnd()) {
        ++line_number;
        const QString line = input.readLine();
        if (line.trimmed().isEmpty()) continue;
        const QStringList fields = line.split(',');
        if (fields.size() != headers.size()) {
            emit event_message(QString("Replay rejected: malformed row %1").arg(line_number)); return;
        }
        // Missing optional legacy columns are allowed.  Existing numeric
        // fields must still be valid; only analytic reference Jerk may be NaN
        // because a trapezoid has acceleration jumps (not finite Jerk).
        for (auto it = column.cbegin(); it != column.cend(); ++it) {
            if (it.key() == "motion_state" || it.key() == "state" ||
                it.key() == "command_status" || it.key() == "fault_code" || it.key() == "fault") continue;
            bool valid = false;
            const double value = fields[it.value()].toDouble(&valid);
            const bool analytic_jerk = it.key().startsWith("j") && it.key().contains("ref");
            if (!valid || (!std::isfinite(value) && !(analytic_jerk && std::isnan(value)))) {
                emit event_message(QString("Replay rejected: invalid %1 at row %2").arg(it.key()).arg(line_number)); return;
            }
        }
        auto number = [&](const QString& name, double fallback, bool* ok_out = nullptr) {
            const int index = column.value(name, -1);
            if (index < 0 || index >= fields.size()) {
                if (ok_out) *ok_out = true;
                return fallback;
            }
            bool ok = false;
            const double value = fields[index].toDouble(&ok);
            if (ok_out) *ok_out = ok && std::isfinite(value);
            return value;
        };
        bool ok = false;
        GuiSnapshot snapshot;
        snapshot.time_s = number("time_s", 0.0, &ok);
        if (!ok || snapshot.time_s < 0.0 ||
            (!parsed.isEmpty() && snapshot.time_s <= previous_time)) {
            emit event_message(QString("Replay rejected: invalid time at CSV line %1").arg(line_number));
            return;
        }
        previous_time = snapshot.time_s;
        snapshot.x_mm = number("x_actual_mm", 0.0, &ok); if (!ok) return (void)emit event_message(QString("Replay rejected: non-finite X at line %1").arg(line_number));
        snapshot.y_mm = number("y_actual_mm", 0.0, &ok); if (!ok) return (void)emit event_message(QString("Replay rejected: non-finite Y at line %1").arg(line_number));
        snapshot.z_mm = number("z_actual_mm", 0.0, &ok); if (!ok) return (void)emit event_message(QString("Replay rejected: non-finite Z at line %1").arg(line_number));
        snapshot.vx_mm_s = number("vx_actual_mm_s", 0.0, &ok); if (!ok) return (void)emit event_message(QString("Replay rejected: non-finite velocity at line %1").arg(line_number));
        snapshot.vy_mm_s = number("vy_actual_mm_s", 0.0, &ok); if (!ok) return (void)emit event_message(QString("Replay rejected: non-finite velocity at line %1").arg(line_number));
        snapshot.vz_mm_s = number("vz_actual_mm_s", 0.0, &ok); if (!ok) return (void)emit event_message(QString("Replay rejected: non-finite velocity at line %1").arg(line_number));
        snapshot.x_ref_mm = number("x_ref_mm", snapshot.x_mm);
        snapshot.y_ref_mm = number("y_ref_mm", snapshot.y_mm);
        snapshot.z_ref_mm = number("z_ref_mm", snapshot.z_mm);
        snapshot.vx_ref_mm_s = number("vx_ref_mm_s", 0.0);
        snapshot.vy_ref_mm_s = number("vy_ref_mm_s", 0.0);
        snapshot.vz_ref_mm_s = number("vz_ref_mm_s", 0.0);
        snapshot.ax_ref_mm_s2 = number("ax_ref_mm_s2", 0.0);
        snapshot.ay_ref_mm_s2 = number("ay_ref_mm_s2", 0.0);
        snapshot.az_ref_mm_s2 = number("az_ref_mm_s2", 0.0);
        snapshot.jx_ref_mm_s3 = number("jx_ref_mm_s3", 0.0);
        snapshot.jy_ref_mm_s3 = number("jy_ref_mm_s3", 0.0);
        snapshot.jz_ref_mm_s3 = number("jz_ref_mm_s3", 0.0);
        snapshot.jx_actual_est_mm_s3 = number("jx_actual_est_mm_s3", 0.0);
        snapshot.jy_actual_est_mm_s3 = number("jy_actual_est_mm_s3", 0.0);
        snapshot.jz_actual_est_mm_s3 = number("jz_actual_est_mm_s3", 0.0);
        snapshot.ax_mm_s2 = number("ax_actual_mm_s2", 0.0);
        snapshot.ay_mm_s2 = number("ay_actual_mm_s2", 0.0);
        snapshot.az_mm_s2 = number("az_actual_mm_s2", 0.0);
        snapshot.following_error_mm = number("following_error_mm", 0.0);
        snapshot.drive_x_n = number("drive_force_x_n", 0.0);
        snapshot.drive_y_n = number("drive_force_y_n", 0.0);
        snapshot.drive_z_n = number("drive_force_z_n", 0.0);
        const auto text = [&](const QString& name, const QString& fallback) {
            const int index = column.value(name, -1);
            return index >= 0 && index < fields.size() ? fields[index] : fallback;
        };
        snapshot.state = text("motion_state", text("state", "Replay"));
        snapshot.fault = text("fault_code", text("fault", "None"));
        snapshot.command_status = text("command_status", "replay");
        snapshot.command_id = text("command_id", "0").toULongLong();
        snapshot.fault_active = text("fault_active", "0") == "1";
        snapshot.homed_x = text("homed_x", "0") == "1";
        snapshot.homed_y = text("homed_y", "0") == "1";
        snapshot.homed_z = text("homed_z", "0") == "1";
        snapshot.soft_limit_margin_x_mm = number("soft_limit_margin_x_mm", 0.0);
        snapshot.soft_limit_margin_y_mm = number("soft_limit_margin_y_mm", 0.0);
        snapshot.soft_limit_margin_z_mm = number("soft_limit_margin_z_mm", 0.0);
        parsed.push_back(snapshot);
        constexpr int kReplayMemoryLimit = 200000;
        if (parsed.size() > kReplayMemoryLimit) {
            emit event_message("Replay rejected: more than 200000 samples; select a shorter CSV window"); return;
        }
    }
    if (parsed.isEmpty()) {
        emit event_message("Replay rejected: CSV contains no samples");
        return;
    }
    replay_samples_ = std::move(parsed);
    replay_index_ = 0;
    replay_clock_s_ = replay_samples_.front().time_s;
    replay_mode_ = true;
    replay_playing_ = false;
    running_ = false;
    emit event_message(QString("Offline replay loaded: %1 samples from %2")
                           .arg(replay_samples_.size()).arg(path));
    emit_replay_snapshot();
}

void SimulationWorker::export_csv(const QString& path) {
    if (path.isEmpty()) {
        emit csv_exported(path, false, "empty output path");
        return;
    }
    if (!history_log_.isOpen() || !history_log_.flush()) {
        emit csv_exported(path, false, "complete history stream is unavailable"); return;
    }
    if (QFileInfo(path).absoluteFilePath() == QFileInfo(history_log_path_).absoluteFilePath()) {
        emit csv_exported(path, false, "output cannot overwrite active history stream"); return;
    }
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        emit csv_exported(path, false, file.errorString());
        return;
    }
    QFile source(history_log_path_);
    if (!source.open(QIODevice::ReadOnly)) {
        emit csv_exported(path, false, source.errorString()); return;
    }
    // Copy in bounded chunks, never read the whole experiment into memory.
    while (!source.atEnd()) {
        const auto chunk = source.read(64 * 1024);
        if (file.write(chunk) != chunk.size()) {
            emit csv_exported(path, false, file.errorString()); return;
        }
    }
    emit csv_exported(path, true, {});
}

void SimulationWorker::initialise_history_log() {
    if (history_log_.isOpen()) history_log_.close();
    if (history_log_path_.isEmpty()) return;
    QFile::remove(history_log_path_);
    history_log_.setFileName(history_log_path_);
    if (!history_log_.open(QIODevice::WriteOnly | QIODevice::Text)) {
        emit event_message(QString("CSV history stream unavailable: %1").arg(history_log_.errorString()));
        return;
    }
    QTextStream output(&history_log_);
    write_csv_header(output);
    output.flush();
}

void SimulationWorker::append_history_sample(const GuiSnapshot& snapshot) {
    // State-changing GUI events can emit a snapshot without advancing the
    // simulation clock.  They belong in the event log, not as duplicate rows
    // in the motion time series.
    if (have_logged_time_ && snapshot.time_s <= last_logged_time_s_ + 1e-12) return;
    have_logged_time_ = true;
    last_logged_time_s_ = snapshot.time_s;
    if (history_log_.isOpen()) {
        QTextStream output(&history_log_);
        write_csv_row(output, snapshot);
        output.flush();
    }
    samples_.push_back({snapshot});
    constexpr int kDisplayCacheLimit = 20000;
    if (samples_.size() > kDisplayCacheLimit)
        samples_.remove(0, samples_.size() - kDisplayCacheLimit);
}

void SimulationWorker::write_csv_header(QTextStream& output) {
    output << "time_s,motion_state,command_id,command_status,fault_code,fault_active,"
              "x_ref_mm,y_ref_mm,z_ref_mm,"
              "x_actual_mm,y_actual_mm,z_actual_mm,vx_actual_mm_s,vy_actual_mm_s,vz_actual_mm_s,"
              "ax_actual_mm_s2,ay_actual_mm_s2,az_actual_mm_s2,following_error_mm,"
              "drive_force_x_n,drive_force_y_n,drive_force_z_n,homed_x,homed_y,homed_z,"
              "soft_limit_margin_x_mm,soft_limit_margin_y_mm,soft_limit_margin_z_mm,"
              "vx_ref_mm_s,vy_ref_mm_s,vz_ref_mm_s,ax_ref_mm_s2,ay_ref_mm_s2,az_ref_mm_s2,"
              "jx_ref_mm_s3,jy_ref_mm_s3,jz_ref_mm_s3,"
              "jx_actual_est_mm_s3,jy_actual_est_mm_s3,jz_actual_est_mm_s3\n";
}

void SimulationWorker::write_csv_row(QTextStream& output, const GuiSnapshot& s) {
    output << QString::number(s.time_s, 'f', 6) << ',' << s.state << ','
           << s.command_id << ',' << s.command_status << ',' << s.fault << ','
           << (s.fault_active ? "1" : "0") << ','
           << QString::number(s.x_ref_mm, 'g', 12) << ','
           << QString::number(s.y_ref_mm, 'g', 12) << ','
           << QString::number(s.z_ref_mm, 'g', 12) << ','
           << QString::number(s.x_mm, 'g', 12) << ','
           << QString::number(s.y_mm, 'g', 12) << ','
           << QString::number(s.z_mm, 'g', 12) << ','
           << QString::number(s.vx_mm_s, 'g', 12) << ','
           << QString::number(s.vy_mm_s, 'g', 12) << ','
           << QString::number(s.vz_mm_s, 'g', 12) << ','
           << QString::number(s.ax_mm_s2, 'g', 12) << ','
           << QString::number(s.ay_mm_s2, 'g', 12) << ','
           << QString::number(s.az_mm_s2, 'g', 12) << ','
           << QString::number(s.following_error_mm, 'g', 12) << ','
           << QString::number(s.drive_x_n, 'g', 12) << ','
           << QString::number(s.drive_y_n, 'g', 12) << ','
           << QString::number(s.drive_z_n, 'g', 12) << ','
           << (s.homed_x ? "1" : "0") << ','
           << (s.homed_y ? "1" : "0") << ','
           << (s.homed_z ? "1" : "0") << ','
           << QString::number(s.soft_limit_margin_x_mm, 'g', 12) << ','
           << QString::number(s.soft_limit_margin_y_mm, 'g', 12) << ','
           << QString::number(s.soft_limit_margin_z_mm, 'g', 12);
    const double additional[] = {s.vx_ref_mm_s, s.vy_ref_mm_s, s.vz_ref_mm_s,
        s.ax_ref_mm_s2, s.ay_ref_mm_s2, s.az_ref_mm_s2,
        s.jx_ref_mm_s3, s.jy_ref_mm_s3, s.jz_ref_mm_s3,
        s.jx_actual_est_mm_s3, s.jy_actual_est_mm_s3, s.jz_actual_est_mm_s3};
    for (double value : additional) output << ',' << QString::number(value, 'g', 12);
    output << '\n';
}


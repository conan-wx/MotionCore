#include "simulation_worker.hpp"

#include <QApplication>
#include <QFile>
#include <QTemporaryDir>
#include <QTextStream>

#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace {

bool tick(SimulationWorker& worker) {
    return QMetaObject::invokeMethod(&worker, "tick", Qt::DirectConnection);
}

bool home_to_ready(SimulationWorker& worker) {
    worker.enable_controller();
    worker.home_axes();
    worker.set_speed(20.0);
    worker.set_running(true);
    for (int i = 0; i < 80; ++i) {
        if (!tick(worker)) return false;
    }
    return true;
}

bool has_event(const std::vector<QString>& events, const QString& needle) {
    for (const auto& event : events) {
        if (event.contains(needle)) return true;
    }
    return false;
}

bool csv_has_strictly_increasing_time(const QString& path, int* rows) {
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return false;
    QTextStream input(&file);
    const auto header = input.readLine();
    if (!header.startsWith("time_s,motion_state,command_id,command_status")) return false;
    double previous = -std::numeric_limits<double>::infinity();
    int count = 0;
    while (!input.atEnd()) {
        const auto line = input.readLine();
        if (line.trimmed().isEmpty()) continue;
        const auto fields = line.split(',');
        if (fields.size() < 5) return false;
        bool ok = false;
        const double time = fields[0].toDouble(&ok);
        if (!ok || !std::isfinite(time) || (count > 0 && time <= previous)) return false;
        previous = time;
        ++count;
    }
    if (rows) *rows = count;
    return count > 1;
}

}  // namespace

int main(int argc, char** argv) {
    QApplication application(argc, argv);
    qRegisterMetaType<GuiSnapshot>();

    // Stop and EmergencyStop must be serviced while ordinary advancement is
    // paused.  The worker's zero-time control path keeps the time stamp fixed.
    {
        SimulationWorker worker;
        GuiSnapshot latest;
        std::vector<QString> events;
        QObject::connect(&worker, &SimulationWorker::snapshot_ready,
                         [&](const GuiSnapshot& snapshot) { latest = snapshot; });
        QObject::connect(&worker, &SimulationWorker::event_message,
                         [&](const QString& event) { events.push_back(event); });
        worker.start();
        if (!home_to_ready(worker) || latest.state != "Ready") {
            std::cerr << "homing did not reach Ready\n";
            return 1;
        }
        worker.move_line(50.0, 30.0, 10.0, 50.0, 100.0, 500.0, true);
        if (!tick(worker) || latest.state != "Running") {
            std::cerr << "line did not enter Running\n";
            return 1;
        }
        worker.set_running(false);
        const double paused_time = latest.time_s;
        worker.controlled_stop();
        if (latest.state != "ControlledStopping" || latest.time_s != paused_time) {
            std::cerr << "paused Stop was not processed without a time step\n";
            return 1;
        }
        for (int i = 0; i < 80 && latest.state != "Ready"; ++i) tick(worker);
        if (latest.state != "Ready") {
            std::cerr << "controlled stop did not finish after resume\n";
            return 1;
        }

        worker.move_line(50.0, 30.0, 10.0, 50.0, 100.0, 500.0, true);
        tick(worker);
        worker.set_running(false);
        const double before_emergency_x = latest.x_mm;
        const double emergency_time = latest.time_s;
        worker.emergency_stop();
        if (latest.state != "EmergencyStopped" || latest.time_s != emergency_time) {
            std::cerr << "paused EmergencyStop was not latched immediately\n";
            return 1;
        }
        for (int i = 0; i < 8; ++i) tick(worker);
        if (std::abs(latest.x_mm - before_emergency_x) < 1e-12 &&
            std::abs(latest.vx_mm_s) < 1e-12) {
            std::cerr << "emergency inertial dynamics did not continue\n";
            return 1;
        }
        if (!has_event(events, "Controlled Stop requested") ||
            !has_event(events, "Emergency Stop requested")) {
            std::cerr << "stop event log is incomplete\n";
            return 1;
        }
    }

    // Absolute GUI commands are chained through accepted endpoints, allowing
    // a line -> arc -> line FIFO to be queued before the first step.
    {
        SimulationWorker worker;
        std::vector<QString> events;
        GuiSnapshot latest;
        QObject::connect(&worker, &SimulationWorker::event_message,
                         [&](const QString& event) { events.push_back(event); });
        QObject::connect(&worker, &SimulationWorker::snapshot_ready,
                         [&](const GuiSnapshot& snapshot) { latest = snapshot; });
        worker.start();
        if (!home_to_ready(worker) || latest.state != "Ready") return 1;
        worker.move_line(50.0, 30.0, 10.0, 40.0, 80.0, 400.0, true);
        worker.move_arc_demo(40.0, 80.0, 400.0, true);
        worker.move_line(10.0, 10.0, 10.0, 40.0, 80.0, 400.0, true);
        if (!has_event(events, "MoveLine queued id=") ||
            !has_event(events, "Arc queued id=")) {
            std::cerr << "continuous queue command was rejected at enqueue\n";
            return 1;
        }
        worker.set_speed(20.0);
        worker.set_running(true);
        for (int i = 0; i < 180 && latest.state != "Ready"; ++i) tick(worker);
        if (latest.state != "Ready" || has_event(events, "rejected")) {
            std::cerr << "line-arc-line queue did not complete FIFO\n";
            return 1;
        }

        QTemporaryDir temporary;
        const QString csv = temporary.filePath("gui_regression.csv");
        bool exported = false;
        QObject::connect(&worker, &SimulationWorker::csv_exported,
                         [&](const QString&, bool ok, const QString&) { exported = ok; });
        worker.export_csv(csv);
        int rows = 0;
        if (!exported || !csv_has_strictly_increasing_time(csv, &rows) || rows < 2) {
            std::cerr << "GUI CSV time axis is not strictly increasing\n";
            return 1;
        }
        // The exported file is also used as a real offline replay input.  A
        // replay step only emits a stored sample; it never calls the motion
        // controller, so it cannot create a new command or alter mechanics.
        worker.load_replay_csv(csv);
        worker.set_running(true);
        tick(worker);
        const double replay_start = latest.time_s;
        tick(worker);
        if (!(latest.time_s > replay_start) || !has_event(events, "Offline replay loaded")) {
            std::cerr << "offline CSV replay did not advance validated samples\n";
            return 1;
        }
        const auto replay_command_count = latest.command_id;
        worker.move_line(1.0, 1.0, 1.0, 20.0, 40.0, 200.0, true);
        if (latest.command_id != replay_command_count ||
            !has_event(events, "offline replay mode")) {
            std::cerr << "offline replay accepted a motion command\n";
            return 1;
        }
        std::cout << "phase6_gui_regression_test passed rows=" << rows
                  << " final_state=" << latest.state.toStdString() << "\n";
    }
    return 0;
}

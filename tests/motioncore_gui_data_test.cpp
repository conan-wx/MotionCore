#include "simulation_worker.hpp"

#include <QApplication>
#include <QFile>
#include <QTemporaryDir>

#include <cmath>
#include <iostream>

int main(int argc, char** argv) {
    QApplication application(argc, argv);
    qRegisterMetaType<GuiSnapshot>();

    SimulationWorker worker;
    GuiSnapshot latest;
    int snapshots = 0;
    QObject::connect(&worker, &SimulationWorker::snapshot_ready,
                     [&](const GuiSnapshot& snapshot) {
                         latest = snapshot;
                         ++snapshots;
                     });

    worker.start();
    if (snapshots < 1 || latest.state != "Disabled") {
        std::cerr << "initial snapshot was not delivered\n";
        return 1;
    }

    worker.enable_controller();
    if (latest.state != "NotHomed") {
        std::cerr << "enable did not produce NotHomed\n";
        return 1;
    }

    worker.home_axes();
    if (latest.state != "Homing") {
        std::cerr << "home command did not produce Homing\n";
        return 1;
    }

    // The GUI-facing command path must reject non-finite values instead of
    // passing them into the numerical controller.
    worker.move_line(NAN, 40.0, 50.0, 50.0, 100.0, 500.0, true);
    if (latest.command_status == "running" || latest.command_status == "completed") {
        std::cerr << "non-finite target was accepted\n";
        return 1;
    }

    worker.set_running(false);
    QTemporaryDir temporary;
    const QString csv_path = temporary.filePath("gui_data.csv");
    bool csv_ok = false;
    QObject::connect(&worker, &SimulationWorker::csv_exported,
                     [&](const QString&, bool ok, const QString&) { csv_ok = ok; });
    worker.export_csv(csv_path);
    if (!csv_ok || !QFile::exists(csv_path)) {
        std::cerr << "GUI CSV export did not complete\n";
        return 1;
    }

    std::cout << "phase6_gui_data_test passed snapshots=" << snapshots
              << " state=" << latest.state.toStdString() << "\n";
    return 0;
}

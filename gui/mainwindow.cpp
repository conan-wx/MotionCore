#include "mainwindow.hpp"

#include "trajectory_view.hpp"
#include "telemetry_view.hpp"

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSettings>
#include <QSpinBox>
#include <QSplitter>
#include <QStatusBar>
#include <QThread>
#include <QVBoxLayout>
#include <QWidget>

#include <array>

namespace {

QDoubleSpinBox* make_parameter(double value, double min, double max, double step,
                               QWidget* parent) {
    auto* box = new QDoubleSpinBox(parent);
    box->setRange(min, max);
    box->setSingleStep(step);
    box->setDecimals(4);
    box->setValue(value);
    return box;
}

}  // namespace

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
    qRegisterMetaType<GuiSnapshot>();
    build_ui();
    worker_thread_ = new QThread(this);
    worker_ = new SimulationWorker();
    worker_->moveToThread(worker_thread_);
    connect(worker_thread_, &QThread::started, worker_, &SimulationWorker::start);
    connect(worker_, &SimulationWorker::snapshot_ready,
            this, &MainWindow::update_snapshot, Qt::QueuedConnection);
    connect(worker_, &SimulationWorker::event_message,
            this, &MainWindow::append_event, Qt::QueuedConnection);
    connect(worker_, &SimulationWorker::csv_exported, this,
            [this](const QString& path, bool ok, const QString& error) {
                append_event(ok ? QString("CSV exported: %1").arg(path)
                                : QString("CSV export failed: %1").arg(error));
            }, Qt::QueuedConnection);
    worker_thread_->start();
    setWindowTitle("MotionCore — XYZ 工业运动控制仿真");
    resize(1280, 820);
}

MainWindow::~MainWindow() {
    if (worker_thread_) {
        auto* worker = worker_;
        QMetaObject::invokeMethod(worker, "set_running", Qt::BlockingQueuedConnection,
                                  Q_ARG(bool, false));
        // Delete the worker and its QTimer in the worker thread. Deleting a
        // QObject with timer children from the GUI thread produces
        // cross-thread killTimer warnings during normal window shutdown.
        QMetaObject::invokeMethod(worker, [worker] { delete worker; },
                                  Qt::BlockingQueuedConnection);
        worker_ = nullptr;
        worker_thread_->quit();
        worker_thread_->wait();
    }
}

void MainWindow::build_ui() {
    auto* central = new QWidget(this);
    auto* root = new QVBoxLayout(central);

    auto* status_box = new QGroupBox("控制器状态", central);
    auto* status_layout = new QGridLayout(status_box);
    state_label_ = new QLabel("Disabled", status_box);
    command_label_ = new QLabel("-", status_box);
    time_label_ = new QLabel("0.000 s", status_box);
    fault_label_ = new QLabel("None", status_box);
    status_layout->addWidget(new QLabel("状态"), 0, 0);
    status_layout->addWidget(state_label_, 0, 1);
    status_layout->addWidget(new QLabel("当前指令"), 0, 2);
    status_layout->addWidget(command_label_, 0, 3);
    status_layout->addWidget(new QLabel("仿真时间"), 0, 4);
    status_layout->addWidget(time_label_, 0, 5);
    status_layout->addWidget(new QLabel("故障"), 0, 6);
    status_layout->addWidget(fault_label_, 0, 7);
    root->addWidget(status_box);

    auto* controls = new QGroupBox("运动指令", central);
    auto* controls_layout = new QGridLayout(controls);
    const QStringList axes{"X (mm)", "Y (mm)", "Z (mm)"};
    for (int i = 0; i < 3; ++i) {
        target_edits_[i] = new QLineEdit(i == 0 ? "30" : (i == 1 ? "40" : "50"), controls);
        target_edits_[i]->setPlaceholderText(axes[i]);
        controls_layout->addWidget(new QLabel(axes[i]), 0, i * 2);
        controls_layout->addWidget(target_edits_[i], 0, i * 2 + 1);
    }
    vmax_ = make_parameter(50.0, 0.01, 500.0, 1.0, controls);
    amax_ = make_parameter(100.0, 0.01, 2000.0, 5.0, controls);
    jmax_ = make_parameter(500.0, 0.01, 10000.0, 25.0, controls);
    profile_combo_ = new QComboBox(controls);
    profile_combo_->addItems({"Trapezoid", "S-Curve"});
    profile_combo_->setCurrentIndex(1);
    controls_layout->addWidget(new QLabel("Vmax"), 1, 0);
    controls_layout->addWidget(vmax_, 1, 1);
    controls_layout->addWidget(new QLabel("Amax"), 1, 2);
    controls_layout->addWidget(amax_, 1, 3);
    controls_layout->addWidget(new QLabel("Jmax"), 1, 4);
    controls_layout->addWidget(jmax_, 1, 5);
    controls_layout->addWidget(new QLabel("规划器"), 1, 6);
    controls_layout->addWidget(profile_combo_, 1, 7);

    auto* enable = new QPushButton("Enable", controls);
    auto* home = new QPushButton("Home", controls);
    auto* move = new QPushButton("Move Line", controls);
    auto* arc = new QPushButton("Arc Demo", controls);
    auto* stop = new QPushButton("Stop", controls);
    auto* emergency = new QPushButton("EmergencyStop", controls);
    auto* reset = new QPushButton("Reset", controls);
    auto* pause = new QPushButton("Pause/Continue", controls);
    auto* restart = new QPushButton("Restart", controls);
    auto* csv = new QPushButton("Export CSV", controls);
    auto* image = new QPushButton("Save Trajectory", controls);
    auto* save_params = new QPushButton("Save Params", controls);
    auto* load_params = new QPushButton("Load Params", controls);
    auto* load_replay = new QPushButton("Load CSV Replay", controls);
    auto* replay_pause = new QPushButton("Replay Pause/Continue", controls);
    const std::array<QPushButton*, 15> buttons{enable, home, move, arc, stop, emergency,
                                                reset, pause, restart, csv, image,
                                                save_params, load_params, load_replay, replay_pause};
    for (int i = 0; i < static_cast<int>(buttons.size()); ++i) {
        control_buttons_[i] = buttons[static_cast<std::size_t>(i)];
        controls_layout->addWidget(buttons[static_cast<std::size_t>(i)], 2 + i / 7, i % 7);
    }
    connect(enable, &QPushButton::clicked, this, [this] { invoke_worker("enable_controller"); });
    connect(home, &QPushButton::clicked, this, [this] { invoke_worker("home_axes"); });
    connect(move, &QPushButton::clicked, this, [this] {
        bool ok[3]{};
        const double x = target_edits_[0]->text().toDouble(&ok[0]);
        const double y = target_edits_[1]->text().toDouble(&ok[1]);
        const double z = target_edits_[2]->text().toDouble(&ok[2]);
        if (!ok[0] || !ok[1] || !ok[2]) { append_event("Move rejected: invalid target"); return; }
        QMetaObject::invokeMethod(worker_, "move_line", Qt::QueuedConnection,
                                  Q_ARG(double, x), Q_ARG(double, y), Q_ARG(double, z),
                                  Q_ARG(double, vmax_->value()), Q_ARG(double, amax_->value()),
                                  Q_ARG(double, jmax_->value()),
                                  Q_ARG(bool, profile_combo_->currentIndex() == 1));
    });
    connect(arc, &QPushButton::clicked, this, [this] {
        QMetaObject::invokeMethod(worker_, "move_arc_demo", Qt::QueuedConnection,
                                  Q_ARG(double, vmax_->value()), Q_ARG(double, amax_->value()),
                                  Q_ARG(double, jmax_->value()),
                                  Q_ARG(bool, profile_combo_->currentIndex() == 1));
    });
    connect(stop, &QPushButton::clicked, this, [this] { invoke_worker("controlled_stop"); });
    connect(emergency, &QPushButton::clicked, this, [this] { invoke_worker("emergency_stop"); });
    connect(reset, &QPushButton::clicked, this, [this] { invoke_worker("reset_fault_or_emergency"); });
    connect(pause, &QPushButton::clicked, this, [this] {
        static bool running = true;
        running = !running;
        QMetaObject::invokeMethod(worker_, "set_running", Qt::QueuedConnection, Q_ARG(bool, running));
        append_event(running ? "Simulation continued" : "Simulation paused");
    });
    connect(restart, &QPushButton::clicked, this, [this] { invoke_worker("reset_simulation"); });
    connect(csv, &QPushButton::clicked, this, &MainWindow::export_csv);
    connect(image, &QPushButton::clicked, this, &MainWindow::save_trajectory_image);
    connect(save_params, &QPushButton::clicked, this, &MainWindow::save_parameters);
    connect(load_params, &QPushButton::clicked, this, &MainWindow::load_parameters);
    connect(load_replay, &QPushButton::clicked, this, &MainWindow::load_replay_csv);
    connect(replay_pause, &QPushButton::clicked, this, &MainWindow::toggle_replay);
    root->addWidget(controls);

    auto* splitter = new QSplitter(Qt::Horizontal, central);
    trajectory_view_ = new TrajectoryView(splitter);
    auto* telemetry_panel = new QWidget(splitter);
    auto* telemetry_layout = new QVBoxLayout(telemetry_panel);
    curve_combo_ = new QComboBox(telemetry_panel);
    telemetry_view_ = new TelemetryView(telemetry_panel);
    curve_combo_->addItems({"Position", "Velocity", "Acceleration", "Following error", "Drive force"});
    telemetry_view_->set_metric(curve_combo_->currentText());
    connect(curve_combo_, &QComboBox::currentTextChanged,
            telemetry_view_, &TelemetryView::set_metric);
    telemetry_layout->addWidget(curve_combo_);
    telemetry_layout->addWidget(telemetry_view_, 1);
    splitter->addWidget(trajectory_view_);
    splitter->addWidget(telemetry_panel);
    splitter->setStretchFactor(0, 1);
    splitter->setStretchFactor(1, 1);
    root->addWidget(splitter, 1);

    event_log_ = new QPlainTextEdit(central);
    event_log_->setReadOnly(true);
    event_log_->setMaximumBlockCount(300);
    event_log_->setPlaceholderText("事件日志");
    root->addWidget(event_log_);
    setCentralWidget(central);
    statusBar()->showMessage("仿真 1 ms 步长，GUI 刷新约 50 FPS");
}

void MainWindow::invoke_worker(const char* method) {
    QMetaObject::invokeMethod(worker_, method, Qt::QueuedConnection);
}

void MainWindow::update_snapshot(const GuiSnapshot& snapshot) {
    state_label_->setText(snapshot.state);
    command_label_->setText(snapshot.command_id == 0
                                 ? "-"
                                 : QString("%1 (%2)").arg(snapshot.command_id).arg(snapshot.command_status));
    time_label_->setText(QString::number(snapshot.time_s, 'f', 3) + " s");
    fault_label_->setText(snapshot.fault);
    trajectory_view_->append(snapshot.x_ref_mm, snapshot.y_ref_mm, snapshot.x_mm, snapshot.y_mm);
    telemetry_view_->append(snapshot);
    configure_buttons(snapshot.state);
}

void MainWindow::configure_buttons(const QString& state) {
    const bool disabled = state == "Disabled";
    const bool homing = state == "Homing";
    const bool not_homed = state == "NotHomed";
    const bool ready = state == "Ready";
    const bool running = state == "Running";
    statusBar()->showMessage(QString("%1 | 1 ms simulation clock").arg(state));
    // The core still checks every precondition; this only reflects the current
    // state in the UI and prevents accidental clicks.
    if (centralWidget()) {
        const auto edits = centralWidget()->findChildren<QLineEdit*>();
        for (auto* edit : edits) edit->setEnabled(ready || running);
    }
    // Keep the visible command affordances consistent with the same state
    // preconditions enforced again inside MotionController.
    if (control_buttons_[0]) control_buttons_[0]->setEnabled(disabled);
    if (control_buttons_[1]) control_buttons_[1]->setEnabled(not_homed);
    if (control_buttons_[2]) control_buttons_[2]->setEnabled(ready || running);
    if (control_buttons_[3]) control_buttons_[3]->setEnabled(ready || running);
    if (control_buttons_[4]) control_buttons_[4]->setEnabled(running || homing);
    if (control_buttons_[5]) control_buttons_[5]->setEnabled(running || homing || ready);
    if (control_buttons_[6]) control_buttons_[6]->setEnabled(state == "Fault" || state == "EmergencyStopped");
    if (control_buttons_[7]) control_buttons_[7]->setEnabled(!disabled);
    if (control_buttons_[8]) control_buttons_[8]->setEnabled(true);
    for (int i = 9; i < 15; ++i)
        if (control_buttons_[i]) control_buttons_[i]->setEnabled(true);
}

void MainWindow::append_event(const QString& message) {
    event_log_->appendPlainText(message);
}

void MainWindow::export_csv() {
    const auto path = QFileDialog::getSaveFileName(this, "Export CSV", "motioncore_gui.csv", "CSV (*.csv)");
    if (!path.isEmpty()) QMetaObject::invokeMethod(worker_, "export_csv", Qt::QueuedConnection,
                                                  Q_ARG(QString, path));
}

QString MainWindow::parameter_file() const { return "motioncore_gui.ini"; }

void MainWindow::save_parameters() {
    QSettings settings(parameter_file(), QSettings::IniFormat);
    settings.setValue("vmax", vmax_->value());
    settings.setValue("amax", amax_->value());
    settings.setValue("jmax", jmax_->value());
    settings.setValue("profile", profile_combo_->currentIndex());
    for (int i = 0; i < 3; ++i) settings.setValue(QString("target%1").arg(i), target_edits_[i]->text());
    append_event("Parameters saved");
}

void MainWindow::load_parameters() {
    QSettings settings(parameter_file(), QSettings::IniFormat);
    vmax_->setValue(settings.value("vmax", vmax_->value()).toDouble());
    amax_->setValue(settings.value("amax", amax_->value()).toDouble());
    jmax_->setValue(settings.value("jmax", jmax_->value()).toDouble());
    profile_combo_->setCurrentIndex(settings.value("profile", 1).toInt());
    for (int i = 0; i < 3; ++i) target_edits_[i]->setText(settings.value(QString("target%1").arg(i), target_edits_[i]->text()).toString());
    append_event("Parameters loaded");
}

void MainWindow::save_trajectory_image() {
    const auto path = QFileDialog::getSaveFileName(this, "Save trajectory image", "trajectory.png", "PNG (*.png)");
    if (!path.isEmpty() && !trajectory_view_->grab().save(path)) append_event("Trajectory image save failed");
}

void MainWindow::load_replay_csv() {
    const auto path = QFileDialog::getOpenFileName(this, "Load MotionCore CSV",
                                                   QString(), "CSV (*.csv)");
    if (!path.isEmpty())
        QMetaObject::invokeMethod(worker_, "load_replay_csv", Qt::QueuedConnection,
                                  Q_ARG(QString, path));
}

void MainWindow::toggle_replay() {
    static bool playing = false;
    playing = !playing;
    QMetaObject::invokeMethod(worker_, "set_running", Qt::QueuedConnection,
                              Q_ARG(bool, playing));
    append_event(playing ? "Offline replay continued" : "Offline replay paused");
}


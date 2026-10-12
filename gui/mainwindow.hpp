#pragma once

#include "simulation_worker.hpp"

#include <QMainWindow>

class QLabel;
class QLineEdit;
class QComboBox;
class QPlainTextEdit;
class QDoubleSpinBox;
class QPushButton;
class QThread;
class QTimer;
class TrajectoryView;
class TelemetryView;

class MainWindow final : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

private slots:
    void update_snapshot(const GuiSnapshot& snapshot);
    void append_event(const QString& message);
    void export_csv();
    void save_parameters();
    void load_parameters();
    void save_trajectory_image();
    void load_replay_csv();
    void toggle_replay();

private:
    void invoke_worker(const char* method);
    void configure_buttons(const QString& state);
    void build_ui();
    QString parameter_file() const;

    QThread* worker_thread_{nullptr};
    SimulationWorker* worker_{nullptr};

    QLabel* state_label_{nullptr};
    QLabel* command_label_{nullptr};
    QLabel* time_label_{nullptr};
    QLabel* fault_label_{nullptr};
    QLineEdit* target_edits_[3]{};
    QDoubleSpinBox* vmax_{nullptr};
    QDoubleSpinBox* amax_{nullptr};
    QDoubleSpinBox* jmax_{nullptr};
    QComboBox* profile_combo_{nullptr};
    QComboBox* curve_combo_{nullptr};
    QPushButton* control_buttons_[15]{};
    QPlainTextEdit* event_log_{nullptr};
    TrajectoryView* trajectory_view_{nullptr};
    TelemetryView* telemetry_view_{nullptr};
};


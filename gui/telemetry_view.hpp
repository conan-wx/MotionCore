#pragma once

#include "simulation_worker.hpp"

#include <QWidget>

#include <QVector>

class TelemetryView final : public QWidget {
    Q_OBJECT
public:
    explicit TelemetryView(QWidget* parent = nullptr);
    void append(const GuiSnapshot& snapshot);

public slots:
    void set_metric(const QString& metric);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    struct Sample {
        double time{0.0};
        double x{0.0}; double y{0.0}; double z{0.0};
        double vx{0.0}; double vy{0.0}; double vz{0.0};
        double ax{0.0}; double ay{0.0}; double az{0.0};
        double error{0.0}; double drive{0.0};
    };
    QVector<Sample> samples_;
    QString metric_{"Position"};
};


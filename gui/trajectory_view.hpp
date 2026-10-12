#pragma once

#include <QWidget>

#include <QVector>

class TrajectoryView final : public QWidget {
    Q_OBJECT
public:
    explicit TrajectoryView(QWidget* parent = nullptr);
    void append(double x_ref, double y_ref, double x_actual, double y_actual);
    void clear();

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    struct Point { double x{0.0}; double y{0.0}; };
    QVector<Point> reference_;
    QVector<Point> actual_;
};


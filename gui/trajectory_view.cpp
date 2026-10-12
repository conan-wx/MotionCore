#include "trajectory_view.hpp"

#include <QPainter>
#include <QPainterPath>
#include <QPaintEvent>

#include <algorithm>
#include <cmath>

TrajectoryView::TrajectoryView(QWidget* parent) : QWidget(parent) {
    setMinimumSize(420, 300);
    setAutoFillBackground(true);
}

void TrajectoryView::append(double x_ref, double y_ref, double x_actual, double y_actual) {
    reference_.push_back({x_ref, y_ref});
    actual_.push_back({x_actual, y_actual});
    if (reference_.size() > 10000) {
        reference_.remove(0, reference_.size() - 10000);
        actual_.remove(0, actual_.size() - 10000);
    }
    update();
}

void TrajectoryView::clear() {
    reference_.clear();
    actual_.clear();
    update();
}

void TrajectoryView::paintEvent(QPaintEvent*) {
    QPainter painter(this);
    painter.fillRect(rect(), QColor(24, 28, 34));
    painter.setRenderHint(QPainter::Antialiasing);
    const QRectF plot = rect().adjusted(48, 24, -24, -42);
    painter.setPen(QPen(QColor(100, 110, 120), 1));
    painter.drawRect(plot);
    painter.setPen(Qt::white);
    painter.drawText(12, 18, "XY trajectory (mm)");
    painter.drawText(55, height() - 12, "X");
    painter.drawText(12, 30, "Y");
    if (reference_.isEmpty()) return;
    double min_x = 0.0, max_x = 1.0, min_y = 0.0, max_y = 1.0;
    auto grow = [&](const QVector<Point>& points) {
        for (const auto& point : points) {
            min_x = std::min(min_x, point.x); max_x = std::max(max_x, point.x);
            min_y = std::min(min_y, point.y); max_y = std::max(max_y, point.y);
        }
    };
    grow(reference_); grow(actual_);
    const double margin_x = std::max(1.0, (max_x - min_x) * .1);
    const double margin_y = std::max(1.0, (max_y - min_y) * .1);
    min_x -= margin_x; max_x += margin_x; min_y -= margin_y; max_y += margin_y;
    const auto map = [&](const Point& p) {
        const double x = plot.left() + (p.x - min_x) / (max_x - min_x) * plot.width();
        const double y = plot.bottom() - (p.y - min_y) / (max_y - min_y) * plot.height();
        return QPointF(x, y);
    };
    auto draw = [&](const QVector<Point>& points, const QPen& pen) {
        if (points.size() < 2) return;
        QPainterPath path(map(points.front()));
        for (int i = 1; i < points.size(); ++i) path.lineTo(map(points[i]));
        painter.setPen(pen); painter.drawPath(path);
    };
    draw(reference_, QPen(QColor(80, 180, 255), 2, Qt::DashLine));
    draw(actual_, QPen(QColor(80, 230, 130), 2));
    painter.setPen(Qt::white);
    painter.drawText(plot.left(), plot.bottom() + 28, "reference");
    painter.setPen(QColor(80, 180, 255));
    painter.drawLine(plot.left() + 70, plot.bottom() + 23, plot.left() + 100, plot.bottom() + 23);
    painter.setPen(QColor(80, 230, 130));
    painter.drawText(plot.left() + 112, plot.bottom() + 28, "actual");
}


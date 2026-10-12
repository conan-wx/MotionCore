#include "telemetry_view.hpp"

#include <QPainter>
#include <QPainterPath>

#include <algorithm>
#include <cmath>

TelemetryView::TelemetryView(QWidget* parent) : QWidget(parent) {
    setMinimumSize(420, 300);
    setAutoFillBackground(true);
}

void TelemetryView::append(const GuiSnapshot& snapshot) {
    samples_.push_back({snapshot.time_s, snapshot.x_mm, snapshot.y_mm, snapshot.z_mm,
                        snapshot.vx_mm_s, snapshot.vy_mm_s, snapshot.vz_mm_s,
                        snapshot.ax_mm_s2, snapshot.ay_mm_s2, snapshot.az_mm_s2,
                        snapshot.following_error_mm,
                        std::hypot(snapshot.drive_x_n, std::hypot(snapshot.drive_y_n, snapshot.drive_z_n))});
    if (samples_.size() > 10000) samples_.remove(0, samples_.size() - 10000);
    update();
}

void TelemetryView::set_metric(const QString& metric) {
    metric_ = metric;
    update();
}

void TelemetryView::paintEvent(QPaintEvent*) {
    QPainter painter(this);
    painter.fillRect(rect(), QColor(24, 28, 34));
    painter.setRenderHint(QPainter::Antialiasing);
    const QRectF plot = rect().adjusted(48, 24, -24, -42);
    painter.setPen(QPen(QColor(100, 110, 120), 1));
    painter.drawRect(plot);
    painter.setPen(Qt::white);
    painter.drawText(12, 18, metric_ + " (simulation)" );
    if (samples_.size() < 2) return;
    double max_time = samples_.back().time;
    double min_time = samples_.front().time;
    if (max_time <= min_time) max_time = min_time + 1.0;
    const bool velocity_metric = metric_.contains("velocity", Qt::CaseInsensitive);
    const bool acceleration_metric = metric_.contains("acceleration", Qt::CaseInsensitive);
    const bool error_metric = metric_.contains("error", Qt::CaseInsensitive);
    const bool drive_metric = metric_.contains("drive", Qt::CaseInsensitive);
    auto value = [&](const Sample& s, int axis) {
        if (velocity_metric) return axis == 0 ? s.vx : (axis == 1 ? s.vy : s.vz);
        if (acceleration_metric) return axis == 0 ? s.ax : (axis == 1 ? s.ay : s.az);
        if (error_metric) return s.error;
        if (drive_metric) return s.drive;
        return axis == 0 ? s.x : (axis == 1 ? s.y : s.z);
    };
    double min_value = 0.0, max_value = 1.0;
    for (const auto& s : samples_) for (int axis = 0; axis < (error_metric || drive_metric ? 1 : 3); ++axis) {
        min_value = std::min(min_value, value(s, axis));
        max_value = std::max(max_value, value(s, axis));
    }
    if (std::abs(max_value - min_value) < 1e-9) max_value = min_value + 1.0;
    const auto map = [&](const Sample& s, double v) {
        const double x = plot.left() + (s.time - min_time) / (max_time - min_time) * plot.width();
        const double y = plot.bottom() - (v - min_value) / (max_value - min_value) * plot.height();
        return QPointF(x, y);
    };
    const QPen pens[] = {QPen(QColor(235, 90, 90), 2), QPen(QColor(90, 190, 255), 2),
                         QPen(QColor(100, 230, 130), 2)};
    const int count = (error_metric || drive_metric) ? 1 : 3;
    for (int axis = 0; axis < count; ++axis) {
        QPainterPath path(map(samples_.front(), value(samples_.front(), axis)));
        for (int i = 1; i < samples_.size(); ++i) path.lineTo(map(samples_[i], value(samples_[i], axis)));
        painter.setPen(pens[axis]); painter.drawPath(path);
    }
    painter.setPen(Qt::white);
    painter.drawText(plot.left(), plot.bottom() + 28, "time (s)");
}


#include "motioncore/control/p_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace motioncore::control {

PController::PController(PControllerConfig config) : config_(config) {
    if (!std::isfinite(config_.kp)
        || !std::isfinite(config_.output_min)
        || !std::isfinite(config_.output_max)
        || config_.kp < 0.0
        || config_.output_min > config_.output_max) {
        throw std::invalid_argument("invalid P controller configuration");
    }
}

double PController::update(double error) {
    if (!std::isfinite(error)) {
        throw std::invalid_argument("P controller error must be finite");
    }
    // 比例环只看当前误差；输出限幅保证后级执行器不会收到非法指令。
    last_output_ = std::clamp(config_.kp * error,
                              config_.output_min,
                              config_.output_max);
    return last_output_;
}

}  // namespace motioncore::control

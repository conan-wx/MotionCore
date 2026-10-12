#include "motioncore/control/pi_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace motioncore::control {

PIController::PIController(PIControllerConfig config) : config_(config) {
    if (!std::isfinite(config_.kp) || !std::isfinite(config_.ki)
        || !std::isfinite(config_.output_min) || !std::isfinite(config_.output_max)
        || !std::isfinite(config_.integral_min) || !std::isfinite(config_.integral_max)
        || config_.kp < 0.0 || config_.ki < 0.0
        || config_.output_min > config_.output_max
        || config_.integral_min > config_.integral_max) {
        throw std::invalid_argument("invalid PI controller configuration");
    }
}

void PIController::reset() noexcept {
    integral_state_ = 0.0;
    last_output_ = 0.0;
}

double PIController::update(double error, double dt_seconds) {
    if (!std::isfinite(error)) {
        throw std::invalid_argument("PI controller error must be finite");
    }
    if (!std::isfinite(dt_seconds) || dt_seconds <= 0.0) {
        throw std::invalid_argument("dt_seconds must be finite and positive");
    }

    const double candidate_integral = std::clamp(
        integral_state_ + error * dt_seconds,
        config_.integral_min,
        config_.integral_max);
    const double candidate_output = config_.kp * error
        + config_.ki * candidate_integral;
    const double limited_candidate_output = std::clamp(
        candidate_output, config_.output_min, config_.output_max);
    const bool saturated_high = candidate_output > config_.output_max && error > 0.0;
    const bool saturated_low = candidate_output < config_.output_min && error < 0.0;

    // 抗积分饱和：执行器已经饱和且误差还会把输出继续推向同一侧时，
    // 冻结旧积分；如果误差方向相反，则允许积分把输出从饱和状态拉回来。
    if (!config_.anti_windup || (!saturated_high && !saturated_low)) {
        integral_state_ = candidate_integral;
    }
    // 输出使用本次候选积分计算，积分状态则按上面的抗饱和规则决定是否提交。
    // 这样即使第一拍触发限幅，执行器仍会得到明确的饱和值。
    last_output_ = limited_candidate_output;
    return last_output_;
}

}  // namespace motioncore::control

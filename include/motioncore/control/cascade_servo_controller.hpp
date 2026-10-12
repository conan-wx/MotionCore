#pragma once

#include "motioncore/control/p_controller.hpp"
#include "motioncore/control/pi_controller.hpp"
#include "motioncore/model/dynamic_axis.hpp"

namespace motioncore::control {

enum class SpeedControllerMode { P, PI };

// 外位置环 + 内速度环的串级伺服控制器。
// 所有输入、输出均采用 SI 单位：位置 m、速度 m/s、驱动力 N。
struct CascadeServoConfig {
    double position_kp_per_s{8.0};
    double max_target_velocity_mps{0.1};
    double max_target_acceleration_mps2{0.8};
    SpeedControllerMode speed_mode{SpeedControllerMode::PI};
    double speed_kp{20.0};
    double speed_ki{80.0};
    double max_drive_force_n{20.0};
    double integral_min{-0.25};
    double integral_max{0.25};
    bool anti_windup{true};
    bool damping_feedforward{false};
    double viscous_damping_n_s_per_m{3.0};
};

struct CascadeServoOutput {
    double position_error_m{0.0};
    double target_velocity_mps{0.0};
    double speed_error_mps{0.0};
    double drive_force_n{0.0};
    bool drive_saturated{false};
};

class CascadedServoController {
public:
    explicit CascadedServoController(CascadeServoConfig config = {});

    CascadeServoOutput update(double target_position_m,
                              const model::DynamicAxisState& measured_state,
                              double dt_seconds);

    // Phase 3 轨迹跟踪接口：在位置误差修正之外加入规划器给出的速度前馈。
    // 原 update() 保持 Phase 2 行为，默认速度前馈为 0。
    CascadeServoOutput update_with_velocity_reference(
        double target_position_m,
        double reference_velocity_mps,
        const model::DynamicAxisState& measured_state,
        double dt_seconds);
    void reset() noexcept;

    [[nodiscard]] const CascadeServoConfig& config() const noexcept { return config_; }

private:
    CascadeServoConfig config_;
    PController speed_p_;
    PIController speed_pi_;
    double previous_target_velocity_mps_{0.0};
    bool has_previous_target_velocity_{false};
};

}  // namespace motioncore::control

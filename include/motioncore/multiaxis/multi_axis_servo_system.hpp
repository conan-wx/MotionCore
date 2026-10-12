#pragma once

#include "motioncore/control/cascade_servo_controller.hpp"
#include "motioncore/model/dynamic_axis.hpp"
#include "motioncore/multiaxis/interpolator.hpp"

#include <array>

namespace motioncore::multiaxis {

struct MultiAxisServoAxisConfig {
    model::DynamicAxisConfig dynamics{};
    control::CascadeServoConfig controller{};
};

struct MultiAxisServoState {
    double time_s{0.0};
    MultiAxisTrajectoryState reference{};
    Vector3 actual_position_m{};
    Vector3 actual_velocity_mps{};
    Vector3 actual_acceleration_mps2{};
    Vector3 drive_force_n{};
    Vector3 load_force_n{};
    std::array<bool, 3> drive_saturated{false, false, false};
};

// 三个轴共享一个 step 调用和同一个 dt。每个轴拥有独立的动力学模型、
// 速度控制器和积分状态，但不会因为某一轴先到位而提前停止其它轴。
class MultiAxisServoSystem {
public:
    explicit MultiAxisServoSystem(
        std::array<MultiAxisServoAxisConfig, 3> configs = default_configs());

    void reset();
    // 将另一个同构仿真系统中已经由动力学推进得到的轴状态接管过来。
    // 这是回零控制器交接给主调度器的状态同步，不是把位置强行写成目标值。
    void adopt_physical_state(const MultiAxisServoSystem& other);
    const MultiAxisServoState& step(const MultiAxisTrajectoryState& reference,
                                    double dt_seconds);
    // 急停软件仿真使用的驱动禁能路径：不给动力学模型施加驱动力，
    // 但仍按惯性、阻尼和负载推进位置与速度。
    const MultiAxisServoState& step_drive_disabled(double dt_seconds);

    [[nodiscard]] const MultiAxisServoState& state() const noexcept { return state_; }
    [[nodiscard]] const std::array<model::DynamicAxisModel, 3>& axes() const noexcept {
        return axes_;
    }

    static std::array<MultiAxisServoAxisConfig, 3> default_configs();

private:
    std::array<model::DynamicAxisModel, 3> axes_;
    std::array<control::CascadedServoController, 3> controllers_;
    MultiAxisServoState state_{};
};

}  // namespace motioncore::multiaxis

#pragma once

#include <array>
#include <cstddef>
#include <string>

namespace motioncore::manager {

// 软件控制器的高层状态。该枚举描述“允许哪些命令”，并不代表真实
// 驱动器的安全认证状态。
enum class MotionState {
    Disabled,
    NotHomed,
    Homing,
    Ready,
    Running,
    ControlledStopping,
    Fault,
    EmergencyStopped
};

enum class MotionEvent {
    Enable,
    Disable,
    StartHoming,
    HomingComplete,
    MotionStarted,
    MotionComplete,
    ControlledStopComplete,
    Stop,
    FaultDetected,
    EmergencyStop,
    ResetFault,
    ResetEmergency
};

enum class StateTransitionError {
    None,
    InvalidEvent,
    HomingRequired,
    FaultLatched,
    EmergencyLatched,
    NotEnabled,
    AlreadyInState
};

struct StateTransitionResult {
    bool accepted{false};
    StateTransitionError error{StateTransitionError::InvalidEvent};
    MotionState previous{MotionState::Disabled};
    MotionState current{MotionState::Disabled};
    std::string message;
};

// 纯逻辑状态机：不执行电机更新，也不直接修改任何轴的位置。
// 所有状态改变必须经过 dispatch()，因此很适合用单元测试覆盖非法转换。
class MotionStateMachine {
public:
    MotionStateMachine() = default;

    [[nodiscard]] MotionState state() const noexcept { return state_; }
    [[nodiscard]] bool homed() const noexcept {
        return homed_[0] && homed_[1] && homed_[2];
    }
    [[nodiscard]] const std::array<bool, 3>& homed_axes() const noexcept {
        return homed_;
    }

    // 回零控制器在各轴完成时调用该接口；状态仍只能由事件驱动。
    [[nodiscard]] bool set_axis_homed(std::size_t axis, bool value) noexcept;
    void clear_homing() noexcept;

    [[nodiscard]] StateTransitionResult dispatch(MotionEvent event,
                                                 std::string detail = {});
    // 常用事件的便捷封装，仍然经过同一个 dispatch 规则表。
    [[nodiscard]] bool enable() { return dispatch(MotionEvent::Enable).accepted; }
    [[nodiscard]] bool disable() { return dispatch(MotionEvent::Disable).accepted; }
    [[nodiscard]] bool start_homing() { return dispatch(MotionEvent::StartHoming).accepted; }
    [[nodiscard]] bool homing_complete() { return dispatch(MotionEvent::HomingComplete).accepted; }
    [[nodiscard]] bool motion_started() { return dispatch(MotionEvent::MotionStarted).accepted; }
    [[nodiscard]] bool motion_complete() { return dispatch(MotionEvent::MotionComplete).accepted; }
    [[nodiscard]] bool controlled_stop_complete() { return dispatch(MotionEvent::ControlledStopComplete).accepted; }
    [[nodiscard]] bool stop() { return dispatch(MotionEvent::Stop).accepted; }
    [[nodiscard]] bool emergency_stop() { return dispatch(MotionEvent::EmergencyStop).accepted; }

private:
    MotionState state_{MotionState::Disabled};
    std::array<bool, 3> homed_{{false, false, false}};
};

[[nodiscard]] const char* to_string(MotionState state) noexcept;
[[nodiscard]] const char* to_string(MotionEvent event) noexcept;
[[nodiscard]] const char* to_string(StateTransitionError error) noexcept;

}  // namespace motioncore::manager

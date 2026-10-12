#include "motioncore/motion_manager/motion_state_machine.hpp"

#include <cstddef>

namespace motioncore::manager {

bool MotionStateMachine::set_axis_homed(std::size_t axis, bool value) noexcept {
    if (axis >= homed_.size()) return false;
    homed_[axis] = value;
    return true;
}

void MotionStateMachine::clear_homing() noexcept {
    homed_.fill(false);
}

StateTransitionResult MotionStateMachine::dispatch(MotionEvent event, std::string detail) {
    const MotionState previous = state_;
    StateTransitionResult result{false, StateTransitionError::InvalidEvent, previous, previous,
                                 std::move(detail)};

    auto accept = [&](MotionState state) {
        state_ = state;
        result.accepted = true;
        result.error = StateTransitionError::None;
        result.current = state_;
    };
    auto reject = [&](StateTransitionError error, const char* message) {
        result.error = error;
        if (result.message.empty()) result.message = message;
    };

    switch (event) {
    case MotionEvent::Enable:
        if (state_ != MotionState::Disabled) {
            reject(StateTransitionError::AlreadyInState, "controller is already enabled");
        } else {
            accept(MotionState::NotHomed);
        }
        break;
    case MotionEvent::Disable:
        if (state_ == MotionState::Fault || state_ == MotionState::EmergencyStopped) {
            reject(state_ == MotionState::Fault ? StateTransitionError::FaultLatched
                                                 : StateTransitionError::EmergencyLatched,
                   "latched state requires explicit reset");
        } else {
            accept(MotionState::Disabled);
            clear_homing();
        }
        break;
    case MotionEvent::StartHoming:
        if (state_ == MotionState::Disabled) {
            reject(StateTransitionError::NotEnabled, "enable controller before homing");
        } else if (state_ == MotionState::Fault) {
            reject(StateTransitionError::FaultLatched, "reset fault before homing");
        } else if (state_ == MotionState::EmergencyStopped) {
            reject(StateTransitionError::EmergencyLatched, "reset emergency stop before homing");
        } else {
            accept(MotionState::Homing);
        }
        break;
    case MotionEvent::HomingComplete:
        if (state_ != MotionState::Homing) {
            reject(StateTransitionError::InvalidEvent, "homing is not active");
        } else if (!homed()) {
            reject(StateTransitionError::HomingRequired, "all axes must be homed");
        } else {
            accept(MotionState::Ready);
        }
        break;
    case MotionEvent::MotionStarted:
        if (state_ != MotionState::Ready) {
            reject(state_ == MotionState::NotHomed ? StateTransitionError::HomingRequired
                                                    : StateTransitionError::InvalidEvent,
                   "motion requires Ready state and all axes homed");
        } else if (!homed()) {
            reject(StateTransitionError::HomingRequired, "all axes must be homed before motion");
        } else {
            accept(MotionState::Running);
        }
        break;
    case MotionEvent::MotionComplete:
        if (state_ != MotionState::Running && state_ != MotionState::ControlledStopping) {
            reject(StateTransitionError::InvalidEvent, "no motion is running");
        } else {
            accept(MotionState::Ready);
        }
        break;
    case MotionEvent::ControlledStopComplete:
        if (state_ != MotionState::ControlledStopping) {
            reject(StateTransitionError::InvalidEvent, "no controlled stop is active");
        } else {
            accept(homed() ? MotionState::Ready : MotionState::NotHomed);
        }
        break;
    case MotionEvent::Stop:
        if (state_ != MotionState::Running && state_ != MotionState::Homing) {
            reject(StateTransitionError::InvalidEvent, "stop requires running or homing state");
        } else {
            accept(MotionState::ControlledStopping);
        }
        break;
    case MotionEvent::FaultDetected:
        if (state_ == MotionState::EmergencyStopped) {
            reject(StateTransitionError::EmergencyLatched, "emergency stop has priority");
        } else {
            accept(MotionState::Fault);
        }
        break;
    case MotionEvent::EmergencyStop:
        if (state_ != MotionState::EmergencyStopped) accept(MotionState::EmergencyStopped);
        else reject(StateTransitionError::AlreadyInState, "emergency stop already active");
        break;
    case MotionEvent::ResetFault:
        if (state_ != MotionState::Fault) {
            reject(StateTransitionError::InvalidEvent, "no latched fault is active");
        } else {
            accept(MotionState::NotHomed);
            clear_homing();
        }
        break;
    case MotionEvent::ResetEmergency:
        if (state_ != MotionState::EmergencyStopped) {
            reject(StateTransitionError::InvalidEvent, "emergency stop is not active");
        } else {
            accept(MotionState::NotHomed);
            clear_homing();
        }
        break;
    }
    return result;
}

const char* to_string(MotionState state) noexcept {
    switch (state) {
    case MotionState::Disabled: return "Disabled";
    case MotionState::NotHomed: return "NotHomed";
    case MotionState::Homing: return "Homing";
    case MotionState::Ready: return "Ready";
    case MotionState::Running: return "Running";
    case MotionState::ControlledStopping: return "ControlledStopping";
    case MotionState::Fault: return "Fault";
    case MotionState::EmergencyStopped: return "EmergencyStopped";
    }
    return "Unknown";
}

const char* to_string(MotionEvent event) noexcept {
    switch (event) {
    case MotionEvent::Enable: return "Enable";
    case MotionEvent::Disable: return "Disable";
    case MotionEvent::StartHoming: return "StartHoming";
    case MotionEvent::HomingComplete: return "HomingComplete";
    case MotionEvent::MotionStarted: return "MotionStarted";
    case MotionEvent::MotionComplete: return "MotionComplete";
    case MotionEvent::ControlledStopComplete: return "ControlledStopComplete";
    case MotionEvent::Stop: return "Stop";
    case MotionEvent::FaultDetected: return "FaultDetected";
    case MotionEvent::EmergencyStop: return "EmergencyStop";
    case MotionEvent::ResetFault: return "ResetFault";
    case MotionEvent::ResetEmergency: return "ResetEmergency";
    }
    return "Unknown";
}

const char* to_string(StateTransitionError error) noexcept {
    switch (error) {
    case StateTransitionError::None: return "None";
    case StateTransitionError::InvalidEvent: return "InvalidEvent";
    case StateTransitionError::HomingRequired: return "HomingRequired";
    case StateTransitionError::FaultLatched: return "FaultLatched";
    case StateTransitionError::EmergencyLatched: return "EmergencyLatched";
    case StateTransitionError::NotEnabled: return "NotEnabled";
    case StateTransitionError::AlreadyInState: return "AlreadyInState";
    }
    return "Unknown";
}

}  // namespace motioncore::manager

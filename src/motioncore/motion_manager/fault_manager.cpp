#include "motioncore/motion_manager/fault_manager.hpp"

#include <cmath>

namespace motioncore::manager {

bool FaultManager::raise(FaultCode code, double time_s, std::string source,
                         std::string message, FaultSeverity severity) {
    if (code == FaultCode::None || !std::isfinite(time_s)) return false;
    FaultRecord record{code, severity, true, severity == FaultSeverity::Latched,
                       time_s, std::move(source), std::move(message)};
    history_.push_back(record);
    // 故障采用锁存优先：警告不能覆盖现有严重故障，重复故障只更新历史。
    if (!active_.active || static_cast<int>(record.severity) > static_cast<int>(active_.severity)) {
        active_ = record;
    }
    return true;
}

bool FaultManager::raise_warning(FaultCode code, double time_s, std::string source,
                                 std::string message) {
    return raise(code, time_s, std::move(source), std::move(message), FaultSeverity::Warning);
}

bool FaultManager::reset(bool cause_cleared, double time_s, std::string source) {
    if (!cause_cleared || !active_.active || !std::isfinite(time_s)) return false;
    FaultRecord event = active_;
    event.active = false;
    event.time_s = time_s;
    if (!source.empty()) event.source = std::move(source);
    event.message = "fault reset";
    history_.push_back(event);
    active_ = FaultRecord{};
    return true;
}

const char* to_string(FaultCode code) noexcept {
    switch (code) {
    case FaultCode::None: return "None";
    case FaultCode::FollowingError: return "FollowingError";
    case FaultCode::SoftLimitViolation: return "SoftLimitViolation";
    case FaultCode::HomingTimeout: return "HomingTimeout";
    case FaultCode::HomeSensorFault: return "HomeSensorFault";
    case FaultCode::InvalidMotionCommand: return "InvalidMotionCommand";
    case FaultCode::NumericalError: return "NumericalError";
    case FaultCode::MotionTimeout: return "MotionTimeout";
    case FaultCode::EmergencyStopTriggered: return "EmergencyStopTriggered";
    }
    return "Unknown";
}

const char* to_string(FaultSeverity severity) noexcept {
    switch (severity) {
    case FaultSeverity::Info: return "Info";
    case FaultSeverity::Warning: return "Warning";
    case FaultSeverity::Latched: return "Latched";
    }
    return "Unknown";
}

}  // namespace motioncore::manager

#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace motioncore::manager {

enum class FaultCode {
    None,
    FollowingError,
    SoftLimitViolation,
    HomingTimeout,
    HomeSensorFault,
    InvalidMotionCommand,
    NumericalError,
    MotionTimeout,
    EmergencyStopTriggered
};

enum class FaultSeverity { Info, Warning, Latched };

struct FaultRecord {
    FaultCode code{FaultCode::None};
    FaultSeverity severity{FaultSeverity::Info};
    bool active{false};
    bool latched{false};
    double time_s{0.0};
    std::string source;
    std::string message;
};

// 故障管理器只负责记录、锁存和显式复位，不擅自改变运动状态。
class FaultManager {
public:
    [[nodiscard]] const FaultRecord& active_fault() const noexcept { return active_; }
    [[nodiscard]] bool active() const noexcept { return active_.active; }
    [[nodiscard]] bool has_fault() const noexcept { return active(); }
    [[nodiscard]] bool latched() const noexcept { return active_.active && active_.latched; }
    [[nodiscard]] FaultCode code() const noexcept { return active_.code; }
    [[nodiscard]] const std::vector<FaultRecord>& history() const noexcept { return history_; }

    // 返回 true 表示故障已登记；None、非有限时间会被拒绝。
    bool raise(FaultCode code, double time_s, std::string source = {},
               std::string message = {},
               FaultSeverity severity = FaultSeverity::Latched);
    bool raise_warning(FaultCode code, double time_s, std::string source = {},
                       std::string message = {});

    // 复位必须由上层确认故障来源已经消除。不会自动清除活动故障。
    bool reset(bool cause_cleared, double time_s, std::string source = {});
    [[nodiscard]] bool clear(bool cause_cleared, double time_s, std::string source = {}) {
        return reset(cause_cleared, time_s, std::move(source));
    }
    void clear_history() noexcept { history_.clear(); }

private:
    FaultRecord active_{};
    std::vector<FaultRecord> history_;
};

[[nodiscard]] const char* to_string(FaultCode code) noexcept;
[[nodiscard]] const char* to_string(FaultSeverity severity) noexcept;

}  // namespace motioncore::manager

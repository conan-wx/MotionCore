#pragma once

#include "motioncore/multiaxis/vector3.hpp"
#include "motioncore/multiaxis/interpolator.hpp"
#include "motioncore/planning/trajectory_profile.hpp"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <string>

namespace motioncore::manager {

enum class MotionCommandType { MoveLine, MoveArc, Dwell, Stop, EmergencyStop };
enum class CommandStatus { Queued, Running, Completed, Rejected, Canceled };
enum class ArcCommandDirection { CW, CCW };

struct MotionCommand {
    std::uint64_t id{0};
    MotionCommandType type{MotionCommandType::MoveLine};
    multiaxis::Vector3 start_m{};
    multiaxis::Vector3 target_m{};
    multiaxis::Vector3 center_m{};  // 仅圆弧使用
    ArcCommandDirection direction{ArcCommandDirection::CCW};
    bool full_circle{false};
    planning::MotionConstraints constraints{};
    multiaxis::PlanningMode planning_mode{multiaxis::PlanningMode::SCurve};
    double dwell_seconds{0.0};
    CommandStatus status{CommandStatus::Queued};
    double start_time_s{0.0};
    double end_time_s{0.0};
    std::string result_message;

    [[nodiscard]] bool finite_geometry() const noexcept;
    [[nodiscard]] bool is_high_priority() const noexcept {
        return type == MotionCommandType::Stop ||
               type == MotionCommandType::EmergencyStop;
    }
};

enum class QueueError {
    None,
    Full,
    InvalidCommand,
    Empty,
    NotFound
};

struct QueueResult {
    bool accepted{false};
    QueueError error{QueueError::InvalidCommand};
    std::uint64_t command_id{0};
    std::string message;
};

// 普通运动采用 FIFO；Stop/EmergencyStop 进入独立的高优先级队列，
// 不会排在前面的长轨迹后面等待。队列只管理数据，不驱动仿真。
class MotionCommandQueue {
public:
    explicit MotionCommandQueue(std::size_t capacity = 64) noexcept
        : capacity_(capacity) {}

    [[nodiscard]] std::size_t capacity() const noexcept { return capacity_; }
    [[nodiscard]] std::size_t size() const noexcept {
        return count_queued(normal_) + count_queued(priority_);
    }
    [[nodiscard]] bool empty() const noexcept { return size() == 0; }
    [[nodiscard]] std::size_t normal_size() const noexcept { return count_queued(normal_); }
    [[nodiscard]] std::size_t priority_size() const noexcept { return count_queued(priority_); }

    QueueResult enqueue(MotionCommand command);
    QueueResult enqueue_line(const multiaxis::Vector3& start_m,
                             const multiaxis::Vector3& target_m,
                             const planning::MotionConstraints& constraints,
                             multiaxis::PlanningMode mode = multiaxis::PlanningMode::SCurve);
    QueueResult enqueue_arc(const multiaxis::Vector3& start_m,
                            const multiaxis::Vector3& target_m,
                            const multiaxis::Vector3& center_m,
                            ArcCommandDirection direction,
                            const planning::MotionConstraints& constraints,
                            bool full_circle = false,
                            multiaxis::PlanningMode mode = multiaxis::PlanningMode::SCurve);
    QueueResult enqueue_dwell(double seconds);
    QueueResult enqueue_stop();
    QueueResult enqueue_emergency_stop();

    // 只取出一条命令；优先级队列先取。取出后状态仍为 Queued，
    // 执行器应显式调用 mark_running/mark_completed。
    [[nodiscard]] std::optional<MotionCommand> pop_next();
    [[nodiscard]] std::optional<MotionCommand> peek_next() const;
    // 返回当前执行序列中最后一个仍可能执行的几何命令终点。
    // MotionController 用它把“绝对目标”命令的起点解析到队列尾端，
    // 避免调用方在前一条命令尚未完成时猜测实际位置。
    [[nodiscard]] std::optional<multiaxis::Vector3> planned_endpoint() const;
    [[nodiscard]] std::optional<MotionCommand> find(std::uint64_t id) const;
    bool mark_running(std::uint64_t id, double time_s);
    bool mark_completed(std::uint64_t id, double time_s, std::string message = {});
    bool mark_rejected(std::uint64_t id, std::string message = {});
    bool cancel(std::uint64_t id, std::string message = {});
    std::size_t cancel_queued(std::string message = {});
    void clear() noexcept;

private:
    [[nodiscard]] static std::size_t count_queued(const std::deque<MotionCommand>& queue) noexcept {
        std::size_t count = 0;
        for (const auto& command : queue) {
            if (command.status == CommandStatus::Queued) ++count;
        }
        return count;
    }

    [[nodiscard]] std::uint64_t next_id() noexcept { return next_id_++; }
    [[nodiscard]] bool valid(const MotionCommand& command) const noexcept;
    static bool mark(std::deque<MotionCommand>& queue, std::uint64_t id,
                     CommandStatus status, double time_s, const std::string& message);

    std::size_t capacity_{64};
    std::uint64_t next_id_{1};
    std::deque<MotionCommand> normal_;
    std::deque<MotionCommand> priority_;
    // 已经 pop 出去但仍需由执行器更新状态的命令。保留记录便于日志和
    // mark_completed/cancel，不让一次 pop 丢失命令的生命周期。
    std::deque<MotionCommand> active_;
};

[[nodiscard]] const char* to_string(MotionCommandType type) noexcept;
[[nodiscard]] const char* to_string(CommandStatus status) noexcept;

}  // namespace motioncore::manager

#include "motioncore/motion_manager/motion_command_queue.hpp"

#include <cmath>
#include <sstream>

namespace motioncore::manager {

bool MotionCommand::finite_geometry() const noexcept {
    if (!start_m.is_finite() || !target_m.is_finite() || !center_m.is_finite()) return false;
    const auto finite = [](double v) { return std::isfinite(v); };
    if (!finite(constraints.max_velocity_mps) || !finite(constraints.max_acceleration_mps2) ||
        !finite(constraints.max_jerk_mps3)) return false;
    if (type == MotionCommandType::Dwell) return finite(dwell_seconds);
    if (type == MotionCommandType::Stop || type == MotionCommandType::EmergencyStop) return true;
    return constraints.max_velocity_mps > 0.0 && constraints.max_acceleration_mps2 > 0.0 &&
           constraints.max_jerk_mps3 > 0.0;
}

bool MotionCommandQueue::valid(const MotionCommand& command) const noexcept {
    if (command.type == MotionCommandType::Dwell) {
        return std::isfinite(command.dwell_seconds) && command.dwell_seconds >= 0.0;
    }
    return command.finite_geometry();
}

QueueResult MotionCommandQueue::enqueue(MotionCommand command) {
    QueueResult result{};
    // Safety events have a separate channel; a full ordinary FIFO must not
    // prevent Stop/EmergencyStop from reaching the scheduler.
    if (!command.is_high_priority() && normal_size() >= capacity_) {
        result.error = QueueError::Full;
        result.message = "command queue is full";
        return result;
    }
    if (!valid(command)) {
        result.error = QueueError::InvalidCommand;
        result.message = "command contains non-finite or invalid parameters";
        return result;
    }
    command.id = next_id();
    command.status = CommandStatus::Queued;
    if (command.is_high_priority()) {
        // EmergencyStop outranks an already queued Controlled Stop.  Keeping
        // it at the front makes the priority channel deterministic even when
        // both events arrive before the next scheduler cycle.
        if (command.type == MotionCommandType::EmergencyStop)
            priority_.push_front(std::move(command));
        else
            priority_.push_back(std::move(command));
    }
    else normal_.push_back(std::move(command));
    result.accepted = true;
    result.error = QueueError::None;
    result.command_id = next_id_ - 1;
    return result;
}

QueueResult MotionCommandQueue::enqueue_line(const multiaxis::Vector3& start_m,
                                             const multiaxis::Vector3& target_m,
                                             const planning::MotionConstraints& constraints,
                                             multiaxis::PlanningMode mode) {
    MotionCommand command{};
    command.type = MotionCommandType::MoveLine;
    command.start_m = start_m; command.target_m = target_m; command.constraints = constraints;
    command.planning_mode = mode;
    return enqueue(std::move(command));
}

QueueResult MotionCommandQueue::enqueue_arc(const multiaxis::Vector3& start_m,
                                            const multiaxis::Vector3& target_m,
                                            const multiaxis::Vector3& center_m,
                                            ArcCommandDirection direction,
                                            const planning::MotionConstraints& constraints,
                                            bool full_circle,
                                            multiaxis::PlanningMode mode) {
    MotionCommand command{};
    command.type = MotionCommandType::MoveArc;
    command.start_m = start_m; command.target_m = target_m; command.center_m = center_m;
    command.direction = direction; command.constraints = constraints;
    command.planning_mode = mode;
    command.full_circle = full_circle;
    return enqueue(std::move(command));
}

QueueResult MotionCommandQueue::enqueue_dwell(double seconds) {
    MotionCommand command{};
    command.type = MotionCommandType::Dwell; command.dwell_seconds = seconds;
    return enqueue(std::move(command));
}

QueueResult MotionCommandQueue::enqueue_stop() {
    MotionCommand command{}; command.type = MotionCommandType::Stop;
    return enqueue(std::move(command));
}

QueueResult MotionCommandQueue::enqueue_emergency_stop() {
    MotionCommand command{}; command.type = MotionCommandType::EmergencyStop;
    return enqueue(std::move(command));
}

std::optional<MotionCommand> MotionCommandQueue::pop_next() {
    // 取消命令仍保留在队列记录中，方便审计和日志查询；但它们绝不能
    // 再次成为执行对象。这里在取出时把终态记录移到 active_ 历史区，
    // 直到找到真正 Queued 的命令。这样急停复位、重新回零之后，旧的
    // canceled/rejected/completed 命令不会被误启动。
    const auto take_queued = [this](std::deque<MotionCommand>& queue)
        -> std::optional<MotionCommand> {
        while (!queue.empty()) {
            MotionCommand command = std::move(queue.front());
            queue.pop_front();
            if (command.status != CommandStatus::Queued) {
                active_.push_back(command);
                continue;
            }
            active_.push_back(command);
            return command;
        }
        return std::nullopt;
    };

    if (auto command = take_queued(priority_)) return command;
    if (auto command = take_queued(normal_)) return command;
    return std::nullopt;
}

std::optional<MotionCommand> MotionCommandQueue::peek_next() const {
    const auto peek_queued = [](const std::deque<MotionCommand>& queue)
        -> std::optional<MotionCommand> {
        for (const auto& command : queue) {
            if (command.status == CommandStatus::Queued) return command;
        }
        return std::nullopt;
    };
    if (auto command = peek_queued(priority_)) return command;
    if (auto command = peek_queued(normal_)) return command;
    return std::nullopt;
}

std::optional<multiaxis::Vector3> MotionCommandQueue::planned_endpoint() const {
    std::optional<multiaxis::Vector3> endpoint;
    const auto consider = [&endpoint](const MotionCommand& command) {
        // 终态命令只保留作审计记录，不能把已经取消的路径再次当成
        // 当前执行序列的尾端。Dwell 没有几何终点，故保持已有尾端。
        if (command.status != CommandStatus::Queued &&
            command.status != CommandStatus::Running) return;
        if (command.type == MotionCommandType::MoveLine ||
            command.type == MotionCommandType::MoveArc) {
            endpoint = command.target_m;
        }
    };
    // active_ 按 pop 顺序保存当前/历史命令，normal_ 保存尚未执行命令。
    // 忽略 priority_，因为 Stop/EmergencyStop 不代表几何路径。
    for (const auto& command : active_) consider(command);
    for (const auto& command : normal_) consider(command);
    return endpoint;
}

std::optional<MotionCommand> MotionCommandQueue::find(std::uint64_t id) const {
    const auto lookup = [id](const auto& queue) -> std::optional<MotionCommand> {
        for (const auto& command : queue) if (command.id == id) return command;
        return std::nullopt;
    };
    if (auto value = lookup(normal_)) return value;
    if (auto value = lookup(priority_)) return value;
    return lookup(active_);
}

bool MotionCommandQueue::mark(std::deque<MotionCommand>& queue, std::uint64_t id,
                              CommandStatus status, double time_s,
                              const std::string& message) {
    for (auto& command : queue) {
        if (command.id == id) {
            // 命令状态只允许沿着明确的生命周期前进：
            // Queued -> Running -> Completed/Canceled，或 Queued -> Rejected。
            // Stop/EmergencyStop 是高优先级事件，控制器可能在 pop 后
            // 直接将其标记为 Completed，因此允许这两个类型走
            // Queued -> Completed；普通运动命令不允许跳过 Running。
            if (status == CommandStatus::Running) {
                if (command.status != CommandStatus::Queued) return false;
            } else if (status == CommandStatus::Rejected) {
                if (command.status != CommandStatus::Queued) return false;
            } else if (status == CommandStatus::Canceled) {
                if (command.status != CommandStatus::Queued &&
                    command.status != CommandStatus::Running) return false;
            } else if (status == CommandStatus::Completed) {
                const bool running = command.status == CommandStatus::Running;
                const bool priority_queued = command.status == CommandStatus::Queued &&
                    command.is_high_priority();
                if (!running && !priority_queued) return false;
            } else {
                return false;
            }
            command.status = status;
            if (status == CommandStatus::Running) command.start_time_s = time_s;
            if (status == CommandStatus::Completed || status == CommandStatus::Rejected || status == CommandStatus::Canceled) command.end_time_s = time_s;
            command.result_message = message;
            return true;
        }
    }
    return false;
}

bool MotionCommandQueue::mark_running(std::uint64_t id, double time_s) {
    return mark(normal_, id, CommandStatus::Running, time_s, {}) ||
           mark(priority_, id, CommandStatus::Running, time_s, {}) ||
           mark(active_, id, CommandStatus::Running, time_s, {});
}
bool MotionCommandQueue::mark_completed(std::uint64_t id, double time_s, std::string message) {
    return mark(normal_, id, CommandStatus::Completed, time_s, message) ||
           mark(priority_, id, CommandStatus::Completed, time_s, message) ||
           mark(active_, id, CommandStatus::Completed, time_s, message);
}
bool MotionCommandQueue::mark_rejected(std::uint64_t id, std::string message) {
    return mark(normal_, id, CommandStatus::Rejected, 0.0, message) ||
           mark(priority_, id, CommandStatus::Rejected, 0.0, message) ||
           mark(active_, id, CommandStatus::Rejected, 0.0, message);
}
bool MotionCommandQueue::cancel(std::uint64_t id, std::string message) {
    return mark(normal_, id, CommandStatus::Canceled, 0.0, message) ||
           mark(priority_, id, CommandStatus::Canceled, 0.0, message) ||
           mark(active_, id, CommandStatus::Canceled, 0.0, message);
}

std::size_t MotionCommandQueue::cancel_queued(std::string message) {
    std::size_t count = 0;
    auto cancel_queue = [&](auto& queue) {
        for (auto& command : queue) {
            if (command.status == CommandStatus::Queued) {
                command.status = CommandStatus::Canceled; command.result_message = message; ++count;
            }
        }
    };
    cancel_queue(normal_); cancel_queue(priority_); return count;
}

void MotionCommandQueue::clear() noexcept { normal_.clear(); priority_.clear(); active_.clear(); }

const char* to_string(MotionCommandType type) noexcept {
    switch (type) {
    case MotionCommandType::MoveLine: return "MoveLine";
    case MotionCommandType::MoveArc: return "MoveArc";
    case MotionCommandType::Dwell: return "Dwell";
    case MotionCommandType::Stop: return "Stop";
    case MotionCommandType::EmergencyStop: return "EmergencyStop";
    }
    return "Unknown";
}
const char* to_string(CommandStatus status) noexcept {
    switch (status) {
    case CommandStatus::Queued: return "queued";
    case CommandStatus::Running: return "running";
    case CommandStatus::Completed: return "completed";
    case CommandStatus::Rejected: return "rejected";
    case CommandStatus::Canceled: return "canceled";
    }
    return "unknown";
}

}  // namespace motioncore::manager

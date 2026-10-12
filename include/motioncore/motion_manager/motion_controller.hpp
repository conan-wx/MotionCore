#pragma once

#include "motioncore/motion_manager/fault_manager.hpp"
#include "motioncore/motion_manager/homing_controller.hpp"
#include "motioncore/motion_manager/motion_command_queue.hpp"
#include "motioncore/motion_manager/motion_state_machine.hpp"
#include "motioncore/motion_manager/soft_limit_manager.hpp"
#include "motioncore/multiaxis/arc_interpolator.hpp"
#include "motioncore/multiaxis/line_interpolator.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace motioncore::manager {

struct MotionControllerConfig {
    std::size_t queue_capacity{64};
    std::array<multiaxis::MultiAxisServoAxisConfig, 3> servo_configs{
        multiaxis::MultiAxisServoSystem::default_configs()};
    HomingConfig homing{};
    std::array<AxisSoftLimit, 3> soft_limits{};
    double settle_time_s{0.05};
    double position_tolerance_m{2e-4};
    double velocity_tolerance_mps{5e-4};
    double following_error_limit_m{0.01};
    double following_error_hold_s{0.02};
    double motion_timeout_factor{3.0};
};

struct MotionEventRecord {
    double time_s{0.0};
    std::string event;
    MotionState state{MotionState::Disabled};
    std::uint64_t command_id{0};
    FaultCode fault_code{FaultCode::None};
    std::string message;
};

struct MotionControllerStatus {
    double time_s{0.0};
    MotionState state{MotionState::Disabled};
    std::uint64_t command_id{0};
    CommandStatus command_status{CommandStatus::Queued};
    multiaxis::Vector3 reference_position_m{};
    multiaxis::Vector3 reference_velocity_mps{};
    multiaxis::Vector3 reference_acceleration_mps2{};
    multiaxis::Vector3 reference_jerk_mps3{};
    multiaxis::Vector3 actual_position_m{};
    multiaxis::Vector3 actual_velocity_mps{};
    multiaxis::Vector3 actual_acceleration_mps2{};
    multiaxis::Vector3 drive_force_n{};
    std::array<bool, 3> homed{false, false, false};
    double following_error_m{0.0};
    multiaxis::Vector3 soft_limit_margin_m{};
    FaultCode fault_code{FaultCode::None};
    bool fault_active{false};
};

// Phase 5 的统一仿真调度器。它只负责“谁在什么时候调用谁”，不会替换
// Phase 2-4 的动力学、控制器或几何规划算法。所有坐标仍用 SI 单位。
class MotionController {
public:
    explicit MotionController(MotionControllerConfig config = {});

    bool enable();
    bool disable();
    bool home();
    // start_m is used for the first idle command.  If another geometric
    // command is Running/Queued, the controller resolves the start to the
    // planned queue endpoint so absolute targets can be submitted ahead of
    // time without a GUI-side prediction of the future actual position.
    QueueResult enqueue_move_line(const multiaxis::Vector3& start_m,
                                  const multiaxis::Vector3& target_m,
                                  const planning::MotionConstraints& constraints,
                                  multiaxis::PlanningMode mode = multiaxis::PlanningMode::SCurve);
    QueueResult enqueue_move_arc(const multiaxis::Vector3& start_m,
                                 const multiaxis::Vector3& target_m,
                                 const multiaxis::Vector3& center_m,
                                 ArcCommandDirection direction,
                                 const planning::MotionConstraints& constraints,
                                 multiaxis::PlanningMode mode = multiaxis::PlanningMode::SCurve,
                                 bool full_circle = false);
    QueueResult enqueue_dwell(double seconds);
    bool stop();
    bool emergency_stop();
    bool reset_fault();
    void release_emergency_input();
    bool reset_emergency();

    // 每次调用推进同一个三轴仿真时钟。dt 必须为有限正数；推荐固定为 0.001。
    bool step(double dt_seconds = 0.001);

    // 处理 Stop/EmergencyStop 高优先级事件但不推进仿真时钟。GUI 暂停时
    // 使用此接口可立即更新锁存状态；机械惯性仍由后续 step() 按动力学
    // 继续计算。普通 FIFO 命令不会在这里被启动。
    bool process_control_events();

    // 仅供确定性故障场景和测试注入，仍经过 FaultManager 和状态机锁存。
    bool inject_fault(FaultCode code, const std::string& message = "test injection");

    [[nodiscard]] MotionState state() const noexcept { return state_machine_.state(); }
    [[nodiscard]] const MotionControllerStatus& status() const noexcept { return status_; }
    [[nodiscard]] const MotionControllerConfig& config() const noexcept { return config_; }
    [[nodiscard]] const MotionCommandQueue& queue() const noexcept { return queue_; }
    [[nodiscard]] const FaultManager& faults() const noexcept { return faults_; }
    [[nodiscard]] const std::vector<MotionEventRecord>& events() const noexcept { return events_; }

private:
    bool start_next_command();
    bool build_active_profile(const MotionCommand& command);
    void update_status_from_servo();
    void log_event(const std::string& event, std::uint64_t id = 0,
                   FaultCode code = FaultCode::None, const std::string& message = {});
    bool handle_priority_event(const MotionCommand& command);
    void begin_controlled_stop();
    void finish_command(const std::string& message);
    void latch_fault(FaultCode code, const std::string& message);
    // 在每个伺服周期检查实际位置和当前速度是否仍有足够的安全裕量。
    // 该检查只使用动力学模型返回的实际状态，不会裁剪或覆盖实际位置。
    [[nodiscard]] bool monitor_dynamic_limits(double dt_seconds);
    [[nodiscard]] bool finite_status() const noexcept;

    MotionControllerConfig config_;
    MotionStateMachine state_machine_;
    MotionCommandQueue queue_;
    SoftLimitManager soft_limits_;
    FaultManager faults_;
    HomingController homing_;
    multiaxis::MultiAxisServoSystem servo_;
    MotionControllerStatus status_{};
    std::vector<MotionEventRecord> events_;
    std::unique_ptr<multiaxis::Interpolator> active_profile_;
    MotionCommand active_command_{};
    bool has_active_command_{false};
    double active_elapsed_s_{0.0};
    double settle_elapsed_s_{0.0};
    double following_error_elapsed_s_{0.0};
    multiaxis::MultiAxisTrajectoryState last_reference_{};
    multiaxis::Vector3 machine_offset_m_{};
    bool have_reference_{false};
    bool emergency_input_active_{false};
};

}  // namespace motioncore::manager


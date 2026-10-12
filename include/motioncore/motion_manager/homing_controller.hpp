#pragma once

#include "motioncore/multiaxis/multi_axis_servo_system.hpp"

#include <array>
#include <cstddef>
#include <string>

namespace motioncore::manager {

// 回零搜索方向。方向只影响虚拟传感器的触发条件和搜索运动方向。
enum class HomeDirection : int { Negative = -1, Positive = 1 };

enum class HomingPhase {
    Idle,
    SearchFast,
    Backoff,
    SearchSlow,
    ApplyOffset,
    Homed,
    Stopped,
    EmergencyStopped,
    Fault
};

// 软件中的原点开关。trigger_position_m 是传感器安装在轴坐标系中的位置，
// 不是把电机位置直接改成零的“魔法值”。
struct HomingSensorConfig {
    double trigger_position_m{0.0};
    HomeDirection direction{HomeDirection::Negative};
    // active_high 只描述电气有效电平；sensor_active() 返回物理触发状态。
    bool active_high{true};
    // 测试传感器异常用的显式注入开关。正常运行时保持 false；它们不会
    // 修改动力学位置，只会让状态机走异常分支。
    bool simulate_stuck_active{false};
    bool simulate_stuck_inactive{false};
    double debounce_time_s{0.003};
};

struct HomingAxisConfig {
    HomingSensorConfig sensor{};
    double fast_speed_mps{0.01};
    double slow_speed_mps{0.002};
    double backoff_distance_m{0.005};
    double search_range_m{0.20};
    double timeout_s{30.0};
    // 应用偏移后，传感器触发点在机床坐标中的坐标。
    double offset_m{0.0};
    double position_tolerance_m{1e-6};
};

struct HomingConfig {
    std::array<HomingAxisConfig, 3> axes{};
    // 默认回零顺序 Z -> X -> Y。顺序是软件仿真策略，不代表真实设备安全顺序。
    std::array<std::size_t, 3> order{{2U, 0U, 1U}};
};

struct HomingAxisStatus {
    HomingPhase phase{HomingPhase::Idle};
    bool sensor_level{false};
    bool sensor_active{false};
    bool homed{false};
    double elapsed_s{0.0};
    double search_start_position_m{0.0};
    double trigger_position_m{0.0};
    double machine_position_m{0.0};
    std::string message{};
};

struct HomingStatus {
    double time_s{0.0};
    std::array<HomingAxisStatus, 3> axes{};
    std::size_t active_axis{0U};
    bool running{false};
    bool completed{false};
    bool fault{false};
    bool emergency_stopped{false};
    std::string fault_message{};
};

// 纯软件三轴回零控制器。
//
// 每个 step 都通过 MultiAxisServoSystem 产生真实的驱动力并推进动力学模型，
// 因此回零不会把实际位置直接写成零。传感器只改变状态机；ApplyOffset 阶段
// 用 machine_position = physical_position - trigger + offset 建立机床坐标。
class HomingController {
public:
    explicit HomingController(
        HomingConfig config = {},
        std::array<multiaxis::MultiAxisServoAxisConfig, 3> servo_configs =
            multiaxis::MultiAxisServoSystem::default_configs());

    void reset();
    bool start();
    bool step(double dt_seconds);
    void stop();
    void emergency_stop();
    // 解除停止锁存。对于 Fault，调用 reset() 后才能重新开始，避免带着未知坐标运行。
    bool clear_stop();
    // 显式确认回零故障原因已排除；保留物理轴状态，下一次仍从 NotHomed 搜索。
    bool clear_fault();

    [[nodiscard]] bool all_homed() const noexcept { return status_.completed; }
    [[nodiscard]] bool running() const noexcept { return status_.running; }
    [[nodiscard]] HomingPhase phase(std::size_t axis) const noexcept {
        return axis < 3U ? status_.axes[axis].phase : HomingPhase::Fault;
    }
    [[nodiscard]] bool axis_homed(std::size_t axis) const noexcept {
        return axis < 3U && status_.axes[axis].homed;
    }
    [[nodiscard]] const HomingConfig& config() const noexcept { return config_; }
    [[nodiscard]] const HomingStatus& status() const noexcept { return status_; }
    [[nodiscard]] const multiaxis::MultiAxisServoSystem& servo() const noexcept {
        return servo_;
    }
    [[nodiscard]] multiaxis::Vector3 physical_position_m() const noexcept;
    [[nodiscard]] multiaxis::Vector3 machine_position_m() const noexcept;
    [[nodiscard]] multiaxis::Vector3 machine_offset_m() const noexcept { return machine_offsets_m_; }

private:
    static void validate_config(const HomingConfig& config);
    static double axis_component(const multiaxis::Vector3& value, std::size_t axis);
    static void set_axis_component(multiaxis::Vector3& value, std::size_t axis, double x);
    bool update_sensor(std::size_t axis, double dt_seconds);
    void set_fault(std::size_t axis, const std::string& message);
    bool advance_axis(std::size_t axis, double dt_seconds);
    [[nodiscard]] bool reached_position(std::size_t axis, double target) const;
    void update_machine_positions();

    HomingConfig config_;
    multiaxis::MultiAxisServoSystem servo_;
    HomingStatus status_{};
    multiaxis::Vector3 command_position_m_{};
    multiaxis::Vector3 machine_offsets_m_{};
    std::array<bool, 3> debounce_candidate_{{false, false, false}};
    std::array<double, 3> debounce_elapsed_{{0.0, 0.0, 0.0}};
    std::array<double, 3> backoff_target_m_{{0.0, 0.0, 0.0}};
    double completion_settle_elapsed_s_{0.0};
};

}  // namespace motioncore::manager

// Some clients used motioncore::motion_manager while the rest of the project
// uses motioncore::manager. Keep a source-compatible alias without duplicating
// the implementation.
namespace motioncore {
namespace motion_manager = manager;
}


#pragma once

// 便于应用和测试一次性引入 Phase 5 的纯逻辑管理模块。
#include "motioncore/motion_manager/fault_manager.hpp"
#include "motioncore/motion_manager/motion_command_queue.hpp"
#include "motioncore/motion_manager/motion_state_machine.hpp"
#include "motioncore/motion_manager/soft_limit_manager.hpp"
#include "motioncore/motion_manager/homing_controller.hpp"

namespace motioncore {
namespace motion_manager = manager;
}

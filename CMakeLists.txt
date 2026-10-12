cmake_minimum_required(VERSION 3.20)

project(MotionCore VERSION 0.1 LANGUAGES CXX)

option(MOTIONCORE_BUILD_GUI "Build the Qt 6 Widgets desktop controller" OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(motion_control
    src/pid_controller.cpp
)

target_include_directories(motion_control
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_options(motion_control
    PRIVATE
        -Wall -Wextra -Wpedantic
)

add_executable(pid_demo
    apps/pid_demo.cpp
)
target_link_libraries(pid_demo PRIVATE motion_control)
target_compile_options(pid_demo PRIVATE -Wall -Wextra -Wpedantic)

add_executable(test_pid
    tests/test_pid.cpp
)
target_link_libraries(test_pid PRIVATE motion_control)
target_compile_options(test_pid PRIVATE -Wall -Wextra -Wpedantic)

add_library(motioncore
    src/motioncore/model/axis_simulator.cpp
    src/motioncore/model/dynamic_axis.cpp
    src/motioncore/control/basic_servo_controller.cpp
    src/motioncore/control/p_controller.cpp
    src/motioncore/control/pi_controller.cpp
    src/motioncore/control/pid_controller.cpp
    src/motioncore/control/cascade_servo_controller.cpp
    src/motioncore/planning/trajectory_profile.cpp
    src/motioncore/planning/trapezoidal_profile.cpp
    src/motioncore/planning/s_curve_profile.cpp
    src/motioncore/multiaxis/interpolator.cpp
    src/motioncore/multiaxis/line_interpolator.cpp
    src/motioncore/multiaxis/arc_interpolator.cpp
    src/motioncore/multiaxis/coordinated_trajectory_planner.cpp
    src/motioncore/multiaxis/multi_axis_servo_system.cpp
    src/motioncore/metrics/metrics.cpp
    src/motioncore/io/csv_logger.cpp
    src/motioncore/motion_manager/motion_state_machine.cpp
    src/motioncore/motion_manager/motion_command_queue.cpp
    src/motioncore/motion_manager/soft_limit_manager.cpp
    src/motioncore/motion_manager/fault_manager.cpp
    src/motioncore/motion_manager/homing_controller.cpp
    src/motioncore/motion_manager/motion_controller.cpp
)

target_include_directories(motioncore
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_options(motioncore
    PRIVATE
        -Wall -Wextra -Wpedantic
)

add_executable(motioncore_phase1
    apps/motioncore_phase1.cpp
)
target_link_libraries(motioncore_phase1 PRIVATE motioncore)
target_compile_options(motioncore_phase1 PRIVATE -Wall -Wextra -Wpedantic)

add_executable(motioncore_phase2
    apps/motioncore_phase2.cpp
)
target_link_libraries(motioncore_phase2 PRIVATE motioncore)
target_compile_options(motioncore_phase2 PRIVATE -Wall -Wextra -Wpedantic)

add_executable(motioncore_phase3
    apps/motioncore_phase3.cpp
)
target_link_libraries(motioncore_phase3 PRIVATE motioncore)
target_compile_options(motioncore_phase3 PRIVATE -Wall -Wextra -Wpedantic)

add_executable(motioncore_tests
    tests/motioncore_tests.cpp
)
target_link_libraries(motioncore_tests PRIVATE motioncore)
target_compile_options(motioncore_tests PRIVATE -Wall -Wextra -Wpedantic)

add_executable(motioncore_phase2_tests
    tests/motioncore_phase2_tests.cpp
)
target_link_libraries(motioncore_phase2_tests PRIVATE motioncore)
target_compile_options(motioncore_phase2_tests PRIVATE -Wall -Wextra -Wpedantic)

add_executable(motioncore_phase3_tests
    tests/motioncore_phase3_tests.cpp
)
target_link_libraries(motioncore_phase3_tests PRIVATE motioncore)
target_compile_options(motioncore_phase3_tests PRIVATE -Wall -Wextra -Wpedantic)

add_executable(motioncore_phase4
    apps/motioncore_phase4.cpp
)
target_link_libraries(motioncore_phase4 PRIVATE motioncore)
target_compile_options(motioncore_phase4 PRIVATE -Wall -Wextra -Wpedantic)

add_executable(motioncore_phase4_tests
    tests/motioncore_phase4_tests.cpp
)
target_link_libraries(motioncore_phase4_tests PRIVATE motioncore)
target_compile_options(motioncore_phase4_tests PRIVATE -Wall -Wextra -Wpedantic)

add_executable(motioncore_phase4_fix_tests
    tests/motioncore_phase4_fix_tests.cpp
)
target_link_libraries(motioncore_phase4_fix_tests PRIVATE motioncore)
target_compile_options(motioncore_phase4_fix_tests PRIVATE -Wall -Wextra -Wpedantic)

add_executable(motioncore_phase5
    apps/motioncore_phase5.cpp
)
target_link_libraries(motioncore_phase5 PRIVATE motioncore)
target_compile_options(motioncore_phase5 PRIVATE -Wall -Wextra -Wpedantic)

add_executable(motioncore_phase5_tests
    tests/motioncore_phase5_tests.cpp
)
target_link_libraries(motioncore_phase5_tests PRIVATE motioncore)
target_compile_options(motioncore_phase5_tests PRIVATE -Wall -Wextra -Wpedantic)

# Phase 5.1 回归测试：验证急停取消后的命令生命周期不会在复位回零后重启。
add_executable(phase5_1_queue_regression_test
    tests/phase5_1_queue_regression_test.cpp
)
target_link_libraries(phase5_1_queue_regression_test PRIVATE motioncore)
target_compile_options(phase5_1_queue_regression_test PRIVATE -Wall -Wextra -Wpedantic)

# Phase 5.1 回归测试：运行中的实际轴状态和预测制动距离必须触发软限位保护。
add_executable(phase5_1_dynamic_limit_test
    tests/phase5_1_dynamic_limit_test.cpp
)
target_link_libraries(phase5_1_dynamic_limit_test PRIVATE motioncore)
target_compile_options(phase5_1_dynamic_limit_test PRIVATE -Wall -Wextra -Wpedantic)

# 最终核心回归：故障自由滑行、连续直线及直线-圆弧-直线队列衔接。
add_executable(phase5_final_core_regression_test
    tests/phase5_final_core_regression_test.cpp
)
target_link_libraries(phase5_final_core_regression_test PRIVATE motioncore)
target_compile_options(phase5_final_core_regression_test PRIVATE -Wall -Wextra -Wpedantic)

enable_testing()
add_test(NAME pid_unit_test COMMAND test_pid)
add_test(NAME motioncore_phase1_test COMMAND motioncore_tests)
add_test(NAME motioncore_phase2_test COMMAND motioncore_phase2_tests)
add_test(NAME motioncore_phase3_test COMMAND motioncore_phase3_tests)
add_test(NAME motioncore_phase4_test COMMAND motioncore_phase4_tests)
add_test(NAME motioncore_phase4_fix_test COMMAND motioncore_phase4_fix_tests)
add_test(
    NAME motioncore_phase4_csv_test
    COMMAND ${CMAKE_COMMAND}
        -DMOTIONCORE_PHASE4_EXE=$<TARGET_FILE:motioncore_phase4>
        -DOUTPUT_CSV=${CMAKE_CURRENT_BINARY_DIR}/phase4_csv_test.csv
        -DSOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
        -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/check_phase4_csv.cmake
)

if(MOTIONCORE_BUILD_GUI)
    set(CMAKE_AUTOMOC ON)
    find_package(Qt6 6.2 REQUIRED COMPONENTS Widgets Test)
    add_executable(motioncore_gui
        apps/motioncore_gui.cpp
        gui/mainwindow.hpp
        gui/simulation_worker.hpp
        gui/trajectory_view.hpp
        gui/telemetry_view.hpp
        gui/mainwindow.cpp
        gui/simulation_worker.cpp
        gui/trajectory_view.cpp
        gui/telemetry_view.cpp
    )
    target_include_directories(motioncore_gui PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/gui)
    target_link_libraries(motioncore_gui PRIVATE motioncore Qt6::Widgets)
    target_compile_options(motioncore_gui PRIVATE -Wall -Wextra -Wpedantic)

    add_executable(motioncore_gui_smoke_test
        tests/motioncore_gui_smoke_test.cpp
        gui/mainwindow.hpp
        gui/simulation_worker.hpp
        gui/trajectory_view.hpp
        gui/telemetry_view.hpp
        gui/mainwindow.cpp
        gui/simulation_worker.cpp
        gui/trajectory_view.cpp
        gui/telemetry_view.cpp
    )
    target_include_directories(motioncore_gui_smoke_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/gui)
    target_link_libraries(motioncore_gui_smoke_test PRIVATE motioncore Qt6::Widgets Qt6::Test)
    target_compile_options(motioncore_gui_smoke_test PRIVATE -Wall -Wextra -Wpedantic)
    add_test(NAME phase6_gui_smoke_test COMMAND motioncore_gui_smoke_test)
    set_tests_properties(phase6_gui_smoke_test PROPERTIES
        ENVIRONMENT "QT_QPA_PLATFORM=offscreen")

    add_executable(motioncore_gui_data_test
        tests/motioncore_gui_data_test.cpp
        gui/simulation_worker.hpp
        gui/simulation_worker.cpp
    )
    target_include_directories(motioncore_gui_data_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/gui)
    target_link_libraries(motioncore_gui_data_test PRIVATE motioncore Qt6::Widgets Qt6::Test)
    target_compile_options(motioncore_gui_data_test PRIVATE -Wall -Wextra -Wpedantic)
    add_test(NAME phase6_gui_data_test COMMAND motioncore_gui_data_test)
    set_tests_properties(phase6_gui_data_test PROPERTIES
        ENVIRONMENT "QT_QPA_PLATFORM=offscreen")

    add_executable(phase6_gui_regression_test
        tests/phase6_gui_regression_test.cpp
        gui/simulation_worker.hpp
        gui/simulation_worker.cpp
    )
    target_include_directories(phase6_gui_regression_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/gui)
    target_link_libraries(phase6_gui_regression_test PRIVATE motioncore Qt6::Widgets Qt6::Test)
    target_compile_options(phase6_gui_regression_test PRIVATE -Wall -Wextra -Wpedantic)
    add_test(NAME phase6_gui_regression_test COMMAND phase6_gui_regression_test)
    set_tests_properties(phase6_gui_regression_test PROPERTIES
        ENVIRONMENT "QT_QPA_PLATFORM=offscreen")
endif()
add_test(NAME phase5_state_machine_test COMMAND motioncore_phase5_tests state)
add_test(NAME phase5_command_queue_test COMMAND motioncore_phase5_tests queue)
add_test(NAME phase5_homing_test COMMAND motioncore_phase5_tests homing)
add_test(NAME phase5_soft_limit_test COMMAND motioncore_phase5_tests soft)
add_test(NAME phase5_stop_test COMMAND motioncore_phase5_tests stop)
add_test(NAME phase5_fault_test COMMAND motioncore_phase5_tests fault)
add_test(NAME phase5_integration_test COMMAND motioncore_phase5_tests integration)
add_test(NAME phase5_1_queue_lifecycle_test COMMAND phase5_1_queue_regression_test)
add_test(NAME phase5_1_dynamic_limit_test COMMAND phase5_1_dynamic_limit_test)
add_test(NAME phase5_final_core_regression_test COMMAND phase5_final_core_regression_test)
add_test(
    NAME phase5_1_jerk_unit_test
    COMMAND ${CMAKE_COMMAND}
        -DMOTIONCORE_PHASE4_EXE=$<TARGET_FILE:motioncore_phase4>
        -DOUTPUT_CSV=${CMAKE_CURRENT_BINARY_DIR}/phase5_1_jerk_unit_test.csv
        -DSOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
        -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/check_phase4_csv.cmake
)
add_test(
    NAME phase5_csv_test
    COMMAND ${CMAKE_COMMAND}
        -DPHASE5_EXE=$<TARGET_FILE:motioncore_phase5>
        -DOUTPUT_DIR=${CMAKE_CURRENT_BINARY_DIR}/phase5_csv_test
        -DSOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
        -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/check_phase5_csv.cmake
)

cmake_minimum_required(VERSION 3.12)
project(CruiseControl CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置可执行文件输出到 bin 文件夹
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

# 包含路径
include_directories(include)

# 业务源文件列表
set(CC_SOURCES
    src/math/abs_val.cpp
    src/math/calc_acceleration.cpp
    src/math/clamp_val.cpp
    src/decision/cc_check_speed_substate.cpp
    src/decision/cc_check_system_substate.cpp
    src/decision/cc_determine_state.cpp
    src/decision/cc_process_command.cpp
    src/decision/cc_evaluate_decision.cpp
    src/decision/cc_determine_next_active.cpp
    src/decision/cc_determine_next_target.cpp
    src/decision/cc_determine_next_last_target.cpp
    src/control/cc_calc_speed_error.cpp
    src/control/cc_calc_control_torque.cpp
    src/control/cc_check_target_change.cpp
    src/control/cc_calc_target_deviation.cpp
    src/control/cc_update_virtual_target.cpp
    src/control/cc_limit_virtual_target.cpp
    src/control/cc_sppvt_step.cpp
    src/control/cc_update_cooldown_timer.cpp
    src/control/cc_determine_next_cooldown_timer.cpp
    src/control/cc_calc_new_virtual_target.cpp
    src/control/cc_determine_next_virtual_target.cpp
    src/control/cc_determine_next_step_index.cpp
    src/control/cc_determine_torque_command.cpp
    src/control/cc_determine_triggered.cpp
)

# 生成静态链接库
add_library(cruise_control STATIC ${CC_SOURCES})

# 添加测试可执行文件
add_executable(test_decision test/test_decision.cpp)
target_link_libraries(test_decision cruise_control)

add_executable(test_control test/test_control.cpp)
target_link_libraries(test_control cruise_control)

add_executable(test_integration test/test_integration.cpp)
target_link_libraries(test_integration cruise_control)

#include <iostream>
#include <cassert>
#include "decision/cc_decision_def.h"
#include "decision/cc_check_speed_substate.h"
#include "decision/cc_check_system_substate.h"
#include "decision/cc_determine_state.h"
#include "decision/cc_process_command.h"

using namespace cc::decision;

void testSpeedSubstate() {
    DecisionConfig config;
    config.minSpeed = 10.0; // 36 km/h
    config.maxSpeed = 30.0; // 108 km/h

    // 适速 Sv0
    assert(checkSpeedSubstate(15.0, config) == SPEED_SUBSTATE_SUITABLE);
    assert(checkSpeedSubstate(10.0, config) == SPEED_SUBSTATE_SUITABLE);
    assert(checkSpeedSubstate(30.0, config) == SPEED_SUBSTATE_SUITABLE);

    // 低速 Sv1
    assert(checkSpeedSubstate(9.9, config) == SPEED_SUBSTATE_LOW);
    assert(checkSpeedSubstate(0.0, config) == SPEED_SUBSTATE_LOW);

    // 高速 Sv2
    assert(checkSpeedSubstate(30.1, config) == SPEED_SUBSTATE_HIGH);

    std::cout << "[PASS] 车速子态判定测试成功！" << std::endl;
}

void testSystemSubstate() {
    // controlActive, throttlePressed, hasHistoryTarget
    assert(checkSystemSubstate(false, false, false) == SYS_SUBSTATE_S0);
    assert(checkSystemSubstate(false, false, true) == SYS_SUBSTATE_S1);
    assert(checkSystemSubstate(false, true, false) == SYS_SUBSTATE_S2);
    assert(checkSystemSubstate(false, true, true) == SYS_SUBSTATE_S3);
    assert(checkSystemSubstate(true, false, false) == SYS_SUBSTATE_S4);
    assert(checkSystemSubstate(true, false, true) == SYS_SUBSTATE_S5);
    assert(checkSystemSubstate(true, true, false) == SYS_SUBSTATE_S6);
    assert(checkSystemSubstate(true, true, true) == SYS_SUBSTATE_S7);

    std::cout << "[PASS] 系统子态编码测试成功！" << std::endl;
}

void testStateDetermination() {
    // 适速车速下的映射 S0, S2, S3, S4
    assert(determineState(SPEED_SUBSTATE_SUITABLE, SYS_SUBSTATE_S0) == SYS_STATE_S3); // 适速无油无史待命
    assert(determineState(SPEED_SUBSTATE_SUITABLE, SYS_SUBSTATE_S1) == SYS_STATE_S2); // 适速无油有史待命
    assert(determineState(SPEED_SUBSTATE_SUITABLE, SYS_SUBSTATE_S2) == SYS_STATE_S4); // 有油待命
    assert(determineState(SPEED_SUBSTATE_SUITABLE, SYS_SUBSTATE_S3) == SYS_STATE_S4); // 有油待命
    assert(determineState(SPEED_SUBSTATE_SUITABLE, SYS_SUBSTATE_S4) == SYS_STATE_S0); // 适速在控
    assert(determineState(SPEED_SUBSTATE_SUITABLE, SYS_SUBSTATE_S7) == SYS_STATE_S0); // 适速在控

    // 低速车速下的映射 S5, S6
    assert(determineState(SPEED_SUBSTATE_LOW, SYS_SUBSTATE_S0) == SYS_STATE_S5); // 非适速待命
    assert(determineState(SPEED_SUBSTATE_LOW, SYS_SUBSTATE_S4) == SYS_STATE_S6); // 低速在控

    // 高速车速下的映射 S5, S1
    assert(determineState(SPEED_SUBSTATE_HIGH, SYS_SUBSTATE_S1) == SYS_STATE_S5); // 非适速待命
    assert(determineState(SPEED_SUBSTATE_HIGH, SYS_SUBSTATE_S5) == SYS_STATE_S1); // 高速在控

    std::cout << "[PASS] 系统状态流转表映射测试成功！" << std::endl;
}

void testDecisionProcess() {
    DecisionConfig config;
    config.minSpeed = 10.0;
    config.maxSpeed = 30.0;
    config.lowSpeedTimeout = 3.0; // 3秒超时
    config.deltaSpeedStep = 1.0;

    DecisionState state;
    state.currentState = SYS_STATE_S3; // 适速无油无史待命
    state.targetSpeed = 0.0;
    state.lastTargetSpeed = 0.0;
    state.lowSpeedTimer = 0.0;

    // 1. 在待命状态且无历史记录下，按增速键 I0 -> 触发 Set (R5) 并进入控制
    DecisionInput input1;
    input1.currentSpeed = 15.0;
    input1.isBrakePressed = false;
    input1.isThrottlePressed = false;
    input1.hasHistoryTarget = false;
    input1.controlActive = false;
    input1.driverCommand = COMMAND_INC;
    input1.dt = 0.1;

    DecisionOutput out1 = processCommand(input1, config, state);
    assert(out1.decision == RESULT_R5);
    assert(out1.updatedTargetSpeed == 15.0);
    assert(state.targetSpeed == 15.0);
    assert(out1.nextState == SYS_STATE_S0); // 预测下一状态为适速在控

    // 2. 正常适速在控状态 S0，无指令 -> 保持，目标车速不变
    input1.controlActive = true;
    input1.driverCommand = COMMAND_NONE;
    DecisionOutput out2 = processCommand(input1, config, state);
    assert(out2.decision == RESULT_NONE);
    assert(out2.updatedTargetSpeed == 15.0);

    // 3. 正常适速在控状态 S0，按增速键 I0 -> 目标增量 (R1) 变 16.0
    input1.driverCommand = COMMAND_INC;
    DecisionOutput out3 = processCommand(input1, config, state);
    assert(out3.decision == RESULT_R1);
    assert(out3.updatedTargetSpeed == 16.0);
    assert(state.targetSpeed == 16.0);

    // 4. 刹车踩下 -> 退出控制进入待命 (R6)，且当前在控所以保存历史车速 16.0
    input1.driverCommand = COMMAND_BRAKE;
    DecisionOutput out4 = processCommand(input1, config, state);
    assert(out4.decision == RESULT_R6);
    assert(state.lastTargetSpeed == 16.0); // 保存了历史值

    // 5. 处于适速无油有史待命，按下增速键 -> 恢复控制 (R4)，车速设为 16.0
    input1.controlActive = false;
    input1.hasHistoryTarget = true;
    input1.driverCommand = COMMAND_INC;
    DecisionOutput out5 = processCommand(input1, config, state);
    assert(out5.decision == RESULT_R4);
    assert(out5.updatedTargetSpeed == 16.0);
    assert(state.targetSpeed == 16.0);

    // 6. 低速在控 S6 超时保护逻辑
    state.currentState = SYS_STATE_S6;
    state.lowSpeedTimer = 0.0;
    input1.controlActive = true;
    input1.currentSpeed = 8.0; // 低于最小限速 10.0
    input1.driverCommand = COMMAND_NONE;
    
    // 运行 2.9 秒，仍不超时
    input1.dt = 2.9;
    DecisionOutput out6 = processCommand(input1, config, state);
    assert(out6.decision == RESULT_NONE);
    assert(state.lowSpeedTimer == 2.9);

    // 再运行 0.2 秒（累计 3.1 秒），触发主动退出报错 (R7)
    input1.dt = 0.2;
    DecisionOutput out7 = processCommand(input1, config, state);
    assert(out7.decision == RESULT_R7); // 安全保护退出！
    assert(out7.nextState == SYS_STATE_S5); // 预测下一状态为非适速待命

    std::cout << "[PASS] 系统决策流程与安全超时逻辑测试成功！" << std::endl;
}

int main() {
    std::cout << "========= 开始决策业务库测试 =========" << std::endl;
    testSpeedSubstate();
    testSystemSubstate();
    testStateDetermination();
    testDecisionProcess();
    std::cout << "======================================" << std::endl;
    std::cout << "所有决策库测试通过！" << std::endl;
    return 0;
}

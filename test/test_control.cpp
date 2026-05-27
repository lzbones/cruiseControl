#include <iostream>
#include <cassert>
#include "control/cc_control_def.h"
#include "control/cc_calc_speed_error.h"
#include "control/cc_calc_control_torque.h"
#include "control/cc_check_target_change.h"
#include "control/cc_calc_target_deviation.h"
#include "control/cc_update_virtual_target.h"
#include "control/cc_limit_virtual_target.h"
#include "control/cc_sppvt_step.h"

using namespace cc::control;

void testControlMath() {
    SppvtConfig config;
    config.Kp = 50.0;
    config.rho = 0.5;
    config.delta = 0.05;
    config.maxOffset = 3.0; // max virtual target Vt + 3.0
    config.cooldownTime = 2.0;
    config.minTorque = 0.0;
    config.maxTorque = 300.0;

    // 1. 测试基础误差计算: e_i = V_i - v_i
    assert(calcSpeedError(20.0, 18.0) == 2.0);
    assert(calcSpeedError(15.0, 16.0) == -1.0);

    // 2. 测试扭矩比例计算与限幅
    assert(calcControlTorque(config.Kp, 2.0, config) == 100.0);   // 50 * 2 = 100
    assert(calcControlTorque(config.Kp, 10.0, config) == 300.0);  // 50 * 10 = 500 -> 限幅到 300
    assert(calcControlTorque(config.Kp, -1.0, config) == 0.0);    // 50 * -1 = -50 -> 下限幅到 0 (定速巡航不倒车/制动)

    // 3. 测试变目标判定条件 |v_dot| < delta 且冷却时间为 0
    assert(checkTargetChange(0.04, 0.05, 0.0) == true);
    assert(checkTargetChange(-0.04, 0.05, 0.0) == true); // 双边绝对值限幅！
    assert(checkTargetChange(0.06, 0.05, 0.0) == false);
    assert(checkTargetChange(0.01, 0.05, 1.0) == false); // 冷却未结束，不触发

    // 4. 目标车速偏差计算 ΔV_ti = V_t - v_i
    assert(calcTargetDeviation(20.0, 18.0) == 2.0);

    // 5. 虚拟目标速度更新 V_next = V_i + rho * ΔV_ti
    assert(updateVirtualTarget(20.0, 0.5, 2.0) == 21.0);

    // 6. 虚拟目标限幅 V_next <= V_t + V_max_offset
    assert(limitVirtualTarget(21.0, 20.0, 3.0) == 21.0);
    assert(limitVirtualTarget(25.0, 20.0, 3.0) == 23.0); // 被限制在 20 + 3 = 23.0

    std::cout << "[PASS] 基础控制公式单元测试成功！" << std::endl;
}

void testSppvtPipeline() {
    SppvtConfig config;
    config.Kp = 50.0;
    config.rho = 0.5;
    config.delta = 0.05;
    config.maxOffset = 3.0; // 最大超设定 3.0m/s
    config.cooldownTime = 2.0;
    config.minTorque = 0.0;
    config.maxTorque = 300.0;

    SppvtState state;
    state.virtualTargetSpeed = 0.0;
    state.stepIndex = 0;
    state.cooldownTimer = 0.0;
    state.controlActive = false;

    // 1. 新的控制会话初始化 (式 7)
    SppvtInput input1;
    input1.currentSpeed = 15.0;
    input1.targetSpeed = 20.0;
    input1.currentAcceleration = 0.0;
    input1.dt = 0.1;
    input1.pauseControl = false;
    input1.isNewControlSession = true;

    SppvtOutput out1 = sppvtStep(config, input1, state);
    assert(state.controlActive == true);
    assert(state.stepIndex == 1);
    assert(state.virtualTargetSpeed == 20.0); // 初始 V1 = V_t = 20.0
    assert(out1.activeVirtualTarget == 20.0);
    // 扭矩输出 = Kp * (V_i - v) = 50 * (20 - 15) = 250
    assert(out1.torqueCommand == 250.0);

    // 2. 正常控制运行，未达到稳定点，加速度为 0.2 (大于 delta 0.05) -> 不更新目标
    input1.isNewControlSession = false;
    input1.currentAcceleration = 0.2;
    SppvtOutput out2 = sppvtStep(config, input1, state);
    assert(state.stepIndex == 1);
    assert(state.virtualTargetSpeed == 20.0);

    // 3. 达到平稳点 (加速度变为 0.01)，且当前车速 18.0 < 最终设定目标 20.0 -> 触发目标更新
    input1.currentSpeed = 18.0;
    input1.currentAcceleration = 0.01;
    SppvtOutput out3 = sppvtStep(config, input1, state);
    // 变目标计算:
    // deviation = 20.0 - 18.0 = 2.0
    // newTarget = 20.0 + 0.5 * 2.0 = 21.0
    assert(state.stepIndex == 2);
    assert(state.virtualTargetSpeed == 21.0);
    assert(state.cooldownTimer == 2.0); // 触发冷却
    // 扭矩输出 = 50 * (21 - 18) = 150
    assert(out3.torqueCommand == 150.0);

    // 4. 处于冷却期内，即使加速度再次降低，也不更新目标
    input1.currentSpeed = 19.0;
    input1.currentAcceleration = 0.01;
    input1.dt = 0.5; // 冷却定时器变为 2.0 - 0.5 = 1.5
    SppvtOutput out4 = sppvtStep(config, input1, state);
    assert(state.stepIndex == 2);
    assert(state.virtualTargetSpeed == 21.0);
    assert(state.cooldownTimer == 1.5);

    // 5. 跨越冷却期后，车速 19.0 仍小于 20.0，加速度 0.01 -> 再次触发更新并触发上限保护限制
    input1.dt = 1.6; // 冷却计时器自减到 0
    SppvtOutput out5 = sppvtStep(config, input1, state);
    // 变目标计算:
    // deviation = 20.0 - 19.0 = 1.0
    // V_3 = V_2 + rho * deviation = 21.0 + 0.5 * 1.0 = 21.5
    assert(state.stepIndex == 3);
    assert(state.virtualTargetSpeed == 21.5);
    assert(state.cooldownTimer == 2.0);

    // 6. 控制暂停处理
    input1.pauseControl = true;
    input1.dt = 0.1;
    SppvtOutput out6 = sppvtStep(config, input1, state);
    assert(state.controlActive == false);
    assert(out6.torqueCommand == 0.0);

    std::cout << "[PASS] SPPVT 变目标控制流流水线测试成功！" << std::endl;
}

int main() {
    std::cout << "========= 开始控制业务库测试 =========" << std::endl;
    testControlMath();
    testSppvtPipeline();
    std::cout << "======================================" << std::endl;
    std::cout << "所有控制库测试通过！" << std::endl;
    return 0;
}

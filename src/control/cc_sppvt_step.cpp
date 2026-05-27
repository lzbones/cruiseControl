#include "control/cc_sppvt_step.h"
#include "control/cc_calc_speed_error.h"
#include "control/cc_calc_control_torque.h"
#include "control/cc_check_target_change.h"
#include "control/cc_calc_target_deviation.h"
#include "control/cc_update_virtual_target.h"
#include "control/cc_limit_virtual_target.h"

namespace cc {
namespace control {

SppvtOutput sppvtStep(const SppvtConfig& config, const SppvtInput& input, SppvtState& state) {
    // 1. 冷却计时器自减
    if (state.cooldownTimer > 0.0) {
        state.cooldownTimer -= input.dt;
        if (state.cooldownTimer < 0.0) {
            state.cooldownTimer = 0.0;
        }
    }

    // 2. 控制会话重置逻辑 (式 7)
    if (input.isNewControlSession) {
        state.virtualTargetSpeed = input.targetSpeed;
        state.stepIndex = 1;
        state.cooldownTimer = 0.0; // 重置冷却计时器
        state.controlActive = true;
    }

    // 3. 处理暂停控制逻辑
    if (input.pauseControl) {
        state.controlActive = false;
        SppvtOutput out;
        out.torqueCommand = 0.0;
        out.activeVirtualTarget = 0.0;
        return out;
    }

    // 4. 变目标时刻判定与虚拟目标速度更新 (式 3, 4, 5, 6)
    // 新会话的首个周期直接跳过变目标检测（因为速度不可能已在当前虚拟目标下稳定）
    bool isTargetChangeInstant = false;
    if (!input.isNewControlSession) {
        isTargetChangeInstant = checkTargetChange(input.currentAcceleration, config.delta, state.cooldownTimer);
    }
    // 更新条件还需满足当前车速小于 CC 最终设定目标车速
    bool speedConditionMet = (input.currentSpeed < input.targetSpeed);

    if (isTargetChangeInstant && speedConditionMet) {
        double deviation = calcTargetDeviation(input.targetSpeed, input.currentSpeed);
        double updatedTarget = updateVirtualTarget(state.virtualTargetSpeed, config.rho, deviation);
        // 上限限幅保护，防暴冲 (积分饱和抗饱和机制)
        state.virtualTargetSpeed = limitVirtualTarget(updatedTarget, input.targetSpeed, config.maxOffset);
        
        state.stepIndex += 1;
        state.cooldownTimer = config.cooldownTime; // 开启冷却锁存，防止高频触发
    }

    // 5. 计算控制误差与输出控制扭矩 (式 1, 2)
    double speedError = calcSpeedError(state.virtualTargetSpeed, input.currentSpeed);
    double torque = calcControlTorque(config.Kp, speedError, config);

    // 6. 生成输出数据
    SppvtOutput out;
    out.torqueCommand = torque;
    out.activeVirtualTarget = state.virtualTargetSpeed;
    
    return out;
}

} // namespace control
} // namespace cc

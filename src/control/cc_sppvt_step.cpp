#include "control/cc_sppvt_step.h"
#include "control/cc_calc_speed_error.h"
#include "control/cc_calc_control_torque.h"
#include "control/cc_check_target_change.h"
#include "control/cc_calc_target_deviation.h"
#include "control/cc_update_virtual_target.h"
#include "control/cc_limit_virtual_target.h"

namespace cc {
namespace control {

namespace {

double updateCooldownTimer(double currentTimer, double dt) {
    const double decTimer = currentTimer - dt;
    const double result = (currentTimer > 0.0) ? ((decTimer < 0.0) ? 0.0 : decTimer) : currentTimer;
    return result;
}

double determineNextCooldownTimer(const SppvtConfig& config, const SppvtInput& input, const SppvtState& state, bool isTriggered) {
    const double timerDec = updateCooldownTimer(state.cooldownTimer, input.dt);
    const double timerSession = input.isNewControlSession ? 0.0 : timerDec;
    const double nextTimer = isTriggered ? config.cooldownTime : timerSession;
    return nextTimer;
}

double calcNewVirtualTarget(const SppvtConfig& config, const SppvtInput& input, const SppvtState& state) {
    const double deviation = calcTargetDeviation(input.targetSpeed, input.currentSpeed);
    const double updatedTarget = updateVirtualTarget(state.virtualTargetSpeed, config.rho, deviation);
    const double limitedTarget = limitVirtualTarget(updatedTarget, input.targetSpeed, config.maxOffset);
    return limitedTarget;
}

double determineNextVirtualTarget(const SppvtConfig& config, const SppvtInput& input, const SppvtState& state, bool isTriggered) {
    const double targetSession = input.isNewControlSession ? input.targetSpeed : state.virtualTargetSpeed;
    const double nextTarget = isTriggered ? calcNewVirtualTarget(config, input, state) : targetSession;
    return nextTarget;
}

int determineNextStepIndex(const SppvtInput& input, const SppvtState& state, bool isTriggered) {
    const int indexSession = input.isNewControlSession ? 1 : state.stepIndex;
    const int nextIndex = isTriggered ? (indexSession + 1) : indexSession;
    return nextIndex;
}

double determineTorqueCommand(const SppvtConfig& config, const SppvtInput& input, double virtualTargetSpeed) {
    const double speedError = calcSpeedError(virtualTargetSpeed, input.currentSpeed);
    const double torque = calcControlTorque(config.Kp, speedError, config);
    const double finalTorque = input.pauseControl ? 0.0 : torque;
    return finalTorque;
}

} // namespace

SppvtOutput sppvtStep(const SppvtConfig& config, const SppvtInput& input, SppvtState& state) {
    // 1. 计算是否触发变目标
    const double tempTimer = (state.cooldownTimer > 0.0) ? 
        ((state.cooldownTimer - input.dt < 0.0) ? 0.0 : state.cooldownTimer - input.dt) : 0.0;
    const double currentTimerForCheck = input.isNewControlSession ? 0.0 : tempTimer;

    const bool isTargetChangeInstant = !input.isNewControlSession && 
        checkTargetChange(input.currentAcceleration, config.delta, currentTimerForCheck);
    const bool speedConditionMet = (input.currentSpeed < input.targetSpeed);
    const bool isTriggered = !input.pauseControl && isTargetChangeInstant && speedConditionMet;

    // 2. 计算下一状态值 (SSA)
    const double nextTimer = input.pauseControl ? state.cooldownTimer :
        determineNextCooldownTimer(config, input, state, isTriggered);
    const double nextVirtualTarget = input.pauseControl ? state.virtualTargetSpeed :
        determineNextVirtualTarget(config, input, state, isTriggered);
    const int nextStepIndex = input.pauseControl ? state.stepIndex :
        determineNextStepIndex(input, state, isTriggered);
    const bool nextControlActive = input.pauseControl ? false :
        (input.isNewControlSession ? true : state.controlActive);

    // 3. 更新持久状态
    state.cooldownTimer = nextTimer;
    state.virtualTargetSpeed = nextVirtualTarget;
    state.stepIndex = nextStepIndex;
    state.controlActive = nextControlActive;

    // 4. 计算控制扭矩与输出数据
    const double torque = determineTorqueCommand(config, input, nextVirtualTarget);
    const double activeTarget = input.pauseControl ? 0.0 : nextVirtualTarget;

    const SppvtOutput out = {torque, activeTarget};
    return out;
}

} // namespace control
} // namespace cc

#include "control/cc_sppvt_step.h"
#include "control/cc_determine_triggered.h"
#include "control/cc_determine_next_cooldown_timer.h"
#include "control/cc_determine_next_virtual_target.h"
#include "control/cc_determine_next_step_index.h"
#include "control/cc_determine_torque_command.h"

namespace cc {
namespace control {

SppvtOutput sppvtStep(const SppvtConfig& config, const SppvtInput& input, SppvtState& state) {
    // 1. 判断是否触发变目标条件
    const bool isTriggered = determineTriggered(config, input, state);

    // 2. 计算下一时刻状态值 (SSA)
    const double nextTimer = input.pauseControl ? state.cooldownTimer :
        determineNextCooldownTimer(config, input, state, isTriggered);
    const double nextVirtualTarget = input.pauseControl ? state.virtualTargetSpeed :
        determineNextVirtualTarget(config, input, state, isTriggered);
    const int nextStepIndex = input.pauseControl ? state.stepIndex :
        determineNextStepIndex(input, state, isTriggered);
    const bool nextControlActive = input.pauseControl ? false :
        (input.isNewControlSession ? true : state.controlActive);

    // 3. 更新持久化状态
    state.cooldownTimer = nextTimer;
    state.virtualTargetSpeed = nextVirtualTarget;
    state.stepIndex = nextStepIndex;
    state.controlActive = nextControlActive;

    // 4. 计算控制输出扭矩与目标车速
    const double torque = determineTorqueCommand(config, input, nextVirtualTarget);
    const double activeTarget = input.pauseControl ? 0.0 : nextVirtualTarget;

    const SppvtOutput out = {torque, activeTarget};
    return out;
}

} // namespace control
} // namespace cc


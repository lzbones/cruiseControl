#include "control/cc_determine_triggered.h"
#include "control/cc_check_target_change.h"

namespace cc {
namespace control {

bool determineTriggered(const SppvtConfig& config, const SppvtInput& input, const SppvtState& state) {
    const double tempTimer = (state.cooldownTimer > 0.0) ? 
        ((state.cooldownTimer - input.dt < 0.0) ? 0.0 : state.cooldownTimer - input.dt) : 0.0;
    const double currentTimerForCheck = input.isNewControlSession ? 0.0 : tempTimer;

    const bool isTargetChangeInstant = !input.isNewControlSession && 
        checkTargetChange(input.currentAcceleration, config.delta, currentTimerForCheck);
    const bool speedConditionMet = (input.currentSpeed < input.targetSpeed);
    const bool isTriggered = !input.pauseControl && isTargetChangeInstant && speedConditionMet;
    
    return isTriggered;
}

} // namespace control
} // namespace cc

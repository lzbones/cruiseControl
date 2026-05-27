#include "control/cc_determine_next_cooldown_timer.h"
#include "control/cc_update_cooldown_timer.h"

namespace cc {
namespace control {

double determineNextCooldownTimer(const SppvtConfig& config, const SppvtInput& input, const SppvtState& state, bool isTriggered) {
    const double timerDec = updateCooldownTimer(state.cooldownTimer, input.dt);
    const double timerSession = input.isNewControlSession ? 0.0 : timerDec;
    const double nextTimer = isTriggered ? config.cooldownTime : timerSession;
    return nextTimer;
}

} // namespace control
} // namespace cc

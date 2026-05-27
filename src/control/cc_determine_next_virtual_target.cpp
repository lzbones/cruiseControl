#include "control/cc_determine_next_virtual_target.h"
#include "control/cc_calc_new_virtual_target.h"

namespace cc {
namespace control {

double determineNextVirtualTarget(const SppvtConfig& config, const SppvtInput& input, const SppvtState& state, bool isTriggered) {
    const double targetSession = input.isNewControlSession ? input.targetSpeed : state.virtualTargetSpeed;
    const double nextTarget = isTriggered ? calcNewVirtualTarget(config, input, state) : targetSession;
    return nextTarget;
}

} // namespace control
} // namespace cc

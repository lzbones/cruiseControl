#include "control/cc_determine_next_step_index.h"

namespace cc {
namespace control {

int determineNextStepIndex(const SppvtInput& input, const SppvtState& state, bool isTriggered) {
    const int indexSession = input.isNewControlSession ? 1 : state.stepIndex;
    const int nextIndex = isTriggered ? (indexSession + 1) : indexSession;
    return nextIndex;
}

} // namespace control
} // namespace cc

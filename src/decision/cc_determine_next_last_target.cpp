#include "decision/cc_determine_next_last_target.h"

namespace cc {
namespace decision {

double determineNextLastTarget(DecisionResult result, SystemState determinedState, const DecisionState& state) {
    const bool isCurrentlyActive = (determinedState == SYS_STATE_S0 || determinedState == SYS_STATE_S1 || determinedState == SYS_STATE_S6);
    const bool isExit = (result == RESULT_R6 || result == RESULT_R7 || result == RESULT_R8);
    const double nextLastTarget = (isExit && isCurrentlyActive) ? state.targetSpeed : state.lastTargetSpeed;
    return nextLastTarget;
}

} // namespace decision
} // namespace cc

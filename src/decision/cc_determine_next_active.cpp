#include "decision/cc_determine_next_active.h"

namespace cc {
namespace decision {

bool determineNextActive(DecisionResult result, bool currentActive) {
    const bool isExit = (result == RESULT_R6 || result == RESULT_R7 || result == RESULT_R8);
    const bool isEnter = (result == RESULT_R1 || result == RESULT_R2 || result == RESULT_R3 || result == RESULT_R4 || result == RESULT_R5);
    const bool nextActive = isExit ? false : (isEnter ? true : currentActive);
    return nextActive;
}

} // namespace decision
} // namespace cc

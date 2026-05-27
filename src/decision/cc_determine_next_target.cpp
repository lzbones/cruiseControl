#include "decision/cc_determine_next_target.h"

namespace cc {
namespace decision {

double determineNextTarget(DecisionResult result, const DecisionInput& input, const DecisionConfig& config, const DecisionState& state) {
    const double targetIncRaw = state.targetSpeed + config.deltaSpeedStep;
    const double targetInc = (targetIncRaw > config.maxSpeed) ? config.maxSpeed : targetIncRaw;

    const double targetDecRaw = state.targetSpeed - config.deltaSpeedStep;
    const double targetDec = (targetDecRaw < config.minSpeed) ? config.minSpeed : targetDecRaw;

    const double targetSetRaw = input.currentSpeed;
    const double targetSet = (targetSetRaw < config.minSpeed) ? config.minSpeed :
                             ((targetSetRaw > config.maxSpeed) ? config.maxSpeed : targetSetRaw);

    const double nextTarget = 
        (result == RESULT_R1) ? targetInc :
        (result == RESULT_R2) ? targetDec :
        (result == RESULT_R4) ? state.lastTargetSpeed :
        (result == RESULT_R5) ? targetSet : state.targetSpeed;

    return nextTarget;
}

} // namespace decision
} // namespace cc

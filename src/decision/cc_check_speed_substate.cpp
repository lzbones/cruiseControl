#include "decision/cc_check_speed_substate.h"

namespace cc {
namespace decision {

SpeedSubstate checkSpeedSubstate(double currentSpeed, const DecisionConfig& config) {
    const SpeedSubstate result = (currentSpeed < config.minSpeed) ? SPEED_SUBSTATE_LOW :
                                 ((currentSpeed > config.maxSpeed) ? SPEED_SUBSTATE_HIGH : SPEED_SUBSTATE_SUITABLE);
    return result;
}

} // namespace decision
} // namespace cc

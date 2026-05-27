#include "decision/cc_check_speed_substate.h"

namespace cc {
namespace decision {

SpeedSubstate checkSpeedSubstate(double currentSpeed, const DecisionConfig& config) {
    if (currentSpeed < config.minSpeed) {
        return SPEED_SUBSTATE_LOW;
    } else if (currentSpeed > config.maxSpeed) {
        return SPEED_SUBSTATE_HIGH;
    } else {
        return SPEED_SUBSTATE_SUITABLE;
    }
}

} // namespace decision
} // namespace cc

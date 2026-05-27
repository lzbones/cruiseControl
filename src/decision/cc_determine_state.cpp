#include "decision/cc_determine_state.h"

namespace cc {
namespace decision {

SystemState determineState(SpeedSubstate speedSubstate, SystemSubstate systemSubstate) {
    // 待命状态集合: Ss0 ~ Ss3 (j 为 0 ~ 3)
    const bool isStandby = (static_cast<int>(systemSubstate) <= 3);
    
    const SystemState result = 
        (speedSubstate == SPEED_SUBSTATE_SUITABLE) ? (
            (systemSubstate == SYS_SUBSTATE_S0) ? SYS_STATE_S3 : (
            (systemSubstate == SYS_SUBSTATE_S1) ? SYS_STATE_S2 : (
            (systemSubstate == SYS_SUBSTATE_S2 || systemSubstate == SYS_SUBSTATE_S3) ? SYS_STATE_S4 :
            SYS_STATE_S0
            ))
        ) : (
        (speedSubstate == SPEED_SUBSTATE_LOW) ? (
            isStandby ? SYS_STATE_S5 : SYS_STATE_S6
        ) : ( // SPEED_SUBSTATE_HIGH
            isStandby ? SYS_STATE_S5 : SYS_STATE_S1
        ));
        
    return result;
}

} // namespace decision
} // namespace cc

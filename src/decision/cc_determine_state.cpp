#include "decision/cc_determine_state.h"

namespace cc {
namespace decision {

SystemState determineState(SpeedSubstate speedSubstate, SystemSubstate systemSubstate) {
    // 待命状态集合: Ss0 ~ Ss3 (j 为 0 ~ 3)
    bool isStandby = (static_cast<int>(systemSubstate) <= 3);
    
    if (speedSubstate == SPEED_SUBSTATE_SUITABLE) {
        if (systemSubstate == SYS_SUBSTATE_S0) {
            return SYS_STATE_S3; // 适速无油无史待命
        } else if (systemSubstate == SYS_SUBSTATE_S1) {
            return SYS_STATE_S2; // 适速无油有史待命
        } else if (systemSubstate == SYS_SUBSTATE_S2 || systemSubstate == SYS_SUBSTATE_S3) {
            return SYS_STATE_S4; // 有油待命
        } else {
            return SYS_STATE_S0; // 适速在控 (S4 ~ S7)
        }
    } else if (speedSubstate == SPEED_SUBSTATE_LOW) {
        if (isStandby) {
            return SYS_STATE_S5; // 非适速待命
        } else {
            return SYS_STATE_S6; // 低速在控
        }
    } else { // SPEED_SUBSTATE_HIGH
        if (isStandby) {
            return SYS_STATE_S5; // 非适速待命
        } else {
            return SYS_STATE_S1; // 高速在控
        }
    }
}

} // namespace decision
} // namespace cc

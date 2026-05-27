#include "decision/cc_evaluate_decision.h"

namespace cc {
namespace decision {

DecisionResult evaluateDecision(SystemState determinedState, double lowSpeedTimer, double lowSpeedTimeout, DriverCommand driverCommand) {
    const DecisionResult result = 
        (determinedState == SYS_STATE_S6 && lowSpeedTimer >= lowSpeedTimeout) ? RESULT_R7 :
        (driverCommand == COMMAND_INC) ? (
            (determinedState == SYS_STATE_S0) ? RESULT_R1 :
            (determinedState == SYS_STATE_S1) ? RESULT_R3 :
            (determinedState == SYS_STATE_S2) ? RESULT_R4 :
            (determinedState == SYS_STATE_S3 || determinedState == SYS_STATE_S4) ? RESULT_R5 :
            (determinedState == SYS_STATE_S5) ? RESULT_R6 :
            (determinedState == SYS_STATE_S6) ? RESULT_R7 : RESULT_NONE
        ) :
        (driverCommand == COMMAND_DEC) ? (
            (determinedState == SYS_STATE_S0) ? RESULT_R2 :
            (determinedState == SYS_STATE_S1) ? RESULT_R3 :
            (determinedState == SYS_STATE_S2) ? RESULT_R6 :
            (determinedState == SYS_STATE_S3 || determinedState == SYS_STATE_S4) ? RESULT_R5 :
            (determinedState == SYS_STATE_S5) ? RESULT_R6 :
            (determinedState == SYS_STATE_S6) ? RESULT_R7 : RESULT_NONE
        ) :
        (driverCommand == COMMAND_CANCEL) ? RESULT_R6 :
        (driverCommand == COMMAND_BRAKE) ? RESULT_R6 :
        (driverCommand == COMMAND_EXIT) ? RESULT_R8 : RESULT_NONE;
        
    return result;
}

} // namespace decision
} // namespace cc

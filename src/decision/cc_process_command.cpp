#include "decision/cc_process_command.h"
#include "decision/cc_check_speed_substate.h"
#include "decision/cc_check_system_substate.h"
#include "decision/cc_determine_state.h"

namespace cc {
namespace decision {

namespace {

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

bool determineNextActive(DecisionResult result, bool currentActive) {
    const bool isExit = (result == RESULT_R6 || result == RESULT_R7 || result == RESULT_R8);
    const bool isEnter = (result == RESULT_R1 || result == RESULT_R2 || result == RESULT_R3 || result == RESULT_R4 || result == RESULT_R5);
    const bool nextActive = isExit ? false : (isEnter ? true : currentActive);
    return nextActive;
}

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

double determineNextLastTarget(DecisionResult result, SystemState determinedState, const DecisionState& state) {
    const bool isCurrentlyActive = (determinedState == SYS_STATE_S0 || determinedState == SYS_STATE_S1 || determinedState == SYS_STATE_S6);
    const bool isExit = (result == RESULT_R6 || result == RESULT_R7 || result == RESULT_R8);
    const double nextLastTarget = (isExit && isCurrentlyActive) ? state.targetSpeed : state.lastTargetSpeed;
    return nextLastTarget;
}

} // namespace

DecisionOutput processCommand(const DecisionInput& input, const DecisionConfig& config, DecisionState& state) {
    // 1. 识别当前车速子态 Sv 与系统子态集 Ss，决定当前瞬间状态 Sn
    const SpeedSubstate speedSub = checkSpeedSubstate(input.currentSpeed, config);
    const SystemSubstate systemSub = checkSystemSubstate(input.controlActive, input.isThrottlePressed, input.hasHistoryTarget);
    const SystemState determinedState = determineState(speedSub, systemSub);
    
    // 更新当前状态
    state.currentState = determinedState;

    // 2. 更新低速在控状态 S6 的持续计时器
    state.lowSpeedTimer = (determinedState == SYS_STATE_S6) ? (state.lowSpeedTimer + input.dt) : 0.0;

    // 3. 执行决策判定 (表 5 + 安全超时主动触发)
    const DecisionResult result = evaluateDecision(determinedState, state.lowSpeedTimer, config.lowSpeedTimeout, input.driverCommand);

    // 4. 执行决策对应的状态与目标速度更新
    const double nextTarget = determineNextTarget(result, input, config, state);
    const bool nextActive = determineNextActive(result, input.controlActive);
    const double nextLastTarget = determineNextLastTarget(result, determinedState, state);

    state.targetSpeed = nextTarget;
    state.lastTargetSpeed = nextLastTarget;

    // 5. 预测并流转至下一状态
    const bool nextHasHistory = (nextLastTarget > 0.0);
    const SystemSubstate nextSysSub = checkSystemSubstate(nextActive, input.isThrottlePressed, nextHasHistory);
    const SystemState nextDeterminedState = determineState(speedSub, nextSysSub);

    const DecisionOutput output = {result, nextDeterminedState, nextTarget};
    return output;
}

} // namespace decision
} // namespace cc

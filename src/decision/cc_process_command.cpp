#include "decision/cc_process_command.h"
#include "decision/cc_check_speed_substate.h"
#include "decision/cc_check_system_substate.h"
#include "decision/cc_determine_state.h"

namespace cc {
namespace decision {

DecisionOutput processCommand(const DecisionInput& input, const DecisionConfig& config, DecisionState& state) {
    // 1. 识别当前车速子态 Sv 与系统子态集 Ss，决定当前瞬间状态 Sn
    SpeedSubstate speedSub = checkSpeedSubstate(input.currentSpeed, config);
    SystemSubstate systemSub = checkSystemSubstate(input.controlActive, input.isThrottlePressed, input.hasHistoryTarget);
    SystemState determinedState = determineState(speedSub, systemSub);
    
    // 更新当前状态
    state.currentState = determinedState;

    // 2. 更新低速在控状态 S6 的持续计时器
    if (determinedState == SYS_STATE_S6) {
        state.lowSpeedTimer += input.dt;
    } else {
        state.lowSpeedTimer = 0.0;
    }

    // 3. 执行决策判定 (表 5 + 安全超时主动触发)
    DecisionResult result = RESULT_NONE;

    // A. 优先处理低速在控主动超时退出保护 (安全优化)
    if (determinedState == SYS_STATE_S6 && state.lowSpeedTimer >= config.lowSpeedTimeout) {
        result = RESULT_R7; // 触发报错待命
    }
    // B. 处理正常驾驶指令
    else {
        switch (input.driverCommand) {
            case COMMAND_NONE:
                result = RESULT_NONE;
                break;
                
            case COMMAND_INC: // I0: 增速键
                if (determinedState == SYS_STATE_S0) {
                    result = RESULT_R1; // 目标增量
                } else if (determinedState == SYS_STATE_S1) {
                    result = RESULT_R3; // 保持控制
                } else if (determinedState == SYS_STATE_S2) {
                    result = RESULT_R4; // 继承控制 (Resume)
                } else if (determinedState == SYS_STATE_S3 || determinedState == SYS_STATE_S4) {
                    result = RESULT_R5; // 无继控制 (Set)
                } else if (determinedState == SYS_STATE_S5) {
                    result = RESULT_R6; // 系统待命
                } else if (determinedState == SYS_STATE_S6) {
                    result = RESULT_R7; // 报错待命
                }
                break;
                
            case COMMAND_DEC: // I1: 降速键
                if (determinedState == SYS_STATE_S0) {
                    result = RESULT_R2; // 目标减量
                } else if (determinedState == SYS_STATE_S1) {
                    result = RESULT_R3; // 保持控制
                } else if (determinedState == SYS_STATE_S2) {
                    result = RESULT_R6; // 系统待命
                } else if (determinedState == SYS_STATE_S3 || determinedState == SYS_STATE_S4) {
                    result = RESULT_R5; // 无继控制 (Set)
                } else if (determinedState == SYS_STATE_S5) {
                    result = RESULT_R6; // 系统待命
                } else if (determinedState == SYS_STATE_S6) {
                    result = RESULT_R7; // 报错待命
                }
                break;
                
            case COMMAND_CANCEL: // I2: 取消键
                result = RESULT_R6; // 系统待命
                break;
                
            case COMMAND_BRAKE: // I3: 刹车踩下
                result = RESULT_R6; // 系统待命
                break;
                
            case COMMAND_EXIT: // I4: 退出键
                result = RESULT_R8; // 系统退出
                break;
        }
    }

    // 4. 执行决策对应的状态与目标速度更新
    double nextTarget = state.targetSpeed;
    bool nextActive = input.controlActive;

    // 当发生控制退出决策时，如果当前在控，需要把当前设定车速存入历史目标中以便后续继承
    bool isCurrentlyActive = (determinedState == SYS_STATE_S0 || determinedState == SYS_STATE_S1 || determinedState == SYS_STATE_S6);
    if (result == RESULT_R6 || result == RESULT_R7 || result == RESULT_R8) {
        if (isCurrentlyActive) {
            state.lastTargetSpeed = state.targetSpeed;
        }
        nextActive = false;
    }

    // 执行目标速度修改
    switch (result) {
        case RESULT_R1: // 目标增量
            nextTarget = state.targetSpeed + config.deltaSpeedStep;
            if (nextTarget > config.maxSpeed) {
                nextTarget = config.maxSpeed;
            }
            nextActive = true;
            break;
            
        case RESULT_R2: // 目标减量
            nextTarget = state.targetSpeed - config.deltaSpeedStep;
            if (nextTarget < config.minSpeed) {
                nextTarget = config.minSpeed;
            }
            nextActive = true;
            break;
            
        case RESULT_R4: // 继承控制
            nextTarget = state.lastTargetSpeed;
            nextActive = true;
            break;
            
        case RESULT_R5: // 无继控制
            nextTarget = input.currentSpeed;
            if (nextTarget < config.minSpeed) {
                nextTarget = config.minSpeed;
            } else if (nextTarget > config.maxSpeed) {
                nextTarget = config.maxSpeed;
            }
            nextActive = true;
            break;
            
        case RESULT_R3: // 保持控制
            nextActive = true;
            break;
            
        default:
            break;
    }
    
    state.targetSpeed = nextTarget;

    // 5. 预测并流转至下一状态
    bool nextHasHistory = (state.lastTargetSpeed > 0.0);
    SystemSubstate nextSysSub = checkSystemSubstate(nextActive, input.isThrottlePressed, nextHasHistory);
    // 注意：这里的车速子态在下一个控制周期到来前，可以使用当前车速进行预估
    SystemState nextDeterminedState = determineState(speedSub, nextSysSub);

    DecisionOutput output;
    output.decision = result;
    output.nextState = nextDeterminedState;
    output.updatedTargetSpeed = nextTarget;

    return output;
}

} // namespace decision
} // namespace cc

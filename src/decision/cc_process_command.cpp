#include "decision/cc_process_command.h"
#include "decision/cc_check_speed_substate.h"
#include "decision/cc_check_system_substate.h"
#include "decision/cc_determine_state.h"
#include "decision/cc_evaluate_decision.h"
#include "decision/cc_determine_next_active.h"
#include "decision/cc_determine_next_target.h"
#include "decision/cc_determine_next_last_target.h"

namespace cc {
namespace decision {

// 私有文件级静态状态变量，初始状态为适速无油无史待命 (SYS_STATE_S3)
static DecisionState s_state = {SYS_STATE_S3, 0.0, 0.0, 0.0};

const DecisionState& getDecisionState() {
  return s_state;
}

void setDecisionState(const DecisionState& state) {
  s_state = state;
}

DecisionOutput processCommand(const DecisionInput& input, const DecisionConfig& config) {
  // 1. 识别当前车速子态 Sv 与系统子态集 Ss，决定当前瞬间状态 Sn
  const SpeedSubstate speedSub = checkSpeedSubstate(input.currentSpeed, config);
  const SystemSubstate systemSub = checkSystemSubstate(input.controlActive, input.isThrottlePressed, input.hasHistoryTarget);
  const SystemState determinedState = determineState(speedSub, systemSub);
  
  // 更新当前状态
  s_state.currentState = determinedState;

  // 2. 更新低速在控状态 S6 的持续计时器
  s_state.lowSpeedTimer = (determinedState == SYS_STATE_S6) ? (s_state.lowSpeedTimer + input.dt) : 0.0;

  // 3. 执行决策判定 (表 5 + 安全超时主动触发)
  const DecisionResult result = evaluateDecision(determinedState, s_state.lowSpeedTimer, config.lowSpeedTimeout, input.driverCommand);

  // 4. 执行决策对应的状态与目标速度更新
  const double nextTarget = determineNextTarget(result, input, config, s_state);
  const bool nextActive = determineNextActive(result, input.controlActive);
  const double nextLastTarget = determineNextLastTarget(result, determinedState, s_state);

  s_state.targetSpeed = nextTarget;
  s_state.lastTargetSpeed = nextLastTarget;

  // 5. 预测并流转至下一状态
  const bool nextHasHistory = (nextLastTarget > 0.0);
  const SystemSubstate nextSysSub = checkSystemSubstate(nextActive, input.isThrottlePressed, nextHasHistory);
  const SystemState nextDeterminedState = determineState(speedSub, nextSysSub);

  const DecisionOutput output = {result, nextDeterminedState, nextTarget};
  return output;
}
} // namespace decision
} // namespace cc


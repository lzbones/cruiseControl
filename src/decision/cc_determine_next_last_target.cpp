#include "decision/cc_determine_next_last_target.h"

namespace cc {
namespace decision {

double determineNextLastTarget(DecisionResult result, SystemState determinedState, const DecisionState& state) {
  const bool isCurrentlyActive = (determinedState == SYS_STATE_S0 || determinedState == SYS_STATE_S1 || determinedState == SYS_STATE_S6);
  const bool isExit = (result == RESULT_R6 || result == RESULT_R7 || result == RESULT_R8);
  
  // 严格 SSA 赋值：如果是 R8 系统退出，则历史车速清空为 0.0；
  // 如果是 R6/R7 待命且当前在控，则保存当前目标车速；否则保持历史车速。
  const double nextLastTarget = (result == RESULT_R8) ? 0.0 : 
                                ((isExit && isCurrentlyActive) ? state.targetSpeed : state.lastTargetSpeed);
  
  return nextLastTarget;
}


} // namespace decision
} // namespace cc

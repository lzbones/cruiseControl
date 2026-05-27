#pragma once

#include "decision/cc_decision_def.h"

namespace cc {
namespace decision {

/**
 * @brief 根据当前决策判定结果决定下一时刻控制环是否处于激活（在控）状态
 * @param result 当前周期决策响应结果 Rk
 * @param currentActive 当前周期控制激活状态
 * @return 是否激活下一周期控制环
 */
bool determineNextActive(DecisionResult result, bool currentActive);

} // namespace decision
} // namespace cc

#pragma once

#include "decision/cc_decision_def.h"

namespace cc {
namespace decision {

/**
 * @brief 根据当前决策判定结果决定下一时刻控制环是否处于激活（在控）状态
 * 
 * 模块要素界定：
 * - 输入 (Input)：
 *   - result: 当前周期决策响应结果 Rk
 *   - currentActive: 当前周期控制激活状态
 * - 输出 (Output)：
 *   - 返回值: 是否激活下一周期控制环
 * - 状态 (State)：
 *   - 无 (由上级同步管理)
 * - 参数 (Param)：
 *   - 无
 */
bool determineNextActive(DecisionResult result, bool currentActive);

} // namespace decision
} // namespace cc

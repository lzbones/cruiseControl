#pragma once

#include "decision/cc_decision_def.h"

namespace cc {
namespace decision {

/**
 * @brief 根据当前运行状态、低速在控计时器与驾驶员按键指令决定系统响应动作 Rk
 * 
 * 模块要素界定：
 * - 输入 (Input)：
 *   - determinedState: 当前判定的系统状态 Sn
 *   - lowSpeedTimer: 低速在控计时器 (s)
 *   - driverCommand: 驾驶员按键/踏板指令
 * - 输出 (Output)：
 *   - 返回值: DecisionResult 系统决策响应动作 Rk
 * - 状态 (State)：
 *   - 无 (无持久化状态)
 * - 参数 (Param)：
 *   - lowSpeedTimeout: 最低车速超时保护时长 (s)
 */
DecisionResult evaluateDecision(SystemState determinedState, double lowSpeedTimer, double lowSpeedTimeout, DriverCommand driverCommand);

} // namespace decision
} // namespace cc

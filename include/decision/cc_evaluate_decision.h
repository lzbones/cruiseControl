#pragma once

#include "decision/cc_decision_def.h"

namespace cc {
namespace decision {

/**
 * @brief 根据当前运行状态、低速在控计时器与驾驶员按键指令决定系统响应动作 Rk
 * @param determinedState 当前判定的系统状态 Sn
 * @param lowSpeedTimer 低速在控计时器 (s)
 * @param lowSpeedTimeout 低速超时保护阈值 (s)
 * @param driverCommand 驾驶员按键/踏板指令
 * @return DecisionResult 系统决策响应动作 Rk
 */
DecisionResult evaluateDecision(SystemState determinedState, double lowSpeedTimer, double lowSpeedTimeout, DriverCommand driverCommand);

} // namespace decision
} // namespace cc

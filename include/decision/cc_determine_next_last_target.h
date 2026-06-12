#pragma once

#include "decision/cc_decision_def.h"

namespace cc {
namespace decision {

/**
 * @brief 根据当前运行状态及决策结果计算下一时刻的历史保存目标车速
 * 
 * 模块要素界定：
 * - 输入 (Input)：
 *   - result: 决策响应结果 Rk
 *   - determinedState: 当前周期系统状态 Sn
 * - 输出 (Output)：
 *   - 返回值: 下一周期历史目标车速 (m/s)
 * - 状态 (State)：
 *   - state.targetSpeed: 当前周期目标车速 (m/s)
 *   - state.lastTargetSpeed: 当前周期历史目标车速 (m/s)
 * - 参数 (Param)：
 *   - 无
 */
double determineNextLastTarget(DecisionResult result, SystemState determinedState, const DecisionState& state);

} // namespace decision
} // namespace cc

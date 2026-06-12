#pragma once

#include "decision/cc_decision_def.h"

namespace cc {
namespace decision {

/**
 * @brief 根据决策结果计算下一时刻设定目标车速
 * 
 * 模块要素界定：
 * - 输入 (Input)：
 *   - result: 决策响应结果 Rk
 *   - input.currentSpeed: 当前实际车速 (m/s)
 * - 输出 (Output)：
 *   - 返回值: 下一周期设定目标车速 (m/s)
 * - 状态 (State)：
 *   - state.targetSpeed: 当前设定目标车速 (m/s)
 *   - state.lastTargetSpeed: 上一次历史保存目标车速 (m/s)
 * - 参数 (Param)：
 *   - config.minSpeed: 最低车速限幅 (m/s)
 *   - config.maxSpeed: 最高车速限幅 (m/s)
 *   - config.deltaSpeedStep: 增减速步长 (m/s)
 */
double determineNextTarget(DecisionResult result, const DecisionInput& input, const DecisionConfig& config, const DecisionState& state);

} // namespace decision
} // namespace cc

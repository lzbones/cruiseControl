#pragma once

#include "decision/cc_decision_def.h"

namespace cc {
namespace decision {

/**
 * @brief 根据决策结果计算下一时刻设定目标车速
 * @param result 决策响应结果 Rk
 * @param input 决策周期输入信号
 * @param config 决策配置参数
 * @param state 当前决策状态变量
 * @return 下一周期设定目标车速 (m/s)
 */
double determineNextTarget(DecisionResult result, const DecisionInput& input, const DecisionConfig& config, const DecisionState& state);

} // namespace decision
} // namespace cc

#pragma once

#include "decision/cc_decision_def.h"

namespace cc {
namespace decision {

/**
 * @brief 根据当前运行状态及决策结果计算下一时刻的历史保存目标车速
 * @param result 决策响应结果 Rk
 * @param determinedState 当前判定的系统状态 Sn
 * @param state 当前决策状态变量
 * @return 下一周期历史目标车速 (m/s)
 */
double determineNextLastTarget(DecisionResult result, SystemState determinedState, const DecisionState& state);

} // namespace decision
} // namespace cc

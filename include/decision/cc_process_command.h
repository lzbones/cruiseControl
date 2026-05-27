#pragma once

#include "decision/cc_decision_def.h"

namespace cc {
namespace decision {

/**
 * @brief 根据当前输入信号、系统状态和配置参数计算决策行为，并更新状态机
 * @param input 周期输入信号
 * @param config 配置参数
 * @param state 当前持久化状态变量 (调用后会被就地更新)
 * @return DecisionOutput 周期决策输出 (Rk 结果与流转后的新状态)
 */
DecisionOutput processCommand(const DecisionInput& input, const DecisionConfig& config, DecisionState& state);

} // namespace decision
} // namespace cc


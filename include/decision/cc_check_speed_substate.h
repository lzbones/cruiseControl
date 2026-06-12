#pragma once

#include "decision/cc_decision_def.h"

namespace cc {
namespace decision {

/**
 * @brief 根据当前车速和阈值判定车速子态 Sv
 * 
 * 模块要素界定：
 * - 输入 (Input)：
 *   - currentSpeed: 当前实际车速 (m/s)
 * - 输出 (Output)：
 *   - 返回值: SpeedSubstate 车速子态
 * - 状态 (State)：
 *   - 无 (无持久化状态)
 * - 参数 (Param)：
 *   - config.minSpeed: 最低适控车速
 *   - config.maxSpeed: 最高适控车速
 */
SpeedSubstate checkSpeedSubstate(double currentSpeed, const DecisionConfig& config);

} // namespace decision
} // namespace cc


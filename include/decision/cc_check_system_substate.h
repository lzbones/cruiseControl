#pragma once

#include "decision/cc_decision_def.h"

namespace cc {
namespace decision {

/**
 * @brief 根据控制状态、油门状态与历史目标状态编码计算系统子态 Ss
 * 
 * 模块要素界定：
 * - 输入 (Input)：
 *   - controlActive: 是否在控 (b2: 0=待命, 1=在控)
 *   - isThrottlePressed: 油门是否被踩下 (b1: 0=无油, 1=有油)
 *   - hasHistoryTarget: 是否有历史目标车速 (b0: 0=无史, 1=有史)
 * - 输出 (Output)：
 *   - 返回值: SystemSubstate 编码后的系统子态 (0~7)
 * - 状态 (State)：
 *   - 无 (无持久化状态)
 * - 参数 (Param)：
 *   - 无
 */
SystemSubstate checkSystemSubstate(bool controlActive, bool isThrottlePressed, bool hasHistoryTarget);

} // namespace decision
} // namespace cc


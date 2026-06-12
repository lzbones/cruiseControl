#pragma once

#include "decision/cc_decision_def.h"

namespace cc {
namespace decision {

/**
 * @brief 根据车速子态和系统子态，通过表格映射确定当前的系统状态 Sn (表 4)
 * 
 * 模块要素界定：
 * - 输入 (Input)：
 *   - speedSubstate: 车速子态 Sv
 *   - systemSubstate: 系统子态 Ss
 * - 输出 (Output)：
 *   - 返回值: SystemState 确定的系统状态 Sn
 * - 状态 (State)：
 *   - 无 (无持久化状态)
 * - 参数 (Param)：
 *   - 无
 */
SystemState determineState(SpeedSubstate speedSubstate, SystemSubstate systemSubstate);

} // namespace decision
} // namespace cc


#pragma once

#include "decision/cc_decision_def.h"

namespace cc {
namespace decision {

/**
 * @brief 根据车速子态和系统子态，通过表格映射确定当前的系统状态 Sn (表 4)
 * @param speedSubstate 车速子态 Sv
 * @param systemSubstate 系统子态 Ss
 * @return SystemState 确定的系统状态 Sn
 */
SystemState determineState(SpeedSubstate speedSubstate, SystemSubstate systemSubstate);

} // namespace decision
} // namespace cc


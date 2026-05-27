#pragma once

#include "control/cc_control_def.h"

namespace cc {
namespace control {

/**
 * @brief 根据会话状态和是否触发变目标计算下一时刻阶数 i
 * @param input 输入信号
 * @param state 当前控制器状态
 * @param isTriggered 是否触发变目标
 * @return 下一时刻的阶数
 */
int determineNextStepIndex(const SppvtInput& input, const SppvtState& state, bool isTriggered);

} // namespace control
} // namespace cc

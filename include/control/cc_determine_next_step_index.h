#pragma once

#include "control/cc_control_def.h"

namespace cc {
namespace control {

/**
 * @brief 根据会话状态和是否触发变目标计算下一时刻阶数 i
 * 
 * 模块要素界定：
 * - 输入 (Input)：
 *   - input.isNewControlSession: 是否是新开启的控制会话
 *   - isTriggered: 是否触发了目标更新
 * - 输出 (Output)：
 *   - 返回值: 下一时刻的阶数
 * - 状态 (State)：
 *   - state.stepIndex: 当前阶数 i (跨周期持久化)
 * - 参数 (Param)：
 *   - 无
 */
int determineNextStepIndex(const SppvtInput& input, const SppvtState& state, bool isTriggered);

} // namespace control
} // namespace cc

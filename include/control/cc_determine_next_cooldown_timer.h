#pragma once

#include "control/cc_control_def.h"

namespace cc {
namespace control {

/**
 * @brief 计算下一时刻的冷却计时器值
 * 
 * 模块要素界定：
 * - 输入 (Input)：
 *   - input.isNewControlSession: 是否是新开启的控制会话
 *   - input.dt: 采样周期步长 (s)
 *   - isTriggered: 本周期是否触发了目标更新
 * - 输出 (Output)：
 *   - 返回值: 下一时刻的冷却计时器值 (s)
 * - 状态 (State)：
 *   - state.cooldownTimer: 当前冷却计时器 (跨周期持久化)
 * - 参数 (Param)：
 *   - config.cooldownTime: 冷却重置时间 (s)
 */
double determineNextCooldownTimer(const SppvtConfig& config, const SppvtInput& input, const SppvtState& state, bool isTriggered);

} // namespace control
} // namespace cc

#pragma once

#include "control/cc_control_def.h"

namespace cc {
namespace control {

/**
 * @brief 计算下一时刻的冷却计时器值
 * @param config 控制配置
 * @param input 输入信号
 * @param state 当前控制器状态
 * @param isTriggered 是否触发变目标
 * @return 下一时刻的冷却计时器值 (s)
 */
double determineNextCooldownTimer(const SppvtConfig& config, const SppvtInput& input, const SppvtState& state, bool isTriggered);

} // namespace control
} // namespace cc

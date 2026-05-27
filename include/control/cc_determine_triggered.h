#pragma once

#include "control/cc_control_def.h"

namespace cc {
namespace control {

/**
 * @brief 判断当前周期是否满足触发变目标条件（包括冷却期检查、平地稳定判定及车速限制等）
 * @param config 控制配置
 * @param input 输入信号
 * @param state 当前控制器状态
 * @return 是否触发变目标
 */
bool determineTriggered(const SppvtConfig& config, const SppvtInput& input, const SppvtState& state);

} // namespace control
} // namespace cc

#pragma once

#include "control/cc_control_def.h"

namespace cc {
namespace control {

/**
 * @brief 在变目标触发时，计算并限幅新的虚拟目标车速
 * @param config 控制配置
 * @param input 输入信号
 * @param state 当前控制器状态
 * @return 新的虚拟目标车速 (m/s)
 */
double calcNewVirtualTarget(const SppvtConfig& config, const SppvtInput& input, const SppvtState& state);

} // namespace control
} // namespace cc

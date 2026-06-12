#pragma once

#include "control/cc_control_def.h"

namespace cc {
namespace control {

/**
 * @brief 在变目标触发时，计算并限幅新的虚拟目标车速
 * 
 * 模块要素界定：
 * - 输入 (Input)：
 *   - input.targetSpeed: 最终巡航车速 V_t (m/s)
 *   - input.currentSpeed: 当前实际速度 (m/s)
 * - 输出 (Output)：
 *   - 返回值: 新的虚拟目标车速 (m/s)
 * - 状态 (State)：
 *   - state.virtualTargetSpeed: 当前虚拟目标车速 (m/s) (跨周期持久化)
 * - 参数 (Param)：
 *   - config.rho: 惩罚系数
 *   - config.maxOffset: 最大允许偏移量
 */
double calcNewVirtualTarget(const SppvtConfig& config, const SppvtInput& input, const SppvtState& state);

} // namespace control
} // namespace cc

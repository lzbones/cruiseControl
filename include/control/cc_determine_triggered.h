#pragma once

#include "control/cc_control_def.h"

namespace cc {
namespace control {

/**
 * @brief 判断当前周期是否满足触发变目标条件（包括冷却期检查、平地稳定判定及车速限制等）
 * 
 * 模块要素界定：
 * - 输入 (Input)：
 *   - input.isNewControlSession: 是否是新开启的控制会话
 *   - input.dt: 采样周期步长 (s)
 *   - input.currentAcceleration: 当前车辆加速度 (m/s^2)
 *   - input.currentSpeed: 当前实际速度 (m/s)
 *   - input.targetSpeed: 最终设定巡航速度 V_t (m/s)
 *   - input.pauseControl: 是否处于暂停控制状态
 * - 输出 (Output)：
 *   - 返回值: 是否触发变目标
 * - 状态 (State)：
 *   - state.cooldownTimer: 当前剩余冷却计时值 (s) (跨周期持久化)
 * - 参数 (Param)：
 *   - config.delta: 平稳判定加速度阈值
 */
bool determineTriggered(const SppvtConfig& config, const SppvtInput& input, const SppvtState& state);

} // namespace control
} // namespace cc

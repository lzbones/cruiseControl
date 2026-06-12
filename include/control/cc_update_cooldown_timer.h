#pragma once

namespace cc {
namespace control {

/**
 * @brief 更新冷却计时器的值（时间递减，限幅不小于 0）
 * 
 * 模块要素界定：
 * - 输入 (Input)：
 *   - dt: 周期时间步长 (s)
 * - 输出 (Output)：
 *   - 返回值: 更新后的冷却时间 (s)
 * - 状态 (State)：
 *   - currentTimer: 当前冷却计时器值 (跨周期持久化)
 * - 参数 (Param)：
 *   - 无
 */
double updateCooldownTimer(double currentTimer, double dt);

} // namespace control
} // namespace cc

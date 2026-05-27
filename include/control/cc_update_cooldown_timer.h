#pragma once

namespace cc {
namespace control {

/**
 * @brief 更新冷却计时器的值（时间递减，限幅不小于 0）
 * @param currentTimer 当前冷却计时器值
 * @param dt 周期时间步长 (s)
 * @return 更新后的冷却时间 (s)
 */
double updateCooldownTimer(double currentTimer, double dt);

} // namespace control
} // namespace cc

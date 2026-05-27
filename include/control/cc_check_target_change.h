#pragma once

namespace cc {
namespace control {

/**
 * @brief 判定是否达到变目标时刻 t_i
 * @param currentAcceleration 当前车辆加速度 v_dot (m/s^2)
 * @param delta 变目标加速度阈值常量 (m/s^2)
 * @param cooldownTimer 变目标冷却剩余时间 (s)
 * @return 是否满足变目标时刻条件 (满足且不在冷却期内返回 true)
 */
bool checkTargetChange(double currentAcceleration, double delta, double cooldownTimer);

} // namespace control
} // namespace cc


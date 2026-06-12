#pragma once

namespace cc {
namespace control {

/**
 * @brief 判定是否达到变目标时刻 t_i (|\dot{v}(t)| < \delta)
 * 
 * 模块要素界定：
 * - 输入 (Input)：
 *   - currentAcceleration: 当前车辆加速度 v_dot (m/s^2)
 *   - cooldownTimer: 跨周期持久保存的冷却剩余时间 (s)
 * - 输出 (Output)：
 *   - 返回值: 是否满足变目标时刻条件 (满足且不在冷却期内返回 true)
 * - 状态 (State)：
 *   - 无 (无持久化状态)
 * - 参数 (Param)：
 *   - delta: 变目标加速度阈值常量 (m/s^2)
 */
bool checkTargetChange(double currentAcceleration, double delta, double cooldownTimer);

} // namespace control
} // namespace cc


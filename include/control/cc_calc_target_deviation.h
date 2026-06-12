#pragma once

namespace cc {
namespace control {

/**
 * @brief 计算当前级与最终目标车速的偏差 ΔV_ti (式 5)
 * 
 * 模块要素界定：
 * - 输入 (Input)：
 *   - targetSpeed: 最终的巡航设定车速 V_t (m/s)
 *   - currentSpeed: 当前变目标时刻的实际车速 v_i(t_i) (m/s)
 * - 输出 (Output)：
 *   - 返回值: 目标车速偏差 (m/s)
 * - 状态 (State)：
 *   - 无 (无持久化状态)
 * - 参数 (Param)：
 *   - 无
 */
double calcTargetDeviation(double targetSpeed, double currentSpeed);

} // namespace control
} // namespace cc


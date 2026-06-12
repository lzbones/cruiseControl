#pragma once

namespace cc {
namespace control {

/**
 * @brief 计算第 i 级车速控制偏差 Δv_i(t)
 * 
 * 模块要素界定：
 * - 输入 (Input)：
 *   - virtualTarget: 第 i 级内部虚拟目标车速 V_i (m/s)
 *   - currentSpeed: 当前实际车速 v_i(t) (m/s)
 * - 输出 (Output)：
 *   - 返回值: 车速控制偏差 (m/s)
 * - 状态 (State)：
 *   - 无 (无持久化状态)
 * - 参数 (Param)：
 *   - 无
 */
double calcSpeedError(double virtualTarget, double currentSpeed);

} // namespace control
} // namespace cc


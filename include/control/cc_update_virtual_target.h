#pragma once

namespace cc {
namespace control {

/**
 * @brief 在变目标时刻更新内部虚拟目标车速 V_{i+1} (式 4)
 * 
 * 模块要素界定：
 * - 输入 (Input)：
 *   - targetDeviation: 当前级巡航车速偏差 ΔV_ti (m/s)
 * - 输出 (Output)：
 *   - 返回值: 更新后的虚拟目标车速 V_{i+1} (m/s)
 * - 状态 (State)：
 *   - virtualTarget: 当前级虚拟目标车速 V_i (m/s) (跨周期持久化)
 * - 参数 (Param)：
 *   - rho: 惩罚系数
 */
double updateVirtualTarget(double virtualTarget, double rho, double targetDeviation);

} // namespace control
} // namespace cc


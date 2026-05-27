#pragma once

namespace cc {
namespace control {

/**
 * @brief 在变目标时刻更新内部虚拟目标车速 V_{i+1} (式 4)
 * @param virtualTarget 当前级虚拟目标车速 V_i (m/s)
 * @param rho 惩罚系数
 * @param targetDeviation 当前偏差 ΔV_ti (m/s)
 * @return 更新后的虚拟目标车速 V_{i+1} (m/s)
 */
double updateVirtualTarget(double virtualTarget, double rho, double targetDeviation);

} // namespace control
} // namespace cc


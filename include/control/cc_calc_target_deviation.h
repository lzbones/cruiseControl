#pragma once

namespace cc {
namespace control {

/**
 * @brief 计算当前级与最终目标车速的偏差 ΔV_ti (式 5)
 * @param targetSpeed 最终的巡航设定车速 V_t (m/s)
 * @param currentSpeed 当前变目标时刻的实际车速 v_i(t_i) (m/s)
 * @return 目标车速偏差 (m/s)
 */
double calcTargetDeviation(double targetSpeed, double currentSpeed);

} // namespace control
} // namespace cc


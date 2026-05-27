#pragma once

namespace cc {
namespace control {

/**
 * @brief 计算第 i 级车速控制偏差 Δv_i(t)
 * @param virtualTarget 第 i 级内部虚拟目标车速 V_i (m/s)
 * @param currentSpeed 当前实际车速 v_i(t) (m/s)
 * @return 车速控制偏差 (m/s)
 */
double calcSpeedError(double virtualTarget, double currentSpeed);

} // namespace control
} // namespace cc


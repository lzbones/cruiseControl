#pragma once

#include "control/cc_control_def.h"

namespace cc {
namespace control {

/**
 * @brief 根据比例系数与误差计算控制扭矩并限幅
 * 
 * 模块要素界定：
 * - 输入 (Input)：
 *   - speedError: 车速控制偏差 Δv_i(t) (m/s)
 * - 输出 (Output)：
 *   - 返回值: 限幅后的扭矩命令 T_i(t) (Nm)
 * - 状态 (State)：
 *   - 无 (无持久化状态)
 * - 参数 (Param)：
 *   - Kp: 比例控制系数 (Nm * s / m)
 *   - config.minTorque: 最小扭矩限制 (一般为 0.0)
 *   - config.maxTorque: 最大扭矩限制
 */
double calcControlTorque(double Kp, double speedError, const SppvtConfig& config);

} // namespace control
} // namespace cc


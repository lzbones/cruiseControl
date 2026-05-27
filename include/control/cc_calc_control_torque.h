#pragma once

#include "control/cc_control_def.h"

namespace cc {
namespace control {

/**
 * @brief 根据比例系数与误差计算控制扭矩并限幅
 * @param Kp 比例控制系数 (Nm * s / m)
 * @param speedError 车速控制偏差 Δv_i(t) (m/s)
 * @param config 控制器配置参数 (限幅使用)
 * @return 限幅后的扭矩命令 T_i(t) (Nm)
 */
double calcControlTorque(double Kp, double speedError, const SppvtConfig& config);

} // namespace control
} // namespace cc


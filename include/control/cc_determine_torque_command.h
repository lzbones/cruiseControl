#pragma once

#include "control/cc_control_def.h"

namespace cc {
namespace control {

/**
 * @brief 根据当前虚拟目标车速与实际车速计算最终输出驱动扭矩
 * @param config 控制配置
 * @param input 输入信号
 * @param virtualTargetSpeed 当前正在起作用的虚拟目标车速 (m/s)
 * @return 控制输出扭矩 T_i(t) (Nm)
 */
double determineTorqueCommand(const SppvtConfig& config, const SppvtInput& input, double virtualTargetSpeed);

} // namespace control
} // namespace cc

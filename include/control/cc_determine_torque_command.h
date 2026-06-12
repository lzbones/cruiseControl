#pragma once

#include "control/cc_control_def.h"

namespace cc {
namespace control {

/**
 * @brief 根据当前虚拟目标车速与实际车速计算最终输出驱动扭矩
 * 
 * 模块要素界定：
 * - 输入 (Input)：
 *   - input.currentSpeed: 当前实际车速 (m/s)
 *   - input.pauseControl: 是否暂停控制
 *   - virtualTargetSpeed: 当前正在起作用的虚拟目标车速 (m/s)
 * - 输出 (Output)：
 *   - 返回值: 控制输出扭矩 T_i(t) (Nm)
 * - 状态 (State)：
 *   - 无 (无持久化状态)
 * - 参数 (Param)：
 *   - config.Kp: 比例系数
 *   - config.minTorque: 最小扭矩
 *   - config.maxTorque: 最大扭矩
 */
double determineTorqueCommand(const SppvtConfig& config, const SppvtInput& input, double virtualTargetSpeed);

} // namespace control
} // namespace cc

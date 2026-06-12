#pragma once

namespace cc {
namespace control {

/**
 * @brief 对内部虚拟目标车速进行上限限制保护，防止积分饱和 (式 6)
 * 
 * 模块要素界定：
 * - 输入 (Input)：
 *   - virtualTarget: 未限幅的虚拟目标车速 V_i (m/s)
 *   - targetSpeed: 最终设定目标车速 V_t (m/s)
 * - 输出 (Output)：
 *   - 返回值: 限幅保护后的虚拟目标车速 (m/s)
 * - 状态 (State)：
 *   - 无 (无持久化状态)
 * - 参数 (Param)：
 *   - maxOffset: 最大允许偏移量 V_max_offset (m/s)
 */
double limitVirtualTarget(double virtualTarget, double targetSpeed, double maxOffset);

} // namespace control
} // namespace cc


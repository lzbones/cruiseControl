#ifndef CC_LIMIT_VIRTUAL_TARGET_H_
#define CC_LIMIT_VIRTUAL_TARGET_H_

namespace cc {
namespace control {

/**
 * @brief 对内部虚拟目标车速进行上限限制保护，防止积分饱和 (式 6)
 * @param virtualTarget 未限幅的虚拟目标车速 V_i (m/s)
 * @param targetSpeed 最终设定目标车速 V_t (m/s)
 * @param maxOffset 最大允许偏移量 V_max_offset (m/s)
 * @return 限幅保护后的虚拟目标车速 (m/s)
 */
double limitVirtualTarget(double virtualTarget, double targetSpeed, double maxOffset);

} // namespace control
} // namespace cc

#endif // CC_LIMIT_VIRTUAL_TARGET_H_

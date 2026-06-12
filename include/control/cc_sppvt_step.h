#pragma once

#include "control/cc_control_def.h"

namespace cc {
namespace control {

/**
 * @brief 执行单步 SPPVT 控制算法流程
 * 
 * 模块要素界定：
 * - 输入 (Input)：
 *   - input.currentSpeed: 当前实际速度 (m/s)
 *   - input.targetSpeed: 最终设定巡航速度 V_t (m/s)
 *   - input.currentAcceleration: 当前实际加速度 (m/s^2)
 *   - input.dt: 采样周期步长 (s)
 *   - input.pauseControl: 是否暂停控制
 *   - input.isNewControlSession: 是否新开启控制会话
 * - 输出 (Output)：
 *   - 返回值: SppvtOutput (输出扭矩命令 T_i(t), 激活的虚拟目标车速)
 * - 状态 (State)：
 *   - state.virtualTargetSpeed: 内部虚拟目标车速 V_i (m/s) (跨周期持久化)
 *   - state.stepIndex: 阶式阶数 i (跨周期持久化)
 *   - state.cooldownTimer: 变目标冷却计时器 (s) (跨周期持久化)
 *   - state.controlActive: 控制器激活标志 (跨周期持久化)
 * - 参数 (Param)：
 *   - config.Kp: 比例控制系数 (Nm * s / m)
 *   - config.rho: 惩罚系数
 *   - config.delta: 变目标加速度阈值 (m/s^2)
 *   - config.maxOffset: 虚拟目标允许的最大偏置量 (m/s)
 *   - config.cooldownTime: 变目标冷却锁存时长 (s)
 *   - config.maxTorque: 最大驱动限幅 (Nm)
 *   - config.minTorque: 最小驱动限幅 (一般为 0.0)
 */
SppvtOutput sppvtStep(const SppvtConfig& config, const SppvtInput& input, SppvtState& state);

} // namespace control
} // namespace cc


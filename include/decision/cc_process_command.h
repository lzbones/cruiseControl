#pragma once

#include "decision/cc_decision_def.h"

namespace cc {
namespace decision {

/**
 * @brief 根据当前输入信号、系统状态和配置参数计算决策行为，并更新状态机
 * 
 * 模块要素界定：
 * - 输入 (Input)：
 *   - input.currentSpeed: 当前实际车速 (m/s)
 *   - input.isBrakePressed: 刹车踏板踩下
 *   - input.isThrottlePressed: 开启人工油门
 *   - input.hasHistoryTarget: 是否有历史车速
 *   - input.controlActive: 当前控制环是否激活
 *   - input.driverCommand: 按键指令
 *   - input.dt: 周期步长
 * - 输出 (Output)：
 *   - 返回值: DecisionOutput (决策动作 Rk, 预测的下一时刻 Sn, 新的目标车速)
 * - 状态 (State)：
 *   - s_state: 模块私有文件作用域 static 变量，跨周期保存当前状态、目标车速、历史目标和低速超时计时
 * - 参数 (Param)：
 *   - config.minSpeed: 最低车速区间值
 *   - config.maxSpeed: 最高车速区间值
 *   - config.lowSpeedTimeout: 低速超时阈值
 *   - config.deltaSpeedStep: 修改步长
 */
DecisionOutput processCommand(const DecisionInput& input, const DecisionConfig& config);
const DecisionState& getDecisionState();
void setDecisionState(const DecisionState& state);

} // namespace decision
} // namespace cc



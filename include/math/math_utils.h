#pragma once

namespace cc {
namespace math {

/**
 * @brief 计算绝对值
 * 
 * 模块要素界定：
 * - 输入 (Input)：
 *   - val: 输入的实数值
 * - 输出 (Output)：
 *   - 返回值: 绝对值后的非负实数
 * - 状态 (State)：
 *   - 无
 * - 参数 (Param)：
 *   - 无
 */
double absVal(double val);

/**
 * @brief 计算数值导数（加速度）
 * 
 * 模块要素界定：
 * - 输入 (Input)：
 *   - currentSpeed: 当前车速 (m/s)
 *   - lastSpeed: 上一周期车速 (m/s)
 *   - dt: 时间步长 (s)
 * - 输出 (Output)：
 *   - 返回值: 计算得到的加速度 (m/s^2)
 * - 状态 (State)：
 *   - 无
 * - 参数 (Param)：
 *   - 无
 */
double calcAcceleration(double currentSpeed, double lastSpeed, double dt);

/**
 * @brief 数值限幅
 * 
 * 模块要素界定：
 * - 输入 (Input)：
 *   - val: 待限幅输入值
 * - 输出 (Output)：
 *   - 返回值: 限幅截断后的值
 * - 状态 (State)：
 *   - 无
 * - 参数 (Param)：
 *   - minVal: 最小值限制下限
 *   - maxVal: 最大值限制上限
 */
double clampVal(double val, double minVal, double maxVal);

} // namespace math
} // namespace cc

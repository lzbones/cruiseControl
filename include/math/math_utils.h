#ifndef CC_MATH_UTILS_H_
#define CC_MATH_UTILS_H_

namespace cc {
namespace math {

/**
 * @brief 计算绝对值
 * @param val 输入值
 * @return 绝对值
 */
double absVal(double val);

/**
 * @brief 计算数值导数（加速度）
 * @param currentSpeed 当前车速 (m/s)
 * @param lastSpeed 上一周期车速 (m/s)
 * @param dt 时间步长 (s)
 * @return 计算得到的加速度 (m/s^2)
 */
double calcAcceleration(double currentSpeed, double lastSpeed, double dt);

/**
 * @brief 数值限幅
 * @param val 输入值
 * @param minVal 最小值限制
 * @param maxVal 最大值限制
 * @return 限幅后的值
 */
double clampVal(double val, double minVal, double maxVal);

} // namespace math
} // namespace cc

#endif // CC_MATH_UTILS_H_

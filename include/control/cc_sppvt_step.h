#ifndef CC_SPPVT_STEP_H_
#define CC_SPPVT_STEP_H_

#include "control/cc_control_def.h"

namespace cc {
namespace control {

/**
 * @brief 执行单步 SPPVT 控制算法流程
 * @param config 算法配置参数
 * @param input 本周期输入信号
 * @param state 持久化运行状态变量 (调用后会被就地更新)
 * @return SppvtOutput 本周期的控制命令输出
 */
SppvtOutput sppvtStep(const SppvtConfig& config, const SppvtInput& input, SppvtState& state);

} // namespace control
} // namespace cc

#endif // CC_SPPVT_STEP_H_

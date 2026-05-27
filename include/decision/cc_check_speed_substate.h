#ifndef CC_CHECK_SPEED_SUBSTATE_H_
#define CC_CHECK_SPEED_SUBSTATE_H_

#include "decision/cc_decision_def.h"

namespace cc {
namespace decision {

/**
 * @brief 根据当前车速和阈值判定车速子态 Sv
 * @param currentSpeed 当前实际车速 (m/s)
 * @param config 决策配置参数 (内含 minSpeed/maxSpeed)
 * @return SpeedSubstate 车速子态 (SPEED_SUBSTATE_SUITABLE, SPEED_SUBSTATE_LOW, SPEED_SUBSTATE_HIGH)
 */
SpeedSubstate checkSpeedSubstate(double currentSpeed, const DecisionConfig& config);

} // namespace decision
} // namespace cc

#endif // CC_CHECK_SPEED_SUBSTATE_H_

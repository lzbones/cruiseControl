#include "control/cc_check_target_change.h"
#include "math/math_utils.h"

namespace cc {
namespace control {

bool checkTargetChange(double currentAcceleration, double delta, double cooldownTimer) {
    // 1. 判断冷却是否完成
    const bool cooldownActive = (cooldownTimer > 0.0);
    
    // 2. 双边绝对值限幅条件判断: |v_dot| < delta
    const bool isFlat = (cc::math::absVal(currentAcceleration) < delta);
    
    const bool result = cooldownActive ? false : isFlat;
    return result;
}

} // namespace control
} // namespace cc

#include "control/cc_check_target_change.h"
#include "math/math_utils.h"

namespace cc {
namespace control {

bool checkTargetChange(double currentAcceleration, double delta, double cooldownTimer) {
    // 1. 判断冷却是否完成
    if (cooldownTimer > 0.0) {
        return false;
    }
    
    // 2. 双边绝对值限幅条件判断: |v_dot| < delta
    return cc::math::absVal(currentAcceleration) < delta;
}

} // namespace control
} // namespace cc

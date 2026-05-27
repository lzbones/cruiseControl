#include "control/cc_calc_control_torque.h"
#include "math/math_utils.h"

namespace cc {
namespace control {

double calcControlTorque(double Kp, double speedError, const SppvtConfig& config) {
    const double rawTorque = Kp * speedError;
    const double torque = cc::math::clampVal(rawTorque, config.minTorque, config.maxTorque);
    return torque;
}

} // namespace control
} // namespace cc

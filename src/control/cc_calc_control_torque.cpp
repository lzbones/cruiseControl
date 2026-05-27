#include "control/cc_calc_control_torque.h"
#include "math/math_utils.h"

namespace cc {
namespace control {

double calcControlTorque(double Kp, double speedError, const SppvtConfig& config) {
    double rawTorque = Kp * speedError;
    return cc::math::clampVal(rawTorque, config.minTorque, config.maxTorque);
}

} // namespace control
} // namespace cc

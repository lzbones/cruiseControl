#include "control/cc_determine_torque_command.h"
#include "control/cc_calc_speed_error.h"
#include "control/cc_calc_control_torque.h"

namespace cc {
namespace control {

double determineTorqueCommand(const SppvtConfig& config, const SppvtInput& input, double virtualTargetSpeed) {
    const double speedError = calcSpeedError(virtualTargetSpeed, input.currentSpeed);
    const double torque = calcControlTorque(config.Kp, speedError, config);
    const double finalTorque = input.pauseControl ? 0.0 : torque;
    return finalTorque;
}

} // namespace control
} // namespace cc

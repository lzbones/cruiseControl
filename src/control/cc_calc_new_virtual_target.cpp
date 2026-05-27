#include "control/cc_calc_new_virtual_target.h"
#include "control/cc_calc_target_deviation.h"
#include "control/cc_update_virtual_target.h"
#include "control/cc_limit_virtual_target.h"

namespace cc {
namespace control {

double calcNewVirtualTarget(const SppvtConfig& config, const SppvtInput& input, const SppvtState& state) {
    const double deviation = calcTargetDeviation(input.targetSpeed, input.currentSpeed);
    const double updatedTarget = updateVirtualTarget(state.virtualTargetSpeed, config.rho, deviation);
    const double limitedTarget = limitVirtualTarget(updatedTarget, input.targetSpeed, config.maxOffset);
    return limitedTarget;
}

} // namespace control
} // namespace cc

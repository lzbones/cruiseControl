#include "control/cc_update_virtual_target.h"

namespace cc {
namespace control {

double updateVirtualTarget(double virtualTarget, double rho, double targetDeviation) {
    return virtualTarget + rho * targetDeviation;
}

} // namespace control
} // namespace cc

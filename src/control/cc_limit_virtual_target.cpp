#include "control/cc_limit_virtual_target.h"

namespace cc {
namespace control {

double limitVirtualTarget(double virtualTarget, double targetSpeed, double maxOffset) {
    double limitUpper = targetSpeed + maxOffset;
    if (virtualTarget > limitUpper) {
        return limitUpper;
    }
    return virtualTarget;
}

} // namespace control
} // namespace cc

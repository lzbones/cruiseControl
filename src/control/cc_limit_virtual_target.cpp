#include "control/cc_limit_virtual_target.h"

namespace cc {
namespace control {

double limitVirtualTarget(double virtualTarget, double targetSpeed, double maxOffset) {
    const double limitUpper = targetSpeed + maxOffset;
    const double result = (virtualTarget > limitUpper) ? limitUpper : virtualTarget;
    return result;
}

} // namespace control
} // namespace cc

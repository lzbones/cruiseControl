#include "control/cc_calc_speed_error.h"

namespace cc {
namespace control {

double calcSpeedError(double virtualTarget, double currentSpeed) {
    return virtualTarget - currentSpeed;
}

} // namespace control
} // namespace cc

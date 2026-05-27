#include "control/cc_calc_target_deviation.h"

namespace cc {
namespace control {

double calcTargetDeviation(double targetSpeed, double currentSpeed) {
    return targetSpeed - currentSpeed;
}

} // namespace control
} // namespace cc

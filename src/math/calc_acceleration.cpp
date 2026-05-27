#include "math/math_utils.h"

namespace cc {
namespace math {

double calcAcceleration(double currentSpeed, double lastSpeed, double dt) {
    const double result = (dt <= 1e-6) ? 0.0 : ((currentSpeed - lastSpeed) / dt);
    return result;
}

} // namespace math
} // namespace cc

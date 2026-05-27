#include "math/math_utils.h"

namespace cc {
namespace math {

double calcAcceleration(double currentSpeed, double lastSpeed, double dt) {
    if (dt <= 1e-6) {
        return 0.0;
    }
    return (currentSpeed - lastSpeed) / dt;
}

} // namespace math
} // namespace cc

#include "math/math_utils.h"

namespace cc {
namespace math {

double clampVal(double val, double minVal, double maxVal) {
    if (val < minVal) {
        return minVal;
    }
    if (val > maxVal) {
        return maxVal;
    }
    return val;
}

} // namespace math
} // namespace cc

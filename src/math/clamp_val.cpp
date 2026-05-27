#include "math/math_utils.h"

namespace cc {
namespace math {

double clampVal(double val, double minVal, double maxVal) {
    const double temp = (val < minVal) ? minVal : val;
    const double result = (temp > maxVal) ? maxVal : temp;
    return result;
}

} // namespace math
} // namespace cc

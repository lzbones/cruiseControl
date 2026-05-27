#include "math/math_utils.h"

namespace cc {
namespace math {

double absVal(double val) {
    return (val >= 0.0) ? val : -val;
}

} // namespace math
} // namespace cc

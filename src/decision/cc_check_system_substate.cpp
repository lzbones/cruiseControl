#include "decision/cc_check_system_substate.h"

namespace cc {
namespace decision {

SystemSubstate checkSystemSubstate(bool controlActive, bool isThrottlePressed, bool hasHistoryTarget) {
    const int b2 = controlActive ? 1 : 0;
    const int b1 = isThrottlePressed ? 1 : 0;
    const int b0 = hasHistoryTarget ? 1 : 0;
    
    const int j = (b2 << 2) | (b1 << 1) | b0;
    const SystemSubstate result = static_cast<SystemSubstate>(j);
    return result;
}

} // namespace decision
} // namespace cc

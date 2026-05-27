#include "decision/cc_check_system_substate.h"

namespace cc {
namespace decision {

SystemSubstate checkSystemSubstate(bool controlActive, bool isThrottlePressed, bool hasHistoryTarget) {
    int b2 = controlActive ? 1 : 0;
    int b1 = isThrottlePressed ? 1 : 0;
    int b0 = hasHistoryTarget ? 1 : 0;
    
    int j = (b2 << 2) | (b1 << 1) | b0;
    return static_cast<SystemSubstate>(j);
}

} // namespace decision
} // namespace cc

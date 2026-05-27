#include "control/cc_update_cooldown_timer.h"

namespace cc {
namespace control {

double updateCooldownTimer(double currentTimer, double dt) {
    const double decTimer = currentTimer - dt;
    const double result = (currentTimer > 0.0) ? ((decTimer < 0.0) ? 0.0 : decTimer) : currentTimer;
    return result;
}

} // namespace control
} // namespace cc

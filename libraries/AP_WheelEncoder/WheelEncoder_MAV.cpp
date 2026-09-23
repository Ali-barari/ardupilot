#include "WheelEncoder_MAV.h"

#include <AP_HAL/AP_HAL.h>

// consume a WHEEL_DISTANCE message, taking this instance's wheel from the
// matching index of the distance[] array
void AP_WheelEncoder_MAV::handle_msg(const mavlink_wheel_distance_t &packet)
{
    const uint8_t instance = _state.instance;

    // the sender reports how many wheels it is carrying; anything beyond
    // that is not populated and must not be read
    if (instance >= packet.count ||
        instance >= MAVLINK_MSG_WHEEL_DISTANCE_FIELD_DISTANCE_LEN) {
        return;
    }

    const float radius = _frontend.get_wheel_radius(instance);
    const uint16_t cpr = _frontend.get_counts_per_revolution(instance);
    if (!is_positive(radius) || cpr == 0) {
        // unconfigured; counting up errors makes the misconfiguration
        // visible through the usual signal-quality reporting
        copy_state_to_frontend(_state.distance_count,
                               _state.total_count,
                               _state.error_count + 1,
                               AP_HAL::millis());
        return;
    }

    // metres -> counts. This mirrors the frontend's counts -> metres
    // conversion exactly, so get_distance() returns what the sender sent.
    const float counts_per_metre = (float)cpr / (M_2PI * radius);
    const int32_t distance_count = (int32_t)roundf(packet.distance[instance] * counts_per_metre);

    // timestamp with our own clock, not the sender's: packet.time_usec is
    // the sending board's own counter and is not synchronised to the flight
    // controller, whereas the frontend's rate calculation and the EKF both
    // need this in autopilot time
    copy_state_to_frontend(distance_count,
                           _state.total_count + 1,
                           _state.error_count,
                           AP_HAL::millis());
}

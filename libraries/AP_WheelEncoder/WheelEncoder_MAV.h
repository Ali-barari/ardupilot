/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include "AP_WheelEncoder.h"
#include "WheelEncoder_Backend.h"

// Wheel encoder backend fed by the standard MAVLink WHEEL_DISTANCE message
// (id 9000) rather than by locally decoded quadrature pulses.
//
// Intended for vehicles whose encoders are read by an external board - e.g.
// one with more wheels than the flight controller has interrupt-safe GPIO
// pairs - which streams cumulative per-wheel distance over a serial link.
// Each configured instance takes its distance from the matching index of the
// message's distance[] array, so instance 0 reads distance[0], and so on; the
// mapping of array index to physical wheel is by agreement between the sender
// and the vehicle, as the message definition specifies.
//
// WHEEL_DISTANCE reports metres, whereas this library stores counts, so the
// incoming distance is converted using this instance's WENCx_CPR and
// WENCx_RADIUS. Those two values cancel again when the frontend converts back
// (get_distance() returns metres, and the EKF is handed an angle which it
// multiplies by the same radius), so they do not affect fused velocity - but
// WENCx_RADIUS must still be the real wheel radius for get_rate() to return a
// physically meaningful rad/s, and CPR sets the quantisation of the stored
// count.

class AP_WheelEncoder_MAV : public AP_WheelEncoder_Backend {
public:

    // constructor
    using AP_WheelEncoder_Backend::AP_WheelEncoder_Backend;

    // called at the vehicle's wheel encoder update rate; the data itself
    // arrives asynchronously via handle_msg(), so there is nothing to poll
    void update(void) override {}

    // a gap in the stream means we have no reading, not that the wheel
    // stopped, so consumers must not treat stale data as zero motion
    bool no_data_means_stopped(void) const override { return false; }

    // consume a decoded WHEEL_DISTANCE message
    void handle_msg(const mavlink_wheel_distance_t &packet);
};

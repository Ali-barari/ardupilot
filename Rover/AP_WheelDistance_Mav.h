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

// Rover-only sink for the standard MAVLink WHEEL_DISTANCE message (id 9000).
//
// Arcteron's 4-wheel rover reads its quadrature encoders on an external
// STM32 board (not on the flight controller's own GPIOs - see
// docs/ARCTERON_WHEEL_ENCODERS_TRACTION_CONTROL.md and
// docs/ARCTERON_WHEEL_DISTANCE_MAVLINK_SPEC.md in the seair repo for the
// full rationale and interface contract) and streams cumulative per-wheel
// distance to the FC as WHEEL_DISTANCE over a spare UART.
//
// This is deliberately a standalone store rather than a 4th/5th
// AP_WheelEncoder backend: AP_WheelEncoder is hard-capped at
// WHEELENCODER_MAX_INSTANCES (2), and AP_WheelRateControl on top of it is
// separately hardcoded to 2 instances (one per side). Neither can express
// 4 independent wheels without its own modification, so extending
// AP_WheelEncoder would buy nothing for this ingest step - seair's
// traction-control work (per-wheel slip detection) is expected to consume
// this store directly rather than go through AP_WheelEncoder/AP_WheelRateControl.

#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

#define AP_WHEELDISTANCE_MAV_NUM_WHEELS   4
#define AP_WHEELDISTANCE_MAV_TIMEOUT_MS   500   // consider the link down if no packet in this long

class AP_WheelDistance_Mav {
public:
    AP_WheelDistance_Mav();

    CLASS_NO_COPY(AP_WheelDistance_Mav);

    static const struct AP_Param::GroupInfo var_info[];

    // consume an incoming WHEEL_DISTANCE mavlink message
    void handle_msg(const mavlink_message_t &msg);

    // true if the feature is enabled via WHLDIST_ENABLE
    bool enabled() const { return _enable; }

    // true if we've heard from the encoder board recently
    bool healthy() const;

    // number of wheels most recently reported (0 until the first packet arrives)
    uint8_t num_wheels() const { return _num_wheels; }

    // wheel radius shared by all 4 wheels (m), used to convert reported
    // linear distance into angular distance for EKF fusion
    float get_wheel_radius() const { return _wheel_radius; }

    // body-frame position offset of the given wheel's hub (m)
    const Vector3f &get_pos_offset(uint8_t instance) const;

    // latest cumulative distance for the given wheel (m), and the system
    // time (ms) the packet carrying it was received
    float get_distance(uint8_t instance) const;
    uint32_t get_last_reading_ms() const { return _last_update_ms; }

    // raw board-side timestamp (us since board boot, NOT wall-clock -
    // see the MAVLink spec doc) of the most recent packet. Only valid for
    // computing deltas between consecutive packets, never as an absolute time.
    uint64_t get_last_time_usec() const { return _last_time_usec; }

private:
    // parameters
    AP_Int8 _enable;
    AP_Float _wheel_radius;
    AP_Vector3f _pos_offset[AP_WHEELDISTANCE_MAV_NUM_WHEELS];

    // latest state
    uint8_t _num_wheels;
    float _distance_m[AP_WHEELDISTANCE_MAV_NUM_WHEELS];
    uint64_t _last_time_usec;
    uint32_t _last_update_ms;

    Vector3f _pos_offset_zero;   // returned for out-of-range instance requests
};

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

/*
  Wheel State Estimator for ground vehicles.

  Turns the raw wheel encoder readings into per-wheel speed and
  distance, the mean speed and distance of all wheels, and whether the
  wheels are at rest.  It only measures; controllers such as
  AR_HillHoldControl read from it.
 */
#pragma once

#include "AR_WheelState_config.h"

#if AR_WHEELSTATE_ENABLED

#include <AP_Param/AP_Param.h>
#include <AP_WheelEncoder/AP_WheelEncoder.h>
#include <Filter/LowPassFilter.h>

class AR_WheelState {
public:

    AR_WheelState(const AP_WheelEncoder &wheel_encoder);

    CLASS_NO_COPY(AR_WheelState);

    // update from the wheel encoders.  Call after AP_WheelEncoder::update()
    void update();

    // number of wheels with an enabled encoder
    uint8_t num_wheels() const { return _num_wheels; }

    // true if at least one wheel is enabled and every enabled wheel
    // encoder is healthy and has reported recently
    bool healthy() const { return _healthy; }

    // filtered wheel speed in m/s (+ve forward) for one wheel
    float get_wheel_speed(uint8_t instance) const;

    // distance travelled in metres (+ve forward) for one wheel
    float get_wheel_distance(uint8_t instance) const;

    // mean filtered speed of all wheels in m/s (+ve forward)
    float get_mean_speed() const { return _mean_speed; }

    // mean distance travelled by all wheels in metres (+ve forward)
    float get_mean_distance() const { return _mean_distance; }

    // true if the wheels have been at rest for WST_STILL_TIME
    bool is_stationary() const { return _stationary; }

    // distance in metres the wheels have moved since they last came to
    // rest.  Zero while stationary
    float get_distance_since_stationary() const;

    static const struct AP_Param::GroupInfo var_info[];

private:

    // update standstill detection (AR_WheelState_Roll.cpp)
    void update_standstill(uint32_t now_ms);

    // parameters
    AP_Float _speed_filt_hz;    // wheel speed filter cutoff frequency
    AP_Float _still_speed;      // speed below which the wheels are considered at rest
    AP_Float _still_time;       // time the wheels must stay below _still_speed to be stationary

    const AP_WheelEncoder &_wheel_encoder;

    // per-wheel state
    LowPassFilterFloat _speed_filter[WHEELENCODER_MAX_INSTANCES];
    float _wheel_speed[WHEELENCODER_MAX_INSTANCES];
    float _wheel_distance[WHEELENCODER_MAX_INSTANCES];

    // combined state
    uint8_t _num_wheels;
    bool _healthy;
    float _mean_speed;
    float _mean_distance;
    uint32_t _last_update_us;

    // standstill state
    bool _stationary;
    uint32_t _still_start_ms;       // time mean speed first fell below _still_speed, 0 if moving
    float _stationary_distance;     // mean distance when the wheels last came to rest
};

#endif  // AR_WHEELSTATE_ENABLED

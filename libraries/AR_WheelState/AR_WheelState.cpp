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

#include "AR_WheelState.h"

#if AR_WHEELSTATE_ENABLED

#include <AP_HAL/AP_HAL.h>

// an encoder that has not reported for this long is unhealthy
#define AR_WHEELSTATE_TIMEOUT_MS 500

const AP_Param::GroupInfo AR_WheelState::var_info[] = {

    // @Param: SPD_FILT
    // @DisplayName: Wheel speed filter
    // @Description: Cutoff frequency of the low-pass filter applied to each wheel's speed
    // @Units: Hz
    // @Range: 1 25
    // @Increment: 0.5
    // @User: Advanced
    AP_GROUPINFO("SPD_FILT", 1, AR_WheelState, _speed_filt_hz, 5.0f),

    // @Param: STILL_SPD
    // @DisplayName: Wheel standstill speed
    // @Description: The wheels are considered at rest when their mean speed stays below this value for WST_STILL_TIME
    // @Units: m/s
    // @Range: 0.005 0.2
    // @Increment: 0.005
    // @User: Advanced
    AP_GROUPINFO("STILL_SPD", 2, AR_WheelState, _still_speed, 0.02f),

    // @Param: STILL_TIME
    // @DisplayName: Wheel standstill time
    // @Description: Time the mean wheel speed must stay below WST_STILL_SPD before the wheels are considered at rest
    // @Units: s
    // @Range: 0 2
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("STILL_TIME", 3, AR_WheelState, _still_time, 0.3f),

    AP_GROUPEND
};

AR_WheelState::AR_WheelState(const AP_WheelEncoder &wheel_encoder) :
    _wheel_encoder(wheel_encoder)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// update from the wheel encoders.  Call after AP_WheelEncoder::update()
void AR_WheelState::update()
{
    const uint32_t now_us = AP_HAL::micros();
    const uint32_t now_ms = AP_HAL::millis();
    const float dt = (_last_update_us == 0) ? 0.0f : (now_us - _last_update_us) * 1.0e-6f;
    _last_update_us = now_us;

    uint8_t num_wheels = 0;
    bool all_healthy = true;
    float speed_sum = 0.0f;
    float distance_sum = 0.0f;
    uint8_t side_count[2] {};
    float side_speed_sum[2] {};
    float side_distance_sum[2] {};

    for (uint8_t i = 0; i < WHEELENCODER_MAX_INSTANCES; i++) {
        _wheel_side[i] = -1;
    }
    for (uint8_t i = 0; i < _wheel_encoder.num_sensors(); i++) {
        if (!_wheel_encoder.enabled(i)) {
            continue;
        }
        num_wheels++;

        const bool fresh = (now_ms - _wheel_encoder.get_last_reading_ms(i)) < AR_WHEELSTATE_TIMEOUT_MS;
        if (!_wheel_encoder.healthy(i) || !fresh) {
            all_healthy = false;
        }

        // wheel rate in rad/s times radius gives speed in m/s
        const float raw_speed = _wheel_encoder.get_rate(i) * _wheel_encoder.get_wheel_radius(i);
        _speed_filter[i].set_cutoff_frequency(_speed_filt_hz);
        if (is_positive(dt)) {
            _wheel_speed[i] = _speed_filter[i].apply(raw_speed, dt);
        } else {
            _speed_filter[i].reset(raw_speed);
            _wheel_speed[i] = raw_speed;
        }
        _wheel_distance[i] = _wheel_encoder.get_distance(i);

        speed_sum += _wheel_speed[i];
        distance_sum += _wheel_distance[i];

        // +ve Y offset is the right side, -ve the left; centred wheels count for neither
        const float pos_y = _wheel_encoder.get_pos_offset(i).y;
        if (!is_zero(pos_y)) {
            const uint8_t side = is_positive(pos_y) ? uint8_t(Side::RIGHT) : uint8_t(Side::LEFT);
            _wheel_side[i] = side;
            side_count[side]++;
            side_speed_sum[side] += _wheel_speed[i];
            side_distance_sum[side] += _wheel_distance[i];
        }
    }

    _num_wheels = num_wheels;
    _healthy = (num_wheels > 0) && all_healthy;
    if (num_wheels > 0) {
        _mean_speed = speed_sum / num_wheels;
        _mean_distance = distance_sum / num_wheels;
    } else {
        _mean_speed = 0.0f;
        _mean_distance = 0.0f;
    }

    for (uint8_t s = 0; s < 2; s++) {
        _side_count[s] = side_count[s];
        _side_speed[s] = (side_count[s] > 0) ? side_speed_sum[s] / side_count[s] : _mean_speed;
        _side_distance[s] = (side_count[s] > 0) ? side_distance_sum[s] / side_count[s] : _mean_distance;
    }

    update_standstill(now_ms);
}

// mean distance (m) of the wheels on one side
float AR_WheelState::get_side_distance(Side side) const
{
    return _side_distance[uint8_t(side)];
}

// mean filtered speed (m/s) of the wheels on one side
float AR_WheelState::get_side_speed(Side side) const
{
    return _side_speed[uint8_t(side)];
}

// side of one wheel (0 = left, 1 = right), or -1 if centred or not enabled
int8_t AR_WheelState::get_wheel_side(uint8_t instance) const
{
    if (instance >= WHEELENCODER_MAX_INSTANCES) {
        return -1;
    }
    return _wheel_side[instance];
}

// filtered wheel speed in m/s (+ve forward) for one wheel
float AR_WheelState::get_wheel_speed(uint8_t instance) const
{
    if (instance >= WHEELENCODER_MAX_INSTANCES) {
        return 0.0f;
    }
    return _wheel_speed[instance];
}

// distance travelled in metres (+ve forward) for one wheel
float AR_WheelState::get_wheel_distance(uint8_t instance) const
{
    if (instance >= WHEELENCODER_MAX_INSTANCES) {
        return 0.0f;
    }
    return _wheel_distance[instance];
}

#endif  // AR_WHEELSTATE_ENABLED

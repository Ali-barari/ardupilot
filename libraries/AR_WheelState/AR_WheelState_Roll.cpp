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
  Standstill and roll detection for the Wheel State Estimator.
 */

#include "AR_WheelState.h"

#if AR_WHEELSTATE_ENABLED

// update standstill detection
void AR_WheelState::update_standstill(uint32_t now_ms)
{
    if (!_healthy || fabsf(_mean_speed) >= _still_speed) {
        _still_start_ms = 0;
        _stationary = false;
        return;
    }

    // speed is below the threshold, start or continue the timer
    if (_still_start_ms == 0) {
        _still_start_ms = now_ms;
    }
    if (!_stationary && (now_ms - _still_start_ms) >= (uint32_t)(MAX(_still_time, 0.0f) * 1000.0f)) {
        _stationary = true;
    }

    // keep the reference at the current position while at rest so that
    // slow creep below the threshold is not counted later
    if (_stationary) {
        _stationary_distance = _mean_distance;
    }
}

// distance in metres the wheels have moved since they last came to
// rest.  Zero while stationary
float AR_WheelState::get_distance_since_stationary() const
{
    if (_stationary || !_healthy) {
        return 0.0f;
    }
    return _mean_distance - _stationary_distance;
}

#endif  // AR_WHEELSTATE_ENABLED

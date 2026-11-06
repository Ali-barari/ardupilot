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
  Hill Hold Control for ground vehicles.

  When the vehicle's mode asks for zero speed and the wheels have nearly
  stopped, the wheel positions are captured as a hold point and the
  vehicle is held there, so back-drivable drive trains do not roll down
  a slope.  Skid-steer vehicles are held per side, which also holds the
  heading; other vehicles are held on the mean of all wheels.

  For each side a controller turns the distance from the hold point
  (P and I) and the wheel speed (damping) into a corrective speed, which
  is scaled to throttle with CRUISE_THROTTLE / CRUISE_SPEED.  Optional
  pitch feedforward, dead-zone compensation (HHC_MIN) and direction
  hysteresis (HHC_HYST, to stop direction relays chattering) are added.

  When a speed is requested again the holding throttle is faded out over
  HHC_REL_TIME so the vehicle does not drop back before the speed
  controller takes over.  Uses the wheel encoders only; it does not need
  GPS or an EKF position.
 */
#pragma once

#include "AR_HillHoldControl_config.h"

#if AR_HILLHOLDCONTROL_ENABLED

#include <AP_Param/AP_Param.h>
#include <AR_WheelState/AR_WheelState.h>

class AR_HillHoldControl {
public:

    AR_HillHoldControl(const AR_WheelState &wheel_state);

    CLASS_NO_COPY(AR_HillHoldControl);

    enum class State : uint8_t {
        OFF       = 0,  // disabled, disarmed or wheel encoders unhealthy
        IDLE      = 1,  // ready, vehicle moving or speed requested
        HOLDING   = 2,  // holding the vehicle at the hold point
        RELEASING = 3,  // speed requested, holding throttle fading out
        TIMEOUT   = 4,  // held for HHC_TMAX, output stopped until a speed is requested
    };

    // true if enabled by the user
    bool enabled() const { return _enable > 0; }

    // call each loop while the mode asks for zero speed.  throttle_out
    // holds the stop controller's output (-100 to +100) and is replaced
    // with the holding throttle while holding.  cruise_speed (m/s) and
    // cruise_throttle (0 to 1) scale the corrective speed to throttle.
    // Returns true while holding
    bool update_stopped(float cruise_speed, float cruise_throttle, float dt, float &throttle_out);

    // call after update_stopped with the mode's desired turn rate (rad/s)
    // and steering output (-1 to +1).  While holding with no turn
    // requested the steering is replaced with the output that holds the
    // heading.  A turn request suspends the heading hold
    void update_steering(float desired_turn_rate, float &steering_out);

    // call each loop while the mode asks for a non-zero speed.
    // throttle_out holds the speed controller's output (-100 to +100);
    // the fading holding throttle is added to it while releasing
    void update_moving(float target_speed, float dt, float &throttle_out);

    // current state
    State get_state() const { return _state; }

    static const struct AP_Param::GroupInfo var_info[];

private:

    // the controller for one side of the vehicle, or for the whole
    // vehicle if it has no sides
    struct SideHold {
        float integrator;   // corrective speed from the I term (m/s)
        int8_t direction;   // direction of the last non-zero output (+1, -1), 0 if none
        float output;       // last output (-1 to +1)
    };

    // true if enabled, armed and the wheel encoders are healthy
    bool can_run() const;

    // change state
    void set_state(State new_state);

    // start holding at the current wheel position
    void start_hold(uint32_t now_ms, float throttle_per_speed, float ff);

    // run one side's controller and return its throttle (-1 to +1)
    float update_side(SideHold &side, float error, float speed, float throttle_per_speed, float ff, float dt);

    // write the HHC log message
    void write_log(float error_fwd, float error_yaw, float ff);

    // parameters
    AP_Int8 _enable;
    AP_Float _slope_ff;         // throttle % per unit sin(pitch)
    AP_Float _thr_max;          // maximum holding throttle %
    AP_Float _deadband;         // distance error deadband in m
    AP_Float _engage_speed;     // hold engages below this mean wheel speed in m/s
    AP_Float _release_time;     // holding throttle fade-out time in s
    AP_Float _timeout;          // give up after holding this long in s, 0 = never
    AP_Float _kp;               // corrective speed per m from the hold point
    AP_Float _ki;               // corrective speed per m.s from the hold point
    AP_Float _imax;             // I term limit in m/s
    AP_Float _damp;             // corrective speed per m/s of wheel speed
    AP_Float _thr_min;          // dead-zone compensation, throttle %
    AP_Float _dir_hyst;         // direction change hysteresis, throttle %

    const AR_WheelState &_wheel_state;

    // state
    State _state = State::OFF;
    uint32_t _last_update_ms;   // last time update_stopped or update_moving ran
    uint32_t _hold_start_ms;    // time the hold started
    float _hold_fwd;            // mean wheel distance at the hold point
    float _hold_diff;           // half the right minus left distance at the hold point
    int8_t _coast_dir;          // direction the wheels were still turning when the hold engaged (+1, -1), 0 once locked
    bool _use_sides;            // true if holding each side separately
    bool _yaw_hold;             // true if the heading is being held (no turn requested)
    SideHold _side[2];          // left/right, or [0] for the whole vehicle
    float _hold_output;         // mean holding throttle (-1 to +1)
    float _hold_steering;       // steering that holds the heading (-1 to +1)
    uint32_t _release_start_ms; // time the release started
    float _release_output;      // holding throttle when the release started (-1 to +1)
    uint32_t _last_log_ms;      // last time the HHC message was logged

    // last steady hold, used to preload the next hold on the same slope
    bool _steady_valid;         // true once a steady hold has been seen
    float _steady_output;       // mean holding throttle of the last steady hold (-1 to +1)
    float _steady_pitch;        // pitch of the last steady hold in radians
};

#endif  // AR_HILLHOLDCONTROL_ENABLED

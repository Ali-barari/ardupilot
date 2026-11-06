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

#include "AR_HillHoldControl.h"

#if AR_HILLHOLDCONTROL_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>
#include <APM_Control/AR_AttitudeControl.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

// if neither update method has run for this long the vehicle's mode is
// not using hill hold (e.g. Manual) and the next update starts afresh
#define AR_HHC_UPDATE_TIMEOUT_MS    200

// maximum rate of the HHC log message
#define AR_HHC_LOG_INTERVAL_MS      40

// a hold is steady once it has run this long with the wheels at rest
// near the hold point; its throttle is then remembered for the next hold
#define AR_HHC_STEADY_TIME_MS       1000

// pitch difference within which the next hold is assumed to be on the same slope
#define AR_HHC_SAME_SLOPE_DEG       2.0f

// time constant in s with which an I term too small to turn the wheels is bled away
#define AR_HHC_BLEED_TC             1.0f

// a desired turn rate above this suspends the heading hold
#define AR_HHC_TURN_RATE_MIN_DEG    1.0f

// after a turn the heading is held again once the sides' speed
// difference (half right minus left) has fallen below this, in m/s
#define AR_HHC_YAW_LOCK_SPEED       0.02f

const AP_Param::GroupInfo AR_HillHoldControl::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Hill Hold Control enable
    // @Description: Holds the vehicle in place using the wheel encoders when the mode asks for zero speed, so it does not roll down a slope. Active in every mode that uses the speed controller and in Hold mode, never in Manual. Requires wheel encoders.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 1, AR_HillHoldControl, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // 2 was the POS_ PID group, replaced by P, I, IMAX and DAMP

    // @Param: SLOPE_FF
    // @DisplayName: Hill hold slope feedforward
    // @Description: Throttle added per unit of sin(pitch) while holding, so the hold pushes against the slope before the vehicle moves. 0 holds using the wheel encoders only
    // @Units: %
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SLOPE_FF", 3, AR_HillHoldControl, _slope_ff, 0.0f),

    // @Param: MAX
    // @DisplayName: Hill hold throttle maximum
    // @Description: Maximum throttle the hold may use in either direction
    // @Units: %
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("MAX", 4, AR_HillHoldControl, _thr_max, 50.0f),

    // @Param: DB
    // @DisplayName: Hill hold deadband
    // @Description: Distance from the hold point within which the hold does not correct. Stops the motors hunting around the hold point
    // @Units: m
    // @Range: 0 0.05
    // @Increment: 0.001
    // @User: Standard
    AP_GROUPINFO("DB", 5, AR_HillHoldControl, _deadband, 0.003f),

    // @Param: SPD
    // @DisplayName: Hill hold engage speed
    // @Description: Once zero speed is requested, the hold engages when the mean wheel speed falls below this value
    // @Units: m/s
    // @Range: 0.01 0.5
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("SPD", 6, AR_HillHoldControl, _engage_speed, 0.15f),

    // @Param: REL_TIME
    // @DisplayName: Hill hold release time
    // @Description: When a speed is requested, the holding throttle fades out over this time while the speed controller takes over. 0 releases immediately
    // @Units: s
    // @Range: 0 3
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("REL_TIME", 7, AR_HillHoldControl, _release_time, 0.5f),

    // @Param: TMAX
    // @DisplayName: Hill hold time limit
    // @Description: Stop holding after this long and let the vehicle roll, to protect the motors. 0 holds without a time limit
    // @Units: s
    // @Range: 0 600
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("TMAX", 8, AR_HillHoldControl, _timeout, 0.0f),

    // @Param: P
    // @DisplayName: Hill hold stiffness
    // @Description: Corrective speed (m/s) per metre from the hold point, scaled to throttle by CRUISE_THROTTLE / CRUISE_SPEED. Higher values push back harder against a disturbance
    // @Range: 0 100
    // @Increment: 0.5
    // @User: Standard
    AP_GROUPINFO("P", 9, AR_HillHoldControl, _kp, 10.0f),

    // @Param: I
    // @DisplayName: Hill hold I gain
    // @Description: Builds up the corrective speed (m/s, scaled to throttle by CRUISE_THROTTLE / CRUISE_SPEED) needed to hold the vehicle on a slope, per metre-second from the hold point
    // @Range: 0 100
    // @Increment: 0.5
    // @User: Standard
    AP_GROUPINFO("I", 10, AR_HillHoldControl, _ki, 5.0f),

    // @Param: IMAX
    // @DisplayName: Hill hold I term maximum
    // @Description: Maximum corrective speed the I term may build up. HHC_MAX also limits the final throttle
    // @Units: m/s
    // @Range: 0 10
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("IMAX", 11, AR_HillHoldControl, _imax, 5.0f),

    // @Param: DAMP
    // @DisplayName: Hill hold damping
    // @Description: Corrective speed (m/s) per m/s of filtered wheel speed. Resists movement as soon as the wheels turn and damps the return to the hold point
    // @Range: 0 10
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("DAMP", 12, AR_HillHoldControl, _damp, 1.0f),

    // @Param: MIN
    // @DisplayName: Hill hold dead-zone compensation
    // @Description: Throttle added in the direction of the correction whenever the vehicle is outside the deadband, so the hold produces torque straight away on motor drivers that need a minimum duty to turn the wheels. An I term below this value is bled away while inside the deadband
    // @Units: %
    // @Range: 0 20
    // @Increment: 0.5
    // @User: Standard
    AP_GROUPINFO("MIN", 13, AR_HillHoldControl, _thr_min, 0.0f),

    // @Param: HYST
    // @DisplayName: Hill hold direction hysteresis
    // @Description: The holding output of a side only changes direction once it exceeds this throttle in the new direction; below it the output is zero. Stops direction relays on brushed motors chattering around zero
    // @Units: %
    // @Range: 0 10
    // @Increment: 0.5
    // @User: Advanced
    AP_GROUPINFO("HYST", 14, AR_HillHoldControl, _dir_hyst, 1.0f),

    AP_GROUPEND
};

AR_HillHoldControl::AR_HillHoldControl(const AR_WheelState &wheel_state) :
    _wheel_state(wheel_state)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// true if enabled, armed and the wheel encoders are healthy
bool AR_HillHoldControl::can_run() const
{
    return enabled() && hal.util->get_soft_armed() && _wheel_state.healthy();
}

// change state
void AR_HillHoldControl::set_state(State new_state)
{
    if (_state == new_state) {
        return;
    }
    _state = new_state;
    if (_state != State::HOLDING) {
        _hold_output = 0.0f;
        _hold_steering = 0.0f;
    }
#if HAL_LOGGING_ENABLED
    // log every state change, regardless of the rate limit
    _last_log_ms = 0;
    write_log(0.0f, 0.0f, 0.0f);
#endif
}

// start holding at the current wheel position.  The I terms are
// preloaded so the hold starts with the torque the slope needs instead of
// zero (bumpless transfer), minus whatever the slope feedforward covers:
// - the last steady hold's throttle if the pitch is about the same (same slope)
// - otherwise the speed controller's I term, which holds part of the
//   slope and friction throttle while the vehicle slows down
void AR_HillHoldControl::start_hold(uint32_t now_ms, float throttle_per_speed, float ff)
{
    _use_sides = _wheel_state.have_sides();
    _yaw_hold = true;
    _hold_fwd = _wheel_state.get_mean_distance();
    _hold_diff = _use_sides ? 0.5f * (_wheel_state.get_side_distance(AR_WheelState::Side::RIGHT) - _wheel_state.get_side_distance(AR_WheelState::Side::LEFT)) : 0.0f;
    _hold_start_ms = now_ms;

    // the hold can engage while the vehicle is still coasting to a stop.
    // Remember the direction so the hold point can follow it until the
    // wheels stop, and the vehicle is held where it stops
    const float speed = _wheel_state.get_mean_speed();
    _coast_dir = _wheel_state.is_stationary() ? 0 : (is_negative(speed) ? -1 : 1);

    float expected_output = 0.0f;
    const float pitch = AP::ahrs().get_pitch();
    if (_steady_valid && (fabsf(pitch - _steady_pitch) < radians(AR_HHC_SAME_SLOPE_DEG))) {
        expected_output = _steady_output;
    } else {
        const AR_AttitudeControl *attitude_control = AR_AttitudeControl::get_singleton();
        if ((attitude_control != nullptr) && attitude_control->speed_control_active()) {
            expected_output = attitude_control->get_throttle_speed_pid_info().I;
        }
    }
    const float imax = MAX(_imax, 0.0f);
    const float preload = is_positive(throttle_per_speed) ? constrain_float((expected_output - ff) / throttle_per_speed, -imax, imax) : 0.0f;
    for (SideHold &side : _side) {
        side.integrator = preload;
        side.direction = 0;
        side.output = 0.0f;
    }

    set_state(State::HOLDING);
}

// run one side's controller and return its throttle (-1 to +1)
float AR_HillHoldControl::update_side(SideHold &side, float error, float speed, float throttle_per_speed, float ff, float dt)
{
    // distance from the hold point outside the deadband
    const float deadband = MAX(_deadband, 0.0f);
    float error_db = 0.0f;
    if (error > deadband) {
        error_db = error - deadband;
    } else if (error < -deadband) {
        error_db = error + deadband;
    }

    // I term, not winding further while the output is saturated in that direction
    const float out_max = constrain_float(_thr_max, 0.0f, 100.0f) * 0.01f;
    const bool saturated = fabsf(side.output) >= out_max;
    if (!(saturated && (error_db * side.output) > 0.0f)) {
        side.integrator += _ki * error_db * dt;
    }
    const float imax = MAX(_imax, 0.0f);
    side.integrator = constrain_float(side.integrator, -imax, imax);

    // stiffness, I term and damping give a corrective speed, scaled to
    // throttle, plus slope feedforward
    float out = (_kp * error_db + side.integrator - _damp * speed) * throttle_per_speed + ff;

    const float thr_min = MAX(_thr_min, 0.0f) * 0.01f;
    if (!is_zero(error_db)) {
        // outside the deadband: start from the drivers' minimum working
        // duty so the correction produces torque straight away
        out += is_positive(error_db) ? thr_min : -thr_min;
    } else if (fabsf(out) < thr_min) {
        // inside the deadband with an output too small to turn the
        // wheels: bleed the I term away instead of holding a useless current
        side.integrator *= MAX(1.0f - dt / AR_HHC_BLEED_TC, 0.0f);
    }

    // only change direction once the output clearly asks for it
    const float hyst = MAX(_dir_hyst, 0.0f) * 0.01f;
    const int8_t dir = is_positive(out) ? 1 : (is_negative(out) ? -1 : 0);
    if ((side.direction != 0) && (dir != side.direction) && (fabsf(out) < hyst)) {
        out = 0.0f;
    } else if ((dir != 0) && (fabsf(out) >= hyst)) {
        side.direction = dir;
    }

    side.output = constrain_float(out, -out_max, out_max);
    return side.output;
}

// call each loop while the mode asks for zero speed.  throttle_out
// holds the stop controller's output (-100 to +100) and is replaced
// with the holding throttle while holding.  cruise_speed (m/s) and
// cruise_throttle (0 to 1) scale the corrective speed to throttle.
// Returns true while holding
bool AR_HillHoldControl::update_stopped(float cruise_speed, float cruise_throttle, float dt, float &throttle_out)
{
    const uint32_t now_ms = AP_HAL::millis();

    if (!can_run()) {
        set_state(State::OFF);
        if (!hal.util->get_soft_armed()) {
            // the slope may be different when next armed
            _steady_valid = false;
        }
        _last_update_ms = now_ms;
        return false;
    }

    // not updated recently, so the mode was not using hill hold: start afresh
    if ((now_ms - _last_update_ms) > AR_HHC_UPDATE_TIMEOUT_MS) {
        set_state(State::IDLE);
    }
    _last_update_ms = now_ms;

    const float throttle_per_speed = (is_positive(cruise_speed) && is_positive(cruise_throttle)) ? (cruise_throttle / cruise_speed) : 1.0f;
    const float ff = 0.01f * _slope_ff * sinf(AP::ahrs().get_pitch());

    switch (_state) {
    case State::TIMEOUT:
        // stay released until a speed is requested
        return false;

    case State::OFF:
    case State::RELEASING:
        // zero speed requested again before the release finished: capture a new hold point
        set_state(State::IDLE);
        FALLTHROUGH;

    case State::IDLE:
        // let the stop controller slow the vehicle until the wheels have nearly stopped
        if (fabsf(_wheel_state.get_mean_speed()) > _engage_speed) {
            return false;
        }
        start_hold(now_ms, throttle_per_speed, ff);
        FALLTHROUGH;

    case State::HOLDING:
        break;
    }

    // give up after the time limit
    if (is_positive(_timeout) && (now_ms - _hold_start_ms) > (uint32_t)(_timeout * 1000.0f)) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Hill hold: time limit, released");
        set_state(State::TIMEOUT);
        return false;
    }

    const float fwd = _wheel_state.get_mean_distance();
    const float diff = _use_sides ? 0.5f * (_wheel_state.get_side_distance(AR_WheelState::Side::RIGHT) - _wheel_state.get_side_distance(AR_WheelState::Side::LEFT)) : 0.0f;

    // while still coasting in the direction it was travelling, move the
    // hold point with the vehicle.  Lock it once the wheels stop or reverse
    if (_coast_dir != 0) {
        const float speed = _wheel_state.get_mean_speed();
        if (_wheel_state.is_stationary() || ((speed * _coast_dir) <= 0.0f)) {
            _coast_dir = 0;
        } else if (((fwd - _hold_fwd) * _coast_dir) > 0.0f) {
            _hold_fwd = fwd;
        }
    }

    // while a turn is requested the heading follows the vehicle
    if (!_yaw_hold) {
        _hold_diff = diff;
    }

    // distance from the hold point (+ve means behind it) and heading error
    const float error_fwd = _hold_fwd - fwd;
    const float error_yaw = _hold_diff - diff;

    if (_use_sides) {
        const float out_left = update_side(_side[0], error_fwd - error_yaw, _wheel_state.get_side_speed(AR_WheelState::Side::LEFT), throttle_per_speed, ff, dt);
        const float out_right = update_side(_side[1], error_fwd + error_yaw, _wheel_state.get_side_speed(AR_WheelState::Side::RIGHT), throttle_per_speed, ff, dt);
        _hold_output = 0.5f * (out_left + out_right);
        _hold_steering = 0.5f * (out_left - out_right);
    } else {
        _hold_output = update_side(_side[0], error_fwd, _wheel_state.get_mean_speed(), throttle_per_speed, ff, dt);
        _hold_steering = 0.0f;
    }

    throttle_out = _hold_output * 100.0f;

    // remember the throttle of a steady hold for the next hold on this slope
    const float deadband = MAX(_deadband, 0.0f);
    if (((now_ms - _hold_start_ms) > AR_HHC_STEADY_TIME_MS) && _wheel_state.is_stationary() &&
        (fabsf(error_fwd) <= MAX(2.0f * deadband, 0.01f)) &&
        (fabsf(_hold_output) < constrain_float(_thr_max, 0.0f, 100.0f) * 0.01f)) {
        _steady_valid = true;
        _steady_output = _hold_output;
        _steady_pitch = AP::ahrs().get_pitch();
    }

#if HAL_LOGGING_ENABLED
    write_log(error_fwd, error_yaw, ff);
#endif
    return true;
}

// call after update_stopped with the mode's desired turn rate (rad/s)
// and steering output (-1 to +1).  While holding with no turn
// requested the steering is replaced with the output that holds the
// heading.  A turn request suspends the heading hold
void AR_HillHoldControl::update_steering(float desired_turn_rate, float &steering_out)
{
    if ((_state != State::HOLDING) || !_use_sides) {
        return;
    }
    if (fabsf(desired_turn_rate) > radians(AR_HHC_TURN_RATE_MIN_DEG)) {
        // turning in place: leave steering to the mode
        _yaw_hold = false;
        return;
    }
    if (!_yaw_hold) {
        // turn finished: let the mode stop the rotation, then hold the
        // heading where it stops (from the next update)
        const float rotation_speed = 0.5f * (_wheel_state.get_side_speed(AR_WheelState::Side::RIGHT) - _wheel_state.get_side_speed(AR_WheelState::Side::LEFT));
        if (fabsf(rotation_speed) < AR_HHC_YAW_LOCK_SPEED) {
            _yaw_hold = true;
        }
        return;
    }
    steering_out = _hold_steering;
}

// call each loop while the mode asks for a non-zero speed.
// throttle_out holds the speed controller's output (-100 to +100);
// the fading holding throttle is added to it while releasing
void AR_HillHoldControl::update_moving(float target_speed, float dt, float &throttle_out)
{
    const uint32_t now_ms = AP_HAL::millis();

    if (!can_run()) {
        set_state(State::OFF);
        _last_update_ms = now_ms;
        return;
    }

    const bool stale = (now_ms - _last_update_ms) > AR_HHC_UPDATE_TIMEOUT_MS;
    _last_update_ms = now_ms;
    if (stale) {
        set_state(State::IDLE);
        return;
    }

    switch (_state) {
    case State::OFF:
    case State::IDLE:
    case State::TIMEOUT:
        set_state(State::IDLE);
        return;

    case State::HOLDING: {
        // fade out the holding throttle only if it pushes the way the
        // vehicle has been asked to go, e.g. uphill
        const float hold_output = _hold_output;
        if (!is_positive(_release_time) || is_zero(hold_output) || (target_speed * hold_output) < 0.0f) {
            set_state(State::IDLE);
            return;
        }
        _release_output = hold_output;
        _release_start_ms = now_ms;
        set_state(State::RELEASING);
        break;
    }

    case State::RELEASING:
        break;
    }

    // release is done once the time is up or the vehicle is moving the requested way
    const float elapsed = (now_ms - _release_start_ms) * 0.001f;
    const bool moving_as_requested = (_wheel_state.get_mean_speed() * (is_negative(target_speed) ? -1.0f : 1.0f)) > _engage_speed;
    if (elapsed >= _release_time || moving_as_requested) {
        set_state(State::IDLE);
        return;
    }

    const float blend = _release_output * (1.0f - elapsed / _release_time);
    throttle_out = constrain_float(throttle_out + blend * 100.0f, -100.0f, 100.0f);

#if HAL_LOGGING_ENABLED
    write_log(0.0f, 0.0f, 0.0f);
#endif
}

#if HAL_LOGGING_ENABLED
// write the HHC log message
void AR_HillHoldControl::write_log(float error_fwd, float error_yaw, float ff)
{
    const uint32_t now_ms = AP_HAL::millis();
    if ((_last_log_ms != 0) && (now_ms - _last_log_ms) < AR_HHC_LOG_INTERVAL_MS) {
        return;
    }
    _last_log_ms = now_ms;

    // @LoggerMessage: HHC
    // @Description: Hill Hold Control
    // @Field: TimeUS: Time since system startup
    // @Field: St: State: 0=Off, 1=Idle, 2=Holding, 3=Releasing, 4=Timeout
    // @Field: EF: Distance from the hold point, +ve means the vehicle is behind it
    // @Field: EY: Heading error as half the right minus left side distance error
    // @Field: Spd: Mean wheel speed
    // @Field: Pitch: Vehicle pitch
    // @Field: OutL: Left side (or whole vehicle) holding throttle
    // @Field: OutR: Right side holding throttle
    // @Field: IL: Left side (or whole vehicle) I term
    // @Field: IR: Right side I term
    // @Field: FF: Slope feedforward throttle
    // @Field: Str: Steering that holds the heading
    // @Field: Out: Mean holding throttle while holding, fading throttle while releasing
    // @Field: SOut: Throttle of the last steady hold, used to preload the next hold on the same slope. SOut / sin(SPitch) * 100 gives a starting value for HHC_SLOPE_FF
    // @Field: SPitch: Pitch of the last steady hold
    float out = _hold_output;
    if (_state == State::RELEASING) {
        out = _release_output * (1.0f - constrain_float((now_ms - _release_start_ms) * 0.001f / MAX(_release_time.get(), 0.001f), 0.0f, 1.0f));
    }
    AP::logger().WriteStreaming("HHC", "TimeUS,St,EF,EY,Spd,Pitch,OutL,OutR,IL,IR,FF,Str,Out,SOut,SPitch",
                                "s-mmnd--nn----d", "F-0000000000000", "QBfffffffffffff",
                                AP_HAL::micros64(),
                                (uint8_t)_state,
                                (double)error_fwd,
                                (double)error_yaw,
                                (double)_wheel_state.get_mean_speed(),
                                (double)degrees(AP::ahrs().get_pitch()),
                                (double)_side[0].output,
                                (double)_side[1].output,
                                (double)_side[0].integrator,
                                (double)_side[1].integrator,
                                (double)ff,
                                (double)_hold_steering,
                                (double)out,
                                (double)(_steady_valid ? _steady_output : 0.0f),
                                (double)(_steady_valid ? degrees(_steady_pitch) : 0.0f));
}
#endif  // HAL_LOGGING_ENABLED

#endif  // AR_HILLHOLDCONTROL_ENABLED

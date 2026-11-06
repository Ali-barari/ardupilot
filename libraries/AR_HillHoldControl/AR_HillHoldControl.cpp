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

const AP_Param::GroupInfo AR_HillHoldControl::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Hill Hold Control enable
    // @Description: Holds the vehicle in place using the wheel encoders when the mode asks for zero speed, so it does not roll down a slope. Active in every mode that uses the speed controller and in Hold mode, never in Manual. Requires wheel encoders.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 1, AR_HillHoldControl, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: POS_P
    // @DisplayName: Hill hold position P gain
    // @Description: Converts the distance from the hold point (in metres) to a corrective speed (in m/s), which is scaled to throttle by CRUISE_THROTTLE / CRUISE_SPEED
    // @Range: 0.0 20.0
    // @Increment: 0.1
    // @User: Standard

    // @Param: POS_I
    // @DisplayName: Hill hold position I gain
    // @Description: Builds up the corrective speed (in m/s, scaled to throttle by CRUISE_THROTTLE / CRUISE_SPEED) needed to hold the vehicle on a slope from the distance from the hold point (in metres)
    // @Range: 0.0 20.0
    // @Increment: 0.1
    // @User: Standard

    // @Param: POS_IMAX
    // @DisplayName: Hill hold position I gain maximum
    // @Description: Constrains the corrective speed (in m/s) that the I term will generate. HHC_MAX also limits the final throttle
    // @Range: 0.0 10.0
    // @Increment: 0.1
    // @User: Standard

    // @Param: POS_D
    // @DisplayName: Hill hold position D gain
    // @Description: Damps movement around the hold point. Converts the rate of change of the distance from the hold point (in m/s) to a corrective speed (in m/s)
    // @Range: 0.0 5.0
    // @Increment: 0.01
    // @User: Standard

    // @Param: POS_FF
    // @DisplayName: Hill hold position feed forward
    // @Description: Unused, leave at zero. Use HHC_SLOPE_FF for slope feedforward
    // @Range: 0.000 0.000
    // @User: Advanced

    // @Param: POS_FLTT
    // @DisplayName: Hill hold position target filter frequency
    // @Description: Target filter frequency in Hz
    // @Range: 0.000 100.000
    // @Increment: 0.1
    // @Units: Hz
    // @User: Advanced

    // @Param: POS_FLTE
    // @DisplayName: Hill hold position error filter frequency
    // @Description: Error filter frequency in Hz
    // @Range: 0.000 100.000
    // @Increment: 0.1
    // @Units: Hz
    // @User: Advanced

    // @Param: POS_FLTD
    // @DisplayName: Hill hold position derivative filter frequency
    // @Description: Derivative filter frequency in Hz
    // @Range: 0.000 100.000
    // @Increment: 0.1
    // @Units: Hz
    // @User: Advanced

    // @Param: POS_SMAX
    // @DisplayName: Hill hold position slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. A value of zero disables this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    // @Param: POS_PDMX
    // @DisplayName: Hill hold position PD sum maximum
    // @Description: The maximum/minimum corrective speed (in m/s) that the sum of the P and D term can output. 0 disables the limit
    // @Range: 0.0 10.0
    // @Increment: 0.01
    // @User: Advanced

    // @Param: POS_D_FF
    // @DisplayName: Hill hold position derivative feedforward gain
    // @Description: Unused, leave at zero
    // @Range: 0 0
    // @User: Advanced

    // @Param: POS_NTF
    // @DisplayName: Hill hold position target notch filter index
    // @Description: Target notch filter index
    // @Range: 1 8
    // @User: Advanced

    // @Param: POS_NEF
    // @DisplayName: Hill hold position error notch filter index
    // @Description: Error notch filter index
    // @Range: 1 8
    // @User: Advanced
    AP_SUBGROUPINFO(_pid, "POS_", 2, AR_HillHoldControl, AC_PID),

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
    // @Description: Distance from the hold point within which the hold does not correct. Stops the motors hunting around the hold point on flat ground
    // @Units: m
    // @Range: 0 0.05
    // @Increment: 0.001
    // @User: Standard
    AP_GROUPINFO("DB", 5, AR_HillHoldControl, _deadband, 0.005f),

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
        _output_limited = false;
    }
#if HAL_LOGGING_ENABLED
    // log every state change, regardless of the rate limit
    _last_log_ms = 0;
    write_log(0.0f, 0.0f, 0.0f);
#endif
}

// start holding at the current wheel position.  The I term is
// preloaded so the hold starts with the torque the slope needs instead of
// zero (bumpless transfer), minus whatever the slope feedforward covers:
// - the last steady hold's throttle if the pitch is about the same (same slope)
// - otherwise the speed controller's I term, which holds part of the
//   slope and friction throttle while the vehicle slows down
void AR_HillHoldControl::start_hold(uint32_t now_ms, float throttle_per_speed, float ff)
{
    _hold_distance = _wheel_state.get_mean_distance();
    _hold_start_ms = now_ms;
    _pid.reset_filter();

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
    const float imax = _pid.imax();
    const float preload = is_positive(throttle_per_speed) ? (expected_output - ff) / throttle_per_speed : 0.0f;
    _pid.set_integrator(constrain_float(preload, -imax, imax));

    set_state(State::HOLDING);
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
        _last_update_ms = now_ms;
        return false;
    }

    // not updated recently, so the mode was not using hill hold: start afresh
    if ((now_ms - _last_update_ms) > AR_HHC_UPDATE_TIMEOUT_MS) {
        set_state(State::IDLE);
    }
    _last_update_ms = now_ms;

    // the PID gives a corrective speed, converted to throttle with the
    // cruise ratio.  Slope feedforward adds the throttle the slope needs
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

    // while still coasting in the direction it was travelling, move the
    // hold point with the vehicle.  Lock it once the wheels stop or reverse
    if (_coast_dir != 0) {
        const float speed = _wheel_state.get_mean_speed();
        if (_wheel_state.is_stationary() || ((speed * _coast_dir) <= 0.0f)) {
            _coast_dir = 0;
        } else if (((_wheel_state.get_mean_distance() - _hold_distance) * _coast_dir) > 0.0f) {
            _hold_distance = _wheel_state.get_mean_distance();
        }
    }

    // distance from the hold point (+ve means the vehicle is behind it), with deadband
    const float error = _hold_distance - _wheel_state.get_mean_distance();
    const float deadband = MAX(_deadband, 0.0f);
    float error_db = 0.0f;
    if (error > deadband) {
        error_db = error - deadband;
    } else if (error < -deadband) {
        error_db = error + deadband;
    }

    // PID on the distance from the hold point, plus slope feedforward
    const float pid_out = _pid.update_all(error_db, 0.0f, dt, _output_limited) * throttle_per_speed;
    const float out_max = constrain_float(_thr_max, 0.0f, 100.0f) * 0.01f;
    const float out_unlimited = pid_out + ff;
    _output_limited = fabsf(out_unlimited) >= out_max;
    _hold_output = constrain_float(out_unlimited, -out_max, out_max);

    throttle_out = _hold_output * 100.0f;

    // remember the throttle of a steady hold for the next hold on this slope
    if (((now_ms - _hold_start_ms) > AR_HHC_STEADY_TIME_MS) && !_output_limited &&
        _wheel_state.is_stationary() && (fabsf(error) <= MAX(2.0f * deadband, 0.01f))) {
        _steady_valid = true;
        _steady_output = _hold_output;
        _steady_pitch = AP::ahrs().get_pitch();
    }

#if HAL_LOGGING_ENABLED
    write_log(error, ff, _hold_output);
#endif
    return true;
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
    write_log(0.0f, 0.0f, blend);
#endif
}

#if HAL_LOGGING_ENABLED
// write the HHC log message
void AR_HillHoldControl::write_log(float error, float ff, float out)
{
    const uint32_t now_ms = AP_HAL::millis();
    if ((_last_log_ms != 0) && (now_ms - _last_log_ms) < AR_HHC_LOG_INTERVAL_MS) {
        return;
    }
    _last_log_ms = now_ms;

    const AP_PIDInfo &pid_info = _pid.get_pid_info();

    // @LoggerMessage: HHC
    // @Description: Hill Hold Control
    // @Field: TimeUS: Time since system startup
    // @Field: St: State: 0=Off, 1=Idle, 2=Holding, 3=Releasing, 4=Timeout
    // @Field: Err: Distance from the hold point, +ve means the vehicle is behind it
    // @Field: Spd: Mean wheel speed
    // @Field: Pitch: Vehicle pitch
    // @Field: P: P term corrective speed
    // @Field: I: I term corrective speed
    // @Field: D: D term corrective speed
    // @Field: FF: Slope feedforward output
    // @Field: Out: Holding throttle while holding, fading throttle while releasing
    // @Field: SOut: Throttle of the last steady hold, used to preload the next hold on the same slope. SOut / sin(SPitch) * 100 gives a starting value for HHC_SLOPE_FF
    // @Field: SPitch: Pitch of the last steady hold
    AP::logger().WriteStreaming("HHC", "TimeUS,St,Err,Spd,Pitch,P,I,D,FF,Out,SOut,SPitch",
                                "s-mndnnn---d", "F-0000000000", "QBffffffffff",
                                AP_HAL::micros64(),
                                (uint8_t)_state,
                                (double)error,
                                (double)_wheel_state.get_mean_speed(),
                                (double)degrees(AP::ahrs().get_pitch()),
                                (double)pid_info.P,
                                (double)pid_info.I,
                                (double)pid_info.D,
                                (double)ff,
                                (double)out,
                                (double)(_steady_valid ? _steady_output : 0.0f),
                                (double)(_steady_valid ? degrees(_steady_pitch) : 0.0f));
}
#endif  // HAL_LOGGING_ENABLED

#endif  // AR_HILLHOLDCONTROL_ENABLED

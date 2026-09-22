#include "Rover.h"

#include <AP_RangeFinder/AP_RangeFinder_Backend.h>

// check for new compass data - 10Hz
void Rover::update_compass(void)
{
    compass.read();
}

// Save compass offsets
void Rover::compass_save() {
    if (AP::compass().available() &&
        compass.get_learn_type() >= Compass::LEARN_INTERNAL &&
        !arming.is_armed()) {
        compass.save_offsets();
    }
}

// update wheel encoders
void Rover::update_wheel_encoder()
{
    // exit immediately if not enabled
    if (g2.wheel_encoder.num_sensors() == 0) {
        return;
    }

    // update encoders
    g2.wheel_encoder.update();

    // save cumulative distances at current time (in meters) for reporting to GCS
    for (uint8_t i = 0; i < g2.wheel_encoder.num_sensors(); i++) {
        wheel_encoder_last_distance_m[i] = g2.wheel_encoder.get_distance(i);
    }

    // send wheel encoder delta angle and delta time to EKF
    // this should not be done at more than 50hz
    // initialise on first iteration
    if (!wheel_encoder_initialised) {
        wheel_encoder_initialised = true;
        for (uint8_t i = 0; i < g2.wheel_encoder.num_sensors(); i++) {
            wheel_encoder_last_angle_rad[i] = g2.wheel_encoder.get_delta_angle(i);
            wheel_encoder_last_reading_ms[i] = g2.wheel_encoder.get_last_reading_ms(i);
        }
        return;
    }

    // on each iteration send data from alternative wheel encoders
    wheel_encoder_last_index_sent++;
    if (wheel_encoder_last_index_sent >= g2.wheel_encoder.num_sensors()) {
        wheel_encoder_last_index_sent = 0;
    }

    // get current time, total delta angle (since startup) and update time from sensor
    const float curr_angle_rad = g2.wheel_encoder.get_delta_angle(wheel_encoder_last_index_sent);
    const uint32_t sensor_reading_ms = g2.wheel_encoder.get_last_reading_ms(wheel_encoder_last_index_sent);
    const uint32_t now_ms = AP_HAL::millis();

    // calculate angular change (in radians)
#if HAL_NAVEKF3_AVAILABLE
    const float delta_angle = curr_angle_rad - wheel_encoder_last_angle_rad[wheel_encoder_last_index_sent];
#endif
    wheel_encoder_last_angle_rad[wheel_encoder_last_index_sent] = curr_angle_rad;

    // calculate delta time using time between sensor readings or time since last send to ekf (whichever is shorter)
    uint32_t sensor_diff_ms = sensor_reading_ms - wheel_encoder_last_reading_ms[wheel_encoder_last_index_sent];
    if (sensor_diff_ms == 0 || sensor_diff_ms > 100) {
        // if no sensor update or time difference between sensor readings is too long use time since last send to ekf
        sensor_diff_ms = now_ms - wheel_encoder_last_reading_ms[wheel_encoder_last_index_sent];
        wheel_encoder_last_reading_ms[wheel_encoder_last_index_sent] = now_ms;
    } else {
        wheel_encoder_last_reading_ms[wheel_encoder_last_index_sent] = sensor_reading_ms;
    }
#if HAL_NAVEKF3_AVAILABLE
    const float delta_time = sensor_diff_ms * 0.001f;

    /* delAng is the measured change in angular position from the previous measurement where a positive rotation is produced by forward motion of the vehicle (rad)
     * delTime is the time interval for the measurement of delAng (sec)
     * timeStamp_ms is the time when the rotation was last measured (msec)
     * posOffset is the XYZ body frame position of the wheel hub (m)
     */
    ahrs.EKF3.writeWheelOdom(delta_angle,
                        delta_time,
                        wheel_encoder_last_reading_ms[wheel_encoder_last_index_sent],
                        g2.wheel_encoder.get_pos_offset(wheel_encoder_last_index_sent),
                        g2.wheel_encoder.get_wheel_radius(wheel_encoder_last_index_sent));
#endif
}

// update wheel distance from an external (MAVLink-fed) encoder board
//
// Feeds the same EKF3 wheel-odometry input as update_wheel_encoder() above,
// just sourced from AP_WheelDistance_Mav (WHEEL_DISTANCE messages from an
// external STM32 board) instead of native AP_WheelEncoder GPIO instances.
// Mutually exclusive with update_wheel_encoder(): both ultimately call
// ahrs.EKF3.writeWheelOdom() with no per-source instance id, so running
// both at once would interleave two independent odometry sources into one
// EKF input. See docs/ARCTERON_WHEEL_ENCODERS_TRACTION_CONTROL.md and
// docs/ARCTERON_ARDUPILOT_WHEEL_DISTANCE_PATCH.md in the seair repo.
void Rover::update_wheel_distance_mav()
{
    // exit immediately if not enabled, or if native wheel encoders are
    // also configured (see function header - the two paths cannot coexist)
    if (!g2.wheel_distance_mav.enabled()) {
        return;
    }
    if (g2.wheel_encoder.num_sensors() > 0) {
        const uint32_t now_ms = AP_HAL::millis();
        if (now_ms - wheel_distance_mav_conflict_warn_ms > 5000) {
            wheel_distance_mav_conflict_warn_ms = now_ms;
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "WheelDistMav: disabled, WENC_TYPE also set");
        }
        return;
    }

    const uint8_t num_wheels = g2.wheel_distance_mav.num_wheels();
    if (num_wheels == 0) {
        // no WHEEL_DISTANCE packet received yet
        return;
    }

    // initialise on first iteration
    if (!wheel_distance_mav_initialised) {
        wheel_distance_mav_initialised = true;
        const uint32_t now_ms = AP_HAL::millis();
        for (uint8_t i = 0; i < num_wheels; i++) {
            wheel_distance_mav_last_distance_m[i] = g2.wheel_distance_mav.get_distance(i);
            wheel_distance_mav_last_time_usec[i] = g2.wheel_distance_mav.get_last_time_usec();
            wheel_distance_mav_last_reading_ms[i] = now_ms;
        }
        return;
    }

    // on each iteration send data from an alternative wheel, same
    // round-robin approach as update_wheel_encoder() above
    wheel_distance_mav_last_index_sent++;
    if (wheel_distance_mav_last_index_sent >= num_wheels) {
        wheel_distance_mav_last_index_sent = 0;
    }
    const uint8_t idx = wheel_distance_mav_last_index_sent;

    const float wheel_radius = g2.wheel_distance_mav.get_wheel_radius();
    if (!is_positive(wheel_radius)) {
        // avoid divide-by-zero if WHLDIST_RADIUS hasn't been configured
        return;
    }

    const float curr_distance_m = g2.wheel_distance_mav.get_distance(idx);
    const uint64_t curr_time_usec = g2.wheel_distance_mav.get_last_time_usec();
    const uint32_t now_ms = AP_HAL::millis();

    // convert linear distance delta (m) to angular delta (rad) using the
    // configured wheel radius, matching AP_WheelEncoder's convention that
    // writeWheelOdom() takes an angle, not a distance
#if HAL_NAVEKF3_AVAILABLE
    const float delta_distance_m = curr_distance_m - wheel_distance_mav_last_distance_m[idx];
    const float delta_angle = delta_distance_m / wheel_radius;
#endif
    wheel_distance_mav_last_distance_m[idx] = curr_distance_m;

    // calculate delta time using the board's own packet timestamps where
    // they've actually advanced (time_usec is monotonic-since-boot on the
    // board, NOT wall-clock - see ARCTERON_WHEEL_DISTANCE_MAVLINK_SPEC.md
    // - so only deltas between our own two readings of it are meaningful).
    // Fall back to wall-clock time since our last EKF feed if the board
    // hasn't produced a new packet between two round-robin visits to this
    // wheel (packet rate and this 50Hz round-robin aren't necessarily
    // synchronised).
    uint32_t sensor_diff_ms = (uint32_t)((curr_time_usec - wheel_distance_mav_last_time_usec[idx]) / 1000ULL);
    if (sensor_diff_ms == 0 || sensor_diff_ms > 100) {
        // board timestamp hasn't usefully advanced (stale/no new packet
        // since our last visit to this wheel) - fall back to our own
        // wall-clock gap since we last fed this wheel to the EKF
        sensor_diff_ms = now_ms - wheel_distance_mav_last_reading_ms[idx];
    }
    wheel_distance_mav_last_time_usec[idx] = curr_time_usec;
    wheel_distance_mav_last_reading_ms[idx] = now_ms;

    if (sensor_diff_ms == 0) {
        // nothing new since the last feed for this wheel - skip rather
        // than feed the EKF a zero-dt observation
        return;
    }

#if HAL_NAVEKF3_AVAILABLE
    const float delta_time = sensor_diff_ms * 0.001f;

    /* delAng is the measured change in angular position from the previous measurement where a positive rotation is produced by forward motion of the vehicle (rad)
     * delTime is the time interval for the measurement of delAng (sec)
     * timeStamp_ms is the time when the rotation was last measured (msec)
     * posOffset is the XYZ body frame position of the wheel hub (m)
     */
    ahrs.EKF3.writeWheelOdom(delta_angle,
                        delta_time,
                        now_ms,
                        g2.wheel_distance_mav.get_pos_offset(idx),
                        wheel_radius);
#endif
}

#if AP_RANGEFINDER_ENABLED
// read the rangefinders
void Rover::read_rangefinders(void)
{
    rangefinder.update();
#if HAL_LOGGING_ENABLED
    Log_Write_Depth();
#endif
}
#endif

#include "AP_WheelDistance_Mav.h"

#include <AP_HAL/AP_HAL.h>

const AP_Param::GroupInfo AP_WheelDistance_Mav::var_info[] = {

    // @Param: _ENABLE
    // @DisplayName: External wheel-distance (MAVLink) enable
    // @Description: Enables ingestion of WHEEL_DISTANCE mavlink messages (e.g. from an external 4-wheel quadrature encoder board) and feeds them to the EKF as wheel odometry. Do not enable at the same time as native wheel encoders (WENC_TYPE) - both paths feed the same EKF wheel-odometry input and are mutually exclusive.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("_ENABLE", 0, AP_WheelDistance_Mav, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: _RADIUS
    // @DisplayName: Wheel radius
    // @Description: Rolling radius of the wheels reported by the external encoder board, used to convert reported linear distance into angular distance for EKF fusion. All wheels are assumed to share the same radius.
    // @Units: m
    // @Range: 0.01 2.0
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("_RADIUS", 1, AP_WheelDistance_Mav, _wheel_radius, 0.15f),

    // @Param: 1_POS_X
    // @DisplayName: Wheel 1 (Right-Front) X position offset
    // @Description: X position of the Right-Front wheel hub in body frame. Positive X is forward of the origin.
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced

    // @Param: 1_POS_Y
    // @DisplayName: Wheel 1 (Right-Front) Y position offset
    // @Description: Y position of the Right-Front wheel hub in body frame. Positive Y is to the right of the origin.
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced

    // @Param: 1_POS_Z
    // @DisplayName: Wheel 1 (Right-Front) Z position offset
    // @Description: Z position of the Right-Front wheel hub in body frame. Positive Z is down from the origin.
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("1_POS", 2, AP_WheelDistance_Mav, _pos_offset[0], 0.0f),

    // @Param: 2_POS_X
    // @DisplayName: Wheel 2 (Left-Front) X position offset
    // @Description: X position of the Left-Front wheel hub in body frame. Positive X is forward of the origin.
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced

    // @Param: 2_POS_Y
    // @DisplayName: Wheel 2 (Left-Front) Y position offset
    // @Description: Y position of the Left-Front wheel hub in body frame. Positive Y is to the right of the origin.
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced

    // @Param: 2_POS_Z
    // @DisplayName: Wheel 2 (Left-Front) Z position offset
    // @Description: Z position of the Left-Front wheel hub in body frame. Positive Z is down from the origin.
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("2_POS", 3, AP_WheelDistance_Mav, _pos_offset[1], 0.0f),

    // @Param: 3_POS_X
    // @DisplayName: Wheel 3 (Right-Rear) X position offset
    // @Description: X position of the Right-Rear wheel hub in body frame. Positive X is forward of the origin.
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced

    // @Param: 3_POS_Y
    // @DisplayName: Wheel 3 (Right-Rear) Y position offset
    // @Description: Y position of the Right-Rear wheel hub in body frame. Positive Y is to the right of the origin.
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced

    // @Param: 3_POS_Z
    // @DisplayName: Wheel 3 (Right-Rear) Z position offset
    // @Description: Z position of the Right-Rear wheel hub in body frame. Positive Z is down from the origin.
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("3_POS", 4, AP_WheelDistance_Mav, _pos_offset[2], 0.0f),

    // @Param: 4_POS_X
    // @DisplayName: Wheel 4 (Left-Rear) X position offset
    // @Description: X position of the Left-Rear wheel hub in body frame. Positive X is forward of the origin.
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced

    // @Param: 4_POS_Y
    // @DisplayName: Wheel 4 (Left-Rear) Y position offset
    // @Description: Y position of the Left-Rear wheel hub in body frame. Positive Y is to the right of the origin.
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced

    // @Param: 4_POS_Z
    // @DisplayName: Wheel 4 (Left-Rear) Z position offset
    // @Description: Z position of the Left-Rear wheel hub in body frame. Positive Z is down from the origin.
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("4_POS", 5, AP_WheelDistance_Mav, _pos_offset[3], 0.0f),

    AP_GROUPEND
};

AP_WheelDistance_Mav::AP_WheelDistance_Mav()
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_WheelDistance_Mav::handle_msg(const mavlink_message_t &msg)
{
    if (!_enable) {
        return;
    }

    mavlink_wheel_distance_t packet;
    mavlink_msg_wheel_distance_decode(&msg, &packet);

    // distance[] carries up to 16 wheels; this rover only has 4 (see
    // ARCTERON_WHEEL_DISTANCE_MAVLINK_SPEC.md - RF/LF/RR/LR, indices 0..3)
    _num_wheels = MIN(packet.count, (uint8_t)AP_WHEELDISTANCE_MAV_NUM_WHEELS);
    for (uint8_t i = 0; i < _num_wheels; i++) {
        _distance_m[i] = (float)packet.distance[i];
    }

    _last_time_usec = packet.time_usec;
    _last_update_ms = AP_HAL::millis();
}

bool AP_WheelDistance_Mav::healthy() const
{
    if (!_enable || _num_wheels == 0) {
        return false;
    }
    return (AP_HAL::millis() - _last_update_ms) < AP_WHEELDISTANCE_MAV_TIMEOUT_MS;
}

float AP_WheelDistance_Mav::get_distance(uint8_t instance) const
{
    if (instance >= AP_WHEELDISTANCE_MAV_NUM_WHEELS) {
        return 0.0f;
    }
    return _distance_m[instance];
}

const Vector3f &AP_WheelDistance_Mav::get_pos_offset(uint8_t instance) const
{
    // for invalid instances return zero vector
    if (instance >= AP_WHEELDISTANCE_MAV_NUM_WHEELS) {
        return _pos_offset_zero;
    }
    return _pos_offset[instance];
}

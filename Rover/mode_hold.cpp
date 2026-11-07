#include "Rover.h"

void ModeHold::update()
{
    float throttle = 0.0f;
    float steering = 0.0f;

    // if vehicle is balance bot, calculate actual throttle required for balancing
    if (rover.is_balancebot()) {
        rover.balancebot_pitch_control(throttle);
    }
#if AR_HILLHOLDCONTROL_ENABLED
    else {
        // stop the vehicle rolling down a slope
        g2.hill_hold.update_stopped(g.speed_cruise, g.throttle_cruise * 0.01f, throttle);
        g2.hill_hold.update_steering(0.0f, steering);
    }
#endif

    // relax mainsail
    g2.sailboat.relax_sails();

    // hold position - stop motors and center steering (unless hill hold is holding the heading)
    g2.motors.set_throttle(throttle);
    g2.motors.set_steering(steering * 4500.0f);
}

#include "Copter.h"
#include <AP_Motors/AP_Motors_Class.h>
#include <GCS_MAVLink/GCS.h>
#include "mode.h"
// #include "tilt_control.h"
// #include <AP_HAL/AP_HAL.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Logger/AP_Logger.h>
void Copter::tilt_servo_setpoint()
{

    if (copter.flightmode->mode_number() != Mode::Number::GEOMETRIC)
    {
        copter._tilt_setpoint = 1500;
    }

    static uint16_t PWM5;
    static uint16_t PWM6;

    PWM5 = copter._tilt_setpoint;
    PWM6 = copter._tilt_setpoint;
    // uint16_t PWM5=1145;
    // uint16_t PWM6=1150;
    motors->rc_write(4, PWM5);
    motors->rc_write(5, PWM6);

    AP::logger().Write("TILT", "TimeUS,m5,m6", "QHH",
                       AP_HAL::micros64(),
                       PWM5,
                       PWM6);
}

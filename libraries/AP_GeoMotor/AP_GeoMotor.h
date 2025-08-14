#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Motors/AP_Motors_Class.h> // for sending motor speed
#include <AP_Math/AP_Math.h>

#ifndef GEO_PWM_OUT_MIN
#define GEO_PWM_OUT_MIN 1
#endif

VectorN<float, 4> motorMixSimple(VectorN<float, 4> thrustMomentCmd, int8_t is_real);

void GeoMotorOutput(VectorN<float, 4> motot_pwm,int8_t motorEnable);


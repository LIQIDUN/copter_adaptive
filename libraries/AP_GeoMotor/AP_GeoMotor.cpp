#include "AP_GeoMotor.h"

VectorN<float, 4> motorMixSimple(VectorN<float, 4> thrustMomentCmd, int8_t is_real)
{
    VectorN<float, 4> motor_pwm;
    float D;
    float a_F;
    float b_F;
    if (!is_real) // SITL
    {             // const float L = 0.25; // for x layout
        D = 0.25;
        a_F = 0.0014597;
        b_F = 0.043693;
        // const float a_M = 0.000011667;
        // const float b_M = 0.0059137;
    }
    else if (is_real) // parameters for real drone
    {
        // const float L = 0.25; // longer distance between adjacent motors
        D = 0.25; // shorter distance between adjacent motors

        // 完整曲线的参数
        // const float a_F = 0.000361;
        // const float b_F = 0.067732;

        // 前半段电机曲线的参数
        a_F = 0.001021;
        b_F = 0.036329;
        // const float a_M = 0.00000503;
        // const float b_M = 0.00007975;
    }
    const float ct = 0.0480; // Genfen 5045-3

    // float thrust_total = thrustMomentCmd[0];
    // float M1 = thrustMomentCmd[1];
    // float M2 = thrustMomentCmd[2];
    // float M3 = thrustMomentCmd[3];

    VectorN<float, 4> motor_force;
    VectorN<float, 4> quad_output_mat_fm2f1;
    VectorN<float, 4> quad_output_mat_fm2f2;
    VectorN<float, 4> quad_output_mat_fm2f3;
    VectorN<float, 4> quad_output_mat_fm2f4;

    // quad X
    quad_output_mat_fm2f1[0] = 0.25;
    quad_output_mat_fm2f1[1] = 0.25;
    quad_output_mat_fm2f1[2] = 0.25;
    quad_output_mat_fm2f1[3] = 0.25;

    quad_output_mat_fm2f2[0] = -0.5 / D;
    quad_output_mat_fm2f2[1] = 0.5 / D;
    quad_output_mat_fm2f2[2] = 0.5 / D;
    quad_output_mat_fm2f2[3] = -0.5 / D;

    quad_output_mat_fm2f3[0] = 0.5 / D;
    quad_output_mat_fm2f3[1] = -0.5 / D;
    quad_output_mat_fm2f3[2] = 0.5 / D;
    quad_output_mat_fm2f3[3] = -0.5 / D;

    quad_output_mat_fm2f4[0] = 0.25 / ct;
    quad_output_mat_fm2f4[1] = 0.25 / ct;
    quad_output_mat_fm2f4[2] = -0.25 / ct;
    quad_output_mat_fm2f4[3] = -0.25 / ct;

    motor_force[0] = quad_output_mat_fm2f1[0] * thrustMomentCmd[0] + quad_output_mat_fm2f2[0] * thrustMomentCmd[1] + quad_output_mat_fm2f3[0] * thrustMomentCmd[2] + quad_output_mat_fm2f4[0] * thrustMomentCmd[3];
    motor_force[1] = quad_output_mat_fm2f1[1] * thrustMomentCmd[0] + quad_output_mat_fm2f2[1] * thrustMomentCmd[1] + quad_output_mat_fm2f3[1] * thrustMomentCmd[2] + quad_output_mat_fm2f4[1] * thrustMomentCmd[3];
    motor_force[2] = quad_output_mat_fm2f1[2] * thrustMomentCmd[0] + quad_output_mat_fm2f2[2] * thrustMomentCmd[1] + quad_output_mat_fm2f3[2] * thrustMomentCmd[2] + quad_output_mat_fm2f4[2] * thrustMomentCmd[3];
    motor_force[3] = quad_output_mat_fm2f1[3] * thrustMomentCmd[0] + quad_output_mat_fm2f2[3] * thrustMomentCmd[1] + quad_output_mat_fm2f3[3] * thrustMomentCmd[2] + quad_output_mat_fm2f4[3] * thrustMomentCmd[3];
    // solve for linearizing point
    // float w0 = (-b_F + sqrtF(b_F * b_F + a_F * thrustMomentCmd[0])) / 2 / a_F;
    motor_pwm[0] = 0;
    motor_pwm[1] = 0;
    motor_pwm[2] = 0;
    motor_pwm[3] = 0;
    if (motor_force[0] > 0)
    {
        motor_pwm[0] = (-b_F + sqrtF(b_F * b_F + a_F * motor_force[0] * 4)) / 2 / a_F;
    }
    if (motor_force[1] > 0)
    {
        motor_pwm[1] = (-b_F + sqrtF(b_F * b_F + a_F * motor_force[1] * 4)) / 2 / a_F;
    }

    if (motor_force[2] > 0)
    {
        motor_pwm[2] = (-b_F + sqrtF(b_F * b_F + a_F * motor_force[2] * 4)) / 2 / a_F;
    }
    if (motor_force[3] > 0)
    {
        motor_pwm[3] = (-b_F + sqrtF(b_F * b_F + a_F * motor_force[3] * 4)) / 2 / a_F;
    }

    // motorPWM saturation
    if (motor_pwm[0] < GEO_PWM_OUT_MIN)
    {
        motor_pwm[0] = GEO_PWM_OUT_MIN;
    }
    else if (motor_pwm[0] > 100)
    {
        motor_pwm[0] = 100;
    }
    if (motor_pwm[1] < GEO_PWM_OUT_MIN)
    {
        motor_pwm[1] = GEO_PWM_OUT_MIN;
    }
    else if (motor_pwm[1] > 100)
    {
        motor_pwm[1] = 100;
    }
    if (motor_pwm[2] < GEO_PWM_OUT_MIN)
    {
        motor_pwm[2] = GEO_PWM_OUT_MIN;
    }
    else if (motor_pwm[2] > 100)
    {
        motor_pwm[2] = 100;
    }
    if (motor_pwm[3] < GEO_PWM_OUT_MIN)
    {
        motor_pwm[3] = GEO_PWM_OUT_MIN;
    }
    else if (motor_pwm[3] > 100)
    {
        motor_pwm[3] = 100;
    }

    AP::logger().Write("MOMX", "TimeUS,mf1,mf2,mf3,mf4,mp1,mp2,mp3,mp4", "Qffffffff",
                       AP_HAL::micros64(),
                       (motor_force[0]),
                       (motor_force[1]),
                       (motor_force[2]),
                       (motor_force[3]),
                       (motor_pwm[0]),
                       (motor_pwm[1]),
                       (motor_pwm[2]),
                       (motor_pwm[3]));

    return motor_pwm;
}

void GeoMotorOutput(VectorN<float, 4> motot_pwm,int8_t motorEnable){


}
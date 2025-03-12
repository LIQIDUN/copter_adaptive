#include "Copter.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Motors/AP_Motors_Class.h> // for sending motor speed
#include "mode.h"
#include "Geometric_Trajectory_Generate.h"
#include <AP_Math/AP_Math.h>
/*
 * Init and run calls for Geometric flight mode
 */

// Geometric_run - runs the main Geometric controller
// should be called at 100hz or more
void ModeGeometric::run()
{

    if (!motors->armed())
    {
        // Motors should be Stopped
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    }
    else if (copter.ap.throttle_zero || (copter.air_mode == AirMode::AIRMODE_ENABLED && motors->get_spool_state() == AP_Motors::SpoolState::SHUT_DOWN))
    {
        // throttle_zero is never true in air mode, but the motors should be allowed to go through ground idle
        // in order to facilitate the spoolup block

        // Attempting to Land
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    }
    else
    {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    switch (motors->get_spool_state())
    {
    case AP_Motors::SpoolState::SHUT_DOWN:
        // Motors Stopped
        // attitude_control->reset_yaw_target_and_rate();
        // attitude_control->reset_rate_controller_I_terms();
        break;

    case AP_Motors::SpoolState::GROUND_IDLE:
        // Landed
        // attitude_control->reset_yaw_target_and_rate();
        // attitude_control->reset_rate_controller_I_terms_smoothly();
        break;

    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        // clear landing flag above zero throttle
        if (!motors->limit.throttle_lower)
        {
            set_land_complete(false);
        }
        break;

    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // do nothing
        break;
    }

    // start of the Geometric  controller

    VectorN<float, 4> thrustAndMomentCmd;

    // 1.测试:  只进行姿态控制,保持水平姿态，假设角速度0

    /**/
    // Matrix3f target_attitude = JoyStickToTargetAttitude();
    // thrustAndMomentCmd=ModeGeometric::geometricAttitudeController(target_attitude);
    /**/

    // 2.使用生成的轨迹
    /**/
    static uint32_t initial_time_in_geometric = 0;
    static uint32_t last_time_in_geometric = 0;
    static Vector3f enterpos;    // 进入跟踪模式的位置
    static Vector3f TRJstartpos; // 完成初始加速，开始跟踪的位置
    static int8_t getposAvailable = 0;
    static int8_t trajectory_num = 0;
    uint32_t now_time_in_geometric = AP_HAL::micros();

    if (initial_time_in_geometric == 0 || 0.000001f * (now_time_in_geometric - last_time_in_geometric) > 0.1)
    { // first or reenter this mode
        initial_time_in_geometric = AP_HAL::micros();
        getposAvailable = ahrs.get_relative_position_NED_origin(enterpos);

        // uint8_t rc6value = rc().channel(CH_6)->percent_input();
        // if(rc6value<=40){
        //     trajectory_num = 1;
        //     gcs().send_text(MAV_SEVERITY_CRITICAL, "POS");
        // }
        // if(rc6value>40 && rc6value<60){
        //     trajectory_num = 2;
        //     gcs().send_text(MAV_SEVERITY_CRITICAL, "EIGHT");
        // }
        // if(rc6value>=60){
        //     trajectory_num = 3;
        //     gcs().send_text(MAV_SEVERITY_CRITICAL, "CIRCLE");
        // }
        trajectory_num = g.GeoCtrl_NUM;
        gcs().send_text(MAV_SEVERITY_INFO, "Init of Trajectory");
    }

    float timeInThisRun = (float)0.000001f * (now_time_in_geometric - initial_time_in_geometric);

    Vector3f targetPos;
    Vector3f targetVel;
    Vector3f targetAcc;
    Vector3f targetJerk;
    Vector3f targetSnap;
    Vector2f targetYaw;
    Vector2f targetYaw_dot;
    Vector2f targetYaw_ddot;

    Vector3f error_w;
    Vector3f error_R;
    Vector3f error_x;
    Vector3f error_v;

    // exit, switch parm ,enter again
    switch (trajectory_num)
    {
    case 0:
        // POS
        Trajectory_Generate_POS(&targetPos, &targetVel, &targetAcc, &targetJerk, &targetSnap, &targetYaw, &targetYaw_dot, &targetYaw_ddot);

        break;
    case 1:
        Trajectory_Generate_TAKEOFF_AUTO(timeInThisRun, &targetPos, &targetVel, &targetAcc, &targetJerk, &targetSnap, &targetYaw, &targetYaw_dot, &targetYaw_ddot);

        break;

    case 2:
        // circle
        {
            float r_circle = g.GeoCtrl_RDI;
            float T_circle = g.GeoCtrl_TIM;
            float v_circle = 2 * M_PI * r_circle / T_circle;
            const float acc_start = 0.5; // 加速度
            float acc_time = v_circle / acc_start;
            if (timeInThisRun <= acc_time)
            {
                Trajectory_Generate_START(timeInThisRun, &targetPos, &targetVel, &targetAcc, &targetJerk, &targetSnap, &targetYaw, &targetYaw_dot, &targetYaw_ddot);
                TRJstartpos = targetPos;
            }
            else if (timeInThisRun > acc_time && timeInThisRun <= acc_time + T_circle)
            {
                Trajectory_Generate_CIRCLE(timeInThisRun - acc_time, r_circle, T_circle, &targetPos, &targetVel, &targetAcc, &targetJerk, &targetSnap, &targetYaw, &targetYaw_dot, &targetYaw_ddot);
                targetPos = targetPos + TRJstartpos;
            }
            else if (timeInThisRun > acc_time + T_circle && timeInThisRun <= acc_time + T_circle + acc_time)
            {
                Trajectory_Generate_EXIT(timeInThisRun - acc_time - T_circle, v_circle, &targetPos, &targetVel, &targetAcc, &targetJerk, &targetSnap, &targetYaw, &targetYaw_dot, &targetYaw_ddot);
                targetPos = targetPos + TRJstartpos;
            }
            else
            {
                Trajectory_Generate_POS(&targetPos, &targetVel, &targetAcc, &targetJerk, &targetSnap, &targetYaw, &targetYaw_dot, &targetYaw_ddot);
                targetPos = targetPos + TRJstartpos + TRJstartpos;
            }
        }

        break;

    case 3:
        // 8
        {
            float r_circle = g.GeoCtrl_RDI;
            float T_circle = g.GeoCtrl_TIM;
            float v_circle = 2 * M_PI * r_circle / T_circle;
            const float acc_start = 0.5; // 加速度
            float acc_time = v_circle / acc_start;
            if (timeInThisRun <= acc_time)
            {
                Trajectory_Generate_START(timeInThisRun, &targetPos, &targetVel, &targetAcc, &targetJerk, &targetSnap, &targetYaw, &targetYaw_dot, &targetYaw_ddot);
                TRJstartpos = targetPos;
            }
            else if (timeInThisRun > acc_time && timeInThisRun <= acc_time + T_circle + T_circle)
            {
                Trajectory_Generate_EIGHT(timeInThisRun - acc_time, r_circle, T_circle, &targetPos, &targetVel, &targetAcc, &targetJerk, &targetSnap, &targetYaw, &targetYaw_dot, &targetYaw_ddot);
                targetPos = targetPos + TRJstartpos;
            }
            else if (timeInThisRun > acc_time + T_circle + T_circle && timeInThisRun <= acc_time + T_circle + T_circle + acc_time)
            {
                Trajectory_Generate_EXIT(timeInThisRun - acc_time - T_circle - T_circle, v_circle, &targetPos, &targetVel, &targetAcc, &targetJerk, &targetSnap, &targetYaw, &targetYaw_dot, &targetYaw_ddot);
                targetPos = targetPos + TRJstartpos;
            }
            else
            {
                Trajectory_Generate_POS(&targetPos, &targetVel, &targetAcc, &targetJerk, &targetSnap, &targetYaw, &targetYaw_dot, &targetYaw_ddot);
                targetPos = targetPos + TRJstartpos + TRJstartpos;
            }
        }

        break;
    case 4:
        Trajectory_Generate_LINE(timeInThisRun, &targetPos, &targetVel, &targetAcc, &targetJerk, &targetSnap, &targetYaw, &targetYaw_dot, &targetYaw_ddot);
        break;

    default:
        Trajectory_Generate_POS(&targetPos, &targetVel, &targetAcc, &targetJerk, &targetSnap, &targetYaw, &targetYaw_dot, &targetYaw_ddot);
        gcs().send_text(MAV_SEVERITY_CRITICAL, "VOID Trajectory NUM");
        break;
    }

    // Trajectory_Generate_SINWAVE(timeInThisRun, &targetPos, &targetVel, &targetAcc, &targetJerk, &targetSnap, &targetYaw, &targetYaw_dot, &targetYaw_ddot);
    // Trajectory_Generate_BIGSINWAVE(timeInThisRun, &targetPos, &targetVel, &targetAcc, &targetJerk, &targetSnap, &targetYaw, &targetYaw_dot, &targetYaw_ddot);
    // Trajectory_Generate_CIRCLE(timeInThisRun, &targetPos, &targetVel, &targetAcc, &targetJerk, &targetSnap, &targetYaw, &targetYaw_dot, &targetYaw_ddot);

    // Trajectory_Generate_POS(&targetPos, &targetVel, &targetAcc, &targetJerk, &targetSnap, &targetYaw, &targetYaw_dot, &targetYaw_ddot);

    // start the trajectory tracking from current pos ,instead of the origin
    if (getposAvailable)
    {
        targetPos = targetPos + enterpos;
    }

    thrustAndMomentCmd = ModeGeometric::GeometricTrajectoryController(targetPos, targetVel, targetAcc, targetJerk, targetSnap, targetYaw, targetYaw_dot, targetYaw_ddot, &error_w, &error_R, &error_x, &error_v);
    /**/

    // end

    VectorN<float, 4> adaptiveTerm;
    if (g.GeoCtrl_ADP == 0)
    {
        // 关闭自适应
        adaptiveTerm[0] = 0;
        adaptiveTerm[1] = 0;
        adaptiveTerm[2] = 0;
        adaptiveTerm[3] = 0;
    }
    else if (g.GeoCtrl_ADP == 1)
    {
        // 打开自适应
        adaptiveTerm = ModeGeometric::AdaptiveController(error_w, error_R, error_x, error_v);

        // 测试，加入扰动
        // thrustAndMomentCmd[0] = thrustAndMomentCmd[0] + 1;
        // thrustAndMomentCmd[1] = thrustAndMomentCmd[1] + 0.05;
    }
    else
    {
        // 关闭自适应
        adaptiveTerm[0] = 0;
        adaptiveTerm[1] = 0;
        adaptiveTerm[2] = 0;
        adaptiveTerm[3] = 0;
    }

    // motor mixing
    VectorN<float, 4> motorPWM;
    if (g.GeoCtrl_MIX == 0)
    {
        motorPWM = motorMixSimple(thrustAndMomentCmd + adaptiveTerm);
        motorPWM = motorMixing(thrustAndMomentCmd + adaptiveTerm);
    }
    else
    {
        motorPWM = motorMixing(thrustAndMomentCmd + adaptiveTerm);
        motorPWM = motorMixSimple(thrustAndMomentCmd + adaptiveTerm);
    }
    // motorPWM saturation

    #ifndef GEO_PWM_OUT_MIN
    #define GEO_PWM_OUT_MIN 1
    #endif
    if (motorPWM[0] < GEO_PWM_OUT_MIN)
    {
        motorPWM[0] = GEO_PWM_OUT_MIN;
    }
    else if (motorPWM[0] > 100)
    {
        motorPWM[0] = 100;
    }
    if (motorPWM[1] < GEO_PWM_OUT_MIN)
    {
        motorPWM[1] = GEO_PWM_OUT_MIN;
    }
    else if (motorPWM[1] > 100)
    {
        motorPWM[1] = 100;
    }
    if (motorPWM[2] < GEO_PWM_OUT_MIN)
    {
        motorPWM[2] = GEO_PWM_OUT_MIN;
    }
    else if (motorPWM[2] > 100)
    {
        motorPWM[2] = 100;
    }
    if (motorPWM[3] < GEO_PWM_OUT_MIN)
    {
        motorPWM[3] = GEO_PWM_OUT_MIN;
    }
    else if (motorPWM[3] > 100)
    {
        motorPWM[3] = 100;
    }

    // disarm the vehicle by setting PWM to 1 when landing is completed

    int8_t motorEnable = 1;
    if (motors->armed()) // only command the motor PWM when the vehicle is armed.
    {
        motors->rc_write(0, 1000 + motorEnable * 10 * motorPWM[0]); // manual set motor speed: PWM_MIN/MAX has been forced to 1000/2000
        motors->rc_write(1, 1000 + motorEnable * 10 * motorPWM[1]); // rc_write is called from <AP_Motors/AP_Motors_Class.h>
        motors->rc_write(2, 1000 + motorEnable * 10 * motorPWM[2]);
        motors->rc_write(3, 1000 + motorEnable * 10 * motorPWM[3]);

        // motors->rc_write(0,motorEnable *1500);
        // motors->rc_write(1,motorEnable *1500);
        // motors->rc_write(2,motorEnable *1500);
        // motors->rc_write(3,motorEnable *1500);

        // if (timeInThisRun < 5)
        // {
        //     motors->rc_write(0, motorEnable * 1500);
        //     motors->rc_write(1, motorEnable * 1500);
        //     motors->rc_write(2, motorEnable * 1500);
        //     motors->rc_write(3, motorEnable * 1500);
        // }
        // else if (timeInThisRun < 10)
        // {
        //     motors->rc_write(0, motorEnable * 0);
        //     motors->rc_write(1, motorEnable * 0);
        //     motors->rc_write(2, motorEnable * 0);
        //     motors->rc_write(3, motorEnable * 0);
        // }
        // else if (timeInThisRun < 15)
        // {
        //     motors->rc_write(0, motorEnable * 1200);
        //     motors->rc_write(1, motorEnable * 1200);
        //     motors->rc_write(2, motorEnable * 1200);
        //     motors->rc_write(3, motorEnable * 1200);
        // }
        // else if (timeInThisRun < 20)
        // {
        //     motors->rc_write(0, motorEnable * 0);
        //     motors->rc_write(1, motorEnable * 0);
        //     motors->rc_write(2, motorEnable * 0);
        //     motors->rc_write(3, motorEnable * 0);
        // }
        // else{
        //     motors->rc_write(0, motorEnable * 1100);
        //     motors->rc_write(1, motorEnable * 1100);
        //     motors->rc_write(2, motorEnable * 1100);
        //     motors->rc_write(3, motorEnable * 1100);
        // }
    }
    else
    {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Vehicle not armed.");
        motorEnable = 0; // if the vehicle is not armed, disable the flight.
    }
    // logging
    AP::logger().Write("MOUT", "TimeUS,m1,m2,m3,m4", "Qffff",
                       AP_HAL::micros64(),
                       motorPWM[0],
                       motorPWM[1],
                       motorPWM[2],
                       motorPWM[3]);
    last_time_in_geometric = now_time_in_geometric;
}

VectorN<float, 4> ModeGeometric::geometricAttitudeController(Matrix3f target_attitude)
{
    Vector3f r_error;
    Vector3f v_error;
    Vector3f target_force;
    Vector3f z_axis;
    Vector3f x_axis_desired;
    Vector3f y_axis_desired;
    Vector3f x_c_des;
    Vector3f eR, ew, M;
    // Vector3f e3 = {0, 0, 1};

    Vector3f targetAcc = {0, 0, 0};

    Vector3f statePos;
    Vector3f stateVel;

    Vector2f positionNE;

    int locAvailable = ahrs.get_relative_position_NED_origin(statePos);
    if (!locAvailable)
    {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "location unavailable.");
    }

    // Ground velocity in meters/second, North/East/Down
    // order. Check if have_inertial_nav() is true before assigning values to stateVel.
    if (ahrs.have_inertial_nav())
    {
        if (ahrs.get_velocity_NED(stateVel))
        {
            ;
        }
    }
    else
    {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "inertial navigation is inactive");
    }

    // Position Error (ep)
    // r_error = statePos - targetPos;

    // Velocity Error (ev)
    v_error = stateVel;

    // Target force
    target_force.x = kg_vehicleMass * targetAcc.x - g.GeoCtrl_Kvx * v_error.x;
    target_force.y = kg_vehicleMass * targetAcc.y - g.GeoCtrl_Kvy * v_error.y;
    target_force.z = kg_vehicleMass * (targetAcc.z - GRAVITY_MAGNITUDE) - g.GeoCtrl_Kvz * v_error.z;

    // Z-Axis [zB]
    Quaternion q;
    ahrs.get_quat_body_to_ned(q);

    Matrix3f R;
    q.rotation_matrix(R); // transforming the quaternion q to rotation matrix R

    z_axis = R.colz();

    // target thrust [F]
    float target_thrust = -target_force * z_axis;

    // Calculate axis [zB_des]
    Vector3f z_axis_desired = -target_force;
    z_axis_desired.normalize();

    // [eR]
    Matrix3f Rdes = target_attitude;

    Matrix3f eRM = (Rdes.transposed() * R - R.transposed() * Rdes) / 2;
    eR = veeOperator(eRM);

    Vector3f Omega = AP::ahrs().get_gyro();

    // compute Omegad: this comes from Appendix F in https://arxiv.org/pdf/1003.2005v3.pdf

    Vector3f Omegad = {0, 0, 0};
    Vector3f Omegad_dot = veeOperator(-hatOperator(Omegad) * hatOperator(Omegad));

    // eomega (angular velocity error)
    ew = Omega - R.transposed() * Rdes * Omegad;

    // Compute the moment
    M.x = -g.GeoCtrl_KRx * eR.x - g.GeoCtrl_KOx * ew.x;
    M.y = -g.GeoCtrl_KRy * eR.y - g.GeoCtrl_KOy * ew.y;
    M.z = -g.GeoCtrl_KRz * eR.z - g.GeoCtrl_KOz * ew.z;
    M = M - J * (hatOperator(Omega) * R.transposed() * Rdes * Omegad - R.transposed() * Rdes * Omegad_dot);
    Vector3f momentAdd = Omega % (J * Omega); // J is the inertia matrix
    M = M + momentAdd;

    VectorN<float, 4> thrustMomentCmd;
    thrustMomentCmd[0] = target_thrust;
    thrustMomentCmd[1] = M.x;
    thrustMomentCmd[2] = M.y;
    thrustMomentCmd[3] = M.z;

    // logging
    // log the desired rotation matrix and the actual rotation matrix
    AP::logger().Write("L1AF", "Rd11,Rd12,Rd13,Rd21,Rd22,Rd23,Rd31,Rd32,Rd33", "fffffffff",
                       Rdes.a.x,
                       Rdes.a.y,
                       Rdes.a.z,
                       Rdes.b.x,
                       Rdes.b.y,
                       Rdes.b.z,
                       Rdes.c.x,
                       Rdes.c.y,
                       Rdes.c.z);
    AP::logger().Write("L1AG", "R11,R12,R13,R21,R22,R23,R31,R32,R33", "fffffffff",
                       R.a.x,
                       R.a.y,
                       R.a.z,
                       R.b.x,
                       R.b.y,
                       R.b.z,
                       R.c.x,
                       R.c.y,
                       R.c.z);

    return thrustMomentCmd;
}

VectorN<float, 4> ModeGeometric::GeometricTrajectoryController(
    Vector3f targetPos,
    Vector3f targetVel,
    Vector3f targetAcc,
    Vector3f targetJerk,
    Vector3f targetSnap,
    Vector2f targetYaw,
    Vector2f targetYaw_dot,
    Vector2f targetYaw_ddot,
    Vector3f *error_w,
    Vector3f *error_R,
    Vector3f *error_x,
    Vector3f *error_v)
{
    Vector3f r_error;
    Vector3f v_error;
    Vector3f target_force;
    Vector3f z_axis;
    Vector3f x_axis_desired;
    Vector3f y_axis_desired;
    Vector3f x_c_des;
    Vector3f eR, ew, M;
    Vector3f e3 = {0, 0, 1};

    Vector3f statePos;
    Vector3f stateVel;

    Vector2f positionNE;

    int locAvailable = ahrs.get_relative_position_NED_origin(statePos);
    if (!locAvailable)
    {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "location unavailable.");
    }

    // Ground velocity in meters/second, North/East/Down
    // order. Check if have_inertial_nav() is true before assigning values to stateVel.
    if (ahrs.have_inertial_nav())
    {
        if (ahrs.get_velocity_NED(stateVel))
        {
            // gcs().send_text(MAV_SEVERITY_INFO, "Vel available.");
        }
    }
    else
    {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "inertial navigation is inactive");
    }

    // Position Error (ep)
    r_error = statePos - targetPos;
    *error_x = r_error;

    // Velocity Error (ev)
    v_error = stateVel - targetVel;
    *error_v = v_error;

    // kg_vehicleMass = g.GeoCtrl_MAS; // weight for the real drone

    // Target force
    target_force.x = kg_vehicleMass * targetAcc.x - g.GeoCtrl_Kpx * r_error.x - g.GeoCtrl_Kvx * v_error.x;
    target_force.y = kg_vehicleMass * targetAcc.y - g.GeoCtrl_Kpy * r_error.y - g.GeoCtrl_Kvy * v_error.y;
    target_force.z = kg_vehicleMass * (targetAcc.z - GRAVITY_MAGNITUDE) - g.GeoCtrl_Kpz * r_error.z - g.GeoCtrl_Kvz * v_error.z;

    // Z-Axis [zB]
    Quaternion q;
    ahrs.get_quat_body_to_ned(q);

    Matrix3f R;
    q.rotation_matrix(R); // transforming the quaternion q to rotation matrix R

    z_axis = R.colz();

    // target thrust [F]
    float target_thrust = -target_force * z_axis;

    // Calculate axis [zB_des]
    Vector3f z_axis_desired = -target_force;
    z_axis_desired.normalize();

    // [xC_des]
    // x_axis_desired = z_axis_desired x [cos(yaw), sin(yaw), 0]^T
    x_c_des[0] = targetYaw[0]; // x
    x_c_des[1] = targetYaw[1]; // y
    x_c_des[2] = 0;            // z

    Vector3f x_c_des_dot = {targetYaw_dot, 0};   // time derivative of x_c_des
    Vector3f x_c_des_ddot = {targetYaw_ddot, 0}; // time derivative of x_c_des_dot

    // [yB_des]
    y_axis_desired = (z_axis_desired % x_c_des);
    y_axis_desired.normalize();
    // [xB_des]
    x_axis_desired = y_axis_desired % z_axis_desired;

    // [eR]
    Matrix3f Rdes(Vector3f(x_axis_desired.x, y_axis_desired.x, z_axis_desired.x),
                  Vector3f(x_axis_desired.y, y_axis_desired.y, z_axis_desired.y),
                  Vector3f(x_axis_desired.z, y_axis_desired.z, z_axis_desired.z));

    Matrix3f eRM = (Rdes.transposed() * R - R.transposed() * Rdes) / 2;
    eR = veeOperator(eRM);
    *error_R = eR;

    Vector3f Omega = AP::ahrs().get_gyro();

    // compute Omegad: this comes from Appendix F in https://arxiv.org/pdf/1003.2005v3.pdf
    Vector3f a_error; // error on acceleration
    a_error = e3 * GRAVITY_MAGNITUDE - R.colz() * target_thrust / kg_vehicleMass - targetAcc;

    Vector3f target_force_dot; // derivative of target_force
    target_force_dot.x = -g.GeoCtrl_Kpx * v_error.x - g.GeoCtrl_Kvx * a_error.x + kg_vehicleMass * targetJerk.x;
    target_force_dot.y = -g.GeoCtrl_Kpy * v_error.y - g.GeoCtrl_Kvy * a_error.y + kg_vehicleMass * targetJerk.y;
    target_force_dot.z = -g.GeoCtrl_Kpz * v_error.z - g.GeoCtrl_Kvz * a_error.z + kg_vehicleMass * targetJerk.z;

    Vector3f b3_dot = R * hatOperator(Omega) * e3;

    float target_thrust_dot = -target_force_dot * R.colz() - target_force * b3_dot;

    Vector3f j_error; // error on jerk
    j_error = -R.colz() * target_thrust_dot / kg_vehicleMass - b3_dot * target_thrust / kg_vehicleMass - targetJerk;

    Vector3f target_force_ddot; // derivative of target_force_dot
    target_force_ddot.x = -g.GeoCtrl_Kpx * a_error.x - g.GeoCtrl_Kvx * j_error.x + kg_vehicleMass * targetSnap.x;
    target_force_ddot.y = -g.GeoCtrl_Kpy * a_error.y - g.GeoCtrl_Kvy * j_error.y + kg_vehicleMass * targetSnap.y;
    target_force_ddot.z = -g.GeoCtrl_Kpz * a_error.z - g.GeoCtrl_Kvz * j_error.z + kg_vehicleMass * targetSnap.z;

    VectorN<float, 9> b3cCollection;                                                // collection of three three-dimensional vectors b3c, b3c_dot, b3c_ddot
    b3cCollection = unit_vec(-target_force, -target_force_dot, -target_force_ddot); // unit_vec function is from geometric controller's git repo: https://github.com/fdcl-gwu/uav_geometric_control/blob/master/matlab/aux_functions/deriv_unit_vector.m

    Vector3f b3c;
    Vector3f b3c_dot;
    Vector3f b3c_ddot;

    b3c[0] = b3cCollection[0];
    b3c[1] = b3cCollection[1];
    b3c[2] = b3cCollection[2];

    b3c_dot[0] = b3cCollection[3];
    b3c_dot[1] = b3cCollection[4];
    b3c_dot[2] = b3cCollection[5];

    b3c_ddot[0] = b3cCollection[6];
    b3c_ddot[1] = b3cCollection[7];
    b3c_ddot[2] = b3cCollection[8];

    Vector3f A2 = -hatOperator(x_c_des) * b3c;
    Vector3f A2_dot = -hatOperator(x_c_des_dot) * b3c - hatOperator(x_c_des) * b3c_dot;
    Vector3f A2_ddot = -hatOperator(x_c_des_ddot) * b3c - hatOperator(x_c_des_dot) * b3c_dot * 2 - hatOperator(x_c_des) * b3c_ddot;

    VectorN<float, 9> b2cCollection;               // collection of three three-dimensional vectors b2c, b2c_dot, b2c_ddot
    b2cCollection = unit_vec(A2, A2_dot, A2_ddot); // unit_vec function is from geometric controller's git repo: https://github.com/fdcl-gwu/uav_geometric_control/blob/master/matlab/aux_functions/deriv_unit_vector.m

    Vector3f b2c;
    Vector3f b2c_dot;
    Vector3f b2c_ddot;

    b2c[0] = b2cCollection[0];
    b2c[1] = b2cCollection[1];
    b2c[2] = b2cCollection[2];

    b2c_dot[0] = b2cCollection[3];
    b2c_dot[1] = b2cCollection[4];
    b2c_dot[2] = b2cCollection[5];

    b2c_ddot[0] = b2cCollection[6];
    b2c_ddot[1] = b2cCollection[7];
    b2c_ddot[2] = b2cCollection[8];

    Vector3f b1c_dot = hatOperator(b2c_dot) * b3c + hatOperator(b2c) * b3c_dot;
    Vector3f b1c_ddot = hatOperator(b2c_ddot) * b3c + hatOperator(b2c_dot) * b3c_dot * 2 + hatOperator(b2c) * b3c_ddot;

    Matrix3f Rd_dot;  // derivative of Rdes
    Matrix3f Rd_ddot; // derivative of Rd_dot

    Rd_dot.a = b1c_dot;
    Rd_dot.b = b2c_dot;
    Rd_dot.c = b3c_dot;
    Rd_dot.transpose();

    Rd_ddot.a = b1c_ddot;
    Rd_ddot.b = b2c_ddot;
    Rd_ddot.c = b3c_ddot;
    Rd_ddot.transpose();

    Vector3f Omegad = veeOperator(Rdes.transposed() * Rd_dot);
    Vector3f Omegad_dot = veeOperator(Rdes.transposed() * Rd_ddot - hatOperator(Omegad) * hatOperator(Omegad));

    // eomega (angular velocity error)
    ew = Omega - R.transposed() * Rdes * Omegad;
    *error_w = ew;

    // Compute the moment
    M.x = -g.GeoCtrl_KRx * eR.x - g.GeoCtrl_KOx * ew.x;
    M.y = -g.GeoCtrl_KRy * eR.y - g.GeoCtrl_KOy * ew.y;
    M.z = -g.GeoCtrl_KRz * eR.z - g.GeoCtrl_KOz * ew.z;
    M = M - J * (hatOperator(Omega) * R.transposed() * Rdes * Omegad - R.transposed() * Rdes * Omegad_dot);
    Vector3f momentAdd = Omega % (J * Omega); // J is the inertia matrix
    M = M + momentAdd;

    VectorN<float, 4> thrustMomentCmd;
    thrustMomentCmd[0] = target_thrust;
    thrustMomentCmd[1] = M.x;
    thrustMomentCmd[2] = M.y;
    thrustMomentCmd[3] = M.z;

    // logging
    AP::logger().Write("GEOM", "TimeUS,exx,exy,exz,evx,evy,evz,erx,ery,erz,ewx,ewy,ewz", "Qffffffffffff",
                       AP_HAL::micros64(),
                       (error_x->x),
                       (error_x->y),
                       (error_x->z),
                       (error_v->x),
                       (error_v->y),
                       (error_v->z),
                       (error_R->x),
                       (error_R->y),
                       (error_R->z),
                       (error_w->x),
                       (error_w->y),
                       (error_w->z));
    AP::logger().Write("GECT", "TimeUS,tfx,tfy,tfz,tt,mx,my,mz", "Qfffffff",
                       AP_HAL::micros64(),
                       (target_force.x),
                       (target_force.y),
                       (target_force.z),
                       (target_thrust),
                       (M.x),
                       (M.y),
                       (M.z)

    );
    AP::logger().Write("GETA", "TimeUS,tpx,tpy,tpz,spx,spy,spz", "Qffffff",
                       AP_HAL::micros64(),
                       (targetPos.x),
                       (targetPos.y),
                       (targetPos.z),
                       (statePos.x),
                       (statePos.y),
                       (statePos.z)

    );
    // log the desired rotation matrix and the actual rotation matrix
    AP::logger().Write("L1AF", "TimeUS,Rd11,Rd12,Rd13,Rd21,Rd22,Rd23,Rd31,Rd32,Rd33", "Qfffffffff",
                       AP_HAL::micros64(),
                       Rdes.a.x,
                       Rdes.a.y,
                       Rdes.a.z,
                       Rdes.b.x,
                       Rdes.b.y,
                       Rdes.b.z,
                       Rdes.c.x,
                       Rdes.c.y,
                       Rdes.c.z);
    AP::logger().Write("L1AG", "TimeUS,R11,R12,R13,R21,R22,R23,R31,R32,R33", "Qfffffffff",
                       AP_HAL::micros64(),
                       R.a.x,
                       R.a.y,
                       R.a.z,
                       R.b.x,
                       R.b.y,
                       R.b.z,
                       R.c.x,
                       R.c.y,
                       R.c.z);
    return thrustMomentCmd;
}
VectorN<float, 4> ModeGeometric::AdaptiveController(Vector3f error_w,
                                                    Vector3f error_R,
                                                    Vector3f error_x,
                                                    Vector3f error_v)
{
    static Matrix3f W_x(Vector3f(1, 0, 0), Vector3f(0, 1, 0), Vector3f(0, 0, 1));
    static Vector3f theta_x = Vector3f(0, 0, 0);

    static Matrix3f W_R(Vector3f(0.1, 0, 0), Vector3f(0, 0.1, 0), Vector3f(0, 0, 0.1));
    static Vector3f theta_R = Vector3f(0, 0, 0);

    static uint32_t now_time = 0;
    static uint32_t last_time = 0;
    float dt = 0;

    Vector3f theta_x_dot = {0, 0, 0};
    Vector3f theta_R_dot = {0, 0, 0};

    now_time = AP_HAL::micros();
    if (last_time == 0)
    {
        last_time = now_time;
    }

    dt = (float)0.000001f * (now_time - last_time);
    // gcs().send_text(MAV_SEVERITY_CRITICAL, "%.8f\n",dt);

    Vector3f ev_c1ex;
    ev_c1ex = error_v + error_x * g.GeoCtrl_C1;

    float norm_theta_x = vector_2norm(theta_x);
    Matrix3f theta_x_T(Vector3f(theta_x.x, 0, 0), Vector3f(theta_x.y, 0, 0), Vector3f(theta_x.z, 0, 0));
    Matrix3f theta_x_Mat(Vector3f(theta_x.x, theta_x.y, theta_x.z), Vector3f(0, 0, 0), Vector3f(0, 0, 0));

    if (norm_theta_x < g.GeoCtrl_BX || ((fabsf(norm_theta_x - g.GeoCtrl_BX) <= 1e-3f) && ((theta_x_T * (W_x.transposed()) * ev_c1ex).x <= 0)))
    {
        theta_x_dot = (W_x.transposed()) * ev_c1ex * g.GeoCtrl_GAX;

        // gcs().send_text(MAV_SEVERITY_CRITICAL, "case1\n");
    }
    else
    {

        Matrix3f theta_x_T_theta = (theta_x_T * theta_x_Mat);
        Matrix3f theta_x_T_theta_inv;
        if (theta_x_T_theta.inverse(theta_x_T_theta_inv))
        {
            Matrix3f eye3(Vector3f(1, 0, 0), Vector3f(0, 1, 0), Vector3f(0, 0, 1));
            Matrix3f I_theta = eye3 - (theta_x_Mat * theta_x_T) * theta_x_T_theta_inv;
            theta_x_dot = I_theta * W_x.transposed() * ev_c1ex;

            theta_x_dot.x = g.GeoCtrl_GAX * theta_x_dot.x;
            theta_x_dot.y = g.GeoCtrl_GAX * theta_x_dot.y;
            theta_x_dot.z = g.GeoCtrl_GAX * theta_x_dot.z;

            // gcs().send_text(MAV_SEVERITY_CRITICAL, "case2\n");
        }
        else
        {
            theta_x_dot.x = 0;
            theta_x_dot.y = 0;
            theta_x_dot.z = 0;

            // Matrix3f eye3(Vector3f(1,0,0),Vector3f(0,1,0),Vector3f(0,0,1));
            // Matrix3f I_theta = eye3 - (theta_x_Mat * theta_x_T) * theta_x_T_theta_inv;
            // theta_x_dot =  I_theta * W_x.transposed() * ev_c1ex;

            // theta_x_dot.x = g.GeoCtrl_GAX * theta_x_dot.x;
            // theta_x_dot.y = g.GeoCtrl_GAX * theta_x_dot.y;
            // theta_x_dot.z = g.GeoCtrl_GAX * theta_x_dot.z;

            // gcs().send_text(MAV_SEVERITY_CRITICAL, "case3\n");
        }
    }

    theta_x = theta_x + theta_x_dot * dt;
    // gcs().send_text(MAV_SEVERITY_CRITICAL, "%.8f\n", theta_x.x);
    // gcs().send_text(MAV_SEVERITY_CRITICAL, "%.8f\n", theta_x.y);
    // gcs().send_text(MAV_SEVERITY_CRITICAL, "%.8f\n", theta_x.z);

    // Z-Axis [zB]
    Quaternion q;
    ahrs.get_quat_body_to_ned(q);

    Matrix3f R;
    q.rotation_matrix(R); // transforming the quaternion q to rotation matrix R

    Vector3f z_axis;
    z_axis = R.colz();

    // adaptive thrust [F]
    float adaptive_thrust = -(-W_x * theta_x) * z_axis;
    // gcs().send_text(MAV_SEVERITY_CRITICAL, "%.8f\n",adaptive_thrust);

    // Vector3f ev_c1ex;
    // ev_c1ex.x=(error_v.x + g.GeoCtrl_C1  * error_x.x);
    // ev_c1ex.y=(error_v.y + g.GeoCtrl_C1  * error_x.y);
    // ev_c1ex.z=(error_v.z + g.GeoCtrl_C1  * error_x.z);

    Vector3f ew_c2er;
    // ew_c2er.x=(error_w.x + g.GeoCtrl_C2  * error_R.x);
    // ew_c2er.y=(error_w.y + g.GeoCtrl_C2  * error_R.y);
    // ew_c2er.z=(error_w.z + g.GeoCtrl_C2  * error_R.z);
    ew_c2er = error_w + error_R * g.GeoCtrl_C2;

    theta_R_dot = (W_R.transposed()) * ew_c2er * g.GeoCtrl_GAR;

    theta_R = theta_R + theta_R_dot * dt;

    // gcs().send_text(MAV_SEVERITY_CRITICAL, "%.8f\n", theta_R.x);
    // gcs().send_text(MAV_SEVERITY_CRITICAL, "%.8f\n", theta_R.y);
    // gcs().send_text(MAV_SEVERITY_CRITICAL, "%.8f\n", theta_R.z);

    Vector3f adaptive_Moment;
    adaptive_Moment = -W_R * theta_R;

    // gcs().send_text(MAV_SEVERITY_CRITICAL, "%.8f\n",adaptive_Moment.x);
    // gcs().send_text(MAV_SEVERITY_CRITICAL, "%.8f\n",adaptive_Moment.y);
    // gcs().send_text(MAV_SEVERITY_CRITICAL, "%.8f\n",adaptive_Moment.z);

    // gcs().send_text(MAV_SEVERITY_CRITICAL, "%.8f\n",error_x.x);
    // gcs().send_text(MAV_SEVERITY_CRITICAL, "%.8f\n",error_x.y);
    // gcs().send_text(MAV_SEVERITY_CRITICAL, "%.8f\n",error_x.z);

    VectorN<float, 4> adaptiveTermOutput;
    adaptiveTermOutput[0] = adaptive_thrust;
    adaptiveTermOutput[1] = adaptive_Moment.x;
    adaptiveTermOutput[2] = adaptive_Moment.y;
    adaptiveTermOutput[3] = adaptive_Moment.z;

    // logging
    AP::logger().Write("THET", "TimeUS,tx1,tx2,tx3,tr1,tr2,tr3,the", "Qfffffff",
                       AP_HAL::micros64(),
                       (theta_x.x),
                       (theta_x.y),
                       (theta_x.z),
                       (theta_R.x),
                       (theta_R.y),
                       (theta_R.z),
                       norm_theta_x);

    last_time = now_time;
    return adaptiveTermOutput;
}

Matrix3f ModeGeometric::hatOperator(Vector3f input)
{
    // hatOperator: convert R^3 to so(3)
    Matrix3f output;
    output = output * 0; // initialize by zero
    // const T ax, const T ay, const T az,
    // const T bx, const T by, const T bz,
    // const T cx, const T cy, const T cz
    output.a.x = 0;
    output.a.y = -input.z;
    output.a.z = input.y;
    output.b.x = input.z;
    output.b.y = 0;
    output.b.z = -input.x;
    output.c.x = -input.y;
    output.c.y = input.x;
    output.c.z = 0;

    return output;
}

Vector3f ModeGeometric::veeOperator(Matrix3f input)
{
    // veeOperator: convert so(3) to R^3
    Vector3f output;
    // const T ax, const T ay, const T az,
    // const T bx, const T by, const T bz,
    // const T cx, const T cy, const T cz
    output.x = input.c.y;
    output.y = input.a.z;
    output.z = input.b.x;

    return output;
}

VectorN<float, 4> ModeGeometric::motorMixSimple(VectorN<float, 4> thrustMomentCmd)
{
    VectorN<float, 4> motor_pwm;
#if (!REAL_OR_SITL) // SITL
    // const float L = 0.25; // for x layout
    const float D = 0.25;
    const float a_F = 0.0014597;
    const float b_F = 0.043693;
    // const float a_M = 0.000011667;
    // const float b_M = 0.0059137;
#elif (REAL_OR_SITL) // parameters for real drone
    // const float L = 0.25; // longer distance between adjacent motors
    const float D = 0.25; // shorter distance between adjacent motors

    // 完整曲线的参数
    // const float a_F = 0.000361;
    // const float b_F = 0.067732;

    // 前半段电机曲线的参数
    const float a_F = 0.001021;
    const float b_F = 0.036329;
    // const float a_M = 0.00000503;
    // const float b_M = 0.00007975;
#endif
    const float ct = 0.0480;
    // float thrust_total = thrustMomentCmd[0];
    // float M1 = thrustMomentCmd[1];
    // float M2 = thrustMomentCmd[2];
    // float M3 = thrustMomentCmd[3];

    VectorN<float, 4> motor_force;
    VectorN<float, 4> quad_output_mat_fm2f1;
    VectorN<float, 4> quad_output_mat_fm2f2;
    VectorN<float, 4> quad_output_mat_fm2f3;
    VectorN<float, 4> quad_output_mat_fm2f4;

    // quad_output_mat_fm2f1[0] = 0.25;
    // quad_output_mat_fm2f1[1] = 0.25;
    // quad_output_mat_fm2f1[2] = 0.25;
    // quad_output_mat_fm2f1[3] = 0.25;

    // quad_output_mat_fm2f2[0] = 0;
    // quad_output_mat_fm2f2[1] = -2;
    // quad_output_mat_fm2f2[2] = 0;
    // quad_output_mat_fm2f2[3] = 2;

    // quad_output_mat_fm2f3[0] = 2;
    // quad_output_mat_fm2f3[1] = 0;
    // quad_output_mat_fm2f3[2] = -2;
    // quad_output_mat_fm2f3[3] = 0;

    // quad_output_mat_fm2f4[0] = -5.20833;
    // quad_output_mat_fm2f4[1] = 5.20833;
    // quad_output_mat_fm2f4[2] = -5.20833;
    // quad_output_mat_fm2f4[3] = 5.20833;

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

    AP::logger().Write("MOMX", "TimeUS,mf1,mf2,mf3,mf4,mp1,mp2,mp3,mp4", "Qffffffff",
                       AP_HAL::micros64(),
                       (motor_force[0]),
                       (motor_force[1]),
                       (motor_force[2]),
                       (motor_force[3]),
                       (motor_pwm[0]),
                       (motor_pwm[1]),
                       (motor_pwm[2]),
                       (motor_pwm[3])

    );

    return motor_pwm;
}

VectorN<float, 4> ModeGeometric::motorMixing(VectorN<float, 4> thrustMomentCmd)
{
    VectorN<float, 4> w;
#if (!REAL_OR_SITL)       // SITL
    const float L = 0.25; // for x layout
    const float D = 0.25;
    const float a_F = 0.0014597;
    const float b_F = 0.043693;
    const float a_M = 0.000011667;
    const float b_M = 0.0059137;
#elif (REAL_OR_SITL) // parameters for real drone
    const float L = 0.25; // longer distance between adjacent motors
    const float D = 0.25; // shorter distance between adjacent motors
    // const float a_F = 0.0009251;
    // const float b_F = 0.021145;
    // const float a_M = 0.00001211;
    // const float b_M = 0.0009864;
    // const float a_F = 0.000361;
    // const float b_F = 0.067732;
    const float a_F = 0.001021;
    const float b_F = 0.036329;
    const float a_M = 0.00000503;
    const float b_M = 0.00007975;
#endif

    // solve for linearizing point
    float w0 = (-b_F + sqrtF(b_F * b_F + a_F * thrustMomentCmd[0])) / 2 / a_F;

    float c_F = 2 * a_F * w0 + b_F;
    float c_M = 2 * a_M * w0 + b_M;

    float thrust_biased = 2 * thrustMomentCmd[0] - 4 * b_F * w0;
    float M1 = thrustMomentCmd[1];
    float M2 = thrustMomentCmd[2];
    float M3 = thrustMomentCmd[3];

    // motor mixing for x layout
    const float c_F4_inv = 1 / (4 * c_F);
    const float c_FL_inv = 1 / (2 * L * c_F);
    const float c_FD_inv = 1 / (2 * D * c_F);
    const float c_M4_inv = 1 / (4 * c_M);

    w[0] = c_F4_inv * thrust_biased - c_FL_inv * M1 + c_FD_inv * M2 + c_M4_inv * M3;
    w[1] = c_F4_inv * thrust_biased + c_FL_inv * M1 - c_FD_inv * M2 + c_M4_inv * M3;
    w[2] = c_F4_inv * thrust_biased + c_FL_inv * M1 + c_FD_inv * M2 - c_M4_inv * M3;
    w[3] = c_F4_inv * thrust_biased - c_FL_inv * M1 - c_FD_inv * M2 - c_M4_inv * M3;

    // 2nd shot on solving for motor speed
    // output: VectorN<float, 4> new motor speed
    // input: a_F, b_F, a_M, b_M, w, L, D, thrustMomentCmd

    // motor speed after the second iteration
    VectorN<float, 4> w2;
    w2 = iterativeMotorMixing(w, thrustMomentCmd, a_F, b_F, a_M, b_M, L, D);

    // motor speed after the third iteration
    VectorN<float, 4> w3;
    w3 = iterativeMotorMixing(w2, thrustMomentCmd, a_F, b_F, a_M, b_M, L, D);

    // logging
    AP::logger().Write("L1A1", "TimeUS,m1,m2,m3,m4", "Qffff",
                       AP_HAL::micros64(),
                       (double)w[0],
                       (double)w[1],
                       (double)w[2],
                       (double)w[3]);
    AP::logger().Write("L1A2", "TimeUS,m1,m2,m3,m4", "Qffff",
                       AP_HAL::micros64(),
                       (double)w2[0],
                       (double)w2[1],
                       (double)w2[2],
                       (double)w2[3]);
    AP::logger().Write("L1A3", "TimeUS,m1,m2,m3,m4", "Qffff",
                       AP_HAL::micros64(),
                       (double)w3[0],
                       (double)w3[1],
                       (double)w3[2],
                       (double)w3[3]);
    return w3;
}

VectorN<float, 4> ModeGeometric::iterativeMotorMixing(VectorN<float, 4> w_input, VectorN<float, 4> thrustMomentCmd, float a_F, float b_F, float a_M, float b_M, float L, float D)
{
    // The function iterativeMotorMixing computes the motor speed to achieve the desired thrustMoment command
    // input:
    // VectorN<float, 4> w_input -- initial guess of the motor speed (linearizing point)
    // VectorN<float, 4> thrustMomentCmd -- desired thrust and moment command
    // float a_F -- 2nd-order coefficient for motor's thrust-speed curve
    // float b_F -- 1st-order coefficient for motor's thrust-speed curve
    // float a_M -- 2nd-order coefficient for motor's torque-speed curve
    // float b_M -- 1st-order coefficient for motor's torque-speed curve
    // float L -- longer distance between adjacent motors
    // float D -- shorter distance between adjacent motors

    // output:
    // VectorN<float, 4> w_new new motor speed

    VectorN<float, 4> w_new; // new motor speed

    float w1_square = w_input[0] * w_input[0];
    float w2_square = w_input[1] * w_input[1];
    float w3_square = w_input[2] * w_input[2];
    float w4_square = w_input[3] * w_input[3];

    float c_F1 = -a_F * w1_square;
    float c_F2 = -a_F * w2_square;
    float c_F3 = -a_F * w3_square;
    float c_F4 = -a_F * w4_square;

    float c_M1 = -a_M * w1_square;
    float c_M2 = -a_M * w2_square;
    float c_M3 = -a_M * w3_square;
    float c_M4 = -a_M * w4_square;

    float d_F1 = 2 * a_F * w_input[0] + b_F;
    float d_F2 = 2 * a_F * w_input[1] + b_F;
    float d_F3 = 2 * a_F * w_input[2] + b_F;
    float d_F4 = 2 * a_F * w_input[3] + b_F;

    float d_M1 = 2 * a_M * w_input[0] + b_M;
    float d_M2 = 2 * a_M * w_input[1] + b_M;
    float d_M3 = 2 * a_M * w_input[2] + b_M;
    float d_M4 = 2 * a_M * w_input[3] + b_M;

    VectorN<float, 4> coefficientRow1;
    VectorN<float, 4> coefficientRow2;
    VectorN<float, 4> coefficientRow3;
    VectorN<float, 4> coefficientRow4;

    coefficientRow1[0] = d_F1;
    coefficientRow1[1] = d_F2;
    coefficientRow1[2] = d_F3;
    coefficientRow1[3] = d_F4;

    coefficientRow2[0] = -d_F1;
    coefficientRow2[1] = d_F2;
    coefficientRow2[2] = d_F3;
    coefficientRow2[3] = -d_F4;

    coefficientRow3[0] = d_F1;
    coefficientRow3[1] = -d_F2;
    coefficientRow3[2] = d_F3;
    coefficientRow3[3] = -d_F4;

    coefficientRow4[0] = d_M1;
    coefficientRow4[1] = d_M2;
    coefficientRow4[2] = -d_M3;
    coefficientRow4[3] = -d_M4;

    VectorN<float, 16> coefficientMatrixInv = mat4Inv(coefficientRow1, coefficientRow2, coefficientRow3, coefficientRow4);

    VectorN<float, 4> coefficientInvRow1;
    VectorN<float, 4> coefficientInvRow2;
    VectorN<float, 4> coefficientInvRow3;
    VectorN<float, 4> coefficientInvRow4;

    coefficientInvRow1[0] = coefficientMatrixInv[0];
    coefficientInvRow1[1] = coefficientMatrixInv[1];
    coefficientInvRow1[2] = coefficientMatrixInv[2];
    coefficientInvRow1[3] = coefficientMatrixInv[3];

    coefficientInvRow2[0] = coefficientMatrixInv[4];
    coefficientInvRow2[1] = coefficientMatrixInv[5];
    coefficientInvRow2[2] = coefficientMatrixInv[6];
    coefficientInvRow2[3] = coefficientMatrixInv[7];

    coefficientInvRow3[0] = coefficientMatrixInv[8];
    coefficientInvRow3[1] = coefficientMatrixInv[9];
    coefficientInvRow3[2] = coefficientMatrixInv[10];
    coefficientInvRow3[3] = coefficientMatrixInv[11];

    coefficientInvRow4[0] = coefficientMatrixInv[12];
    coefficientInvRow4[1] = coefficientMatrixInv[13];
    coefficientInvRow4[2] = coefficientMatrixInv[14];
    coefficientInvRow4[3] = coefficientMatrixInv[15];

    VectorN<float, 4> shiftedCmd;
    shiftedCmd[0] = thrustMomentCmd[0] - c_F1 - c_F2 - c_F3 - c_F4;
    shiftedCmd[1] = 2 * thrustMomentCmd[1] / L + c_F1 - c_F2 - c_F3 + c_F4;
    shiftedCmd[2] = 2 * thrustMomentCmd[2] / D - c_F1 + c_F2 - c_F3 + c_F4;
    shiftedCmd[3] = thrustMomentCmd[3] - c_M1 - c_M2 + c_M3 + c_M4;

    w_new[0] = coefficientInvRow1 * shiftedCmd;
    w_new[1] = coefficientInvRow2 * shiftedCmd;
    w_new[2] = coefficientInvRow3 * shiftedCmd;
    w_new[3] = coefficientInvRow4 * shiftedCmd;

    return w_new;
}
VectorN<float, 16> ModeGeometric::mat4Inv(VectorN<float, 4> coefficientRow1, VectorN<float, 4> coefficientRow2, VectorN<float, 4> coefficientRow3, VectorN<float, 4> coefficientRow4)
{
    // inverse of a 4x4 matrix
    // modified from https://stackoverflow.com/a/44446912
    float A2323 = coefficientRow3[2] * coefficientRow4[3] - coefficientRow3[3] * coefficientRow4[2];
    float A1323 = coefficientRow3[1] * coefficientRow4[3] - coefficientRow3[3] * coefficientRow4[1];
    float A1223 = coefficientRow3[1] * coefficientRow4[2] - coefficientRow3[2] * coefficientRow4[1];
    float A0323 = coefficientRow3[0] * coefficientRow4[3] - coefficientRow3[3] * coefficientRow4[0];
    float A0223 = coefficientRow3[0] * coefficientRow4[2] - coefficientRow3[2] * coefficientRow4[0];
    float A0123 = coefficientRow3[0] * coefficientRow4[1] - coefficientRow3[1] * coefficientRow4[0];
    float A2313 = coefficientRow2[2] * coefficientRow4[3] - coefficientRow2[3] * coefficientRow4[2];
    float A1313 = coefficientRow2[1] * coefficientRow4[3] - coefficientRow2[3] * coefficientRow4[1];
    float A1213 = coefficientRow2[1] * coefficientRow4[2] - coefficientRow2[2] * coefficientRow4[1];
    float A2312 = coefficientRow2[2] * coefficientRow3[3] - coefficientRow2[3] * coefficientRow3[2];
    float A1312 = coefficientRow2[1] * coefficientRow3[3] - coefficientRow2[3] * coefficientRow3[1];
    float A1212 = coefficientRow2[1] * coefficientRow3[2] - coefficientRow2[2] * coefficientRow3[1];
    float A0313 = coefficientRow2[0] * coefficientRow4[3] - coefficientRow2[3] * coefficientRow4[0];
    float A0213 = coefficientRow2[0] * coefficientRow4[2] - coefficientRow2[2] * coefficientRow4[0];
    float A0312 = coefficientRow2[0] * coefficientRow3[3] - coefficientRow2[3] * coefficientRow3[0];
    float A0212 = coefficientRow2[0] * coefficientRow3[2] - coefficientRow2[2] * coefficientRow3[0];
    float A0113 = coefficientRow2[0] * coefficientRow4[1] - coefficientRow2[1] * coefficientRow4[0];
    float A0112 = coefficientRow2[0] * coefficientRow3[1] - coefficientRow2[1] * coefficientRow3[0];

    float det = coefficientRow1[0] * (coefficientRow2[1] * A2323 - coefficientRow2[2] * A1323 + coefficientRow2[3] * A1223) - coefficientRow1[1] * (coefficientRow2[0] * A2323 - coefficientRow2[2] * A0323 + coefficientRow2[3] * A0223) + coefficientRow1[2] * (coefficientRow2[0] * A1323 - coefficientRow2[1] * A0323 + coefficientRow2[3] * A0123) - coefficientRow1[3] * (coefficientRow2[0] * A1223 - coefficientRow2[1] * A0223 + coefficientRow2[2] * A0123);
    det = 1 / det;

    VectorN<float, 16> inv;
    inv[0] = det * (coefficientRow2[1] * A2323 - coefficientRow2[2] * A1323 + coefficientRow2[3] * A1223);
    inv[1] = det * -(coefficientRow1[1] * A2323 - coefficientRow1[2] * A1323 + coefficientRow1[3] * A1223);
    inv[2] = det * (coefficientRow1[1] * A2313 - coefficientRow1[2] * A1313 + coefficientRow1[3] * A1213);
    inv[3] = det * -(coefficientRow1[1] * A2312 - coefficientRow1[2] * A1312 + coefficientRow1[3] * A1212);
    inv[4] = det * -(coefficientRow2[0] * A2323 - coefficientRow2[2] * A0323 + coefficientRow2[3] * A0223);
    inv[5] = det * (coefficientRow1[0] * A2323 - coefficientRow1[2] * A0323 + coefficientRow1[3] * A0223);
    inv[6] = det * -(coefficientRow1[0] * A2313 - coefficientRow1[2] * A0313 + coefficientRow1[3] * A0213);
    inv[7] = det * (coefficientRow1[0] * A2312 - coefficientRow1[2] * A0312 + coefficientRow1[3] * A0212);
    inv[8] = det * (coefficientRow2[0] * A1323 - coefficientRow2[1] * A0323 + coefficientRow2[3] * A0123);
    inv[9] = det * -(coefficientRow1[0] * A1323 - coefficientRow1[1] * A0323 + coefficientRow1[3] * A0123);
    inv[10] = det * (coefficientRow1[0] * A1313 - coefficientRow1[1] * A0313 + coefficientRow1[3] * A0113);
    inv[11] = det * -(coefficientRow1[0] * A1312 - coefficientRow1[1] * A0312 + coefficientRow1[3] * A0112);
    inv[12] = det * -(coefficientRow2[0] * A1223 - coefficientRow2[1] * A0223 + coefficientRow2[2] * A0123);
    inv[13] = det * (coefficientRow1[0] * A1223 - coefficientRow1[1] * A0223 + coefficientRow1[2] * A0123);
    inv[14] = det * -(coefficientRow1[0] * A1213 - coefficientRow1[1] * A0213 + coefficientRow1[2] * A0113);
    inv[15] = det * (coefficientRow1[0] * A1212 - coefficientRow1[1] * A0212 + coefficientRow1[2] * A0112);

    return inv;
}
VectorN<float, 9> ModeGeometric::unit_vec(Vector3f q, Vector3f q_dot, Vector3f q_ddot)
{
    // This function comes from Appendix F in https://arxiv.org/pdf/1003.2005v3.pdf
    VectorN<float, 9> uCollection; // for storage of the output
    float nq = q.length();
    Vector3f u = q / nq;
    Vector3f u_dot = q_dot / nq - q * (q * q_dot) / powF(nq, 3);
    Vector3f u_ddot = q_ddot / nq - q_dot / powF(nq, 3) * 2 * (q * q_dot) - q / powF(nq, 3) * (q_dot * q_dot + q * q_ddot) + q * 3 / powF(nq, 5) * powF(q * q_dot, 2);

    uCollection[0] = u[0];
    uCollection[1] = u[1];
    uCollection[2] = u[2];

    uCollection[3] = u_dot[0];
    uCollection[4] = u_dot[1];
    uCollection[5] = u_dot[2];

    uCollection[6] = u_ddot[0];
    uCollection[7] = u_ddot[1];
    uCollection[8] = u_ddot[2];

    return uCollection;
}

Matrix3f ModeGeometric::JoyStickToTargetAttitude()
{

    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);
    // convert to deg again
    target_roll = target_roll / 100.0;
    target_pitch = target_pitch / 100.0;
    // limit the value
    if (target_pitch >= 15)
    {
        target_pitch = 15;
    }
    if (target_pitch <= -15)
    {
        target_pitch = -15;
    }
    if (target_roll >= 15)
    {
        target_roll = 15;
    }
    if (target_roll <= -15)
    {
        target_roll = -15;
    }
    // deg 2 rad
    target_roll *= DEG_TO_RAD;
    target_pitch *= DEG_TO_RAD;

    // Vector3f x_axis_attitude_target={1,0,0};
    // Vector3f y_axis_attitude_target={0,1,0};
    // Vector3f z_axis_attitude_target={0,0,1};

    Vector3f x_axis_attitude_target = {cosf(-target_pitch), 0, sinf(-target_pitch)};
    Vector3f y_axis_attitude_target = {sinf(-target_roll) * sinf(-target_pitch), cosf(-target_roll), -sinf(-target_roll) * cosf(-target_pitch)};
    Vector3f z_axis_attitude_target = {-cosf(-target_roll) * sinf(-target_pitch), sinf(-target_roll), cosf(-target_roll) * cosf(-target_pitch)};

    Matrix3f target_attitude(Vector3f(x_axis_attitude_target.x, y_axis_attitude_target.x, z_axis_attitude_target.x),
                             Vector3f(x_axis_attitude_target.y, y_axis_attitude_target.y, z_axis_attitude_target.y),
                             Vector3f(x_axis_attitude_target.z, y_axis_attitude_target.z, z_axis_attitude_target.z));
    return target_attitude;
}

float ModeGeometric::vector_2norm(const Vector3f A)
{
    float ans;
    ans = (A[0]) * (A[0]) + (A[1]) * (A[1]) + (A[2]) * (A[2]);
    return sqrtf(ans);
}

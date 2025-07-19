#include "Copter.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Motors/AP_Motors_Class.h> // for sending motor speed
#include "mode.h"
#include "Geometric_Trajectory_Generate.h"
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>
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

    /**/
    static uint32_t initial_time_in_geometric = 0;
    static uint32_t last_time_in_geometric = 0;
    static Vector3f enterpos;    // 进入跟踪模式的位置
    static Vector3f TRJstartpos; // 完成初始加速，开始跟踪的位置
    static int8_t getposAvailable = 0;
    static int8_t trajectory_num = 0;
    static int8_t info_send_flag = 0;
    static int8_t rc_lost_info_flag = 0;
    static float init_alt = 0;                  // NED D alt 起飞点高度
    static float take_off_time = g.GeoCtrl_TFT; // 输入起飞时间
    uint32_t now_time_in_geometric = AP_HAL::micros();

    static float vb_time_start = 0;
    static float vb_time_end = 0;
    static float vs_time_start = 0;
    static float vs_time_end = 0;
    static float vd_time_start = 0;
    static float vd_time_end = 0;
    static float vl_time_start = 0;
    static float vl_time_end = 0;
    static float qb_time_start = 0;
    static float qb_time_end = 0;
    static float qs_time_start = 0;
    static float qs_time_end = 0;
    static float qd_time_start = 0;
    static float qd_time_end = 0;
    static float ql_time_start = 0;
    static float ql_time_end = 0;

    if (initial_time_in_geometric == 0 || 0.000001f * (now_time_in_geometric - last_time_in_geometric) > 0.1)
    { // 第一次或者再次进入geo模式，也就是新的一次geo飞行
        // 相当自己写了一个初始化
        initial_time_in_geometric = AP_HAL::micros();
        getposAvailable = ahrs.get_relative_position_NED_origin(enterpos);
        init_alt = enterpos.z; // 记录起飞点高度

        info_send_flag = 0;
        trajectory_num = g.GeoCtrl_NUM;
        kg_vehicleMass = g.GeoCtrl_MAS;
        gcs().send_text(MAV_SEVERITY_INFO, "Init of Trajectory");

        vb_time_start = g.GeoCtrl_VBS;
        vb_time_end = g.GeoCtrl_VBE;
        vs_time_start = g.GeoCtrl_VSS;
        vs_time_end = g.GeoCtrl_VSE;
        vd_time_start = g.GeoCtrl_VDS;
        vd_time_end = g.GeoCtrl_VDE;
        vl_time_start = g.GeoCtrl_VLS;
        vl_time_end = g.GeoCtrl_VLE;
        qb_time_start = g.GeoCtrl_QBS;
        qb_time_end = g.GeoCtrl_QBE;
        qs_time_start = g.GeoCtrl_QSS;
        qs_time_end = g.GeoCtrl_QSE;
        qd_time_start = g.GeoCtrl_QDS;
        qd_time_end = g.GeoCtrl_QDE;
        ql_time_start = g.GeoCtrl_QLS;
        ql_time_end = g.GeoCtrl_QLE;
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

    // Vector3f error_w;
    // Vector3f error_R;
    // Vector3f error_x;
    // Vector3f error_v;

    // exit, switch parm ,enter again
    switch (trajectory_num)
    {
    case 0:
        // POS
        Trajectory_Generate_POS(&targetPos, &targetVel, &targetAcc, &targetJerk, &targetSnap, &targetYaw, &targetYaw_dot, &targetYaw_ddot);

        break;
    case 1:
    {
        float T_circle = g.GeoCtrl_TIM;
        const float targetAlt = 2;
        in_horizon_flight = false;
        Trajectory_Generate_POS_AUTO(timeInThisRun, targetAlt, take_off_time, T_circle, &in_horizon_flight, &targetPos, &targetVel, &targetAcc, &targetJerk, &targetSnap, &targetYaw, &targetYaw_dot, &targetYaw_ddot);
    }

    break;

    case 2:
        // circle
        {
            float r_circle = g.GeoCtrl_RDI;
            float T_circle = g.GeoCtrl_TIM;
            const float targetAlt = 2;
            in_horizon_flight = false;
            in_trj_flight = false;
            Trajectory_Generate_CIRCLE_AUTO(timeInThisRun, targetAlt, take_off_time, r_circle, T_circle, &in_horizon_flight, &in_trj_flight, &targetPos, &targetVel, &targetAcc, &targetJerk, &targetSnap, &targetYaw, &targetYaw_dot, &targetYaw_ddot);
        }
        break;

    case 3:
        // 8
        {
            float r_circle = g.GeoCtrl_RDI;
            float T_circle = g.GeoCtrl_TIM;
            const float targetAlt = 2;
            in_horizon_flight = false;
            in_trj_flight = false;
            Trajectory_Generate_EIGHT_AUTO(timeInThisRun, targetAlt, take_off_time, r_circle, T_circle, &in_horizon_flight, &in_trj_flight, &targetPos, &targetVel, &targetAcc, &targetJerk, &targetSnap, &targetYaw, &targetYaw_dot, &targetYaw_ddot);
        }

        break;
    case 4:
    {
        float r_circle = g.GeoCtrl_RDI;
        float T_circle = g.GeoCtrl_TIM;
        const float targetAlt = 2;
        in_horizon_flight = false;
        in_trj_flight = false;
        Trajectory_Generate_LSR_AUTO(timeInThisRun, targetAlt, take_off_time, r_circle, T_circle, &in_horizon_flight, &in_trj_flight, &targetPos, &targetVel, &targetAcc, &targetJerk, &targetSnap, &targetYaw, &targetYaw_dot, &targetYaw_ddot);
    }
    break;

    default:
        Trajectory_Generate_POS(&targetPos, &targetVel, &targetAcc, &targetJerk, &targetSnap, &targetYaw, &targetYaw_dot, &targetYaw_ddot);
        gcs().send_text(MAV_SEVERITY_CRITICAL, "VOID Trajectory NUM");
        break;
    }

    // 从进入geo模式的位置开始跟踪
    if (getposAvailable)
    {
        targetPos = targetPos + enterpos;
    }

    // 尝试取消ap自带控制器的影响，实际上没有效果
    motors->set_pitch(0);
    motors->set_pitch_ff(0);
    motors->set_roll(0);
    motors->set_roll_ff(0);
    motors->set_yaw(0);
    motors->set_yaw_ff(0);

    /**/

    // end

    if (g.GeoCtrl_ADP == 0)
    {
        // 关闭自适应
        thrustAndMomentCmd = ModeGeometric::GeometricTrajectoryController(targetPos, targetVel, targetAcc, targetJerk, targetSnap, targetYaw, targetYaw_dot, targetYaw_ddot, timeInThisRun,
                                                                          vb_time_start, vb_time_end,
                                                                          vs_time_start, vs_time_end,
                                                                          vd_time_start, vd_time_end,
                                                                          vl_time_start, vl_time_end,
                                                                          qb_time_start, qb_time_end,
                                                                          qs_time_start, qs_time_end,
                                                                          qd_time_start, qd_time_end,
                                                                          ql_time_start, ql_time_end);
    }
    else if (g.GeoCtrl_ADP == 1)
    {
        // 打开自适应
        thrustAndMomentCmd = ModeGeometric::AdaptiveController(targetPos, targetVel, targetAcc, targetJerk, targetSnap, targetYaw, targetYaw_dot, targetYaw_ddot, timeInThisRun);

        if (is_in_horizon_flight(timeInThisRun))
        {
            // 测试，加入扰动
            if (g.GeoCtrl_DBV == 0)
            {
                thrustAndMomentCmd[0] += g.GeoCtrl_DB0;
                thrustAndMomentCmd[1] += g.GeoCtrl_DB1;
                thrustAndMomentCmd[2] += g.GeoCtrl_DB2;
                thrustAndMomentCmd[3] += g.GeoCtrl_DB3;
            }
            else if (g.GeoCtrl_DBV == 1)
            {
                float sin_time = timeInThisRun;
                float sin_time_w = 2 * M_PI / sin_time_T;

                thrustAndMomentCmd[0] += g.GeoCtrl_DB0 * sinf(sin_time_w * sin_time);
                thrustAndMomentCmd[1] += g.GeoCtrl_DB1 * sinf(sin_time_w * sin_time);
                thrustAndMomentCmd[2] += g.GeoCtrl_DB2 * sinf(sin_time_w * sin_time);
                thrustAndMomentCmd[3] += g.GeoCtrl_DB3 * sinf(sin_time_w * sin_time);
            }
        }
    }
    else if (g.GeoCtrl_ADP == 2)
    {
        // 关闭自适应，但是还有扰动
        thrustAndMomentCmd = ModeGeometric::GeometricTrajectoryController(targetPos, targetVel, targetAcc, targetJerk, targetSnap, targetYaw, targetYaw_dot, targetYaw_ddot, timeInThisRun,
                                                                          vb_time_start, vb_time_end,
                                                                          vs_time_start, vs_time_end,
                                                                          vd_time_start, vd_time_end,
                                                                          vl_time_start, vl_time_end,
                                                                          qb_time_start, qb_time_end,
                                                                          qs_time_start, qs_time_end,
                                                                          qd_time_start, qd_time_end,
                                                                          ql_time_start, ql_time_end);

        if (is_in_horizon_flight(timeInThisRun))
        {
            // 测试，加入扰动
            if (g.GeoCtrl_DBV == 0)
            {
                thrustAndMomentCmd[0] += g.GeoCtrl_DB0;
                thrustAndMomentCmd[1] += g.GeoCtrl_DB1;
                thrustAndMomentCmd[2] += g.GeoCtrl_DB2;
                thrustAndMomentCmd[3] += g.GeoCtrl_DB3;
            }
            else if (g.GeoCtrl_DBV == 1)
            {
                float sin_time = timeInThisRun;
                float sin_time_w = 2 * M_PI / sin_time_T;

                thrustAndMomentCmd[0] += g.GeoCtrl_DB0 * sinf(sin_time_w * sin_time);
                thrustAndMomentCmd[1] += g.GeoCtrl_DB1 * sinf(sin_time_w * sin_time);
                thrustAndMomentCmd[2] += g.GeoCtrl_DB2 * sinf(sin_time_w * sin_time);
                thrustAndMomentCmd[3] += g.GeoCtrl_DB3 * sinf(sin_time_w * sin_time);
            }
        }
    }

    // motor mixing
    VectorN<float, 4> motorPWM;
    if (g.GeoCtrl_MIX == 0)
    {
        motorPWM = motorMixSimple(thrustAndMomentCmd);
    }
    else
    {
        motorPWM = motorMixSimple(thrustAndMomentCmd);
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

    // 手动紧急停止 Estop开关
    bool interlock = !SRV_Channels::get_emergency_stop(); // interlock 1 转 0 不转

    // 遥控器失控保护
    // 在ARM之后可以用，在显示FLYING之后不生效，QGC设置如果失控进入LAND模式
    if ((copter.failsafe.radio || !copter.ap.rc_receiver_present))
    {
        if (rc_lost_info_flag == 0)
        {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "RC_LOST");
            rc_lost_info_flag = 1;
        }

        interlock = 0;
        motorEnable = 0;
        motors->set_interlock(false); // 不转
    }

    // 根据interlock判断是否关闭电机 1 转 0 不转
    if (interlock)
    {
        motorEnable = 1;
        motors->set_interlock(true); // 转
        // LOGGER_WRITE_EVENT(LogEvent::MOTORS_INTERLOCK_ENABLED);
    }
    else if (!interlock)
    {
        motorEnable = 0;
        motors->set_interlock(false); // 不转

        // LOGGER_WRITE_EVENT(LogEvent::MOTORS_INTERLOCK_DISABLED);
    }

    // 降落检测
    GEO_land_detect(init_alt);
    if (land_is_ok_flag == 1 && timeInThisRun > 10)
    {
        motorEnable = 0;
        motors->set_interlock(false); // 不转
    }

    // logging
    AP::logger().Write("FSAF", "TimeUS,radi,rece,mabl", "Qbbb",
                       AP_HAL::micros64(),
                       copter.failsafe.radio,
                       copter.ap.rc_receiver_present,
                       motorEnable);

    if (motors->armed() && motorEnable == 1) // only command the motor PWM when the vehicle is armed.
    {
        motors->rc_write(0, 1000 + motorEnable * 10 * motorPWM[0]); // manual set motor speed: PWM_MIN/MAX has been forced to 1000/2000
        motors->rc_write(1, 1000 + motorEnable * 10 * motorPWM[1]); // rc_write is called from <AP_Motors/AP_Motors_Class.h>
        motors->rc_write(2, 1000 + motorEnable * 10 * motorPWM[2]);
        motors->rc_write(3, 1000 + motorEnable * 10 * motorPWM[3]);

        //     motors->rc_write(0, motorEnable * 0);
        //     motors->rc_write(1, motorEnable * 0);
        //     motors->rc_write(2, motorEnable * 0);
        //     motors->rc_write(3, motorEnable * 0);
    }
    else
    {
        if (info_send_flag == 0)
        {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Vehicle not armed.");
            info_send_flag = 1;
        }

        motorEnable = 0; // if the vehicle is not armed, disable the flight.
        motors->rc_write(0, motorEnable * 1000);
        motors->rc_write(1, motorEnable * 1000);
        motors->rc_write(2, motorEnable * 1000);
        motors->rc_write(3, motorEnable * 1000);
    }
    // logging
    AP::logger().Write("MOUT", "TimeUS,m1,m2,m3,m4,able", "Qffffb",
                       AP_HAL::micros64(),
                       motorPWM[0],
                       motorPWM[1],
                       motorPWM[2],
                       motorPWM[3],
                       motorEnable);
    AP::logger().Write("TMRD", "TimeUS,tm0,tm1,tm2,tm3", "Qffff",
                       AP_HAL::micros64(),
                       (thrustAndMomentCmd[0]),
                       (thrustAndMomentCmd[1]),
                       (thrustAndMomentCmd[2]),
                       (thrustAndMomentCmd[3]));
    AP::logger().Write("TIMT", "TimeUS,isf,trj", "Qbb",
                       AP_HAL::micros64(),
                       (in_horizon_flight),
                       (in_trj_flight));
    last_time_in_geometric = now_time_in_geometric;
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
    float timeInThisRun,
    float param_vb_time_start,
    float param_vb_time_end,
    float param_vs_time_start,
    float param_vs_time_end,
    float param_vd_time_start,
    float param_vd_time_end,
    float param_vl_time_start,
    float param_vl_time_end,
    float param_qb_time_start,
    float param_qb_time_end,
    float param_qs_time_start,
    float param_qs_time_end,
    float param_qd_time_start,
    float param_qd_time_end,
    float param_ql_time_start,
    float param_ql_time_end)
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

    // 尝试取消ap自带控制器的影响，实际上没有效果
    motors->set_pitch(0);
    motors->set_pitch_ff(0);
    motors->set_roll(0);
    motors->set_roll_ff(0);
    motors->set_yaw(0);
    motors->set_yaw_ff(0);

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

    static float v_lock;
    static int v_lock_flag = 0;
    const float vb_time_start = param_vb_time_start;
    const float vb_time_end = param_vb_time_end;
    const float vs_time_start = param_vs_time_start;
    const float vs_time_end = param_vs_time_end;
    const float vd_time_start = param_vd_time_start;
    const float vd_time_end = param_vd_time_end;
    const float vl_time_start = param_vl_time_start;
    const float vl_time_end = param_vl_time_end;
    int8_t vb_flag = 0;
    int8_t vs_flag = 0;
    int8_t vd_flag = 0;
    int8_t vl_flag = 0;
    if (timeInThisRun > vb_time_start && timeInThisRun < vb_time_end)
    {
        stateVel.y += 1;
        vb_flag = 1;
    }
    if (timeInThisRun > vs_time_start && timeInThisRun < vs_time_end)
    {
        stateVel.y = (1.0 / 2) * stateVel.y;
        vs_flag = 1;
    }
    if (timeInThisRun > vd_time_start && timeInThisRun < vd_time_end)
    {
        stateVel.y += 0.2 * (timeInThisRun - vd_time_start);
        vd_flag = 1;
    }
    if (timeInThisRun > vl_time_start && timeInThisRun < vl_time_end)
    {
        if (v_lock_flag == 0)
        {
            v_lock = stateVel.y;
            v_lock_flag = 1;
        }
        stateVel.y = v_lock;
        vl_flag = 1;
    }

    // Velocity Error (ev)
    v_error = stateVel - targetVel;

    // kg_vehicleMass = g.GeoCtrl_MAS; // weight for the real drone

    // Target force
    target_force.x = kg_vehicleMass * targetAcc.x - g.GeoCtrl_Kpx * r_error.x - g.GeoCtrl_Kvx * v_error.x;
    target_force.y = kg_vehicleMass * targetAcc.y - g.GeoCtrl_Kpy * r_error.y - g.GeoCtrl_Kvy * v_error.y;
    target_force.z = kg_vehicleMass * (targetAcc.z - GRAVITY_MAGNITUDE) - g.GeoCtrl_Kpz * r_error.z - g.GeoCtrl_Kvz * v_error.z;

    // // Z-Axis [zB]
    // Quaternion q;
    // ahrs.get_quat_body_to_ned(q);

    // Matrix3f R;
    // q.rotation_matrix(R); // transforming the quaternion q to rotation matrix R

    Matrix3f R;
    Vector3f euler;
    if (!ahrs.get_secondary_attitude(euler))
    {
        VectorN<float, 4> no_out_put;
        no_out_put[0] = 0;
        no_out_put[1] = 0;
        no_out_put[2] = 0;
        no_out_put[3] = 0;
        return no_out_put;
    }
    float roll_rad = euler.x;
    float pitch_rad = euler.y;
    float yaw_rad = euler.z;

    float cosPhi = cosf(roll_rad);
    float sinPhi = sinf(roll_rad);
    float cosTheta = cosf(pitch_rad);
    float sinTheta = sinf(pitch_rad);
    float cosPsi = cosf(yaw_rad);
    float sinPsi = sinf(yaw_rad);

    // R(Vector3f(), Vector3f(), Vector3f());

    // 第一行
    R[0][0] = cosPsi * cosTheta;
    R[0][1] = cosPsi * sinTheta * sinPhi - sinPsi * cosPhi;
    R[0][2] = cosPsi * sinTheta * cosPhi + sinPsi * sinPhi;

    // 第二行
    R[1][0] = sinPsi * cosTheta;
    R[1][1] = sinPsi * sinTheta * sinPhi + cosPsi * cosPhi;
    R[1][2] = sinPsi * sinTheta * cosPhi - cosPsi * sinPhi;

    // 第三行
    R[2][0] = -sinTheta;
    R[2][1] = cosTheta * sinPhi;
    R[2][2] = cosTheta * cosPhi;

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

    Vector3f Omega = AP::ahrs().get_gyro();

    static float q_lock;
    static int q_lock_flag = 0;
    const float qb_time_start = param_qb_time_start;
    const float qb_time_end = param_qb_time_end;
    const float qs_time_start = param_qs_time_start;
    const float qs_time_end = param_qs_time_end;
    const float qd_time_start = param_qd_time_start;
    const float qd_time_end = param_qd_time_end;
    const float ql_time_start = param_ql_time_start;
    const float ql_time_end = param_ql_time_end;
    int8_t qb_flag = 0;
    int8_t qs_flag = 0;
    int8_t qd_flag = 0;
    int8_t ql_flag = 0;
    if (timeInThisRun > qb_time_start && timeInThisRun < qb_time_end)
    {
        Omega.y += 1;
        qb_flag = 1;
    }
    if (timeInThisRun > qs_time_start && timeInThisRun < qs_time_end)
    {
        Omega.y = (1.0 / 2) * Omega.y;
        qs_flag = 1;
    }
    if (timeInThisRun > qd_time_start && timeInThisRun < qd_time_end)
    {
        Omega.y += 0.2 * (timeInThisRun - qd_time_start);
        qd_flag = 1;
    }
    if (timeInThisRun > ql_time_start && timeInThisRun < ql_time_end)
    {
        if (q_lock_flag == 0)
        {
            q_lock = Omega.y;
            q_lock_flag = 1;
        }
        Omega.y = q_lock;
        ql_flag = 1;
    }

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
    AP::logger().Write("FTFG", "TimeUS,vb,vs,vd,vl,qb,qs,qd,ql", "Qbbbbbbbb",
                       AP_HAL::micros64(),
                       (vb_flag),
                       (vs_flag),
                       (vd_flag),
                       (vl_flag),
                       (qb_flag),
                       (qs_flag),
                       (qd_flag),
                       (ql_flag));
    AP::logger().Write("OMGE", "TimeUS,p,q,r,mp,mq,mr", "Qffffff",
                       AP_HAL::micros64(),
                       (Omega.x),
                       (Omega.y),
                       (Omega.z),
                       (M.x),
                       (M.y),
                       (M.z));

    AP::logger().Write("GEOM", "TimeUS,exx,exy,exz,evx,evy,evz,erx,ery,erz,ewx,ewy,ewz", "Qffffffffffff",
                       AP_HAL::micros64(),
                       (r_error.x),
                       (r_error.y),
                       (r_error.z),
                       (v_error.x),
                       (v_error.y),
                       (v_error.z),
                       (eR.x),
                       (eR.y),
                       (eR.z),
                       (ew.x),
                       (ew.y),
                       (ew.z));
    AP::logger().Write("GECT", "TimeUS,tfx,tfy,tfz,tt,mx,my,mz", "Qfffffff",
                       AP_HAL::micros64(),
                       (target_force.x),
                       (target_force.y),
                       (target_force.z),
                       (target_thrust),
                       (M.x),
                       (M.y),
                       (M.z));
    AP::logger().Write("GETA", "TimeUS,tpx,tpy,tpz,spx,spy,spz,svx,svy,syz", "Qfffffffff",
                       AP_HAL::micros64(),
                       (targetPos.x),
                       (targetPos.y),
                       (targetPos.z),
                       (statePos.x),
                       (statePos.y),
                       (statePos.z),
                       (stateVel.x),
                       (stateVel.y),
                       (stateVel.z));
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

VectorN<float, 4> ModeGeometric::AdaptiveController(Vector3f targetPos,
                                                    Vector3f targetVel,
                                                    Vector3f targetAcc,
                                                    Vector3f targetJerk,
                                                    Vector3f targetSnap,
                                                    Vector2f targetYaw,
                                                    Vector2f targetYaw_dot,
                                                    Vector2f targetYaw_ddot,
                                                    float timeinrun)
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

    // 尝试取消ap自带控制器的影响，实际上没有效果
    motors->set_pitch(0);
    motors->set_pitch_ff(0);
    motors->set_roll(0);
    motors->set_roll_ff(0);
    motors->set_yaw(0);
    motors->set_yaw_ff(0);

    static uint32_t now_time = 0;
    static uint32_t last_time = 0;
    // static uint32_t init_time = 0;

    Matrix3f W_x;
    Matrix3f W_x_dot;
    Matrix3f W_x_ddot;

    Matrix3f W_R;
    static Vector3f theta_x = Vector3f(0, 0, 0);
    static Vector3f theta_R = Vector3f(0, 0, 0);
    Vector3f theta_x_dot = {0, 0, 0};
    Vector3f theta_x_ddot = {0, 0, 0};
    Vector3f theta_R_dot = {0, 0, 0};

    bool locAvailable = ahrs.get_relative_position_NED_origin(statePos);
    if (!locAvailable)
    {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "location unavailable.");
    }

    now_time = AP_HAL::micros();
    if (last_time == 0 || 0.000001f * (now_time - last_time) > 0.1)
    {
        // 判断 是不是第一次进入ADP
        last_time = now_time;
        // init_time = now_time;
        theta_x = Vector3f{0, 0, 0};
        theta_R = Vector3f{0, 0, 0};
        theta_x_dot = Vector3f{0, 0, 0};
        theta_x_ddot = Vector3f{0, 0, 0};
        theta_R_dot = Vector3f{0, 0, 0};
        // initalt = statePos.z;
        gcs().send_text(MAV_SEVERITY_CRITICAL, "NEW ADP");
    }

    // 计算程序运行时间周期
    // dt = (float)0.000001f * (now_time - last_time);
    const float dt = 0.0025;

    // float time_in_adp = (float)0.000001f * (now_time - init_time);

    if (g.GeoCtrl_DBV == 0)
    {
        W_x(Vector3f(1, 0, 0),
            Vector3f(0, 1, 0),
            Vector3f(0, 0, 1));
        W_x_dot(Vector3f(0, 0, 0),
                Vector3f(0, 0, 0),
                Vector3f(0, 0, 0));
        W_x_ddot(Vector3f(0, 0, 0),
                 Vector3f(0, 0, 0),
                 Vector3f(0, 0, 0));

        W_R(Vector3f(1, 0, 0),
            Vector3f(0, 1, 0),
            Vector3f(0, 0, 1));
    }
    else if (g.GeoCtrl_DBV == 1)
    {

        // float sin_time = time_in_adp;
        float sin_time = timeinrun;
        float sin_time_w = 2 * M_PI / sin_time_T;

        W_x(Vector3f(sinf(sin_time_w * sin_time), 0, 0),
            Vector3f(0, sinf(sin_time_w * sin_time), 0),
            Vector3f(0, 0, sinf(sin_time_w * sin_time)));
        W_x_dot(Vector3f(sin_time_w * cosf(sin_time_w * sin_time), 0, 0),
                Vector3f(0, sin_time_w * cosf(sin_time_w * sin_time), 0),
                Vector3f(0, 0, sin_time_w * cosf(sin_time_w * sin_time)));
        W_x_ddot(Vector3f(-sin_time_w * sin_time_w * sinf(sin_time_w * sin_time), 0, 0),
                 Vector3f(0, -sin_time_w * sin_time_w * sinf(sin_time_w * sin_time), 0),
                 Vector3f(0, 0, -sin_time_w * sin_time_w * sinf(sin_time_w * sin_time)));

        // W_x(Vector3f(0, 0, 0),
        //     Vector3f(0, 0, 0),
        //     Vector3f(0, 0, sinf(sin_time_w * sin_time)));
        // W_x_dot(Vector3f(0, 0, 0),
        //         Vector3f(0, 0, 0),
        //         Vector3f(0, 0, sin_time_w * cosf(sin_time_w * sin_time)));
        // W_x_ddot(Vector3f(0, 0, 0),
        //          Vector3f(0, 0, 0),
        //          Vector3f(0, 0, -sin_time_w * sin_time_w * sinf(sin_time_w * sin_time)));

        W_R(Vector3f(sinf(sin_time_w * sin_time), 0, 0),
            Vector3f(0, sinf(sin_time_w * sin_time), 0),
            Vector3f(0, 0, sinf(sin_time_w * sin_time)));
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

    Vector2f paramc;
    paramc.x = g.GeoCtrl_C1; // c1
    paramc.y = g.GeoCtrl_C2; // c2
    // paramc.z = 0;

    float gamma_x = g.GeoCtrl_GAX;

    // Position Error (ep)
    r_error = statePos - targetPos;

    // Velocity Error (ev)
    v_error = stateVel - targetVel;

    // kg_vehicleMass = g.GeoCtrl_MAS; // weight for the real drone

    // Target force
    target_force.x = kg_vehicleMass * targetAcc.x - g.GeoCtrl_Kpx * r_error.x - g.GeoCtrl_Kvx * v_error.x;
    target_force.y = kg_vehicleMass * targetAcc.y - g.GeoCtrl_Kpy * r_error.y - g.GeoCtrl_Kvy * v_error.y;
    target_force.z = kg_vehicleMass * (targetAcc.z - GRAVITY_MAGNITUDE) - g.GeoCtrl_Kpz * r_error.z - g.GeoCtrl_Kvz * v_error.z;
    target_force = target_force - W_x * theta_x; // 加入自适应项
    //
    Vector3f ev_c1ex = v_error + r_error * paramc.x;

    float norm_theta_x = vector_2norm(theta_x);
    // Matrix3f theta_x_T(Vector3f(theta_x.x, 0, 0), Vector3f(theta_x.y, 0, 0), Vector3f(theta_x.z, 0, 0));
    // Matrix3f theta_x_Mat(Vector3f(theta_x.x, theta_x.y, theta_x.z), Vector3f(0, 0, 0), Vector3f(0, 0, 0));

    // if (norm_theta_x < g.GeoCtrl_BX || ((fabsf(norm_theta_x - g.GeoCtrl_BX) <= 1e-3f) && ((theta_x_T * (W_x.transposed()) * ev_c1ex).x <= 0)))
    if (norm_theta_x < g.GeoCtrl_BX)
    {
        theta_x_dot = (W_x.transposed()) * ev_c1ex * gamma_x;

        // gcs().send_text(MAV_SEVERITY_CRITICAL, "case1\n");
    }
    // else
    // {

    //     // Matrix3f theta_x_T_theta = (theta_x_T * theta_x_Mat);
    //     // Matrix3f theta_x_T_theta_inv;
    //     // if (theta_x_T_theta.inverse(theta_x_T_theta_inv))
    //     // {
    //     //     Matrix3f eye3(Vector3f(1, 0, 0), Vector3f(0, 1, 0), Vector3f(0, 0, 1));
    //     //     Matrix3f I_theta = eye3 - (theta_x_Mat * theta_x_T) * theta_x_T_theta_inv;
    //     //     theta_x_dot = I_theta * W_x.transposed() * ev_c1ex * gamma_x;

    //     //     // gcs().send_text(MAV_SEVERITY_CRITICAL, "case2\n");
    //     // }
    //     // else
    //     // {
    //     //     theta_x_dot.x = 0;
    //     //     theta_x_dot.y = 0;
    //     //     theta_x_dot.z = 0;

    //     //     // Matrix3f eye3(Vector3f(1,0,0),Vector3f(0,1,0),Vector3f(0,0,1));
    //     //     // Matrix3f I_theta = eye3 - (theta_x_Mat * theta_x_T) * theta_x_T_theta_inv;
    //     //     // theta_x_dot =  I_theta * W_x.transposed() * ev_c1ex;

    //     //     // theta_x_dot.x = g.GeoCtrl_GAX * theta_x_dot.x;
    //     //     // theta_x_dot.y = g.GeoCtrl_GAX * theta_x_dot.y;
    //     //     // theta_x_dot.z = g.GeoCtrl_GAX * theta_x_dot.z;

    //     //     // gcs().send_text(MAV_SEVERITY_CRITICAL, "case3\n");
    //     // }
    // }

    // // Z-Axis [zB]
    // Quaternion q;
    // ahrs.get_quat_body_to_ned(q);

    // Matrix3f R;
    // q.rotation_matrix(R); // transforming the quaternion q to rotation matrix R

    Matrix3f R;
    Vector3f euler;
    if (!ahrs.get_secondary_attitude(euler))
    {
        VectorN<float, 4> no_out_put;
        no_out_put[0] = 0;
        no_out_put[1] = 0;
        no_out_put[2] = 0;
        no_out_put[3] = 0;
        return no_out_put;
    }
    float roll_rad = euler.x;
    float pitch_rad = euler.y;
    float yaw_rad = euler.z;

    float cosPhi = cosf(roll_rad);
    float sinPhi = sinf(roll_rad);
    float cosTheta = cosf(pitch_rad);
    float sinTheta = sinf(pitch_rad);
    float cosPsi = cosf(yaw_rad);
    float sinPsi = sinf(yaw_rad);

    // R(Vector3f(), Vector3f(), Vector3f());

    // 第一行
    R[0][0] = cosPsi * cosTheta;
    R[0][1] = cosPsi * sinTheta * sinPhi - sinPsi * cosPhi;
    R[0][2] = cosPsi * sinTheta * cosPhi + sinPsi * sinPhi;

    // 第二行
    R[1][0] = sinPsi * cosTheta;
    R[1][1] = sinPsi * sinTheta * sinPhi + cosPsi * cosPhi;
    R[1][2] = sinPsi * sinTheta * cosPhi - cosPsi * sinPhi;

    // 第三行
    R[2][0] = -sinTheta;
    R[2][1] = cosTheta * sinPhi;
    R[2][2] = cosTheta * cosPhi;

    z_axis = R.colz(); // b3

    // target thrust [F]
    float target_thrust = -target_force * z_axis;

    // compute Omegad: this comes from Appendix F in https://arxiv.org/pdf/1003.2005v3.pdf
    Vector3f a_error;                                                                         // ev_dot   dot(v-v_des)
    a_error = e3 * GRAVITY_MAGNITUDE - R.colz() * target_thrust / kg_vehicleMass - targetAcc; // ev_dot

    Vector3f target_force_dot; // derivative of target_force
    target_force_dot.x = -g.GeoCtrl_Kpx * v_error.x - g.GeoCtrl_Kvx * a_error.x + kg_vehicleMass * targetJerk.x;
    target_force_dot.y = -g.GeoCtrl_Kpy * v_error.y - g.GeoCtrl_Kvy * a_error.y + kg_vehicleMass * targetJerk.y;
    target_force_dot.z = -g.GeoCtrl_Kpz * v_error.z - g.GeoCtrl_Kvz * a_error.z + kg_vehicleMass * targetJerk.z;
    target_force_dot = target_force_dot - W_x_dot * theta_x - W_x * theta_x_dot;

    // norm_theta_x = vector_2norm(theta_x);
    // Matrix3f theta_x_T(Vector3f(theta_x.x, 0, 0), Vector3f(theta_x.y, 0, 0), Vector3f(theta_x.z, 0, 0));
    // Matrix3f theta_x_Mat(Vector3f(theta_x.x, theta_x.y, theta_x.z), Vector3f(0, 0, 0), Vector3f(0, 0, 0));

    // if (norm_theta_x < g.GeoCtrl_BX || ((fabsf(norm_theta_x - g.GeoCtrl_BX) <= 1e-3f) && ((theta_x_T * (W_x.transposed()) * ev_c1ex).x <= 0)))
    if (norm_theta_x < g.GeoCtrl_BX)
    {

        theta_x_ddot = (W_x_dot.transposed()) * ev_c1ex * gamma_x + (W_x.transposed()) * (a_error + v_error * paramc.x) * gamma_x;

        // gcs().send_text(MAV_SEVERITY_CRITICAL, "case1\n");
    }
    // else
    // {

    //     // Matrix3f theta_x_T_theta = (theta_x_T * theta_x_Mat);
    //     // Matrix3f theta_x_T_theta_inv;
    //     // if (theta_x_T_theta.inverse(theta_x_T_theta_inv))
    //     // {
    //     //     Matrix3f eye3(Vector3f(1, 0, 0), Vector3f(0, 1, 0), Vector3f(0, 0, 1));
    //     //     Matrix3f I_theta = eye3 - (theta_x_Mat * theta_x_T) * theta_x_T_theta_inv;
    //     //     theta_x_dot = I_theta * W_x.transposed() * ev_c1ex * gamma_x;

    //     //     // gcs().send_text(MAV_SEVERITY_CRITICAL, "case2\n");
    //     // }
    //     // else
    //     // {
    //     //     theta_x_ddot.x = 0;
    //     //     theta_x_ddot.y = 0;
    //     //     theta_x_ddot.z = 0;

    //     //     // gcs().send_text(MAV_SEVERITY_CRITICAL, "case3\n");
    //     // }
    // }

    Vector3f Omega = AP::ahrs().get_gyro();
    Vector3f b3_dot = R * hatOperator(Omega) * e3;

    float target_thrust_dot = -target_force_dot * R.colz() - target_force * b3_dot; // f_dot

    Vector3f j_error; // error on jerk  ev_2dot
    j_error = -R.colz() * target_thrust_dot / kg_vehicleMass - b3_dot * target_thrust / kg_vehicleMass - targetJerk;

    Vector3f target_force_ddot; // derivative of target_force_dot
    target_force_ddot.x = -g.GeoCtrl_Kpx * a_error.x - g.GeoCtrl_Kvx * j_error.x + kg_vehicleMass * targetSnap.x;
    target_force_ddot.y = -g.GeoCtrl_Kpy * a_error.y - g.GeoCtrl_Kvy * j_error.y + kg_vehicleMass * targetSnap.y;
    target_force_ddot.z = -g.GeoCtrl_Kpz * a_error.z - g.GeoCtrl_Kvz * j_error.z + kg_vehicleMass * targetSnap.z;
    target_force_ddot = target_force_ddot - W_x_ddot * theta_x - W_x_dot * theta_x_dot * 2 - W_x * theta_x_ddot;

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

    // x_axis_desired = z_axis_desired x [cos(yaw), sin(yaw), 0]^T
    x_c_des[0] = targetYaw[0]; // x
    x_c_des[1] = targetYaw[1]; // y
    x_c_des[2] = 0;            // z

    Vector3f x_c_des_dot = {targetYaw_dot, 0};   // time derivative of x_c_des
    Vector3f x_c_des_ddot = {targetYaw_ddot, 0}; // time derivative of x_c_des_dot

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

    Vector3f b1c = hatOperator(b2c) * b3c;
    Vector3f b1c_dot = hatOperator(b2c_dot) * b3c + hatOperator(b2c) * b3c_dot;
    Vector3f b1c_ddot = hatOperator(b2c_ddot) * b3c + hatOperator(b2c_dot) * b3c_dot * 2 + hatOperator(b2c) * b3c_ddot;

    // [eR]
    Matrix3f Rdes(b1c, b2c, b3c);
    Matrix3f Rd_dot(b1c_dot, b2c_dot, b3c_dot);
    Matrix3f Rd_ddot(b1c_ddot, b2c_ddot, b3c_ddot);
    Rdes.transpose();
    Rd_dot.transpose();
    Rd_ddot.transpose();

    Matrix3f eRM = (Rdes.transposed() * R - R.transposed() * Rdes) / 2;
    eR = veeOperator(eRM);

    Vector3f Omegad = veeOperator(Rdes.transposed() * Rd_dot);
    Vector3f Omegad_dot = veeOperator(Rdes.transposed() * Rd_ddot - hatOperator(Omegad) * hatOperator(Omegad));

    // eomega (angular velocity error)
    ew = Omega - R.transposed() * Rdes * Omegad;

    // Compute the moment
    M.x = -g.GeoCtrl_KRx * eR.x - g.GeoCtrl_KOx * ew.x;
    M.y = -g.GeoCtrl_KRy * eR.y - g.GeoCtrl_KOy * ew.y;
    M.z = -g.GeoCtrl_KRz * eR.z - g.GeoCtrl_KOz * ew.z;
    M = M - J * (hatOperator(Omega) * R.transposed() * Rdes * Omegad - R.transposed() * Rdes * Omegad_dot);
    Vector3f momentAdd = Omega % (J * Omega); // J is the inertia matrix
    M = M + momentAdd;
    M = M - W_R * theta_R;

    Vector3f ew_c2er;
    ew_c2er = ew + eR * paramc.y;
    theta_R_dot = (W_R.transposed()) * ew_c2er * g.GeoCtrl_GAR;

    // Vector3f theta_R_ddot = {0, 0, 0};
    // theta_x = theta_x + theta_x_dot * dt + theta_x_ddot * dt * dt * 0.5;
    // theta_R = theta_R + theta_R_dot * dt + theta_R_ddot * dt * dt * 0.5;

    theta_x = theta_x + theta_x_dot * dt;
    theta_R = theta_R + theta_R_dot * dt;

    if (land_is_ok_flag == 1 || (!is_in_horizon_flight(timeinrun)))
    {
        theta_x = {0, 0, 0};
        theta_R = {0, 0, 0};
    }

    VectorN<float, 4> thrustMomentCmd;
    thrustMomentCmd[0] = target_thrust;
    thrustMomentCmd[1] = M.x;
    thrustMomentCmd[2] = M.y;
    thrustMomentCmd[3] = M.z;

    // logging
    AP::logger().Write("GEOM", "TimeUS,exx,exy,exz,evx,evy,evz,erx,ery,erz,ewx,ewy,ewz", "Qffffffffffff",
                       AP_HAL::micros64(),
                       (r_error.x),
                       (r_error.y),
                       (r_error.z),
                       (v_error.x),
                       (v_error.y),
                       (v_error.z),
                       (eR.x),
                       (eR.y),
                       (eR.z),
                       (ew.x),
                       (ew.y),
                       (ew.z));
    AP::logger().Write("GECT", "TimeUS,tfx,tfy,tfz,tt,mx,my,mz", "Qfffffff",
                       AP_HAL::micros64(),
                       (target_force.x),
                       (target_force.y),
                       (target_force.z),
                       (target_thrust),
                       (M.x),
                       (M.y),
                       (M.z));
    AP::logger().Write("GETA", "TimeUS,tpx,tpy,tpz,spx,spy,spz", "Qffffff",
                       AP_HAL::micros64(),
                       (targetPos.x),
                       (targetPos.y),
                       (targetPos.z),
                       (statePos.x),
                       (statePos.y),
                       (statePos.z));
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
    // // logging
    AP::logger().Write("THET", "TimeUS,tx1,tx2,tx3,tr1,tr2,tr3,the,dt", "Qffffffff",
                       AP_HAL::micros64(),
                       (theta_x.x),
                       (theta_x.y),
                       (theta_x.z),
                       (theta_R.x),
                       (theta_R.y),
                       (theta_R.z),
                       norm_theta_x,
                       dt);

    last_time = now_time;
    return thrustMomentCmd;
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
                       (motor_pwm[3]));

    return motor_pwm;
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

float ModeGeometric::vector_2norm(Vector3f A)
{
    float ans;
    ans = (A[0]) * (A[0]) + (A[1]) * (A[1]) + (A[2]) * (A[2]);
    return sqrtf(ans);
}
// bool ModeGeometric::att_not_safe()
// {
//     return false;
// }
void ModeGeometric::GEO_land_detect(float initalt)
{ // Ground velocity in meters/second, North/East/Down
    // order. Check if have_inertial_nav() is true before assigning values to stateVel.
    Vector3f statePosLAND;
    // Vector3f stateVel;
    int8_t locAvailable = ahrs.get_relative_position_NED_origin(statePosLAND);
    float flight_alt_now = 0;
    if (locAvailable)
    {
        flight_alt_now = statePosLAND.z;
    }
    land_is_ok_flag = 0;

    // 降落检测--飞行高度
    if ((-flight_alt_now) - (-initalt) <= g.GeoCtrl_ALL)
    {
        land_is_ok_flag = 1;
        return;
    }

    return;
}

bool ModeGeometric::is_in_horizon_flight(float flight_time)
{

    if (flight_time > g.GeoCtrl_TFT && in_horizon_flight)
    {
        return true;
    }
    else
    {
        return false;
    }
}
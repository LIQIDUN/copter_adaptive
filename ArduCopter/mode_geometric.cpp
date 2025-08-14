#include "Copter.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
// #include <AP_Motors/AP_Motors_Class.h> // for sending motor speed
#include "mode.h"
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>

/*
 * Init and run calls for Geometric flight mode
 */
bool ModeGeometric::init(bool ignore_checks)
{
    ControllerIn.kpx = g.GeoCtrl_Kpx;
    ControllerIn.kpy = g.GeoCtrl_Kpy;
    ControllerIn.kpz = g.GeoCtrl_Kpz;
    ControllerIn.kvx = g.GeoCtrl_Kvx;
    ControllerIn.kvy = g.GeoCtrl_Kvy;
    ControllerIn.kvz = g.GeoCtrl_Kvz;
    ControllerIn.krx = g.GeoCtrl_KRx;
    ControllerIn.kry = g.GeoCtrl_KRy;
    ControllerIn.krz = g.GeoCtrl_KRz;
    ControllerIn.kox = g.GeoCtrl_KOx;
    ControllerIn.koy = g.GeoCtrl_KOy;
    ControllerIn.koz = g.GeoCtrl_KOz;
    ControllerIn.GRAVITY_MAGNITUDE = GRAVITY_MAGNITUDE;
    ControllerIn.kg_vehicleMass = g.GeoCtrl_MAS;
    ControllerIn.J = J; // inertia matrix
    DBInput.DB_type = g.GeoCtrl_DBV;

    initial_time_in_geometric = AP_HAL::micros();
    getposAvailable = ahrs.get_relative_position_NED_origin(enterpos);
    init_alt = enterpos.z; // 记录起飞点高度
    info_send_flag = 0;
    trajectory_num = g.GeoCtrl_NUM;
    take_off_time = g.GeoCtrl_TFT;

    gcs().send_text(MAV_SEVERITY_INFO, "Init of Trajectory");

    return true;
}

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

    uint32_t now_time_in_geometric = AP_HAL::micros();
    float timeInThisRun = (float)0.000001f * (now_time_in_geometric - initial_time_in_geometric);
    // gcs().send_text(MAV_SEVERITY_INFO, "%f", timeInThisRun);

    Vector3f targetPos;
    Vector3f targetVel;
    Vector3f targetAcc;
    Vector3f targetJerk;
    Vector3f targetSnap;
    Vector2f targetYaw;
    Vector2f targetYaw_dot;
    Vector2f targetYaw_ddot;

    Vector3f statePos;
    Vector3f stateVel;
    Vector3f euler;

    switch (trajectory_num) // exit, switch parm ,enter again
    {
    case 0:
    {
        // POS
        Trajectory_Generate_POS(&targetPos, &targetVel, &targetAcc, &targetJerk, &targetSnap, &targetYaw, &targetYaw_dot, &targetYaw_ddot);
    }
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
    { // circle
        float r_circle = g.GeoCtrl_RDI;
        float T_circle = g.GeoCtrl_TIM;
        const float targetAlt = 2;
        in_horizon_flight = false;
        in_trj_flight = false;
        Trajectory_Generate_CIRCLE_AUTO(timeInThisRun, targetAlt, take_off_time, r_circle, T_circle, &in_horizon_flight, &in_trj_flight, &targetPos, &targetVel, &targetAcc, &targetJerk, &targetSnap, &targetYaw, &targetYaw_dot, &targetYaw_ddot);
    }
    break;
    case 3:
    { // 8
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
    {
        Trajectory_Generate_POS(&targetPos, &targetVel, &targetAcc, &targetJerk, &targetSnap, &targetYaw, &targetYaw_dot, &targetYaw_ddot);
        gcs().send_text(MAV_SEVERITY_CRITICAL, "VOID Trajectory NUM");
    }
    break;
    }

    // 从进入geo模式的位置开始跟踪
    if (getposAvailable)
    {
        targetPos = targetPos + enterpos;
    }
    // gcs().send_text(MAV_SEVERITY_INFO, "%f", targetPos.z);
    // gcs().send_text(MAV_SEVERITY_INFO, "%d", getposAvailable);

    ControllerIn.targetPos = targetPos;
    ControllerIn.targetVel = targetVel;
    ControllerIn.targetAcc = targetAcc;
    ControllerIn.targetJerk = targetJerk;
    ControllerIn.targetSnap = targetSnap;
    ControllerIn.targetYaw = targetYaw;
    ControllerIn.targetYaw_dot = targetYaw_dot;
    ControllerIn.targetYaw_ddot = targetYaw_ddot;

    // gcs().send_text(MAV_SEVERITY_INFO, "%f", ControllerIn.targetPos.z);

    bool pos_estimate_error = 0;
    // get statePos
    int8_t locAvailable = ahrs.get_relative_position_NED_origin(statePos);
    if (!locAvailable)
    {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "location unavailable.");
        pos_estimate_error = 1;
    }
    // get stateVel
    if (ahrs.have_inertial_nav()) // Ground velocity in meters/second, North/East/Down
    {                             // order. Check if have_inertial_nav() is true before assigning values to stateVel.
        if (ahrs.get_velocity_NED(stateVel))
        {
            // gcs().send_text(MAV_SEVERITY_INFO, "Vel available.");
        }
    }
    else
    {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "inertial navigation is inactive");
        pos_estimate_error = 1;
    }
    // get Euler
    if (!ahrs.get_secondary_attitude(euler))
    {

        pos_estimate_error = 1;
    }

    ControllerIn.statePos = statePos;
    ControllerIn.stateVel = stateVel;
    ControllerIn.euler = euler;
    ControllerIn.timeInThisRun = timeInThisRun;

    VectorN<float, 4> thrustAndMomentCmd;

    if (is_in_horizon_flight(timeInThisRun))
    {
    }
    switch (g.GeoCtrl_ADP)
    {
    case 0:
    { // 关闭自适应
        thrustAndMomentCmd = GeometricTrajectoryController(ControllerIn);
    }
    break;
    case 1:
    { // 打开自适应
        thrustAndMomentCmd = AdaptiveController(ControllerIn, DBInput);
        if (is_in_horizon_flight(timeInThisRun))
        {
            // 测试，加入扰动
            thrustAndMomentCmd = add_Disturbance(DBInput, thrustAndMomentCmd, ControllerIn);
        }
    }
    break;
    case 2:
    { // 关闭自适应，但是还有扰动
        thrustAndMomentCmd = GeometricTrajectoryController(ControllerIn);
        if (is_in_horizon_flight(timeInThisRun))
        {
            // 测试，加入扰动
            thrustAndMomentCmd = add_Disturbance(DBInput, thrustAndMomentCmd, ControllerIn);
        }
    }
    break;
    default:
    { // 关闭自适应
        thrustAndMomentCmd = GeometricTrajectoryController(ControllerIn);
    }
    break;
    }

    // motor output
    VectorN<float, 4> motorPWM;
    motorPWM = motorMixSimple(thrustAndMomentCmd, REAL_OR_SITL);
    bool motorEnable = actuator_control(motorPWM, timeInThisRun, pos_estimate_error);

    // logging
    AP::logger().Write("FSAF", "TimeUS,radi,rece,mabl", "Qbbb",
                       AP_HAL::micros64(),
                       copter.failsafe.radio,
                       copter.ap.rc_receiver_present,
                       motorEnable);
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
    ControllerIn.land_is_ok_flag = land_is_ok_flag;
    // 降落检测--飞行高度
    if ((-flight_alt_now) - (-initalt) <= g.GeoCtrl_ALL)
    {
        land_is_ok_flag = 1;
        ControllerIn.land_is_ok_flag = land_is_ok_flag;
        return;
    }

    return;
}

bool ModeGeometric::is_in_horizon_flight(float flight_time)
{

    if (flight_time > g.GeoCtrl_TFT && in_horizon_flight)
    {
        ControllerIn.is_in_horizon_flight_flag = 1;
        return true;
    }
    else
    {
        ControllerIn.is_in_horizon_flight_flag = 0;
        return false;
    }
}

bool ModeGeometric::actuator_control(VectorN<float, 4> motorPWM, float timeInThisRun, bool pos_estimate_error)
{
    // disarm the vehicle by setting PWM to 1 when landing is completed

    bool motorEnable = 1;

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

    if (pos_estimate_error == 1)
    {
        motorEnable = 0;
    }

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
    return motorEnable;
}

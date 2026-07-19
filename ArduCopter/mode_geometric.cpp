#include "Copter.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Motors/AP_Motors_Class.h> // for sending motor speed
#include "mode.h"
#include "Geometric_Trajectory_Generate.h"
#include "ACRL_trajectories.h"
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>
// #include "tilt_control.h"
/*
 * Init and run calls for Geometric flight mode
 */
bool ModeGeometric::init(bool ignore_checks)
{

    initial_time_in_geometric = AP_HAL::micros();
    getposAvailable = ahrs.get_relative_position_NED_origin(enterpos);
    init_alt = enterpos.z; // 记录起飞点高度

    info_send_flag = 0;
    trajectory_num = g.GeoCtrl_NUM;
    take_off_time = g.GeoCtrl_TFT;
    radiusX = g.GeoCtrl_RDI;     // 轨迹参数，半径
    radiusY = g.GeoCtrl_RDY;     // 轨迹参数，半径
    targetSpeed = g.GeoCtrl_VEL; // 轨迹参数，速度
    kg_vehicleMass = g.GeoCtrl_MAS;
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

    VectorN<float, 5> thrustAndMomentCmd;

    uint32_t now_time_in_geometric = AP_HAL::micros();
    float timeInThisRun = (float)0.000001f * (now_time_in_geometric - initial_time_in_geometric);

    Vector3f targetPos;
    Vector3f targetVel;
    Vector3f targetAcc;
    Vector3f targetJerk;
    Vector3f targetSnap;
    Vector3f targetHead;
    Vector3f targetHead_dot;
    Vector3f targetHead_ddot;
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
        const float targetAlt = g.GeoCtrl_ALT;
        in_horizon_flight = false;
        Trajectory_Generate_POS_AUTO(timeInThisRun, targetAlt, take_off_time, T_circle, &in_horizon_flight, &targetPos, &targetVel, &targetAcc, &targetJerk, &targetSnap, &targetYaw, &targetYaw_dot, &targetYaw_ddot);
    }

    break;

    case 2:
        // circle
        {
            float r_circle = g.GeoCtrl_RDI;
            float T_circle = g.GeoCtrl_TIM;
            const float targetAlt = g.GeoCtrl_ALT;
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
            const float targetAlt = g.GeoCtrl_ALT;
            in_horizon_flight = false;
            in_trj_flight = false;
            Trajectory_Generate_EIGHT_AUTO(timeInThisRun, targetAlt, take_off_time, r_circle, T_circle, &in_horizon_flight, &in_trj_flight, &targetPos, &targetVel, &targetAcc, &targetJerk, &targetSnap, &targetYaw, &targetYaw_dot, &targetYaw_ddot);
        }

        break;
    case 4:
    {
        float r_circle = g.GeoCtrl_RDI;
        float T_circle = g.GeoCtrl_TIM;
        const float targetAlt = g.GeoCtrl_ALT;
        in_horizon_flight = false;
        in_trj_flight = false;
        Trajectory_Generate_LSR_AUTO(timeInThisRun, targetAlt, take_off_time, r_circle, T_circle, &in_horizon_flight, &in_trj_flight, &targetPos, &targetVel, &targetAcc, &targetJerk, &targetSnap, &targetYaw, &targetYaw_dot, &targetYaw_ddot);
    }
    break;
    case 5:
        // circular trajectory with variable yaw
        {
            if (timeInThisRun < 2)
            {
                // takeoff from (x,y,z) = (0,0,0) to (0,0,-1) in 2 secondss
                ACRL_trajectory_takeoff(timeInThisRun, &targetPos, &targetVel, &targetAcc, &targetJerk, &targetSnap, &targetYaw, &targetYaw_dot, &targetYaw_ddot);
            }
            else
            {

                // #if (!REAL_OR_SITL) // SITL
                //                 const float timeOffset = 2;
                //                 ACRL_trajectory_circle_variable_yaw(timeInThisRun, radiusX, timeOffset, targetSpeed, &targetPos, &targetVel, &targetAcc, &targetJerk, &targetSnap, &targetYaw, &targetYaw_dot, &targetYaw_ddot);
                // #elif (REAL_OR_SITL) // Real
                if (timeInThisRun >= 2 && timeInThisRun < 4)
                {
                    // transition from (0,0,-1) to (0,-radiusX,-1) in 2 seconds
                    const float timeOffset = 2;
                    ACRL_trajectory_transition_to_start(timeInThisRun, radiusX, timeOffset, &targetPos, &targetVel, &targetAcc, &targetJerk, &targetSnap, &targetYaw, &targetYaw_dot, &targetYaw_ddot);
                }
                else if (timeInThisRun >= 4)
                {
                    // start the circle trajectory
                    const float timeOffset = 4;
                    ACRL_trajectory_circle_variable_yaw(timeInThisRun, radiusX, timeOffset, targetSpeed, &targetPos, &targetVel, &targetAcc, &targetJerk, &targetSnap, &targetYaw, &targetYaw_dot, &targetYaw_ddot);
                }
                // #endif
            }
        }
        break;
    case 6:
        // circular trajectory with fixed yaw
        {
            if (timeInThisRun < 2)
            {
                // takeoff from (x,y,z) = (0,0,0) to (0,0,-1) in 2 secondss
                ACRL_trajectory_takeoff(timeInThisRun, &targetPos, &targetVel, &targetAcc, &targetJerk, &targetSnap, &targetYaw, &targetYaw_dot, &targetYaw_ddot);
            }
            else
            {

                // #if (!REAL_OR_SITL) // SITL
                //             const float timeOffset = 2;
                //             ACRL_trajectory_circle_fixed_yaw(timeInThisRun, radiusX, timeOffset, targetSpeed, &targetPos, &targetVel, &targetAcc, &targetJerk, &targetSnap, &targetYaw, &targetYaw_dot, &targetYaw_ddot);
                // #elif (REAL_OR_SITL) // Real
                if (timeInThisRun >= 2 && timeInThisRun < 4)
                {
                    // transition from (0,0,-1) to (0,-radiusX,-1) in 2 seconds
                    const float timeOffset = 2;
                    ACRL_trajectory_transition_to_start(timeInThisRun, radiusX, timeOffset, &targetPos, &targetVel, &targetAcc, &targetJerk, &targetSnap, &targetYaw, &targetYaw_dot, &targetYaw_ddot);
                }
                else if (timeInThisRun >= 4)
                {
                    // start the circle trajectory
                    const float timeOffset = 4;
                    ACRL_trajectory_circle_fixed_yaw(timeInThisRun, radiusX, timeOffset, targetSpeed, &targetPos, &targetVel, &targetAcc, &targetJerk, &targetSnap, &targetYaw, &targetYaw_dot, &targetYaw_ddot);
                }
                // #endif
            }
        }
        break;
    case 7: // figure8 trajectory with fixed yaw
    {
        ACRL_trajectory_figure8_fixed_yaw(timeInThisRun, radiusX, radiusY, targetSpeed, &targetPos, &targetVel, &targetAcc, &targetJerk, &targetSnap, &targetYaw, &targetYaw_dot, &targetYaw_ddot);
        break;
    }
    case 8: // cy45 add: figure8 trajectory with tilted altitude
    {
        ACRL_trajectory_figure8_tilted(timeInThisRun, radiusX, radiusY, targetSpeed, &targetPos, &targetVel, &targetAcc, &targetJerk, &targetSnap, &targetYaw, &targetYaw_dot, &targetYaw_ddot);
        break;
    }
    case 9: //
    {
        float T_circle = g.GeoCtrl_TIM;
        const float targetAlt = g.GeoCtrl_ALT;
        in_horizon_flight = false;
        Trajectory_Generate_POS_TILT_AUTO(timeInThisRun, targetAlt, take_off_time, T_circle, &in_horizon_flight, &targetPos, &targetVel, &targetAcc, &targetJerk, &targetSnap, &targetHead, &targetHead_dot, &targetHead_ddot);
        break;
    }
    case 10: // 固定翼前飞过渡测试 (动态倾转)
    {
        float transition_time = 30.0f;
        const float targetAlt = g.GeoCtrl_ALT;
        float forward_vel = g.GeoCtrl_VEL;
        in_horizon_flight = false;
        Trajectory_Generate_FW_TRANSITION(timeInThisRun, targetAlt, take_off_time, transition_time, forward_vel, &in_horizon_flight, &targetPos, &targetVel, &targetAcc, &targetJerk, &targetSnap, &targetHead, &targetHead_dot, &targetHead_ddot);
        break;
    }
    default:
        Trajectory_Generate_POS(&targetPos, &targetVel, &targetAcc, &targetJerk, &targetSnap, &targetYaw, &targetYaw_dot, &targetYaw_ddot);
        gcs().send_text(MAV_SEVERITY_CRITICAL, "VOID Trajectory NUM");
        break;
    }

    // fallback: 非倾转轨迹不设置 targetHead，默认机头指向北(NED x轴)
    if (targetHead.is_zero()) {
        targetHead = Vector3f{1, 0, 0};
        targetHead_dot = Vector3f{0, 0, 0};
        targetHead_ddot = Vector3f{0, 0, 0};
    }

    // initialize for landing mode
    if (g.LandFlag && !landingTriggered)
    {
        landingTriggered = 1;              // set landingTriggered to 1
        landingTimeOffset = timeInThisRun; // store the time offset
    }

    // executing landing mode
    if (g.LandFlag && landingTriggered) // switch to landing mode
    {
        if (ahrs.get_relative_position_NED_origin(currentPosition))
        {
            ;
        } // save current position
        if (ahrs.get_velocity_NED(currentVelocity))
        {
            ;
        }
        currentYaw = ahrs.get_yaw(); // save current yaw
        GEO_land_detect(init_alt);   // land_is_ok_flag

        if (land_is_ok_flag) // if the initial altitude upon entering land mode is within 30 cm, then set landComplete to 1 to overwrite the motor throttle to 1.
        {
            if (!landingComplete)
            {
                landingComplete = 1;
                gcs().send_text(MAV_SEVERITY_INFO, "Quadrotor is on the ground. Motor commands set to minimum.");
            }
        }
        else
        {
            float decRate = 1; // 1m/s^2
            landingComplete = ACRL_trajectory_land(timeInThisRun - landingTimeOffset, currentPosition, currentVelocity, currentYaw, decRate, &targetPos, &targetVel, &targetAcc, &targetJerk, &targetSnap, &targetYaw, &targetYaw_dot, &targetYaw_ddot);
        }
    }

    // 从进入geo模式的位置开始跟踪
    if (getposAvailable)
    {
        targetPos = targetPos + enterpos;
    }


    // ==========================================================
    // 数学解耦 targetHead：垂直方向代表倾转，水平方向代表航向
    // ==========================================================
    // 1. 提取倾转角 (NED坐标系中Z轴朝下，由前飞轨迹 targetHead.z = -sin(tilt) 得出，故 tilt = asin(-Z))
    float target_tilt = asinf(constrain_float(-targetHead.z, -1.0f, 1.0f));

    // 2. 提取纯水平航向 (强制投影到 XY 平面，避免 targetHead 朝下引发偏航发散奇异点)
    Vector3f geoHead = Vector3f(targetHead.x, targetHead.y, 0.0f);
    if (geoHead.is_zero()) {
        geoHead = Vector3f(1.0f, 0.0f, 0.0f); // 如果完全垂直，默认给个正北方向防止报错
    } else {
        geoHead.normalize();
    }
    
    // 导数也同步水平化
    Vector3f geoHead_dot = Vector3f(targetHead_dot.x, targetHead_dot.y, 0.0f);
    Vector3f geoHead_ddot = Vector3f(targetHead_ddot.x, targetHead_ddot.y, 0.0f);

    
    // ==========================================================
    // 几何控制计算与多旋翼/固定翼模糊分配逻辑
    // ==========================================================
    // 3. 将纯水平的期望航向喂给几何控制器，此时它只负责安全地进行水平姿态维稳
    thrustAndMomentCmd = ModeGeometric::GeometricTrajectoryController(targetPos, targetVel, targetAcc, targetJerk, targetSnap, geoHead, geoHead_dot, geoHead_ddot);

    // 4. 将解耦出的倾转角直接塞给控制指令的第 5 位
    thrustAndMomentCmd[4] = target_tilt;

    // 获取当前状态（利用地面速度近似空速）
    Vector3f statePos, stateVel;
    if (!ahrs.get_relative_position_NED_origin(statePos)) {
        statePos.zero();
    }
    if (!ahrs.get_velocity_NED(stateVel)) {
        stateVel.zero();
    }
    float current_airspeed = stateVel.length();

    // 通过外部函数获取控制输出系数
    float mc_coeff = 1.0f; // 多旋翼系数
    float fw_coeff = 0.0f; // 固定翼系数
    calculate_blend_coefficients(current_airspeed, mc_coeff, fw_coeff);

    // --- 1. 多旋翼扭矩叠加衰减 ---
    thrustAndMomentCmd[1] *= mc_coeff;
    thrustAndMomentCmd[2] *= mc_coeff;
    thrustAndMomentCmd[3] *= mc_coeff;

    // --- 2. 【核心修复】：解决严重掉高问题 ---
    // 原始推力是由几何控制器算出来的抗重力推力 (geo_thrust)
    float geo_thrust = thrustAndMomentCmd[0];
    
    // 补偿物理倾转带来的垂直升力损失：需要推力 = 期望垂直力 / cos(tilt)
    float cos_tilt = cosf(target_tilt);
    if (cos_tilt < 0.2f) cos_tilt = 0.2f; // 限制最大补偿为5倍，防止贴近90度时推力爆炸
    float lift_thrust = geo_thrust / cos_tilt;

    // 计算前向加速需要的额外推进力
    float err_vx = targetVel.x - stateVel.x;
    float fw_base_thrust = kg_vehicleMass * 9.8f * 0.3f + err_vx * 2.0f;
    fw_base_thrust = constrain_float(fw_base_thrust, 0.0f, kg_vehicleMass * 9.8f * 2.0f);

    // 终极推力融合策略：取两者中的最大值！
    // 几何控制器的Z轴PID自带对机翼升力的感知(机翼有升力了它算出的geo_thrust就会变小)。
    // 这里绝对不能用 mc_coeff 去衰减 lift_thrust，否则必掉高！
    thrustAndMomentCmd[0] = fmaxf(lift_thrust, fw_base_thrust * fw_coeff);

    // --- 2. 固定翼自稳与定高控制 (Auto-Level & Alt Hold) ---
    // 获取当前实际姿态与角速度
    float current_roll = ahrs.get_roll();
    float current_pitch = ahrs.get_pitch();
    Vector3f gyro = ahrs.get_gyro(); // gyro.x 是横滚角速度，gyro.y 是俯仰角速度

    // (1) 高度环：计算目标俯仰角 (Target Pitch)
    // NED 坐标系下 Z 轴朝下。如果 statePos.z > targetPos.z，说明飞机偏低，err_z 为正
    float err_z = statePos.z - targetPos.z;
    float err_vz = stateVel.z - targetVel.z;
    
    float Kp_alt = 0.08f;  // 高度误差 -> 俯仰角的比例增益
    float Kd_alt = 0.02f;
    // 飞机偏低 (err_z > 0) 时，需要正的 Pitch (抬头)。限制最大仰角为 ±15度 (约0.26 rad)
    float target_pitch = constrain_float(Kp_alt * err_z + Kd_alt * err_vz, -0.26f, 0.26f);

    // (2) 姿态环：计算舵面指令
    // 注意：Kp、Kd 的正负极性视你的 Gazebo SDF 设定而定。
    // 如果你在仿真中发现姿态发散，直接把这两个增益的符号反过来即可。
    float Kp_pitch_att = 1500.0f; // 俯仰姿态 P
    float Kd_pitch_att = 200.0f;  // 俯仰角速度 D
    float pitch_cmd = constrain_float(Kp_pitch_att * (target_pitch - current_pitch) + Kd_pitch_att * (0.0f - gyro.y), -400.0f, 400.0f);

    // 横滚永远目标为 0 (机翼水平)
    float Kp_roll_att = -1000.0f;  // 横滚姿态 P
    float Kd_roll_att = -100.0f;   // 横滚角速度 D
    float roll_cmd = constrain_float(Kp_roll_att * (0.0f - current_roll) + Kd_roll_att * (0.0f - gyro.x), -400.0f, 400.0f);

    // (3) 根据固定翼系数 (fw_coeff) 逐渐释放舵面控制权
    int16_t weighted_pitch = (int16_t)(pitch_cmd * fw_coeff);
    int16_t weighted_roll  = (int16_t)(roll_cmd * fw_coeff);

    // 混控到 PWM (1500 为中立位置)
    uint16_t pwm_elevon_left  = constrain_int16(1500 + weighted_pitch + weighted_roll, 1100, 1900);
    uint16_t pwm_elevon_right = constrain_int16(1500 + weighted_pitch - weighted_roll, 1100, 1900);
    uint16_t pwm_elevator     = constrain_int16(1500 + weighted_pitch, 1100, 1900);

    // 发送 PWM 信号至对应 Gazebo 仿真的 8, 9, 10 通道
    hal.rcout->write(8, pwm_elevon_left);
    hal.rcout->write(9, pwm_elevon_right);
    hal.rcout->write(10, pwm_elevator);
    // ==========================================================
    // ==========================================================
    // ==========================================================

    // motor mixing
    VectorN<float, 4> motorPWM;

    motorPWM = motorMixSimple(thrustAndMomentCmd);

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
        // uint8_t rc_tilt_in  = rc().channel(CH_9)->percent_input();
        // copter._tilt_setpoint = (uint16_t)(1500.0+700.0*((float)rc_tilt_in-50.0)/50.0);
        copter._tilt_setpoint = (uint16_t)tilt_angle_2_PWM(thrustAndMomentCmd[4]);
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

VectorN<float, 5> ModeGeometric::GeometricTrajectoryController(
    Vector3f targetPos,
    Vector3f targetVel,
    Vector3f targetAcc,
    Vector3f targetJerk,
    Vector3f targetSnap,
    Vector3f targetHead,
    Vector3f targetHead_dot,
    Vector3f targetHead_ddot)
{
    // NaN 保护：targetHead 为零向量会导致后续 normalize/cross 运算产生 NaN
    if (targetHead.is_zero()) {
        targetHead = Vector3f{1, 0, 0};
    }

    Vector3f r_error;
    Vector3f v_error;
    Vector3f target_force;
    Vector3f thrust_axis;
    Vector3f x_axis_desired;
    Vector3f y_axis_desired;
    Vector3f z_axis_desired;

    Vector3f eR, ew, M;

    Vector3f e3 = {0, 0, 1};

    Vector3f statePos;
    Vector3f stateVel;

    Vector2f positionNE;

    float tilt_angle;

    int locAvailable = ahrs.get_relative_position_NED_origin(statePos);
    if (!locAvailable)
    {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "location unavailable.");
        VectorN<float, 5> no_out_put;
        no_out_put[0] = 0;
        no_out_put[1] = 0;
        no_out_put[2] = 0;
        no_out_put[3] = 0;
        no_out_put[4] = 0;
        return no_out_put;
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

    // Velocity Error (ev)
    v_error = stateVel - targetVel;


    // =================================================================
    // 【核心修复】：动态剥夺多旋翼对X方向的控制权，解决多旋翼无X控制的问题
    // =================================================================
    if (trajectory_num == 10) 
    {
        uint32_t now_time = AP_HAL::micros();
        float time_in_trj = (float)0.000001f * (now_time - initial_time_in_geometric);
        
        // 计算当前是否已经超过了起飞悬停时间 (take_off_time)
        float dt_after_takeoff = time_in_trj - g.GeoCtrl_TFT;
        float x_weight = 1.0f;
        
        if (dt_after_takeoff > 0.0f) {
            // 起飞阶段结束，开始倾转。在刚开始倾转的 3 秒内，平滑地将X轴控制权剥夺为0
            // 这样机身就不会试图去强行抬头对抗倾转电机产生的前飞速度！
            // x_weight = 1.0f - (dt_after_takeoff / 3.0f);
            // x_weight = constrain_float(x_weight, 0.0f, 1.0f);
            x_weight = 0.0f;
        }

        r_error.x *= x_weight; 
        v_error.x *= x_weight; 
        targetAcc.x *= x_weight;  
        targetJerk.x *= x_weight; 
        targetSnap.x *= x_weight; 

        // 提示：我们故意没有对 r_error.y 和 v_error.y 进行衰减！
        // 这就保证了Y方向(横向)控制在多旋翼和倾转过程中，永远百分之百生效！
    }

    // kg_vehicleMass = g.GeoCtrl_MAS; // weight for the real drone

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
    Quaternion q;
    if (!ahrs.get_secondary_quaternion(q))
    {
        VectorN<float, 5> no_out_put;
        no_out_put[0] = 0;
        no_out_put[1] = 0;
        no_out_put[2] = 0;
        no_out_put[3] = 0;
        no_out_put[4] = 0;
        return no_out_put;
    }
    q.rotation_matrix(R); // transforming the quaternion q to rotation matrix R

    // Matrix3f R_body2World = R;
    // Vector3f targetHead_norm = targetHead.normalized();
    // float input_Pitch_rad = asinf(-targetHead_norm[2]); // NED系：机头向上，z 为负。仰角为正
    // tilt_angle = input_Pitch_rad;
    // Matrix3f R_motor2body;
    // R_motor2body.from_euler(0, -tilt_angle, 0); // 旋翼坐标系转机体坐标系

    // // gcs().send_text(MAV_SEVERITY_INFO, "cosPitch:%f,sinPitch:%f", cosPitch, sinPitch);
    // // Matrix3f R_motor2body = Matrix3f(Vector3f(cosPitch, 0, sinPitch),
    // //                                  Vector3f(0, 1, 0),
    // //                                  Vector3f(-sinPitch, 0, cosPitch));

    // Matrix3f R_motor2world = R_body2World * R_motor2body; // 旋翼坐标系转机体坐标系
    Matrix3f R_body2World = R;
    
    // 强制禁用过驱动旋转，让几何控制器认为电机永远朝上
    tilt_angle = 0.0f; 
    Matrix3f R_motor2body;
    R_motor2body.identity(); // 单位阵
    Matrix3f R_motor2world = R_body2World;

    thrust_axis = R_motor2world * e3; // 计算推力方向 b3

    // target thrust [F]
    float target_thrust = -target_force * thrust_axis;
    // float target_thrust = target_force.length();

    // Calculate axis [zB_des]
    // z_axis_desired = R_motor2body * (-target_force);
    // Matrix3f R_body2world_des;
    Matrix3f R_motor2world_des;
    Vector3f motor2world_des_x;
    Vector3f motor2world_des_y;
    Vector3f motor2world_des_z;

    motor2world_des_z = (-target_force);
    motor2world_des_z.normalize();

    motor2world_des_x = targetHead;

    motor2world_des_y = motor2world_des_z % motor2world_des_x;
    motor2world_des_y.normalize();

    motor2world_des_x = motor2world_des_y % motor2world_des_z;
    motor2world_des_x.normalize();

    R_motor2world_des.a = motor2world_des_x;
    R_motor2world_des.b = motor2world_des_y;
    R_motor2world_des.c = motor2world_des_z;
    R_motor2world_des.transpose();

    // R_body2world_des = R_motor2world_des * R_motor2body.transposed();

    // [eR]
    // Matrix3f Rdes(Vector3f(x_axis_desired.x, y_axis_desired.x, z_axis_desired.x),
    //               Vector3f(x_axis_desired.y, y_axis_desired.y, z_axis_desired.y),
    //               Vector3f(x_axis_desired.z, y_axis_desired.z, z_axis_desired.z));
    Matrix3f Rdes = R_motor2world_des;

    // [xC_des]
    Vector3f x_c_des = targetHead;
    Vector3f x_c_des_dot = targetHead_dot;   // time derivative of x_c_des
    Vector3f x_c_des_ddot = targetHead_ddot; // time derivative of x_c_des_dot
    Matrix3f eRM = (R_motor2world_des.transposed() * R_motor2world - R_motor2world.transposed() * R_motor2world_des) / 2;
    eR = veeOperator(eRM);

    Vector3f Omega_in_body = AP::ahrs().get_gyro();
    Vector3f Omega_in_motor = R_motor2body.transposed() * Omega_in_body;

    // compute Omegad: this comes from Appendix F in https://arxiv.org/pdf/1003.2005v3.pdf
    Vector3f a_error; // error on acceleration
    // a_error = e3 * GRAVITY_MAGNITUDE - R.colz() * target_thrust / kg_vehicleMass - targetAcc;
    a_error = e3 * GRAVITY_MAGNITUDE - R_motor2world.colz() * target_thrust / kg_vehicleMass - targetAcc;

    Vector3f target_force_dot; // derivative of target_force
    target_force_dot.x = -g.GeoCtrl_Kpx * v_error.x - g.GeoCtrl_Kvx * a_error.x + kg_vehicleMass * targetJerk.x;
    target_force_dot.y = -g.GeoCtrl_Kpy * v_error.y - g.GeoCtrl_Kvy * a_error.y + kg_vehicleMass * targetJerk.y;
    target_force_dot.z = -g.GeoCtrl_Kpz * v_error.z - g.GeoCtrl_Kvz * a_error.z + kg_vehicleMass * targetJerk.z;

    // 推力轴方向b3 = R_motor2world * e3
    Vector3f b3_dot = R_motor2world * hatOperator(Omega_in_motor) * e3;

    float target_thrust_dot = -target_force_dot * R_motor2world.colz() - target_force * b3_dot;

    Vector3f j_error; // error on jerk
    j_error = -R_motor2world.colz() * target_thrust_dot / kg_vehicleMass - b3_dot * target_thrust / kg_vehicleMass - targetJerk;

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

    // in motor frame
    Vector3f Omegad = veeOperator(R_motor2world_des.transposed() * Rd_dot);
    Vector3f Omegad_dot = veeOperator(R_motor2world_des.transposed() * Rd_ddot - hatOperator(Omegad) * hatOperator(Omegad));

    // eomega (angular velocity error)
    ew = Omega_in_motor - R_motor2world.transposed() * R_motor2world_des * Omegad;

    Matrix3f J_motor_frame;
    J_motor_frame = R_motor2body.transposed() * J * R_motor2body; // 转到旋翼坐标系下的惯量矩阵

    // Compute the moment
    M.x = -g.GeoCtrl_KRx * eR.x - g.GeoCtrl_KOx * ew.x;
    M.y = -g.GeoCtrl_KRy * eR.y - g.GeoCtrl_KOy * ew.y;
    M.z = -g.GeoCtrl_KRz * eR.z - g.GeoCtrl_KOz * ew.z;
    M = M - J_motor_frame * (hatOperator(Omega_in_motor) * R_motor2world.transposed() * R_motor2world_des * Omegad - R_motor2world.transposed() * R_motor2world_des * Omegad_dot);
    Vector3f momentAdd = Omega_in_motor % (J_motor_frame * Omega_in_motor); // J is the inertia matrix
    M = M + momentAdd;

    VectorN<float, 5> thrustMomentCmd;
    thrustMomentCmd[0] = target_thrust;
    thrustMomentCmd[1] = M.x;
    thrustMomentCmd[2] = M.y;
    thrustMomentCmd[3] = M.z;
    thrustMomentCmd[4] = tilt_angle;

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
    AP::logger().Write("GECT", "TimeUS,tfx,tfy,tfz,tt,mx,my,mz,tlt,tax,tay,taz", "Qfffffffffff",
                       AP_HAL::micros64(),
                       (target_force.x),
                       (target_force.y),
                       (target_force.z),
                       (target_thrust),
                       (M.x),
                       (M.y),
                       (M.z),
                       (tilt_angle),
                       (thrust_axis.x),
                       (thrust_axis.y),
                       (thrust_axis.z));
    AP::logger().Write("GETA", "TimeUS,tpx,tpy,tpz,spx,spy,spz,tvx,tvy,tvz,svx,svy,svz", "Qffffffffffff",
                       AP_HAL::micros64(),
                       (targetPos.x),
                       (targetPos.y),
                       (targetPos.z),
                       (statePos.x),
                       (statePos.y),
                       (statePos.z),
                       (targetVel.x),
                       (targetVel.y),
                       (targetVel.z),
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
    AP::logger().Write("MOWD", "TimeUS,R11,R12,R13,R21,R22,R23,R31,R32,R33", "Qfffffffff",
                       AP_HAL::micros64(),
                       R_motor2world.a.x,
                       R_motor2world.a.y,
                       R_motor2world.a.z,
                       R_motor2world.b.x,
                       R_motor2world.b.y,
                       R_motor2world.b.z,
                       R_motor2world.c.x,
                       R_motor2world.c.y,
                       R_motor2world.c.z);
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

VectorN<float, 4> ModeGeometric::motorMixSimple(VectorN<float, 5> thrustMomentCmd)
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

    // quad_output_mat_fm2f3[0] = 0.5 / (D * cosf(thrustMomentCmd[4]));
    // quad_output_mat_fm2f3[1] = -0.5 / (D * cosf(thrustMomentCmd[4]));
    // quad_output_mat_fm2f3[2] = 0.5 / (D * cosf(thrustMomentCmd[4]));
    // quad_output_mat_fm2f3[3] = -0.5 / (D * cosf(thrustMomentCmd[4]));
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
uint16_t ModeGeometric::tilt_angle_2_PWM(float tilt_angle)
{
    float ans;
    ans = 1850 + (-2100.0 / M_PI) * tilt_angle;
    if (ans > 1850)
    {
        ans = 1850;
    }
    if (ans < 800)
    {
        ans = 800;
    }
    return (uint16_t)ans;
}
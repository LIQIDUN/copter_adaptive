#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Motors/AP_Motors_Class.h> // for sending motor speed
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Operator/AP_Operator.h>

struct GeoInput
{
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

    float kg_vehicleMass;

    float GRAVITY_MAGNITUDE;

    float kpx;
    float kpy;
    float kpz;

    float kvx;
    float kvy;
    float kvz;

    float krx;
    float kry;
    float krz;

    float kox;
    float koy;
    float koz;

    Matrix3f J;

    float timeInThisRun;

    float c1;
    float c2;
    float gar;
    float gax;
    float bx;

    bool land_is_ok_flag = 0;
    bool is_in_horizon_flight_flag = 0;

    /* data */
};
struct DisturbanceInput
{
    int8_t DB_type; // 0:const 1:sine
    float total_thrust_dist;
    float x_axis_torque_dist;
    float y_axis_torque_dist;
    float z_axis_torque_dist;
};
VectorN<float, 4> GeometricTrajectoryController(GeoInput ControllIn); // controller for trajectory control

const float sin_time_T = 4;
VectorN<float, 4> AdaptiveController(GeoInput ControllIn, DisturbanceInput DisturbanceIn);

VectorN<float, 4> add_Disturbance(DisturbanceInput DisturbanceIn, VectorN<float, 4> thrustAndMomentCmd,GeoInput ControllIn);
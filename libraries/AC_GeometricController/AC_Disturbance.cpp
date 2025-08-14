#include "AC_GeoCtrl.h"

VectorN<float, 4> add_Disturbance(DisturbanceInput DisturbanceIn, VectorN<float, 4> thrustAndMomentCmd, GeoInput ControllIn)
{
    if (DisturbanceIn.DB_type == 0) // constant disturbance
    {
        thrustAndMomentCmd[0] += DisturbanceIn.total_thrust_dist;
        thrustAndMomentCmd[1] += DisturbanceIn.x_axis_torque_dist;
        thrustAndMomentCmd[2] += DisturbanceIn.y_axis_torque_dist;
        thrustAndMomentCmd[3] += DisturbanceIn.z_axis_torque_dist;
    }
    else if (DisturbanceIn.DB_type == 1) // sine disturbance
    {
        float sin_time = ControllIn.timeInThisRun;
        float sin_time_w = 2 * M_PI / sin_time_T;
        thrustAndMomentCmd[0] += DisturbanceIn.total_thrust_dist * sinf(sin_time_w * sin_time);
        thrustAndMomentCmd[1] += DisturbanceIn.x_axis_torque_dist * sinf(sin_time_w * sin_time);
        thrustAndMomentCmd[2] += DisturbanceIn.y_axis_torque_dist * sinf(sin_time_w * sin_time);
        thrustAndMomentCmd[3] += DisturbanceIn.z_axis_torque_dist * sinf(sin_time_w * sin_time);
    }
    return thrustAndMomentCmd;
}

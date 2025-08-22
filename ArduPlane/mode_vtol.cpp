#include "mode.h"
#include "Plane.h"



bool ModeVTOL::_enter()
{
// #if HAL_SOARING_ENABLED
//     // for ArduSoar soaring_controller
//     plane.g2.soaring_controller.init_cruising();
// #endif

//     plane.set_target_altitude_current();
    initial_time_in_geometric=  AP_HAL::micros();
    return true;
}

void ModeVTOL::update()
{
    // gcs().send_text(MAV_SEVERITY_DEBUG, "VTOL update called");
    // Thanks to Yury MonZon for the altitude limit code!
    // plane.nav_roll_cd = plane.channel_roll->norm_input() * plane.roll_limit_cd;
    // plane.update_load_factor();
    // plane.update_fbwb_speed_height();
    uint32_t now_time_in_geometric=  AP_HAL::micros();
    float timeInThisRun = (float)0.000001f * (now_time_in_geometric - initial_time_in_geometric);
    float time = (float)0.000001f * (now_time_in_geometric-last_time_in_geometric);

    gcs().send_text(MAV_SEVERITY_DEBUG, "VTOL update called %f %f",time,timeInThisRun);


    motors->rc_write(0,1145);
    motors->rc_write(1,1145);
    motors->rc_write(2,1145);
    motors->rc_write(3,1145);
    motors->rc_write(4,1145);
    motors->rc_write(5,1145);
    motors->rc_write(6,1145);
    motors->rc_write(7,1145);

    last_time_in_geometric = now_time_in_geometric;
}


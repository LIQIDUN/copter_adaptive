/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include <AP_Param/AP_Param.h>
#include "transition.h"

class QuadPlane;
class AP_MotorsMulticopter;
class Tiltrotor_Transition;
class Tiltrotor
{
friend class QuadPlane;
friend class Plane;
friend class Tiltrotor_Transition;
public:

    Tiltrotor(QuadPlane& _quadplane, AP_MotorsMulticopter*& _motors);

    bool enabled() const { return (enable > 0) && setup_complete;}

    void setup();

    void slew(float tilt);
    void binary_slew(bool forward);
    void update();
    void continuous_update();
    void binary_update();
    void vectoring();
    void bicopter_output();
    void tilt_compensate_angle(float *thrust, uint8_t num_motors, float non_tilted_mul, float tilted_mul);
    void tilt_compensate(float *thrust, uint8_t num_motors);
    bool tilt_over_max_angle(void) const;

    bool is_motor_tilting(uint8_t motor) const {
        return tilt_mask.get() & (1U<<motor);
    }

    bool fully_fwd() const;
    bool fully_up() const;
    float tilt_max_change(bool up, bool in_flap_range = false) const;
    float get_fully_forward_tilt() const;
    float get_forward_flight_tilt() const;

    // update yaw target for tiltrotor transition
    void update_yaw_target();

    bool is_vectored() const { return enabled() && _is_vectored; }

    bool has_fw_motor() const { return _have_fw_motor; }

    bool has_vtol_motor() const { return _have_vtol_motor; }

    bool motors_active() const { return enabled() && _motors_active; }

    // DCPTilt research forward-transition helpers.
    bool dcptilt_enabled() const { return dcptilt_enable > 0; }
    float dcptilt_tilt_profile(float progress) const;
    void dcptilt_set_tilt_direct(float tilt);
    float dcptilt_capture_forward_output() const;
    // Declaration is unconditional on purpose. tiltrotor.h is included
    // before Plane.h in tiltrotor.cpp, so HAL_LOGGING_ENABLED is not
    // guaranteed to be defined when this header is parsed. The
    // implementation and call remain guarded in tiltrotor.cpp.
    void dcptilt_write_log();

    // true if the tilts have completed slewing
    // always return true if not enabled or not a continuous type
    bool tilt_angle_achieved() const { return !enabled() || (type != TILT_TYPE_CONTINUOUS) || angle_achieved; }

    AP_Int8 enable;
    AP_Int16 tilt_mask;
    AP_Int16 max_rate_up_dps;
    AP_Int16 max_rate_down_dps;
    AP_Int8  max_angle_deg;
    AP_Int8  type;
    AP_Float tilt_yaw_angle;
    AP_Float fixed_angle;
    AP_Float fixed_gain;
    AP_Float flap_angle_deg;

    // DCPTilt research forward-transition parameters
    AP_Int8 dcptilt_enable;
    AP_Float dcptilt_transition_time_s;
    AP_Float dcptilt_handover_time_s;

    // DCPTilt transition state shared with the tilt-output path and logger
    bool dcptilt_transition_active = false;
    uint32_t dcptilt_transition_start_ms = 0;
    float dcptilt_progress = 0.0f;
    float dcptilt_elapsed_s = 0.0f;
    float dcptilt_target_tilt = 0.0f;

    // DCPTilt post-transition throttle handover. This is deliberately
    // separate from dcptilt_progress: the primary tilt transition still
    // completes exactly at Q_TILT_DCPT_TIME.
    bool dcptilt_handover_active = false;
    uint32_t dcptilt_handover_start_ms = 0;
    float dcptilt_handover_start_throttle = 0.0f;
    float dcptilt_handover_progress = 0.0f;
    float dcptilt_handover_output = 0.0f;

    // Heading captured at the start of the DCPTilt forward transition.
    bool dcptilt_yaw_lock_active = false;
    float dcptilt_yaw_target_cd = 0.0f;

    uint32_t dcptilt_last_log_ms = 0;

    float current_tilt;
    float current_throttle;
    bool _motors_active:1;
    float transition_yaw_cd;
    uint32_t transition_yaw_set_ms;
    bool _is_vectored;

    // types of tilt mechanisms
    enum {TILT_TYPE_CONTINUOUS    =0,
          TILT_TYPE_BINARY        =1,
          TILT_TYPE_VECTORED_YAW  =2,
          TILT_TYPE_BICOPTER      =3
    };

    static const struct AP_Param::GroupInfo var_info[];

private:

    bool setup_complete;

    // true if a fixed forward motor is setup
    bool _have_fw_motor;

    // true if all motors tilt with no fixed VTOL motor
    bool _have_vtol_motor;

    // true if the current tilt angle is equal to the desired
    // with slow tilt rates the tilt angle can lag
    bool angle_achieved;

    // refences for convenience
    QuadPlane& quadplane;
    AP_MotorsMulticopter*& motors;

    Tiltrotor_Transition* transition;

};

// Transition for separate left thrust quadplanes
class Tiltrotor_Transition : public SLT_Transition
{
friend class Tiltrotor;
public:

    Tiltrotor_Transition(QuadPlane& _quadplane, AP_MotorsMulticopter*& _motors, Tiltrotor& _tiltrotor):SLT_Transition(_quadplane, _motors), tiltrotor(_tiltrotor) {};

    void update() override;
    void VTOL_update() override;
    void force_transition_complete() override;
    void restart() override;

    bool update_yaw_target(float& yaw_target_cd) override;

    bool show_vtol_view() const override;

private:

    void dcptilt_update();
    void dcptilt_reset_state();

    // Once the research time-scheduled transition has completed, the
    // standard SLT state machine is used only as the post-transition
    // Q_ASSIST/safety path. It must not start another DCPTilt transition.
    bool dcptilt_primary_transition_complete = false;

    Tiltrotor& tiltrotor;

};
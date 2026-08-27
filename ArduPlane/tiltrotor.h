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
    float dcptilt_update_td3_profile(uint32_t now_ms);
    void dcptilt_update_control_weights();
    float dcptilt_fis_fww(float velocity_mps, float tilt_normalized) const;
    float dcptilt_strategy_speed() const;
    float dcptilt_lift_speed() const;
    void dcptilt_update_altitude_controller();
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
    AP_Int8 dcptilt_mode;
    AP_Int8 dcptilt_profile;

    // DCPTilt common altitude / vertical-force controller parameters.
    // These are shared by all controller-allocation / tilt-profile experiments
    // so the selected strategy and tilt profile remain the intended variables.
    AP_Float dcptilt_alt_p;
    AP_Float dcptilt_alt_d;
    AP_Float dcptilt_accel_max;
    AP_Float dcptilt_fw_alt_p;
    AP_Float dcptilt_fw_alt_d;
    AP_Float dcptilt_fw_pitch_max_deg;
    AP_Float dcptilt_vlift_mps;
    AP_Float dcptilt_cos_regularizer;
    AP_Float dcptilt_lift_ratio_max;
    AP_Float dcptilt_thrust_filter_s;
    AP_Float dcptilt_terminal_window;
    AP_Float dcptilt_terminal_gain;

    // Common DCPTilt transition-heading rudder controller. These parameters
    // are not part of the 3x6 experimental allocation variables.
    AP_Float dcptilt_yaw_p;
    AP_Float dcptilt_yaw_d;
    AP_Float dcptilt_yaw_max;

    // Strategy-specific experimental parameters. SWITCH keeps the historical
    // three-state weights, exposes both speed thresholds, and makes the middle
    // fixed-wing weight configurable. Fake NMPC reuses MODE=0 legacy FUZZ
    // allocation and adds only the early longitudinal nose-up transient.
    AP_Float dcptilt_switch_low_mps;
    AP_Float dcptilt_switch_high_mps;
    AP_Float dcptilt_switch_mid_fw;
    AP_Float dcptilt_switch_pitch_kick_deg;
    AP_Float dcptilt_switch_pitch_kick_time_s;
    AP_Float dcptilt_nmpc_pitch_gain_deg;
    AP_Float dcptilt_nmpc_pitch_max_deg;

    // TD3 trajectory-generation parameter (PROF=6..8 only).
    AP_Float dcptilt_td3_rate_scale;

    // DCPTilt transition state shared with the tilt-output path and logger
    bool dcptilt_transition_active = false;
    uint32_t dcptilt_transition_start_ms = 0;
    float dcptilt_progress = 0.0f;
    float dcptilt_elapsed_s = 0.0f;
    float dcptilt_target_tilt = 0.0f;

    // Closed-loop TD3 tilt-profile state (PROF=6..8).
    // Actor inference runs at 20 Hz. Training observations are:
    //   eh = z_NED-z_ref = h_ref-h
    //   Vnorm = 1.2*V/20
    //   MotorInput = DCPTilt normalized motor-thrust command + 0.3858
    // The network output is converted to normalized tilt rate by
    // Q_TILT_DCPT_TD3S, then integrated into lambda in [0,1].
    uint32_t dcptilt_td3_last_update_ms = 0;
    float dcptilt_td3_lambda = 0.0f;
    float dcptilt_td3_eh_m = 0.0f;
    float dcptilt_td3_vnorm = 0.0f;
    float dcptilt_td3_motor_norm = 0.0f;
    float dcptilt_td3_output = 0.0f;
    float dcptilt_td3_lambda_rate = 0.0f;
    float dcptilt_td3_delta_lambda = 0.0f;

    // DCPTilt controller-allocation state. These two coefficients reproduce
    // the selected FUZZ / SWITCH / NMPC / FIS allocation strategy.
    float dcptilt_strategy_speed_mps = 0.0f;
    float dcptilt_mc_weight = 1.0f;
    float dcptilt_fw_weight = 0.0f;

    // Common DCPTilt altitude and vertical-force state. Altitude is local
    // NED Z-up in metres. The target is captured at transition entry.
    bool dcptilt_alt_target_valid = false;
    float dcptilt_alt_target_m = 0.0f;
    float dcptilt_altitude_m = 0.0f;
    float dcptilt_alt_error_m = 0.0f;
    float dcptilt_vz_up_mps = 0.0f;
    float dcptilt_accel_up_cmd_mss = 0.0f;
    float dcptilt_lift_speed_mps = 0.0f;
    float dcptilt_wing_lift_ratio = 0.0f;
    float dcptilt_thrust_ratio_tau = 1.0f;
    float dcptilt_throttle_raw = 0.0f;
    float dcptilt_throttle_cmd = 0.0f;
    int32_t dcptilt_fw_pitch_target_cd = 0;
    float dcptilt_fw_pitch_base_deg = 0.0f;
    float dcptilt_nmpc_pitch_shape = 0.0f;
    float dcptilt_nmpc_pitch_bias_deg = 0.0f;

    // MODE=1 historical hard-switch handover transient.
    uint32_t dcptilt_switch_kick_start_ms = 0;
    float dcptilt_switch_kick_initial_deg = 0.0f;
    float dcptilt_switch_pitch_bias_deg = 0.0f;
    float dcptilt_switch_last_step = 0.0f;

    bool dcptilt_thrust_saturated = false;
    uint32_t dcptilt_alt_last_ms = 0;

    // Terminal-height predictor state. The raw altitude target/error above
    // remain untouched for logging and experiment comparability.
    float dcptilt_alt_control_error_m = 0.0f;
    float dcptilt_terminal_time_remaining_s = 0.0f;
    float dcptilt_terminal_pred_alt_m = 0.0f;
    float dcptilt_terminal_error_m = 0.0f;
    float dcptilt_terminal_blend = 0.0f;

    // Yaw diagnostic/control state. MC yaw is intentionally independent of
    // the experimental MCW; the fixed-wing rudder independently holds the
    // same transition-entry heading and is blended in with FWW.
    float dcptilt_mc_yaw_raw = 0.0f;
    float dcptilt_mc_yaw_weighted = 0.0f;
    float dcptilt_rudder_raw = 0.0f;
    float dcptilt_rudder_weight = 0.0f;
    float dcptilt_rudder_output = 0.0f;

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

    // DCPTilt controller-allocation strategy selector
    enum DCPTiltMode : uint8_t {
        DCPT_MODE_FUZZ = 0,
        DCPT_MODE_SWITCH = 1,
        DCPT_MODE_NMPC = 2,
        DCPT_MODE_FIS = 3
    };

    // DCPTilt tilt-profile selector
    enum DCPTiltProfile : uint8_t {
        DCPT_PROFILE_LINEAR = 0,
        DCPT_PROFILE_SMOOTHSTEP = 1,
        DCPT_PROFILE_POPT_A = 2,
        DCPT_PROFILE_POPT_B = 3,
        DCPT_PROFILE_POPT_C = 4,
        DCPT_PROFILE_POPT_D = 5,
        DCPT_PROFILE_TD3_A = 6,
        DCPT_PROFILE_TD3_B = 7,
        DCPT_PROFILE_TD3_C = 8
    };

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

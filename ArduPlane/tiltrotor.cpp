#include "tiltrotor.h"
#include "Plane.h"
#include "dcptilt_td3_weights.h"
#include <AP_Logger/AP_Logger.h>

#if HAL_QUADPLANE_ENABLED
const AP_Param::GroupInfo Tiltrotor::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable Tiltrotor functionality
    // @Values: 0:Disable, 1:Enable
    // @Description: This enables Tiltrotor functionality
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("ENABLE", 1, Tiltrotor, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: MASK
    // @DisplayName: Tiltrotor mask
    // @Description: This is a bitmask of motors that are tiltable in a tiltrotor (or tiltwing). The mask is in terms of the standard motor order for the frame type.
    // @User: Standard
    // @Bitmask: 0:Motor 1, 1:Motor 2, 2:Motor 3, 3:Motor 4, 4:Motor 5, 5:Motor 6, 6:Motor 7, 7:Motor 8, 8:Motor 9, 9:Motor 10, 10:Motor 11, 11:Motor 12
    AP_GROUPINFO("MASK", 2, Tiltrotor, tilt_mask, 0),

    // @Param: RATE_UP
    // @DisplayName: Tiltrotor upwards tilt rate
    // @Description: This is the maximum speed at which the motor angle will change for a tiltrotor when moving from forward flight to hover
    // @Units: deg/s
    // @Increment: 1
    // @Range: 10 300
    // @User: Standard
    AP_GROUPINFO("RATE_UP", 3, Tiltrotor, max_rate_up_dps, 40),

    // @Param: MAX
    // @DisplayName: Tiltrotor maximum VTOL angle
    // @Description: This is the maximum angle of the tiltable motors at which multicopter control will be enabled. Beyond this angle the plane will fly solely as a fixed wing aircraft and the motors will tilt to their maximum angle at the TILT_RATE
    // @Units: deg
    // @Increment: 1
    // @Range: 20 80
    // @User: Standard
    AP_GROUPINFO("MAX", 4, Tiltrotor, max_angle_deg, 45),

    // @Param: TYPE
    // @DisplayName: Tiltrotor type
    // @Description: This is the type of tiltrotor when TILT_MASK is non-zero. A continuous tiltrotor can tilt the rotors to any angle on demand. A binary tiltrotor assumes a retract style servo where the servo is either fully forward or fully up. In both cases the servo can't move faster than Q_TILT_RATE. A vectored yaw tiltrotor will use the tilt of the motors to control yaw in hover, Bicopter tiltrotor must use the tailsitter frame class (10)
    // @Values: 0:Continuous,1:Binary,2:VectoredYaw,3:Bicopter
    AP_GROUPINFO("TYPE", 5, Tiltrotor, type, TILT_TYPE_CONTINUOUS),

    // @Param: RATE_DN
    // @DisplayName: Tiltrotor downwards tilt rate
    // @Description: This is the maximum speed at which the motor angle will change for a tiltrotor when moving from hover to forward flight. When this is zero the Q_TILT_RATE_UP value is used.
    // @Units: deg/s
    // @Increment: 1
    // @Range: 10 300
    // @User: Standard
    AP_GROUPINFO("RATE_DN", 6, Tiltrotor, max_rate_down_dps, 0),

    // @Param: YAW_ANGLE
    // @DisplayName: Tilt minimum angle for vectored yaw
    // @Description: This is the angle of the tilt servos when in VTOL mode and at minimum output (fully back). This needs to be set in addition to Q_TILT_TYPE=2, to enable vectored control for yaw in tilt quadplanes. This is also used to limit the forward travel of bicopter tilts(Q_TILT_TYPE=3) when in VTOL modes.
    // @Range: 0 30
    AP_GROUPINFO("YAW_ANGLE", 7, Tiltrotor, tilt_yaw_angle, 0),

    // @Param: FIX_ANGLE
    // @DisplayName: Fixed wing tiltrotor angle
    // @Description: This is the angle the motors tilt down when at maximum output for forward flight. Set this to a non-zero value to enable vectoring for roll/pitch in forward flight on tilt-vectored aircraft
    // @Units: deg
    // @Range: 0 30
    // @User: Standard
    AP_GROUPINFO("FIX_ANGLE", 8, Tiltrotor, fixed_angle, 0),

    // @Param: FIX_GAIN
    // @DisplayName: Fixed wing tiltrotor gain
    // @Description: This is the gain for use of tilting motors in fixed wing flight for tilt vectored quadplanes
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("FIX_GAIN", 9, Tiltrotor, fixed_gain, 0),

    // @Param: WING_FLAP
    // @DisplayName: Tiltrotor tilt angle that will be used as flap
    // @Description: For use on tilt wings, the wing will tilt up to this angle for flap, transition will be complete when the wing reaches this angle from the forward fight position, 0 disables
    // @Units: deg
    // @Increment: 1
    // @Range: 0 15
    // @User: Standard
    AP_GROUPINFO("WING_FLAP", 10, Tiltrotor, flap_angle_deg, 0),

    // @Param: DCPT_EN
    // @DisplayName: DCPTilt research transition enable
    // @Description: Selects the forward VTOL-to-fixed-wing transition path. 0 uses the native ArduPilot SLT transition; 1 uses DCPTilt. The selection is latched at the start of each armed forward transition, so changing this parameter in flight does not switch controllers mid-transition. Fixed-wing-to-VTOL transitions always use the native ArduPilot path.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("DCPT_EN", 11, Tiltrotor, dcptilt_enable, 0),

    // @Param: DCPT_TIME
    // @DisplayName: DCPTilt forward transition time
    // @Description: Total duration of the DCPTilt forward transition. The transition starts immediately when entering fixed-wing flight from VTOL and completes when this time has elapsed.
    // @Units: s
    // @Range: 1 120
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("DCPT_TIME", 12, Tiltrotor, dcptilt_transition_time_s, 30.0f),

    // @Param: DCPT_HNDT
    // @DisplayName: DCPTilt throttle handover time
    // @Description: Time used after the primary DCPTilt tilt transition completes to smoothly blend the actual tilting-motor output at completion into the fixed-wing throttle demand. This handover does not extend DCPTilt transition progress. Set to zero to disable.
    // @Units: s
    // @Range: 0 10
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("DCPT_HNDT", 13, Tiltrotor, dcptilt_handover_time_s, 3.0f),

    // @Param: DCPT_MODE
    // @DisplayName: DCPTilt controller allocation strategy
    // @Description: Selects the controller-allocation strategy used only during the DCPTilt forward transition.
    // @Values: 0:FUZZ,1:SWITCH,2:NMPC,3:FIS
    // @User: Advanced
    AP_GROUPINFO("DCPT_MODE", 14, Tiltrotor, dcptilt_mode, DCPT_MODE_FUZZ),

    // @Param: DCPT_PROF
    // @DisplayName: DCPTilt tilt profile
    // @Description: Selects the DCPTilt tilt-angle trajectory. Profiles 0-5 are the six original time-scheduled trajectories. Profiles 6-8 are the three supplied TD3 actors evaluated at 20 Hz with incremental lambda integration.
    // @Values: 0:Linear,1:Smoothstep,2:POptA,3:POptB,4:POptC,5:POptD,6:TD3A,7:TD3B,8:TD3C
    // @User: Advanced
    AP_GROUPINFO("DCPT_PROF", 15, Tiltrotor, dcptilt_profile, DCPT_PROFILE_LINEAR),

    // @Param: DCPT_ALTP
    // @DisplayName: DCPTilt altitude acceleration P
    // @Description: Altitude-error gain used by the common DCPTilt vertical-force controller. Positive altitude error means the aircraft is below the transition-entry altitude and commands upward acceleration.
    // @Units: m/s/s/m
    // @Range: 0 3
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("DCPT_ALTP", 16, Tiltrotor, dcptilt_alt_p, 0.50f),

    // @Param: DCPT_ALTD
    // @DisplayName: DCPTilt vertical-speed damping
    // @Description: Vertical-speed damping gain used by the common DCPTilt altitude controller.
    // @Range: 0 5
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("DCPT_ALTD", 17, Tiltrotor, dcptilt_alt_d, 0.80f),

    // @Param: DCPT_AMAX
    // @DisplayName: DCPTilt maximum vertical acceleration
    // @Description: Absolute limit on the upward/downward acceleration correction generated by the DCPTilt altitude loop.
    // @Units: m/s/s
    // @Range: 0.2 6
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("DCPT_AMAX", 18, Tiltrotor, dcptilt_accel_max, 3.0f),

    // @Param: DCPT_FWAP
    // @DisplayName: DCPTilt fixed-wing altitude pitch P
    // @Description: Fixed-wing pitch-target gain in degrees per metre of altitude error during DCPTilt. This controller is independent of the selected FBWA/FBWB mode logic.
    // @Units: deg/m
    // @Range: 0 10
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("DCPT_FWAP", 19, Tiltrotor, dcptilt_fw_alt_p, 2.0f),

    // @Param: DCPT_FWAD
    // @DisplayName: DCPTilt fixed-wing vertical-speed damping
    // @Description: Fixed-wing pitch-target damping in degrees per m/s of vertical speed during DCPTilt.
    // @Range: 0 15
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("DCPT_FWAD", 20, Tiltrotor, dcptilt_fw_alt_d, 3.0f),

    // @Param: DCPT_PMAX
    // @DisplayName: DCPTilt fixed-wing pitch limit
    // @Description: Maximum absolute fixed-wing pitch target generated by the DCPTilt altitude controller.
    // @Units: deg
    // @Range: 3 30
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("DCPT_PMAX", 21, Tiltrotor, dcptilt_fw_pitch_max_deg, 15.0f),

    // @Param: DCPT_VLFT
    // @DisplayName: DCPTilt wing lift reference speed
    // @Description: Reference speed used by the common Lwing/mg=(V/VLFT)^2 model. The estimate is capped by DCPT_LMAX.
    // @Units: m/s
    // @Range: 5 40
    // @Increment: 0.5
    // @User: Advanced
    AP_GROUPINFO("DCPT_VLFT", 22, Tiltrotor, dcptilt_vlift_mps, 22.0f),

    // @Param: DCPT_CREG
    // @DisplayName: DCPTilt cosine regularizer
    // @Description: Positive regularization term used in sqrt(cos(theta)^2+CREG^2) to prevent large-angle thrust singularity and high-frequency amplification.
    // @Range: 0.03 0.5
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("DCPT_CREG", 23, Tiltrotor, dcptilt_cos_regularizer, 0.12f),

    // @Param: DCPT_LMAX
    // @DisplayName: DCPTilt maximum modeled wing lift ratio
    // @Description: Maximum Lwing/(mg) used during the transition. Values below 1 retain a small rotor reserve and reduce sensitivity to an optimistic wing-lift model.
    // @Range: 0.5 1
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("DCPT_LMAX", 24, Tiltrotor, dcptilt_lift_ratio_max, 0.95f),

    // @Param: DCPT_TFLT
    // @DisplayName: DCPTilt thrust command filter
    // @Description: First-order time constant applied only to the common DCPTilt total-thrust command. Tilt angle itself is not filtered, so the six selected tilt profiles are unchanged. Zero disables filtering.
    // @Units: s
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("DCPT_TFLT", 25, Tiltrotor, dcptilt_thrust_filter_s, 0.15f),

    // @Param: DCPT_TWIN
    // @DisplayName: DCPTilt terminal predictor window
    // @Description: Fraction of the DCPTilt transition duration over which the terminal-height predictor is blended in. The predictor estimates end altitude as h+Vz*t_remaining and is shared by all controller-allocation strategies and tilt profiles.
    // @Range: 0.05 0.50
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("DCPT_TWIN", 26, Tiltrotor, dcptilt_terminal_window, 0.25f),

    // @Param: DCPT_TGN
    // @DisplayName: DCPTilt terminal predictor gain
    // @Description: Dimensionless gain applied to predicted terminal-height error before it is added to the normal altitude error. Zero disables terminal correction. A value of 1 uses the full predicted end-height error.
    // @Range: 0 3
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("DCPT_TGN", 27, Tiltrotor, dcptilt_terminal_gain, 1.0f),

    // @Param: DCPT_YAWP
    // @DisplayName: DCPTilt legacy transition-rudder P
    // @Description: Legacy v3.28 hand-written rudder-PD gain retained for parameter compatibility. v3.29 and later AP-yaw-rate control does not use this parameter.
    // @Range: 0 400
    // @Increment: 5
    // @User: Advanced
    AP_GROUPINFO("DCPT_YAWP", 28, Tiltrotor, dcptilt_yaw_p, 100.0f),

    // @Param: DCPT_YAWD
    // @DisplayName: DCPTilt legacy transition-rudder D
    // @Description: Legacy v3.28 hand-written rudder-PD damping gain retained for parameter compatibility. v3.29 and later AP-yaw-rate control does not use this parameter.
    // @Range: 0 500
    // @Increment: 5
    // @User: Advanced
    AP_GROUPINFO("DCPT_YAWD", 29, Tiltrotor, dcptilt_yaw_d, 150.0f),

    // @Param: DCPT_YMAX
    // @DisplayName: DCPTilt transition rudder limit
    // @Description: Maximum absolute DCPTilt rudder command before the fixed-wing allocation weight is applied. ArduPlane scaled surface output uses 4500 as full deflection.
    // @Range: 0 4500
    // @Increment: 100
    // @User: Advanced
    AP_GROUPINFO("DCPT_YMAX", 30, Tiltrotor, dcptilt_yaw_max, 3000.0f),

    // @Param: DCPT_SWLO
    // @DisplayName: DCPTilt hard-switch lower speed
    // @Description: Lower airspeed threshold for SWITCH mode. At or below this speed controller allocation is MCW=1 and FWW=0. Between SWLO and SWHI the fixed-wing weight is DCPT_SWMD and MCW=1-DCPT_SWMD.
    // @Units: m/s
    // @Range: 0 40
    // @Increment: 0.5
    // @User: Advanced
    AP_GROUPINFO("DCPT_SWLO", 31, Tiltrotor, dcptilt_switch_low_mps, 6.0f),

    // @Param: DCPT_SWHI
    // @DisplayName: DCPTilt hard-switch upper speed
    // @Description: Upper airspeed threshold for SWITCH mode. At or above this speed controller allocation is MCW=0 and FWW=1. Between SWLO and SWHI the fixed-wing weight is DCPT_SWMD and MCW=1-DCPT_SWMD.
    // @Units: m/s
    // @Range: 0.5 50
    // @Increment: 0.5
    // @User: Advanced
    AP_GROUPINFO("DCPT_SWHI", 32, Tiltrotor, dcptilt_switch_high_mps, 17.0f),

    // @Param: DCPT_NPIT
    // @DisplayName: DCPTilt NMPC pitch-transient gain
    // @Description: Positive pitch-bias gain used only in NMPC mode. MODE=2 uses MODE=0 FUZZ allocation; this gain shapes only the early MCW*sin(tilt) nose-up transient, which smoothly fades to zero by 35 percent transition progress.
    // @Units: deg
    // @Range: 0 80
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("DCPT_NPIT", 33, Tiltrotor, dcptilt_nmpc_pitch_gain_deg, 35.0f),

    // @Param: DCPT_NMAX
    // @DisplayName: DCPTilt NMPC pitch-transient limit
    // @Description: Maximum positive pitch bias generated by the early NMPC-like transient. Default 4 degrees. Set to zero to disable the transient without changing MODE=2's MODE=0 FUZZ controller-allocation weights.
    // @Units: deg
    // @Range: 0 20
    // @Increment: 0.5
    // @User: Advanced
    AP_GROUPINFO("DCPT_NMAX", 34, Tiltrotor, dcptilt_nmpc_pitch_max_deg, 4.0f),

    // @Param: DCPT_SWMD
    // @DisplayName: DCPTilt hard-switch middle FW weight
    // @Description: Fixed-wing controller weight used only between DCPT_SWLO and DCPT_SWHI in SWITCH mode. MCW is automatically 1 minus this value. 0.5 reproduces the historical hard-switch logic; values farther from 0.5 make one of the two controller-allocation steps larger.
    // @Range: 0 1
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("DCPT_SWMD", 35, Tiltrotor, dcptilt_switch_mid_fw, 0.50f),

    // @Param: DCPT_SWPK
    // @DisplayName: DCPTilt hard-switch pitch-kick gain
    // @Description: Nose-down fixed-wing pitch-target kick for a full positive FWW step of 1.0. The applied kick is scaled by delta-FWW. This models the longitudinal handover mismatch seen in the historical hard-switch implementation without adding periodic oscillation. Set to zero to disable.
    // @Units: deg
    // @Range: 0 10
    // @Increment: 0.25
    // @User: Advanced
    AP_GROUPINFO("DCPT_SWPK", 36, Tiltrotor, dcptilt_switch_pitch_kick_deg, 4.0f),

    // @Param: DCPT_SWKT
    // @DisplayName: DCPTilt hard-switch pitch-kick decay time
    // @Description: Smooth decay duration of the hard-switch pitch-target kick. The altitude controller remains unchanged and generates the recovery naturally.
    // @Units: s
    // @Range: 0.2 3
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("DCPT_SWKT", 37, Tiltrotor, dcptilt_switch_pitch_kick_time_s, 1.2f),

    // @Param: DCPT_TD3S
    // @DisplayName: DCPTilt TD3 tilt-rate scale
    // @Description: Scale applied to the TD3 actor Sigmoid output to obtain normalized tilt rate for profiles 6 through 8. lambda_dot = TD3S * actor_output. The result is integrated at 20 Hz. Default 0.084 reproduces the original candidate scale.
    // @Units: 1/s
    // @Range: 0 1
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("DCPT_TD3S", 38, Tiltrotor, dcptilt_td3_rate_scale, 0.084f),

    // @Param: DCPT_TD3V
    // @DisplayName: DCPTilt TD3 speed source
    // @Description: Selects the raw speed used only by TD3 profiles 6 through 8 before Vnorm=1.2*V/20. Mode 0 reproduces the old strategy-speed path (NED 3D velocity magnitude when available). Mode 1 uses the ArduPilot AHRS airspeed estimate and falls back to the old strategy-speed path only if no valid airspeed estimate is available. Mode 2 uses groundspeed for diagnosis. Mode 3 uses NED 3D velocity magnitude directly and falls back to groundspeed if NED velocity is unavailable.
    // @Values: 0:LegacyStrategy,1:AirspeedAuto,2:Groundspeed,3:NED3D
    // @Range: 0 3
    // @User: Advanced
    AP_GROUPINFO("DCPT_TD3V", 39, Tiltrotor, dcptilt_td3_speed_mode, 1),

    // @Param: DCPT_TD3VF
    // @DisplayName: DCPTilt TD3 platform flat-flight airspeed
    // @Description: Typical steady flat-flight airspeed of the CURRENT aircraft in m/s. The selected TD3 speed is mapped to the 25 m/s training-aircraft speed scale before applying the original Vnorm normalization. Set 25 to reproduce the old normalization. For the present SITL aircraft use about 18 m/s.
    // @Units: m/s
    // @Range: 5 50
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("DCPT_TD3F", 40, Tiltrotor, dcptilt_td3_flat_speed_mps, 25.0f),

    // @Param: DCPT_VEXP
    // @DisplayName: DCPTilt TD3 speed normalization exponent
    // @Description: Diagnostic exponent for the TD3 speed observation. Vnorm = 1.5*(Vused/Vflat)^VEXP. Set 1.0 for the current linear speed scaling. Set 2.0 to test a normalized dynamic-pressure/lift-like squared-speed scaling. TD3 profiles 6 through 8 only.
    // @Range: 1 2
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("DCPT_VEXP", 41, Tiltrotor, dcptilt_td3_speed_exponent, 1.0f),

    // @Param: DCPT_EHF
    // @DisplayName: DCPTilt TD3 Eh freeze time
    // @Description: Diagnostic only. Time after transition start at which the TD3 height-error observation Eh is frozen and held for the remainder of the transition. Set 0 to disable. For the proposed late-transition diagnosis use 20 seconds. TD3 profiles 6 through 8 only.
    // @Units: s
    // @Range: 0 60
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("DCPT_EHF", 42, Tiltrotor, dcptilt_td3_eh_freeze_s, 0.0f),

    // @Param: DCPT_EHL
    // @DisplayName: DCPTilt TD3 Eh input limit
    // @Description: Absolute limit applied only to the TD3 actor height-error observation. The raw NED-equivalent Eh remains available for logging and the altitude controller is NOT limited. Set 0 to disable clipping. The trained policy was primarily exposed to about +/-0.5 m, so 0.5 m is the recommended/default limit.
    // @Units: m
    // @Range: 0 2
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("DCPT_EHL", 43, Tiltrotor, dcptilt_td3_eh_limit_m, 0.5f),

    // @Param: DCPT_YHP
    // @DisplayName: DCPTilt heading-to-yaw-rate P
    // @Description: Outer-loop gain converting DCPTilt heading error in degrees to desired fixed-wing yaw rate in deg/s. The desired rate is then controlled by ArduPlane's native yawController rate PID. This outer loop has no integrator; steady disturbance rejection is provided by the native yaw-rate PID I term.
    // @Units: 1/s
    // @Range: 0 4
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("DCPT_YHP", 44, Tiltrotor, dcptilt_yaw_heading_p, 1.0f),

    // @Param: DCPT_YRM
    // @DisplayName: DCPTilt maximum FW yaw-rate demand
    // @Description: Absolute limit on the heading outer-loop yaw-rate demand sent to ArduPlane's native yawController during DCPTilt. This limit is applied before the fixed-wing FWW allocation.
    // @Units: deg/s
    // @Range: 1 60
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("DCPT_YRM", 45, Tiltrotor, dcptilt_yaw_rate_max_dps, 20.0f),

    AP_GROUPEND
};

/*
  control code for tiltrotors and tiltwings. Enabled by setting
  Q_TILT_MASK to a non-zero value
 */

Tiltrotor::Tiltrotor(QuadPlane& _quadplane, AP_MotorsMulticopter*& _motors):quadplane(_quadplane),motors(_motors)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void Tiltrotor::setup()
{

    if (!enable.configured() && ((tilt_mask != 0) || (type == TILT_TYPE_BICOPTER))) {
        enable.set_and_save(1);
    }

    if (enable <= 0) {
        return;
    }

    _is_vectored = tilt_mask != 0 && type == TILT_TYPE_VECTORED_YAW;

    // true if a fixed forward motor is configured, either throttle, throttle left  or throttle right.
    // bicopter tiltrotors use throttle left and right as tilting motors, so they don't count in that case.
    _have_fw_motor = SRV_Channels::function_assigned(SRV_Channel::k_throttle) ||
                    ((SRV_Channels::function_assigned(SRV_Channel::k_throttleLeft) || SRV_Channels::function_assigned(SRV_Channel::k_throttleRight))
                        && (type != TILT_TYPE_BICOPTER));


    // check if there are any permanent VTOL motors
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; ++i) {
        if (motors->is_motor_enabled(i) && ((tilt_mask & (1U<<1)) == 0)) {
            // enabled motor not set in tilt mask
            _have_vtol_motor = true;
            break;
        }
    }

    if (_is_vectored) {
        // we will be using vectoring for yaw
        motors->disable_yaw_torque();
    }

    if (tilt_mask != 0) {
        // setup tilt compensation
        motors->set_thrust_compensation_callback(FUNCTOR_BIND_MEMBER(&Tiltrotor::tilt_compensate, void, float *, uint8_t));
        if (type == TILT_TYPE_VECTORED_YAW) {
            // setup tilt servos for vectored yaw
            SRV_Channels::set_range(SRV_Channel::k_tiltMotorLeft,  1000);
            SRV_Channels::set_range(SRV_Channel::k_tiltMotorRight, 1000);
            SRV_Channels::set_range(SRV_Channel::k_tiltMotorRear,  1000);
            SRV_Channels::set_range(SRV_Channel::k_tiltMotorRearLeft, 1000);
            SRV_Channels::set_range(SRV_Channel::k_tiltMotorRearRight, 1000);
        }
    }

    transition = new Tiltrotor_Transition(quadplane, motors, *this);
    if (!transition) {
        AP_BoardConfig::allocation_error("tiltrotor transition");
    }
    quadplane.transition = transition;

    setup_complete = true;
}

/*
  calculate maximum tilt change as a proportion from 0 to 1 of tilt
 */
float Tiltrotor::tilt_max_change(bool up, bool in_flap_range) const
{
    float rate;
    if (up || max_rate_down_dps <= 0) {
        rate = max_rate_up_dps;
    } else {
        rate = max_rate_down_dps;
    }
    if (type != TILT_TYPE_BINARY && !up && !in_flap_range) {
        bool fast_tilt = false;
        if (plane.control_mode == &plane.mode_manual) {
            fast_tilt = true;
        }
        if (plane.arming.is_armed_and_safety_off() && !quadplane.in_vtol_mode() && !quadplane.assisted_flight) {
            fast_tilt = true;
        }
        if (fast_tilt) {
            // allow a minimum of 90 DPS in manual or if we are not
            // stabilising, to give fast control
            rate = MAX(rate, 90);
        }
    }
    return rate * plane.G_Dt * (1/90.0);
}

/*
  output a slew limited tiltrotor angle. tilt is from 0 to 1
 */
void Tiltrotor::slew(float newtilt)
{
    float max_change = tilt_max_change(newtilt<current_tilt, newtilt > get_fully_forward_tilt());
    current_tilt = constrain_float(newtilt, current_tilt-max_change, current_tilt+max_change);

    angle_achieved = is_equal(newtilt, current_tilt);

    // translate to 0..1000 range and output
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor_tilt, 1000 * current_tilt);
}

// DCPTilt tilt profile. The six profiles below are a direct consolidation
// of the six alternatives kept in the user's FUZZ/SWITCH/NMPC SITL sources.
// Input/output are both normalized: 0=vertical, 1=fully forward.
float Tiltrotor::dcptilt_tilt_profile(float progress) const
{
    const float p = constrain_float(progress, 0.0f, 1.0f);

    if (p <= 0.0f) {
        return 0.0f;
    }
    if (p >= 1.0f) {
        return 1.0f;
    }

    switch ((DCPTiltProfile)dcptilt_profile.get()) {
    case DCPT_PROFILE_LINEAR:
        return p;

    case DCPT_PROFILE_SMOOTHSTEP:
        // 3p^2 - 2p^3
        return p * p * (3.0f - 2.0f * p);

    case DCPT_PROFILE_POPT_A:
    case DCPT_PROFILE_POPT_B:
    case DCPT_PROFILE_POPT_C:
    case DCPT_PROFILE_POPT_D: {
        static const float progress_nodes[7] = {
            0.0f, 0.166667f, 0.333333f, 0.5f, 0.666667f, 0.833333f, 1.0f
        };
        static const float profile_a[7] = {
            0.0f, 0.110f, 0.166f, 0.351f, 0.709f, 0.940f, 1.0f
        };
        static const float profile_b[7] = {
            0.0f, 0.04f, 0.09f, 0.16f, 0.30f, 0.55f, 1.0f
        };
        static const float profile_c[7] = {
            0.0f, 0.40f, 0.65f, 0.80f, 0.90f, 0.95f, 1.0f
        };
        static const float profile_d[7] = {
            0.0f, 0.1029f, 0.21985f, 0.33117f, 0.53781f, 0.78641f, 1.0f
        };

        const float *lambda_nodes = profile_a;
        switch ((DCPTiltProfile)dcptilt_profile.get()) {
        case DCPT_PROFILE_POPT_B:
            lambda_nodes = profile_b;
            break;
        case DCPT_PROFILE_POPT_C:
            lambda_nodes = profile_c;
            break;
        case DCPT_PROFILE_POPT_D:
            lambda_nodes = profile_d;
            break;
        default:
            break;
        }

        for (uint8_t i = 0; i < 6; i++) {
            if (p <= progress_nodes[i + 1]) {
                const float local =
                    (p - progress_nodes[i]) /
                    (progress_nodes[i + 1] - progress_nodes[i]);
                return lambda_nodes[i] +
                       local * (lambda_nodes[i + 1] - lambda_nodes[i]);
            }
        }
        return 1.0f;
    }

    default:
        return p;
    }
}

namespace {

static constexpr uint32_t DCPTILT_TD3_PERIOD_MS = 50U;
static constexpr float DCPTILT_TD3_DT_S = 0.05f;

float dcptilt_td3_actor_forward(uint8_t actor_index,
                                float eh_m,
                                float vnorm,
                                float motor_norm)
{
    actor_index = MIN(actor_index, uint8_t(2));
    const DCPTiltTD3::ActorWeights &actor = DCPTiltTD3::actors[actor_index];

    const float input[3] = {eh_m, vnorm, motor_norm};
    float h1[64];
    float h2[64];

    for (uint8_t i = 0; i < 64; i++) {
        float sum = actor.b1[i];
        const float *row = &actor.w1[i * 3U];
        for (uint8_t j = 0; j < 3; j++) {
            sum += row[j] * input[j];
        }
        h1[i] = MAX(sum, 0.0f);
    }

    for (uint8_t i = 0; i < 64; i++) {
        float sum = actor.b2[i];
        const float *row = &actor.w2[i * 64U];
        for (uint8_t j = 0; j < 64; j++) {
            sum += row[j] * h1[j];
        }
        h2[i] = MAX(sum, 0.0f);
    }

    float z = actor.b3;
    for (uint8_t j = 0; j < 64; j++) {
        z += actor.w3[j] * h2[j];
    }

    return 1.0f / (1.0f + expf(-z));
}

} // namespace

// PROF=6..8: incremental TD3 closed-loop tilt trajectory.
// x = [h_ref-h (NED-equivalent), 1.2*V/20, DCPTilt normalized motor-thrust command + 0.3858].
// lambda_dot = Q_TILT_DCPT_TD3S * actor_output
// lambda(k+1) = constrain(lambda(k) + lambda_dot*0.05, 0, 1)
float Tiltrotor::dcptilt_update_td3_profile(uint32_t now_ms)
{
    // This flag is consumed by dcptilt_update() after the tilt command is
    // published. It is true only on an actual scheduled 20 Hz Actor update.
    dcptilt_td3_runtime_updated = false;

    const int8_t profile = dcptilt_profile.get();
    if (profile < DCPT_PROFILE_TD3_A || profile > DCPT_PROFILE_TD3_C) {
        return constrain_float(dcptilt_td3_lambda, 0.0f, 1.0f);
    }

    if (dcptilt_td3_last_update_ms == 0U) {
        dcptilt_td3_last_update_ms = now_ms;
        return constrain_float(dcptilt_td3_lambda, 0.0f, 1.0f);
    }

    if ((now_ms - dcptilt_td3_last_update_ms) < DCPTILT_TD3_PERIOD_MS) {
        return constrain_float(dcptilt_td3_lambda, 0.0f, 1.0f);
    }

    // Keep the sampling grid tied to 50 ms rather than accumulating loop jitter.
    dcptilt_td3_last_update_ms += DCPTILT_TD3_PERIOD_MS;

    // Start timing only after the scheduler has determined that this is a real
    // Actor sample. TotalUS therefore measures one policy execution, not the
    // much faster outer ArduPilot loop calls that simply reuse the previous
    // lambda.
    dcptilt_td3_runtime_updated = true;
    dcptilt_td3_runtime_start_us = AP_HAL::micros();
    dcptilt_td3_actor_us = 0U;
    dcptilt_td3_proj_us = 0U;
    dcptilt_td3_total_us = 0U;
    dcptilt_td3_runtime_seq++;

    const float altitude_m =
        quadplane.inertial_nav.get_position_z_up_cm() * 0.01f;

    // The original TD3 policy formed height error in NED coordinates:
    //
    //   Eh = z_actual - z_ref
    //
    // ArduPilot's altitude_m here is Z-up, while NED z is down-positive:
    //
    //   z_NED = -h_up
    //
    // Therefore the equivalent observation in this implementation is:
    //
    //   Eh = h_ref - h_actual
    //
    // A positive Eh means the aircraft is below the reference altitude,
    // matching the original NED training convention.
    dcptilt_td3_eh_raw_m =
        dcptilt_alt_target_valid ? (dcptilt_alt_target_m - altitude_m) : 0.0f;

    // Optional late-transition Eh freeze diagnostic.
    //
    // EHF=0: normal policy input, Eh follows the aircraft continuously.
    // EHF>0: when elapsed time first reaches EHF, capture the current NED-
    // equivalent Eh and hold that same value for the remaining transition.
    //
    // This does NOT alter the altitude controller. It changes only the
    // observation sent to the TD3 actor.
    const float eh_freeze_s =
        constrain_float(dcptilt_td3_eh_freeze_s.get(), 0.0f, 60.0f);

    float td3_eh_candidate_m = dcptilt_td3_eh_raw_m;

    if (eh_freeze_s > 0.0f) {
        if (!dcptilt_td3_eh_frozen && dcptilt_elapsed_s >= eh_freeze_s) {
            dcptilt_td3_eh_frozen_m = dcptilt_td3_eh_raw_m;
            dcptilt_td3_eh_frozen = true;
        }

        td3_eh_candidate_m =
            dcptilt_td3_eh_frozen ?
            dcptilt_td3_eh_frozen_m :
            dcptilt_td3_eh_raw_m;
    } else {
        dcptilt_td3_eh_frozen = false;
        dcptilt_td3_eh_frozen_m = 0.0f;
    }

    // Keep the actor observation inside the height-error region represented
    // during training. This clipping affects ONLY the TD3 actor input.
    // The altitude controller still sees and acts on the full real error.
    const float eh_limit_m =
        constrain_float(dcptilt_td3_eh_limit_m.get(), 0.0f, 2.0f);

    if (eh_limit_m > 0.0f) {
        dcptilt_td3_eh_m =
            constrain_float(td3_eh_candidate_m, -eh_limit_m, eh_limit_m);
    } else {
        dcptilt_td3_eh_m = td3_eh_candidate_m;
    }

    // TD3 speed-source diagnostic.
    //
    // The original policy's V is expected to be AIRSPEED. The old DCPTilt
    // implementation instead used dcptilt_strategy_speed(), whose first
    // choice is the 3D NED velocity magnitude (ground-relative).
    //
    // Sample all candidate quantities on the actor's 20 Hz grid so the log
    // can diagnose the semantics without changing the rest of the controller.
    float airspeed_mps = 0.0f;
    dcptilt_td3_airspeed_valid =
        quadplane.ahrs.airspeed_estimate(airspeed_mps) &&
        isfinite(airspeed_mps) &&
        airspeed_mps >= 0.0f;
    dcptilt_td3_airspeed_mps =
        dcptilt_td3_airspeed_valid ? airspeed_mps : 0.0f;

    Vector3f velocity_ned;
    if (quadplane.ahrs.get_velocity_NED(velocity_ned)) {
        dcptilt_td3_ned3_speed_mps = velocity_ned.length();
    } else {
        dcptilt_td3_ned3_speed_mps = 0.0f;
    }

    dcptilt_td3_legacy_speed_mps =
        MAX(dcptilt_strategy_speed(), 0.0f);

    const int8_t td3_speed_mode =
        constrain_int16(dcptilt_td3_speed_mode.get(), 0, 3);

    switch (td3_speed_mode) {
    case 1: // ArduPilot AHRS airspeed; universal SITL/real-airframe path
        dcptilt_td3_speed_used_mps =
            dcptilt_td3_airspeed_valid ?
            dcptilt_td3_airspeed_mps :
            dcptilt_td3_legacy_speed_mps;
        break;

    case 2: // groundspeed diagnostic
        dcptilt_td3_speed_used_mps =
            MAX(quadplane.ahrs.groundspeed(), 0.0f);
        break;

    case 3: // explicit NED 3D magnitude diagnostic
        dcptilt_td3_speed_used_mps =
            (dcptilt_td3_ned3_speed_mps > 0.0f) ?
            dcptilt_td3_ned3_speed_mps :
            MAX(quadplane.ahrs.groundspeed(), 0.0f);
        break;

    case 0:
    default: // exact legacy strategy-speed behavior
        dcptilt_td3_speed_used_mps =
            dcptilt_td3_legacy_speed_mps;
        break;
    }

    // Platform-speed diagnostic normalization.
    //
    // The training aircraft's nominal flat-flight airspeed was about 25 m/s.
    // VEq is retained for comparison with the previous linear mapping:
    //
    //   V_equiv = V_used * 25 / V_flat_current
    //
    // The actor speed observation is now diagnostic-configurable:
    //
    //   Vnorm = 1.5 * (V_used / V_flat_current)^p
    //
    // where p = Q_TILT_DCPT_VEXP.
    //
    // p=1:
    //   exactly reproduces v3.20_vscale:
    //   Vnorm = 1.5*V/Vflat = 1.2*V_equiv/20
    //
    // p=2:
    //   tests a normalized dynamic-pressure / lift-like V^2 relationship
    //   while preserving Vnorm=1.5 at the configured flat-flight speed.
    static constexpr float DCPTILT_TD3_TRAIN_FLAT_SPEED_MPS = 25.0f;

    const float platform_flat_speed_mps =
        MAX(dcptilt_td3_flat_speed_mps.get(), 1.0f);

    const float speed_used_nonnegative =
        MAX(dcptilt_td3_speed_used_mps, 0.0f);

    dcptilt_td3_speed_equiv_mps =
        speed_used_nonnegative *
        (DCPTILT_TD3_TRAIN_FLAT_SPEED_MPS / platform_flat_speed_mps);

    const float speed_ratio =
        speed_used_nonnegative / platform_flat_speed_mps;

    const float speed_exponent =
        constrain_float(dcptilt_td3_speed_exponent.get(), 1.0f, 2.0f);

    dcptilt_td3_vnorm =
        1.5f * powf(speed_ratio, speed_exponent);

    // Training observation 3 is intended to represent normalized motor speed.
    // For this ArduPilot implementation use the final normalized rotor-thrust
    // command that DCPTilt sends toward AP_Motors as the speed/load proxy.
    //
    // This quantity:
    //   - is already normalized to [0,1],
    //   - is independent of PWM_MIN/PWM_MAX,
    //   - follows the final commanded motor effort after the DCPTilt thrust
    //     model and filtering,
    //   - does not use Tau (which is normalized to mg and is near 1 in hover).
    //
    // The +0.3858 offset is part of the trained observation definition:
    //   MotorInput = MotorProxyNorm + 0.3858
    const float motor_proxy_norm = constrain_float(
        dcptilt_throttle_cmd,
        0.0f,
        1.0f);

    dcptilt_td3_motor_norm =
        motor_proxy_norm + 0.3858f;

    const uint8_t actor_index = uint8_t(profile - DCPT_PROFILE_TD3_A);

    // ActorUS: ONLY the neural-network forward pass.
    const uint32_t actor_start_us = AP_HAL::micros();
    dcptilt_td3_output = dcptilt_td3_actor_forward(
        actor_index,
        dcptilt_td3_eh_m,
        dcptilt_td3_vnorm,
        dcptilt_td3_motor_norm);
    const uint32_t actor_end_us = AP_HAL::micros();
    dcptilt_td3_actor_us = actor_end_us - actor_start_us;

    // ProjUS begins after the Actor has produced its scalar action.
    const uint32_t proj_start_us = actor_end_us;

    const float td3_rate_scale =
        constrain_float(dcptilt_td3_rate_scale.get(), 0.0f, 1.0f);
    dcptilt_td3_lambda_rate =
        td3_rate_scale * dcptilt_td3_output;
    dcptilt_td3_delta_lambda =
        dcptilt_td3_lambda_rate * DCPTILT_TD3_DT_S;

    dcptilt_td3_lambda = constrain_float(
        dcptilt_td3_lambda + dcptilt_td3_delta_lambda,
        0.0f,
        1.0f);

    // Projection/mapping timing ends once the feasible lambda command has
    // been formed. Servo-output publication is intentionally accounted for
    // later in TotalUS, not in ProjUS.
    dcptilt_td3_proj_us = AP_HAL::micros() - proj_start_us;

    return dcptilt_td3_lambda;
}

// Membership functions used by TestTiltFuzzy.fis. Keep these local to this
// translation unit so MODE=3 reproduces the supplied Mamdani FIS without
// introducing a dependency on an external fuzzy-logic library.
static float dcptilt_fis_linzmf(float x, float a, float b)
{
    if (x <= a) {
        return 1.0f;
    }
    if (x >= b) {
        return 0.0f;
    }
    return (b - x) / (b - a);
}

static float dcptilt_fis_linsmf(float x, float a, float b)
{
    if (x <= a) {
        return 0.0f;
    }
    if (x >= b) {
        return 1.0f;
    }
    return (x - a) / (b - a);
}

static float dcptilt_fis_zmf(float x, float a, float b)
{
    if (x <= a) {
        return 1.0f;
    }
    if (x >= b) {
        return 0.0f;
    }

    const float mid = 0.5f * (a + b);
    const float span = b - a;
    if (x <= mid) {
        const float u = (x - a) / span;
        return 1.0f - 2.0f * u * u;
    }

    const float u = (x - b) / span;
    return 2.0f * u * u;
}

static float dcptilt_fis_smf(float x, float a, float b)
{
    if (x <= a) {
        return 0.0f;
    }
    if (x >= b) {
        return 1.0f;
    }

    const float mid = 0.5f * (a + b);
    const float span = b - a;
    if (x <= mid) {
        const float u = (x - a) / span;
        return 2.0f * u * u;
    }

    const float u = (x - b) / span;
    return 1.0f - 2.0f * u * u;
}

static float dcptilt_fis_trimf(float x, float a, float b, float c)
{
    if (x <= a || x >= c) {
        return 0.0f;
    }
    if (x <= b) {
        return (x - a) / (b - a);
    }
    return (c - x) / (c - b);
}

static float dcptilt_fis_trapmf(float x, float a, float b, float c, float d)
{
    if (x <= a || x >= d) {
        return 0.0f;
    }
    if (x >= b && x <= c) {
        return 1.0f;
    }
    if (x < b) {
        return (x - a) / (b - a);
    }
    return (d - x) / (d - c);
}

// Reproduce TestTiltFuzzy.fis exactly as a two-input Mamdani controller:
//   Input1 Velocity  [0,30] m/s
//   Input2 TiltAngle [0,1] (0=vertical, 1=fully forward)
//   Output1 FWW      [0,1]
// AND=min, implication=min, aggregation=max, defuzzification=centroid.
// The centroid is evaluated on 101 uniformly spaced output samples y=0..1,
// matching the intended MATLAB evalfis reference used for this project.
float Tiltrotor::dcptilt_fis_fww(float velocity_mps, float tilt_normalized) const
{
    const float velocity = constrain_float(velocity_mps, 0.0f, 30.0f);
    const float tilt = constrain_float(tilt_normalized, 0.0f, 1.0f);

    // Input 1: Velocity.
    const float v_slow = dcptilt_fis_zmf(velocity,
                                         2.32224168126094f,
                                         10.3222416812609f);
    const float v_mid = dcptilt_fis_trapmf(velocity,
                                           7.95973f,
                                           11.6357f,
                                           12.0f,
                                           18.5727f);
    const float v_fast = dcptilt_fis_smf(velocity,
                                         10.5341506129597f,
                                         15.5341506129597f);

    // Input 2: normalized TiltAngle.
    const float t_little = dcptilt_fis_linzmf(tilt, 0.0556f, 0.3333f);
    const float t_mid = dcptilt_fis_trapmf(tilt,
                                           0.138330777680141f,
                                           0.282221277680141f,
                                           0.306661277680141f,
                                           0.518418277680141f);
    const float t_high = dcptilt_fis_linsmf(tilt,
                                             0.292601054481547f,
                                             0.582601054481547f);

    // Seven FIS rules, reduced only by max-combining rules that have the same
    // consequent. This is algebraically identical to the supplied rule list.
    const float alpha_quad = MIN(v_slow, t_little);
    const float alpha_tilt = MAX(MIN(v_slow, t_mid),
                                 MAX(MIN(v_mid, t_mid),
                                     MIN(v_mid, t_little)));
    const float alpha_fw = MAX(v_fast,
                               MAX(MIN(v_slow, t_high),
                                   MIN(v_mid, t_high)));

    float numerator = 0.0f;
    float denominator = 0.0f;

    // Output MFs and 101-point centroid, y = 0.00, 0.01, ..., 1.00.
    for (uint8_t i = 0; i <= 100; i++) {
        const float y = 0.01f * i;

        const float mu_quad = dcptilt_fis_linzmf(y, 0.0f, 0.4f);
        const float mu_tilt = dcptilt_fis_trimf(y,
                                                0.403339191564148f,
                                                0.550966608084358f,
                                                0.68804920913884f);
        const float mu_fw = dcptilt_fis_linsmf(y, 0.7f, 1.0f);

        const float implied_quad = MIN(alpha_quad, mu_quad);
        const float implied_tilt = MIN(alpha_tilt, mu_tilt);
        const float implied_fw = MIN(alpha_fw, mu_fw);
        const float aggregated = MAX(implied_quad,
                                     MAX(implied_tilt, implied_fw));

        numerator += y * aggregated;
        denominator += aggregated;
    }

    // The supplied input MFs cover the valid input domain. Keep a deterministic
    // fallback for defensive robustness if the FIS is edited in the future.
    if (denominator <= 1.0e-6f) {
        return 0.5f;
    }

    return constrain_float(numerator / denominator, 0.0f, 1.0f);
}

// Reproduce the "current_airspeed = stateVel.length()" quantity from the
// user's SITL strategy code. If NED velocity is unavailable, fall back to
// ArduPlane's airspeed estimate and then groundspeed.
float Tiltrotor::dcptilt_strategy_speed() const
{
    Vector3f velocity_ned;
    if (quadplane.ahrs.get_velocity_NED(velocity_ned)) {
        return velocity_ned.length();
    }

    float aspeed = 0.0f;
    if (quadplane.ahrs.airspeed_estimate(aspeed)) {
        return MAX(aspeed, 0.0f);
    }

    return MAX(quadplane.ahrs.groundspeed(), 0.0f);
}

// Controller-allocation strategies reproduced from the research code sets.
//
// FUZZ:   3..15 m/s, fw=t^3, mc=1-fw
// SWITCH: <=SWLO -> MC; SWLO..SWHI -> configurable SWMD/(1-SWMD); >=SWHI -> FW
// NMPC:   Fake-NMPC longitudinal variant. Reuse MODE=0's legacy FUZZ
//         allocation exactly, then add only a short early positive-pitch
//         transient. Roll/yaw therefore follow the same allocation path as
//         MODE=0 rather than the MODE=3 Mamdani FIS.
// FIS:    TestTiltFuzzy.fis Mamdani output1=FWW using Velocity + current tilt
void Tiltrotor::dcptilt_update_control_weights()
{
    dcptilt_strategy_speed_mps = dcptilt_strategy_speed();

    float mc = 1.0f;
    float fw = 0.0f;

    switch ((DCPTiltMode)dcptilt_mode.get()) {
    case DCPT_MODE_FUZZ: {
        const float min_speed = 3.0f;
        const float max_speed = 15.0f;
        if (dcptilt_strategy_speed_mps <= min_speed) {
            mc = 1.0f;
            fw = 0.0f;
        } else if (dcptilt_strategy_speed_mps >= max_speed) {
            mc = 0.0f;
            fw = 1.0f;
        } else {
            const float t = constrain_float(
                (dcptilt_strategy_speed_mps - min_speed) / (max_speed - min_speed),
                0.0f, 1.0f);
            fw = t * t * t;
            mc = 1.0f - fw;
        }
        break;
    }

    case DCPT_MODE_SWITCH: {
        const float switch_low = constrain_float(dcptilt_switch_low_mps.get(), 0.0f, 49.5f);
        // Keep the three-state hard-switch definition valid even if a user
        // accidentally sets SWHI <= SWLO. The saved parameter is untouched;
        // only the runtime threshold is guarded.
        const float switch_high = MAX(
            constrain_float(dcptilt_switch_high_mps.get(), 0.5f, 50.0f),
            switch_low + 0.5f);

        const float switch_mid_fw =
            constrain_float(dcptilt_switch_mid_fw.get(), 0.0f, 1.0f);

        if (dcptilt_strategy_speed_mps <= switch_low) {
            mc = 1.0f;
            fw = 0.0f;
        } else if (dcptilt_strategy_speed_mps >= switch_high) {
            mc = 0.0f;
            fw = 1.0f;
        } else {
            fw = switch_mid_fw;
            mc = 1.0f - switch_mid_fw;
        }
        break;
    }

    case DCPT_MODE_NMPC: {
        // Fake NMPC is intentionally built on MODE=0's legacy FUZZ allocation.
        // Keep these equations identical to DCPT_MODE_FUZZ so the only
        // NMPC-specific change is the early longitudinal pitch transient.
        const float min_speed = 3.0f;
        const float max_speed = 15.0f;
        if (dcptilt_strategy_speed_mps <= min_speed) {
            mc = 1.0f;
            fw = 0.0f;
        } else if (dcptilt_strategy_speed_mps >= max_speed) {
            mc = 0.0f;
            fw = 1.0f;
        } else {
            const float t = constrain_float(
                (dcptilt_strategy_speed_mps - min_speed) / (max_speed - min_speed),
                0.0f, 1.0f);
            fw = t * t * t;
            mc = 1.0f - fw;
        }
        break;
    }

    case DCPT_MODE_FIS:
        // output1 in TestTiltFuzzy.fis is the fixed-wing authority FWW.
        // Use ArduPilot's current normalized tilt state, not transition time,
        // so the fuzzy allocation depends on the same physical quantities as
        // the original FIS: Velocity and TiltAngle.
        fw = dcptilt_fis_fww(dcptilt_strategy_speed_mps, current_tilt);
        mc = 1.0f - fw;
        break;

    default:
        mc = 1.0f;
        fw = 0.0f;
        break;
    }

    const float previous_fw_weight =
        constrain_float(dcptilt_fw_weight, 0.0f, 1.0f);
    const float new_fw_weight = constrain_float(fw, 0.0f, 1.0f);

    // The historical SWITCH changed between dynamically different
    // longitudinal controllers, which produced a visible handover transient.
    // The present common-altitude architecture is much smoother, so preserve
    // that discontinuity as one short nose-down FW pitch-target mismatch
    // whenever fixed-wing authority jumps upward.
    if ((DCPTiltMode)dcptilt_mode.get() == DCPT_MODE_SWITCH) {
        const float delta_fw = new_fw_weight - previous_fw_weight;
        if (delta_fw > 0.05f) {
            const float kick_gain_deg =
                constrain_float(dcptilt_switch_pitch_kick_deg.get(), 0.0f, 10.0f);
            dcptilt_switch_last_step = delta_fw;
            dcptilt_switch_kick_initial_deg = -kick_gain_deg * delta_fw;
            dcptilt_switch_kick_start_ms = AP_HAL::millis();
        }
    } else {
        dcptilt_switch_kick_start_ms = 0;
        dcptilt_switch_kick_initial_deg = 0.0f;
        dcptilt_switch_pitch_bias_deg = 0.0f;
        dcptilt_switch_last_step = 0.0f;
    }

    dcptilt_mc_weight = constrain_float(mc, 0.0f, 1.0f);
    dcptilt_fw_weight = new_fw_weight;
}

// Speed used only by the common wing-lift estimator. Prefer the AHRS
// airspeed estimate because Lwing is aerodynamic. When unavailable, use
// horizontal NED velocity. This value is not a transition completion input.
float Tiltrotor::dcptilt_lift_speed() const
{
    float aspeed = 0.0f;
    if (quadplane.ahrs.airspeed_estimate(aspeed) && is_positive(aspeed)) {
        return aspeed;
    }

    Vector3f velocity_ned;
    if (quadplane.ahrs.get_velocity_NED(velocity_ned)) {
        return safe_sqrt(velocity_ned.x * velocity_ned.x +
                         velocity_ned.y * velocity_ned.y);
    }

    return MAX(quadplane.ahrs.groundspeed(), 0.0f);
}

// Common altitude / vertical-force controller used by every DCPTilt
// strategy/profile combination. RC throttle and target fixed-wing mode
// are intentionally absent from this calculation.
//
// Normalized model:
//   tau*cos(theta) + lambda_w = 1 + a_z/g
// where tau=T/(mg), lambda_w=Lwing/(mg), and positive a_z is upward.
//
// A smooth regularized cosine magnitude is used instead of 1/cos(theta):
//   tau = residual / sqrt(cos(theta)^2 + eps^2)
// This behaves like 1/cos away from 90deg, but remains bounded at large tilt.
void Tiltrotor::dcptilt_update_altitude_controller()
{
    const uint32_t now = AP_HAL::millis();

    dcptilt_altitude_m = quadplane.inertial_nav.get_position_z_up_cm() * 0.01f;
    dcptilt_vz_up_mps = quadplane.inertial_nav.get_velocity_z_up_cms() * 0.01f;

    if (!dcptilt_alt_target_valid) {
        dcptilt_alt_target_m = dcptilt_altitude_m;
        dcptilt_alt_target_valid = true;
    }

    dcptilt_alt_error_m = dcptilt_alt_target_m - dcptilt_altitude_m;

    // Common terminal-height predictor for all 3x6 experiments. The raw
    // altitude reference never changes. During only the final portion of the
    // scheduled transition we predict where the aircraft would finish if the
    // current vertical speed persisted:
    //
    //   h_end_pred = h + Vz * t_remaining
    //   e_terminal = h_ref - h_end_pred
    //
    // A smoothstep blend adds this predicted terminal error to the ordinary
    // altitude error. The resulting control error is then used by BOTH the
    // rotor vertical-force branch and the fixed-wing pitch branch below.
    // This keeps the terminal constraint common to FUZZ/SWITCH/NMPC and to
    // all six tilt profiles rather than tuning a separate controller per case.
    const float total_time_s = constrain_float(dcptilt_transition_time_s.get(), 0.1f, 300.0f);
    dcptilt_terminal_time_remaining_s = MAX(total_time_s - dcptilt_elapsed_s, 0.0f);
    dcptilt_terminal_pred_alt_m =
        dcptilt_altitude_m + dcptilt_vz_up_mps * dcptilt_terminal_time_remaining_s;
    dcptilt_terminal_error_m =
        dcptilt_alt_target_m - dcptilt_terminal_pred_alt_m;

    const float terminal_window =
        constrain_float(dcptilt_terminal_window.get(), 0.05f, 0.50f);
    const float terminal_start = 1.0f - terminal_window;
    const float terminal_phase = constrain_float(
        (dcptilt_progress - terminal_start) / terminal_window,
        0.0f, 1.0f);
    dcptilt_terminal_blend =
        terminal_phase * terminal_phase * (3.0f - 2.0f * terminal_phase);

    const float terminal_gain = MAX(dcptilt_terminal_gain.get(), 0.0f);
    dcptilt_alt_control_error_m =
        dcptilt_alt_error_m +
        dcptilt_terminal_blend * terminal_gain * dcptilt_terminal_error_m;

    const float accel_limit = MAX(dcptilt_accel_max.get(), 0.1f);
    dcptilt_accel_up_cmd_mss = constrain_float(
        dcptilt_alt_p.get() * dcptilt_alt_control_error_m -
        dcptilt_alt_d.get() * dcptilt_vz_up_mps,
        -accel_limit, accel_limit);

    const float vertical_force_ratio = MAX(
        0.0f, 1.0f + dcptilt_accel_up_cmd_mss / GRAVITY_MSS);

    dcptilt_lift_speed_mps = dcptilt_lift_speed();
    const float vlift = MAX(dcptilt_vlift_mps.get(), 1.0f);
    const float speed_ratio = dcptilt_lift_speed_mps / vlift;
    const float lift_cap =
        constrain_float(dcptilt_lift_ratio_max.get(), 0.0f, 1.0f);
    dcptilt_wing_lift_ratio = constrain_float(
        speed_ratio * speed_ratio, 0.0f, lift_cap);

    // Rotor cannot generate negative thrust. If the wing model exceeds the
    // requested vertical force, the fixed-wing pitch loop unloads the wing.
    const float rotor_vertical_ratio = MAX(
        vertical_force_ratio - dcptilt_wing_lift_ratio, 0.0f);

    const float tilt_rad =
        radians(constrain_float(dcptilt_target_tilt, 0.0f, 1.0f) * 90.0f);
    const float cos_tilt = MAX(cosf(tilt_rad), 0.0f);
    const float eps =
        constrain_float(dcptilt_cos_regularizer.get(), 0.01f, 1.0f);
    const float regularized_cos =
        safe_sqrt(cos_tilt * cos_tilt + eps * eps);

    dcptilt_thrust_ratio_tau =
        rotor_vertical_ratio / MAX(regularized_cos, 0.01f);

    // AP_Motors throttle is a normalized thrust request. Q_M_THST_HOVER is
    // therefore the in-system calibration for tau=1 (T=mg). The standard
    // AP_Motors thrust curve and mixer remain responsible for actuator PWM.
    const float hover_thrust = motors->get_throttle_hover();
    const float throttle_max =
        constrain_float(motors->get_throttle_thrust_max(), 0.0f, 1.0f);
    const float raw_unconstrained =
        dcptilt_thrust_ratio_tau * hover_thrust;
    dcptilt_thrust_saturated = raw_unconstrained > throttle_max;
    dcptilt_throttle_raw =
        constrain_float(raw_unconstrained, 0.0f, throttle_max);

    // Mild thrust-only filtering suppresses model/measurement jitter near
    // large tilt. The selected tilt profile itself is not filtered.
    const float filter_tc = MAX(dcptilt_thrust_filter_s.get(), 0.0f);
    if (dcptilt_alt_last_ms == 0 || filter_tc <= 0.0f) {
        dcptilt_throttle_cmd = dcptilt_throttle_raw;
    } else {
        const float dt = constrain_float(
            (now - dcptilt_alt_last_ms) * 0.001f, 0.0f, 0.2f);
        const float alpha = dt / (filter_tc + dt);
        dcptilt_throttle_cmd +=
            alpha * (dcptilt_throttle_raw - dcptilt_throttle_cmd);
    }
    dcptilt_alt_last_ms = now;

    // Fixed-wing branch: same altitude target, converted to a pitch target.
    // This is separate from FBWA/FBWB nav_pitch. Keep the altitude-generated
    // target and the NMPC-like transient separate for diagnostics.
    const float pitch_limit_deg =
        constrain_float(dcptilt_fw_pitch_max_deg.get(), 1.0f, 45.0f);
    dcptilt_fw_pitch_base_deg = constrain_float(
        dcptilt_fw_alt_p.get() * dcptilt_alt_control_error_m -
        dcptilt_fw_alt_d.get() * dcptilt_vz_up_mps,
        -pitch_limit_deg, pitch_limit_deg);

    // Fake-NMPC pitch transient. MODE=2 already uses MODE=0's FUZZ MCW/FWW,
    // so use that same MC authority only to shape a short positive pitch bias:
    //     raw_shape = MCW * sin(tilt)
    //
    // The bias must reproduce only the early nose-up characteristic, not fight
    // the common altitude controller throughout the 30 s transition. Apply a
    // fixed progress window shared by every tilt profile:
    //     progress <= 0.15 : window = 1
    //     0.15..0.35       : smoothstep fade 1 -> 0
    //     progress >= 0.35 : window = 0
    //
    // Final shape = raw_shape * window, then bias=min(NPIT*shape,NMAX).
    dcptilt_nmpc_pitch_shape = 0.0f;
    dcptilt_nmpc_pitch_bias_deg = 0.0f;
    if ((DCPTiltMode)dcptilt_mode.get() == DCPT_MODE_NMPC) {
        const float nmpc_tilt_rad =
            constrain_float(current_tilt, 0.0f, 1.0f) * M_PI_2;
        const float progress = constrain_float(dcptilt_progress, 0.0f, 1.0f);

        float transient_window = 1.0f;
        if (progress >= 0.35f) {
            transient_window = 0.0f;
        } else if (progress > 0.15f) {
            const float x = constrain_float((progress - 0.15f) / 0.20f, 0.0f, 1.0f);
            const float smooth = x * x * (3.0f - 2.0f * x);
            transient_window = 1.0f - smooth;
        }

        const float raw_shape =
            constrain_float(dcptilt_mc_weight, 0.0f, 1.0f) *
            MAX(sinf(nmpc_tilt_rad), 0.0f);
        dcptilt_nmpc_pitch_shape = raw_shape * transient_window;

        const float nmpc_gain_deg =
            MAX(dcptilt_nmpc_pitch_gain_deg.get(), 0.0f);
        const float nmpc_max_deg =
            constrain_float(dcptilt_nmpc_pitch_max_deg.get(), 0.0f, 20.0f);
        dcptilt_nmpc_pitch_bias_deg = constrain_float(
            nmpc_gain_deg * dcptilt_nmpc_pitch_shape,
            0.0f, nmpc_max_deg);
    }

    // MODE=1 hard-switch handover transient. This is a single decaying
    // nose-down mismatch, not a synthetic oscillation. The unchanged altitude
    // controller supplies the subsequent recovery/overshoot.
    dcptilt_switch_pitch_bias_deg = 0.0f;
    if ((DCPTiltMode)dcptilt_mode.get() == DCPT_MODE_SWITCH &&
        dcptilt_switch_kick_start_ms != 0) {
        const float kick_time_s =
            constrain_float(dcptilt_switch_pitch_kick_time_s.get(), 0.2f, 3.0f);
        const float age_s =
            (now - dcptilt_switch_kick_start_ms) * 0.001f;

        if (age_s < kick_time_s) {
            const float x = constrain_float(age_s / kick_time_s, 0.0f, 1.0f);
            const float smooth = x * x * (3.0f - 2.0f * x);
            dcptilt_switch_pitch_bias_deg =
                dcptilt_switch_kick_initial_deg * (1.0f - smooth);
        } else {
            dcptilt_switch_kick_start_ms = 0;
            dcptilt_switch_kick_initial_deg = 0.0f;
            dcptilt_switch_last_step = 0.0f;
        }
    }

    const float pitch_target_deg = constrain_float(
        dcptilt_fw_pitch_base_deg +
        dcptilt_nmpc_pitch_bias_deg +
        dcptilt_switch_pitch_bias_deg,
        -pitch_limit_deg, pitch_limit_deg);
    dcptilt_fw_pitch_target_cd =
        (int32_t)(pitch_target_deg * 100.0f);
}

// Fixed-wing altitude-hold pitch is injected once at the native Plane
// stabilize_pitch_get_pitch_out() boundary. Roll is intentionally left on
// the normal FBWA/FBWB Plane path. This avoids running the Plane pitch PID
// twice in one loop while keeping the transition altitude controller
// independent of FBWA versus FBWB pitch/height behaviour.

// Direct DCPTilt output. This intentionally bypasses slew(),
// tilt_max_change(), Q_TILT_RATE_UP and Q_TILT_RATE_DN.
void Tiltrotor::dcptilt_set_tilt_direct(float newtilt)
{
    const float constrained_tilt = constrain_float(newtilt, 0.0f, 1.0f);
    current_tilt = constrained_tilt;
    angle_achieved = is_equal(newtilt, constrained_tilt);
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor_tilt, 1000.0f * current_tilt);
}

// Capture the actuator level that the tilting motors are actually outputting
// at the end of the primary transition. The post-transition handover remains
// deliberately separate from the research controller, so we preserve the
// actual actuator endpoint rather than assuming it equals an internal demand.
float Tiltrotor::dcptilt_capture_forward_output() const
{
    const int16_t pwm_min = motors->get_pwm_output_min();
    const int16_t pwm_max = motors->get_pwm_output_max();

    if (pwm_max <= pwm_min) {
        return constrain_float(motors->get_throttle(), 0.0f, 1.0f);
    }

    float output_sum = 0.0f;
    uint8_t output_count = 0;

    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (!motors->is_motor_enabled(i) || !is_motor_tilting(i)) {
            continue;
        }

        uint16_t pwm = 0;
        if (!SRV_Channels::get_output_pwm(SRV_Channels::get_motor_function(i), pwm)) {
            continue;
        }

        const float output = (float(pwm) - float(pwm_min)) / float(pwm_max - pwm_min);
        output_sum += constrain_float(output, 0.0f, 1.0f);
        output_count++;
    }

    if (output_count == 0) {
        return constrain_float(motors->get_throttle(), 0.0f, 1.0f);
    }

    return output_sum / output_count;
}

#if HAL_LOGGING_ENABLED
// DCPTilt research logging. This deliberately follows the same dynamic
// AP::logger().Write() pattern used by the user's geometric-controller project
// instead of adding a Plane static LogStructure.
//
// LogStructure/FMT fields are intentionally split into two messages so that
// each format stays within ArduPilot logger field/label limits.
void Tiltrotor::dcptilt_write_log()
{
    const uint32_t now = AP_HAL::millis();

    // Use the latched forward-transition selection while it exists. This keeps
    // DCPT diagnostics running even if Q_TILT_DCPT_EN is edited after the
    // transition has already been safely latched.
    const bool dcptilt_selected =
        dcptilt_transition_active ||
        (transition != nullptr &&
         transition->forward_transition_selection_latched &&
         transition->forward_transition_use_dcptilt);

    if (!dcptilt_selected || (now - dcptilt_last_log_ms < 40U)) {
        return;
    }
    dcptilt_last_log_ms = now;

    uint16_t tilt_pwm = 0;
    SRV_Channels::get_output_pwm(SRV_Channel::k_motor_tilt, tilt_pwm);

    float airspeed = 0.0f;
    quadplane.ahrs.airspeed_estimate(airspeed);

    float desired_alt_m = 0.0f;
    if (dcptilt_transition_active && dcptilt_alt_target_valid) {
        desired_alt_m = dcptilt_alt_target_m;
    } else if (plane.control_mode != &plane.mode_qstabilize) {
        desired_alt_m = quadplane.pos_control->get_pos_target_z_cm() * 0.01f;
    }

    const uint8_t state = (transition != nullptr) ? transition->get_log_transition_state() : 0U;
    const float fw_throttle = MAX(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle), 0.0f) * 0.01f;

    // Core DCPTilt transition variables. 10 fields, labels length < 64.
    AP::logger().Write(
        "DCPT",
        "TimeUS,St,Prog,Elap,TTot,TiltT,TiltC,TiltP,ASpd,GSpd",
        "Qbffffffff",
        AP_HAL::micros64(),
        (int8_t)state,
        dcptilt_progress,
        dcptilt_elapsed_s,
        dcptilt_transition_time_s.get(),
        dcptilt_target_tilt * 90.0f,
        current_tilt * 90.0f,
        (float)tilt_pwm,
        airspeed,
        quadplane.ahrs.groundspeed());

    // Auxiliary controller/flight-state variables. 9 fields, labels length < 64.
    AP::logger().Write(
        "DCPA",
        "TimeUS,MCThr,FWThr,DAlt,Alt,AltE,Roll,Pitch,Ast",
        "Qfffffffb",
        AP_HAL::micros64(),
        motors->get_throttle(),
        fw_throttle,
        desired_alt_m,
        quadplane.inertial_nav.get_position_z_up_cm() * 0.01f,
        dcptilt_transition_active ? dcptilt_alt_error_m : plane.altitude_error_cm * 0.01f,
        plane.ahrs.roll_sensor * 0.01f,
        plane.ahrs.pitch_sensor * 0.01f,
        (int8_t)quadplane.assisted_flight);

    // Experimental controller-allocation selection and weights.
    AP::logger().Write(
        "DCPW",
        "TimeUS,Mode,Prof,Spd,MCW,FWW",
        "Qbbfff",
        AP_HAL::micros64(),
        (int8_t)dcptilt_mode.get(),
        (int8_t)dcptilt_profile.get(),
        dcptilt_strategy_speed_mps,
        dcptilt_mc_weight,
        dcptilt_fw_weight);

    // PROF=6..8 TD3 incremental tilt-profile diagnostics.
    if (dcptilt_profile.get() >= DCPT_PROFILE_TD3_A &&
        dcptilt_profile.get() <= DCPT_PROFILE_TD3_C) {
        AP::logger().Write(
            "DCTD",
            "TimeUS,Prof,Eh,Vn,Mn,Out,Rate,Dlt,Lam",
            "Qbfffffff",
            AP_HAL::micros64(),
            (int8_t)dcptilt_profile.get(),
            dcptilt_td3_eh_m,
            dcptilt_td3_vnorm,
            dcptilt_td3_motor_norm,
            dcptilt_td3_output,
            dcptilt_td3_lambda_rate,
            dcptilt_td3_delta_lambda,
            dcptilt_td3_lambda);

        // TD3 raw-speed diagnostics. VUse is the exact raw V used to form
        // DCTD.Vn. VAS is AHRS airspeed, VN3 is NED 3D velocity magnitude,
        // VLeg is the historical dcptilt_strategy_speed(), and GSpd is
        // groundspeed. ASOK=1 means AHRS airspeed was valid at the actor sample.
        AP::logger().Write(
            "DCTV",
            "TimeUS,Mode,VUse,VEq,VAS,VN3,VLeg,GSpd,VFlt,Vn,ASOK",
            "Qbffffffffb",
            AP_HAL::micros64(),
            (int8_t)constrain_int16(dcptilt_td3_speed_mode.get(), 0, 3),
            dcptilt_td3_speed_used_mps,
            dcptilt_td3_speed_equiv_mps,
            dcptilt_td3_airspeed_mps,
            dcptilt_td3_ned3_speed_mps,
            dcptilt_td3_legacy_speed_mps,
            MAX(quadplane.ahrs.groundspeed(), 0.0f),
            MAX(dcptilt_td3_flat_speed_mps.get(), 1.0f),
            dcptilt_td3_vnorm,
            (int8_t)dcptilt_td3_airspeed_valid);

        // TD3 observation diagnostics:
        // VExp is the speed-normalization exponent.
        // EHF is the configured Eh freeze time.
        // EHL is the actor-input absolute Eh limit.
        // ERaw is the live NED-equivalent Eh.
        // EUse is the actual clipped value sent to the actor.
        // Held=1 means Eh has already been latched by EHF.
        AP::logger().Write(
            "DCTE",
            "TimeUS,VExp,EHF,EHL,ERaw,EUse,Held",
            "Qfffffb",
            AP_HAL::micros64(),
            constrain_float(dcptilt_td3_speed_exponent.get(), 1.0f, 2.0f),
            constrain_float(dcptilt_td3_eh_freeze_s.get(), 0.0f, 60.0f),
            constrain_float(dcptilt_td3_eh_limit_m.get(), 0.0f, 2.0f),
            dcptilt_td3_eh_raw_m,
            dcptilt_td3_eh_m,
            (int8_t)dcptilt_td3_eh_frozen);
    }

    // MODE=1 historical hard-switch handover diagnostics.
    if ((DCPTiltMode)dcptilt_mode.get() == DCPT_MODE_SWITCH) {
        AP::logger().Write(
            "DCPS",
            "TimeUS,Spd,MCW,FWW,dFW,PBias,PBase,PTot",
            "Qfffffff",
            AP_HAL::micros64(),
            dcptilt_strategy_speed_mps,
            dcptilt_mc_weight,
            dcptilt_fw_weight,
            dcptilt_switch_last_step,
            dcptilt_switch_pitch_bias_deg,
            dcptilt_fw_pitch_base_deg,
            dcptilt_fw_pitch_target_cd * 0.01f);
    }

    // Extra diagnostics for the supplied two-input Mamdani FIS strategy.
    // Kept in a separate message so the long-standing DCPW schema is unchanged.
    if ((DCPTiltMode)dcptilt_mode.get() == DCPT_MODE_FIS) {
        AP::logger().Write(
            "DCPF",
            "TimeUS,Vel,Tilt,FWW,MCW",
            "Qffff",
            AP_HAL::micros64(),
            dcptilt_strategy_speed_mps,
            current_tilt,
            dcptilt_fw_weight,
            dcptilt_mc_weight);
    }

    // Fake-NMPC diagnostics. MCW/FWW are the same legacy FUZZ allocation used by MODE=0.
    // Shape is the short, progress-windowed NMPC pitch-transient envelope only. PBase is the common altitude
    // pitch target before the transient; PBias is the mode-specific positive
    // pitch term; PTot is the final target sent to Plane's native FW pitch PID.
    if ((DCPTiltMode)dcptilt_mode.get() == DCPT_MODE_NMPC) {
        AP::logger().Write(
            "DCPN",
            "TimeUS,Spd,Tilt,MCW,FWW,Shape,PBias,PBase,PTot",
            "Qffffffff",
            AP_HAL::micros64(),
            dcptilt_strategy_speed_mps,
            current_tilt,
            dcptilt_mc_weight,
            dcptilt_fw_weight,
            dcptilt_nmpc_pitch_shape,
            dcptilt_nmpc_pitch_bias_deg,
            dcptilt_fw_pitch_base_deg,
            dcptilt_fw_pitch_target_cd * 0.01f);
    }

    // Common altitude / wing-lift / thrust allocation diagnostics.
    AP::logger().Write(
        "DCPZ",
        "TimeUS,AltR,Alt,AltE,Vz,Az,LSpd,Lam,Tau,Thr,Ptch,Sat",
        "Qffffffffffb",
        AP_HAL::micros64(),
        dcptilt_alt_target_m,
        dcptilt_altitude_m,
        dcptilt_alt_error_m,
        dcptilt_vz_up_mps,
        dcptilt_accel_up_cmd_mss,
        dcptilt_lift_speed_mps,
        dcptilt_wing_lift_ratio,
        dcptilt_thrust_ratio_tau,
        dcptilt_throttle_cmd,
        dcptilt_fw_pitch_target_cd * 0.01f,
        (int8_t)dcptilt_thrust_saturated);

    // Terminal-height predictor diagnostics. Raw AltE in DCPZ is retained
    // unchanged; CErr is the predictor-augmented error actually used by both
    // altitude-control branches.
    AP::logger().Write(
        "DCPX",
        "TimeUS,Rem,Pred,TErr,TW,CErr",
        "Qfffff",
        AP_HAL::micros64(),
        dcptilt_terminal_time_remaining_s,
        dcptilt_terminal_pred_alt_m,
        dcptilt_terminal_error_m,
        dcptilt_terminal_blend,
        dcptilt_alt_control_error_m);

    // Yaw diagnostics. YRaw is the native multicopter yaw-moment demand and
    // YWgt is the actually applied MC yaw demand after complementary MCW/FWW
    // allocation. During the DCPTilt primary transition YWgt should therefore
    // equal MCW-scaled yaw demand rather than always matching YRaw.
    const float yaw_target_deg = dcptilt_yaw_target_cd * 0.01f;
    const float yaw_deg = plane.ahrs.yaw_sensor * 0.01f;
    AP::logger().Write(
        "DCPY",
        "TimeUS,Tilt,MCW,YRaw,YWgt,YawT,Yaw,YErr,YRate",
        "Qffffffff",
        AP_HAL::micros64(),
        current_tilt * 90.0f,
        dcptilt_mc_weight,
        dcptilt_mc_yaw_raw,
        dcptilt_mc_yaw_weighted,
        yaw_target_deg,
        yaw_deg,
        wrap_180(yaw_target_deg - yaw_deg),
        degrees(quadplane.ahrs.get_yaw_rate_earth()));

    // v3.29 fixed-wing yaw diagnostics. The heading outer loop produces RtT
    // (desired yaw rate); ArduPlane's native yawController rate PID supplies
    // the rudder command and its own I term. RudW is the fixed-wing allocation
    // FWW, and RudOut is the actually applied rudder after weighting.
    const auto &fw_yaw_pid = plane.yawController.get_pid_info();
    AP::logger().Write(
        "DCPR",
        "TimeUS,YawT,Yaw,YErr,RtT,RtA,YI,RudRaw,RudW,RudOut,RollT",
        "Qffffffffff",
        AP_HAL::micros64(),
        yaw_target_deg,
        yaw_deg,
        wrap_180(yaw_target_deg - yaw_deg),
        dcptilt_fw_yaw_rate_target_dps,
        fw_yaw_pid.actual,
        fw_yaw_pid.I,
        dcptilt_rudder_raw,
        dcptilt_rudder_weight,
        dcptilt_rudder_output,
        plane.nav_roll_cd * 0.01f);

    // Post-transition handover and heading-lock diagnostics.
    AP::logger().Write(
        "DCPH",
        "TimeUS,Hnd,HPr,H0,FW,HOut,YawT,Yaw",
        "Qbffffff",
        AP_HAL::micros64(),
        (int8_t)dcptilt_handover_active,
        dcptilt_handover_progress,
        dcptilt_handover_start_throttle,
        fw_throttle,
        dcptilt_handover_output,
        dcptilt_yaw_target_cd * 0.01f,
        plane.ahrs.yaw_sensor * 0.01f);
}
#endif

// return the current tilt value that represents forward flight
// tilt wings can sustain forward flight with some amount of wing tilt
float Tiltrotor::get_fully_forward_tilt() const
{
    return 1.0 - (flap_angle_deg * (1/90.0));
}

// return the target tilt value for forward flight
float Tiltrotor::get_forward_flight_tilt() const
{
    return 1.0 - ((flap_angle_deg * (1/90.0)) * SRV_Channels::get_slew_limited_output_scaled(SRV_Channel::k_flap_auto) * 0.01);
}

/*
  update motor tilt for continuous tilt servos
 */
void Tiltrotor::continuous_update(void)
{
    // default to inactive
    _motors_active = false;

    // The stock Q_ASSIST path has priority over the research handover.
    // When assistance becomes active we leave the handover permanently and
    // let the official assisted-flight tilt/motor logic take control.
    if (dcptilt_handover_active && quadplane.assisted_flight) {
        dcptilt_handover_active = false;
    }

    // the maximum rate of throttle change
    float max_change;

    if (!quadplane.in_vtol_mode() && (!plane.arming.is_armed_and_safety_off() || !quadplane.assisted_flight)) {
        // we are in pure fixed wing mode. Move the tiltable motors all the way forward and run them as
        // a forward motor

        // option set then if disarmed move to VTOL position to prevent ground strikes, allow tilt forward in manual mode for testing
        const bool disarmed_tilt_up = !plane.arming.is_armed_and_safety_off() && (plane.control_mode != &plane.mode_manual) && quadplane.option_is_set(QuadPlane::OPTION::DISARMED_TILT_UP);
        slew(disarmed_tilt_up ? 0.0 : get_forward_flight_tilt());

        max_change = tilt_max_change(false);

        float new_throttle = constrain_float(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle)*0.01, 0, 1);

        if (dcptilt_handover_active && current_tilt >= get_fully_forward_tilt()) {
            const float handover_time_s = dcptilt_handover_time_s.get();
            if (handover_time_s <= 0.0f) {
                dcptilt_handover_progress = 1.0f;
                dcptilt_handover_output = new_throttle;
                dcptilt_handover_active = false;
                current_throttle = new_throttle;
            } else {
                const float elapsed_s = (AP_HAL::millis() - dcptilt_handover_start_ms) * 0.001f;
                dcptilt_handover_progress = constrain_float(elapsed_s / handover_time_s, 0.0f, 1.0f);

                // Smoothstep gives zero slope at both ends of the handover and
                // removes both the output step and a derivative step.
                const float p = dcptilt_handover_progress;
                const float blend = p * p * (3.0f - 2.0f * p);
                dcptilt_handover_output =
                    dcptilt_handover_start_throttle * (1.0f - blend) +
                    new_throttle * blend;
                current_throttle = constrain_float(dcptilt_handover_output, 0.0f, 1.0f);

                if (dcptilt_handover_progress >= 1.0f) {
                    dcptilt_handover_active = false;
                    current_throttle = new_throttle;
                    dcptilt_handover_output = current_throttle;
                }
            }
        } else if (current_tilt < get_fully_forward_tilt()) {
            current_throttle = constrain_float(new_throttle,
                                                    current_throttle-max_change,
                                                    current_throttle+max_change);
        } else {
            current_throttle = new_throttle;
            dcptilt_handover_output = current_throttle;
        }

        if (!plane.arming.is_armed_and_safety_off()) {
            current_throttle = 0;
            dcptilt_handover_active = false;
        } else {
            // prevent motor shutdown
            _motors_active = true;
        }
        if (!quadplane.motor_test.running) {
            // the motors are all the way forward, start using them for fwd thrust
            const uint16_t mask = is_zero(current_throttle)?0U:tilt_mask.get();
            motors->output_motor_mask(current_throttle, mask, plane.rudder_dt);
        }
        return;
    }

    // During a DCPTilt forward transition the transition object owns the
    // tilt target. Do not let the standard Q_TILT_RATE_* slew path overwrite it.
    if (dcptilt_transition_active) {
        current_throttle = motors->get_throttle();
        dcptilt_set_tilt_direct(dcptilt_target_tilt);
        return;
    }

    // remember the throttle level we're using for VTOL flight
    float motors_throttle = motors->get_throttle();
    max_change = tilt_max_change(motors_throttle<current_throttle);
    current_throttle = constrain_float(motors_throttle,
                                            current_throttle-max_change,
                                            current_throttle+max_change);

    /*
      we are in a VTOL mode. We need to work out how much tilt is
      needed. There are 5 strategies we will use:

      1) With use of a forward throttle controlled by Q_FWD_THR_GAIN in
         VTOL modes except Q_AUTOTUNE determined by Q_FWD_THR_USE. We set the angle based on a calculated
         forward throttle.

      2) With manual forward throttle control we set the angle based on the
         RC input demanded forward throttle for QACRO, QSTABILIZE and QHOVER.

      3) Without a RC input or calculated forward throttle value, the angle
         will be set to zero in QAUTOTUNE, QACRO, QSTABILIZE and QHOVER.
         This enables these modes to be used as a safe recovery mode.

      4) In fixed wing assisted flight or velocity controlled modes we will
         set the angle based on the demanded forward throttle, with a maximum
         tilt given by Q_TILT_MAX. This relies on Q_FWD_THR_GAIN or Q_VFWD_GAIN
         being set.

      5) if we are in TRANSITION_TIMER mode then we are transitioning
         to forward flight and should put the rotors all the way forward
    */

#if QAUTOTUNE_ENABLED
    if (plane.control_mode == &plane.mode_qautotune) {
        slew(0);
        return;
    }
#endif

    if (!quadplane.assisted_flight &&
        quadplane.get_vfwd_method() == QuadPlane::ActiveFwdThr::NEW &&
        quadplane.is_flying_vtol())
    {
        // We are using the rotor tilt functionality controlled by Q_FWD_THR_GAIN which can
        // operate in all VTOL modes except Q_AUTOTUNE. Forward rotor tilt is used to produce
        // forward thrust equivalent to what would have been produced by a forward thrust motor
        // set to quadplane.forward_throttle_pct()
        const float fwd_g_demand = 0.01 * quadplane.forward_throttle_pct();
        const float fwd_tilt_deg = MIN(degrees(atanf(fwd_g_demand)), (float)max_angle_deg);
        slew(MIN(fwd_tilt_deg * (1/90.0), get_forward_flight_tilt()));
        return;
    } else if (!quadplane.assisted_flight &&
               (plane.control_mode == &plane.mode_qacro ||
               plane.control_mode == &plane.mode_qstabilize ||
               plane.control_mode == &plane.mode_qhover))
    {
        if (quadplane.rc_fwd_thr_ch == nullptr) {
            // no manual throttle control, set angle to zero
            slew(0);
        } else {
            // manual control of forward throttle up to max VTOL angle
            float settilt = .01f * quadplane.forward_throttle_pct();
            slew(MIN(settilt * max_angle_deg * (1/90.0), get_forward_flight_tilt())); 
        }
        return;
    }

    if (quadplane.assisted_flight &&
        transition->transition_state >= Tiltrotor_Transition::TRANSITION_TIMER) {
        // we are transitioning to fixed wing - tilt the motors all
        // the way forward
        slew(get_forward_flight_tilt());
    } else {
        // until we have completed the transition we limit the tilt to
        // Q_TILT_MAX. Anything above 50% throttle gets
        // Q_TILT_MAX. Below 50% throttle we decrease linearly. This
        // relies heavily on Q_VFWD_GAIN being set appropriately.
       float settilt = constrain_float((SRV_Channels::get_output_scaled(SRV_Channel::k_throttle)-MAX(plane.aparm.throttle_min.get(),0)) * 0.02, 0, 1);
       slew(MIN(settilt * max_angle_deg * (1/90.0), get_forward_flight_tilt())); 
    }
}


/*
  output a slew limited tiltrotor angle. tilt is 0 or 1
 */
void Tiltrotor::binary_slew(bool forward)
{
    // The servo output is binary, not slew rate limited
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor_tilt, forward?1000:0);

    // rate limiting current_tilt has the effect of delaying throttle in tiltrotor_binary_update
    float max_change = tilt_max_change(!forward);
    if (forward) {
        current_tilt = constrain_float(current_tilt+max_change, 0, 1);
    } else {
        current_tilt = constrain_float(current_tilt-max_change, 0, 1);
    }
}

/*
  update motor tilt for binary tilt servos
 */
void Tiltrotor::binary_update(void)
{
    // motors always active
    _motors_active = true;

    if (!quadplane.in_vtol_mode()) {
        // we are in pure fixed wing mode. Move the tiltable motors
        // all the way forward and run them as a forward motor
        binary_slew(true);

        float new_throttle = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle)*0.01f;
        if (current_tilt >= 1) {
            const uint16_t mask = is_zero(new_throttle)?0U:tilt_mask.get();
            // the motors are all the way forward, start using them for fwd thrust
            motors->output_motor_mask(new_throttle, mask, plane.rudder_dt);
        }
    } else {
        binary_slew(false);
    }
}


/*
  update motor tilt
 */
void Tiltrotor::update(void)
{
    if (!enabled() || tilt_mask == 0) {
        // no motors to tilt
        return;
    }

    if (type == TILT_TYPE_BINARY) {
        binary_update();
    } else {
        continuous_update();
    }

    if (type == TILT_TYPE_VECTORED_YAW) {
        vectoring();
    }

#if HAL_LOGGING_ENABLED
    dcptilt_write_log();
#endif
}

/*
  tilt compensation for angle of tilt. When the rotors are tilted the
  roll effect of differential thrust on the tilted rotors is decreased
  and the yaw effect increased
  We have two factors we apply.

  1) when we are transitioning to fwd flight we scale the tilted rotors by 1/cos(angle). This pushes us towards more flight speed

  2) when we are transitioning to hover we scale the non-tilted rotors by cos(angle). This pushes us towards lower fwd thrust

  We also apply an equalisation to the tilted motors in proportion to
  how much tilt we have. This smoothly reduces the impact of the roll
  gains as we tilt further forward.

  For yaw, we apply differential thrust in proportion to the demanded
  yaw control and sin of the tilt angle

  Finally we ensure no requested thrust is over 1 by scaling back all
  motors so the largest thrust is at most 1.0
 */
void Tiltrotor::tilt_compensate_angle(float *thrust, uint8_t num_motors, float non_tilted_mul, float tilted_mul)
{
    float tilt_total = 0;
    uint8_t tilt_count = 0;
    
    // apply tilt_factors first
    for (uint8_t i=0; i<num_motors; i++) {
        if (!is_motor_tilting(i)) {
            thrust[i] *= non_tilted_mul;
        } else {
            thrust[i] *= tilted_mul;
            tilt_total += thrust[i];
            tilt_count++;
        }
    }

    float largest_tilted = 0;
    const float sin_tilt = sinf(radians(current_tilt*90));
    // yaw_gain relates the amount of differential thrust we get from
    // tilt, so that the scaling of the yaw control is the same at any
    // tilt angle
    const float yaw_gain = sinf(radians(tilt_yaw_angle));
    const float avg_tilt_thrust = tilt_total / tilt_count;

    for (uint8_t i=0; i<num_motors; i++) {
        if (is_motor_tilting(i)) {
            // as we tilt we need to reduce the impact of the roll
            // controller. This simple method keeps the same average,
            // but moves us to no roll control as the angle increases
            thrust[i] = current_tilt * avg_tilt_thrust + thrust[i] * (1-current_tilt);
            // add in differential thrust for yaw control, scaled by tilt angle
            const float diff_thrust = motors->get_roll_factor(i) * (motors->get_yaw()+motors->get_yaw_ff()) * sin_tilt * yaw_gain;
            thrust[i] += diff_thrust;
            largest_tilted = MAX(largest_tilted, thrust[i]);
        }
    }

    // if we are saturating one of the motors then reduce all motors
    // to keep them in proportion to the original thrust. This helps
    // maintain stability when tilted at a large angle
    if (largest_tilted > 1.0f) {
        float scale = 1.0f / largest_tilted;
        for (uint8_t i=0; i<num_motors; i++) {
            thrust[i] *= scale;
        }
    }
}

/*
  choose up or down tilt compensation based on flight mode When going
  to a fixed wing mode we use tilt_compensate_down, when going to a
  VTOL mode we use tilt_compensate_up
 */
void Tiltrotor::tilt_compensate(float *thrust, uint8_t num_motors)
{
    // DCPTilt computes total thrust explicitly from the shared altitude /
    // wing-lift model. Applying ArduPlane's native 1/cos(tilt) compensation
    // here would double-compensate the same physics and re-introduce the
    // large-angle saturation seen in the earlier experiments.
    if (dcptilt_transition_active) {
        return;
    }

    if (current_tilt <= 0) {
        // the motors are not tilted, no compensation needed
        return;
    }
    if (quadplane.in_vtol_mode()) {
        // we are transitioning to VTOL flight
        const float tilt_factor = cosf(radians(current_tilt*90));
        tilt_compensate_angle(thrust, num_motors, tilt_factor, 1);
    } else {
        float inv_tilt_factor;
        if (current_tilt > 0.98f) {
            inv_tilt_factor = 1.0 / cosf(radians(0.98f*90));
        } else {
            inv_tilt_factor = 1.0 / cosf(radians(current_tilt*90));
        }
        tilt_compensate_angle(thrust, num_motors, 1, inv_tilt_factor);
    }
}

/*
  return true if the rotors are fully tilted forward
 */
bool Tiltrotor::fully_fwd(void) const
{
    if (!enabled() || (tilt_mask == 0)) {
        return false;
    }
    return (current_tilt >= get_fully_forward_tilt());
}

/*
  return true if the rotors are fully tilted up
 */
bool Tiltrotor::fully_up(void) const
{
    if (!enabled() || (tilt_mask == 0)) {
        return false;
    }
    return (current_tilt <= 0);
}

/*
  control vectoring for tilt multicopters
 */
void Tiltrotor::vectoring(void)
{
    // total angle the tilt can go through
    const float total_angle = 90 + tilt_yaw_angle + fixed_angle;
    // output value (0 to 1) to get motors pointed straight up
    const float zero_out = tilt_yaw_angle / total_angle;
    const float fixed_tilt_limit = fixed_angle / total_angle;
    const float level_out = 1.0 - fixed_tilt_limit;

    // calculate the basic tilt amount from current_tilt
    float base_output = zero_out + (current_tilt * (level_out - zero_out));
    // for testing when disarmed, apply vectored yaw in proportion to rudder stick
    // Wait TILT_DELAY_MS after disarming to allow props to spin down first.
    constexpr uint32_t TILT_DELAY_MS = 3000;
    uint32_t now = AP_HAL::millis();
    if (!plane.arming.is_armed_and_safety_off() && plane.quadplane.option_is_set(QuadPlane::OPTION::DISARMED_TILT)) {
        // this test is subject to wrapping at ~49 days, but the consequences are insignificant
        if ((now - hal.util->get_last_armed_change()) > TILT_DELAY_MS) {
            if (quadplane.in_vtol_mode()) {
                float yaw_out = plane.channel_rudder->get_control_in();
                yaw_out /= plane.channel_rudder->get_range();
                float yaw_range = zero_out;

                SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft,  1000 * constrain_float(base_output + yaw_out * yaw_range,0,1));
                SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, 1000 * constrain_float(base_output - yaw_out * yaw_range,0,1));
                SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRear,  1000 * constrain_float(base_output,0,1));
                SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRearLeft,  1000 * constrain_float(base_output + yaw_out * yaw_range,0,1));
                SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRearRight, 1000 * constrain_float(base_output - yaw_out * yaw_range,0,1));
            } else {
                // fixed wing tilt
                const float gain = fixed_gain * fixed_tilt_limit;
                // base the tilt on elevon mixing, which means it
                // takes account of the MIXING_GAIN. The rear tilt is
                // based on elevator
                const float right = gain * SRV_Channels::get_output_scaled(SRV_Channel::k_elevon_right) * (1/4500.0);
                const float left  = gain * SRV_Channels::get_output_scaled(SRV_Channel::k_elevon_left) * (1/4500.0);
                const float mid  = gain * SRV_Channels::get_output_scaled(SRV_Channel::k_elevator) * (1/4500.0);
                // front tilt is effective canards, so need to swap and use negative. Rear motors are treated live elevons.
                SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft,1000 * constrain_float(base_output - right,0,1));
                SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight,1000 * constrain_float(base_output - left,0,1));
                SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRearLeft,1000 * constrain_float(base_output + left,0,1));
                SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRearRight,1000 * constrain_float(base_output + right,0,1));
                SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRear,  1000 * constrain_float(base_output + mid,0,1));
            }
        }
        return;
    }

    const bool no_yaw = tilt_over_max_angle();
    if (no_yaw) {
        // fixed wing  We need to apply inverse scaling with throttle, and remove the surface speed scaling as
        // we don't want tilt impacted by airspeed
        const float scaler = plane.control_mode == &plane.mode_manual?1:(quadplane.FW_vector_throttle_scaling() / plane.get_speed_scaler());
        const float gain = fixed_gain * fixed_tilt_limit * scaler;
        const float right = gain * SRV_Channels::get_output_scaled(SRV_Channel::k_elevon_right) * (1/4500.0);
        const float left  = gain * SRV_Channels::get_output_scaled(SRV_Channel::k_elevon_left) * (1/4500.0);
        const float mid  = gain * SRV_Channels::get_output_scaled(SRV_Channel::k_elevator) * (1/4500.0);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft,1000 * constrain_float(base_output - right,0,1));
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight,1000 * constrain_float(base_output - left,0,1));
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRearLeft,1000 * constrain_float(base_output + left,0,1));
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRearRight,1000 * constrain_float(base_output + right,0,1));
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRear,  1000 * constrain_float(base_output + mid,0,1));
    } else {
        const float yaw_out = motors->get_yaw()+motors->get_yaw_ff();
        const float roll_out = motors->get_roll()+motors->get_roll_ff();
        const float yaw_range = zero_out;

        // Scaling yaw with throttle
        const float throttle = motors->get_throttle_out();
        const float scale_min = 0.5;
        const float scale_max = 2.0;
        float throttle_scaler = scale_max;
        if (is_positive(throttle)) {
            throttle_scaler = constrain_float(motors->get_throttle_hover() / throttle, scale_min, scale_max);
        }

        // now apply vectored thrust for yaw and roll.
        const float tilt_rad = radians(current_tilt*90);
        const float sin_tilt = sinf(tilt_rad);
        const float cos_tilt = cosf(tilt_rad);
        // the MotorsMatrix library normalises roll factor to 0.5, so
        // we need to use the same factor here to keep the same roll
        // gains when tilted as we have when not tilted
        const float avg_roll_factor = 0.5;
        float tilt_scale = throttle_scaler * yaw_out * cos_tilt + avg_roll_factor * roll_out * sin_tilt;

        if (fabsf(tilt_scale) > 1.0) {
            tilt_scale = constrain_float(tilt_scale, -1.0, 1.0);
            motors->limit.yaw = true;
        }

        const float tilt_offset = tilt_scale * yaw_range;

        float left_tilt = base_output + tilt_offset;
        float right_tilt = base_output - tilt_offset;

        // if output saturation of both left and right then set yaw limit flag
        if (((left_tilt > 1.0) || (left_tilt < 0.0)) &&
            ((right_tilt > 1.0) || (right_tilt < 0.0))) {
            motors->limit.yaw = true;
        }

        // constrain and scale to ouput range
        left_tilt = constrain_float(left_tilt,0.0,1.0) * 1000.0;
        right_tilt = constrain_float(right_tilt,0.0,1.0) * 1000.0;

        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, left_tilt);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, right_tilt);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRear, 1000.0 * constrain_float(base_output,0.0,1.0));
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRearLeft, left_tilt);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRearRight, right_tilt);
    }
}

/*
  control bicopter tiltrotors
 */
void Tiltrotor::bicopter_output(void)
{
    if (type != TILT_TYPE_BICOPTER || quadplane.motor_test.running) {
        // don't override motor test with motors_output
        return;
    }

    if (!quadplane.in_vtol_mode() && fully_fwd()) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft,  -SERVO_MAX);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, -SERVO_MAX);
        return;
    }

    float throttle = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);
    if (quadplane.assisted_flight) {
        quadplane.hold_stabilize(throttle * 0.01f);
        quadplane.motors_output(true);
    } else {
        quadplane.motors_output(false);
    }

    // bicopter assumes that trim is up so we scale down so match
    float tilt_left = SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorLeft);
    float tilt_right = SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorRight);

    if (is_negative(tilt_left)) {
        tilt_left *= tilt_yaw_angle * (1/90.0);
    }
    if (is_negative(tilt_right)) {
        tilt_right *= tilt_yaw_angle * (1/90.0);
    }

    // reduce authority of bicopter as motors are tilted forwards
    const float scaling = cosf(current_tilt * M_PI_2);
    tilt_left  *= scaling;
    tilt_right *= scaling;

    // add current tilt and constrain
    tilt_left  = constrain_float(-(current_tilt * SERVO_MAX) + tilt_left,  -SERVO_MAX, SERVO_MAX);
    tilt_right = constrain_float(-(current_tilt * SERVO_MAX) + tilt_right, -SERVO_MAX, SERVO_MAX);

    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft,  tilt_left);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, tilt_right);
}

// Reset only the DCPTilt-owned state. The inherited transition state is
// reset by the corresponding SLT transition method.
void Tiltrotor_Transition::dcptilt_reset_state()
{
    tiltrotor.dcptilt_transition_active = false;
    tiltrotor.dcptilt_transition_start_ms = 0;
    tiltrotor.dcptilt_progress = 0.0f;
    tiltrotor.dcptilt_elapsed_s = 0.0f;
    tiltrotor.dcptilt_target_tilt = 0.0f;
    tiltrotor.dcptilt_td3_last_update_ms = 0;
    tiltrotor.dcptilt_td3_lambda = 0.0f;
    tiltrotor.dcptilt_td3_eh_m = 0.0f;
    tiltrotor.dcptilt_td3_eh_raw_m = 0.0f;
    tiltrotor.dcptilt_td3_eh_frozen_m = 0.0f;
    tiltrotor.dcptilt_td3_eh_frozen = false;
    tiltrotor.dcptilt_td3_vnorm = 0.0f;
    tiltrotor.dcptilt_td3_motor_norm = 0.0f;
    tiltrotor.dcptilt_td3_output = 0.0f;
    tiltrotor.dcptilt_td3_lambda_rate = 0.0f;
    tiltrotor.dcptilt_td3_delta_lambda = 0.0f;
    tiltrotor.dcptilt_td3_runtime_updated = false;
    tiltrotor.dcptilt_td3_runtime_start_us = 0U;
    tiltrotor.dcptilt_td3_actor_us = 0U;
    tiltrotor.dcptilt_td3_proj_us = 0U;
    tiltrotor.dcptilt_td3_total_us = 0U;
    tiltrotor.dcptilt_td3_runtime_seq = 0U;
    tiltrotor.dcptilt_td3_speed_used_mps = 0.0f;
    tiltrotor.dcptilt_td3_speed_equiv_mps = 0.0f;
    tiltrotor.dcptilt_td3_airspeed_mps = 0.0f;
    tiltrotor.dcptilt_td3_ned3_speed_mps = 0.0f;
    tiltrotor.dcptilt_td3_legacy_speed_mps = 0.0f;
    tiltrotor.dcptilt_td3_airspeed_valid = false;
    tiltrotor.dcptilt_strategy_speed_mps = 0.0f;
    tiltrotor.dcptilt_mc_weight = 1.0f;
    tiltrotor.dcptilt_fw_weight = 0.0f;

    tiltrotor.dcptilt_alt_target_valid = false;
    tiltrotor.dcptilt_alt_target_m = 0.0f;
    tiltrotor.dcptilt_altitude_m = 0.0f;
    tiltrotor.dcptilt_alt_error_m = 0.0f;
    tiltrotor.dcptilt_vz_up_mps = 0.0f;
    tiltrotor.dcptilt_accel_up_cmd_mss = 0.0f;
    tiltrotor.dcptilt_lift_speed_mps = 0.0f;
    tiltrotor.dcptilt_wing_lift_ratio = 0.0f;
    tiltrotor.dcptilt_thrust_ratio_tau = 1.0f;
    tiltrotor.dcptilt_throttle_raw = 0.0f;
    tiltrotor.dcptilt_throttle_cmd = 0.0f;
    tiltrotor.dcptilt_fw_pitch_target_cd = 0;
    tiltrotor.dcptilt_fw_pitch_base_deg = 0.0f;
    tiltrotor.dcptilt_nmpc_pitch_shape = 0.0f;
    tiltrotor.dcptilt_nmpc_pitch_bias_deg = 0.0f;
    tiltrotor.dcptilt_switch_kick_start_ms = 0;
    tiltrotor.dcptilt_switch_kick_initial_deg = 0.0f;
    tiltrotor.dcptilt_switch_pitch_bias_deg = 0.0f;
    tiltrotor.dcptilt_switch_last_step = 0.0f;
    tiltrotor.dcptilt_thrust_saturated = false;
    tiltrotor.dcptilt_alt_last_ms = 0;
    tiltrotor.dcptilt_alt_control_error_m = 0.0f;
    tiltrotor.dcptilt_terminal_time_remaining_s = 0.0f;
    tiltrotor.dcptilt_terminal_pred_alt_m = 0.0f;
    tiltrotor.dcptilt_terminal_error_m = 0.0f;
    tiltrotor.dcptilt_terminal_blend = 0.0f;
    tiltrotor.dcptilt_mc_yaw_raw = 0.0f;
    tiltrotor.dcptilt_mc_yaw_weighted = 0.0f;
    tiltrotor.dcptilt_rudder_raw = 0.0f;
    tiltrotor.dcptilt_rudder_weight = 0.0f;
    tiltrotor.dcptilt_rudder_output = 0.0f;
    tiltrotor.dcptilt_fw_yaw_rate_target_dps = 0.0f;

    tiltrotor.dcptilt_handover_active = false;
    tiltrotor.dcptilt_handover_start_ms = 0;
    tiltrotor.dcptilt_handover_start_throttle = 0.0f;
    tiltrotor.dcptilt_handover_progress = 0.0f;
    tiltrotor.dcptilt_handover_output = 0.0f;

    tiltrotor.dcptilt_yaw_lock_active = false;
    tiltrotor.dcptilt_yaw_target_cd = 0.0f;

    forward_transition_selection_latched = false;
    forward_transition_use_dcptilt = false;
    dcptilt_primary_transition_complete = false;
}

void Tiltrotor_Transition::update()
{
    // Do not create a persistent selector latch while disarmed. Preserve the
    // historical disarmed behavior, but only latch once an armed forward
    // transition is actually being flown.
    if (!plane.arming.is_armed_and_safety_off()) {
        forward_transition_selection_latched = false;
        forward_transition_use_dcptilt = false;

        if (tiltrotor.dcptilt_enabled()) {
            dcptilt_update();
        } else {
            SLT_Transition::update();
        }
        return;
    }

    // Safety-critical forward-transition selection:
    // sample Q_TILT_DCPT_EN once and hold that choice for the whole forward
    // transition. A GCS parameter write cannot swap transition controllers
    // halfway through flight.
    if (!forward_transition_selection_latched) {
        forward_transition_selection_latched = true;
        forward_transition_use_dcptilt = tiltrotor.dcptilt_enabled();

        gcs().send_text(
            MAV_SEVERITY_INFO,
            forward_transition_use_dcptilt ?
                "Tilt FW: DCPT latched" :
                "Tilt FW: AP native latched");

#if HAL_LOGGING_ENABLED
        AP::logger().Write(
            "DCSF",
            "TimeUS,Sel,Param,Tilt",
            "Qbbf",
            AP_HAL::micros64(),
            (int8_t)forward_transition_use_dcptilt,
            (int8_t)tiltrotor.dcptilt_enable.get(),
            tiltrotor.current_tilt * 90.0f);
#endif
    }

    if (!forward_transition_use_dcptilt) {
        SLT_Transition::update();
        return;
    }

    // The time-scheduled DCPTilt transition is used only for the primary
    // VTOL->FW transition. After it completes, delegate to the stock SLT
    // logic so Q_ASSIST remains available strictly as a safety mechanism.
    if (dcptilt_primary_transition_complete) {
        SLT_Transition::update();
        return;
    }

    dcptilt_update();
}

void Tiltrotor_Transition::VTOL_update()
{
    // Every FW->VTOL transition is intentionally handed to ArduPilot's native
    // SLT path. If the pilot switches back to a Q mode while DCPTilt is still
    // active, treat that as an explicit abort: do NOT first command 90 deg;
    // start the native return transition from the current physical tilt.
    const bool dcptilt_abort =
        forward_transition_selection_latched &&
        forward_transition_use_dcptilt &&
        tiltrotor.dcptilt_transition_active;

    const bool had_forward_selection = forward_transition_selection_latched;

    if (dcptilt_abort) {
        const float abort_elapsed_s = tiltrotor.dcptilt_elapsed_s;
        const float abort_tilt_deg = tiltrotor.current_tilt * 90.0f;
        const int8_t abort_profile = tiltrotor.dcptilt_profile.get();

        gcs().send_text(
            MAV_SEVERITY_WARNING,
            "DCPTilt ABORT %.1fs %.1fdeg -> AP VTOL",
            (double)abort_elapsed_s,
            (double)abort_tilt_deg);

#if HAL_LOGGING_ENABLED
        AP::logger().Write(
            "DCSA",
            "TimeUS,Code,Elap,Tilt,Prof",
            "Qbffb",
            AP_HAL::micros64(),
            (int8_t)1,
            abort_elapsed_s,
            abort_tilt_deg,
            abort_profile);
#endif
    } else if (had_forward_selection) {
        gcs().send_text(MAV_SEVERITY_INFO, "Tilt VTOL: AP native");
    }

    dcptilt_reset_state();
    SLT_Transition::VTOL_update();
}

void Tiltrotor_Transition::force_transition_complete()
{
    dcptilt_reset_state();
    SLT_Transition::force_transition_complete();
}

void Tiltrotor_Transition::restart()
{
    dcptilt_reset_state();
    SLT_Transition::restart();
}

// DCPTilt primary forward transition:
//  * starts immediately on the first fixed-wing transition update
//  * progress = elapsed / Q_TILT_DCPT_TIME
//  * tilt follows the selected static profile or 20 Hz TD3 closed-loop profile
//  * MC/FW attitude authority follows the selected FUZZ/SWITCH/NMPC strategy
//  * normal completion depends only on progress reaching 1
//  * RC throttle and the selected FBWA/FBWB altitude logic are ignored
//  * a common DCPTilt altitude/vertical-force controller supplies total thrust
void Tiltrotor_Transition::dcptilt_update()
{
    const uint32_t now = AP_HAL::millis();

    if (!plane.arming.is_armed_and_safety_off()) {
        dcptilt_reset_state();
        transition_state = TRANSITION_DONE;
        in_forced_transition = false;
        transition_start_ms = 0;
        transition_low_airspeed_ms = 0;
        quadplane.assisted_flight = false;
        set_last_fw_pitch();
        return;
    }

    if (transition_state == TRANSITION_DONE && !tiltrotor.dcptilt_transition_active) {
        // We are already in fixed-wing flight without having entered a new
        // VTOL->FW transition. Treat DCPTilt primary transition as complete
        // and use the standard SLT path from now on (including Q_ASSIST).
        dcptilt_primary_transition_complete = true;
        SLT_Transition::update();
        return;
    }

    if (!tiltrotor.dcptilt_transition_active) {
        tiltrotor.dcptilt_transition_active = true;
        tiltrotor.dcptilt_transition_start_ms = now;
        tiltrotor.dcptilt_progress = 0.0f;
        tiltrotor.dcptilt_elapsed_s = 0.0f;
        tiltrotor.dcptilt_target_tilt = 0.0f;
        tiltrotor.dcptilt_td3_last_update_ms = now;
        tiltrotor.dcptilt_td3_runtime_updated = false;
        tiltrotor.dcptilt_td3_runtime_start_us = 0U;
        tiltrotor.dcptilt_td3_actor_us = 0U;
        tiltrotor.dcptilt_td3_proj_us = 0U;
        tiltrotor.dcptilt_td3_total_us = 0U;
        tiltrotor.dcptilt_td3_runtime_seq = 0U;
        tiltrotor.dcptilt_td3_lambda = 0.0f;
        tiltrotor.dcptilt_td3_eh_m = 0.0f;
        tiltrotor.dcptilt_td3_eh_raw_m = 0.0f;
        tiltrotor.dcptilt_td3_eh_frozen_m = 0.0f;
        tiltrotor.dcptilt_td3_eh_frozen = false;
        tiltrotor.dcptilt_td3_vnorm = 0.0f;
        tiltrotor.dcptilt_td3_motor_norm = 0.0f;
        tiltrotor.dcptilt_td3_output = 0.0f;
        tiltrotor.dcptilt_td3_lambda_rate = 0.0f;
        tiltrotor.dcptilt_td3_delta_lambda = 0.0f;
        tiltrotor.dcptilt_td3_speed_used_mps = 0.0f;
        tiltrotor.dcptilt_td3_speed_equiv_mps = 0.0f;
        tiltrotor.dcptilt_td3_airspeed_mps = 0.0f;
        tiltrotor.dcptilt_td3_ned3_speed_mps = 0.0f;
        tiltrotor.dcptilt_td3_legacy_speed_mps = 0.0f;
        tiltrotor.dcptilt_td3_airspeed_valid = false;
        tiltrotor.dcptilt_mc_weight = 1.0f;
        tiltrotor.dcptilt_fw_weight = 0.0f;
        tiltrotor.dcptilt_strategy_speed_mps = 0.0f;

        // Capture one common altitude reference at transition entry. All
        // All strategy/profile cases use this same reference and altitude controller.
        tiltrotor.dcptilt_alt_target_m =
            quadplane.inertial_nav.get_position_z_up_cm() * 0.01f;
        tiltrotor.dcptilt_altitude_m = tiltrotor.dcptilt_alt_target_m;
        tiltrotor.dcptilt_alt_target_valid = true;
        tiltrotor.dcptilt_alt_error_m = 0.0f;
        tiltrotor.dcptilt_vz_up_mps =
            quadplane.inertial_nav.get_velocity_z_up_cms() * 0.01f;
        tiltrotor.dcptilt_accel_up_cmd_mss = 0.0f;
        tiltrotor.dcptilt_lift_speed_mps = 0.0f;
        tiltrotor.dcptilt_wing_lift_ratio = 0.0f;
        tiltrotor.dcptilt_thrust_ratio_tau = 1.0f;
        tiltrotor.dcptilt_throttle_raw = motors->get_throttle_hover();
        tiltrotor.dcptilt_throttle_cmd = tiltrotor.dcptilt_throttle_raw;
        tiltrotor.dcptilt_fw_pitch_target_cd = 0;
        tiltrotor.dcptilt_fw_pitch_base_deg = 0.0f;
        tiltrotor.dcptilt_nmpc_pitch_shape = 0.0f;
        tiltrotor.dcptilt_nmpc_pitch_bias_deg = 0.0f;
        tiltrotor.dcptilt_switch_kick_start_ms = 0;
        tiltrotor.dcptilt_switch_kick_initial_deg = 0.0f;
        tiltrotor.dcptilt_switch_pitch_bias_deg = 0.0f;
        tiltrotor.dcptilt_switch_last_step = 0.0f;
        tiltrotor.dcptilt_thrust_saturated = false;
        tiltrotor.dcptilt_alt_last_ms = 0;
        tiltrotor.dcptilt_alt_control_error_m = 0.0f;
        tiltrotor.dcptilt_terminal_time_remaining_s =
            tiltrotor.dcptilt_transition_time_s.get();
        tiltrotor.dcptilt_terminal_pred_alt_m = tiltrotor.dcptilt_alt_target_m;
        tiltrotor.dcptilt_terminal_error_m = 0.0f;
        tiltrotor.dcptilt_terminal_blend = 0.0f;
        tiltrotor.dcptilt_mc_yaw_raw = 0.0f;
        tiltrotor.dcptilt_mc_yaw_weighted = 0.0f;
        tiltrotor.dcptilt_rudder_raw = 0.0f;
        tiltrotor.dcptilt_rudder_weight = 0.0f;
        tiltrotor.dcptilt_rudder_output = 0.0f;

        transition_state = TRANSITION_TIMER;
        transition_start_ms = now;
        transition_low_airspeed_ms = now;
        airspeed_reached_tilt = tiltrotor.current_tilt;
        last_throttle = motors->get_throttle();
        in_forced_transition = false;

        // Integrator-safe transition entry. Start both MC and FW attitude
        // controllers from zero I while keeping their attitude targets intact.
#if HAL_LOGGING_ENABLED
        const auto &is0_mc_r = quadplane.attitude_control->get_rate_roll_pid().get_pid_info();
        const auto &is0_mc_p = quadplane.attitude_control->get_rate_pitch_pid().get_pid_info();
        const auto &is0_mc_y = quadplane.attitude_control->get_rate_yaw_pid().get_pid_info();
        const auto &is0_fw_r = plane.rollController.get_pid_info();
        const auto &is0_fw_p = plane.pitchController.get_pid_info();
        const auto &is0_fw_y = plane.yawController.get_pid_info();
        AP::logger().Write(
            "DCSI",
            "TimeUS,Phase,MRI,MPI,MYI,FRI,FPI,FYI,Tilt",
            "Qbfffffff",
            AP_HAL::micros64(),
            (int8_t)10,
            is0_mc_r.I,
            is0_mc_p.I,
            is0_mc_y.I,
            is0_fw_r.I,
            is0_fw_p.I,
            is0_fw_y.I,
            tiltrotor.current_tilt * 90.0f);
#endif
        quadplane.attitude_control->reset_rate_controller_I_terms();
        plane.rollController.reset_I();
        plane.pitchController.reset_I();
        plane.yawController.reset_I();
#if HAL_LOGGING_ENABLED
        const auto &is1_mc_r = quadplane.attitude_control->get_rate_roll_pid().get_pid_info();
        const auto &is1_mc_p = quadplane.attitude_control->get_rate_pitch_pid().get_pid_info();
        const auto &is1_mc_y = quadplane.attitude_control->get_rate_yaw_pid().get_pid_info();
        const auto &is1_fw_r = plane.rollController.get_pid_info();
        const auto &is1_fw_p = plane.pitchController.get_pid_info();
        const auto &is1_fw_y = plane.yawController.get_pid_info();
        AP::logger().Write(
            "DCSI",
            "TimeUS,Phase,MRI,MPI,MYI,FRI,FPI,FYI,Tilt",
            "Qbfffffff",
            AP_HAL::micros64(),
            (int8_t)11,
            is1_mc_r.I,
            is1_mc_p.I,
            is1_mc_y.I,
            is1_fw_r.I,
            is1_fw_p.I,
            is1_fw_y.I,
            tiltrotor.current_tilt * 90.0f);
#endif
        quadplane.attitude_control->set_throttle_mix_max(1.0f);

        // Initialise the common DCPTilt yaw target at transition entry.
        //
        // v3.29 keeps straight-flight heading lock when bank demand is small,
        // but also reuses ArduPilot's stock Tiltrotor::update_yaw_target()
        // coordinated-turn target evolution when |nav_roll| > 10 deg and
        // airspeed is available. Both MC and FW yaw controllers follow this
        // same target, preserving complementary controller allocation.
        tiltrotor.dcptilt_yaw_target_cd = quadplane.ahrs.yaw_sensor;
        tiltrotor.transition_yaw_cd = tiltrotor.dcptilt_yaw_target_cd;
        tiltrotor.transition_yaw_set_ms = now;
        tiltrotor.dcptilt_yaw_lock_active = true;
        tiltrotor.dcptilt_fw_yaw_rate_target_dps = 0.0f;

        gcs().send_text(MAV_SEVERITY_INFO, "DCPTilt transition start");
    }

    const float total_time_s = constrain_float(tiltrotor.dcptilt_transition_time_s.get(), 0.1f, 300.0f);
    tiltrotor.dcptilt_elapsed_s = (now - tiltrotor.dcptilt_transition_start_ms) * 0.001f;
    tiltrotor.dcptilt_progress = constrain_float(tiltrotor.dcptilt_elapsed_s / total_time_s, 0.0f, 1.0f);

    const int8_t active_profile = tiltrotor.dcptilt_profile.get();
    if (active_profile >= Tiltrotor::DCPT_PROFILE_TD3_A &&
        active_profile <= Tiltrotor::DCPT_PROFILE_TD3_C) {
        tiltrotor.dcptilt_target_tilt =
            tiltrotor.dcptilt_update_td3_profile(now);
    } else {
        tiltrotor.dcptilt_target_tilt =
            tiltrotor.dcptilt_tilt_profile(tiltrotor.dcptilt_progress);
    }

    tiltrotor.dcptilt_update_control_weights();

    // Set the requested tilt before computing the common vertical-force
    // allocation. Native Tiltrotor::tilt_compensate() is bypassed while
    // DCPTilt is active, so there is no second 1/cos compensation.
    tiltrotor.dcptilt_set_tilt_direct(tiltrotor.dcptilt_target_tilt);

    // Runtime log for the TD3 online trajectory module.
    //
    // The policy runs at 20 Hz, so the hard computation budget is 50 ms.
    // Logging is performed AFTER TotalUS is captured, which prevents the
    // logger write itself from inflating the measured online execution time.
    if (tiltrotor.dcptilt_td3_runtime_updated) {
        tiltrotor.dcptilt_td3_total_us =
            AP_HAL::micros() - tiltrotor.dcptilt_td3_runtime_start_us;

        static constexpr uint32_t TD3_POLICY_BUDGET_US =
            DCPTILT_TD3_PERIOD_MS * 1000U;

        const int8_t runtime_miss =
            (tiltrotor.dcptilt_td3_total_us > TD3_POLICY_BUDGET_US) ? 1 : 0;

#if HAL_LOGGING_ENABLED
        AP::logger().Write(
            "RLT",
            "TimeUS,ActorUS,ProjUS,TotalUS,Miss,Seq",
            "QIIIbI",
            AP_HAL::micros64(),
            tiltrotor.dcptilt_td3_actor_us,
            tiltrotor.dcptilt_td3_proj_us,
            tiltrotor.dcptilt_td3_total_us,
            runtime_miss,
            tiltrotor.dcptilt_td3_runtime_seq);
#endif

        // Consume the sample flag so a faster outer loop cannot write the
        // same 20 Hz timing sample more than once.
        tiltrotor.dcptilt_td3_runtime_updated = false;
    }

    tiltrotor.dcptilt_update_altitude_controller();

    plane.TECS_controller.use_synthetic_airspeed();
    quadplane.set_desired_spool_state(
        AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    quadplane.attitude_control->set_throttle_mix_value(0.5f);

    // Multicopter altitude is controlled through the common total-thrust
    // command above. Do not alter the mode-generated roll target here. For all
    // non-NMPC strategies the MC pitch target stays neutral. NMPC alone gets
    // the positive transient bias computed above; its motor pitch moment is
    // still naturally scaled by the selected MCW in QuadPlane::motors_output().
    const int32_t saved_nav_pitch_cd = plane.nav_pitch_cd;
    plane.nav_pitch_cd =
        (int32_t)(tiltrotor.dcptilt_nmpc_pitch_bias_deg * 100.0f);

    quadplane.assisted_flight = true;
    quadplane.hold_stabilize(tiltrotor.dcptilt_throttle_cmd);

    plane.nav_pitch_cd = saved_nav_pitch_cd;

    quadplane.motors_output();
    set_last_fw_pitch();

    // Timed completion restored for all profiles, including TD3 Actor
    // profiles 6/7/8. The Actor determines the natural tilt trajectory
    // during the configured transition window; once Q_TILT_DCPT_TIME
    // expires, the state machine finishes the remaining tilt to 90 deg.
    if (tiltrotor.dcptilt_progress >= 1.0f) {
        tiltrotor.dcptilt_target_tilt = 1.0f;
        tiltrotor.dcptilt_set_tilt_direct(1.0f);

        // motors_output() above has already produced this cycle's final
        // transition motor commands. Capture that actual output before
        // changing to the pure fixed-wing output path. This prevents the
        // observed ~1950 -> ~1500 one-cycle step.
        if (!tiltrotor.has_vtol_motor() && !tiltrotor.has_fw_motor() &&
            tiltrotor.dcptilt_handover_time_s.get() > 0.0f) {
            tiltrotor.dcptilt_handover_start_throttle =
                tiltrotor.dcptilt_capture_forward_output();
            tiltrotor.dcptilt_handover_output =
                tiltrotor.dcptilt_handover_start_throttle;
            tiltrotor.dcptilt_handover_progress = 0.0f;
            tiltrotor.dcptilt_handover_start_ms = now;
            tiltrotor.dcptilt_handover_active = true;
        } else {
            tiltrotor.dcptilt_handover_active = false;
            tiltrotor.dcptilt_handover_progress = 1.0f;
        }

        // Plane's attitude controller ran earlier in this loop while
        // dcptilt_transition_active was still true. Apply the final-cycle FW
        // weight here before clearing the active flag; QuadPlane::update()
        // will otherwise (correctly) skip its normal active-transition scaling.
        const float final_fw_weight =
            constrain_float(tiltrotor.dcptilt_fw_weight, 0.0f, 1.0f);
        SRV_Channels::set_output_scaled(
            SRV_Channel::k_aileron,
            SRV_Channels::get_output_scaled(SRV_Channel::k_aileron) * final_fw_weight);
        SRV_Channels::set_output_scaled(
            SRV_Channel::k_elevator,
            SRV_Channels::get_output_scaled(SRV_Channel::k_elevator) * final_fw_weight);
        SRV_Channels::set_output_scaled(
            SRV_Channel::k_rudder,
            SRV_Channels::get_output_scaled(SRV_Channel::k_rudder) * final_fw_weight);

        // Integrator-safe DCPT -> pure fixed-wing handover. Current-cycle
        // outputs are already formed; zero I now so the next FW loop rebuilds
        // from the actual post-transition state.
#if HAL_LOGGING_ENABLED
        const auto &id0_mc_r = quadplane.attitude_control->get_rate_roll_pid().get_pid_info();
        const auto &id0_mc_p = quadplane.attitude_control->get_rate_pitch_pid().get_pid_info();
        const auto &id0_mc_y = quadplane.attitude_control->get_rate_yaw_pid().get_pid_info();
        const auto &id0_fw_r = plane.rollController.get_pid_info();
        const auto &id0_fw_p = plane.pitchController.get_pid_info();
        const auto &id0_fw_y = plane.yawController.get_pid_info();
        AP::logger().Write(
            "DCSI",
            "TimeUS,Phase,MRI,MPI,MYI,FRI,FPI,FYI,Tilt",
            "Qbfffffff",
            AP_HAL::micros64(),
            (int8_t)20,
            id0_mc_r.I,
            id0_mc_p.I,
            id0_mc_y.I,
            id0_fw_r.I,
            id0_fw_p.I,
            id0_fw_y.I,
            tiltrotor.current_tilt * 90.0f);
#endif
        quadplane.attitude_control->reset_rate_controller_I_terms();
        plane.rollController.reset_I();
        plane.pitchController.reset_I();
        plane.yawController.reset_I();
#if HAL_LOGGING_ENABLED
        const auto &id1_mc_r = quadplane.attitude_control->get_rate_roll_pid().get_pid_info();
        const auto &id1_mc_p = quadplane.attitude_control->get_rate_pitch_pid().get_pid_info();
        const auto &id1_mc_y = quadplane.attitude_control->get_rate_yaw_pid().get_pid_info();
        const auto &id1_fw_r = plane.rollController.get_pid_info();
        const auto &id1_fw_p = plane.pitchController.get_pid_info();
        const auto &id1_fw_y = plane.yawController.get_pid_info();
        AP::logger().Write(
            "DCSI",
            "TimeUS,Phase,MRI,MPI,MYI,FRI,FPI,FYI,Tilt",
            "Qbfffffff",
            AP_HAL::micros64(),
            (int8_t)21,
            id1_mc_r.I,
            id1_mc_p.I,
            id1_mc_y.I,
            id1_fw_r.I,
            id1_fw_p.I,
            id1_fw_y.I,
            tiltrotor.current_tilt * 90.0f);
#endif
        tiltrotor.dcptilt_transition_active = false;
        tiltrotor.dcptilt_yaw_lock_active = false;
        dcptilt_primary_transition_complete = true;

        transition_state = TRANSITION_DONE;
        transition_start_ms = 0;
        transition_low_airspeed_ms = 0;
        in_forced_transition = false;
        quadplane.assisted_flight = false;

        gcs().send_text(MAV_SEVERITY_INFO,
                        "DCPTilt done, handover %.2f",
                        (double)tiltrotor.dcptilt_handover_start_throttle);
    }
}

/*
  when doing a forward transition of a tilt-vectored quadplane we use
  euler angle control to maintain good yaw. This updates the yaw
  target based on pilot input and target roll
 */
void Tiltrotor::update_yaw_target(void)
{
    uint32_t now = AP_HAL::millis();
    if (now - transition_yaw_set_ms > 100 ||
        !is_zero(quadplane.get_pilot_input_yaw_rate_cds())) {
        // lock initial yaw when transition is started or when
        // pilot commands a yaw change. This allows us to track
        // straight in transitions for tilt-vectored planes, but
        // allows for turns when level transition is not wanted
        transition_yaw_cd = quadplane.ahrs.yaw_sensor;
    }

    /*
      now calculate the equivalent yaw rate for a coordinated turn for
      the desired bank angle given the airspeed
     */
    float aspeed;
    bool have_airspeed = quadplane.ahrs.airspeed_estimate(aspeed);
    if (have_airspeed && labs(plane.nav_roll_cd)>1000) {
        float dt = (now - transition_yaw_set_ms) * 0.001;
        // calculate the yaw rate to achieve the desired turn rate
        const float airspeed_min = MAX(plane.aparm.airspeed_min,5);
        const float yaw_rate_cds = fixedwing_turn_rate(plane.nav_roll_cd*0.01, MAX(aspeed,airspeed_min))*100;
        transition_yaw_cd += yaw_rate_cds * dt;
    }
    transition_yaw_set_ms = now;
}

bool Tiltrotor_Transition::update_yaw_target(float& yaw_target_cd)
{
    // DCPTilt primary transition: both MC and FW yaw controllers share the
    // same absolute yaw target. Reuse ArduPilot's stock tiltrotor transition
    // target evolution here so a significant commanded bank can produce the
    // corresponding coordinated-turn yaw target instead of forcing an
    // incompatible straight-heading target.
    if (forward_transition_selection_latched &&
        forward_transition_use_dcptilt &&
        tiltrotor.dcptilt_transition_active &&
        tiltrotor.dcptilt_yaw_lock_active) {
        tiltrotor.transition_yaw_cd = tiltrotor.dcptilt_yaw_target_cd;
        tiltrotor.update_yaw_target();
        tiltrotor.dcptilt_yaw_target_cd = tiltrotor.transition_yaw_cd;
        yaw_target_cd = tiltrotor.dcptilt_yaw_target_cd;
        return true;
    }

    if (!(tiltrotor.is_vectored() &&
        transition_state <= TRANSITION_TIMER)) {
        return false;
    }
    tiltrotor.update_yaw_target();
    yaw_target_cd = tiltrotor.transition_yaw_cd;
    return true;
}

// return true if we should show VTOL view
bool Tiltrotor_Transition::show_vtol_view() const
{
    bool show_vtol = quadplane.in_vtol_mode();

    if (!show_vtol &&
        forward_transition_selection_latched &&
        forward_transition_use_dcptilt &&
        tiltrotor.dcptilt_transition_active &&
        tiltrotor.dcptilt_yaw_lock_active) {
        // DCPTilt uses the multicopter attitude controller with an absolute
        // yaw target during the primary forward transition.
        return true;
    }

    if (!show_vtol && tiltrotor.is_vectored() && transition_state <= TRANSITION_TIMER) {
        // we use multirotor controls during fwd transition for
        // vectored yaw vehicles
        return true;
    }

    return show_vtol;
}

// return true if we are tilted over the max angle threshold
bool Tiltrotor::tilt_over_max_angle(void) const
{
    const float tilt_threshold = (max_angle_deg/90.0f);
    return (current_tilt > MIN(tilt_threshold, get_forward_flight_tilt()));
}

#endif  // HAL_QUADPLANE_ENABLED

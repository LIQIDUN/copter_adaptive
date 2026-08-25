#include "tiltrotor.h"
#include "Plane.h"
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
    // @Description: Enables the DCPTilt time-scheduled forward transition for continuous tiltrotors. Airspeed is monitoring only and is not used as a normal transition completion condition.
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

// DCPTilt v1 tilt profile. 0 means vertical/VTOL and 1 means fully forward.
// Future experimental profiles should be isolated to this function.
float Tiltrotor::dcptilt_tilt_profile(float progress) const
{
    return constrain_float(progress, 0.0f, 1.0f);
}

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
// at the end of the primary transition. Using the actual PWM-derived level is
// important here: motors->get_throttle() is the pre tilt-compensation demand
// and can be much lower than the final motor output when 1/cos(tilt) has
// driven the motors to saturation.
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
    if (!dcptilt_enabled() || (now - dcptilt_last_log_ms < 40U)) {
        return;
    }
    dcptilt_last_log_ms = now;

    uint16_t tilt_pwm = 0;
    SRV_Channels::get_output_pwm(SRV_Channel::k_motor_tilt, tilt_pwm);

    float airspeed = 0.0f;
    quadplane.ahrs.airspeed_estimate(airspeed);

    float desired_alt_m = 0.0f;
    if (plane.control_mode != &plane.mode_qstabilize) {
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
        plane.altitude_error_cm * 0.01f,
        plane.ahrs.roll_sensor * 0.01f,
        plane.ahrs.pitch_sensor * 0.01f,
        (int8_t)quadplane.assisted_flight);

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

    tiltrotor.dcptilt_handover_active = false;
    tiltrotor.dcptilt_handover_start_ms = 0;
    tiltrotor.dcptilt_handover_start_throttle = 0.0f;
    tiltrotor.dcptilt_handover_progress = 0.0f;
    tiltrotor.dcptilt_handover_output = 0.0f;

    tiltrotor.dcptilt_yaw_lock_active = false;
    tiltrotor.dcptilt_yaw_target_cd = 0.0f;

    dcptilt_primary_transition_complete = false;
}

void Tiltrotor_Transition::update()
{
    if (!tiltrotor.dcptilt_enabled()) {
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

// DCPTilt v2 primary forward transition:
//  * starts immediately on the first fixed-wing transition update
//  * progress = elapsed / Q_TILT_DCPT_TIME
//  * tilt follows dcptilt_tilt_profile(progress)
//  * normal completion depends only on progress reaching 1
//  * airspeed is monitoring only
//  * ArduPlane's existing hold_stabilize/throttle blend is retained
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
        tiltrotor.dcptilt_target_tilt = tiltrotor.dcptilt_tilt_profile(0.0f);

        transition_state = TRANSITION_TIMER;
        transition_start_ms = now;
        transition_low_airspeed_ms = now;
        airspeed_reached_tilt = tiltrotor.current_tilt;
        last_throttle = motors->get_throttle();
        in_forced_transition = false;

        plane.pitchController.reset_I();
        plane.rollController.reset_I();
        quadplane.attitude_control->set_throttle_mix_max(1.0f);

        // Lock the heading that existed at transition entry. This is a target
        // angle, not a repeated reset to the already-drifted current yaw.
        tiltrotor.dcptilt_yaw_target_cd = quadplane.ahrs.yaw_sensor;
        tiltrotor.dcptilt_yaw_lock_active = true;

        gcs().send_text(MAV_SEVERITY_INFO, "DCPTilt transition start");
    }

    const float total_time_s = constrain_float(tiltrotor.dcptilt_transition_time_s.get(), 0.1f, 300.0f);
    tiltrotor.dcptilt_elapsed_s = (now - tiltrotor.dcptilt_transition_start_ms) * 0.001f;
    tiltrotor.dcptilt_progress = constrain_float(tiltrotor.dcptilt_elapsed_s / total_time_s, 0.0f, 1.0f);
    tiltrotor.dcptilt_target_tilt = tiltrotor.dcptilt_tilt_profile(tiltrotor.dcptilt_progress);

    // Update current_tilt before motors_output() so native tilt compensation
    // sees the DCPTilt angle in the same control cycle.
    tiltrotor.dcptilt_set_tilt_direct(tiltrotor.dcptilt_target_tilt);

    plane.TECS_controller.use_synthetic_airspeed();
    quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    const float transition_scale = 1.0f - tiltrotor.dcptilt_progress;
    float throttle_scaled = last_throttle * transition_scale;
    quadplane.attitude_control->set_throttle_mix_value(0.5f * transition_scale);

    if (throttle_scaled < 0.01f) {
        throttle_scaled = 0.01f;
    }

    if (!tiltrotor.has_vtol_motor() && !tiltrotor.has_fw_motor()) {
        const float fully_forward_tilt = tiltrotor.get_fully_forward_tilt();
        const float tilt_range = fully_forward_tilt - airspeed_reached_tilt;
        float ratio = 1.0f;
        if (tilt_range > 0.001f) {
            ratio = (constrain_float(tiltrotor.current_tilt, airspeed_reached_tilt, fully_forward_tilt) - airspeed_reached_tilt) / tilt_range;
        }
        const float fw_throttle = MAX(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle), 0.0f) * 0.01f;
        throttle_scaled = constrain_float(throttle_scaled * (1.0f - ratio) + fw_throttle * ratio, 0.0f, 1.0f);
    }

    quadplane.assisted_flight = true;
    quadplane.hold_stabilize(throttle_scaled);

    quadplane.motors_output();
    set_last_fw_pitch();

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
    // DCPTilt primary transition: hold the heading captured at transition
    // entry. QuadPlane::multicopter_attitude_rate_update() uses this hook to
    // enable multicopter attitude control with an absolute yaw target.
    if (tiltrotor.dcptilt_enabled() &&
        tiltrotor.dcptilt_transition_active &&
        tiltrotor.dcptilt_yaw_lock_active) {
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
        tiltrotor.dcptilt_enabled() &&
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
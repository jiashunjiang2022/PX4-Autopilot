/**
 * Path navigation roll slew rate limit.
 *
 * Maximum change in roll angle setpoint per second.
 * Applied in all Auto modes, plus manual Position & Altitude modes.
 *
 * @unit deg/s
 * @min 0
 * @decimal 0
 * @increment 1
 * @group FW Lateral Control
 */
PARAM_DEFINE_FLOAT(FW_PN_R_SLEW_MAX, 90.0f);

/**
 * Minimum groundspeed
 *
 * The controller will increase the commanded airspeed to maintain
 * this minimum groundspeed to the next waypoint.
 *
 * @unit m/s
 * @min 0.0
 * @max 40
 * @decimal 1
 * @increment 0.5
 * @group FW Longitudinal Control
 */
PARAM_DEFINE_FLOAT(FW_GND_SPD_MIN, 5.0f);

/**
 * Guidance Algorithm Selection
 * @min 0
 * @max 2
 * @value 0 NPFG
 * @value 1 L1
 * @value 2 PID
 * @group FW Lateral Control
 */
PARAM_DEFINE_INT32(FW_GUIDANCE_MODE, 0);

/**
 * L1 Period
 * @unit s
 * @min 10.0
 * @max 50.0
 * @decimal 1
 * @increment 1.0
 * @group FW Lateral Control
 */
PARAM_DEFINE_FLOAT(FW_L1_PERIOD, 25.0f);

/**
 * L1 Damping
 *
 * Controls the damping ratio of the L1 controller.
 * 0.7 is a good starting point for most fixed-wing configurations.
 *
 * @min 0.1
 * @max 1.5
 * @decimal 2
 * @increment 0.01
 * @group FW Lateral Control
 */
PARAM_DEFINE_FLOAT(FW_L1_DAMPING, 0.75f);

/**
 * PID Guidance: Proportional Gain
 * 
 * Controls the responsiveness to cross-track error.
 * Higher values = faster correction, but may cause oscillations.
 * 
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.1
 * @group FW Lateral Control
 */
PARAM_DEFINE_FLOAT(FW_PID_XTE_KP, 2.0f);

/**
 * PID Guidance: Integral Gain
 * 
 * Eliminates steady-state error. Use with caution to avoid windup.
 * 
 * @min 0.0
 * @max 2.0
 * @decimal 3
 * @increment 0.01
 * @group FW Lateral Control
 */
PARAM_DEFINE_FLOAT(FW_PID_XTE_KI, 0.1f);

/**
 * PID Guidance: Derivative Gain
 * 
 * Provides damping, reduces overshoot. Higher values = more damping.
 * 
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @increment 0.1
 * @group FW Lateral Control
 */
PARAM_DEFINE_FLOAT(FW_PID_XTE_KD, 1.0f);

/**
 * PID Guidance: Maximum Lateral Acceleration
 * 
 * Limits the maximum lateral acceleration output.
 * 
 * @unit m/s^2
 * @min 1.0
 * @max 10.0
 * @decimal 1
 * @increment 0.5
 * @group FW Lateral Control
 */
PARAM_DEFINE_FLOAT(FW_PID_XTE_MAXA, 6.0f);

/**
 * PID Guidance: Integral Limit
 * 
 * Prevents integral windup. Set to maximum expected steady-state error.
 * 
 * @unit m
 * @min 0.0
 * @max 100.0
 * @decimal 1
 * @increment 1.0
 * @group FW Lateral Control
 */
PARAM_DEFINE_FLOAT(FW_PID_XTE_ILIM, 50.0f);

// ----------longitudinal params----------

/**
 * Throttle max slew rate
 *
 * Maximum slew rate for the commanded throttle
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW Longitudinal Control
 */
PARAM_DEFINE_FLOAT(FW_THR_SLEW_MAX, 0.0f);

/**
 * Low-height threshold for tighter altitude tracking
 *
 * Height above ground threshold below which tighter altitude
 * tracking gets enabled (see FW_LND_THRTC_SC). Below this height, TECS smoothly
 * (1 sec / sec) transitions the altitude tracking time constant from FW_T_ALT_TC
 * to FW_LND_THRTC_SC*FW_T_ALT_TC.
 *
 * -1 to disable.
 *
 * @unit m
 * @min -1
 * @decimal 0
 * @increment 1
 * @group FW Longitudinal Control
 */
PARAM_DEFINE_FLOAT(FW_T_THR_LOW_HGT, -1.f);

/**
 * Throttle damping factor
 *
 * This is the damping gain for the throttle demand loop.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.01
 * @group FW Longitudinal Control
 */
PARAM_DEFINE_FLOAT(FW_T_THR_DAMPING, 0.05f);

/**
 * Integrator gain throttle
 *
 * Increase it to trim out speed and height offsets faster,
 * with the downside of possible overshoots and oscillations.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.005
 * @group FW Longitudinal Control
 */
PARAM_DEFINE_FLOAT(FW_T_THR_INTEG, 0.02f);

/**
 * Integrator gain pitch
 *
 * Increase it to trim out speed and height offsets faster,
 * with the downside of possible overshoots and oscillations.
 *
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @increment 0.05
 * @group FW Longitudinal Control
 */
PARAM_DEFINE_FLOAT(FW_T_I_GAIN_PIT, 0.05f);

/**
 * Maximum vertical acceleration
 *
 * This is the maximum vertical acceleration
 * either up or down that the controller will use to correct speed
 * or height errors.
 *
 * @unit m/s^2
 * @min 1.0
 * @max 10.0
 * @decimal 1
 * @increment 0.5
 * @group FW Longitudinal Control
 */
PARAM_DEFINE_FLOAT(FW_T_VERT_ACC, 4.0f);

/**
 * Airspeed measurement standard deviation
 *
 * For the airspeed filter in TECS.
 *
 * @unit m/s
 * @min 0.01
 * @max 10.0
 * @decimal 2
 * @increment 0.1
 * @group FW Longitudinal Control
 */
PARAM_DEFINE_FLOAT(FW_T_SPD_STD, 0.07f);

/**
 * Airspeed rate measurement standard deviation
 *
 * For the airspeed filter in TECS.
 *
 * @unit m/s^2
 * @min 0.01
 * @max 10.0
 * @decimal 2
 * @increment 0.1
 * @group FW Longitudinal Control
 */
PARAM_DEFINE_FLOAT(FW_T_SPD_DEV_STD, 0.2f);

/**
 * Process noise standard deviation for the airspeed rate
 *
 * This is defining the noise in the airspeed rate for the constant airspeed rate model
 * of the TECS airspeed filter.
 *
 * @unit m/s^2
 * @min 0.01
 * @max 10.0
 * @decimal 2
 * @increment 0.1
 * @group FW Longitudinal Control
 */
PARAM_DEFINE_FLOAT(FW_T_SPD_PRC_STD, 0.2f);

/**
 * Roll -> Throttle feedforward
 *
 * Is used to compensate for the additional drag created by turning.
 * Increase this gain if the aircraft initially loses energy in turns
 * and reduce if the aircraft initially gains energy in turns.
 *
 * @min 0.0
 * @max 20.0
 * @decimal 1
 * @increment 0.5
 * @group FW Longitudinal Control
 */
PARAM_DEFINE_FLOAT(FW_T_RLL2THR, 15.0f);

/**
 * Pitch damping gain
 *
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @increment 0.1
 * @group FW Longitudinal Control
 */
PARAM_DEFINE_FLOAT(FW_T_PTCH_DAMP, 0.3f);

/**
 * Altitude error time constant.
 *
 * @min 2.0
 * @decimal 2
 * @increment 0.5
 * @group FW Longitudinal Control
 */
PARAM_DEFINE_FLOAT(FW_T_ALT_TC, 8.0f);

/**
 * Fast descend: minimum altitude error
 *
 * Minimum altitude error needed to descend with max airspeed and minimal throttle.
 * A negative value disables fast descend.
 *
 * @min -1.0
 * @decimal 0
 * @group FW Longitudinal Control
 */
PARAM_DEFINE_FLOAT(FW_T_F_ALT_ERR, -1.0f);

/**
 * Height rate feed forward
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group FW Longitudinal Control
 */
PARAM_DEFINE_FLOAT(FW_T_HRATE_FF, 0.3f);

/**
 * True airspeed error time constant.
 *
 * @min 2.0
 * @decimal 2
 * @increment 0.5
 * @group FW Longitudinal Control
 */
PARAM_DEFINE_FLOAT(FW_T_TAS_TC, 5.0f);

/**
 * Specific total energy rate first order filter time constant.
 *
 * This filter is applied to the specific total energy rate used for throttle damping.
 *
 * @min 0.0
 * @max 2
 * @decimal 2
 * @increment 0.01
 * @group FW Longitudinal Control
 */
PARAM_DEFINE_FLOAT(FW_T_STE_R_TC, 0.4f);

/**
 * Specific total energy balance rate feedforward gain.
 *
 *
 * @min 0.5
 * @max 3
 * @decimal 2
 * @increment 0.01
 * @group FW Longitudinal Control
 */
PARAM_DEFINE_FLOAT(FW_T_SEB_R_FF, 1.0f);

/**
 * Wind-based airspeed scaling factor
 *
 * Multiplying this factor with the current absolute wind estimate gives the airspeed offset
 * added to the minimum airspeed setpoint limit. This helps to make the
 * system more robust against disturbances (turbulence) in high wind.
 *
 * @min 0
 * @decimal 2
 * @increment 0.01
 * @group FW Longitudinal Control
 */
PARAM_DEFINE_FLOAT(FW_WIND_ARSP_SC, 0.f);

/**
 * Maximum descent rate
 *
 * @unit m/s
 * @min 1.0
 * @max 15.0
 * @decimal 1
 * @increment 0.5
 * @group FW Longitudinal Control
 */
PARAM_DEFINE_FLOAT(FW_T_SINK_MAX, 5.0f);

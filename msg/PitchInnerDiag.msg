# Passive fixed-wing native pitch inner-loop snapshot.
# q/rates: rad/s; q_dot: rad/s^2; controller_dt: s; timestamps: us.
# P/I/D/FF/sum are in the Native RateControl command frame before output scaling.
# Output is normalized controller command, not physical deflection or aerodynamic torque.
uint64 timestamp
uint64 timestamp_sample
uint64 rate_sp_source_timestamp
uint64 inner_update_counter
float32 controller_dt
float32 q_sp_used
float32 q_used
float32 q_error
float32 q_dot_used
float32 pitch_p_term
float32 pitch_i_term_before
float32 pitch_i_term_after
float32 pitch_i_delta
float32 pitch_d_term
float32 effective_ff_gain
float32 pitch_ff_term
float32 pitch_rate_control_sum
float32 gain_compression_pitch
float32 airspeed_scaling
float32 airspeed_scaling_sq
float32 pitch_control_after_scaling
float32 pitch_trim # output-side TRIM_PITCH/airspeed scheduling; not FW_PSP_OFF attitude/TECS trim
float32 pitch_output_preclip
float32 pitch_output_final
float32 fw_pr_p
float32 fw_pr_i
float32 fw_pr_d
float32 fw_pr_ff
float32 fw_pr_imax
bool i_at_limit_before
bool i_at_limit_after
bool reset_integral
bool output_clip # local normalized clipping, not allocator or servo saturation
bool rates_enabled
bool attitude_enabled
bool manual_enabled
bool fixed_wing_or_transition
bool controller_active
bool fixed_wing_active
bool mission_active
bool landed
uint8 nav_state

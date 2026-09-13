uint64 timestamp		# time since system start (microseconds)

# rate controller integrator status
float32 rollspeed_integ
float32 pitchspeed_integ
float32 yawspeed_integ

# Roll-only counterfactual diagnostics. These never enter the control output.
float32 rollspeed_error
float32 rollspeed_integ_delta_raw
float32 rollspeed_integ_delta_pre_imax
float32 rollspeed_integ_shadow_no_imax
float32 rollspeed_integ_raw_drive_accum
bool rollspeed_integ_update_enabled

# Authoritative fixed-b0 Slow experiment status. Fast injection remains zero.
bool flap_slow_enabled
bool flap_slow_gate_valid
bool flap_slow_config_valid
float32 flap_slow_b0_tail
float32 flap_slow_target_tail
float32 flap_slow_applied_tail
float32 flap_slow_internal_scale
float32 flap_slow_applied_torque
float32 flap_slow_roll_baseline
float32 flap_slow_roll_total
bool flap_slow_total_clipped
uint8 flap_slow_exit_reason

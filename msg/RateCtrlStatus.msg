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

# Same-cycle fixed-wing rate-controller snapshot. All rate quantities are rad/s.
uint64 timestamp
uint64 timestamp_sample
uint64 controller_update_counter
float32 controller_dt # dt actually used by native controller and LP1
float32 measured_dt # unclamped sample delta; NaN if unavailable/nonmonotonic
float32 p_sp_used
float32 p_used
float32 p_error_raw
float32 p_error_filtered
float32 tbr_raw
float32 tbr_after_cap
float32 tbr_final
float32 native_roll_output # scaled/trimmed normalized command BEFORE clipping
float32 native_roll_clipped # native outgoing command without TBR
float32 final_roll_output # actual augmented command AFTER clipping
float32 native_roll_p_term # raw PID frame, before scaling/compression/trim
float32 native_roll_i_term # integral actually used, BEFORE native integral update
float32 native_roll_d_term # negative D * angular acceleration
float32 native_roll_ff_term # native airspeed-adjusted feedforward term
float32 native_pitch_output # native output only; no Pitch term instrumentation
bool mission_active
bool tbr_enabled
bool tbr_active
bool config_valid
bool cap_active
bool slew_active
bool controller_clip # local normalized-command clipping, not allocator feedback
uint8 reset_reason
uint8 RESET_NONE=0
uint8 RESET_DISABLED=1
uint8 RESET_DISARMED=2
uint8 RESET_UNSUPPORTED=3
uint8 RESET_NOT_MISSION=4
uint8 RESET_LANDED=5
uint8 RESET_INVALID_DT=6
uint8 RESET_INVALID_CONFIG=7
uint8 RESET_NONFINITE=8
uint8 RESET_CONTROLLER_INACTIVE=9
uint8 RESET_INTEGRAL=10
uint8 RESET_CONFIG_CHANGED=11

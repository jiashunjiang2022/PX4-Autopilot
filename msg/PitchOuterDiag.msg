# Passive fixed-wing native pitch outer-loop snapshot.
# Angles: rad; Euler/body rates: rad/s; controller_dt: s; timestamps: us.
# FW_P_RMAX_* parameter snapshots retain their native units (deg/s).
# Modules run asynchronously: join rate_sp_timestamp to the inner source key.
uint64 timestamp
uint64 timestamp_sample # last successfully copied attitude sample, or 0 if none
uint64 rate_sp_timestamp
uint64 outer_update_counter
uint64 attitude_sp_timestamp
float32 controller_dt
float32 pitch_sp
float32 pitch
float32 pitch_error
float32 pitch_euler_rate_sp
float32 yaw_euler_rate_sp_used
float32 q_sp_body_raw_reconstructed # diagnostic production-formula reconstruction, not control input
float32 q_sp_pitch_ctrl_limited
float32 q_sp_after_autotune
float32 q_sp_autotune_ff
float32 q_sp_final_published
float32 fw_p_tc
float32 fw_p_rmax_pos
float32 fw_p_rmax_neg
bool q_sp_limit_active
bool autotune_pitch_active # a valid autotune vector was added this cycle (any active axis)
bool reset_integral
bool rates_enabled
bool attitude_enabled
bool manual_enabled
bool fixed_wing_or_transition
bool attitude_control_active
bool fixed_wing_active
bool mission_active
bool landed
uint8 nav_state

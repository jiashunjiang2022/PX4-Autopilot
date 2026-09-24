# FAST decision used in this control cycle, not the newly completed predictor frame.
uint64 timestamp
uint64 decision_timestamp
uint64 model_timestamp
float32 decision_dt
uint64 prediction_age_us
uint32 model_frame_seq
bool enabled
bool valid
# !config_ok on every evaluated FAST step; false if no step was evaluated.
bool config_invalid
bool nonfinite_fallback
bool stale_fallback
bool reset_fallback
bool disabled_by_t
bool disabled_by_state
bool affects_gain_compression
bool affects_yaw
float32 prediction
float32 raw
float32 saturated
float32 model_t
float32 live_t_guard
float32 model_gate
float32 live_gate
float32 after_t_gate
float32 after_t_bound
float32 slew_target
float32 final
float32 authority_limit
float32 scale_used
float32 base_roll_raw
float32 augmented_roll_raw
# Same Run, controller frame; terms before scaling/trim. False means unavailable.
bool controller_terms_valid
float32 p_sp_used # rad/s, actual RateControl input
float32 p_used # rad/s, actual RateControl input
float32 p_error_used # p_sp_used - p_used
float32 roll_p_term
float32 roll_d_term # signed derivative contribution
float32 roll_ff_term
# Publication-time I_post/S; live_t_guard above remains the earlier decision input.
float32 native_roll_i_current
float32 slow_s_current
float32 total_t_current # native_roll_i_current + slow_s_current
# Normalized controller-frame torque, before allocator. No physical position measurement.
float32 baseline_roll_output
float32 baseline_pitch_output
float32 augmented_roll_output
float32 applied_delta_roll # augmented_roll_output - baseline_roll_output
bool actuator_config_valid # separate from FAST tuning config_invalid
bool actuator_margin_valid # finite analytic inputs; requires supported configuration
bool actuator_margin_blocked # baseline infeasible or margin unavailable
bool actuator_margin_limited # target or previous actual contracted by actuator bounds
bool actuator_postcheck_failed # unexpected augmented feasibility failure, reverted to baseline
float32 after_actuator_bound # FAST burden units; after_t_bound keeps old meaning
float32 actuator_roll_lower # normalized roll torque, fixed pitch
float32 actuator_roll_upper
float32 actuator_roll_headroom_negative # nonnegative torque magnitude
float32 actuator_roll_headroom_positive
float32 actuator_fast_lower # equivalent FAST burden units
float32 actuator_fast_upper
float32 tail0_baseline # analytic normalized surface, unconstrained baseline
float32 tail1_baseline
float32 tail0_augmented # analytic normalized surface from delivered torque
float32 tail1_augmented
int32 mission_seq_current # latest valid mission_result seq_current, -1 unavailable; logging only
bool ffplus_enabled
bool ffplus_active
bool ffplus_config_valid
float32 ffplus_delta
float32 base_roll_ff_gain
float32 effective_roll_ff_gain

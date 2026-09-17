# Automatic Roll working-point trim, SHADOW ONLY. No control authority.
uint64 timestamp
uint64 timestamp_sample # controller sample; I/S are end-of-cycle states
uint32 reset_epoch
uint32 compute_us
float32 g_cycle
float32 i_actual_raw
float32 s_actual_raw
float32 t_actual_raw
float32 b_obs
float32 b_hat
float32 b_target
float32 b_shadow
float32 virtual_i_offset_raw
float32 i_shadow_raw
float32 b_shadow_total
float32 requested_delta_b
float32 accepted_delta_b
float32 accepted_delta_i_virtual
float32 transfer_mismatch
float32 tail_pitch_raw
float32 tail_pitch_envelope
float32 roll_reserve_pos
float32 roll_reserve_neg
float32 roll_reserve_sym
float32 estimator_std
float32 same_sign_fraction
float32 actual_tail_trim_torque # always literal zero; never connected to actuation
uint16 sample_count
uint8 pitch_invalid # bitmask: stale=1 nonfinite=2 mapping=4 future=8 range=16
bool enabled
bool valid
bool learning_allowed # detector admission; pitch does not affect bias evidence
bool maneuver_active
bool gate
bool reversal_unwind
bool safety_release
bool limited_by_bmax
bool limited_by_shared_axis
bool limited_by_i
bool limited_by_slew
bool pitch_context_valid
bool reset

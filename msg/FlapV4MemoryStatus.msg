# V4 raw-I-equivalent prototype memory. NOT Nm, NOT physical servo angle.
# Transfer snapshots precede native update; post_natural follows native update.
uint64 timestamp
uint32 reset_epoch
uint32 window_sample_count
uint8 nav_state
# ResidualSlowFeedforwardMemory::Reason numeric value; includes non-recovery holds/exits.
uint8 recovery_reason
bool enabled
bool active
bool enable_pending
bool learn_valid
bool apply_valid
bool b_at_cap
bool window_valid
bool evidence_fresh
bool evidence_gap_reset
bool allocator_fresh
bool allocator_pos_sat
bool allocator_neg_sat
bool handback_active
bool recovery_active
# Recovery after output calculation; earlier I/B/T snapshots remain historical.
bool output_recovery
float32 b_pre_raw
float32 b_raw
float32 q_hat_raw
float32 r_hat_raw
float32 innovation_s_raw
float32 i_pre_transfer_raw
float32 i_post_transfer_raw
float32 i_post_natural_raw
float32 t_pre_transfer_raw
float32 t_post_transfer_raw
float32 t_post_natural_raw
float32 requested_delta_b_raw
float32 accepted_delta_b_raw
float32 window_span_s
float32 window_std
float32 window_sign_fraction
# -1 means missing or future sample timestamp
float32 evidence_age_ms
float32 imax
float32 bmax
float32 b_after_gain
float32 i_cycle_end_raw
float32 b_cycle_end_raw

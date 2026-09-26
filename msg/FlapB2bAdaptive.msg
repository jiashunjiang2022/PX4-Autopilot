# Read-only Adaptive V3 B2b diagnostics. Never consumed by an actuator path.
uint64 timestamp
# Configured request vs actual episode-latched mode; 0 ORIGINAL_SLOW, 1 PAIRED_LPF.
# Invalid requests use the existing safety exit. Change treatment only between episodes.
int32 comparator_mode_requested
int32 comparator_mode
bool comparator_active # Valid LPF adaptive Hold evaluated this cycle
bool original_slow_gate_active # Original qualification logic evaluated, not gate approval
bool reversal_logic_active # Original reversal logic evaluated, not sign opposition
# Existing gate remains false in LPF; reversal below is sign opposition only.
# Match rate_ctrl_status by timestamp for before/after I/S and accepted/requested deltas.
# total_equivalent_i_raw below is post-transfer, BEFORE natural integration.
bool enabled
bool entry_est_valid
bool gate
bool limited
bool releasing
bool reversal
float32 entry_est_raw
float32 t_hat_raw
float32 target_raw
float32 std_raw
float32 sign_fraction
float32 hold_cap_raw
float32 reserve_raw
float32 transferred_i_raw
float32 residual_i_raw
float32 total_equivalent_i_raw

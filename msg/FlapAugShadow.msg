# Read-only augmentation Shadow diagnostics. This topic never drives actuators.
uint64 timestamp

uint32 implementation_version
uint32 v2_hash_short
uint32 v3_hash_short

bool input_valid
bool fast_roll_valid
bool fast_pitch_valid
bool v3_model_available
bool history_ready
bool maneuver_prediction_valid
bool slow_residual_valid
bool slow_valid
bool rtk_valid

uint8 PHASE_STRAIGHT_OR_LOW_MANEUVER = 0
uint8 PHASE_TURN_ENTRY = 1
uint8 PHASE_STEADY_TURN = 2
uint8 PHASE_TURN_EXIT = 3
uint8 PHASE_EXTREME_OR_INVALID = 4

uint8 flight_mode
uint8 maneuver_phase
bool phase_transition

float32 maneuver_score
float32 slow_confidence

float32 tail_roll
float32 tail_pitch
float32 roll_i
float32 roll_i_norm
float32 tail_margin

# Counterfactual diagnostic under the observed baseline closed loop.
bool roll_i_diagnostic_valid
bool roll_i_update_enabled
float32 roll_rate_error
float32 roll_i_delta_raw
float32 roll_i_delta_pre_imax
float32 roll_i_shadow_no_imax
float32 roll_i_raw_drive_accum

float32 fast_roll_pred
float32 fast_pitch_pred

float32 maneuver_hat
float32 slow_residual
float32 slow_hat
float32 slow_innovation
float32 slow_slew_rate
bool slow_update_enabled

# Hypothetical future augmentation only. Not connected to actuator control.
float32 requested_slow_aug
float32 requested_fast_roll_aug
float32 requested_fast_pitch_aug
float32 projected_slow_aug
float32 projected_fast_roll_aug
float32 projected_fast_pitch_aug
bool projection_active

# Safety proof: hard-coded zero in this Shadow implementation.
float32 actual_slow_injection
float32 actual_fast_roll_injection
float32 actual_fast_pitch_injection

uint32 fast_compute_us
uint32 slow_compute_us
uint32 total_compute_us

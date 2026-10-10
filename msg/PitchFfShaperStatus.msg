# 1f-targeted Pitch FF shaping; 2f states are INTERNAL nuisance regression only.
# Commands below are controller-equivalent, never physical servo angles or moments.
uint64 timestamp
uint64 timestamp_sample
uint64 rate_sp_source_timestamp
uint64 wing_phase_timestamp
uint64 update_counter
float32 controller_dt
float32 measured_dt
float32 phase_rad
float32 phase_age
float32 flap_frequency_hz
float32 q_sp_used
float32 coeff_cos_1f
float32 coeff_sin_1f
float32 coeff_cos_2f_internal
float32 coeff_sin_2f_internal
float32 baseline
float32 fade
float32 warm_cycles
float32 qhat_1f # unfaded valid estimate; slewed residual during phase dropout
float32 qhat_2f_internal # internal only, NEVER applied
float32 qhat_1f_reference # authoritative faded estimate for offline comparison
float32 qhat_2f_reference_internal
float32 qhat_1f_applied # exactly alpha_effective * qhat_1f
float32 estimation_error
float32 alpha_command
float32 alpha_effective
float32 effective_ff_gain
float32 native_ff_term
float32 shaped_ff_term
float32 ff_delta_rate_control
float32 gain_compression_pitch
float32 airspeed_scaling_sq
float32 normalized_correction
float32 native_pitch_control
float32 shaped_pitch_control
float32 native_pitch_preclip
float32 shaped_pitch_preclip
float32 shaped_pitch_final
bool enabled
bool active
bool armed
bool phase_valid
bool phase_fresh
bool frequency_valid
bool warmup_complete
bool fading
bool fixed_wing_active
bool mission_active
bool landed
bool controller_active
uint8 reset_reason # PitchFfPhaseShaper.hpp ResetReason enum, NONE=0

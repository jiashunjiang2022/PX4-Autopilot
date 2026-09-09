# Logging-only physical airspeed quality. Never consumed by flight control or fusion.
uint64 timestamp
uint64 timestamp_sample                  # [us] Original airspeed observation timestamp; 0 for quality-only monitoring
uint64 ekf_buffer_timestamp_sample       # [us] Delayed EKF buffer timestamp; 0 for quality-only monitoring
uint64 quality_timestamp_sample          # [us] Causal quality-input snapshot used
uint32 quality_age_us                    # [us] Quality age at observation; UINT32_MAX for quality-only monitoring
bool qmon                                # Quality-only monitoring record without an EKF airspeed observation

uint8 SPECTRAL_INVALID_REASON_NONE = 0
uint8 SPECTRAL_INVALID_REASON_INSUFFICIENT_SAMPLES = 1
uint8 SPECTRAL_INVALID_REASON_INPUT_RATE = 2
uint8 SPECTRAL_INVALID_REASON_NYQUIST = 3
uint8 SPECTRAL_INVALID_REASON_REFERENCE_BAND = 4
uint8 SPECTRAL_INVALID_REASON_FLAP_BAND = 5
uint8 SPECTRAL_INVALID_REASON_ZERO_ENERGY = 6
uint8 SPECTRAL_INVALID_REASON_NONFINITE = 7
uint8 SPECTRAL_INVALID_REASON_INPUT_GAP = 8
uint8 SPECTRAL_INVALID_REASON_NO_RECENT_FLAP = 9

uint8 SPECTRAL_RESET_REASON_NONE = 0
uint8 SPECTRAL_RESET_REASON_EXPLICIT = 1
uint8 SPECTRAL_RESET_REASON_QUALITY_DISABLED = 2
uint8 SPECTRAL_RESET_REASON_AIRSPEED_TIMEOUT = 3
uint8 SPECTRAL_RESET_REASON_NO_RECENT_FLAP = 4

float32 airspeed_q
float32 q_raw
uint8 input_source
uint8 quality_source_instance
uint32 quality_device_id
int8 airspeed_source                     # Selected observation source; -1 for quality-only monitoring
uint32 airspeed_device_id
float32 eas2tas
bool quality_input_valid
float32 input_rate_hz
bool spectral_updated
uint32 spectral_update_counter
uint8 spectral_invalid_reason
float32 spectral_reference_lower_hz
float32 spectral_reference_upper_hz
float32 spectral_flap_center_hz
float32 temporal_raw
float32 temporal_filtered
float32 temporal_normalized
bool temporal_valid
float32 nominal_r_as                     # NaN: monitoring does not represent an EKF observation
float32 r_as_used
float32 effective_flap_ratio
bool flap_active
float32 flap_frequency_hz
float32 spectral_ratio
float32 spectral_input_m_s
bool spectral_ratio_valid
bool flap_freq_timed_out
bool q_is_dv_only
uint16 spectral_window_count
uint16 spectral_min_samples
uint32 spectral_age_ms
uint8 spectral_reset_reason
uint32 flap_active_streak_ms
uint32 flap_recent_true_age_ms
uint32 quality_input_missed_count
uint32 estimator_update_time_us
uint32 spectral_evaluation_time_us

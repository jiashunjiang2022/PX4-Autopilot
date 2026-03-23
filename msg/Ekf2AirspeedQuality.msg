uint64 timestamp

uint8 GATE_REASON_NONE = 0
uint8 GATE_REASON_FORCE_BASELINE = 1
uint8 GATE_REASON_Q_BELOW_OFF = 2
uint8 GATE_REASON_HOLD = 3
uint8 GATE_REASON_Q_ABOVE_ON = 4
uint8 GATE_REASON_DV_ONLY = 5

float32 airspeed_q
float32 q_raw
float32 q_smoothed
float32 r_as_used
bool fuse_enabled
bool flap_active
float32 flap_frequency_hz
float32 spectral_ratio
float32 spectral_input_m_s
float32 dv
bool spectral_ratio_valid
bool enough_window
bool do_eval
bool flap_freq_timed_out
bool q_is_dv_only
uint16 spectral_window_count
uint16 spectral_min_samples
uint32 spectral_age_ms
uint8 gate_reason

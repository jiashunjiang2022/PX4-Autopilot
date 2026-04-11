uint64 timestamp                # time since system start (microseconds)

float32 phase_rad               # relative flapping drive phase wrapped to [0, 2*pi) [rad]
float32 phase_unwrapped_rad     # relative flapping drive phase continuously unwrapped from driver start [rad]
float32 phase_sin               # sin(phase_rad)
float32 phase_cos               # cos(phase_rad)
float32 flap_frequency_hz       # flapping frequency [Hz]
uint32 encoder_position_raw     # AS5600 raw encoder position [0, 4095]
int64 encoder_total_count       # accumulated encoder counts from driver start
bool phase_valid                # true when encoder data and flap ratio are valid; does not imply calibrated absolute zero

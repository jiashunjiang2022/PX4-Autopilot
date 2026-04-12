uint64 timestamp

float32 phase_deg               # legacy phase for FUSION control consumers [deg]
float32 phase_rad               # Hall-indexed phase wrapped to [0, 2*pi) [rad]
float32 phase_unwrapped_rad     # Hall-indexed phase since most recent Hall zero [rad]
float32 phase_sin               # sin(phase_rad)
float32 phase_cos               # cos(phase_rad)
float32 flap_frequency_hz       # flapping frequency [Hz]

int64 total_count               # accumulated encoder counts from driver start
uint32 encoder_position_raw     # AS5600 raw encoder position [0, 4095]
uint32 hall_pulse_count         # hall pulse counter latched from rpm_capture
bool phase_valid                # true when a Hall zero has been latched

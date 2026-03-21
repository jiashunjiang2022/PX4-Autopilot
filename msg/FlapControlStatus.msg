uint64 timestamp
uint64 timestamp_sample

uint8 mode                            # FLAP_FM_MODE at publish time

float32 f_eff_sp_hz                   # Base effective frequency setpoint from throttle mapping
float32 f_inst_sp_hz                  # Instantaneous frequency setpoint after modulation
float32 rpm_sp                        # Instantaneous motor RPM setpoint
float32 rpm_meas                      # Measured motor RPM
float32 omega_ref_rad_s               # Instantaneous angular speed reference [rad/s]
float32 omega_meas_rad_s              # Measured angular speed [rad/s]

float32 phase_deg                     # Measured phase used by modulation [deg, wrapped 0..360]
float32 phase_ref_deg                 # Internal phase reference [deg, wrapped 0..360]
float32 phase_err_deg                 # Wrapped phase error [deg, -180..180)

float32 delta_cmd                     # Commanded downstroke time ratio
float32 delta_slewed                  # Slew-limited delta command
float32 delta_applied                 # Delta after limit/constraint handling
float32 delta_meas                    # Measured downstroke ratio from hall-cycle timing
float32 f_up_cmd_hz                   # Commanded upstroke instantaneous frequency [Hz]
float32 f_down_cmd_hz                 # Commanded downstroke instantaneous frequency [Hz]
bool in_downstroke                    # True when current aligned phase is inside the downstroke half-cycle
float32 wave_cmd                      # Commanded normalized flap waveform sample from phase_ref_deg
float32 wave_meas                     # Measured normalized flap waveform sample from phase_deg

float32 u_ref                         # Upstream normalized thrust reference [0..1]
float32 u_pid                         # PID contribution added to u_ref
float32 u_out                         # Published normalized thrust [0..1]

uint8 CURRENT_SOURCE_NONE = 0
uint8 CURRENT_SOURCE_ESC = 1
uint8 CURRENT_SOURCE_BATTERY = 2
uint8 current_source

float32 i_meas_a                      # Measured current used by limiter [A]
float32 v_meas_v                      # Measured voltage paired with limiter source [V]
float32 p_meas_w                      # Instantaneous electrical power estimate [W]
float32 i_limit_a                     # Current limit parameter [A], 0 if disabled

uint16 FLAG_PHASE_INVALID = 1
uint16 FLAG_CURRENT_LIMIT = 2
uint16 FLAG_OUT_SAT_LOW = 4
uint16 FLAG_OUT_SAT_HIGH = 8
uint16 FLAG_DELTA_REDUCED = 16
uint16 FLAG_FEEDBACK_INVALID = 32
uint16 saturation_flags

# TOPICS flap_control_status

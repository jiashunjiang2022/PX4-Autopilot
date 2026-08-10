# Uniform, filtered Pitot input used only by the airspeed quality estimator.

uint64 timestamp                         # [us] Publication time
uint64 timestamp_sample                  # [us] Uniform 50 Hz sample-grid time

uint8 INPUT_SOURCE_UNKNOWN = 0
uint8 INPUT_SOURCE_DIFFERENTIAL_PRESSURE = 1

uint8 RESET_REASON_NONE = 0
uint8 RESET_REASON_INITIALIZATION = 1
uint8 RESET_REASON_DUPLICATE = 2
uint8 RESET_REASON_NON_MONOTONIC = 3
uint8 RESET_REASON_LONG_GAP = 4
uint8 RESET_REASON_INVALID_PRESSURE = 5
uint8 RESET_REASON_INVALID_AIR_DATA = 6
uint8 RESET_REASON_PARAMETER_UPDATE = 7
uint8 RESET_REASON_DEVICE_CHANGE = 8
uint8 RESET_REASON_INSTANCE_CHANGE = 9
uint8 RESET_REASON_DEVICE_ERROR = 10
uint8 RESET_REASON_PRESSURE_RANGE = 11
uint8 RESET_REASON_RATE_INVALID = 12
uint8 RESET_REASON_RATE_CHANGE = 13
uint8 RESET_REASON_DEVICE_ID_INVALID = 14

uint32 device_id                         # Differential-pressure sensor device ID
uint8 source_instance                    # differential_pressure uORB instance
float32 differential_pressure_pa         # [Pa] Calibrated and anti-alias filtered pressure
float32 indicated_airspeed_m_s           # [m/s] Pitot-derived IAS
uint8 input_source                       # [@enum INPUT_SOURCE] Physical input source
bool valid                               # True only for a fresh, bracketed output sample
bool resampled                           # True when interpolated between two source samples
float32 measured_source_rate_hz          # [Hz] Measured differential-pressure producer rate
bool rate_valid                          # True after a stable source-rate estimate is inside the contract
float32 filter_source_rate_hz            # [Hz] Source rate used for current filter coefficients
float32 filter_cutoff_hz                 # [Hz] Configured anti-alias cutoff
float32 output_rate_hz                   # [Hz] Fixed output-grid rate
uint32 gap_count                         # Source discontinuities since startup
uint32 rate_reset_counter                # Filter resets caused by rate invalidity/change
uint8 reset_reason                       # [@enum RESET_REASON] Most recent producer reset

uint8 ORB_QUEUE_LENGTH = 4

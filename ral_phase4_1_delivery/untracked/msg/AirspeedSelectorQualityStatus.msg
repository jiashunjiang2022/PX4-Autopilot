# Airspeed-selector outcome after the optional RA-L quality safeguard.

uint64 timestamp                         # [us] Publication time
uint64 decision_timestamp_sample         # [us] Sample time of the pre-quality physical source
uint64 quality_timestamp_sample          # [us] Quality sample used for the decision
uint32 quality_age_us                    # [us] Age of the quality sample at the decision

uint8 EXPERIMENT_MODE_BASELINE = 0
uint8 EXPERIMENT_MODE_CONSTANT_R = 1
uint8 EXPERIMENT_MODE_VARIANCE_ONLY = 2
uint8 EXPERIMENT_MODE_FULL_PROPOSED = 3

uint8 REJECTION_REASON_NONE = 0
uint8 REJECTION_REASON_ORIGINAL_PX4_INVALID = 1
uint8 REJECTION_REASON_QUALITY_SAFEGUARD = 2
uint8 REJECTION_REASON_SENSOR_TIMEOUT = 3
uint8 REJECTION_REASON_SOURCE_UNAVAILABLE = 4
uint8 REJECTION_REASON_FALLBACK_SELECTED = 5
uint8 REJECTION_REASON_OTHER = 6

uint8 TRIGGER_REASON_NONE = 0
uint8 TRIGGER_REASON_LOW_QUALITY = 1
uint8 TRIGGER_REASON_QUALITY_STALE = 2
uint8 TRIGGER_REASON_QUALITY_INVALID = 3
uint8 TRIGGER_REASON_ORIGINAL_PX4_INVALID = 4
uint8 TRIGGER_REASON_BLOCKAGE = 5
uint8 TRIGGER_REASON_SOURCE_ID_MISMATCH = 6
uint8 TRIGGER_REASON_SENSOR_TIMEOUT = 7
uint8 TRIGGER_REASON_SOURCE_UNAVAILABLE = 8

uint8 FALLBACK_OUTCOME_NONE = 0
uint8 FALLBACK_OUTCOME_ALTERNATE_PHYSICAL = 1
uint8 FALLBACK_OUTCOME_GROUND_MINUS_WIND = 2
uint8 FALLBACK_OUTCOME_SYNTHETIC = 3
uint8 FALLBACK_OUTCOME_UNAVAILABLE = 4

uint8 experiment_mode
bool selector_quality_enabled
int8 pre_quality_source
uint32 pre_quality_device_id
uint32 quality_device_id
uint8 quality_source_instance
bool source_identity_match
bool original_valid
bool concurrent_original_invalid
bool concurrent_blockage
bool quality_rejected
uint8 rejection_reason                    # [@enum REJECTION_REASON]
uint8 trigger_reason                      # [@enum TRIGGER_REASON]
uint8 fallback_outcome                    # [@enum FALLBACK_OUTCOME]
bool fallback_attempted
bool fallback_available
int8 fallback_source
int8 final_selected_source
uint32 final_device_id
bool final_valid

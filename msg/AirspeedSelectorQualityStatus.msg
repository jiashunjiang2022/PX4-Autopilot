# Passive telemetry of the official selector output; no quality gate or custom fallback.
uint64 timestamp                         # [us] Publication time, matches airspeed_validated
uint64 decision_timestamp_sample         # [us] Physical sample time, or publication time for nonphysical sources
int8 final_selected_source               # Same source identifiers as AirspeedValidated
uint32 final_device_id                   # 0 for a nonphysical source
bool original_sensor_valid               # Official validator status for the selected physical sensor
bool final_valid                         # IAS, CAS and TAS are finite
bool fallback_available                  # Valid ground-minus-wind or synthetic output selected

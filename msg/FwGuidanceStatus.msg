uint64 timestamp

uint8 guidance_mode           # 0 = NPFG, 1 = L1, 2 = L1_WIND
bool guidance_active          # true if the selected guidance mode produced a valid command
bool degraded_to_l1           # true if L1_WIND fell back to plain L1 behavior

float32 track_error           # [m] signed cross-track error
float32 raw_track_error       # [m] raw signed cross-track error before any fallback handling
float32 course_setpoint       # [rad] guidance course setpoint
float32 lateral_acceleration  # [m/s^2] lateral acceleration command
float32 roll_setpoint         # [rad] roll setpoint derived from lateral acceleration
float32 switch_distance       # [m] path switch distance

float32 nav_bearing           # [rad] navigation bearing
float32 target_bearing        # [rad] target bearing
float32 ground_speed          # [m/s] horizontal ground speed
float32 airspeed_ref          # [m/s] airspeed reference used by guidance
float32 wind_correction       # [rad] applied wind correction angle
float32 wind_speed            # [m/s] horizontal wind speed estimate
float32 airspeed_used         # [m/s] airspeed used for wind compensation
bool wind_valid               # true if the wind estimate is valid
bool airspeed_valid           # true if the airspeed estimate is valid
float32 base_course           # [rad] guidance course setpoint before wind correction

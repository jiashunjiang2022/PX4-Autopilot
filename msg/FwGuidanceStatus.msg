uint64 timestamp

uint8 guidance_mode           # 0 = NPFG, 1 = L1, 2 = PID

float32 track_error           # [m] signed cross-track error
float32 course_setpoint       # [rad] guidance course setpoint
float32 lateral_acceleration  # [m/s^2] lateral acceleration command
float32 roll_setpoint         # [rad] roll setpoint derived from lateral acceleration
float32 switch_distance       # [m] path switch distance

float32 nav_bearing           # [rad] navigation bearing
float32 target_bearing        # [rad] target bearing
float32 ground_speed          # [m/s] horizontal ground speed
float32 airspeed_ref          # [m/s] airspeed reference (NPFG only, NAN otherwise)

uint64 timestamp        # time since system start (microseconds)
uint64 timestamp_sample # timestamp of the data sample on which this message is based (microseconds)

# Normalized motor command for flapping-wing RPM/frequency control.
# Range: [0, 1]. NaN means "invalid / do not use".
float32 thrust

# TOPICS flap_motor_setpoint

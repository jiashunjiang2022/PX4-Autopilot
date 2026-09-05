# Terms used in one fixed-wing rate controller update, before output airspeed
# scaling, gain compression, trim, yaw override/feedforward and limiting.
# Axes are in the controller frame (fixed-wing frame for tailsitters).
uint64 timestamp                 # publication time [us]
uint64 timestamp_sample          # angular velocity sample time [us]
float32[3] p_term
float32[3] i_term                 # integral used in this output, before integration update
float32[3] d_term                 # includes the negative derivative sign
float32[3] ff_term
float32[3] output                 # sum of P/I/D/FF

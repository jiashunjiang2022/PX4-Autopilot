CONTROL_PATH_NON_INTERFERENCE=PASS
CONTROL_INJECTION_CODE_PATH_EXISTS=NO

Hardening changes only diagnostic history, timing, and uORB publication. No
prediction field is read by control_u, rate integral, torque/actuator setpoints,
allocator, gain compression, airspeed scaling, or Slow transfer logic.

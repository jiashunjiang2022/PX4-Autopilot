# Platform source isolation

Run: python3 tools/roll_tbr_validation/verify_clean_native_platform.py.
Result: PASS. Evidence: /tmp/roll-tbr-formal-lineage-20261009/stage_a_source_audit.log.

Native src/modules/fw_rate_control, src/lib/rate_control, src/modules/fw_att_control,
src/modules/control_allocator, src/lib/control_allocation, src/modules/ekf2,
src/modules/sensors, src/modules/airspeed_selector, src/drivers/rc and
src/lib/dataman_client have zero diff from ec278758.
The native matrix-0 saturation subscription behavior is deliberately retained.

Sensor directories/messages are byte-identical to 26418be sensor-only snapshot.
Minimal message, logger and board registrations are the only surrounding integration.
AS5600 estimates absolute frequency directly from its encoder RPM / (60*FLAP_RATIO),
with 0.2 Hz floor, at 100 Hz. It has no Hall/WingPhase subscription.
Hall/RPMCapture only provides pulse telemetry/filtering; phase requires mechanical-zero
lock. Hall availability cannot gate AS5600 frequency.

Handover five-file diff equals bd6c81c457 patch exactly (metadata ignored).
It gates the entire attitude-setpoint publisher on auto-entry pending until a fresh
non-idle longitudinal setpoint. No timeout, no auto-to-auto rearm, no Roll error or
experimental/treatment parameters. It is shared unchanged by all future A/B/C treatments.

Fixed-wing-only board selection uses only the 753a223354 board hunks. EKF hunks omitted.
Pitch trim d5d881 and roll range a292c125 omitted: actual-aircraft evidence unavailable.
e624a99 anti-windup correction and 4cd4d0a CRSF retry explicitly omitted.

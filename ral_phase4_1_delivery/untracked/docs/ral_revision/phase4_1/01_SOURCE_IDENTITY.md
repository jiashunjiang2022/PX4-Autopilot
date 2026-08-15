# Source identity

`airspeed_quality_input` carries `source_instance`, `device_id`,
`timestamp_sample`, and `valid`. The producer stores the last device, instance,
error count, and sample timestamp. Device/instance changes, device errors,
timestamp faults, long gaps, range faults, invalid air data, invalid source rate,
and relevant parameter changes reset rate estimation, filter state,
interpolation state, and the next 50 Hz output timestamp.

Every runtime fault with a current DP sample now publishes `valid=false` with an
explicit reset reason. `device_id=0` is rejected as
`RESET_REASON_DEVICE_ID_INVALID`; it cannot create a valid quality sample.

For the formal single-Pitot platform, selector identity is true only for exactly
one physical airspeed sensor, pre-quality source 1, DP instance 0, nonzero equal
device IDs, and a valid/fresh quality record. The selector supplies this strict
proof to EKF2. Raw-air-speed fallback cannot prove the sensor-count invariant, so
adaptive R and the quality fusion gate remain disabled on that path.

The offline preflight checker fails multiple active DP instances, multiple or
zero Pitot IDs, producer/EKF/selector identity mismatch, and unexplained final
source transitions.

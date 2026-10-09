# Hardware qualification blockers

READY_FOR_FLASH=NO
READY_FOR_FLIGHT=NO

Actual aircraft parameter file, airframe identity, RC/serial configuration, allocator
geometry, servo/channel/function mapping, mechanical ratio, Hall wiring/filter settings,
travel limits, directions and calibration are NOT_AVAILABLE / NOT_CERTIFIED.
No device was flashed, armed, operated or flown.

Verify actual parameters and RC/allocator/servo mapping, bench-test native platform,
then conduct native qualification before TBR bench/flight approval. Validate AS5600
frequency independently with Hall disconnected; validate Hall origin/interpolation and
CRSF startup separately. Handover has no timeout under donor semantics; verify entry
setpoint production and safe operator recovery on actual hardware.

Do not infer P0, DeltaP, trim/range or shared-tail estimates from repository defaults.
Offline TBR candidates do not certify flight values. No pitch trim restoration was
made without aircraft evidence. Memory headroom and runtime logging loss/CPU load
require bench measurement on the actual platform.

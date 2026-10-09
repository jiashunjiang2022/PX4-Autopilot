# Clean Native V2 Pitch Trim Restoration

Parent: 95c646524ab54d6c6ecda12121bb8623d83ea366.
Donor: d5d881421891f7a706a48b3b8ce07303f406d055.
Historical flight source: 926693950a4b2fe448d4b18d859c54bf9aa8a260.

V2 manually ports the donor's two longitudinal trim hunks.
The complete restored longitudinal .cpp file matches the historical flight blob.
No donor commit is cherry-picked; no experimental implementation is imported.

Let o=radians(FW_PSP_OFF). Both versions pass body pitch-o and pitch limits-o to
TECS. Historical/V2 publish TECS pitch+o as body pitch and tecs_status.pitch_sp_rad;
V1 publishes TECS pitch alone. Finite direct pitch bypasses this reconstruction.
Positive offset increases reconstructed body pitch. No rate gain, actuator trim,
controller error or Roll behavior is patched.

The existing mode gate, handover pending gate, finite-body fallback, manual pitch
limits, parameter-update subscription and resets are retained. Restoration adds
no state. The host test compiles extracted production expressions and guards with
PX4 degree/radian math and independently checks values, signs, invalid inputs,
direct/manual/auto modes and offset changes. It also requires exact historical
longitudinal-file identity. It does not emulate the uORB scheduler or prove flight
trim correctness.

The known V1 saturation-feedback difference remains for Native qualification.
Historical FW_PSP_OFF and all P/I/D/FF/IMAX values remain unchanged in final params.
FORMAL_P0_FROZEN=NO. READY_FOR_FLASH=NO. READY_FOR_FLIGHT=NO.

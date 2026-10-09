# Roll TBR V2 Source Isolation

Direct parent: c42789a3f0143d4be9d327f6c83c00896c0b4df7 (Clean Native V2).
Implementation donor surface: 95c646524ab54d6c6ecda12121bb8623d83ea366
to 8cda2de9dc9ad945066980e34486264afde60847.

All executable TBR blobs, message definitions, five parameter declarations and
host-test sources match TBR V1 exactly. Only the source isolation verifier's
parent constant changes to Clean V2. No mixed commit is cherry-picked and no V1
history is rebased, reset or moved. The restored Pitch trim is inherited from the
direct parent and does not appear in the Clean V2 to TBR V2 diff.

The existing disabled-native parity, fail-zero, Mission gate, cap, slew, reset,
frequency-response and synchronous controller-diagnostic checks are rerun against
this tree. Build/test evidence and final firmware/parameter hashes are recorded in
the external formal-roll-hardware-audit-20261009 restoration manifest.

FLAP_TBR_EN=0 in final parameters. No Roll algorithm, saturation-feedback correction,
Pitch gain, allocator, CRSF, AS5600/Hall, airspeed, handover, PWM or board change is
added. FORMAL_P0_FROZEN=NO. READY_FOR_FLASH=NO. READY_FOR_FLIGHT=NO.

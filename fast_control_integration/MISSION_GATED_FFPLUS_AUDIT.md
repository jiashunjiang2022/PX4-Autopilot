# Mission-gated static roll FF+ audit

Base: b984e3352d86f2db51dc608c5c9f8e8092543249
Branch: mission-gated-ffplus-20260924

The patch adds only FLAP_FFP_EN (default 0) and FLAP_FFP_D (default 0.0, metadata/runtime
range -0.2..0.2). The experiment value .025 is not a firmware default. The helper validates
finite base/delta and finite base+delta; invalid delta fails back to base FW_RR_FF.

FF+ activation is exactly enabled + armed + AUTO_MISSION + not landed + fixed-wing +
non-VTOL + not transition + not failsafe. It has no dependence on T, predictor, p_error,
I/S, actuator margin, FAST thresholds or FAST state. Leaving Mission immediately selects
the base gain on the next controller update; no state is retained. Pitch and yaw gains
remain unchanged. Existing effective gain is passed through the original airspeed scaling
and RateControl feed-forward path.

The synchronous flap_fast_control message records ffplus_enabled, ffplus_active,
ffplus_config_valid, ffplus_delta, base_roll_ff_gain, and effective_roll_ff_gain. The
topic is already published independently of FAST enable, so OFF and FF+ captures carry
these fields. Simultaneous FF+/FAST enable is not overridden; both paths remain finite
and independently observable.

Tests:
- host sanitizer test: PASS; OFF, pre-Mission, Mission .800+.025=.825, Mission exit,
  invalid NaN fallback, and existing FAST/actuator tests.
- source isolation: PASS; predictor/model files byte-identical to parent, FAST defaults
  and shared-tail implementation unchanged, baseline arithmetic/order preserved.
- parameter generation/build: PASS with make px4_fmu-v6c_default (exit 0), FLASH
  1856260 B / 1920 KB (94.41%), AXI SRAM 91212 B / 512 KB (17.40%).
- firmware SHA256: aa451db2afa8bab8ccbacae0a855b20a29ac21e7f9ffaa0369b026c22a0d98ce.
No vehicle parameters were changed and no flash was performed.

Planned profiles:
OFF: FW_RR_P .050, I .100, D 0, FF .800, IMAX .200, FLAP_SLOW_EN 1,
FLAP_FFP_EN 0, FLAP_FFP_D .025, FLAP_FAST_EN 0.
MISSION-GATED FF+: same, FFP_EN 1, D .025, FAST_EN 0; expected .800 outside Mission,
.825 in Mission, .800 after exit.
FAST K3: same baseline, FFP_EN 0, D .025, FAST_EN 1, K 3, MAX .015, TON .150,
TOFF .180, SLEW .150.

FAST model coefficients, scaler, model timing, Slow/B2B, allocator, FAST defaults/ranges,
T gate, margin, stale, warmup and Mission-only FAST logic are unchanged.
READY_FOR_PILOT_FLASH=NO.

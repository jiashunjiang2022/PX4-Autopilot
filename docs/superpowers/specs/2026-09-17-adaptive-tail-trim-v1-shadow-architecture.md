# Automatic Roll working-point trim: shadow architecture

Baseline: 7cd690c5bcf55e0ab18eb12749d54383bca3b194. Branch:
adaptive-tail-trim-v1-shadow-20260917. No active integration is authorized.

Adaptive Tail-Trim V1 is automatic Roll working-point trim driven by persistent
closed-loop burden, not a second Slow controller. RC trim, manual inputs and
attitude/rate setpoints are never changed. The existing pure core is unchanged.

## Coordinates and epochs

Actual source order in FixedwingRateControl::Run at the baseline:
g snapshot (378), B2B update (409), PID torque computation and subsequent natural
I update inside RateControl::update (460), pre-update gain snapshot (466–467),
gain-compression update for NEXT cycle (486), torque publication (627), new
shadow hook. RateControl computes torque before integrating I. S is unchanged
during natural integration; both getters at the hook describe the same end-of-
cycle state, including any actual recovery reset. They are NOT the I/S pair used
to form the just-published torque. No previous Result.residual_i_raw is used.

T_actual = current native I + current transferred S. S is read even during
existing Slow exit, while nonzero S still contributes to actual output. In the
fully OFF state S=0. B_obs=g_cycle*T_actual/1.1 uses the retained pre-compression
g, not the newly updated gain. This is a causal end-of-cycle burden observation,
not an instantaneous reconstruction of the published PID torque.

I_shadow=T_actual+virtual_offset; only accepted virtual DeltaI changes offset.
Normal transfer tolerance is 2e-6 torque units, declared before testing.
Safety release may have finite unmatched handback, as specified by the pure
core; it must be logged, never hidden as conserved. No permanent equality to
B_obs is claimed when g or natural T changes.

## Minimal persistent detector and separate admission

B_obs -> causal LPF -> raw-observation persistence/std/sign gate -> b_target
-> maneuver freeze / actuator admission / core -> virtual states -> logging.
Pitch, its validity, envelope and reserves do NOT enter the estimator. A pitch
failure stops admission and requests virtual safety release without discarding
otherwise valid bias evidence. Control invalidity and maneuver do clear evidence.

Reuse read-only B2B TAU/GWIN/GSIGN. Convert GSTD each cycle to g*GSTD/1.1:
historical physical samples contain historical g, so changes in g may legitimately
reduce stability acceptance. This is not constant-g raw/physical equivalence.
20 Hz causal held samples, fixed capacity 200 (10 seconds), no catch-up copies.
Invalid gaps and maneuver clear LPF/history; reversal reaching zero also clears
history, and the next cycle starts a full new window. LPF uses dt/(tau+dt).

Five parameters only: FLAP_TTR_SHDW (default 0), BMAX (.08, ceiling .10),
BRSV (.03 physical), BSLW (.005 physical/s), RRSV (.15 each direction).
All magnitudes are PROVISIONAL_SHADOW_TUNING, not flight-approved limits.
Internal provisional constants: g minimum .01 (existing pure-core diagnostic
floor, GMIN_DEPLOYMENT_VALIDATED=NO), phi entry/exit 10/5 degrees, p entry/exit
10/5 deg/s, exit dwell 1 s, pitch release tau 1 s, freshness .2 s.
These fixed constants avoid unnecessary runtime tuning controls; flight evidence
must validate them before active use. Changing any configuration resets shadow.

## Isolation and failure behavior

Coordinator takes values only. No RateControl/B2B/torque/actuator pointers.
Hook after publication has a dedicated diagnostics publisher only. Actual trim
torque is a literal zero. No active enable parameter. SHDW=0 resets state and
publishes one terminal reset record if previously enabled; otherwise no logging.

Enable rising, disable, disarm, landed, reset epoch or invalid config reset all
virtual states. Invalid g/nonfinite burden or virtual I outside +/-FW_RR_IMAX
also resets with invalid diagnostics; never silently clamp the actual baseline.
Normal maneuver holds target and b; previously latched unwind and safety may
continue toward zero. Fresh evidence is required on exit.

Canonical non-VTOL attitude setpoint q_d provides phi_sp; rate setpoint roll
provides p_sp. Both require finite, causal, fresh samples. Pitch uses verified
actuator_servos control[0/1], pre-reversal, and the existing exact three-surface
airframe verifier. No generic-airframe mapping claim. Stale, nonfinite, future,
mapping and range failures have distinct diagnostic bits.

## Verification protocol

Behavioral RED against buildable empty helpers, then production helpers GREEN;
unit tests for detector, maneuver, envelope, coordinator and actual-object
isolation; production output arithmetic extraction for exact OFF/ON comparison;
existing core/B2B/RateControl/allocator regressions; both production builds.
Old raw-V3 semantic RED01–04 remain expected RED. Runtime smoke is reported
separately from build success; no simulated result substitutes for flight evidence.
Preserve the user NuttX tools/jlink-nuttx (SHA256
3c5fbd59358b5efe7586c0be90ebb4cd7b376f9371e889ca36468276b1375c66).

# Adaptive Tail-Trim V1 — design only, 2026-09-17

Base: a644ca7868f07004e45658d4ae9a7a8c880c6bb5.
Branch: adaptive-tail-trim-v1-design-red-20260917.
No production implementation, parameter changes, firmware build, push or flight authorization.
Source references below refer to this base and are repository-relative.

## 1. Motivation

Separate a persistent normalized tail working point from reactive feedback while retaining native PID, bounded adaptation and allocator visibility. Bigger trim is not the objective.
Input evidence: /Users/jiangjiashun/Documents/门控数据分析/Slow_trim_feasibility_20260917/SLOW_TRIM_FEASIBILITY_REPORT.md.
Morning log6–13 only: current S physical-equivalent mean |bias| .03904, flight means .02130–.05906, P95 .04663–.07170, max .08311; magnitude burden coverage 49.8%.
Reserve-matched causal coverage at caps .08/.10/.15/.20 is 49.66/50.22/50.22/50.22%; full-bias causal coverage 74.25/80.73/82.54/82.54%. This is descriptive replay using old logged gates, not a new closed-loop controller evaluation. Log11 lacks exact cross-topic matches. Trim-only margin excludes dynamic PID. No evening data or aerodynamic truth.

## 2. Current V3 architecture: source audit

- src/lib/rate_control/rate_control.cpp:183 computes P + I - D + FF; :193 performs natural integration AFTER computing the output. Transfer must therefore precede this snapshot.
- src/modules/fw_rate_control/FixedwingRateControl.cpp:372 computes g = gain_compression.roll * airspeed_scaling². :403 calls transfer; :405 handles reset_required; :409 takes S; :453 calls native rate control.
- Same file :460 snapshots gains_before_update; :467 adds S in raw controller coordinates; :469 scales by gains and airspeed²; :481 updates gain compression for the NEXT cycle; :490 constrains control_u + existing trim into torque setpoint.
- Same file :541 computes effective torque g*S; :585–587 log g, torque, tail-equivalent torque/1.1. :566 uses post-natural I, whereas adaptive diagnostics :589 onward belong to the pre-natural transfer step. Replay must distinguish these epochs.
- src/modules/fw_rate_control/BumplessRollITransfer.hpp:128 update; :137 detects epoch mismatch; :185 checks g>0; :677 enters recovery requesting caller reset. FixedwingRateControl.cpp:118 resets Roll I and synchronizes transfer.
- RateControl::applyRollITransfer at rate_control.cpp:62 bounds accepted I transactions. V3 conserves ΔI+ΔS, with HR=0 total |I+S|<=IMAX. S is transferred raw-I state.
- Allocator feedback: ControlAllocator.cpp:610 computes requested minus allocated torque; FixedwingRateControl.cpp:334–366 maps its signs to saturation flags; rate_control.cpp:219–227 prevents integral growth in the saturated direction.
- Geometry validation: FixedwingRateControl.cpp:100 and flap_aug_shadow/flap_aug_shadow.cpp:123 check left/right roll -.55/+.55, pitch +1/+1, zero trims, channel assignments, REV=17. Servo limits ControlAllocator.cpp:526 onward; publication :748. No second reversal.
- Fixed-wing effectiveness inherits normalize-RPY=false (src/lib/control_allocation/actuator_effectiveness/ActuatorEffectiveness.hpp:155). ControlAllocationPseudoInverse.cpp:116 uses unit roll/pitch scale in this case. Verified configuration is a precondition, not an assumption about every PX4 airframe.

## 3. Problem statement

A raw cap .10 is not a tail cap .10: physical-equivalent contribution is g*S/1.1. Native residual semantics, positive/negative shared authority and maneuver contamination must be explicit.

## 4. Why S is trim-like but not actuator trim

S holds DC demand outside native I but is still multiplied by g. Relabelling S as b fails both gain invariance and physical handoff. Recomputing b=g*S/1.1 every cycle is only a diagnostic coordinate transformation; it does not implement a held physical working point.

## 5. Goals

Normalized tail-state identity; residual native I; allocator-visible complete demand; accepted physical transfer conservation; causal persistent estimation; shared authority, finite exits and auditable state epochs.

## 6. Non-goals

No new Fast/Early estimator, aerodynamic model, runtime ML, servo-layer injection, production edits or tuning. No promise of lower RMS, better path or roll tracking. No automatic replacement of V3. No true wing-asymmetry torque claim.

## 7. Coordinates

b_R is normalized differential-tail command, not servo angle, Nm or measured aerodynamic torque. K_A=.55-(-.55)=1.1 derives from the verified effectiveness matrix; future implementation must validate geometry and normalization and refuse learning on mismatch, not silently use a magic constant.
tail_pitch=(c_L+c_R)/2; tail_roll=(c_R-c_L)/2; c_L=pitch-roll; c_R=pitch+roll, all PRE_REVERSAL.
Existing scheduled manual trim is a separate baseline contribution: include its roll part in r_PID and its pitch part in pitch context. Do not count it again in b.
The simple geometry also assumes no unaccounted allocator auxiliary flap/spoiler offsets, failures or surface slew constraints. Future activation validation must either incorporate those offsets/limits into available authority or reject that configuration; matrix coefficients alone are insufficient.

## 8. Control architecture and timing (Q1–Q4)

Recommend torque-coordinate addition AFTER g scaling and BEFORE final total demand limiting/publication to allocator. Never actuator_servos postprocessing.
tau_total_requested = g*(P+D+FF+I) + tau_existing_trim + K_A*b.
Freeze one pre-update g snapshot across transfer and output composition. Gain compression update affects next cycle only. Do not compensate a next-cycle gain change by silently changing b.
Allocator receives complete limited torque. Preserve directional saturation feedback from total demand; also account for pre-allocator torque clipping, otherwise allocator cannot report demand it never received. Future integration tests must cover both clipping locations. Native I and trim growth both inhibit worsening saturation; release remains possible.
g must be finite and g>g_min. g_min is a strictly positive configuration-validation threshold to determine by replay/SITL conditioning and accepted-I capacity, NOT an invented flight number. No division or normal transfer below it. Invalid gain triggers recovery/authority removal.
Gain compression feedback should see complete demand in a defined torque coordinate, without scaling b again; future integration must explicitly compare this loop against baseline. This design does not claim closed-loop gain stability from algebra alone.

## 9. Estimator (Q6)

A: T=I+K_A*b/g reuses raw estimator but is singular near zero g and gain-sensitive.
B (recommended): B_obs=g*I_pre/K_A+b; causal LP B_hat, initialized from first valid low-maneuver sample. It directly estimates closed-loop persistent residual corrective demand in tail coordinates; closest semantic migration of V3.
C: low-frequency actual command includes P/D/FF, scheduled trim, maneuvers and saturation; identification requires a maneuver model and is not V1.
Use eligible elapsed-time history, never future samples. Do not multiply a raw filtered signal by current g and call it the same filter. Hold estimator and target in turns; discard persistence history on invalid data/maneuver entry and require fresh evidence after return.
V1 holds constant b across g/airspeed changes. Aerodynamic asymmetry may depend on qbar; b(V) is a later evidence-driven extension, not presumed necessary. Gain steps may change B_obs and must require new persistence before growth.

## 10. Gate and maneuver policy (Q5)

Pre-entry evidence, EWIN, elapsed-time GWIN, standard deviation and same-sign fraction remain distinct. Growth needs validity, low maneuver, persistence and actuator reserve. Release needs none of the persistence gate.
V1 learns only in straight/low-maneuver windows, holds target and b through sustained Loiter except safety release. p_sp alone cannot identify steady coordinated turn: combine phi_sp/bank context, yaw/turn demand, p_sp, tracking error and saturation; missing/stale context disables learning.
P/D and FF indicate transient/command components but I itself can absorb sustained maneuver/model demand. Neither allocator output nor I+b identifies structural bias. Alternative maneuver-conditioned subtraction would permit turn learning but needs separate development evidence.
Thresholds, ages, envelopes and dwell times require replay/SITL selection before implementation acceptance. Current logged gate is not the future gate.

## 11. Target law and reserve

Candidate b_target=sign(B_hat)*min(B_MAX,max(|B_hat|-B_RSVD,0)).
Recommend fixed physical B_RSVD=.03 as an OFFLINE candidate; sensitivity .02/.04. Predictable residual authority and no direct gain jitter. Alternative g*.05/1.1 preserves V3 raw reserve but varies with gain and can move target without burden change. LP/persistence and hold target prevent single-cycle g changes from growth.
Target then intersects directional shared-authority constraints. A blocked target does not license integral-limit violation.

## 12. Bumpless handoff

Normal accepted transaction satisfies g*ΔI+K_A*Δb≈0 with frozen g.
Compute feasible ΔI interval from native I bounds, saturation permissions and proposed trim limits; derive Δb=-g*accepted ΔI/K_A. Commit both once, only after validation. Do not accept requested b with a clipped I delta. If no feasible pair, hold or safety release.
Output uses post-transfer/pre-natural I. Natural integrator update is separately accounted, not blamed on transfer mismatch. Gain changes between cycles are outside this invariant. See math contract for tolerances and emergency priority.

## 13. Shared Roll/Pitch authority (Q7)

Instantaneous |pitch| responds fast but chatters and forgets recent excursions. LP is smooth but underestimates abrupt peaks. Recommend causal fast-attack, decaying max envelope p_env=max(|p_current|,max(0,p_env_previous-d_p*dt)); d_p>0 chosen offline. This is not a future-pitch guarantee.
Use valid current requested pitch (including scheduled trim) plus pre-reversal allocated pitch as a conservative max; stale context forbids growth.
Require |b|+p_env+R_RESERVE<=1 for admission/growth. If tightened context makes existing b infeasible, inhibit growth and enter bounded safety release; allocator instantaneously clips total command independently. Never assert that slew-limited b can always meet an abruptly tightened reserve.

## 14. Asymmetric authority

R_pos=1-|pitch|-b; R_neg=1-|pitch|+b. Log both, even if negative; do not hide violations by clamping diagnostics.
b<0 consumes negative/same-direction reserve R_neg; b>0 consumes R_pos. Propose R_MIN_SAME_DIRECTION and R_MIN_OPPOSITE_DIRECTION as design constraints, initially equal positive R_RESERVE. No evidence supports asymmetric numeric thresholds yet.
At each growth admission require both directional minima. Instantaneous dynamic r must remain between -R_neg and R_pos after allocation, not merely satisfy a trim-only margin.

## 15. Reversal

Persistent opposite target unwinds to accepted zero. On that exact cycle clear entry/reversal/gate history, force gate false. Sample again next update; dt=.02/GWIN=3 gives zero cycle offset 0 through 149 blocked, first gate offset 150. No crossing in one step, no retained opposite evidence. Carry these semantics into physical units.

## 16. Normal exit

Disable completes exit before reentry. Feasible handback is exact physical conservation. If native IMAX blocks it, use largest feasible accepted I compensation and decay remaining trim authority at bounded slew; log physical mismatch. Never raise IMAX to force handback.

## 17. Safety exit

| Trigger | Behavior | Continuity |
|---|---|---|
| Normal disable | exact handback else bounded decay | best effort when constrained |
| Pilot abort | latched safety release, no reentry before zero | best effort |
| Failsafe | same, base failsafe retains priority | best effort |
| Rate control disabled | remove trim before inactive output composition | no continuity promise |
| Config invalid | inhibit normal transaction, safety release if geometry known; otherwise suppress trim output | best effort |
| Reset | atomic I/trim/history epoch reset before output | no continuity promise |
| Nonfinite | no unsafe arithmetic; finite fallback, clear corrupted trim | no continuity promise |
| Actuator margin violation | stop growth, release; allocator enforces instant limits | best effort |

Safety slew may exceed normal slew but remains bounded when state/gain/timing are valid. Hard finite state, native bounds and real actuator limits outrank continuity. Zero/invalid dt cannot provide finite wall-clock convergence; remove invalid output rather than promise a rate. Positive minimum execution dt and slew are explicit convergence assumptions.

## 18. Reset/recovery

No hidden trim after an I reset: caller must complete epoch handshake before composing output, as V3 already does. Clear b, target, estimator validity, all histories, and account for removed torque. Recovery is not an exact handback promise. Pure Mission/Stabilized transitions may hold valid state only if policy allows; rate-disable/disarm/config invalid overrides. No resurrection from stale history.

## 19. Logging proposal (no msg edits)

Core: timestamp/sample epoch; state/reason flags (gate/reversal/releasing/normal_exit/safety_exit/recovery); b_trim_tail, b_target_tail, b_hat_tail; accepted_delta_b/i; physical_transfer_mismatch; tail_pitch_context; roll_reserve_pos/neg; g_current; tau_trim/residual/total; limiter flags bmax/shared.
Optional debug: requested_delta_b/i, std/sign fraction, raw pitch, separate clip residual, reset epoch, compute time. roll_reserve_sym=min(pos,neg) is derivable offline. Requested/accepted deltas needed during validation, but avoid duplicating derived values at flight logging rate.

## 20. Parameter proposal only

| Category | Recommendation |
|---|---|
| REUSE | EWIN/GWIN/GSIGN timing/fraction semantics, enable ownership, native IMAX; do not duplicate names blindly |
| RENAME | raw cap/slew/reserve/std/tau interpretation to explicit tail units; never silently reinterpret saved raw values |
| NEW | geometry-valid gate, positive g_min validation and shared dynamic reserve only where not derivable |
| DEPRECATE | raw-S HR expansion for this new mode; retain it for separate current V3 baseline |

Possible names FLAP_TRIM_EN/R_MAX/R_SLEW/RSVD/RRSV/TAU/EWIN/GWIN/GSTD/GSIGN are proposals, not actual parameters; enforce PX4 name length in implementation. Nominal cap .08–.10; proposed configuration hard maximum .10 for V1 pending new evidence. .15/.20 are not authorized defaults. Numeric slew/gates/reserves other than offline sensitivity candidates remain unresolved acceptance inputs.

## 21. Failure modes

Gain singularity; partial-I acceptance; stale pitch/maneuver data; gain snapshot mismatch; total-torque clipping unseen by antiwindup; wrong effectiveness normalization; reversal evidence reuse; missed epoch reset; allocator saturation; heavy real-time history storage; incompatible parameter migration. All require tests, not assertions of safety from coordinates.

## 22. Unit-test contract

Existing 72 V3 + 17 native tests are mandatory passing regressions. Opt-in future probes expose coordinate/authority API gaps; inherited passes are retained, not forced RED. Float absolute transaction tolerance 2e-6 normalized torque, state tolerance 2e-7; recursive replay tolerance 2e-5 declared before future replay, never loosened after seeing results. New API probes cannot substitute for later physical integration tests.

## 23. Replay/SITL plan

Causal sample alignment and pre/post-natural epochs; OLD/NEW wing, straight, entry/exit, sustained Loiter, mode switches, stale topics, gain ramps/steps, saturations and resets. Compare accepted transactions and both reserves. Offline logged gates cannot stand in for new estimator. SITL must exercise complete allocator feedback and clipping and demonstrate native antiwindup direction, fixed-memory timing and parameter validation. No such controller simulation is performed here.

## 24. Real-flight preconditions

Future GREEN tests, replay parity, SITL allocator feedback, FMUv6C resource/stack checks, exact firmware/config identity, bench signs with reversal applied once, gain-step physical invariance, bounded exits and complete ULog; then separate human authorization. This task authorizes no flight.

## 25. Comparison baselines (Q8)

A PX4 Slow OFF; B current V3 raw redistribution HR=0; C physical trim; D larger IMAX to test whether benefit is simply integral capacity; E fixed manual trim to test adaptation benefit. No experiments now.
Falsifiable distinction: hold b numerically constant while varying g in bench/SITL. C's trim torque stays K_A*b; B's S torque changes g*S. Increase pitch context with identical burden: C blocks growth by directional reserve, B has no such input. Measure residual I, same/opposite reserves, exits and gain transitions, not only tracking scores.

## 26. Scientific claims

Permitted: formal normalized working-point semantics, conditional physical transfer conservation and tested bounded adaptation. Not permitted: lower RMS/path/tracking guaranteed, calibrated aerodynamic truth, removal of maneuver bias, flight safety from morning replay. Design is mathematically specified subject to stated preconditions; numeric deployment settings and full-loop stability remain review gates. STOP at design/RED.

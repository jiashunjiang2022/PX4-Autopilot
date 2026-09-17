# Adaptive Tail-Trim V1 mathematical contract

Pure-core stage; no integrated controller. All coordinates PRE_REVERSAL. Physical means normalized command geometry, not measured Nm. Hard conditions apply within specified domains. No flight parameters are frozen by this document. Core-specific rules C19–C26 override integration assumptions where stated.

## Variables

| Symbol | Meaning / units |
|---|---|
| I | native Roll integral, raw controller coordinate, pre-natural-update snapshot |
| b_R | persistent trim, normalized differential tail |
| B_hat | causal estimate of closed-loop persistent burden, tail |
| g | frozen same-cycle roll gain-compression times airspeed scale squared |
| K_A | verified effectiveness conversion .55-(-.55)=1.1, torque-coordinate/tail |
| tau_PID | g*(P+D+FF+I) plus existing scheduled baseline Roll trim |
| tau_trim | K_A*b_R |
| tau_total | tau_PID+tau_trim, requested torque before output clipping |
| r_PID | tau_PID/K_A, includes scheduled baseline trim |
| tail_roll | b_R+r_PID before allocation; achieved value separately labelled |
| tail_pitch | complete pitch command/2 under verified matrix, including baseline trim |
| c_L,c_R | normalized left/right commands before PWM reversal |
| R_pos,R_neg | positive/negative residual Roll capability at given b,pitch |
| B_MAX | tail-state cap, nominal .08–.10; proposed config ceiling .10 |
| B_RSVD | burden retained for native residual I, tail |
| R_RESERVE | strictly positive reserved dynamic tail capability |
| p_env | causal envelope of requested/achieved pitch magnitudes |
| B_SLEW,B_SAFE | positive normal/safety tail-units/s rates, values TBD |
| g_min | strictly positive valid inversion threshold, replay/SITL TBD |
| h | valid execution dt, 0<h_min<=h<=h_max for convergence claims |
| Δ | accepted transaction change unless explicitly labelled requested |

## Contracts C1–C18

| ID / class | Contract and enforcement boundary |
|---|---|
| C1 HARD | finite abs(b_R)<=B_MAX. Invalid cap/state clears state safely; no relabelling a raw cap. |
| C2 HARD | tau_trim=K_A*b_R. Identical b has identical trim contribution independent of g; achieved surfaces may differ under saturation. |
| C3 HARD | requested c_L=tail_pitch-(b_R+r_PID), c_R=tail_pitch+(b_R+r_PID), verified geometry only. |
| C4 HARD | achieved abs(c_L),abs(c_R)<=1 via allocator limits. Requested feasibility iff abs(pitch)+abs(b+r_PID)<=1; never certify arbitrary requested inputs as feasible. |
| C5 HARD | normal feasible accepted transfer abs(g*ΔI+K_A*Δb)<=2e-6; frozen pre-update g>g_min, finite operands. |
| C6 HARD | conservation uses accepted deltas. Feasible intersection and atomic commit, not clipped I plus requested b. |
| C7 HARD | valid normal step abs(Δb)<=B_SLEW*h; safety decay <=B_SAFE*h. Reset/corruption removal explicitly exempt, logged. |
| C8 HARD | without complete growth gate, abs(b) cannot grow. Initial acquisition is growth too. |
| C9 HARD | normal magnitude release does not require persistence gate; it MUST NOT bypass maneuver freeze C21. |
| C10 HARD | opposite target first unwinds to accepted zero; no one-cycle sign crossing. |
| C11 HARD | clear ALL history at zero; zero-cycle gate false; next update first new sample. At h=.02/GWIN=3 offsets 0..149 blocked, 150 earliest gate. |
| C12 HARD | finite abs(I)<=FW_RR_IMAX, including handback. No total-I raw-S cap silently reused as physical cap. |
| C13 HARD admission / BEST_EFFORT recovery | growth requires p_env+abs(b)+R_RESERVE<=1 AND directional minima; abruptly tightened envelope may invalidate existing state, forcing release. Instantaneous actual limits remain C4 HARD. |
| C14 HARD | invalid/nonfinite g or g<=g_min: never invert g; no normal growth/transfer, finite safe output and recovery. |
| C15 HARD | I reset epoch and b/history epoch synchronized before output, no hidden trim; removal torque logged. |
| C16 HARD conditional | latched exit reaches zero in finite steps under valid h>=h_min and decay>=v_min>0, including native-I saturation. N<=ceil(abs(b_start)/(v_min*h_min))+1. Otherwise finite-output fallback, no wall-time claim. |
| C17 HARD | no post-mixer/servo/PWM addition or second reversal. |
| C18 HARD | allocator input includes complete trim + residual demand after documented total clipping; saturation/clip feedback inhibits native I and trim worsening saturation. |

DIAGNOSTIC quantities: physical_transfer_mismatch=g*ΔI+K_A*Δb; requested versus achieved tau_total; R_pos/R_neg; pre/post-natural I; clipping loss. Diagnostics must not be substituted for hard bounds.

## Accepted transaction algebra

1. Latch g before transfer and retain it through output composition; K_A valid and >0.
2. Compute Δb_req with target, slew, state cap, gate and shared authority.
3. Intersect native interval -IMAX-I <= ΔI <= IMAX-I with saturation direction and Δb=-g*ΔI/K_A, requested direction/magnitude, and trim admissible intervals.
4. Choose largest feasible progress toward request; empty normal interval => no normal transfer.
5. Validate finite accepted pair, tolerance and epoch, then commit both once. Rejected transaction mutates neither.
6. Compute output with post-transfer I BEFORE native natural integration. Natural integration belongs to next output; logging labels both.
7. Update gain-compression state for next cycle only.

For g=.7, Δb=.01 would need ΔI=-.0157142857. If only -.005 is feasible, accepted Δb=.00318181818. Requested .01 is NOT committed.
A g change next cycle may change residual PID contribution and B_obs, but never the meaning of stored b. No universal whole-output gain invariance is claimed.

## Reserve and target equations

B_obs=g*I_pre/K_A+b; eligible LP step B_hat+=h/(tau+h)*(B_obs-B_hat).
Initialization uses first valid eligible sample; history is causal, invalid/maneuver gaps clear growth evidence. During turns freeze B_hat/target, allow safety release.
b_target=sign(B_hat)*min(B_MAX,max(|B_hat|-B_RSVD,0)).
Fixed physical reserve .03 is an offline candidate, sensitivity .02/.04; alternative g*.05/K_A is documented but not recommended for V1.
p_env=max(|p_current|,max(0,p_env_previous-d_p*h)), with p_current conservatively covering full requested and achieved pitch.
R_pos=1-|pitch|-b; R_neg=1-|pitch|+b. Same-direction reserve is R_pos for b>0 and R_neg for b<0. Initially require both >=R_RESERVE; asymmetry in numeric minima requires new evidence.
Envelope cannot foresee pitch or r_PID; allocator hard limits handle future transients. Negative reserve is an observable violation, never masked.

## Exits, infeasibility and reset

Normal disable: largest feasible exact pair; remaining trim decays toward zero. Safety triggers latch until zero. Saturated native I cannot stall authority removal.
For decay, choose Δb toward zero at bounded rate; compensate with feasible ΔI. Remaining mismatch m=g*ΔI+K_A*Δb is BEST_EFFORT continuity and must be logged. With compensating sign and no overcompensation, |m|<=K_A*|Δb|.
Rate-disabled/config-unknown/nonfinite/reset may require immediate removal; finite output and bounds outrank slew and continuity. On reset clear I,b, target, estimator validity and all evidence before composition. Do not label emergency torque removal bumpless.
Current V3 recovery requires caller handshake; a helper-only retained S before that handshake is not a hidden-output defect.

## Numerical and validation boundary

Predeclared float32 transaction abs tolerance 2e-6 normalized torque; state comparisons 2e-7; future recursive replay abs tolerance 2e-5. Compare direct transaction delta accounting, not subtraction of unrelated clipped outputs. Near-zero inversion must be gated before arithmetic.
g_min, h bounds, slew, envelope decay, maneuver thresholds, shared reserve and timeouts require preregistered replay/SITL choices. Mathematical closure is conditional and does not constitute numerical tuning completion, stability proof or deployment approval.

## Pure-core extension C19–C26

| ID / class | Contract |
|---|---|
| C19 HARD | g_cycle is an input value snapshot, copied once before transaction; g_used records it. Same snapshot for requested conversion, accepted pair and mismatch; future caller uses it for residual scaling. |
| C20 HARD | Shared/slew/I limiting precedes commit. g_cycle*accepted_delta_i+K_A*accepted_delta_b is within 2e-6 normal tolerance, not requested deltas. |
| C21 HARD | maneuver_active OR !learning_allowed freezes b: no growth, ordinary release or target chasing. Estimator/evidence freezing and fresh GWIN are caller obligations. |
| C22 HARD bounds / BEST_EFFORT continuity | safety_release_required or violated shared reserve overrides freeze. Prior trusted unwind also overrides freeze, only toward zero. Invalid numerical inputs reject; reset clears state explicitly. No I-bound violation to fake continuity. |
| C23 HARD | R_pos=1-abs(pitch_context)-b; R_neg=1-abs(pitch_context)+b; R_sym=min. Growth respects independently configured positive/negative minima. feasible is never true for a violated interval. |
| C24 HARD | finite positive config g_safe_min; g_cycle<=g_safe_min or nonfinite => invalid transaction, zero accepted deltas, finite held b and recovery indication; no inverse division. Deployment threshold TO_BE_SELECTED_BY_REPLAY_SITL. |
| C25 HARD | no new reversal latch accepted during maneuver/untrusted context. Preexisting unwind reaches zero without crossing and reports reversal_reached_zero; stale gate cannot immediately restart growth. |
| C26 DIAGNOSTIC, integration only | report unconstrained total demand and controller-clipped demand; decide controller-side clipping feedback before flight integration. Core does not implement it. |

Config is explicit, with no usable deployment defaults: b_max in [0,.10], positive b_slew, g_safe_min>0, positive directional reserves <=1 and K_A=1.1 for this geometry. Invalid config refuses transactions. Constructor state must be finite and within cap; invalid initial state becomes zero and requires reset. Config is immutable for the life of a core instance.
Inputs include current native I and its finite ordered bounds, target, gate/context, validity and reset epoch. C12 means I+accepted_delta_i stays within supplied bounds. These must be the actual native limits at eventual integration.
Core reset/epoch removal is marked reset_applied, with b_before/b_after; accepted transfer deltas remain zero because no I handback took place. It is exempt from C5/C7, never reported as normal conservation. Caller removes any actuator-visible contribution before output; this core has no output path.
Numerical arithmetic uses bounded double intermediates and float results. Predeclared test tolerances: 2e-7 state/reserve; 2e-6 physical mismatch. Tests record maximum measured mismatch. Feasible uses actual float output state with no optimistic tolerance enlargement.

# Bounded V2C DELTA4 control implementation

Branch fast-v2c-low-authority-control-20260923; baseline 19a108f17666942d199fb3856be5e85bc9aa6f83.
Uncommitted review implementation. No training, frozen coefficient changes or flash.

## Equation and ordering
At a completed predictor frame retain (V2C DELTA4, exact I_post+S argument, frame time, seq, independent validity).
Run computes control before completing the next frame: new prediction first eligible next Run.
The live guard is I_post+S from the previous completed diagnostic publication site.
No second model scheduler or ring. Model scheduling remains fixed-phase 50000 us.

raw=K*prediction; sat=clamp(raw,-MAX,MAX).
gate(T)=1 for |T|<=TON; (TOFF-|T|)/(TOFF-TON) inside; 0 at/above TOFF.
g=min(gate(T_model),gate(T_live)); gated=g*sat.
lo=max(-.20-T_model,-.20-T_live); hi=min(.20-T_model,.20-T_live).
target=clamp(gated,lo,hi).
Normal actual moves toward target by at most SLEW*actual_decision_dt.
Final actual is projected back to both [lo,hi] and [-g*MAX,g*MAX].
Safety contraction takes precedence over normal slew.
The .20 projection is redundant inside this narrow domain but explicitly present.
Outside domain FAST=0; this does not promise to correct an already unsafe baseline burden.

u_aug_raw=u_base_raw+saved_pre_update_gain*airspeed_scale_squared*F.
Final roll=clamp(u_aug_raw+existing_trim,-1,1).
Compression observes unchanged baseline control_u; yaw feedforward uses baseline clipped roll.
Augmented roll is installed after yaw feedforward. VTOL and transition are excluded.
When F=0 the baseline output assignment is untouched (no addition of zero).

## Validity and reset
Independent V2C validity checks its finite features and result after predictor history warmup;
V1 validity is not the control guard. Feature-history reset clears independent validity.
Reset/disarm/landed/disable/invalid configuration/control regime/failsafe: immediate zero.
Quaternion reset counter change and Mission-to-Stabilized abort invalidate FAST only.
Existing full/roll integral-reset helpers invalidate FAST as well.
No native I/Slow/compression reset is added by FAST.
A control reset requires nine NEW valid completed model frames, conservatively flushing any
pre-reset model inputs without changing the frozen predictor. On startup this is in addition
to the model's nine-frame feature warmup. Disabled/disarmed cycles repeatedly invalidate readiness.
Stale model/live data (>100000 us), time reversal, invalid/nonfinite dt, dt>0.1 fail zero and rewarm.
Actual hrt decision interval is used, not the PID's clipped dt.

## Parameters
FLAP_FAST_EN=0, FLAP_FAST_K=1, FLAP_FAST_MAX=.005,
FLAP_FAST_TON=.15, FLAP_FAST_TOFF=.18, FLAP_FAST_SLEW=.05.
Values outside conservative maxima or TON>=TOFF fail zero.
Names comply with 16-character parser limit (src/lib/parameters/px4params/srcparser.py:370).
No pre-existing parameter collisions were found.

## Diagnostics and limits
flap_fast_control is requested at interval zero in default and high-rate logger profiles.
Each publication describes the decision made before the contemporaneous model completion.
decision_timestamp, model_timestamp, prediction_age_us and decision_dt expose true control age;
timestamp is the later rate-status publication timestamp.
Existing flap_fast_shadow still reports the newly completed/held predictor, so seq may differ.
Diagnostic fields contain intermediates, saved scale, baseline and augmented raw roll.
Inactive paths may have zero decision time; state guard distinguishes them.
Parameter changes are recorded through normal ULog parameters.
Interval zero requests all publications but does not prove lossless recording: verify dropout,
topic rate and bandwidth on bench. No runtime latency/ULog measurements exist yet.
No actuator-margin projection is claimed.

READY_FOR_FAST_ON_FLIGHT=NO

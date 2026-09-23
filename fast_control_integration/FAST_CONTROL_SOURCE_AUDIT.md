# FAST control source audit

## Continuation resolution (2026-09-23)

The historical STOP below was resolved by explicit user authorization of previous-completed-frame
control, baseline-only compression observation and roll-only injection after yaw feedforward.
Current implementation references: FixedwingRateControl.cpp:487 saves pre-update scale;
507 observes baseline compression; 521–526 uses actual decision interval and previous cached frame;
594 installs augmented roll after baseline yaw; 688–692 completes the frozen model and caches
its exact I_post+S argument for the NEXT Run. FastV1ShadowModel.hpp:63–65 stores T and
independent finite DELTA4 validity; no coefficient, feature or scheduler changes.
The source-isolation test checks exact baseline arithmetic against the base git object.
Hardware latency remains unmeasured; decision/model timestamps and age are logged explicitly.
Actuator margin remains unproven and unimplemented; see FAST_ACTUATOR_MARGIN_AUDIT.md.
The following section retains the baseline audit and original stopping rationale as history.

Baseline: `19a108f17666942d199fb3856be5e85bc9aa6f83`.
Branch/worktree already existed clean: `fast-v2c-low-authority-control-20260923`,
`/Users/jiangjiashun/PX4/PX4-Autopilot-v6-fast-control-20260923`.
Evidence below comes from git objects at the baseline, not working-tree assumptions.

## Verified algebra and unresolved timing

`src/lib/rate_control/rate_control.cpp:183` computes P+I-D+FF using the
integral before this invocation's natural update. Lines 193–198 update the
integral and then return the previously computed torque. Therefore define
I_used separately from I_post.

`src/modules/fw_rate_control/FixedwingRateControl.cpp:415` applies Slow transfer
before RateControl update at 466. Lines 478–486 add S (when above FLT_EPSILON)
and scale by compression gain times airspeed-scaling squared. Lines 501–502
clamp control_u+trim to [-1,1]. The verified non-VTOL raw equation is
u_R=clamp(g*(P+D+FF+I_used+S_effective)+trim_R,-1,1), where D includes its sign
and S_effective is zero in the existing epsilon branch.

Adding F before scaling has the requested instantaneous algebra. However the
model T is I_post+S at line 584, and model update is not until 635–636, after
control_u, gain-compression update, and output clipping. Model arithmetic is
`FastV1ShadowModel.hpp:59`; independent V2C validity is absent at line 60.
The model Output does not retain current T, only lagged T. It is not justified
to substitute I_used+S or a later live T for the frozen model-frame T gate.

## Feedback that a naive injection would introduce

`FixedwingRateControl.cpp:492` passes control_u into gain compression.
`src/lib/rate_control/gain_compression.cpp:61–81,97–112` shows that when
FW_GC_EN is enabled this changes persistent filter and compression-gain state.
The gain is subsequently supplied to Slow via transfer_inputs.g_current
(`FixedwingRateControl.cpp:383–415`). A pre-scaling FAST addition therefore
does not merely disappear from all controller states when disabled: it can
leave compression-state effects. No claim is made that FW_GC_EN is enabled
on the actual vehicle; no parameters were read or changed.

Roll output also feeds yaw at lines 537–540 when FW_RLL_TO_YAW_FF is nonzero.
This is existing control behavior, but it must be explicit in a FAST decision.

## Reset paths

Full and roll reset helpers at 111–123 reset native/Slow state only.
Call sites: reset_integral at 329–330; landed/non-FW at 334–337;
Bumpless reset_required at 417–418; nonfinite control_u at 504–506;
manual/rates-disabled at 527–530. FAST history is separately cleared only
on regime transitions at 300–305, before land polling at 313/316.
Disarm is passed as Slow hard_reset at 396 but is not an independent FAST
history boundary. The new path would need its own immediate invalidation
and fresh-frame requirement without altering those control resets.

## Shared tail margin

The configured matrix is checked by verify_flap_slow_configuration at 95–108.
Under the ideal two-surface inverse, c_L=tau_P/2-tau_R/1.1 and
c_R=tau_P/2+tau_R/1.1, so normalized unit-range feasibility is
abs(tau_P)/2+abs(tau_R)/1.1<=1. This is an ideal requested-torque margin,
not proof of actual instantaneous allocator/servo margin: allocator scaling,
trim, limits and allocation method must also be established. Do not use a
delayed actuator_servos sample as that proof.
ACTUATOR_MARGIN_IMPLEMENTED=NO

## Stop decision

SOURCE_SEMANTICS_VERIFIED=NO (full assumed timing/state-independence contract;
instantaneous raw algebra alone is verified).
The task explicitly requires stopping the control patch on semantic mismatch.
This audit stops before implementation rather than silently deciding whether
FAST should enter compression feedback or which I/T instant defines the bound.
An implementation contract must explicitly distinguish model-frame I_post+S
from output I_used+S, choose previous-frame hold versus same-cycle late
injection, and specify whether compression observes baseline or augmented
control_u. Preserving baseline compression observation while adding g*F later
is a possible design, not an implementation performed here.

No production edits, build, tests, parameter changes, model changes, commit,
push or flash were performed. No new firmware/hash exists. Requested defaults
remain proposals only: EN=0, K=1, MAX=.005, TON=.15, TOFF=.18, SLEW=.05.
No enabled bench procedure is authorized by this incomplete implementation.
READY_FOR_DISARMED_BENCH=NO
READY_FOR_FAST_ON_FLIGHT=NO

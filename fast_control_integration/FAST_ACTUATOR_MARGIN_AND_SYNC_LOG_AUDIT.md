# FAST shared-tail margin and synchronous diagnostics

Parent: 56d1de129c2e74e994b70f59f014c17b879eb596
Branch: fast-v2c-actuator-margin-logging-20260924
Worktree: /Users/jiangjiashun/PX4/PX4-Autopilot-v6-fast-control-20260923

## Starting point and original user file
Target branch already existed at required parent. Before editing, the only dirty item was
NuttX untracked tools/jlink-nuttx. git ls-files --error-unmatch failed as expected.
Original absolute path:
 /Users/jiangjiashun/PX4/PX4-Autopilot-v6-fast-control-20260923/platforms/nuttx/NuttX/nuttx/tools/jlink-nuttx
Type: regular executable file; size: 50712 bytes.
SHA256: 095c44b5cdcda31e6a67a7aa21983d4c1bc78886d07094275a6624169396828b
Moved to /tmp/px4-fast-jlink-backup-20260924.LZHb0d/jlink-nuttx.
Both top-level and nested git status --short then empty. No other user content moved.
Build regenerates that path; preserve regenerated copy outside repository before restoring
the original after push. Restoration result is reported in final response. No binary staged.

## Proven configuration and scope
Slow verify_flap_slow_configuration is byte-for-byte unchanged.
FAST calls it PLUS independent exact nominal matrix/zero-tail-trim checks and:
- surface 0/1 FLAP and SPOIL coefficients exactly zero, finite;
- surface 0/1 servo slew exactly zero;
- zero rotors, or one fixed-forward rotor with exactly zero Y/Z lever arms;
- if rotor present, finite X position and CT.
Exact zero avoids introducing any unbudgeted auxiliary displacement. This is stricter than
the permitted epsilon and may reject a near-nominal configuration. No parameters changed.
Multiple rotors/offset rotor configurations are conservatively unsupported, not reallocated.

Source path audited:
- FixedWing getEffectivenessMatrix adds rotors and surfaces; rotor propeller torque disabled.
- ActuatorEffectivenessRotors moment=ct*position.cross(axis)-ct*km*axis.
  Fixed-forward axis and zero Y/Z remove torque coupling to surfaces.
- Base ActuatorEffectiveness getDesiredAllocationMethod returns pseudo-inverse,
  getNormalizeRPY returns false. CA_METHOD 0 or AUTO=2 therefore uses non-normalized
  RPY pseudo-inverse. ControlAllocator applies the selected policy.
- ControlAllocationPseudoInverse allocate computes trim+mix*(request-control_trim).
  Fixed-wing configuration has zero linearization point; shared trims checked exactly zero.
  Nominal three-surface torque matrix has independent yaw. Yaw surface auxiliary coefficients
  do not couple back through pseudo-inverse and are deliberately not guarded.
- ControlAllocator allocates, applies flaps/spoilers, updateSetpoint, optional actuator slew,
  then clipping. Shared flap/spoiler coefficients are zero; shared slew disabled.
  FixedWing updateSetpoint only stops motors. Servo min/max are -1/+1.
- FunctionServos passes normalized actuator_servos.control directly. Mixer output-limit
  reversal flips sign only (magnitude invariant); center/min/max conversion maps normalized
  request to PWM, not an added normalized control term. No reversal is applied twice.

This is a model-based pre-allocation actuator-feasibility constraint based on the verified
control-effectiveness configuration, NOT measured physical servo-position feedback.
Proof assumes allocator/controller have loaded the same stable parameters; asynchronous
in-flight allocation-parameter changes are not authorized or synchronized by this patch.
No claim about physical calibration, actuator dynamics or instantaneous measured positions.

## Equations and baseline policy
r=control_u(0)+trim(0), p=control_u(1)+trim(1), from the current Run BEFORE FAST.
c0=p/2-r/1.1; c1=p/2+r/1.1.
Holding p fixed gives L=-min(1,1.1*(1-|p|/2)), U=-L.
If r/p nonfinite, scale nonfinite/nonpositive: invalid/blocked.
If |p|>1, |r| outside [L,U], or either baseline surface outside [-1,1]:
blocked, FAST bounds=[0,0]; never repair or reshape baseline.
For feasible baseline: headroom_negative=r-L; headroom_positive=U-r.
FAST bounds=[(L-r)/scale,(U-r)/scale], contain zero.
No hidden reserve; strict baseline comparisons, 2e-6 epsilon only for postcheck arithmetic.

Order: K -> MAX -> min(model/live gate) -> T_SAFE -> actuator bounds -> slew ->
gate/MAX and T_SAFE contraction -> actuator contraction.
after_t_bound keeps its old meaning; after_actuator_bound is new.
Internal _actual is contracted immediately, even with very small normal slew.
A temporary [0,0] constraint zeros actual without destroying model readiness.
Unsupported configuration is a state failure and retains existing invalidate/rewarm behavior.

Nonzero output is reconstructed as r+scale*final and checked for finite roll/pitch/surfaces
and normalized bounds within EPS. Material final torque-clamp change (>EPS) also fails.
Unexpected failure skips roll replacement, invalidates FAST and logs actuator_postcheck_failed.
Tiny (<EPS) floating-point clipping is the only permitted redundant torque-net adjustment.
Delivered delta is explicitly logged; it can differ from scale*final by floating roundoff.
Zero final skips replacement exactly. Already infeasible baseline is not subjected to an
attempted repair or treated as a new FAST postcheck failure.

Compression observes unchanged baseline; yaw feedforward observes baseline roll;
replacement remains after yaw. FAST never writes I/S, changes predictor or moves its scheduler.

## New diagnostics (all in flap_fast_control)
- controller_terms_valid: RateControl ran and term fields were captured; otherwise zeros unavailable.
- p_sp_used, p_used: actual controller-frame roll-rate arguments, rad/s.
- p_error_used=p_sp_used-p_used.
- roll_p_term, roll_d_term, roll_ff_term: same RateCtrlTerms output; D retains negative sign,
  before gain/airspeed/trim scaling.
- native_roll_i_current, slow_s_current, total_t_current: publication-time I_post, transferred
  raw Slow state and sum. live_t_guard remains previous-completed state used by decision.
- baseline_roll_output, baseline_pitch_output: actual baseline clipped controller-frame torque.
  Feasible active baseline equals the unconstrained margin input; infeasible baseline may be clipped.
- augmented_roll_output: actual controller-frame roll after FAST replacement.
- applied_delta_roll=augmented_roll_output-baseline_roll_output: delivered request contribution.
- actuator_config_valid: independent geometry/auxiliary configuration eligibility.
- actuator_margin_valid: helper has finite valid inputs under supported config.
- actuator_margin_blocked: baseline infeasible or helper unavailable.
- actuator_margin_limited: target or prior actual lay outside dynamic bounds.
- actuator_postcheck_failed: nonzero proposed addition rejected by redundant postcheck.
- after_actuator_bound: FAST units, distinct from after_t_bound.
- actuator_roll_lower/upper: shared-tail-derived roll torque interval.
- actuator_roll_headroom_negative/positive: nonnegative magnitudes in torque units.
- actuator_fast_lower/upper: interval in burden-equivalent FAST units.
- tail0_baseline/tail1_baseline: analytic surface from unconstrained baseline.
- tail0_augmented/tail1_augmented: analytic surface from delivered roll and baseline pitch;
  meaningful only with actuator_margin_valid. For valid feasible cycles these obey shared equations.
- mission_seq_current: cached latest valid mission_result.seq_current, -1 before available or
  after invalid result. This asynchronous label has NO eligibility/prediction/output effect.
Existing raw fields retained. Optional previous allocator feedback omitted to avoid additional
mixed-time fields; it is not used in the limiter.

## Logging/build compatibility
Both existing logger profiles still request flap_fast_control at interval zero.
Generated uORB payload 216 bytes (no trailing padding), ULog data record about 221 bytes.
Expanded format including topic name and NUL: 1394 bytes < messages.h format[1800].
No generated header edits. At 250 Hz, topic alone is about 55.25 kB/s; interval zero is a
request, not proof of lossless delivery. Hardware logging completeness remains to be tested.
New terms are unavailable outside active controller updates, and output coordinates are
controller frame before tailsitter rotation; FAST itself excludes VTOL.

## Verification evidence
All final commands exit 0:
 c++ -std=c++17 -Wall -Wextra -Werror -fsanitize=address,undefined fast_control_integration/fast_control_test.cpp -o /tmp/fast_control_test
 /tmp/fast_control_test
 python3 fast_control_integration/source_isolation_test.py
 git diff --check
 cmake --build build/px4_fmu-v6c_default --target clean
 make px4_fmu-v6c_default
 git diff --exit-code 56d1de129c2e74e994b70f59f014c17b879eb596 -- src/modules/fw_rate_control/FastV1ShadowModel.hpp src/modules/fw_rate_control/FastV2CandidateModels.hpp

Host tests: configuration/geometry/headroom, float boundary, nonfinite, scale validity,
150 full-intersection cases, positive/negative dynamic contraction and readiness retention,
zero-path baseline bit equality; previous 1080 envelope and 77 boundary cases remain passing.
The bit-equality host check exercises the zero-replacement idiom; source checks link this to
production ordering. No full uORB Run or hardware experiment is claimed.
Source checks compare Slow guard text exactly, preserve model/PID, constants/default metadata,
Mission/stale/warmup, state ordering, synchronous identities and mission-label isolation.
Initial build failed on misnamed CA_R0_PY wrapper; corrected to actual CA_ROTOR0_PY/PZ.
Final clean rebuild after all source corrections ran all 1188 steps, exit 0.
Logs: /tmp/fast-margin-final-clean.log and /tmp/fast-margin-final-build.log.
FLASH 1855716 B /1920 KB (94.39%); AXI SRAM 91212 B /512 KB (17.40%).
RWX LOAD linker warning remains. Runtime stack/latency not bench-measured.
Diff reviewed against parent; only FAST integration, diagnostic schema, tests/report changed.

Firmware: build/px4_fmu-v6c_default/px4_fmu-v6c_default.px4
SHA256: 67ad8ce8f32cdccbed59cd5668a8b054cb8e227efbab288fdfb494ab38372baf
Built before commit; embedded revision may carry dirty parent identity.

SYNC_CONTROLLER_LOGGING_IMPLEMENTED=YES
MISSION_SEQ_LOGGING_IMPLEMENTED=YES
SHARED_TAIL_ACTUATOR_MARGIN_IMPLEMENTED=YES
ACTUATOR_MARGIN_USES_PITCH_AND_ROLL=YES
ACTUATOR_MARGIN_IS_PREALLOCATION=YES
ACTUATOR_MARGIN_STATEFUL_FAST_CONSISTENT=YES
BASELINE_INFEASIBLE_FAST_FAIL_ZERO=YES
ACTUATOR_POSTCHECK_IMPLEMENTED=YES
GAIN_COMPRESSION_BASELINE_ONLY=YES
YAW_FF_BASELINE_ONLY=YES
FAST_WRITES_I_OR_S=NO
T_SAFE_STILL_0P20=YES
STALE_TIMEOUT_UNCHANGED=YES
NINE_FRAME_WARMUP_UNCHANGED=YES
AUTO_MISSION_ONLY_UNCHANGED=YES
FAST_DEFAULTS_CHANGED=NO
FAST_RESEARCH_ENVELOPE_CHANGED=NO
MODEL_COEFFICIENTS_CHANGED=NO
MODEL_FRAME_SEMANTICS_CHANGED=NO
HOST_TEST_PASS=YES
SOURCE_ISOLATION_TEST_PASS=YES
CLEAN_BUILD_PASS=YES
FAST_ACTUATOR_CONFIG_GUARD_IMPLEMENTED=YES
FAST_GUARD_REQUIRES_SHARED_TAIL_FLAP_ZERO=YES
FAST_GUARD_REQUIRES_SHARED_TAIL_SPOILER_ZERO=YES
POST_ALLOCATION_ADDITIVE_TERMS_AUDITED=YES
SLOW_CONFIG_BEHAVIOR_UNCHANGED=YES
ACTUATOR_ANALYTIC_MODEL_ASSUMPTIONS_PROVEN=YES
READY_FOR_FAST_ON_FLIGHT=NO

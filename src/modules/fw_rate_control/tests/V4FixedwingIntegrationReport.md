# V4 Fixedwing integration — 2026-09-20

Repository: `/Users/jiangjiashun/PX4/PX4-Autopilot-v4-fixed-b0-exp-20260913`

HEAD: `720090d70bec832e3b9d23bdeffa36852710b481`

Branch: `v4-fw-integration`. Parent: `e624a99f2955addbf76681e636c44162a0c03055`.
`v4-core-green-e624` points at the same starting HEAD. All integration changes
remain uncommitted. No checkout/reset/commit/push, flash, controller connection,
actuator operation, Fast work, or flight-data opening was performed.
The pre-existing untracked `analysis/` was neither opened nor changed.

## Result

READY_FOR_BENCH_AUDIT=YES

FLIGHT_READY=NO

Existing tests 103/103, V4 core 31/31, safety 9/9, integration 38/38 PASS.
Production FMUv6C build PASS, exit 0. Source audit A–N PASS.
MANEUVER_DISCRIMINATION=NOT_FULLY_PROVEN.
No claim of flight-validated tuning, physical torque units, maneuver rejection,
hardware timing, actual ULog capture, or airborne performance follows from this result.

## Production routing

`FixedwingRateControl.cpp:Run` calls `V4MemoryIntegration::before` after the
existing e624 allocator read/routing and before the native controller update.
The allocator routing block is unchanged and its existing five tests still pass.
The adapter owns the unchanged fixed-capacity V4 estimator and references the
actual native RateControl and existing V3 object. It allocates no memory per tick.
The core/history is a module member, not a large automatic local object.

```text
mode/current V4 setpoint/reset + cached config + allocator publication
  -> ownership selection (disarmed acquisition only)
  -> existing controller reset handling
  -> cycle-start I/B snapshot
  -> validate new config/pair BEFORE installing new Roll limit
  -> V4 core paired commit (I_post = I_pre - accepted_delta_B)
  -> B integral context, HR=0
  -> native RateControl::update
       output = P + I_post_transfer + D + FF
       then natural integration under native saturation + shifted HR=0 limits
  -> snapshot I_post_natural
  -> mutually exclusive raw B insertion OR old raw S insertion
  -> frozen pre-compression-update gain and airspeed scaling
  -> existing trim, clamp, torque publication and allocator
```

There is no independent V4 actuator publisher and no allocator/PWM change.
This stage DOES enable B in the existing torque path when explicitly enabled;
it is not an actuator-isolated Shadow implementation. Default enable is zero.
Native anti-windup remains in `RateControl::updateIntegral`.

`Run`, configuration installation, pair commit, natural integration and resets
execute serially on `nav_and_controllers`. Constructor parameter setup precedes
callback registration. No publication/callback occurs between the two writes in
`commitRollMemoryPair`. The gain snapshot is taken before the existing compression
update; B is added once before that gain, airspeed, trim and clamp.

## Ownership and parameters

`FW_V4_EN=0` by default. V4 can acquire ownership only while disarmed and in
non-VTOL fixed-wing configuration. Acquisition resets native Roll I and clears
V3 memory/history; there is no armed S-to-B migration. Armed enable is pending.
While V4 owns the pair, V3 update and V3 output composition are bypassed.
Disable retains ownership until paired B-to-I handback reaches exactly zero;
the following cycle returns ownership to V3, synchronizing its reset epoch.
A limit update deferred during V4 ownership is installed on return to V3.

V4 OFF, without a preceding active handback, retains the original e624/V3 path,
including its setpoint read timing and zero-S arithmetic ordering. The functional
test compiles the actual e624 class from `git show` into the build directory,
renames it, and compares 250 cycles of torque, I and nonzero S with current Run.
Only class/symbol naming and fixture friend access differ in that test baseline.

| Parameter | Prototype default | Meaning |
|---|---:|---|
| FW_V4_EN | 0 | Request V4 ownership; only disarmed acquisition |
| FW_V4_BMAX | .10 | Raw-I-equivalent memory cap |
| FW_V4_R | .05 | Residual deadband reserve |
| FW_V4_TAU | 5 | Evidence LPF seconds |
| FW_V4_KB | .10 | Residual learning gain, NOT flight validated |
| FW_V4_SLEW | .01 | Raw memory change per second |
| FW_V4_GWIN | 3 | Gate window seconds |
| FW_V4_GSTD | .025 | Raw window standard deviation bound |
| FW_V4_GSIGN | .90 | Same-sign fraction |

`FW_RR_IMAX` remains native absolute I/total limit. V4 has no HR parameter;
internal HR=0 and F=0. `FLAP_V4_ENABLE` still means the older specimen-prior
Shadow feature and is not reused. All new defaults are explicitly labelled
NOT FLIGHT VALIDATED in generated parameter metadata.

When V4 owns the pair, `parameters_update` updates P/I/D and pitch/yaw limits
but defers Roll IMAX to core configure. An illegal pair under new IMAX/BMAX
causes joint recovery before new limits reach output. Invalid configurations
retain the last valid limit and cause reasoned recovery. Configure-induced
recovery is not overwritten by an ordinary update in that cycle.

## Time, allocator and modes

Evidence source: `vehicle_angular_velocity.timestamp_sample`, not scheduler time
or clipped dt. It samples the cycle-start native I+B. The adapter offers at most
one sample per 50 ms of strictly advancing actual sensor timestamps. This is
nominal 20 Hz decimation, without interpolation; callback phase can reduce it
(e.g. 20 ms callbacks give 60 ms offers). Qualification uses real elapsed span,
not sample count. Duplicate/backward/future timestamps cannot authorize learning.
`control_dt` is the existing clipped controller dt and is only the exit-step
timing input. The core checks wall-time evidence age on backup ticks too;
actual gaps over 150 ms invalidate qualification and require full rewarm.

The cached surface allocator message uses its actual publication timestamp
(`ControlAllocator.cpp` sets it at publication). Non-VTOL uses instance zero.
Timestamp zero, future timestamp, nonfinite Roll unallocated torque or age above
100 ms means UNKNOWN, inhibiting both learning directions. No new publication
does not erase prior saturation flags. Exactly 100 ms is fresh. Opposite sign
saturation routing and native saturation behavior remain separate from the V4 gate.

Only armed, airborne, valid non-VTOL AUTO_MISSION can learn. Mission eligibility
does not grant learning until the existing core window/std/sign test succeeds.
There is no invented bank/rate threshold and no new maneuver detector. The
adapter's explicit maneuver/stable input is exercised in host tests; production
currently relies on the frozen core's statistical gate alone. This limitation
is why maneuver discrimination is NOT_FULLY_PROVEN, even though an explicit
ordinary-maneuver/gate invalidation holds B in tests.

Non-Mission, Mission-to-STAB/pilot abort, failsafe and disable use paired handback
when the pair is legal. Landed/disarm/rate-reset/control-invalid/epoch mismatch
reset B and evidence via recovery. While V4 owns the pair and is disarmed,
native natural integration is inhibited even if the land detector still says
airborne. V3 behavior in that situation is unchanged.

## Diagnostic semantics

New `flap_v4_memory_status` is published from Run and included in the default
optional logger list with no additional downsampling. Actual ULog throughput,
drop counts and hardware scheduler latency remain bench-audit obligations.
The functional tests subscribe to the real uORB diagnostic publication.

I_pre_transfer/B_pre/T_pre describe entry to adapter before configure/update,
after any preceding controller reset. I_post_transfer/B_raw/T_post_transfer
describe the core result before native update. I_post_natural/T_post_natural
describe native state after natural integration, while `rate_ctrl_terms.i_term`
is the output-time, post-transfer I. B_after_gain uses the current cycle's frozen
gain and airspeed coordinate, not Nm or servo angle.

If output validation resets state AFTER natural integration, `output_recovery`
marks that exception. The earlier I/B/T snapshots are preserved instead of being
overwritten with zero; `i_cycle_end_raw` and `b_cycle_end_raw` identify final reset
state, with final reset epoch/recovery reason and invalid apply/learn/window flags.
Window statistics and B_after_gain then remain historical pre-output-reset
snapshots, not newly qualified evidence or applied output. A subsequent cycle
reads the reset core, so old history cannot reappear.

`evidence_fresh` means a fresh decimated input offered this cycle, not a claim
that a recovery/exit consumed it. `evidence_age_ms=-1` means missing/future source
time. Recovery reason numerically follows the core's Reason enum and also carries
non-recovery hold/exit reasons; `recovery_active` distinguishes those cases.
Old V3 topics/fields are retained, and never relabel q_hat or residual as V3 t_hat.
While V4 owns the pair, old V3 S is zero and its result flags are inactive.

## Verification and failures resolved

All logs below are local `/tmp/` artifacts. No existing oracle was weakened.
Original 31-test source SHA256 remains:
`8de0cfd95a5ffd4fe232668e5cc92f4dcf923acc7315a1de4cd8f8b4118b2811`.

| Step | Exit | Result |
|---|---:|---|
| Baseline configure/build/tests | 0 | 103+31+9 PASS |
| Initial integration configure/build | 1 | CMake core target declared after module dependency; declaration moved earlier |
| Adapter build/test | 0 | Initial 30/30 PASS |
| Initial real Run harness | terminated | Missing test HRT initialization; test-only setup corrected |
| Real Run harness retries | 1 | Fixture parameter-update consumption, unseeded S and gain snapshot assumptions corrected |
| Real Run verification | 0 | Initial 5/5 PASS |
| Disarm/release boundary tests | 1 | Two new behavior REDs reproduced |
| Boundary closure | 0 | Disarmed reintegration inhibited; deferred limit installed on ownership return |
| First FMUv6C make | 2 | Existing core exact float comparisons rejected by ARM GCC `-Werror=float-equal` |
| Second FMUv6C make | 0 | Full firmware linked |
| Post-output reset diagnostic test | 1 | New test reproduced overwritten time snapshots |
| Diagnostic closure | 0 | Historical snapshots and cycle-end reset values separated |
| Final host build/tests | 0 | 181/181 across eight test executables, zero skips |
| Final FMUv6C make | 0 | ELF, BIN and PX4 generated |
| git diff --check | 0 | No whitespace errors |

Compiler compatibility is the only core source change: five exact-zero
comparisons now use `fabsf(x) <= 0` or `fabsf(x) > 0`. All authoritative values
are finite on these paths; no epsilon, coefficient, formula, gate or transfer
boundary changed. No warning suppression was added. Core header, native
RateControl, V3 helper, original core/safety/adapter and existing test files are
unchanged. The installed newer astyle rejects two old repository option names;
that attempted formatting made no changes, and no style-tool PASS is claimed.

Final logs: `v4-integration-final-build.log`, `v4-integration-final-tests.log`,
`v4-integration-v6c-final.log`. Boundary RED evidence:
`v4-integration-exit-tests-red.log`, `v4-integration-output-reset-red.log`.
Functional harness deliberately calls Run synchronously without registering
callbacks or starting work queues; expected 'work queue not running' messages
do not represent an onboard smoke test. HRT is initialized solely for safe
schedule/cancel of each direct invocation.

Reproduce host tests with `CONFIG=px4_sitl_test`, `V4_SLOW_MEMORY_CONTRACT_RED=ON`.
Build the eight CTest targets named in the final log; set
`B2B_ADAPTIVE_GOLDEN_DIR` to an existing temporary directory for the exporter.
Production command: `make px4_fmu-v6c_default` (not upload).

Final firmware: `build/px4_fmu-v6c_default/px4_fmu-v6c_default.px4`.
SHA256: `b1fac3b85bcb969bd8f94c9016ac70333c97f04bbcb6ba83a31d3e43f5d5bf7d`.
ELF SHA256: `8397c16dc8f4519d98e955f13df22939a5e4caf1ec518988b6aed6aa858a5ad4`.
FLASH: 1,855,588 bytes / 1,966,080 (94.38%). Static AXI SRAM: 91,212 bytes.
The ELF contains V4MemoryIntegration::before, core update and the new uORB symbol.
No onboard stack/CPU measurement is claimed. Linker reports its RWX LOAD segment
warning; no linker workaround was applied.
Build also generated untracked `platforms/nuttx/NuttX/nuttx/tools/jlink-nuttx`;
it is left in place and is not a production source modification.

## Source audit A–N

| ID | Result | Evidence |
|---|---|---|
| A OFF preserves V3 | PASS | Actual e624 Run comparison, 250 cycles including nonzero S; bypass gates |
| B S/B exclusive | PASS | select owns one mode; V3 update and S composition are in opposite branch |
| C B once | PASS | One Run composeRaw call before gain; real Run output assertion |
| D HR zero | PASS | Core literal-zero context and bounds; native total-limit integration test |
| E accepted, not requested | PASS | Core joint commit unchanged; paired delta diagnostic assertions |
| F rebase not evidence | PASS | No adapter rebase; core transfer leaves timestamps/count untouched |
| G ordinary maneuver no decay | PASS | Core hold path and explicit adapter maneuver test; classifier limitation above |
| H no same-cycle B cross-zero | PASS | Core zero barrier unchanged, original contracts/safety reversal PASS |
| I no one-sided limit clamp | PASS | Deferred IMAX, configure recovery, actual Run parameter shrink test |
| J stale not fresh | PASS | Real message timestamp cache, 100 ms/future/nonfinite boundary tests |
| K unclipped evidence gap | PASS | Sensor timestamps and wall clock separate from dt; 150 ms/no-publication tests |
| L log time phases | PASS | Real terms/uORB checks, native timing and post-output reset snapshot test |
| M pair before native | PASS | before call precedes update; output terms match post-transfer I |
| N recovery clears qualification | PASS | Core invalidates estimate/history; no adapter copy-back; reset/recovery tests |

SINGLE_THREAD_TRANSACTION_ORDER=PASS

DOUBLE_MEMORY_IMPOSSIBLE=YES

TEST_ORACLE_CONFLICT=NO

Bench audit remains a separate stage: verify firmware identity, disabled default,
runtime topic/ULog timing, stack and CPU margin, ownership transitions and aborts
without assuming any flight authority from this report.

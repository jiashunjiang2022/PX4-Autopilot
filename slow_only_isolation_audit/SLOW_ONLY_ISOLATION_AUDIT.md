# SLOW-only isolation audit — 2026-09-27

## Subsequent upload authorization — 2026-09-27

After receiving the blocked-verification result, the user explicitly requested
upload to GitHub. This commit records the experimental cleanup for that upload,
not acceptance of the full regression gate. The audit below preserves the
pre-commit build/verification snapshot: references to uncommitted changes and
withheld push describe that earlier state. No additional build or test pass is
claimed by this upload. The recorded firmware was built before this commit;
its embedded source identity is not the new commit SHA. Full-regression blockers
remain unresolved. READY_FOR_BENCH=NO and FLIGHT_READY=NO remain unchanged.

BASE_SHA=0e3c15559cf9ae2264502b99849dc2e2240141e5
NEW_BRANCH=slow-only-isolation-20260927
FAST_CONTROL_REMOVED=YES
FAST_SHADOW_REMOVED=YES
FFPLUS_REMOVED=YES
NATIVE_FW_RR_FF_PRESERVED=YES
NATIVE_FF_PATH_PRESERVED=YES
SLOW_MATH_SOURCE_PARITY=PASS
SLOW_COMPARATOR_SOURCE_PARITY=PASS
BASELINE_DISABLED_FAST_PARITY=PASS
ORIGINAL_SLOW_PARITY=PASS
PAIRED_LPF_PARITY=PASS
OLD_RUN_STACK_BYTES=1392
NEW_RUN_STACK_BYTES=880
RUN_STACK_REDUCTION_BYTES=512
TESTS_PASS=PARTIAL (135 relevant tests pass; full regression builds blocked)
FMUV6C_BUILD_PASS=YES
COMMIT_PUSH_AUTHORIZED_BY_ALL_CHECKS=NO
FLIGHT_READY=NO

## Isolation and current state

New worktree: `/Users/jiangjiashun/PX4/PX4-Autopilot-slow-only-20260927`.
`git fetch origin` succeeded. Original diagnostic branch local and remote HEAD
remain `5935ee8fe0e245bde7bdc0dc82b696a55df12060`. New worktree HEAD and the verified
merge-base are exactly BASE_SHA above. Changes remain **uncommitted**, and the new
branch has **not been pushed**, because not all mandatory regression checks passed.
No reset, stash, clean of user files, flash, parameter change or original-worktree
source edit was performed. Existing audit directories and submodule dirt in the
original worktree remain untouched. The new worktree's NuttX submodule reports an
untracked build-created `tools/jlink-nuttx`; it is excluded from intended changes.

## Production removal and preservation

Removed the five specified fw_rate_control FAST/model/FF+ headers, their class
members, FAST-only CA handles, subscriptions, eight parameters, FF+ gain override,
actuation override, shadow/control message types, logger entries and FAST-only tests.
Native gain calculation is `gain_ff / _airspeed_scaling` followed by
`setFeedForwardGain`; all four native FF parameter definitions are byte-preserved.
No stored aircraft parameters were changed; native FW_RR_FF=0.8 remains usable.

An exhaustive search also found the older `flap_aug_shadow::FastPredictor`, which
the board auto-starts on lp_default. Removed its inference, history, model headers,
FAST-only derivative filters and motor/battery/acceleration/wind subscriptions.
Retained shared SLOW feature indices, SLOW/V4 diagnostics and their algorithms.
The legacy mixed `flap_aug_shadow` topic remains for SLOW users: FAST prediction,
validity, computation-time and model-hash fields are zero/false, not live FAST data.
The zero-injection safety constants in mixed SLOW/V4 messages remain unchanged.
No FAST predictor/control symbols remain in the resulting ELF. Dedicated
`flap_fast_shadow` and `flap_fast_control` topics no longer exist.

`BumplessRollITransfer.hpp`, FixedB0Experiment, RateControl, gain compression,
allocator source, all SLOW parameter metadata, and the three required SLOW topics
are byte-identical to the base. Geometry verification, transfer inputs/context,
accepted-delta pairing, status assignments and SLOW reset synchronization are
checked literally against immutable git objects by `slow_only_source_isolation.py`.
The comparator regression suite itself is unchanged. The old source-isolation
script was updated only to replace now-invalid FAST-preservation expectations;
its SLOW math, reversal, metadata and output checks remain.

FAST_CONTROL_PRODUCTION_REFERENCES=0
FAST_FFPLUS_PRODUCTION_REFERENCES=0
RUNTIME_WATERMARK_INCLUDED=NO

SLOW observables remain in:
- rate_ctrl_status: native I/error/natural integration diagnostics, transferred S,
  T, requested/accepted I/S deltas, baseline/total output, clipping and reset fields.
- rate_ctrl_terms: P/I/D/native FF/raw output terms.
- flap_b2b_adaptive: gates/state, comparator request/latched mode, target and estimate.
No replacement high-rate topic was introduced. Historical FAST reports are archival;
see `fast_control_integration/ARCHIVAL_NOTICE.md`.

## Tests and numerical parity

Host configuration: `cmake -S . -B build/px4_sitl_test -G Ninja -DCONFIG=px4_sitl_test`.
Relevant targets built with normal repository warning and sanitizer settings.
Final CTest selection passed 8/8 executables (exit 0):

| Suite | Cases passed |
|---|---:|
| RateControl | 19 |
| FixedB0Experiment | 9 |
| BumplessRollITransfer | 72 |
| SlowLpfComparator | 16 |
| AllocatorSaturationFeedback | 5 |
| SlowIsolationParity | 3 |
| FlapAugShadow, retained SLOW assertions | 6 |
| V4Shadow | 5 |

Total 135. Golden-vector export was enabled with B2B_ADAPTIVE_GOLDEN_DIR pointing
to a new temporary directory; final B2b run has no skipped test. The initial run
without that variable had 71 passes and one skip, and is not the final result.

New parity tests generate actual immutable-base and current output/yaw/status
source fragments in the host build directory; deleted FAST headers are loaded from
BASE_SHA for reference testing only and never compiled into firmware. Each of
disabled SLOW, ORIGINAL_SLOW and PAIRED_LPF runs 5,000 cycles. Comparison is exact
for raw roll output, final roll request and the entire zero-initialized status
message, including I/S/T and transfer diagnostics. Enabled scenarios assert that
transfer actually occurs, and PAIRED_LPF asserts comparator activation. Inputs sweep
airspeed/pre-update compression gains, positive/negative errors, saturation flags,
eligibility, abort and reset. Native roll FF is 0.8; reference FAST/FF+ are disabled.

Limit: this executes real source fragments and real RateControl/SLOW helpers with
deterministic I/O adapters. Compression gains are supplied identically; its update
is stubbed and its production source parity is checked separately. This is not a
full scheduled module, hardware or allocator closed-loop experiment. Clock,
publication and eligibility/configuration adapters do not claim runtime validation.

### Full-regression blockers (not repaired or suppressed)

1. `cmake --build build/px4_sitl_test --target test_results -j 8`: exit 1.
   `src/systemcmds/tests/test_uart_send.c:83`, deprecated `sprintf`, promoted by
   `-Werror` on MacOSX27.0 SDK. Source is outside this diff and remains unchanged.
   Log: `/tmp/slow-only-all-tests.log`.
2. Building all 135 registered unit/functional targets separately: exit 1.
   `unit-AS5600Math` and `unit-WingPhaseMath` fail linking arm64 with undefined
   ASan symbols including `___asan_after_dynamic_init` from fuzztest libraries.
   Their sources and `test/CMakeLists.txt` remain unchanged. No linker/compiler
   setting was weakened. Log: `/tmp/slow-only-host-all-build.log`.

The blockers are outside the cleanup diff; a baseline full-suite reproduction was
not performed, so this report does not claim experimentally proved pre-existence.
No failed tests were removed to permit a commit. All mandatory checks are not yet
satisfied, so commit/push are withheld as explicitly requested.

## Stack evidence

`measure_stack.py` replays the production compilation database for only
FixedwingRateControl.cpp and WorkQueue.cpp with `-fstack-usage`, placing outputs
in a fresh temporary directory and preserving production flags (-Os for the rate
module, -O2 for WorkQueue, Cortex-M7 hard float, GCC 13.2.1). Source, runtime
instrumentation and stack configuration are not modified.

Base artifact: `/tmp/px4_stack_current_v2/build/px4_fmu-v6c_default/src/modules/fw_rate_control/CMakeFiles/modules__fw_rate_control.dir/FixedwingRateControl.cpp.su`
reports Run 1392 B; WorkQueue Run reports 24 B.
New artifacts: `/var/folders/6q/49ktlvlx7bqcjl_f1hx2l0rw0000gn/T/slow_only_stack_gxcwm_j_`.
New GCC .su lists the split-body `0()` at source line 234 as 880 B and Run entry as
8 B. Actual production-object disassembly identifies `Run() [clone .part.0]`:
36 B integer register save + 32 B FP save + 812 B local allocation = **880 B**.
Run entry pops its 8 B frame before tail-branching into the body; it is not 888 B
and must not be reported as merely 8 B. Reduction is **512 B**.
WorkQueue Run remains **24 B**. CONFIG_WQ_NAV_AND_CONTROLLERS_STACKSIZE remains
**2240 B** in generated px4_boardconfig.h. These are static function frames, not
whole-call-chain maxima or runtime stack headroom, and do not establish HardFault
resolution or flight safety.

## Final clean build and identity

`cmake --build build/px4_fmu-v6c_default --target clean` exit 0;
`make px4_fmu-v6c_default` exit 0 after all production edits.
Logs: `/tmp/slow-only-clean.log`, `/tmp/slow-only-clean-build.log`.
Final relevant tests: `/tmp/slow-only-final-tests.log`.
`git diff --check` exit 0. Source-isolation scripts exit 0.

HEAD: `0e3c15559cf9ae2264502b99849dc2e2240141e5`, **dirty isolation working tree**.
This is not an unmodified base artifact and has no new release commit identity.

Firmware: `build/px4_fmu-v6c_default/px4_fmu-v6c_default.px4`
SHA256: `986addd85146131eb9d4615200ba19a284f27b858bf5e87a57defd4d19bc1a9a`

ELF: `build/px4_fmu-v6c_default/px4_fmu-v6c_default.elf`
SHA256: `0f1e52c807b33e7d646d2325980aecd1680cd397cd779ca315767a688f977cbb`

Linker memory summary: FLASH 1,838,596 B / 1,920 KiB (93.52%);
AXI_SRAM 91,212 B / 512 KiB (17.40%). `size`: text 1,834,168,
data 4,428, bss 87,116. Linker emits an RWX LOAD-segment warning (build succeeds).
All eight removed parameter names are absent from generated parameters.json/xml
and raw firmware strings. ELF symbol search finds neither modern FAST nor legacy
FastPredictor classes and neither deleted FAST topic. A negative search's exit 1
means no match, not a failed build.

READY_FOR_BENCH=NO (strict full-regression gate not complete)
FLIGHT_READY=NO
NEXT_ACTION=Resolve full-regression environment/link blockers in an authorized,
separate scope, rerun all mandatory checks, then commit and push only this branch.
Do not flash or treat the static stack reduction as flight clearance.

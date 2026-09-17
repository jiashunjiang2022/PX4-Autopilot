# Adaptive Tail-Trim V1 — future implementation plan

**NOT EXECUTED.** Requires human design review and a separately authorized implementation task.
Base design and math contracts are in ../specs/2026-09-17-adaptive-tail-trim-v1-*.md.
Current branch deliberately contains RED, not a deployable new controller.

## TDD rules and unresolved prerequisites

For each task: write failing behavioral test, run and inspect RED, implement minimal change, run GREEN plus 72 V3/17 native regressions; review diff. Never solve RED by renaming S, relaxing tolerance or increasing cap. Keep V3 independently selectable for comparisons. Resolve numerical g_min, timing bounds, shared reserve, maneuver thresholds and envelope decay with preregistered replay/SITL evidence before flight. Default new feature disabled. No Early/Fast changes.

Commands below are future proposed commands/targets; files and scripts labelled proposed do not exist yet. They are not claimed as executed. U denotes the future host test workflow:
```sh
cmake -S . -B build/px4_sitl_test -G Ninja -DCONFIG=px4_sitl_test
ninja -C build/px4_sitl_test unit-AdaptiveTailTrim
build/px4_sitl_test/unit-AdaptiveTailTrim --gtest_filter='<task filter>'
```
Each task specifies its filter or a separate command; maintain existing targets too.

## Task 1 — physical coordinates and limits

Files (proposed): src/modules/fw_rate_control/AdaptiveTailTrim.hpp; tests/AdaptiveTailTrimTest.cpp; test-only CMake.
Interface: validated Geometry(K_A, normalization, channel identity), tail bounds and directional reserve helper.
Tests: g-independent fixed b, BMAX, both signs, requested vs achieved geometry; randomized pitch/b/residual domain and exact boundaries.
Command: U filter Coordinates.*.
Expected: RED01/03/05/16 become behavioral GREEN without relaxing geometry; invalid geometry refuses activation.

## Task 2 — physical bumpless transaction

Files: proposed physical helper; src/lib/rate_control/rate_control.hpp/.cpp only after explicit authorization.
Interface: preview/commit accepted native ΔI with epoch, frozen g and coupled Δb; no mutation on rejection.
Tests: g=.7/1.4, partial acceptance, IMAX, wrong sign, stale epoch, NaN/Inf/tiny g, mid-cycle gain update, torque mismatch <=2e-6.
Command: U filter PhysicalTransfer.*.
Expected: accepted conservation GREEN; no requested/accepted confusion, no unsafe inverse.

## Task 3 — trim state machine

Files: proposed AdaptiveTailTrim.hpp and tests.
Interface: disabled/evidence/transfer/hold/normal-exit/safety-exit/recovery, fixed-size state.
Tests: bounds, slew, no entry bypass, exit latching before reentry, both-sign zero-first reversal.
Command: U filter StateMachine.*.
Expected: C1/C7/C10/C15 with finite bounded memory; no servo publications.

## Task 4 — physical estimator and gate

Files: helper and tests; migrate V3 timing logic without changing V3 implementation.
Interface: physical B_obs snapshot, learning_allowed, validity/age, estimator/gate result.
Tests: direct physical LP vs raw-LP-times-g, startup, gain steps, no future samples, long Loiter freeze, PID remains active, fresh 150-cycle reversal gate.
Command: U filter EstimatorGate.*.
Expected: C8/C9/C11 GREEN; no assumption that old logged gate is new gate.

## Task 5 — shared-axis limiter

Files: helper and tests.
Interface: requested and achieved pitch context, causal envelope, R_pos/R_neg, growth admission and release reason.
Tests: pitch steps, stale pitch, negative/positive trim, same/opposite reserves, arbitrary residual Roll and geometry violations.
Command: U filter SharedAuthority.*.
Expected: no growth in infeasible context; safe release, separate instantaneous allocator hard limits.

## Task 6 — exit/recovery

Files: helper and tests; native transaction API as needed.
Interface: latched trigger, accepted compensation, unmatched torque, reset epoch synchronization.
Tests: eight documented triggers, both signs, saturated IMAX, zero dt, invalid g, finite-time bounds under valid dt, abrupt reset exemption.
Command: U filter ExitRecovery.*.
Expected: finite zero trim under valid decay conditions; no hidden output after reset and no indefinite saturated handback.

## Task 7 — FixedwingRateControl integration

Files: FixedwingRateControl.cpp/.hpp only in future authorized implementation.
Interface: one frozen g, transfer before PID output snapshot, K_A*b after g, total clipping, allocator feedback, consistent sample timestamps.
Tests: output composition, natural integration order, gain update order, clipping before/inside allocator, roll saturation suppresses worsening native I, pitch coupling, no second reversal, no direct servo write.
Command: U filter Integration.*; then existing unit-BumplessRollITransfer and unit-rate_control_test.
Expected: OFF baseline equivalence; C17/C18; independent legacy V3 path retained.

## Task 8 — parameters and logging

Files: fw_rate_control/module.yaml and relevant diagnostic msg only under later authorization; integration tests.
Interface: versioned units, finite/range validation, core fields in design; reuse timings, do not reinterpret existing raw saved settings.
Tests: disabled defaults, .10 hard ceiling proposal, invalid g_min/reserve, log sample epochs, all limiter/exit reasons, migration from V3 saved parameters.
Command: U filter ConfigurationLogging.*; normal PX4 metadata generation through later SITL build.
Expected: minimal fields, no duplicate derived logging, explicit raw vs tail identity.

## Task 9 — full unit GREEN closure

Files: all new tests, future adapters for current opt-in RED tests; keep original tests unchanged.
Interfaces: real physical controller replaces migration-only adapter.
Command: U filter '*'; run original two targets with golden directory; run converted physical contract binary.
Expected: all 16 behavior cases GREEN, API probes replaced with genuine field/value tests; old 89 regressions PASS. Review tests for vacuous fixtures; retain historical RED report.

## Task 10 — offline replay parity

Files (proposed): Tools/adaptive_tail_trim/replay.py, standalone harness, fixture manifests.
Interface: causal timestamped samples and float32 output/state vectors, source/hash identity, pre-natural I.
Tests: morning logs plus available distinct-wing cohorts, Loiter, switches, invalid data, severe pitch, gain steps, exits; no retraining.
Command (future script): python3 Tools/adaptive_tail_trim/replay.py --manifest <reviewed-manifest> --verify.
Expected: transaction abs <=2e-6, recursive state <=2e-5 predeclared; directional reserves and masks agree. New gate computed independently; old gate only comparison.

## Task 11 — SITL

Files (proposed): tests/adaptive_tail_trim_sitl/ scenarios and analysis.
Interfaces: actual torque publication, allocator saturation/status, native antiwindup.
Command: make px4_sitl_default; run documented available simulator and scenario runner.
Expected: no crashes/NaN, OFF baseline identity, both clipping feedback paths, bounded exits, 50-Hz diagnostic observability, measured worst latency. Do not replace dynamics testing with a helper-only smoke test.

## Task 12 — FMUv6C compile/resources

Files: build artifacts/reports only unless authorized fixes separately reviewed.
Interface: confirmed actual board target, work-queue stack/runtime CPU.
Command: make px4_fmu-v6c_default (first reconfirm board configuration and actual target); arm-none-eabi-size build/px4_fmu-v6c_default/px4 (verify tool/artifact path locally).
Expected: build PASS, flash/RAM delta, bounded stack, no growing vector/dynamic allocation in update. No upload/flash command.

## Task 13 — hardware bench

Files: procedure and signed results, no automatic flight.
Interface: exact firmware commit, params, verified geometry, pre-reversal servo signals and logs.
Commands after separate hardware authorization: ver all; fw_rate_control status; listener rate_ctrl_status; listener actuator_servos; listener control_allocator_status; top.
Expected: correct signs/channel identity, gain-step held-b behavior, pilot abort/disarm/reset paths, logging and resource results. Physical setup and flashing are separate explicit authorization gates.

## Task 14 — real-flight authorization review

Files: signed review packet linking all hashes/results and comparison protocol.
Interface: human go/no-go; no code implementation in this step.
Command: git status --short; git rev-parse HEAD; review packet against tests/build/bench hashes.
Expected: only explicit human authorization can progress. Predefine A OFF, B V3 HR=0, C physical trim; justify D larger IMAX and E fixed manual trim. No claims of better tracking before data.

STOP. This document schedules future work only; no task above was implemented.

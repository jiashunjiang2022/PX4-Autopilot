# FAST-V1 SOURCE-AWARE SHADOW DIAGNOSTICS IMPLEMENTATION

BASE_COMMIT=e624a99f2955addbf76681e636c44162a0c03055  
WORK_BRANCH=fast-v1-shadow-diag-e624

CONTROL_ALGORITHM_CHANGED=NO  
CONTROL_OUTPUT_PATH_CHANGED=NO  
SCHEDULER_CHANGED=NO  
LOGGER_INTERVAL_CHANGED=NO  
PARAMETERS_CHANGED=NO

PRE_IMAX_CUMULATIVE_DIAGNOSTIC=IMPLEMENTED  
ACCEPTED_CUMULATIVE_DIAGNOSTIC=IMPLEMENTED  
BOUND_REJECT_CUMULATIVE_DIAGNOSTIC=IMPLEMENTED  
INTEGRAL_UPDATE_COUNTER=IMPLEMENTED  
STATUS_PUBLICATION_SEQUENCE=IMPLEMENTED  
TRANSFER_CUMULATIVE_DIAGNOSTIC=DEFERRED

SOURCE_IDENTITY_A_TEST=PASS  
SOURCE_IDENTITY_B_TEST=PASS  
SOURCE_IDENTITY_C_TEST=NOT_APPLICABLE  
BUILD_RESULT=FAIL  
UNIT_TEST_RESULT=PASS_WITH_ONE_PREEXISTING_SKIP  
DIAGNOSTIC_FIRMWARE_READY_FOR_BENCH_CHECK=NO  
READY_FOR_FLIGHT=NO

## Implementation

Natural cumulative diagnostics are updated at the source-level roll integral update, immediately after finite pre-Imax and accepted increments are available. They are reset by the existing `resetRollIntegralDiagnostics()` path, so each reset epoch has ordinary cumulative meaning. The publication sequence is incremented exactly once at the existing `rate_ctrl_status` publication point and is not reset with the integral epoch.

The transfer cumulative diagnostic was deferred. `BumplessRollITransfer` has pair-preserving transfer-in/adaptive-hold paths but also normal-exit, safety and limited paths with unmatched deltas. Adding a counter at a publication site would double-count or misattribute transactions; adding a new state-machine hook would exceed this diagnostics-only minimal patch.

## Float precision

The cumulative fields remain float32 to preserve the uORB schema style. A 5–15 minute controller run at approximately 50 Hz is below one million updates. With typical normalized increments around 1e-4 to 1e-2, float32 unit resolution remains far below the diagnostic tolerance over this horizon. Consumers must segment by reset epoch and account for uint32 counter wrap.

## Mandatory non-interference result

See `CONTROL_PATH_NON_INTERFERENCE.md`. New state is only accumulated, reset, copied into `rate_ctrl_status`, or tested. It is never read on a control calculation RHS.

## Build limitation

The Pixhawk 6C build reached the logger compilation and failed because the expanded `RateCtrlStatus` format exceeds the existing logger message-format buffer. The correct follow-up is a separately reviewed message-size/schema design or a smaller diagnostic message; this task does not modify logger infrastructure to hide the failure.

## Bench and flight validation plan

After resolving the message-size design, build the v6C target and perform a stationary bench log. Verify the new fields, monotonic publication sequence, update-count deltas, reset epoch segmentation, and identities A/B. Confirm actuator commands are unchanged. A later diagnostic cohort can use two NEW-wing and two OLD-wing flights with Fast off and Slow V3 unchanged; this is diagnostic validation only. No flash or flight authorization is given here.

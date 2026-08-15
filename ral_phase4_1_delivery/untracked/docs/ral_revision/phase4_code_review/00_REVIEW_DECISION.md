# RA-L Phase 4 code-review decision

## Decision

**APPROVE_WITH_REQUIRED_FIXES**

This is a source-review decision, not a flight-release decision. The reviewed tree has no control-law change and no defect classified as `BLOCKER`, so compile/SITL work and a tightly controlled, non-actuating diagnostic bench are reasonable. The two `P0` findings must be corrected before a formal target flash or authoritative bench ULog. Flight is prohibited until all `P0` and `P1` items are closed with the stated evidence.

| Severity | Count |
|---|---:|
| BLOCKER | 0 |
| P0 | 2 |
| P1 | 7 |
| P2 | 1 |

## Top risks

1. `P0-01`: q is produced from fixed `differential_pressure` instance 0, is not bound to the selector's physical source, and does not reset on `device_id` changes.
2. `P0-02`: published `nominal_r_as`/`r_as_used` are not sample-aligned with the variance placed in the EKF airspeed buffer.
3. `P1-01`: anti-alias coefficients assume 83.3333 Hz although actual rate is only measured, not enforced or used.
4. `P1-05`: about 3 KiB persistent plus about 3 KiB temporary estimator storage and Goertzel work execute in EKF2 without target WCET/stack evidence.
5. `P1-06`/`P1-07`: helper tests and a static logger estimate do not establish integrated behavior or dropout-free target logging.

## Release gates

| Action | Current decision |
|---|---|
| Compile / host unit tests / SITL | Allowed |
| Flash as formal experiment firmware | No; close both P0 findings first |
| Connect a real sensor | Only on a non-actuating, single-Pitot diagnostic bench with the instance/device mapping independently checked; data is not authoritative |
| Begin formal bench ULog collection | No; close P0 and logging/source-identity prerequisites first |
| Flight | No; close all P0/P1 items and complete hardware/ULog verification |

FLASH is `1,828,392 B / 1,920 KiB = 93.00%`: **TOO_CLOSE_FOR_SAFE_ITERATION**. AXI SRAM is `61,892 B / 512 KiB = 11.80%`, but this does not include a demonstrated EKF2 stack high-water margin. No before-patch FLASH measurement is available, so patch growth cannot be attributed.

Review boundary: branch `air`, HEAD `650836c48937f654a0a81fa894b381ff5b130cc2`, 36 core files (25 modified, 11 new). No source was modified by this review.

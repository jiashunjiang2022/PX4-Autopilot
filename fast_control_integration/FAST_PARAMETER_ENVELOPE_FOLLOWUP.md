# Research-envelope follow-up

## Real-flight discovery and evidence boundary
User reports a real flight on e7b2f6de69 with FLAP_FAST_EN=1 and FLAP_FAST_K=1.5.
The old source deterministically rejects K>1 and invalidates FAST to zero. The source audit
establishes this mechanism; no ULog was supplied or independently examined in
this task. Thus the requested classification below combines the user-reported flight settings
with verified software semantics, not an independent forensic verification of that flight.

INVALID_CONFIGURATION_FAIL_ZERO_FLIGHT_VERIFIED=YES (user-reported flight; source mechanism verified)
Do NOT use that flight as K=1.5 FAST performance evidence.

## Verification
Commands, all exit 0:
- c++ -std=c++17 -Wall -Wextra -Werror -fsanitize=address,undefined fast_control_integration/fast_control_test.cpp -o /tmp/fast_control_test
- /tmp/fast_control_test
- python3 fast_control_integration/source_isolation_test.py
- git diff --check
- cmake --build build/px4_fmu-v6c_default --target clean
- make px4_fmu-v6c_default
- git diff --exit-code e7b2f6de69 -- src/modules/fw_rate_control/FastV1ShadowModel.hpp src/modules/fw_rate_control/FastV2CandidateModels.hpp

Tests cover each conservative default, requested intermediate, new ceiling, next float above,
negative and NaN/Inf config, zero/negative/sub-minimum slew, relational errors,
large-K saturation, 1080 expanded authority/gate combinations, near-bound contraction,
and prior 77 cases plus Mission exit, warmup, reset and stale checks.
Source tests preserve baseline arithmetic, compression/yaw ordering and Mission-only guards;
compare predictor source bytes against parent; compare runtime constants and metadata.
These are host/source checks, not a new flight or hardware performance result.

Clean build: 1188 steps; FLASH 1853100 B /1920 KB (94.25%);
AXI SRAM 91212 B /512 KB (17.40%). RWX LOAD-segment linker warning remains unsuppressed.
Logs in /tmp/fast-envelope-clean.log and /tmp/fast-envelope-build.log.
Generated parameters.json reviewed: EN default 0; all five defaults and bounds match source.

Firmware: build/px4_fmu-v6c_default/px4_fmu-v6c_default.px4
SHA256: d8ab0f2c6e3d5b063c5ee5f520908cdaf12b4b044d463aa2f4de58e3f450728a
Firmware was built from this patch before committing; embedded git identity may show dirty parent.
No flash or vehicle-parameter changes.

FAST_DEFAULTS_CHANGED=NO
PARAM_METADATA_MATCHES_RUNTIME=YES (cross-parameter TON<TOFF documented separately)
CONFIG_INVALID_DIAGNOSTIC=YES
T_SAFE_STILL_0P20=YES
STALE_TIMEOUT_UNCHANGED=YES
AUTO_MISSION_ONLY_UNCHANGED=YES
WARMUP_UNCHANGED=YES
MODEL_COEFFICIENTS_CHANGED=NO
MODEL_FRAME_SEMANTICS_CHANGED=NO
HOST_TEST_PASS=YES
SOURCE_ISOLATION_TEST_PASS=YES
BUILD_PASS=YES
ACTUATOR_MARGIN_IMPLEMENTED=NO
READY_FOR_FAST_ON_FLIGHT=NO

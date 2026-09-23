# Mission eligibility and zero-slew safety follow-up

Reviewed parent e50283e7bae525af28011edae79f203e89970626.
Two issues: state guard permitted non-mission modes after warmup; zero slew could
hold prior nonzero actual output. This patch changes only these safety semantics.

After vehicle-status update, any non-AUTO_MISSION state immediately invalidates
FAST before controller processing. The decision state guard also requires AUTO_MISSION.
Other modes cannot regain eligibility through repeated model warmup. Returning to
Mission still requires the existing fresh-frame readiness checks.
Slew <= 0 is invalid configuration, invoking immediate invalidate/zero. Parameter
metadata explicitly documents zero as fail-zero. Defaults and predictor are unchanged.

Verification commands and exit codes:

- c++ -std=c++17 -Wall -Wextra -Werror -fsanitize=address,undefined fast_control_integration/fast_control_test.cpp -o /tmp/fast_control_test: 0
- /tmp/fast_control_test: 0 (77 boundary combinations, existing cases, Mission-exit/non-mission persistence, zero and negative slew from nonzero actual)
- python3 fast_control_integration/source_isolation_test.py: 0 (including production Mission guard linkage)
- cmake --build build/px4_fmu-v6c_default --target clean: 0 (2222 files cleaned)
- make px4_fmu-v6c_default: 0 (1188 build steps)
- git diff --check: 0
- git diff --exit-code e50283e7ba -- src/modules/fw_rate_control/FastV1ShadowModel.hpp src/modules/fw_rate_control/FastV2CandidateModels.hpp: 0

Host tests exercise state_ok and source checks verify its actual Mission mapping;
they do not claim a hardware or full uORB Run simulation.
Build logs: /tmp/fast-safety-clean.log and /tmp/fast-safety-build.log.
Firmware: build/px4_fmu-v6c_default/px4_fmu-v6c_default.px4
SHA256: 01f57e33915d9e2281e9c9b4c2538a473b623d22c51d343666e6b678a9143896
This supersedes the parent report's eligibility and slew-zero descriptions.
No actuator-margin implementation, model changes, flash or hardware testing.

READY_FOR_FAST_ON_FLIGHT=NO

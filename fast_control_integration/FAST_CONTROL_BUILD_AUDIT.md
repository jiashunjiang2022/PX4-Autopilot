# Build and verification evidence

Worktree: /Users/jiangjiashun/PX4/PX4-Autopilot-v6-fast-control-20260923
Branch: fast-v2c-low-authority-control-20260923
HEAD: 19a108f17666942d199fb3856be5e85bc9aa6f83 (uncommitted implementation).

Command: make px4_fmu-v6c_default
Attempt 1 exit 2: new FastControl.hpp used unavailable NuttX <algorithm>.
Attempt 2 exit 2: PX4 -Werror=float-equal rejected exact floating comparison.
Both implementation issues fixed with finite-guarded scalar min/max and ordered comparisons.
Final attempt exit 0. Full logs: BUILD_ATTEMPT_1.log, BUILD_ATTEMPT_2.log, BUILD_FINAL.log.
FLASH 1852908 bytes / 1920 KB (94.24%); AXI SRAM 91212 bytes (17.40%).
Linker warns about RWX LOAD segment; not suppressed. No hardware execution claimed.

Firmware: build/px4_fmu-v6c_default/px4_fmu-v6c_default.px4
SHA256: 07cdb0e4e1b678786d1fbc7fd41be86f7b2e746b65239ae84bdf41ea1f342d13

Host compile: c++ -std=c++17 -Wall -Wextra -Werror -fsanitize=address,undefined
fast_control_integration/fast_control_test.cpp -o /tmp/fast_control_test
Compile and execution exit 0; HOST_TEST_RESULT.txt records 77 boundary cases plus state/history checks.
python3 fast_control_integration/source_isolation_test.py: exit 0; SOURCE_ISOLATION_RESULT.txt.
git diff --check: exit 0. WORKING_SOURCE_DIFF.patch records tracked changes; new files are
present separately. GIT_STATUS.txt records status. Build initialized required submodules;
NuttX has generated untracked tools/jlink-nuttx; not deleted or committed.

Repository astyle wrapper failed on installed formatter's obsolete indent-preprocessor/add-brackets
options; new control helper formatted using supported equivalent basic layout options.
No broad baseline reformatting. Tests/build include the final production source.

BUILD_PASS=YES
READY_FOR_DISARMED_BENCH=YES (software preparation only; bench not performed)
READY_FOR_FAST_ON_FLIGHT=NO

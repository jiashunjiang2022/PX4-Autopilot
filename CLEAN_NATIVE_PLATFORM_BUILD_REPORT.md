# Platform build report

Target: make px4_fmu-v6c_default. Exit code 0.
Fresh independent worktree and initially absent build directory; full 1173-step build PASS.
Final metadata rebuild at 27b38b3c184e6eb080a7fb9e7f9a0ed5aa08ff34 PASS.
Logs: /tmp/roll-tbr-formal-lineage-20261009/stage_a_build.log and stage_a_final_build.log.
No build products reused from the original worktree. Submodules independently cloned
using read-only local object alternates then detached at required gitlinks.

Toolchain: ARM GCC 13.2.1, Ninja, Python 3.14, empy 3.3.4, pyros-genmsg 0.5.8.
Parameters and uORB generated; AS5600, RPMCapture, WingPhase and CRSF compiled/linked.
FLASH=1797188 bytes / 1966080 bytes (91.41%). Static AXI_SRAM=62476 / 524288 bytes.
Static linker RAM excludes runtime heap/stack requirements.
.px4 SHA256=a38ce9ad5da005b8c42e68789dd2666c441ccc38759f2774748d0ed19b49e8c2
.bin SHA256=8a336efe2f20faf4f4239353bba267cdcb6e7455fff3bfb1bd0b515b86d9f800
Artifact: build/px4_fmu-v6c_default/px4_fmu-v6c_default.px4.

sh tools/roll_tbr_validation/run_platform_host_tests.sh: PLATFORM_HOST_TESTS=PASS.
Tests compile actual donor math and handover against generated uORB headers, without mocks.
Source audit and git diff --check PASS. Firmware linker reports native RWX LOAD-segment
warning; no source behavior was changed to suppress it. Build success is not hardware qualification.

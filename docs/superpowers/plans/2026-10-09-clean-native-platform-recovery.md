# Clean native platform recovery

Base: ec278758eda642005a479e2f8d278f2be8795eaf. Strategy: manual minimal port.
Original worktree is read-only; all work and builds use this independent worktree.

## Ordered commits and exact scope

1. CRSF: boards/px4/fmu-v6c/default.px4board, one enable line from
   80ce2e164d432cdcb321b100223cc842e131ee97. No RC driver changes.
2. AS5600: src/drivers/encoder/Kconfig and src/drivers/encoder/as5600/*;
   src/drivers/drv_sensor.h device ID; msg/EncoderCount.msg,
   msg/FlapFrequency.msg, msg/CMakeLists.txt; minimal logger registration in
   src/modules/logger/logged_topics.cpp; boards/px4/fmu-v6c/default.px4board
   and init/rc.board_sensors. Sensor-only snapshot from
   26418be799d5883e5b92567c3f3ed360ce82e822; initial provenance
   5ba7661eaaf11b70ca0ea72802c03f88914e5138 and
   8370f91f193739798ba314844807f93ad863a66b. FLAP_RATIO=8.
3. Hall: src/drivers/rpm_capture/{RPMCapture.cpp,RPMCapture.hpp,rpm_capture_params.c},
   src/modules/wing_phase/*, msg/{HallEvent.msg,WingPhase.msg}, message/logger
   registrations, same board config/startup. Sensor-only snapshot from 26418be;
   provenance f9508db84d21cc462568ca9ddd432c40719daaf7,
   650836c48937f654a0a81fa894b381ff5b130cc2,
   9d1947ee8b893f7ab2dfd89710c13fa92e6cc8f4 and sensor/math hunks only of
   b49bd539f7d0b82d2b82cfa068cae00774007d1d.
4. Fixed-wing board: only default.px4board module-disable hunks from
   753a2233548107b1df08978284ab5a89292c1737. No EKF changes.
5. Handover: exact five-file patch under src/modules/fw_lateral_longitudinal_control
   from bd6c81c457c4c93ece40a192991c6c97a5445432: CMakeLists.txt,
   FwControlHandover.hpp, FwControlHandoverTest.cpp,
   FwLateralLongitudinalControl.cpp/.hpp. Apply standalone patch, no other donor changes.
6. tools/roll_tbr_validation/verify_clean_native_platform.py and host qualification
   harness, reports CLEAN_NATIVE_PLATFORM_MANIFEST.md,
   CLEAN_NATIVE_PLATFORM_SOURCE_ISOLATION.md,
   CLEAN_NATIVE_PLATFORM_BUILD_REPORT.md, HARDWARE_QUALIFICATION_BLOCKERS.md.

No whole mixed commit or donor tree is restored. No trim/range change has sufficient
aircraft evidence. e624a99 saturation correction and 4cd4d0a startup retry are forbidden.

## Verification and freeze

Verify blacklist absence in executable source, byte identity of native rate controller,
rate-control library, allocator, EKF and RC driver against base. Verify exact handover
patch, treatment independence, sensor-only dependencies and board enablement.
Compile/run real sensor math and handover host tests. Generate parameters/uORB and
build from a fresh build directory with independently checked-out pinned submodules.
Run git diff --check, record build exit code, ELF flash/RAM and firmware SHA256.
Commit source before build so firmware provenance names the source commit; commit
reports separately and document build-source versus final frozen documentation SHA.
Freeze final Stage A HEAD only after required checks pass. Stage B branches from that
SHA and never modifies Stage A. No flashing or flight; hardware qualification remains open.

# Clean native platform manifest

Base: ec278758eda642005a479e2f8d278f2be8795eaf.
Branch: clean-native-platform-20261009.
Strategy: STRATEGY_C_FRESH_BASE_MANUAL_PORT.
Sensor donor only: 26418be799d5883e5b92567c3f3ed360ce82e822.
Frozen SHA is the documentation commit containing this manifest (git rev-parse HEAD).
Build-source SHA: 27b38b3c184e6eb080a7fb9e7f9a0ed5aa08ff34.
This final manifest commit adds documentation only; no executable source delta.

## Commits before documentation freeze

```text
77883a5148c89502a453459971f260d57c076edd docs(platform): plan clean native recovery and qualification
3a4d4e7bf38c67797cd74c583ffed5af17faee82 chore(platform): restore FMUv6C CRSF support
c2f5f97158753310a4895293589cef8c21d04fbc feat(platform): restore AS5600 flap frequency sensing
0d82137513dda8801de3fefbd35d9b1e5cd65f9d feat(platform): restore Hall wing phase telemetry
aea94d23ccc7cbcee96ec35e0afffe1eeb9a2398 chore(platform): select fixed-wing FMUv6C modules
f8e6713f66e3a2524b6e268646be433e182838cf feat(platform): restore exact auto-entry setpoint handover
8cd285c098326c42be738683957d1b3f9bc431f0 test(platform): add baseline isolation and host checks
c6eb27798a1cd45190849c0143c311caacf70792 chore(platform): preserve exact sensor donor file bytes
c8ae5cf2be6a870cb85d9ba2d0a81343f28c0684 test(platform): compile host checks against generated uORB headers
27b38b3c184e6eb080a7fb9e7f9a0ed5aa08ff34 test(platform): include PX4 visibility declarations for host build
```

Source whitelist and exact donor SHAs: docs/superpowers/plans/2026-10-09-clean-native-platform-recovery.md.
Executable sensor files match sensor donor bytes. Native rate-control library, fixed-wing
rate controller, attitude controller, allocator, EKF, sensors/airspeed selector, RC driver
and Dataman code match base. CRSF board enable only; no retry.
Blacklist: old FLAP_SLOW/FAST/FFP, FastControl, FastV1/2, B2B, FFPlus,
flap_fast_control, T_SAFE; no old experimental control/allocator paths.
Generic native predictor/slow/shadow wording is classified rather than banned.

Build PASS; host tests PASS; source isolation PASS; git diff --check PASS.
Firmware .px4 SHA256: a38ce9ad5da005b8c42e68789dd2666c441ccc38759f2774748d0ed19b49e8c2.
FLASH=1797188 bytes; static AXI_SRAM=62476 bytes.
Tracked source clean. NuttX contains build-generated untracked tools/jlink-nuttx;
its tracked files remain pinned and unchanged. No original-worktree files used as build output.
Tools directory is canonically Tools/ in Git; lowercase tools/ resolves on this macOS filesystem.

READY_FOR_NATIVE_PLATFORM_FLIGHT=NO
READY_FOR_FLASH=NO
READY_FOR_FLIGHT=NO

# Build results

Command:

```sh
make px4_fmu-v6c_default -j4
```

Result: PASS. The final incremental build regenerated the firmware after the logger-profile change and produced:

- `build/px4_fmu-v6c_default/px4_fmu-v6c_default.elf`
- `build/px4_fmu-v6c_default/px4_fmu-v6c_default.bin`
- `build/px4_fmu-v6c_default/px4_fmu-v6c_default.px4`

Memory summary:

- FLASH: 1,828,392 B / 1,920 KiB, 93.00%
- AXI SRAM: 61,892 B / 512 KiB, 11.80%

The build generated and compiled `airspeed_quality_input`, `airspeed_selector_quality_status`, and extended `ekf2_airspeed_quality` uORB sources. Final log: `artifacts/ral_revision_phase4/build_logs/px4_fmu-v6c-default-final.log`.

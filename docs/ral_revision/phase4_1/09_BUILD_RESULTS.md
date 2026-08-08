# Build results

Final command: `make px4_fmu-v6c_default -j4`.

Result: PASS. uORB generation, parameter generation, NuttX compilation, link,
binary generation, and PX4 package generation completed.

Memory report:

- FLASH: 1,829,352 B / 1,920 KiB, 93.05 percent.
- AXI SRAM: 61,892 B / 512 KiB, 11.80 percent.
- ELF `.text`: see `static_memory/elf-sections.txt`.

The first full build exposed a logger format limit because the diagnostic message
contained redundant fields. Redundant aliases were removed while all required
P0 fields were retained. Final untokenized uORB maximum is 1,418 bytes, below the
logger's 1,450-byte effective bound.

The sample-aligned diagnostic is estimated at the accepted airspeed rate rather
than 50 Hz. Final static profile estimate is 67.1 KiB/s payload and 96.9 KiB/s
with 16 bytes/sample overhead. This is not an SD-card acceptance result.

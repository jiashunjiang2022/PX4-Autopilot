# Phase 4.2 test and build results

## Host tests

All seven requested CTest targets passed:

- `unit-AS5600Math`
- `unit-WingPhaseMath`
- `unit-AirspeedQualityMode`
- `unit-AirspeedSelectorQuality`
- `unit-test_EKF_airspeed`
- `unit-test_AirspeedQuality`
- `unit-AirspeedQualityInput`

The Phase 4.2 evidence suite passed 15 Python tests covering bootstrap,
post-bootstrap mismatch, formal-interval exclusion, selector joins, observation
time R joins, error statistics, fallback mappings, concurrent quality/blockage,
mode behavior, and fail-closed schemas.

## Firmware

`px4_fmu-v6c_default` compiled and linked successfully.

| Region | Used | Capacity | Utilization |
| --- | ---: | ---: | ---: |
| FLASH | 1,829,472 B | 1,920 KiB | 93.05% |
| AXI SRAM | 61,892 B | 512 KiB | 11.80% |

The maximum generated untokenized uORB format is 1418 bytes and the logger
format-size static assertion passed.

No hardware was connected and no flight or hardware test was performed.

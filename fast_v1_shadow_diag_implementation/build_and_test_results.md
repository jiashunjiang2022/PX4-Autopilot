# Build and test results

`make px4_fmu-v6c_default`: FAIL (exit 1).

The relevant failure is:

`src/modules/logger/logger.cpp:72: static assertion failed: msg definition too long / buffer too short`.

Generated `orb_untokenized_fields_max_length` is 1818 while the existing logger format buffer is 1800. This is a message-size limit exposed by the added fields. No logger buffer or unrelated source was changed.

`cmake --build build/px4_sitl_test --target unit-rate_control_test unit-BumplessRollITransfer -j 4`: PASS (exit 0).

`unit-rate_control_test`: 19/19 PASS.  `unit-BumplessRollITransfer`: 71 PASS, 1 pre-existing environment-dependent golden-vector test SKIPPED (`B2B_ADAPTIVE_GOLDEN_DIR` unset).

No production SITL build was requested by this task. No firmware was flashed.

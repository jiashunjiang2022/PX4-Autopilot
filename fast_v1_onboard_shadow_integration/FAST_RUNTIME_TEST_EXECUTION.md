FALLBACK_HOST_RUNTIME_TEST_EXECUTED=YES
FAST_RUNTIME_TEST_COMMAND=c++ -std=c++17 -O2 /tmp/fast_v1_runtime_harness.cpp -o /tmp/fast_v1_runtime_harness && /tmp/fast_v1_runtime_harness
FAST_RUNTIME_TEST_EXIT_CODE=0
FORMAL_PX4_FAST_TEST_TARGET_BUILT=NO
FORMAL_PX4_FAST_TEST_TARGET_EXECUTED=NO

The fallback harness included the actual production `FastV1ShadowModel.hpp`
and covered strict nine-frame warm-up, exact lag indices, reset counter
retention, zero timestamp initialization, phase timing, and missed slots. The
formal SITL test build remains blocked by the unrelated macOS deprecation at
`src/systemcmds/tests/test_uart_send.c:83`; that file was not changed.

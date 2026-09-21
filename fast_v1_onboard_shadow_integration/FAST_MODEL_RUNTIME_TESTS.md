# FAST-V1 runtime tests

`FastV1ShadowModelTest.cpp` is registered as a PX4 unit-gtest target. It covers
warm-up/reset counter retention, non-finite pre-write invalidation, zeroed
invalid output, frame timestamp/delta, fixed-phase advancement, and missed
scheduled slots. Frozen model reference parity remains documented in
`CPP_PARITY_RESULTS_FINAL.md`.

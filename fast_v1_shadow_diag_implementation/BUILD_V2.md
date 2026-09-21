# Build V2

`make px4_fmu-v6c_default` completed successfully (exit 0). Artifact: `build/px4_fmu-v6c_default/px4_fmu-v6c_default.px4`.

The first attempt failed because the new topic was not listed in `msg/CMakeLists.txt`; after adding the message to the normal uORB message list, the build passed. The prior logger format-buffer failure is resolved by restoring `RateCtrlStatus.msg` to e624 and placing the new fields in `FlapFastShadow.msg`; the buffer constant itself was not changed.

Unit tests: RateControl 19/19 pass; BumplessRollITransfer 71 pass, 1 pre-existing golden-vector export skipped because `B2B_ADAPTIVE_GOLDEN_DIR` is unset.

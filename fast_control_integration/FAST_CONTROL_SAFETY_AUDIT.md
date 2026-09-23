# Safety evidence

Host C++ test exercises 77 boundary combinations including symmetric thresholds and NaN/Inf,
plus slew, live veto, stale rewarmup, disable/state/reset/invalid configuration, model history,
independent validity reset and held-frame behavior. Address/undefined sanitizers enabled.
source_isolation_test.py compares protected production files against baseline and verifies exact
baseline arithmetic text, saved-scale use and compression/yaw/model-call ordering.
These are host and source checks, NOT a full Run integration simulation or hardware proof.

V2C_INDEPENDENT_VALID_GUARD=YES
NONFINITE_FAIL_ZERO=YES
T_GATE_IMPLEMENTED=YES
T_BOUND_IMPLEMENTED=YES
SLEW_IMPLEMENTED=YES
RESET_FAIL_ZERO=YES
DISABLED_ZERO_EFFECT=YES (direct source arithmetic/path; not counterfactual closed-loop trajectory)
STALE_FAIL_ZERO=YES
ACTUATOR_MARGIN_IMPLEMENTED=NO

Residual risks: actual configured allocation and shared-tail margin unresolved; bench logging,
latency, work-queue stack headroom and disabled hardware output equivalence not measured.
Indirect future vehicle-state effects on I/S/compression are expected when enabled.
Affects-yaw and affects-compression flags refer only to direct same-cycle influence.

READY_FOR_DISARMED_BENCH=YES (FMUv6C build and host checks pass; bench not performed)
READY_FOR_FAST_ON_FLIGHT=NO

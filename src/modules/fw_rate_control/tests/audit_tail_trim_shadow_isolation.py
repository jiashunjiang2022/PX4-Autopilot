#!/usr/bin/env python3
"""Read-only baseline audit. Run at repository root; no files are changed."""
import pathlib
import subprocess

BASE = "7cd690c5bcf55e0ab18eb12749d54383bca3b194"
path = "src/modules/fw_rate_control/FixedwingRateControl.cpp"
before = subprocess.check_output(["git", "show", f"{BASE}:{path}"], text=True)
after = pathlib.Path(path).read_text()
start = after.index("void FixedwingRateControl::updateTailTrimShadow(")
end = after.index("void FixedwingRateControl::updateActuatorControlsStatus(", start)
hook = after[start:end]
assert "_tail_trim_shadow_pub.publish(log)" in hook
assert "log.actual_tail_trim_torque = 0.f;" in hook
for forbidden in ("_vehicle_torque_setpoint", "_vehicle_thrust_setpoint", "setIntegral",
                  "applyRollITransfer", "resetIntegral", "composeRawRoll"):
    assert forbidden not in hook, forbidden
stripped = after[:start] + after[end:]
stripped = stripped.replace(
    "\t_tail_trim_shadow_pub.advertise(); // allocate diagnostic topic at module construction\n", "")
stripped = stripped.replace(
    "\t\t// All actual torque composition/publication above is complete. LOG ONLY.\n"
    "\t\tupdateTailTrimShadow(dt, pilot_abort_to_stabilized, angular_velocity.timestamp_sample);\n\n", "")
start = stripped.index("\t// Shadow parameters only: reset virtual state on configuration refresh.")
end = stripped.index("\n\treturn PX4_OK;", start)
stripped = stripped[:start] + stripped[end:]
assert stripped == before, "Existing controller code changed outside enumerated shadow-only additions"
protected = ["src/lib/rate_control", "src/modules/fw_rate_control/BumplessRollITransfer.hpp",
             "src/modules/flap_aug_shadow", "src/modules/control_allocator"]
assert not subprocess.check_output(["git", "diff", BASE, "--", *protected], text=True)
for name in ("AdaptiveTailTrimShadow.hpp", "AdaptiveTailTrimShadow.cpp", "AdaptiveTailTrimEstimator.hpp"):
    source = pathlib.Path("src/modules/fw_rate_control", name).read_text()
    for forbidden in ("RateControl *", "RateControl*", "BumplessRollITransfer*", "orb_publish", "std::vector", "malloc("):
        assert forbidden not in source, (name, forbidden)
print("ACTUAL_CONTROL_PATH_BASELINE_EXACT = PASS (after removing enumerated shadow additions)")
print("V3_RATECONTROL_ALLOCATOR_FAST_UNCHANGED = PASS")
print("SHADOW_DIAGNOSTICS_ONLY = PASS")

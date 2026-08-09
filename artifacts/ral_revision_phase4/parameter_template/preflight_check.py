#!/usr/bin/env python3
"""Fail-closed offline preflight check using a bench ULog parameter snapshot."""

import argparse
import json
import math
import sys

try:
    from pyulog import ULog
except ImportError as exc:
    raise SystemExit("pyulog is required (use the PX4 development environment)") from exc


def close_enough(actual, expected):
    return math.isclose(float(actual), float(expected), rel_tol=1e-5, abs_tol=1e-5)


def report(ok, label, detail):
    print(("PASS" if ok else "FAIL") + f" {label}: {detail}")
    return ok


def topic_instances(ulog, name):
    return [dataset for dataset in ulog.data_list if dataset.name == name]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("ulog", help="disarmed/bench ULog captured after reboot")
    parser.add_argument("--config", required=True, help="run-specific frozen JSON derived from the template")
    parser.add_argument("--expected-mode", required=True, type=int, choices=range(4))
    parser.add_argument("--expected-rcst", required=True, type=float)
    args = parser.parse_args()

    with open(args.config, encoding="utf-8") as source:
        config = json.load(source)
    unresolved = [name for name, value in config.get("parameters_exact_freeze_before_bench", {}).items()
                  if value is None]
    if unresolved:
        print("FAIL unresolved frozen parameters: " + ", ".join(unresolved))
        return 1

    ulog = ULog(args.ulog)
    parameters = ulog.initial_parameters
    passed = True
    expected = dict(config.get("parameters_exact", {}))
    expected.update(config.get("parameters_exact_freeze_before_bench", {}))
    expected["EKF2_ASP_MODE"] = args.expected_mode
    expected["EKF2_ASP_RCST"] = args.expected_rcst
    for name, value in expected.items():
        present = name in parameters
        ok = present and close_enough(parameters[name], value)
        passed &= report(ok, name, f"actual={parameters.get(name)!r}, expected={value!r}")

    for name in config.get("parameters_positive", []):
        ok = name in parameters and float(parameters[name]) > 0.0
        passed &= report(ok, name, f"actual={parameters.get(name)!r}, expected > 0")

    profile = int(parameters.get("SDLOG_PROFILE", 0))
    required_bits = int(config.get("required_sdlog_profile_bits", 4096))
    passed &= report((profile & required_bits) == required_bits, "SDLOG_PROFILE",
                     f"actual={profile}, required_bits={required_bits}")

    diff_pressure = topic_instances(ulog, "differential_pressure")
    ms4525_ids = []
    for dataset in diff_pressure:
        for value in dataset.data.get("device_id", []):
            device_id = int(value)
            if ((device_id >> 16) & 0xff) == 0x48:
                ms4525_ids.append(device_id)
    passed &= report(bool(ms4525_ids), "MS4525DO", f"device_ids={sorted(set(ms4525_ids))}")

    encoders = topic_instances(ulog, "encoder_count")
    encoder_ids = {int(value) for dataset in encoders for value in dataset.data.get("device_id", []) if int(value) != 0}
    passed &= report(bool(encoder_ids), "AS5600 producer", f"device_ids={sorted(encoder_ids)}")

    quality = topic_instances(ulog, "airspeed_quality_input")
    quality_ok = False
    if quality:
        data = quality[0].data
        rates = [float(value) for value in data.get("output_rate_hz", []) if math.isfinite(float(value))]
        source_rates = [float(value) for value in data.get("measured_source_rate_hz", []) if math.isfinite(float(value))]
        quality_ok = bool(rates and source_rates and all(close_enough(value, 50.0) for value in rates)
                          and sorted(source_rates)[len(source_rates) // 2] >= 52.0)
    passed &= report(quality_ok, "quality input producer", "50 Hz grid and median physical source rate >= 52 Hz")

    aid_present = bool(topic_instances(ulog, "estimator_aid_src_airspeed"))
    passed &= report(aid_present, "airspeed aid source", "estimator_aid_src_airspeed published")

    required_topics = [
        "ekf2_airspeed_quality", "airspeed_selector_quality_status", "tecs_status",
        "fixed_wing_lateral_guidance_status", "fixed_wing_lateral_status",
        "fixed_wing_lateral_setpoint", "fixed_wing_longitudinal_setpoint",
        "vehicle_rates_setpoint", "vehicle_torque_setpoint", "vehicle_thrust_setpoint",
        "actuator_motors", "actuator_servos",
    ]
    missing = [name for name in required_topics if not topic_instances(ulog, name)]
    passed &= report(not missing, "RA-L logger profile", "missing=" + (", ".join(missing) if missing else "none"))

    logger_status = topic_instances(ulog, "logger_status")
    buffer_dropouts = max((int(value) for dataset in logger_status
                           for value in dataset.data.get("dropouts", [0])), default=0)
    message_gaps = max((int(value) for dataset in logger_status
                        for value in dataset.data.get("message_gaps", [0])), default=0)
    dropout_ok = len(ulog.dropouts) == 0 and buffer_dropouts == 0 and message_gaps == 0
    passed &= report(dropout_ok, "SD/logging continuity",
                     f"ULog dropouts={len(ulog.dropouts)}, buffer dropouts={buffer_dropouts}, message gaps={message_gaps}")

    print("RESULT=" + ("PASS" if passed else "FAIL"))
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())

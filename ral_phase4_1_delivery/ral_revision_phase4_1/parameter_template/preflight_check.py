#!/usr/bin/env python3
"""Fail-closed Phase 4.1 identity and configuration check for a bench ULog."""

import argparse
import json
import math
import sys

try:
    from pyulog import ULog
except ImportError as exc:
    raise SystemExit("pyulog is required from the existing PX4 development environment") from exc


PHYSICAL_SOURCE_MIN = 1
PHYSICAL_SOURCE_MAX = 3
TRIGGER_NONE = 0
FALLBACK_NONE = 0


def close_enough(actual, expected):
    return math.isclose(float(actual), float(expected), rel_tol=1e-5, abs_tol=1e-5)


def report(ok, label, detail):
    print(("PASS" if ok else "FAIL") + f" {label}: {detail}")
    return ok


def topic_instances(ulog, name):
    return [dataset for dataset in ulog.data_list if dataset.name == name]


def field_values(datasets, field):
    return [value for dataset in datasets for value in dataset.data.get(field, [])]


def row_count(dataset):
    return len(dataset.data.get("timestamp", []))


def value_at(dataset, field, index, default=0):
    values = dataset.data.get(field, [])
    return values[index] if index < len(values) else default


def check_source_switches(status_datasets):
    failures = []
    for dataset in status_datasets:
        previous = None
        previous_trigger = TRIGGER_NONE
        previous_fallback = FALLBACK_NONE
        for index in range(row_count(dataset)):
            source = int(value_at(dataset, "final_selected_source", index, -1))
            if previous is not None and source != previous:
                trigger = int(value_at(dataset, "trigger_reason", index, TRIGGER_NONE))
                fallback = int(value_at(dataset, "fallback_outcome", index, FALLBACK_NONE))
                original_invalid = bool(value_at(dataset, "concurrent_original_invalid", index, False))
                blockage = bool(value_at(dataset, "concurrent_blockage", index, False))
                pre_source = int(value_at(dataset, "pre_quality_source", index, -1))
                identity_match = bool(value_at(dataset, "source_identity_match", index, False))
                recovery = (source == pre_source and identity_match
                            and (previous_trigger != TRIGGER_NONE or previous_fallback != FALLBACK_NONE))
                explained = (trigger != TRIGGER_NONE or fallback != FALLBACK_NONE
                             or original_invalid or blockage or recovery)
                if not explained:
                    timestamp = int(value_at(dataset, "timestamp", index, 0))
                    failures.append((timestamp, previous, source))
            previous = source
            previous_trigger = int(value_at(dataset, "trigger_reason", index, TRIGGER_NONE))
            previous_fallback = int(value_at(dataset, "fallback_outcome", index, FALLBACK_NONE))
    return failures


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("ulog", help="disarmed/bench ULog captured after reboot")
    parser.add_argument("--config", required=True, help="run-specific frozen JSON")
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
        ok = name in parameters and close_enough(parameters[name], value)
        passed &= report(ok, name, f"actual={parameters.get(name)!r}, expected={value!r}")

    for name in config.get("parameters_positive", []):
        ok = name in parameters and float(parameters[name]) > 0.0
        passed &= report(ok, name, f"actual={parameters.get(name)!r}, expected > 0")

    profile = int(parameters.get("SDLOG_PROFILE", 0))
    required_bits = int(config.get("required_sdlog_profile_bits", 4096))
    passed &= report((profile & required_bits) == required_bits, "SDLOG_PROFILE",
                     f"actual={profile}, required_bits={required_bits}")

    diff_pressure = topic_instances(ulog, "differential_pressure")
    active_instances = {dataset.multi_id for dataset in diff_pressure if row_count(dataset) > 0}
    device_ids = {int(value) for value in field_values(diff_pressure, "device_id") if int(value) != 0}
    zero_device_seen = any(int(value) == 0 for value in field_values(diff_pressure, "device_id"))
    passed &= report(len(active_instances) == 1, "single differential-pressure instance",
                     f"instances={sorted(active_instances)}")
    passed &= report(len(device_ids) == 1 and not zero_device_seen, "single physical Pitot",
                     f"device_ids={sorted(device_ids)}, zero_seen={zero_device_seen}")
    expected_device = next(iter(device_ids), 0)
    expected_instance = next(iter(active_instances), -1)

    quality = topic_instances(ulog, "airspeed_quality_input")
    quality_devices = {int(value) for value in field_values(quality, "device_id")}
    quality_instances = {int(value) for value in field_values(quality, "source_instance")}
    quality_identity_ok = (bool(quality) and quality_devices == {expected_device}
                           and quality_instances == {expected_instance} and expected_device != 0)
    passed &= report(quality_identity_ok, "quality source identity",
                     f"devices={sorted(quality_devices)}, instances={sorted(quality_instances)}")

    diagnostics = topic_instances(ulog, "ekf2_airspeed_quality")
    diagnostic_identity_ok = bool(diagnostics)
    for dataset in diagnostics:
        for index in range(row_count(dataset)):
            source = int(value_at(dataset, "airspeed_source", index, -1))
            if PHYSICAL_SOURCE_MIN <= source <= PHYSICAL_SOURCE_MAX:
                diagnostic_identity_ok &= int(value_at(dataset, "airspeed_device_id", index, 0)) == expected_device
                diagnostic_identity_ok &= int(value_at(dataset, "quality_device_id", index, 0)) == expected_device
                diagnostic_identity_ok &= int(value_at(dataset, "quality_source_instance", index, 255)) == expected_instance
                diagnostic_identity_ok &= bool(value_at(dataset, "source_identity_match", index, False))
    passed &= report(diagnostic_identity_ok, "EKF observation identity", "physical samples match quality source")

    selector_status = topic_instances(ulog, "airspeed_selector_quality_status")
    selector_identity_ok = bool(selector_status)
    for dataset in selector_status:
        for index in range(row_count(dataset)):
            pre_source = int(value_at(dataset, "pre_quality_source", index, -1))
            quality_enabled = bool(value_at(dataset, "selector_quality_enabled", index, False))
            if quality_enabled and PHYSICAL_SOURCE_MIN <= pre_source <= PHYSICAL_SOURCE_MAX:
                selector_identity_ok &= int(value_at(dataset, "pre_quality_device_id", index, 0)) == expected_device
                selector_identity_ok &= int(value_at(dataset, "quality_device_id", index, 0)) == expected_device
                selector_identity_ok &= int(value_at(dataset, "quality_source_instance", index, 255)) == expected_instance
                selector_identity_ok &= bool(value_at(dataset, "source_identity_match", index, False))
    passed &= report(selector_identity_ok, "selector identity", "FULL path only uses the single matching Pitot")

    unexplained_switches = check_source_switches(selector_status)
    passed &= report(not unexplained_switches, "selected-source transitions",
                     f"unexplained={unexplained_switches[:10]}")

    required_topics = ["airspeed_quality_input", "ekf2_airspeed_quality",
                       "airspeed_selector_quality_status", "estimator_aid_src_airspeed"]
    missing = [name for name in required_topics if not topic_instances(ulog, name)]
    passed &= report(not missing, "required evidence topics",
                     "missing=" + (", ".join(missing) if missing else "none"))

    logger_status = topic_instances(ulog, "logger_status")
    buffer_dropouts = max((int(value) for value in field_values(logger_status, "dropouts")), default=0)
    message_gaps = max((int(value) for value in field_values(logger_status, "message_gaps")), default=0)
    continuity_ok = len(ulog.dropouts) == 0 and buffer_dropouts == 0 and message_gaps == 0
    passed &= report(continuity_ok, "SD/logging continuity",
                     f"ULog dropouts={len(ulog.dropouts)}, buffer={buffer_dropouts}, gaps={message_gaps}")

    print("RESULT=" + ("PASS" if passed else "FAIL"))
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())

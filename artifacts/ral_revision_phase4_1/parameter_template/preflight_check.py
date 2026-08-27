#!/usr/bin/env python3
"""Fail-closed Phase 4.1 identity and configuration check for a bench ULog."""

import argparse
import json
import math
import sys


PHYSICAL_SOURCE_MIN = 1
PHYSICAL_SOURCE_MAX = 3
TRIGGER_NONE = 0
FALLBACK_NONE = 0
DEFAULT_BOOTSTRAP_TIMEOUT_S = 5.0

FROZEN_QUALITY_PARAMETERS = {
    "FLAP_RATIO": 8.0,
    "EKF2_ASP_DTAU": 0.08,
    "EKF2_ASP_DV0": 15.0,
    "EKF2_ASP_QA": 0.70,
    "EKF2_ASP_QB": 0.30,
    "EKF2_ASP_RMAX": 5.0,
    "EKF2_ASP_RCST": 2.5,
    "EKF2_ASP_QOFF": 0.40,
    "EKF2_ASP_QON": 0.60,
    "EKF2_ASP_QTAU": 0.25,
    "EKF2_ASP_TOFF": 0.20,
    "EKF2_ASP_TON": 0.30,
    "EKF2_ASP_THLD": 0.25,
    "EKF2_ASP_RL": 0.5,
    "EKF2_ASP_RU": 8.0,
    "EKF2_ASP_SEVL": 0.5,
    "EKF2_ASP_SWIN": 4.0,
    "EKF2_ASP_DF": 0.5,
    "EKF2_FLAP_F_ON": 1.0,
    "EKF2_FLAP_F_OFF": 0.6,
    "EKF2_FLAP_T_ON": 0.4,
    "EKF2_FLAP_T_OFF": 1.5,
    "EKF2_FLAP_T_TO": 0.8,
    "EKF2_EAS_NOISE": 1.4,
}

EXPERIMENT_MODES = {
    "A": {"name": "Baseline", "EKF2_ASP_MODE": 0, "EKF2_ARSP_THR": 8.0, "FW_USE_AIRSPD": 1},
    "B": {"name": "Constant-R", "EKF2_ASP_MODE": 1, "EKF2_ARSP_THR": 8.0, "FW_USE_AIRSPD": 1},
    "C": {"name": "Variance-only", "EKF2_ASP_MODE": 2, "EKF2_ARSP_THR": 8.0, "FW_USE_AIRSPD": 1},
    "D": {"name": "Full", "EKF2_ASP_MODE": 3, "EKF2_ARSP_THR": 8.0, "FW_USE_AIRSPD": 1},
    "E": {"name": "No-Pitot", "EKF2_ASP_MODE": 0, "EKF2_ARSP_THR": 0.0, "FW_USE_AIRSPD": 0},
}


def experiment_mode_expectations(mode):
    mode = mode.upper()
    if mode not in EXPERIMENT_MODES:
        raise ValueError(f"unknown experiment mode {mode!r}")
    expected = dict(FROZEN_QUALITY_PARAMETERS)
    expected.update({
        "SYS_HAS_NUM_ASPD": 1,
        "ASPD_QBLK_EN": 0,
    })
    expected.update({name: value for name, value in EXPERIMENT_MODES[mode].items() if name != "name"})
    return expected


def required_evidence_topics(mode=None):
    topics = [
        "airspeed_quality_input",
        "ekf2_airspeed_quality",
        "airspeed_selector_quality_status",
    ]

    # Mode E deliberately disables EKF airspeed fusion with EKF2_ARSP_THR=0.
    if mode != "E":
        topics.append("estimator_aid_src_airspeed")

    return topics


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


def dataset_rows(datasets):
    rows = []
    for dataset in datasets:
        for index in range(row_count(dataset)):
            row = {field: values[index] for field, values in dataset.data.items() if index < len(values)}
            row["_multi_id"] = dataset.multi_id
            rows.append(row)
    return sorted(rows, key=lambda row: int(row.get("timestamp", 0)))


def is_physical_source(source):
    return PHYSICAL_SOURCE_MIN <= int(source) <= PHYSICAL_SOURCE_MAX


def identity_row_matches(row, expected_device, expected_instance):
    return (expected_device != 0
            and is_physical_source(row.get("pre_quality_source", -1))
            and int(row.get("pre_quality_device_id", 0)) == expected_device
            and int(row.get("quality_device_id", 0)) == expected_device
            and int(row.get("quality_device_id", 0)) != 0
            and int(row.get("quality_source_instance", 255)) == expected_instance
            and bool(row.get("source_identity_match", False)))


def analyze_identity_bootstrap(rows, expected_device, expected_instance, timeout_us, valid_start_us=None):
    result = {
        "completed": False,
        "bootstrap_sample_count": 0,
        "bootstrap_duration_us": 0,
        "first_matched_timestamp": None,
        "post_bootstrap_physical_sample_count": 0,
        "post_bootstrap_mismatch_count": 0,
        "post_bootstrap_mismatches": [],
        "bootstrap_overlaps_valid_interval": False,
    }
    if not rows:
        return result

    first_timestamp = int(rows[0].get("timestamp", 0))
    first_match_index = None
    for index, row in enumerate(rows):
        if identity_row_matches(row, expected_device, expected_instance):
            first_match_index = index
            break

    if first_match_index is None:
        result["bootstrap_sample_count"] = len(rows)
        result["bootstrap_duration_us"] = max(0, int(rows[-1].get("timestamp", 0)) - first_timestamp)
        return result

    first_match_timestamp = int(rows[first_match_index].get("timestamp", 0))
    result["completed"] = True
    result["bootstrap_sample_count"] = first_match_index
    result["bootstrap_duration_us"] = max(0, first_match_timestamp - first_timestamp)
    result["first_matched_timestamp"] = first_match_timestamp
    result["bootstrap_overlaps_valid_interval"] = (valid_start_us is not None
                                                   and first_match_timestamp > int(valid_start_us))

    for row in rows[first_match_index:]:
        if is_physical_source(row.get("pre_quality_source", -1)):
            result["post_bootstrap_physical_sample_count"] += 1
            if not identity_row_matches(row, expected_device, expected_instance):
                result["post_bootstrap_mismatch_count"] += 1
                result["post_bootstrap_mismatches"].append({
                    "timestamp": int(row.get("timestamp", 0)),
                    "pre_quality_source": int(row.get("pre_quality_source", -1)),
                    "pre_quality_device_id": int(row.get("pre_quality_device_id", 0)),
                    "quality_device_id": int(row.get("quality_device_id", 0)),
                    "quality_source_instance": int(row.get("quality_source_instance", 255)),
                    "source_identity_match": bool(row.get("source_identity_match", False)),
                })

    result["within_timeout"] = result["bootstrap_duration_us"] <= int(timeout_us)
    return result


def first_armed_timestamp(ulog):
    armed_timestamps = []
    for dataset in topic_instances(ulog, "vehicle_status"):
        for index in range(row_count(dataset)):
            if int(value_at(dataset, "arming_state", index, -1)) == 2:
                armed_timestamps.append(int(value_at(dataset, "timestamp", index, 0)))
    return min(armed_timestamps) if armed_timestamps else None


def check_source_switches(status_datasets, minimum_timestamp=None):
    failures = []
    for dataset in status_datasets:
        previous = None
        previous_trigger = TRIGGER_NONE
        previous_fallback = FALLBACK_NONE
        for index in range(row_count(dataset)):
            timestamp = int(value_at(dataset, "timestamp", index, 0))
            if minimum_timestamp is not None and timestamp < int(minimum_timestamp):
                continue
            source = int(value_at(dataset, "final_selected_source", index, -1))
            if previous is not None and source != previous:
                trigger = int(value_at(dataset, "trigger_reason", index, TRIGGER_NONE))
                fallback = int(value_at(dataset, "fallback_outcome", index, FALLBACK_NONE))
                original_invalid = bool(value_at(dataset, "concurrent_original_sensor_invalid", index, False))
                blockage = bool(value_at(dataset, "concurrent_blockage", index, False))
                pre_source = int(value_at(dataset, "pre_quality_source", index, -1))
                identity_match = bool(value_at(dataset, "source_identity_match", index, False))
                recovery = (source == pre_source and identity_match
                            and (previous_trigger != TRIGGER_NONE or previous_fallback != FALLBACK_NONE))
                explained = (trigger != TRIGGER_NONE or fallback != FALLBACK_NONE
                             or original_invalid or blockage or recovery)
                if not explained:
                    failures.append((timestamp, previous, source))
            previous = source
            previous_trigger = int(value_at(dataset, "trigger_reason", index, TRIGGER_NONE))
            previous_fallback = int(value_at(dataset, "fallback_outcome", index, FALLBACK_NONE))
    return failures


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("ulog", help="disarmed/bench ULog captured after reboot")
    parser.add_argument("--config", help="optional run-specific frozen JSON")
    parser.add_argument("--experiment-mode", type=str.upper, choices=EXPERIMENT_MODES,
                        help="formal experiment preset to verify: A, B, C, D, or E")
    parser.add_argument("--expected-mode", type=int, choices=range(4),
                        help="legacy numeric mode check (requires --expected-rcst)")
    parser.add_argument("--expected-rcst", type=float,
                        help="legacy constant-R check (requires --expected-mode)")
    parser.add_argument("--bootstrap-timeout-s", type=float,
                        help="maximum startup identity bootstrap duration")
    parser.add_argument("--valid-start-us", type=int,
                        help="formal valid-interval start; defaults to first armed timestamp when present")
    args = parser.parse_args()

    if args.experiment_mode is None and (args.expected_mode is None or args.expected_rcst is None):
        parser.error("use --experiment-mode A|B|C|D|E, or both --expected-mode and --expected-rcst")
    if args.experiment_mode is not None and (args.expected_mode is not None or args.expected_rcst is not None):
        parser.error("--experiment-mode cannot be combined with legacy --expected-mode/--expected-rcst")

    config = {}
    if args.config:
        with open(args.config, encoding="utf-8") as source:
            config = json.load(source)

    unresolved = [name for name, value in config.get("parameters_exact_freeze_before_bench", {}).items()
                  if value is None]
    if unresolved:
        print("FAIL unresolved frozen parameters: " + ", ".join(unresolved))
        return 1

    try:
        from pyulog import ULog
    except ImportError as exc:
        raise SystemExit("pyulog is required from the existing PX4 development environment") from exc

    ulog = ULog(args.ulog)
    parameters = ulog.initial_parameters
    passed = True
    expected = dict(config.get("parameters_exact", {}))
    expected.update(config.get("parameters_exact_freeze_before_bench", {}))
    if args.experiment_mode is not None:
        mode_info = EXPERIMENT_MODES[args.experiment_mode]
        print(f"EXPERIMENT MODE {args.experiment_mode}: {mode_info['name']}")
        expected.update(experiment_mode_expectations(args.experiment_mode))
    else:
        expected["EKF2_ASP_MODE"] = args.expected_mode
        expected["EKF2_ASP_RCST"] = args.expected_rcst

    for name, value in expected.items():
        ok = name in parameters and close_enough(parameters[name], value)
        passed &= report(ok, name, f"actual={parameters.get(name)!r}, expected={value!r}")

    for name in config.get("parameters_positive", []):
        if name in expected:
            continue
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

    selector_status = topic_instances(ulog, "airspeed_selector_quality_status")
    selector_rows = dataset_rows(selector_status)
    bootstrap_timeout_s = (args.bootstrap_timeout_s if args.bootstrap_timeout_s is not None
                           else float(config.get("identity_bootstrap_timeout_s", DEFAULT_BOOTSTRAP_TIMEOUT_S)))
    valid_start_us = args.valid_start_us if args.valid_start_us is not None else first_armed_timestamp(ulog)
    bootstrap = analyze_identity_bootstrap(selector_rows, expected_device, expected_instance,
                                           max(0, int(bootstrap_timeout_s * 1e6)), valid_start_us)
    bootstrap_ok = (bootstrap["completed"] and bootstrap.get("within_timeout", False)
                    and not bootstrap["bootstrap_overlaps_valid_interval"])
    passed &= report(bootstrap_ok, "selector identity bootstrap",
                     f"samples={bootstrap['bootstrap_sample_count']}, "
                     f"duration_us={bootstrap['bootstrap_duration_us']}, "
                     f"first_match={bootstrap['first_matched_timestamp']}, "
                     f"timeout_s={bootstrap_timeout_s}, valid_start_us={valid_start_us}")

    post_identity_ok = (bootstrap_ok and bootstrap["post_bootstrap_physical_sample_count"] > 0
                        and bootstrap["post_bootstrap_mismatch_count"] == 0)
    passed &= report(post_identity_ok, "post-bootstrap selector identity",
                     f"physical_samples={bootstrap['post_bootstrap_physical_sample_count']}, "
                     f"mismatches={bootstrap['post_bootstrap_mismatch_count']}, "
                     f"examples={bootstrap['post_bootstrap_mismatches'][:10]}")

    quality = topic_instances(ulog, "airspeed_quality_input")
    first_match_timestamp = bootstrap["first_matched_timestamp"]
    post_bootstrap_quality_rows = [row for row in dataset_rows(quality)
                                   if first_match_timestamp is not None
                                   and int(row.get("timestamp", 0)) >= first_match_timestamp]
    quality_devices = {int(row.get("device_id", 0)) for row in post_bootstrap_quality_rows}
    quality_instances = {int(row.get("source_instance", 255)) for row in post_bootstrap_quality_rows}
    quality_identity_ok = (bool(post_bootstrap_quality_rows) and quality_devices == {expected_device}
                           and quality_instances == {expected_instance} and expected_device != 0)
    passed &= report(quality_identity_ok, "post-bootstrap quality source identity",
                     f"devices={sorted(quality_devices)}, instances={sorted(quality_instances)}")

    diagnostics = topic_instances(ulog, "ekf2_airspeed_quality")
    diagnostic_identity_ok = bool(diagnostics)
    for dataset in diagnostics:
        for index in range(row_count(dataset)):
            if first_match_timestamp is None or int(value_at(dataset, "timestamp", index, 0)) < first_match_timestamp:
                continue
            if bool(value_at(dataset, "qmon", index, False)):
                continue
            source = int(value_at(dataset, "airspeed_source", index, -1))
            if PHYSICAL_SOURCE_MIN <= source <= PHYSICAL_SOURCE_MAX:
                diagnostic_identity_ok &= int(value_at(dataset, "airspeed_device_id", index, 0)) == expected_device
                diagnostic_identity_ok &= int(value_at(dataset, "quality_device_id", index, 0)) == expected_device
                diagnostic_identity_ok &= int(value_at(dataset, "quality_source_instance", index, 255)) == expected_instance
                diagnostic_identity_ok &= bool(value_at(dataset, "source_identity_match", index, False))
    passed &= report(diagnostic_identity_ok, "EKF observation identity", "physical samples match quality source")

    unexplained_switches = check_source_switches(selector_status, first_match_timestamp)
    passed &= report(not unexplained_switches, "selected-source transitions",
                     f"unexplained={unexplained_switches[:10]}")

    required_topics = required_evidence_topics(args.experiment_mode)
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

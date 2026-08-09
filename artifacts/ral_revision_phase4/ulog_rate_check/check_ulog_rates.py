#!/usr/bin/env python3
"""Check RA-L ULog topic rates, timing continuity, and logger dropouts."""

import argparse
import csv
import math
import statistics
import sys

try:
    from pyulog import ULog
except ImportError as exc:
    raise SystemExit("pyulog is required (use the PX4 development environment)") from exc


# name: (minimum median rate Hz, maximum gap seconds)
REQUIRED_TOPICS = {
    "differential_pressure": (52.0, 0.10),
    "airspeed_quality_input": (49.0, 0.06),
    "airspeed": (15.0, 0.15),
    "airspeed_validated": (8.0, 0.30),
    "encoder_count": (90.0, 0.05),
    "rpm": (90.0, 0.05),
    "flap_frequency": (90.0, 0.05),
    "wing_phase": (90.0, 0.05),
    "ekf2_airspeed_quality": (45.0, 0.08),
    "estimator_aid_src_airspeed": (15.0, 0.15),
    "estimator_status_flags": (8.0, 0.30),
    "airspeed_selector_quality_status": (8.0, 0.30),
    "tecs_status": (15.0, 0.15),
    "fixed_wing_lateral_guidance_status": (15.0, 0.15),
    "fixed_wing_lateral_status": (15.0, 0.15),
    "fixed_wing_lateral_setpoint": (15.0, 0.15),
    "fixed_wing_longitudinal_setpoint": (15.0, 0.15),
    "vehicle_attitude_setpoint": (15.0, 0.15),
    "vehicle_rates_setpoint": (40.0, 0.08),
    "vehicle_attitude": (40.0, 0.08),
    "vehicle_angular_velocity": (40.0, 0.08),
    "vehicle_torque_setpoint": (40.0, 0.08),
    "vehicle_thrust_setpoint": (40.0, 0.08),
    "actuator_motors": (50.0, 0.06),
    "actuator_servos": (50.0, 0.06),
}


def percentile(sorted_values, fraction):
    if not sorted_values:
        return math.nan
    index = min(len(sorted_values) - 1, math.ceil(fraction * len(sorted_values)) - 1)
    return sorted_values[index]


def dataset_stats(dataset, minimum_rate_hz, maximum_gap_s):
    time_field = "timestamp_sample" if "timestamp_sample" in dataset.data else "timestamp"
    timestamps = [int(value) for value in dataset.data.get(time_field, [])]
    deltas = [(timestamps[index] - timestamps[index - 1]) * 1e-6 for index in range(1, len(timestamps))]
    positive = sorted(delta for delta in deltas if delta > 0.0)
    median_dt = statistics.median(positive) if positive else math.nan
    median_rate = 1.0 / median_dt if median_dt > 0.0 else math.nan
    max_gap = max(positive) if positive else math.nan
    p95_dt = percentile(positive, 0.95)
    duplicates = sum(delta == 0.0 for delta in deltas)
    non_monotonic = sum(delta < 0.0 for delta in deltas)
    expected_dt = 1.0 / minimum_rate_hz
    estimated_dropped = sum(max(0, round(delta / expected_dt) - 1) for delta in positive if delta > 1.5 * expected_dt)
    passed = (len(timestamps) >= 2 and math.isfinite(median_rate)
              and median_rate >= minimum_rate_hz and math.isfinite(max_gap)
              and max_gap <= maximum_gap_s and duplicates == 0 and non_monotonic == 0)
    return {
        "topic": dataset.name,
        "instance": dataset.multi_id,
        "time_field": time_field,
        "count": len(timestamps),
        "median_rate_hz": median_rate,
        "median_dt_s": median_dt,
        "p95_dt_s": p95_dt,
        "max_gap_s": max_gap,
        "duplicates": duplicates,
        "non_monotonic": non_monotonic,
        "estimated_dropped_samples": estimated_dropped,
        "status": "PASS" if passed else "FAIL",
    }


def check_ulog(ulog):
    datasets = {}
    for dataset in ulog.data_list:
        datasets.setdefault(dataset.name, []).append(dataset)

    rows = []
    failed = False
    for topic, (minimum_rate_hz, maximum_gap_s) in REQUIRED_TOPICS.items():
        instances = datasets.get(topic, [])
        if not instances:
            rows.append({"topic": topic, "instance": "", "time_field": "", "count": 0,
                         "median_rate_hz": math.nan, "median_dt_s": math.nan,
                         "p95_dt_s": math.nan, "max_gap_s": math.nan, "duplicates": 0,
                         "non_monotonic": 0, "estimated_dropped_samples": 0,
                         "status": "MISSING"})
            failed = True
            continue
        for dataset in instances:
            row = dataset_stats(dataset, minimum_rate_hz, maximum_gap_s)
            rows.append(row)
            failed |= row["status"] != "PASS"

    wind_present = bool(datasets.get("wind") or datasets.get("estimator_wind"))
    if not wind_present:
        rows.append({"topic": "wind|estimator_wind", "instance": "", "time_field": "", "count": 0,
                     "median_rate_hz": math.nan, "median_dt_s": math.nan,
                     "p95_dt_s": math.nan, "max_gap_s": math.nan, "duplicates": 0,
                     "non_monotonic": 0, "estimated_dropped_samples": 0,
                     "status": "MISSING"})
        failed = True

    logger_dropouts = len(ulog.dropouts)
    logger_status = datasets.get("logger_status", [])
    logger_buffer_dropouts = 0
    logger_message_gaps = 0
    for dataset in logger_status:
        logger_buffer_dropouts = max(logger_buffer_dropouts, max(dataset.data.get("dropouts", [0]), default=0))
        logger_message_gaps = max(logger_message_gaps, max(dataset.data.get("message_gaps", [0]), default=0))
    failed |= logger_dropouts > 0 or logger_buffer_dropouts > 0 or logger_message_gaps > 0
    return rows, failed, logger_dropouts, logger_buffer_dropouts, logger_message_gaps


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("ulog")
    parser.add_argument("--csv", help="write per-topic statistics as CSV")
    args = parser.parse_args()
    ulog = ULog(args.ulog)
    rows, failed, dropouts, buffer_dropouts, message_gaps = check_ulog(ulog)
    fields = list(rows[0].keys())
    if args.csv:
        with open(args.csv, "w", encoding="utf-8", newline="") as output:
            writer = csv.DictWriter(output, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)
    writer = csv.DictWriter(sys.stdout, fieldnames=fields)
    writer.writeheader()
    writer.writerows(rows)
    print(f"logger_dropout_messages={dropouts}, logger_buffer_dropouts={buffer_dropouts}, "
          f"logger_message_gaps={message_gaps}")
    print("RESULT=" + ("FAIL" if failed else "PASS"))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Estimate RA-L profile payload and conservative ULog bandwidth from generated uORB metadata."""

import argparse
import csv
import pathlib
import re
import sys


# (topic, target producer/log rate Hz, configured maximum instances)
PROFILE = [
    ("differential_pressure", 83.0, 2), ("airspeed_quality_input", 50.0, 1),
    ("airspeed", 20.0, 1), ("airspeed_validated", 10.0, 1),
    ("encoder_count", 100.0, 1), ("rpm", 100.0, 2),
    ("flap_frequency", 100.0, 1), ("wing_phase", 100.0, 1),
    ("ekf2_airspeed_quality", 50.0, 1), ("estimator_aid_src_airspeed", 20.0, 2),
    ("wind", 20.0, 1), ("estimator_wind", 20.0, 2),
    ("estimator_status_flags", 20.0, 2), ("airspeed_selector_quality_status", 10.0, 1),
    ("tecs_status", 20.0, 1), ("fixed_wing_lateral_guidance_status", 20.0, 1),
    ("fixed_wing_lateral_status", 20.0, 1), ("fixed_wing_lateral_setpoint", 50.0, 1),
    ("fixed_wing_longitudinal_setpoint", 50.0, 1), ("vehicle_attitude_setpoint", 50.0, 1),
    ("vehicle_rates_setpoint", 50.0, 1), ("vehicle_attitude", 50.0, 1),
    ("vehicle_angular_velocity", 50.0, 1), ("vehicle_torque_setpoint", 100.0, 2),
    ("vehicle_thrust_setpoint", 100.0, 2), ("actuator_motors", 100.0, 1),
    ("actuator_servos", 100.0, 1),
]


def topic_sizes(source_dir):
    sizes = {}
    pattern = re.compile(r"ORB_DEFINE\((\w+),\s+struct\s+\w+_s,\s+(\d+),")
    for path in source_dir.glob("*.cpp"):
        for topic, size in pattern.findall(path.read_text()):
            sizes[topic] = int(size)
    return sizes


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--build-dir", default="build/px4_fmu-v6c_default")
    parser.add_argument("--csv", default="artifacts/ral_revision_phase4/ulog_rate_check/static_bandwidth.csv")
    args = parser.parse_args()
    source_dir = pathlib.Path(args.build_dir) / "msg/topics_sources"
    sizes = topic_sizes(source_dir)
    rows = []
    for topic, rate, instances in PROFILE:
        if topic not in sizes:
            raise RuntimeError(f"cannot find ORB_DEFINE metadata for {topic} in {source_dir}")
        size = sizes[topic]
        payload = size * rate * instances
        conservative = (size + 16) * rate * instances
        rows.append({"topic": topic, "message_bytes": size, "rate_hz": rate,
                     "max_instances": instances, "payload_bytes_s": int(payload),
                     "conservative_bytes_s": int(conservative)})
    output = pathlib.Path(args.csv)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    payload_total = sum(row["payload_bytes_s"] for row in rows)
    conservative_total = sum(row["conservative_bytes_s"] for row in rows)
    print(f"payload={payload_total / 1024:.1f} KiB/s")
    print(f"conservative_with_16B_per_sample_overhead={conservative_total / 1024:.1f} KiB/s")
    print("This is a static profile-only estimate; bench SD throughput and dropout checks remain mandatory.")


if __name__ == "__main__":
    sys.exit(main())

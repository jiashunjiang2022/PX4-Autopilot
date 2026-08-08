#!/usr/bin/env python3
"""Estimate the Phase 4.1 profile from generated uORB message sizes."""

import argparse
import csv
import pathlib
import re


PROFILE = [
    ("differential_pressure", 83.0, 2),
    ("airspeed_quality_input", 50.0, 1),
    ("airspeed", 20.0, 1),
    ("airspeed_validated", 10.0, 1),
    ("ekf2_airspeed_quality", 10.0, 1),
    ("airspeed_selector_quality_status", 10.0, 1),
    ("estimator_aid_src_airspeed", 20.0, 2),
    ("encoder_count", 100.0, 1),
    ("rpm", 100.0, 2),
    ("flap_frequency", 100.0, 1),
    ("wing_phase", 100.0, 1),
    ("wind", 20.0, 1),
    ("estimator_wind", 20.0, 2),
    ("airspeed_wind", 20.0, 4),
    ("tecs_status", 20.0, 1),
    ("fixed_wing_lateral_guidance_status", 20.0, 1),
    ("fixed_wing_lateral_status", 20.0, 1),
    ("fixed_wing_lateral_setpoint", 50.0, 1),
    ("fixed_wing_longitudinal_setpoint", 50.0, 1),
    ("vehicle_attitude_setpoint", 50.0, 1),
    ("vehicle_rates_setpoint", 50.0, 1),
    ("vehicle_attitude", 50.0, 1),
    ("vehicle_angular_velocity", 50.0, 1),
    ("vehicle_torque_setpoint", 100.0, 2),
    ("vehicle_thrust_setpoint", 100.0, 2),
    ("actuator_motors", 100.0, 1),
    ("actuator_servos", 100.0, 1),
]


def topic_sizes(source_dir):
    sizes = {}
    pattern = re.compile(r"ORB_DEFINE\((\w+),\s+struct\s+\w+_s,\s+(\d+),")
    for path in source_dir.glob("*.cpp"):
        for topic, size in pattern.findall(path.read_text(encoding="utf-8")):
            sizes[topic] = int(size)
    return sizes


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--build-dir", default="build/px4_fmu-v6c_default")
    parser.add_argument("--csv", default="artifacts/ral_revision_phase4_1/ulog_schema/static_bandwidth.csv")
    args = parser.parse_args()
    sizes = topic_sizes(pathlib.Path(args.build_dir) / "msg/topics_sources")
    rows = []
    for topic, rate_hz, instances in PROFILE:
        size = sizes[topic]
        rows.append({
            "topic": topic,
            "message_bytes": size,
            "rate_hz": rate_hz,
            "max_instances": instances,
            "payload_bytes_s": int(size * rate_hz * instances),
            "conservative_bytes_s": int((size + 16) * rate_hz * instances),
        })

    output = pathlib.Path(args.csv)
    with output.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)

    payload = sum(row["payload_bytes_s"] for row in rows)
    conservative = sum(row["conservative_bytes_s"] for row in rows)
    print(f"payload={payload / 1024:.1f} KiB/s")
    print(f"conservative={conservative / 1024:.1f} KiB/s")
    print("Bench SD throughput, 30-minute soak, and zero-dropout checks remain mandatory.")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3

import argparse
import csv
import re
import sys
import time

try:
    from pymavlink import mavutil
except ImportError as exc:
    print(f"Failed to import pymavlink: {exc}", file=sys.stderr)
    print("Install with: pip3 install --user pymavlink", file=sys.stderr)
    sys.exit(1)


TOPIC_FIELDS = {
    "rpm": [
        "timestamp",
        "rpm_raw",
        "rpm_estimate",
    ],
    "flap_frequency": [
        "timestamp",
        "frequency_hz",
    ],
    "encoder_count": [
        "timestamp",
        "device_id",
        "total_count",
        "position_raw",
    ],
    "wing_phase": [
        "timestamp",
        "phase_deg",
        "phase_rad",
        "phase_unwrapped_rad",
        "phase_sin",
        "phase_cos",
        "flap_frequency_hz",
        "total_count",
        "encoder_position_raw",
        "hall_pulse_count",
        "phase_valid",
    ],
    "actuator_motors": [
        "timestamp",
        "timestamp_sample",
        "reversible_flags",
        "control[0]",
    ],
    "flap_motor_setpoint": [
        "timestamp",
        "timestamp_sample",
        "thrust",
    ],
    "vehicle_attitude": [
        "timestamp",
        "timestamp_sample",
        "q[0]",
        "q[1]",
        "q[2]",
        "q[3]",
    ],
}

ANSI_ESCAPE = re.compile(r"\x1B\[[0-?]*[ -/]*[@-~]")
KEY_VAL = re.compile(r"^\s*([A-Za-z0-9_\[\]\.]+)\s*:\s*(.+?)\s*$")


class MavlinkNsh:
    def __init__(self, connection_string: str, baudrate: int = 57600, devnum: int = 10):
        self._mav = mavutil.mavlink_connection(connection_string, autoreconnect=True, baud=baudrate)
        self._mav.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GENERIC,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0,
        )
        self._mav.wait_heartbeat(timeout=10)
        self._devnum = devnum
        self._rx_buf = ""
        self.send("\n")
        self.read_until_prompt(timeout_s=1.0)

    def send(self, text: str) -> None:
        data = text.encode("utf-8", errors="ignore")
        offset = 0

        while offset < len(data):
            chunk = data[offset: offset + 70]
            payload = list(chunk) + [0] * (70 - len(chunk))
            self._mav.mav.serial_control_send(
                self._devnum,
                mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE
                | mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND,
                0,
                0,
                len(chunk),
                payload,
            )
            offset += len(chunk)

    def _poll(self, timeout_s: float = 0.05) -> str:
        msg = self._mav.recv_match(
            condition="SERIAL_CONTROL.count!=0",
            type="SERIAL_CONTROL",
            blocking=True,
            timeout=timeout_s,
        )

        if msg is None:
            return ""

        return bytes(msg.data[:msg.count]).decode("utf-8", errors="ignore")

    def read_until_prompt(self, timeout_s: float = 2.0) -> str:
        deadline = time.monotonic() + timeout_s

        while time.monotonic() < deadline:
            chunk = self._poll(timeout_s=0.05)

            if chunk:
                self._rx_buf += chunk

                if "\nNuttShell (NSH)" in self._rx_buf or "\nnsh>" in self._rx_buf or self._rx_buf.endswith("nsh> "):
                    out = self._rx_buf
                    self._rx_buf = ""
                    return out

        out = self._rx_buf
        self._rx_buf = ""
        return out

    def run(self, cmd: str, timeout_s: float = 2.0) -> str:
        self.send(cmd.rstrip() + "\n")
        return self.read_until_prompt(timeout_s=timeout_s)


def clean_output(text: str) -> str:
    return ANSI_ESCAPE.sub("", text).replace("\r", "")


def parse_listener_output(text: str) -> dict:
    result = {}

    for raw_line in clean_output(text).splitlines():
        line = raw_line.strip()
        match = KEY_VAL.match(line)

        if not match:
            continue

        key, value = match.group(1), match.group(2).strip()
        result[key] = value

    return result


def main() -> int:
    parser = argparse.ArgumentParser(description="Dump FUSION uORB topic samples via MAVLink NSH listener as CSV")
    parser.add_argument("--connect", default="udp:127.0.0.1:14550", help="MAVLink connection string")
    parser.add_argument("--baudrate", type=int, default=57600, help="Baudrate for serial links")
    parser.add_argument("--topic", required=True, choices=sorted(TOPIC_FIELDS.keys()), help="uORB topic name")
    parser.add_argument("--instance", type=int, default=0, help="uORB topic instance")
    parser.add_argument("--rate", type=float, default=10.0, help="Sampling rate in Hz")
    parser.add_argument("--count", type=int, default=50, help="Number of samples to print")
    args = parser.parse_args()

    if args.rate <= 0:
        print("--rate must be > 0", file=sys.stderr)
        return 1

    if args.count <= 0:
        print("--count must be > 0", file=sys.stderr)
        return 1

    try:
        nsh = MavlinkNsh(connection_string=args.connect, baudrate=args.baudrate)
    except Exception as exc:
        print(f"Failed to connect to MAVLink/NSH: {exc}", file=sys.stderr)
        return 1

    fields = TOPIC_FIELDS[args.topic]
    csv_out = csv.writer(sys.stdout)
    csv_out.writerow(fields)
    period_s = 1.0 / args.rate

    for _ in range(args.count):
        t0 = time.monotonic()
        cmd = f"listener {args.topic} -i {args.instance} -n 1"
        output = nsh.run(cmd, timeout_s=max(1.0, period_s * 1.5))
        values = parse_listener_output(output)

        row = [values.get(field, "") for field in fields]
        csv_out.writerow(row)
        sys.stdout.flush()

        elapsed = time.monotonic() - t0
        remaining = period_s - elapsed

        if remaining > 0:
            time.sleep(remaining)

    return 0


if __name__ == "__main__":
    sys.exit(main())

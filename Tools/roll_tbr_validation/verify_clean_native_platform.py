#!/usr/bin/env python3
"""Read-only, fail-closed source audit of the clean platform lineage."""
import pathlib
import re
import subprocess

ROOT = pathlib.Path(__file__).resolve().parents[2]
BASE = "ec278758eda642005a479e2f8d278f2be8795eaf"
DONOR = "26418be799d5883e5b92567c3f3ed360ce82e822"
HANDOVER = "bd6c81c457c4c93ece40a192991c6c97a5445432"


def git(*args):
    return subprocess.check_output(["git", *args], cwd=ROOT)


def require(ok, label):
    if not ok:
        raise AssertionError(label)
    print("PASS " + label)


def text(path):
    return (ROOT / path).read_text()


def blacklist():
    # Generic native words (slow, predictor, shadow) are not experiment identifiers.
    banned = re.compile(rb"FLAP_(?:SLOW|FAST|FFP)_|FastControl|FastV[12](?:ShadowModel|CandidateModels)|"
                        rb"FFPlus|flap_fast_control|\bT_SAFE\b|\bB2B\b")
    for path in git("ls-files", "src", "msg", "boards", "ROMFS").decode().splitlines():
        p = ROOT / path
        if p.is_file():
            require(not banned.search(p.read_bytes()), "blacklist " + path) if banned.search(p.read_bytes()) else None
    print("PASS no old experimental control identifiers")


def check():
    blacklist()
    unchanged = ["src/modules/fw_rate_control", "src/lib/rate_control",
                 "src/modules/control_allocator", "src/lib/control_allocation",
                 "src/modules/ekf2", "src/modules/sensors", "src/modules/airspeed_selector",
                 "src/drivers/rc", "src/lib/dataman_client", "src/modules/fw_att_control"]
    require(not git("diff", BASE, "--", *unchanged), "native control/allocator/EKF/RC identity")
    allowed_files = {
        "boards/px4/fmu-v6c/default.px4board", "boards/px4/fmu-v6c/init/rc.board_sensors",
        "src/drivers/drv_sensor.h", "msg/CMakeLists.txt", "src/modules/logger/logged_topics.cpp",
        "msg/EncoderCount.msg", "msg/FlapFrequency.msg", "msg/HallEvent.msg", "msg/WingPhase.msg",
        *["src/drivers/rpm_capture/" + f for f in ("RPMCapture.cpp", "RPMCapture.hpp", "rpm_capture_params.c")],
        *["src/modules/fw_lateral_longitudinal_control/" + f for f in
          ("CMakeLists.txt", "FwControlHandover.hpp", "FwControlHandoverTest.cpp",
           "FwLateralLongitudinalControl.cpp", "FwLateralLongitudinalControl.hpp")],
    }
    for p in git("diff", "--name-only", BASE).decode().splitlines():
        require(p in allowed_files or p.startswith(("src/drivers/encoder/", "src/modules/wing_phase/",
                "tools/roll_tbr_validation/", "Tools/roll_tbr_validation/", "docs/")) or ("/" not in p and p.endswith(".md")), "scope " + p)
    snapshot_paths = ["src/drivers/encoder", "src/modules/wing_phase", "src/drivers/rpm_capture",
                      "msg/EncoderCount.msg", "msg/FlapFrequency.msg", "msg/HallEvent.msg", "msg/WingPhase.msg"]
    require(not git("diff", DONOR, "--", *snapshot_paths), "sensor-only donor byte identity")
    paths = [p for p in allowed_files if p.startswith("src/modules/fw_lateral_longitudinal_control/")]
    def payload(data):
        return b"\n".join(l for l in data.splitlines() if not l.startswith((b"index ", b"@@")))
    require(payload(git("diff", BASE, "--", *paths)) ==
            payload(git("diff", HANDOVER + "^", HANDOVER, "--", *paths)), "exact handover patch")
    helper = text("src/modules/fw_lateral_longitudinal_control/FwControlHandover.hpp")
    require(not re.search(r"FLAP_|TBR|roll|rate_control|param_", helper, re.I), "handover treatment independence")
    sensor = text("src/drivers/encoder/as5600/AS5600.cpp")
    require("hall_event" not in sensor and "wing_phase" not in sensor and "kMinValidFlapFrequencyHz = 0.2f" in sensor,
            "Hall-independent absolute frequency")
    board = text("boards/px4/fmu-v6c/default.px4board")
    for line in ("CONFIG_DRIVERS_RC_CRSF_RC=y", "CONFIG_DRIVERS_ENCODER_AS5600=y",
                 "CONFIG_DRIVERS_RPM_CAPTURE=y", "CONFIG_MODULES_WING_PHASE=y"):
        require(line in board.splitlines(), line)
    require("default: 8.0" in text("src/drivers/encoder/as5600/module.yaml"), "FLAP_RATIO default")
    require("as5600 -X -a 0x36 start" in text("boards/px4/fmu-v6c/init/rc.board_sensors") and
            "wing_phase start" in text("boards/px4/fmu-v6c/init/rc.board_sensors"), "sensor startup")
    require(not git("diff", "--check"), "diff whitespace")
    print("VERIFY_CLEAN_NATIVE_PLATFORM=PASS")


if __name__ == "__main__":
    check()

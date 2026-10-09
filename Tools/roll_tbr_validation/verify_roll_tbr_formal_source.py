#!/usr/bin/env python3
"""Fail-closed audit that Stage B adds only the Roll TBR experiment surface."""
import pathlib
import subprocess

ROOT = pathlib.Path(__file__).resolve().parents[2]
PARENT = "c42789a3f0143d4be9d327f6c83c00896c0b4df7"


def git(*args):
    return subprocess.check_output(["git", *args], cwd=ROOT)


def check(ok, msg):
    if not ok:
        raise AssertionError(msg)
    print("PASS " + msg)


def main():
    changed = git("diff", "--name-only", PARENT).decode().splitlines()
    allowed = {
        "msg/CMakeLists.txt", "msg/RollTbrStatus.msg", "src/modules/logger/logged_topics.cpp",
        "src/modules/fw_rate_control/CMakeLists.txt", "src/modules/fw_rate_control/FixedwingRateControl.cpp",
        "src/modules/fw_rate_control/FixedwingRateControl.hpp", "src/modules/fw_rate_control/RollTbr.hpp",
        "src/modules/fw_rate_control/roll_tbr_params.c",
        "tools/roll_tbr_validation/roll_tbr_host_test.cpp",
        "tools/roll_tbr_validation/run_roll_tbr_host_tests.sh",
        "tools/roll_tbr_validation/verify_roll_tbr_formal_source.py",
    }
    for path in changed:
        check(path in allowed or path.startswith(("Tools/roll_tbr_validation/", "docs/superpowers/plans/", "ROLL_TBR_", "FORMAL_A_B_C_")),
              "allowed path " + path)
    forbidden = [
        "src/drivers/encoder", "src/drivers/rpm_capture", "src/modules/wing_phase",
        "src/modules/fw_lateral_longitudinal_control", "src/drivers/rc", "src/lib/rate_control",
        "src/modules/fw_att_control", "src/modules/control_allocator", "src/lib/control_allocation",
        "src/modules/ekf2", "src/modules/sensors", "src/modules/airspeed_selector",
        "boards/px4/fmu-v6c", "src/lib/dataman_client",
    ]
    for path in forbidden:
        check(not git("diff", "--quiet", PARENT, "--", path), "unchanged " + path)
    executable = [p for p in changed if not p.startswith(("ROLL_TBR_", "FORMAL_A_B_C_", "docs/"))
                  and not p.endswith("verify_roll_tbr_formal_source.py")]
    text = git("diff", PARENT, "--", *executable).decode(errors="replace")
    for token in ("FLAP_SLOW", "FLAP_FAST", "FLAP_FFP", "FastControl", "FastV1", "FastV2",
                  "FFPlus", "flap_fast_control", "T_SAFE", "B2B"):
        check(token not in text, "no legacy token " + token)
    check("FLAP_TBR_EN" in text and "RollTbrStatus" in text and "ROLL_TBR_BEGIN" in text,
          "TBR implementation, parameters and synchronous diagnostics present")
    check("native_pitch_output" in (ROOT / "msg/RollTbrStatus.msg").read_text(), "Pitch output-only diagnostic")
    check("tail" not in (ROOT / "msg/RollTbrStatus.msg").read_text().lower(), "no uncertified shared-tail control")
    check(not git("diff", "--check", PARENT), "diff whitespace")
    print("VERIFY_ROLL_TBR_FORMAL_SOURCE=PASS")


if __name__ == "__main__":
    main()

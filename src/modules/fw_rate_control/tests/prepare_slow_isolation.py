"""Generate immutable-base control fragments only in the host build directory."""
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[4]
BASE = "0e3c15559cf9ae2264502b99849dc2e2240141e5"
out = Path(sys.argv[1])
out.mkdir(parents=True, exist_ok=True)

def original(path):
    return subprocess.check_output(["git", "show", f"{BASE}:{path}"], cwd=ROOT).decode()

def section(s, start, end):
    return s[s.index(start):s.index(end, s.index(start))]

p = "src/modules/fw_rate_control/"
for name in ["FastControl.hpp", "FastFFPlus.hpp", "FastActuatorMargin.hpp"]:
    (out / name).write_text(original(p + name))
old = original(p + "FixedwingRateControl.cpp")
new = (ROOT / p / "FixedwingRateControl.cpp").read_text()
for label, s in [("Base", old), ("Isolated", new)]:
    (out / (label + "Output.inc")).write_text(section(s, "const Vector3f gain_ff(", "/* throttle passed through"))
    (out / (label + "Status.inc")).write_text(section(s, "rate_ctrl_status_s rate_ctrl_status{};", "flap_b2b_adaptive_s adaptive_status{};"))
    (out / (label + "Yaw.inc")).write_text(section(s, "// Add feed-forward from roll control output to yaw control output", "if (_vehicle_status.is_vtol_tailsitter) {\n\t\t\tconst float helper"))
# Pure SLOW helper is also compiled separately from the immutable base object.
(out / "IsolationSlowReference.hpp").write_text(original(p + "BumplessRollITransfer.hpp"))
print("Generated base/current actual output, yaw and status blocks from", BASE)

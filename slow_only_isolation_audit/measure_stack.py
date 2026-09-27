"""Recompile two production translation units in a temporary directory with .su.

Uses the completed production compilation database, preserving all optimization,
architecture and preprocessor flags. Does not modify source or production objects.
"""
import json
from pathlib import Path
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
db = json.loads((ROOT / "build/px4_fmu-v6c_default/compile_commands.json").read_text())
out = Path(tempfile.mkdtemp(prefix="slow_only_stack_"))
for suffix in ["/fw_rate_control/FixedwingRateControl.cpp", "/px4_work_queue/WorkQueue.cpp"]:
    entry, = [e for e in db if e["file"].endswith(suffix)]
    args = shlex.split(entry["command"])
    if Path(args[0]).name == "ccache":
        args.pop(0)
    args[args.index("-o") + 1] = str(out / (Path(entry["file"]).stem + ".o"))
    args.append("-fstack-usage")
    print(shlex.join(args), flush=True)
    subprocess.run(args, cwd=entry["directory"], check=True)
for path in out.glob("*.su"):
    for line in path.read_text().splitlines():
        if "::Run()" in line or ":0()" in line:
            print(line)
print("STACK_ARTIFACT_DIRECTORY=" + str(out))

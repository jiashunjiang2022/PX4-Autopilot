"""Strict production source and SLOW parity checks against the isolation base."""
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[4]
BASE = "0e3c15559cf9ae2264502b99849dc2e2240141e5"
def base(path):
    return subprocess.check_output(["git", "show", f"{BASE}:{path}"], cwd=ROOT).decode()
def current(path):
    return (ROOT / path).read_text()
def block(s, a, b):
    return s[s.index(a):s.index(b, s.index(a))]

for path in [
    "src/modules/fw_rate_control/BumplessRollITransfer.hpp",
    "src/modules/fw_rate_control/FixedB0Experiment.hpp",
    "src/lib/rate_control/rate_control.cpp", "src/lib/rate_control/rate_control.hpp",
    "src/lib/rate_control/gain_compression.cpp", "src/lib/rate_control/gain_compression.hpp",
    "src/modules/control_allocator/ControlAllocator.cpp",
    "src/modules/flap_aug_shadow/module.yaml",
    "src/modules/flap_aug_shadow/slow_engineering_shadow.cpp",
    "src/modules/flap_aug_shadow/slow_engineering_shadow.hpp",
    "src/modules/flap_aug_shadow/v4_shadow.hpp",
    "src/modules/fw_rate_control/tests/BumplessRollITransferTest.cpp",
    "src/modules/fw_rate_control/tests/FixedB0ExperimentTest.cpp",
    "src/modules/fw_rate_control/tests/SlowLpfComparatorTest.cpp",
    "src/modules/fw_rate_control/tests/AllocatorSaturationFeedbackTest.cpp",
    "src/lib/rate_control/rate_control_test.cpp",
    "msg/RateCtrlStatus.msg", "msg/RateCtrlTerms.msg", "msg/FlapB2bAdaptive.msg",
    "platforms/common/px4_work_queue/WorkQueue.cpp",
    "platforms/common/include/px4_platform_common/px4_work_queue/WorkQueue.hpp",
]:
    assert current(path) == base(path), path

p = "src/modules/fw_rate_control/FixedwingRateControl.cpp"
s, b = current(p), base(p)
for start, end in [
    ("bool FixedwingRateControl::verify_flap_slow_configuration()", "bool FixedwingRateControl::verify_fast"),
    ("// saturation handling for axis controlled by differential thrust", "/* bi-linear interpolation"),
    ("/* bi-linear interpolation", "const Vector3f gain_ff("),
    ("const Vector3f gains_before_update", "// Observer above sees baseline only."),
    ("rate_ctrl_status_s rate_ctrl_status{};", "const uint32_t publication_seq"),
]:
    assert block(b, start, end) in s, start
for fn in ["resetIntegralAndTransfer", "resetRollIntegralAndTransfer"]:
    start = "void FixedwingRateControl::" + fn + "()"
    old = block(b, start, "\n}\n")
    assert old.replace("\t_fast_control.invalidate();\n", "") in s
params = "src/modules/fw_rate_control/fw_rate_control_params.c"
assert current(params).rstrip() == base(params).split("/**\n * Enable experimental FAST control")[0].rstrip()
for name in ["FW_RR_FF", "FW_PR_FF", "FW_YR_FF", "FW_RLL_TO_YAW_FF"]:
    assert "PARAM_DEFINE_FLOAT(" + name in current(params)
assert "const Vector3f scaled_gain_ff = gain_ff / _airspeed_scaling;" in s
assert "_rate_control.setFeedForwardGain(scaled_gain_ff);" in s
logger = "src/modules/logger/logged_topics.cpp"
expected = "\n".join(l for l in base(logger).split("\n") if "flap_fast_" not in l)
assert current(logger) == expected
for path in ["msg/FlapFastControl.msg", "msg/FlapFastShadow.msg"]:
    assert not (ROOT / path).exists()

for label, pattern in [
    ("FAST_CONTROL", r"FLAP_FAST_(?:EN|K|MAX|TON|TOFF|SLEW)\b|FastControl|FastActuatorMargin|FastV1ShadowModel|FastV2CandidateModels|FastPredictor|fast_predictor|flap_fast_control|flap_fast_shadow"),
    ("FAST_FFPLUS", r"FLAP_FFP_(?:EN|D)\b|FastFFPlus")
]:
    hits = []
    paths = subprocess.check_output(["git", "ls-files", "src", "msg", "platforms", "boards", "ROMFS"], cwd=ROOT).decode().splitlines()
    for path in paths:
        file = ROOT / path
        if not file.is_file() or "/tests/" in path or file.suffix not in [".cpp", ".hpp", ".h", ".c", ".msg", ".yaml", ".cmake", ".txt"]:
            continue
        if re.search(pattern, file.read_text(errors="replace")):
            hits.append(path)
    assert not hits, hits
    print(label + "_PRODUCTION_REFERENCES=0")
print("NATIVE_FW_RR_FF_PRESERVED=YES\nNATIVE_FF_PATH_PRESERVED=YES")
print("SLOW_MATH_SOURCE_PARITY=PASS\nSLOW_COMPARATOR_SOURCE_PARITY=PASS")

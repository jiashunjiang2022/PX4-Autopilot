"""Source contracts for the SLOW ablation; complements real C++ trajectory tests."""
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[4]
BASE = "926693950a4b2fe448d4b18d859c54bf9aa8a260"


def base(path):
    return subprocess.check_output(["git", "show", f"{BASE}:{path}"], cwd=ROOT).decode()


def current(path):
    return (ROOT / path).read_text()


for path in [
    "src/lib/rate_control/rate_control.cpp",
    "src/lib/rate_control/rate_control.hpp",
    "src/modules/fw_rate_control/FastControl.hpp",
    "src/modules/fw_rate_control/FastActuatorMargin.hpp",
    "src/modules/fw_rate_control/FastFFPlus.hpp",
    "src/modules/fw_rate_control/FastV1ShadowModel.hpp",
    "src/modules/fw_rate_control/FastV2CandidateModels.hpp",
    "src/modules/fw_rate_control/fw_rate_control_params.c",
    "src/modules/fw_rate_control/tests/BumplessRollITransferTest.cpp",
    "src/lib/rate_control/rate_control_test.cpp",
    "src/modules/fw_rate_control/tests/AllocatorSaturationFeedbackTest.cpp",
    "src/modules/control_allocator/ControlAllocator.cpp",
    "src/modules/logger/logged_topics.cpp",
]:
    assert current(path) == base(path), path

path = "src/modules/fw_rate_control/FixedwingRateControl.cpp"
s, b = current(path), base(path)
# Entire output/compression/FAST/FF+ control section must remain byte-identical.
start = "/* bi-linear interpolation over airspeed for actuator trim scheduling */"
end = "flap_b2b_adaptive_s adaptive_status{};"
assert s[s.index(start):s.index(end)] == b[b.index(start):b.index(end)]
start = "const uint32_t publication_seq"
assert s[s.index(start):] == b[b.index(start):]
assert "transfer_inputs.comparator_mode = _param_flap_slow_cmp.get();" in s
for field in ["comparator_mode", "comparator_active", "original_slow_gate_active", "reversal_logic_active"]:
    assert f"adaptive_status.{field} = _bumpless_roll_i_result.{field};" in s
assert "adaptive_status.comparator_mode_requested = _param_flap_slow_cmp.get();" in s
assert 'add_optional_topic("flap_b2b_adaptive", 50)' in current("src/modules/logger/logged_topics.cpp")

path = "src/modules/fw_rate_control/BumplessRollITransfer.hpp"
s, b = current(path), base(path)
# Shared transfer, limits consumption, and original reversal code stay literal.
start, end = "void applyTransfer(", "Result enterRecovery("
assert s[s.index(start):s.index(end)] == b[b.index(start):b.index(end)]
start, end = "if (valuesOppose(_adapt_t_hat_raw, _gate_reference_raw))", "void clearReversalGateAtZero"
assert s[s.index(start):s.index(end)] == b[b.index(start):b.index(end)]
start, end = "void initializeAdaptiveHold", "void updateAdaptiveHold"
assert s[s.index(start):s.index(end)] == b[b.index(start):b.index(end)]
lpf = s[s.index("// PAIRED_LPF_BEGIN"):s.index("// PAIRED_LPF_END")]
for forbidden in ["observeGateSample", "clearReversalGateAtZero", "_reversal_waiting_for_zero", "in.gate_std_raw", "in.gate_same_sign_fraction"]:
    assert forbidden not in lpf, forbidden
assert "applyTransfer(out, rate_control, requested, false, true)" in lpf
assert "towardTargetStep(_transferred_i_raw, target, in.adapt_slew_raw_per_s * dt)" in lpf
assert "int32_t comparator_mode{0};" in s

path = "src/modules/flap_aug_shadow/module.yaml"
s, b = current(path), base(path)
block = re.search(r"        FLAP_SLOW_CMP:\n.*?(?=        FLAP_SLOW_B0:)", s, re.S)
assert block
assert s.replace(block[0], "") == b, "Existing numeric metadata changed"
assert "default: 0" in block[0]
assert "0: ORIGINAL_SLOW" in block[0] and "1: PAIRED_LPF" in block[0]
print("PASS: reference helpers, RateControl, entry filter/target, original reversal, FAST/FF+, allocator, numeric defaults and logger isolation")

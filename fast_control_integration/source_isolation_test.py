"""Read-only source contract checks, not a hardware/closed-loop equivalence test."""
from pathlib import Path
import subprocess

root = Path(__file__).resolve().parents[1]
base = '19a108f17666942d199fb3856be5e85bc9aa6f83'
def original(path):
    return subprocess.check_output(['git', 'show', f'{base}:{path}'], cwd=root).decode()

for path in ['src/lib/rate_control/rate_control.cpp',
             'src/modules/fw_rate_control/BumplessRollITransfer.hpp',
             'src/modules/fw_rate_control/FastV2CandidateModels.hpp',
             'src/modules/control_allocator/ControlAllocator.cpp']:
    assert (root/path).read_text() == original(path), path

path = 'src/modules/fw_rate_control/FixedwingRateControl.cpp'
s = (root/path).read_text()
b = original(path)
start = 'const Vector3f gains_before_update'
end = '_gain_compression.update(control_u, dt);'
assert s[s.index(start):s.index(end)+len(end)] == b[b.index(start):b.index(end)+len(end)]
assert s.index(end) < s.index('fast_augmented = fast_base + fast_scale * fast_decision.final;')
assert s.index('_param_fw_rll_to_yaw_ff.get() *') < s.index('if (apply_fast_roll)') < s.index('_fast_v1_shadow.update(')
assert 'if (fast_decision.final > 0.f || fast_decision.final < 0.f)' in s
assert 'fast_scale = _b2b_g_current;' in s
assert 'matrix::constrain(control_u + trim, -1.f, 1.f).copyTo(_vehicle_torque_setpoint.xyz);' in s
assert '_param_fast_en.get()' in s
state_guard = s[s.index('const bool fast_state ='):s.index('const hrt_abstime fast_now =')]
assert '&& _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION' in state_guard
assert 'if (_vehicle_status.nav_state != vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION) {\n\t\t\t_fast_control.invalidate();' in s
assert 'c.slew > 0.f' in (root/'src/modules/fw_rate_control/FastControl.hpp').read_text()
assert 'PARAM_DEFINE_INT32(FLAP_FAST_EN, 0);' in (root/'src/modules/fw_rate_control/fw_rate_control_params.c').read_text()
print('PASS: protected source identity, baseline arithmetic identity, compression/yaw ordering, next-cycle model ordering, default disable')

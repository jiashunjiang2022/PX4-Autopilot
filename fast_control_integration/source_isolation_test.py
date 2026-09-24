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
import re
parent = '56d1de129c2e74e994b70f59f014c17b879eb596'
runtime = (root/'src/modules/fw_rate_control/FastControl.hpp').read_text()
metadata = (root/'src/modules/fw_rate_control/fw_rate_control_params.c').read_text()
constants = dict((n, float(v)) for n, v in re.findall(r'static constexpr float (\w+) = ([.\d]+)f;', runtime))
for param, constant, default in [('K','K_MAX',1),('MAX','AUTHORITY_MAX',.005),
                               ('TON','TON_MAX',.15),('TOFF','TOFF_MAX',.18),('SLEW','SLEW_MAX',.05)]:
    match = re.search(r'/\*\*((?:(?!\*/).)*)\*/\s*PARAM_DEFINE_FLOAT\(FLAP_FAST_'+param+r', ([.\d]+)f\);', metadata, re.S)
    assert match, param
    assert float(match[2]) == default
    assert float(re.search(r'@max ([.\d]+)', match[1])[1]) == constants[constant]
    expected_min = constants['SLEW_MIN'] if param == 'SLEW' else 0
    assert float(re.search(r'@min ([.\d]+)', match[1])[1]) == expected_min
for expression in ['c.k <= K_MAX','c.max <= AUTHORITY_MAX','c.on <= TON_MAX',
                   'c.off <= TOFF_MAX','c.slew >= SLEW_MIN','c.slew <= SLEW_MAX',
                   'c.on < c.off','d.config_invalid = !config_ok;']:
    assert expression in runtime
assert constants['T_SAFE'] == .20
assert 'fc.config_invalid = fast_decision.config_invalid;' in s
for path in ['src/modules/fw_rate_control/FastV1ShadowModel.hpp',
             'src/modules/fw_rate_control/FastV2CandidateModels.hpp']:
    assert (root/path).read_bytes() == subprocess.check_output(['git','show',f'{parent}:{path}'],cwd=root)
old_runtime = subprocess.check_output(['git','show',f'{parent}:src/modules/fw_rate_control/FastControl.hpp'],cwd=root).decode()
for start_token,end_token in [('void invalidate()', 'Decision step('),
                              ('d.stale =', 'const bool config_ok'),
                              ('if (!c.enabled', 'if (!_frame.valid')]:
    assert runtime[runtime.index(start_token):runtime.index(end_token)] == old_runtime[old_runtime.index(start_token):old_runtime.index(end_token)]
assert 'PARAM_DEFINE_INT32(FLAP_FAST_EN, 0);' in (root/'src/modules/fw_rate_control/fw_rate_control_params.c').read_text()
old_source = subprocess.check_output(['git','show',f'{parent}:src/modules/fw_rate_control/FixedwingRateControl.cpp'],cwd=root).decode()
def section(text, start, end):
    return text[text.index(start):text.index(end)].strip()
assert section(s,'bool FixedwingRateControl::verify_flap_slow_configuration()', 'bool FixedwingRateControl::verify_fast_actuator_margin_configuration()') == section(old_source,'bool FixedwingRateControl::verify_flap_slow_configuration()', 'void FixedwingRateControl::resetIntegralAndTransfer()')
old_metadata = subprocess.check_output(['git','show',f'{parent}:src/modules/fw_rate_control/fw_rate_control_params.c'],cwd=root).decode()
for marker in ['PARAM_DEFINE_FLOAT(FLAP_FAST_K, 1.0f);','PARAM_DEFINE_FLOAT(FLAP_FAST_MAX, 0.005f);',
               'PARAM_DEFINE_FLOAT(FLAP_FAST_TON, 0.15f);','PARAM_DEFINE_FLOAT(FLAP_FAST_TOFF, 0.18f);',
               'PARAM_DEFINE_FLOAT(FLAP_FAST_SLEW, 0.05f);']:
    assert marker in metadata and marker in old_metadata
assert 'FW_RR_FF, 0.8' not in metadata
assert section(runtime,'struct Config', 'struct Frame') == section(old_runtime,'struct Config','struct Frame')
assert section(runtime,'// Software research ceilings', 'struct Config') == section(old_runtime,'// Software research ceilings','struct Config')
assert 'fast_actuator_config_valid && control_u.isAllFinite()' in state_guard
assert s.index('fast_margin = FastActuatorMargin::compute') < s.index('fast_decision = _fast_control.step') < s.index('if (apply_fast_roll)')
assert s.index('_fast_mission_result_sub.update') > s.index('if (apply_fast_roll)')
assert s.count('_fast_mission_seq') == 2
for identity in ['fc.total_t_current = fc.native_roll_i_current + fc.slow_s_current;',
                 'fc.p_error_used = fc.p_sp_used - fc.p_used;',
                 'fc.applied_delta_roll = fc.augmented_roll_output - fc.baseline_roll_output;',
                 'FastActuatorMargin::tail0(fc.augmented_roll_output, fc.baseline_pitch_output)',
                 'FastActuatorMargin::tail1(fc.augmented_roll_output, fc.baseline_pitch_output)']:
    assert identity in s
assert 'FastActuatorMargin::postcheck(request, baseline_pitch_unconstrained)' in s
assert 'fc.actuator_postcheck_failed = true;' in s
assert 'fc.after_t_bound = fast_decision.bounded' in s
assert 'fc.after_actuator_bound = fast_decision.after_actuator_bound' in s
assert (root/'src/modules/logger/logged_topics.cpp').read_text().count('add_optional_topic("flap_fast_control", 0)') == 2
print('PASS: protected source identity, baseline arithmetic identity, compression/yaw ordering, next-cycle model ordering, default disable')

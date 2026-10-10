#!/usr/bin/env python3
"""Fail closed on any change outside passive, marked Pitch instrumentation."""
import argparse
import base64
import hashlib
import json
import pathlib
import re
import subprocess
import zlib

ROOT = pathlib.Path(__file__).resolve().parents[2]
PARENT = '690b3b0f5e0890a50ce170dc2751520c528c654d'
BUILD = ROOT / 'build/px4_fmu-v6c_default'
CONTROLLERS = ['src/modules/fw_att_control/FixedwingAttitudeControl' + suffix for suffix in ('.cpp', '.hpp')]
CONTROLLERS += ['src/modules/fw_rate_control/FixedwingRateControl' + suffix for suffix in ('.cpp', '.hpp')]
TOPICS = ['PitchOuterDiag', 'PitchInnerDiag']
ALLOWED = set(CONTROLLERS + ['msg/CMakeLists.txt', 'msg/PitchOuterDiag.msg', 'msg/PitchInnerDiag.msg',
                            'src/modules/logger/logged_topics.cpp', 'PITCH_NATIVE_DIAGNOSTIC_AUDIT.md'])


def git(*args):
    return subprocess.check_output(['git', *args], cwd=ROOT)


def require(ok, label):
    if not ok:
        raise AssertionError(label)
    print('PASS ' + label)


def parent(path):
    return git('show', PARENT + ':' + path)


def blocks(data):
    inside = False
    kept, added, active = [], [], []
    for line in data.splitlines(keepends=True):
        if line.strip() == b'// PITCH_DIAG_BEGIN':
            require(not inside, 'non-nested diagnostic marker')
            inside, active = True, []
        elif line.strip() == b'// PITCH_DIAG_END':
            require(inside, 'balanced diagnostic marker')
            added.append(b''.join(active))
            inside = False
        elif inside:
            active.append(line)
        else:
            kept.append(line)
    require(not inside, 'closed diagnostic marker')
    return b''.join(kept), added


def feedback_check(path, additions):
    text = '\n'.join(b.decode() for b in additions)
    # Only diagnostic lvalues may be written. Existing state is read via a narrow getter allowlist.
    code = re.sub(r'//[^\n]*|/\*.*?\*/', '', text, flags=re.S)
    bad = r'\b(?:new|delete|malloc|calloc|realloc|free|printf|PX4_INFO|PX4_WARN|fopen|sleep|usleep|mutex|ScheduleDelayed|ScheduleNow)\b'
    require(not re.search(bad, code), 'no blocking/allocation/scheduling diagnostic operation ' + path)
    require(not re.search(r'\b(?:PARAM_DEFINE_\w*|DEFINE_PARAMETERS|ParamFloat|ParamInt|ParamBool)\b', code),
            'no diagnostic parameter declaration ' + path)
    require(not re.search(r'\.(?:update|copy|control_pitch|control_yaw|set\w*|reset\w*)\s*\(', code),
            'no sensor/controller mutation for diagnostics ' + path)
    allowed_calls = {'get', 'getRateControlStatus', 'get_euler_rate_setpoint', 'get_body_rate_setpoint',
                     'getGains', 'publish', 'phi', 'theta', 'advertise'}
    require(set(re.findall(r'\.(\w+)\s*\(', code)) <= allowed_calls, 'diagnostic method allowlist ' + path)
    lvalue = r'([A-Za-z_]\w*(?:(?:\.|->)[A-Za-z_]\w*|\[[^\[\]\n]*\]|\([^()\n]*\))*)'
    for match in re.finditer(lvalue + r'\s*(?:[+*/%&|^-]=|(?<![=!<>])=(?!=))', code):
        lhs = match[1]
        require(lhs.startswith(('pitch_diag_', '_pitch_diag_')), 'diagnostic-only assignment ' + lhs)
    for match in re.finditer(r'\+\+\s*([A-Za-z_]\w*)|([A-Za-z_]\w*)\s*\+\+', code):
        require((match[1] or match[2]).startswith('_pitch_diag_'), 'diagnostic-only counter')
    # getRateControlStatus writes only its diagnostic-local snapshot; publish only publishes diagnostic structs.
    for arg in re.findall(r'\.getRateControlStatus\(([^)]+)\)', code):
        require(arg.startswith('pitch_diag_'), 'integrator snapshot writes diagnostic-local storage')
    for receiver, arg in re.findall(r'(\w+)\.publish\(([^)]+)\)', code):
        require(receiver.startswith('_pitch_diag_') and arg.startswith('pitch_diag_'), 'diagnostic-only publication')
    for receiver in re.findall(r'(\w+)\.advertise\(\)', code):
        require(receiver.startswith('_pitch_diag_'), 'diagnostic-only startup advertisement')
    stripped, _ = blocks((ROOT / path).read_bytes())
    require(not re.search(rb'\b(?:pitch_diag_\w*|_pitch_diag_\w*)\b', stripped), 'no diagnostic reference outside markers ' + path)


def source_check():
    # This existence gate is intentionally the first RED failure, before production instrumentation.
    require(all((ROOT / 'msg' / (n + '.msg')).is_file() for n in TOPICS), 'Pitch diagnostic topics/markers exist')
    head = git('rev-parse', 'HEAD').decode().strip()
    require(head == PARENT or git('rev-parse', 'HEAD^').decode().strip() == PARENT, 'exact diagnostic parent')
    tracked = git('diff', '--name-only', '--ignore-submodules=untracked', PARENT).decode().splitlines()
    untracked = git('ls-files', '--others', '--exclude-standard').decode().splitlines()
    for path in set(tracked + untracked):
        require(path in ALLOWED or path.startswith('Tools/pitch_native_diagnostic/'), 'allowed path ' + path)
    outer, inner = '', ''
    for path in CONTROLLERS:
        data = (ROOT / path).read_bytes()
        stripped, added = blocks(data)
        require(bool(added) and stripped == parent(path), 'marker-stripped bytes equal parent ' + path)
        feedback_check(path, added)
        if path.endswith('.cpp'):
            if '/fw_att_control/' in path:
                outer = data.decode()
            else:
                inner = data.decode()
    logger = 'src/modules/logger/logged_topics.cpp'
    logger_text = (ROOT / logger).read_bytes()
    for name in ('pitch_outer_diag', 'pitch_inner_diag'):
        line = f'\tadd_topic("{name}", 10);\n'.encode()
        require(logger_text.count(line) == 1, 'logger interval10 ' + name)
        logger_text = logger_text.replace(line, b'')
    require(logger_text == parent(logger), 'logger adds only two topics')
    cmake = (ROOT / 'msg/CMakeLists.txt').read_bytes()
    for name in TOPICS:
        line = ('\t' + name + '.msg\n').encode()
        require(cmake.count(line) == 1, 'message registered ' + name)
        cmake = cmake.replace(line, b'')
    require(cmake == parent('msg/CMakeLists.txt'), 'message registration only')
    for topic in TOPICS:
        fields = re.findall(r'^(?:uint64|uint8|float32|bool)\s+(\w+)',
                            (ROOT / 'msg' / (topic + '.msg')).read_text(), re.M)
        require(len(fields) == len(set(fields)), 'unique message fields ' + topic)
        label = 'PITCH_OUTER_FIELD_COUNT' if topic == 'PitchOuterDiag' else 'PITCH_INNER_FIELD_COUNT'
        print(f'{label}={len(fields)}')
    forbidden = ['src/modules/fw_att_control/fw_pitch_controller.cpp', 'src/modules/fw_att_control/fw_pitch_controller.h',
                 'src/lib/rate_control', 'src/modules/fw_rate_control/RollTbr.hpp',
                 'src/modules/fw_rate_control/roll_tbr_params.c', 'msg/RollTbrStatus.msg',
                 'src/modules/control_allocator', 'src/lib/control_allocation', 'src/drivers/encoder',
                 'src/drivers/rpm_capture', 'src/modules/wing_phase', 'src/drivers/rc', 'src/modules/ekf2',
                 'src/modules/sensors', 'src/modules/airspeed_selector', 'boards/px4/fmu-v6c',
                 'src/modules/fw_lateral_longitudinal_control', 'src/drivers/pwm_out', 'src/drivers/px4io',
                 'src/modules/fw_mode_manager', 'Tools/roll_tbr_validation', 'ROMFS']
    require(not git('diff', '--ignore-submodules=untracked', PARENT, '--', *forbidden), 'all frozen paths unchanged')
    for path in CONTROLLERS:
        require('PARAM_DEFINE_' not in (ROOT / path).read_text(), 'no parameter definition ' + path)
    required_outer = ['pitch_diag_outer.rate_sp_timestamp = _rates_sp.timestamp;',
        'pitch_diag_outer.q_sp_final_published = _rates_sp.pitch;',
        'pitch_diag_outer.pitch_sp = pitch_sp;', 'pitch_diag_outer.pitch = euler_angles.theta();',
        'pitch_diag_outer.pitch_error = pitch_sp - euler_angles.theta();',
        'pitch_diag_outer.pitch_euler_rate_sp = _pitch_ctrl.get_euler_rate_setpoint();',
        'pitch_diag_outer.q_sp_pitch_ctrl_limited = _pitch_ctrl.get_body_rate_setpoint();',
        'pitch_diag_outer.q_sp_after_autotune = body_rates_setpoint(1);',
        'pitch_diag_outer.q_sp_autotune_ff = bodyrate_autotune_ff(1);',
        'pitch_diag_outer.timestamp_sample = pitch_diag_attitude_sample;',
        'pitch_diag_attitude_sample = att.timestamp_sample;',
        'pitch_diag_outer.outer_update_counter = ++_pitch_diag_outer_update_counter;']
    required_outer += ['pitch_diag_outer.attitude_sp_timestamp = _att_sp.timestamp;',
        'pitch_diag_outer.controller_dt = dt;',
        'pitch_diag_outer.autotune_pitch_active = false;',
        'pitch_diag_outer.autotune_pitch_active = true;',
        'pitch_diag_outer.landed = _landed;',
        'pitch_diag_outer.nav_state = _vehicle_status.nav_state;']
    for token in required_outer: require(token in outer, 'outer provenance ' + token)
    capture = 'const float pitch_diag_yaw_euler_rate_sp_used = _yaw_ctrl.get_euler_rate_setpoint();'
    pitch_call = '_pitch_ctrl.control_pitch(pitch_sp, _yaw_ctrl.get_euler_rate_setpoint(),'
    require(outer.index(capture) < outer.index(pitch_call) < outer.index('_yaw_ctrl.control_yaw('), 'capture yaw before Pitch and before new yaw cycle')
    require(outer.index('pitch_diag_outer.q_sp_autotune_ff = 0.f;') < outer.index('body_rates_setpoint += bodyrate_autotune_ff;') <
            outer.index('pitch_diag_outer.q_sp_autotune_ff = bodyrate_autotune_ff(1);'), 'autotune captured only inside actual valid addition')
    require(outer.index('pitch_diag_outer.q_sp_after_autotune =') < outer.index('// Tailsitter: transform from FW'), 'outer pre-mapping capture')
    require(outer.index('_rates_sp.pitch =') < outer.index('pitch_diag_outer.q_sp_final_published =') < outer.index('_rate_sp_pub.publish(_rates_sp);', outer.index('_rates_sp.pitch =')), 'outer final published setpoint')
    required_inner = ['pitch_diag_inner.rate_sp_source_timestamp = _rates_sp.timestamp;',
        'pitch_diag_inner.q_sp_used = body_rates_setpoint(1);', 'pitch_diag_inner.q_used = rates(1);',
        'pitch_diag_inner.q_error = body_rates_setpoint(1) - rates(1);', 'pitch_diag_inner.q_dot_used = angular_accel(1);',
        'pitch_diag_inner.effective_ff_gain = scaled_gain_ff(1);',
        'pitch_diag_inner.pitch_ff_term = scaled_gain_ff(1) * body_rates_setpoint(1);',
        'pitch_diag_inner.pitch_i_term_before = pitch_diag_integral_before.pitchspeed_integ;',
        'pitch_diag_inner.pitch_i_term_after = pitch_diag_integral_after.pitchspeed_integ;',
        'pitch_diag_inner.pitch_rate_control_sum = angular_acceleration_setpoint(1);',
        'pitch_diag_inner.pitch_control_after_scaling = control_u(1);',
        'pitch_diag_inner.pitch_trim = trim(1);', 'pitch_diag_inner.pitch_output_preclip = control_u(1) + trim(1);',
        'pitch_diag_inner.pitch_output_final = _vehicle_torque_setpoint.xyz[1];',
        'pitch_diag_inner.reset_integral = _rates_sp.reset_integral;',
        'pitch_diag_inner.timestamp_sample = angular_velocity.timestamp_sample;',
        'pitch_diag_inner.inner_update_counter = ++_pitch_diag_inner_update_counter;']
    required_inner += ['pitch_diag_inner.controller_dt = dt;',
        'pitch_diag_inner.airspeed_scaling = _airspeed_scaling;',
        'pitch_diag_inner.airspeed_scaling_sq = _airspeed_scaling * _airspeed_scaling;',
        'pitch_diag_inner.landed = _landed;',
        'pitch_diag_inner.nav_state = _vehicle_status.nav_state;']
    for suffix in ('p', 'i', 'd', 'ff', 'imax'):
        required_inner.append(f'pitch_diag_inner.fw_pr_{suffix} = _param_fw_pr_{suffix}.get();')
    for token in required_inner: require(token in inner, 'inner provenance ' + token)
    update = inner.index('_rate_control.update(rates, body_rates_setpoint, angular_accel, dt, _landed)')
    require(inner.index('_rate_control.getRateControlStatus(pitch_diag_integral_before);') < update <
            inner.index('_rate_control.getRateControlStatus(pitch_diag_integral_after);') < inner.index('Vector3f control_u ='), 'I-before and I-after at exact update')
    require(inner.index('pitch_diag_inner.gain_compression_pitch =') < inner.index('_gain_compression.update(control_u, dt);'), 'gain captured before adaptation')
    require(inner.index('matrix::constrain(control_u + trim') < inner.index('pitch_diag_inner.pitch_output_final ='), 'post-clip production capture')
    require('_rates_sp_sub.update(&_rates_sp);' in inner and 'Vector3f(_rates_sp.roll, _rates_sp.pitch, _rates_sp.yaw)' in inner, 'inner actual subscribed setpoint origin')
    git('diff', '--check', PARENT)
    for path in ('src/modules/fw_rate_control/RollTbr.hpp', 'src/modules/fw_rate_control/roll_tbr_params.c',
                 'src/modules/fw_att_control/fw_pitch_controller.cpp', 'src/modules/fw_att_control/fw_pitch_controller.h',
                 'src/lib/rate_control/rate_control.cpp', 'src/lib/rate_control/rate_control.hpp',
                 'Tools/roll_tbr_validation/run_roll_tbr_host_tests.sh',
                 'Tools/roll_tbr_validation/verify_roll_tbr_formal_source.py'):
        require((ROOT / path).read_bytes() == parent(path), 'frozen bytes ' + path)
    print('STRIPPED_CONTROLLER_SOURCE_EQUALS_PARENT=YES\nDIAGNOSTIC_FEEDBACK_PATH_FOUND=NO\n'
          'TBR_CONTROL_SOURCE_CHANGED=NO\nPITCH_CONTROL_SOURCE_CHANGED=NO\n'
          'ORIGINAL_TBR_VERIFIER_UNMODIFIED=YES\nDIAGNOSTIC_RUNTIME_SIDE_EFFECT_AUDIT=PASS\n'
          'PITCH_DIAG_SOURCE_ISOLATION=PASS')


def build_check():
    for topic in ('pitch_outer_diag', 'pitch_inner_diag'):
        require((BUILD / 'uORB/topics' / (topic + '.h')).is_file(), 'generated topic ' + topic)
    package = json.loads((BUILD / 'px4_fmu-v6c_default.px4').read_text())
    head = git('rev-parse', 'HEAD').decode().strip()
    require(package['git_hash'] == head and package['board_id'] == 56, 'firmware source identity')
    require(zlib.decompress(base64.b64decode(package['image'])) == (BUILD / 'px4_fmu-v6c_default.bin').read_bytes(), 'package image matches binary')
    baseline = pathlib.Path('/Users/jiangjiashun/PX4/PX4-roll-tbr-v2-20261009/build/px4_fmu-v6c_default/parameters.json')
    require(json.loads((BUILD / 'parameters.json').read_text()) == json.loads(baseline.read_text()), 'entire parameter surface unchanged')
    params = {p['name']:p for p in json.loads((BUILD / 'parameters.json').read_text())['parameters']}
    defaults = {'FLAP_TBR_EN':0, 'FLAP_TBR_FC':0.625, 'FLAP_TBR_K':0.01, 'FLAP_TBR_MAX':0.025, 'FLAP_TBR_SLEW':0.035031594}
    require(all(abs(params[n]['default']-v) < 1e-9 for n,v in defaults.items()), 'five TBR defaults unchanged')
    require({n for n in params if n.startswith('FLAP_TBR_')} == set(defaults), 'exact five TBR parameters')
    print('PARAMETER_SURFACE_CHANGED=NO\nTBR_ONLY_PARAM_COUNT=5')
    print('PITCH_DIAG_FIRMWARE_SHA256=' + hashlib.sha256((BUILD / 'px4_fmu-v6c_default.px4').read_bytes()).hexdigest())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-only', action='store_true')
    args = parser.parse_args()
    source_check()
    if not args.source_only:
        subprocess.run(['sh', 'Tools/pitch_native_diagnostic/run_pitch_diag_host_tests.sh'], cwd=ROOT, check=True)
        subprocess.run(['sh', 'Tools/roll_tbr_validation/run_roll_tbr_host_tests.sh'], cwd=ROOT, check=True)
        build_check()
    print('VERIFY_PITCH_DIAGNOSTIC_LOGGING_ONLY=PASS')


if __name__ == '__main__':
    main()

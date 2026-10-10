#!/usr/bin/env python3
"""Fail closed on modifications outside the frozen PFF feature surface."""
import argparse
import base64
import hashlib
import json
from pathlib import Path
import re
import subprocess
import zlib
ROOT=Path(__file__).resolve().parents[2]
BASE='49d49b63c62c9eb0a8df018337f50995aec75455'
RATE='src/modules/fw_rate_control/'
CONTROLLERS=[RATE+'FixedwingRateControl'+s for s in ('.cpp','.hpp')]
ALLOWED=set(CONTROLLERS+[RATE+'PitchFfPhaseShaper.hpp',RATE+'pitch_ff_phase_shaper_params.c','msg/PitchFfShaperStatus.msg','msg/CMakeLists.txt','src/modules/logger/logged_topics.cpp','PITCH_FF_PHASE_SHAPER_AUDIT.md'])
def git(*args): return subprocess.check_output(['git',*args],cwd=ROOT)
def require(ok,label):
    if not ok: raise AssertionError(label)
def parent(path): return git('show',BASE+':'+path)
def blocks(data):
    inside=False;kept=[];added=[];current=[]
    for line in data.splitlines(keepends=True):
        if line.strip()==b'// PITCH_FF_SHAPER_BEGIN':
            require(not inside,'non-nested marker');inside=True;current=[]
        elif line.strip()==b'// PITCH_FF_SHAPER_END':
            require(inside,'balanced marker');added.append(b''.join(current));inside=False
        elif inside: current.append(line)
        else: kept.append(line)
    require(not inside,'closed marker')
    return b''.join(kept),added
def source_check():
    require((ROOT/'msg/PitchFfShaperStatus.msg').is_file(),'PFF feature integration not implemented')
    head=git('rev-parse','HEAD').decode().strip()
    require(head==BASE or git('rev-parse','HEAD^').decode().strip()==BASE,'exact PFF parent')
    changed=set(git('diff','--name-only','--ignore-submodules=untracked',BASE).decode().splitlines())|set(git('ls-files','--others','--exclude-standard').decode().splitlines())
    require(all(p in ALLOWED or p.startswith('Tools/pitch_ff_phase_shaper/') for p in changed),'allowed paths '+str(sorted(changed-ALLOWED)))
    added=[]
    for path in CONTROLLERS:
        stripped,extra=blocks((ROOT/path).read_bytes())
        require(extra and stripped==parent(path),'marker-stripped exact BASE '+path);added.extend(extra)
        require(not re.search(rb'\b(?:pff_\w*|_pitch_ff_\w*|_param_flap_pff_\w*)\b',stripped),'no escaped PFF reference')
    cpp=(ROOT/CONTROLLERS[0]).read_text()
    code=re.sub(r'//[^\n]*|/\*.*?\*/','',b'\n'.join(added).decode(),flags=re.S)
    require(not re.search(r'\b(?:new|delete|malloc|calloc|realloc|free|printf|PX4_INFO|fopen|sleep|usleep|mutex|ScheduleNow|ScheduleDelayed)\b',code),'no hot-loop allocation/blocking/scheduling')
    lhs=r'([A-Za-z_]\w*(?:(?:\.|->)[A-Za-z_]\w*|\[[^\[\]\n]*\]|\([^()\n]*\))*)'
    for m in re.finditer(lhs+r'\s*(?:[+*/%&|^-]=|(?<![=!<>])=(?!=))',code):
        require(m[1].startswith(('pff_','_pitch_ff_','_wing_phase')) or m[1]=='control_u(1)','PFF-only assignment '+m[1])
    require('control_u(0)' not in code and 'control_u(2)' not in code,'no Roll/Yaw writes')
    require(not re.search(r'_vehicle_torque_setpoint\.xyz\[[02]\]',code),'no Roll/Yaw torque access')
    require(all(t not in code for t in ['_rate_control.','_pitch_ctrl.','_roll_tbr.']),'no native controller mutation')
    for token in ['pff_input.q_sp = body_rates_setpoint(1);','pff_input.controller_active = _vcontrol_mode.flag_control_rates_enabled && _vcontrol_mode.flag_control_auto_enabled;',
        'pff_input.armed = _vcontrol_mode.flag_armed;','pff_input.mission = _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;',
        'pff_input.landed = _landed;','!_vehicle_status.is_vtol_tailsitter','!_vehicle_status.in_transition_mode',
        'pff_input.enable = _param_flap_pff_en.get() == 1;',
        'pff_input.phase_fresh = pitch_ff_phase_shaper::phaseFresh(_wing_phase.timestamp, pff_now);',
        'pff_status.gain_compression_pitch = _gain_compression.getGains()(1);','pff_status.effective_ff_gain = scaled_gain_ff(1);']:
        require(token in code,'production provenance/gate '+token)
    order=['pff_status.gain_compression_pitch =','_gain_compression.update(control_u, dt);',
        'control_u(1) = static_cast<float>(pff_mapping.shaped_pitch_control);','matrix::constrain(control_u + trim',
        'control_u(1) = pff_status.native_pitch_control;','pitch_diag_inner.pitch_output_preclip =','tbr_input.inputs_valid =']
    positions=[cpp.index(t) for t in order]
    require(positions==sorted(positions),'native compression/insertion/restore order')
    require(cpp.count('_gain_compression.update(control_u, dt);')==1,'single native compression update')
    helper=(ROOT/(RATE+'PitchFfPhaseShaper.hpp')).read_text()
    require('estimate=_a1*c1+_b1*s1+_a2*c2+_b2*s2;' in helper and 'constexpr double norm=2.+1e-9;' in helper,'authoritative internal four-basis NLMS')
    require('out.qhat_1f_applied=alpha_effective*q1_unfaded;' in helper and 'r.qhat_1f_applied=r.alpha_effective*r.qhat_1f;' in helper,'alpha times 1f only')
    require('-effective_ff_gain * r.qhat_1f_applied' in helper,'FF-only mapping')
    require(not re.search(r'(?:applied|ff_delta_rate_control|normalized_correction)\s*[=+].*2f',helper),'no applied 2f path')
    params=(ROOT/(RATE+'pitch_ff_phase_shaper_params.c')).read_text()
    require(re.findall(r'PARAM_DEFINE_\w+\((\w+),',params)==['FLAP_PFF_EN','FLAP_PFF_ALPHA','FLAP_PFF_ADAPT'],'exact three parameters')
    for token in ['PARAM_DEFINE_INT32(FLAP_PFF_EN, 0);','PARAM_DEFINE_FLOAT(FLAP_PFF_ALPHA, 0.5f);','PARAM_DEFINE_FLOAT(FLAP_PFF_ADAPT, 1.0f);']: require(token in params,'default '+token)
    require('FLAP_PFF_ALPHA2' not in cpp+helper+params+(ROOT/CONTROLLERS[1]).read_text(),'no alpha2 parameter')
    for path,line in [('msg/CMakeLists.txt',b'\tPitchFfShaperStatus.msg\n'),('src/modules/logger/logged_topics.cpp',b'\tadd_topic("pitch_ff_shaper_status", 10);\n')]:
        data=(ROOT/path).read_bytes();require(data.count(line)==1 and data.replace(line,b'')==parent(path),'single registration '+path)
    fields=re.findall(r'^(?:float32|bool|uint64|uint8)\s+(\w+)',(ROOT/'msg/PitchFfShaperStatus.msg').read_text(),re.M)
    require(len(fields)==len(set(fields)),'unique status fields')
    for field in ['coeff_cos_1f','coeff_sin_1f','coeff_cos_2f_internal','coeff_sin_2f_internal','qhat_1f','qhat_2f_internal','qhat_1f_applied','native_pitch_control','shaped_pitch_final']: require(field in fields,'status '+field)
    git('diff','--check',BASE)
    for label in ['RATE_CONTROL_SOURCE','PITCH_CONTROLLER_SOURCE','ROLL_TBR_SOURCE','ROLL_TBR_PARAMETER_SOURCE','ROLL_TBR_PARAMS','PITCH_OUTER_DIAG_SCHEMA','PITCH_INNER_DIAG_SCHEMA','Q_SP_CONTROL_PATH','Q_MEASUREMENT_PATH','NATIVE_P_PATH','NATIVE_I_PATH','NATIVE_D_PATH','GAIN_COMPRESSION_STATE_INPUT','VERTICAL_YAW_PATH','ALLOCATOR','CRSF','AS5600','HALL','EKF','AIRSPEED_CONTROL','AUTO_ENTRY_HANDOVER','FW_PSP_OFF','PITCH_NATIVE_GAINS','ROLL_NATIVE_GAINS']: print(label+'_CHANGED=NO')
    for label in ['STRIPPED_FW_RATE_CONTROL_EQUALS_BASE','PFF_QSP_READ_ONLY','PFF_RATECONTROL_INPUT_UNCHANGED','PFF_GAIN_COMPRESSION_UPDATE_NATIVE_ONLY','PFF_CORRECTION_AFTER_GAIN_COMPRESSION_UPDATE','PFF_FINAL_EXISTING_CONSTRAIN_REUSED','PFF_CORRECTION_WRITES_ONLY_PITCH_OUTPUT_PATH','PFF_MISSION_ONLY_GATE','PFF_INTERNAL_2F_ESTIMATOR_PRESENT','PFF_APPLIED_QHAT_EQUALS_ALPHA_TIMES_QHAT1F']: print(label+'=YES')
    print('PFF_APPLIED_2F_PATH_PRESENT=NO\nPFF_ALPHA2_PARAMETER_PRESENT=NO\nPFF_NEW_PARAM_COUNT=3\nPITCH_PFF_SOURCE_ISOLATION=PASS')
def build_check():
    build=ROOT/'build/px4_fmu-v6c_default'
    baseline=Path('/Users/jiangjiashun/PX4/PX4-roll-tbr-v2-pitch-diag-20261010/build/px4_fmu-v6c_default')
    require((build/'uORB/topics/pitch_ff_shaper_status.h').is_file(),'generated status')
    old={p['name']:p for p in json.loads((baseline/'parameters.json').read_text())['parameters']}
    new={p['name']:p for p in json.loads((build/'parameters.json').read_text())['parameters']}
    require(set(new)-set(old)=={'FLAP_PFF_EN','FLAP_PFF_ALPHA','FLAP_PFF_ADAPT'},'generated exact three parameters')
    require(all(new[n]==p for n,p in old.items()),'existing parameter surface unchanged')
    for n,v in [('FLAP_PFF_EN',0),('FLAP_PFF_ALPHA',.5),('FLAP_PFF_ADAPT',1.)]: require(new[n]['default']==v,'generated default '+n)
    package=json.loads((build/'px4_fmu-v6c_default.px4').read_text())
    require(package['git_hash']==git('rev-parse','HEAD').decode().strip() and package['board_id']==56,'firmware source/board identity')
    require(zlib.decompress(base64.b64decode(package['image']))==(build/'px4_fmu-v6c_default.bin').read_bytes(),'package/binary identity')
    print('PFF_EXISTING_PARAM_VALUE_DEFAULT_CHANGED=NO')
    print('PITCH_PFF_FIRMWARE_SHA256='+hashlib.sha256((build/'px4_fmu-v6c_default.px4').read_bytes()).hexdigest())
    print('PITCH_PFF_BUILD_PROVENANCE=PASS')
if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--source-only',action='store_true');args=parser.parse_args()
    source_check()
    if not args.source_only: build_check()

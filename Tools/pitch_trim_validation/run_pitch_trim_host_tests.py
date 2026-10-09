#!/usr/bin/env python3
"""Compile production trim expressions and mode guards; compare historical source."""
import pathlib
import re
import subprocess
import tempfile

ROOT = pathlib.Path(__file__).resolve().parents[2]
HISTORICAL = '926693950a4b2fe448d4b18d859c54bf9aa8a260'
PATH = 'src/modules/fw_lateral_longitudinal_control/FwLateralLongitudinalControl.cpp'


def expression(text, target):
    matches = re.findall(r'(?m)^\s*' + re.escape(target) + r'\s*=\s*(.*?);', text, re.S)
    assert len(matches) == 1, (target, matches)
    return matches[0]


def main():
    current = (ROOT / PATH).read_text()
    historical = subprocess.check_output(['git', 'show', HISTORICAL + ':' + PATH], cwd=ROOT).decode()
    assert current == historical, 'Restored longitudinal file must exactly match flown source'
    body = expression(current, 'pitch_sp')
    status = expression(current, 'tecs_status.pitch_sp_rad')
    fallback = expression(current, 'float pitch_body')
    should_run = expression(current, 'const bool should_run')
    gate = re.search(r'if \((should_run[^\n]+)\) \{', current)[1]
    manual_clamp = re.search(r'pitch_body = constrain\((.*?);', current, re.S)[1]
    update = re.search(r'_tecs.update\((.*?)\);', current, re.S)[1]
    inputs = update.split(',')
    assert len(inputs) == 16 and all('_param_fw_psp_off.get()' in inputs[i] for i in (0, 9, 10))
    header = (ROOT / 'src/modules/fw_lateral_longitudinal_control/FwLateralLongitudinalControl.hpp').read_text()
    assert '(ParamFloat<px4::params::FW_PSP_OFF>) _param_fw_psp_off' in header
    assert 'updateParams();' in current and 'if (_parameter_update_sub.updated())' in current
    assert gate == 'should_run && !_auto_control_handover.pending()'
    code = r'''
#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
using math::radians;
using math::constrain;
struct Param { float value; float get() const { return value; } };
struct Tecs { float value; float get_pitch_setpoint() const { return value; } };
struct LongSp { float pitch_direct; };
struct Mode {
    bool flag_control_position_enabled{}, flag_control_velocity_enabled{}, flag_control_acceleration_enabled{};
    bool flag_control_altitude_enabled{}, flag_control_climb_rate_enabled{};
    bool flag_control_manual_enabled{}, flag_control_auto_enabled{};
};
struct Vehicle { int vehicle_type; bool in_transition_mode; };
struct vehicle_status_s { static constexpr int VEHICLE_TYPE_FIXED_WING = 2; };
template<class T> struct Sub { T value; const T &get() const { return value; } };
struct Handover { bool value; bool pending() const { return value; } };
struct Output { float raw, body, status; };
static Output evaluate(float offset, float direct, float tecs, bool manual)
{
    const Param _param_fw_psp_off{offset}, _param_fw_p_lim_min{-5.f}, _param_fw_p_lim_max{30.f};
    const Tecs _tecs{tecs}; const LongSp _long_control_sp{direct};
    float pitch_sp = @BODY@;
    struct { float pitch_sp_rad; } tecs_status{};
    tecs_status.pitch_sp_rad = @STATUS@;
    float pitch_body = @FALLBACK@;
    if (manual) { pitch_body = constrain(@CLAMP@; }
    return {pitch_sp, pitch_body, tecs_status.pitch_sp_rad};
}
static bool active(Mode mode, Vehicle vehicle, bool pending)
{
    const Sub<Mode> _control_mode_sub{mode}; const Sub<Vehicle> _vehicle_status_sub{vehicle};
    const Handover _auto_control_handover{pending};
    const bool should_run = @SHOULD@;
    return @GATE@;
}
static float relative(float body, float offset)
{
    const Param _param_fw_psp_off{offset};
    const struct { float pitch_rad; } _long_control_state{body};
    return @INPUT@;
}
static float minRelative(float pitch_min_rad, float offset)
{
    const Param _param_fw_psp_off{offset}; return @MIN@;
}
static float maxRelative(float pitch_max_rad, float offset)
{
    const Param _param_fw_psp_off{offset}; return @MAX@;
}
int main()
{
    constexpr float tol=2e-6f; const float nan=std::numeric_limits<float>::quiet_NaN();
    // Independent degree-valued expectations catch wrong signs, units and application location.
    for (float off : {0.f, 7.f, -7.f, 15.f, -90.f, 90.f}) {
        auto r=evaluate(off,nan,radians(2.f),false);
        assert(std::fabs(r.raw-radians(2.f+off))<tol);
        assert(r.body==r.raw && r.status==r.raw);
        assert(std::fabs(relative(radians(2.f),off)-radians(2.f-off))<tol);
        assert(std::fabs(minRelative(radians(-5.f),off)-radians(-5.f-off))<tol);
        assert(std::fabs(maxRelative(radians(30.f),off)-radians(30.f-off))<tol);
        auto direct=evaluate(off,radians(3.f),radians(2.f),false);
        assert(direct.raw==radians(3.f));
        assert(std::fabs(direct.status-radians(2.f+off))<tol);
        // Manual altitude/position publishes the same reconstruction, with existing body limits.
        auto manual=evaluate(off,nan,radians(2.f),true);
        assert(manual.body>=radians(-5.f) && manual.body<=radians(30.f));
    }
    assert(std::fabs(evaluate(15.f,nan,radians(-20.f),false).body-radians(-5.f))<tol);
    assert(std::fabs(evaluate(15.f,nan,radians(15.f),false).body-radians(30.f))<tol);
    for (float invalid : {nan, INFINITY, -INFINITY}) {
        auto r=evaluate(15.f,nan,invalid,false);
        assert(!std::isfinite(r.raw) && r.body==0.f && !std::isfinite(r.status));
        auto direct=evaluate(15.f,radians(3.f),invalid,false);
        assert(direct.body==radians(3.f));
        auto fallback=evaluate(15.f,invalid,radians(2.f),false);
        assert(std::fabs(fallback.raw-radians(17.f))<tol);
    }
    // There is no added trim state: changing offsets uses the current parameter value immediately.
    assert(evaluate(0.f,nan,0.f,false).body==0.f);
    assert(evaluate(15.f,nan,0.f,false).body==radians(15.f));
    assert(evaluate(0.f,nan,0.f,false).body==0.f);
    Mode mode{}; Vehicle fixed{2,false}, rotary{1,false}, transition{1,true};
    assert(!active(mode,fixed,false));
    for (int flag=0;flag<5;++flag) {
        Mode m{};
        if(flag==0)m.flag_control_position_enabled=true;
        if(flag==1)m.flag_control_velocity_enabled=true;
        if(flag==2)m.flag_control_acceleration_enabled=true;
        if(flag==3)m.flag_control_altitude_enabled=true;
        if(flag==4)m.flag_control_climb_rate_enabled=true;
        for(bool manual : {false,true})for(bool automatic : {false,true}) {
            m.flag_control_manual_enabled=manual; m.flag_control_auto_enabled=automatic;
            assert(active(m,fixed,false) && active(m,transition,false));
            assert(!active(m,rotary,false) && !active(m,fixed,true));
        }
    }
    std::cout << "PITCH_TRIM_HOST_CASES=PASS\nFW_PSP_OFF_HISTORICAL_SEMANTIC_PARITY=PASS\n";
}
'''
    substitutions = {'BODY': body, 'STATUS': status, 'FALLBACK': fallback, 'CLAMP': manual_clamp,
                     'SHOULD': should_run, 'GATE': gate, 'INPUT': inputs[0], 'MIN': inputs[9], 'MAX': inputs[10]}
    for key, value in substitutions.items():
        code = code.replace('@' + key + '@', value)
    with tempfile.TemporaryDirectory(prefix='pitch-trim-host-') as directory:
        cpp = pathlib.Path(directory) / 'pitch_trim.cpp'
        binary = pathlib.Path(directory) / 'pitch_trim'
        cpp.write_text(code)
        subprocess.run(['c++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror', '-Wno-unused-parameter',
                        '-D__PX4_POSIX', '-I.', '-Isrc', '-Isrc/lib', '-Isrc/lib/matrix', '-Isrc/include',
                        '-Iplatforms/common', '-Iplatforms/common/include',
                        '-Ibuild/px4_fmu-v6c_default', '-include', 'visibility.h', str(cpp), '-o', str(binary)],
                       cwd=ROOT, check=True)
        subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    main()

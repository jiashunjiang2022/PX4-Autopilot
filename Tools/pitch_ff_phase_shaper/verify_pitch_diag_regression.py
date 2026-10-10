#!/usr/bin/env python3
"""Run the unchanged diagnostic verifier on the child's Native projection.

The old verifier deliberately accepts only its original parent. A temporary
detached worktree keeps that check intact; the projected controller, logger,
and registration bytes come from the actual child and must equal frozen BASE.
"""
from pathlib import Path
import subprocess
import tempfile
from verify_pitch_ff_phase_shaper_source import BASE, CONTROLLERS, ROOT, blocks, parent, require


def main():
    (ROOT / 'build').mkdir(exist_ok=True)
    projection = Path(tempfile.mkdtemp(prefix='pff-native-projection-', dir=ROOT / 'build'))
    subprocess.run(['git', 'worktree', 'add', '--detach', str(projection), BASE], cwd=ROOT, check=True,
                   stdout=subprocess.DEVNULL)
    try:
        paths = CONTROLLERS + ['msg/CMakeLists.txt', 'src/modules/logger/logged_topics.cpp']
        for path in paths:
            data = (ROOT / path).read_bytes()
            if path in CONTROLLERS:
                data, _ = blocks(data)
            elif path == 'msg/CMakeLists.txt':
                data = data.replace(b'\tPitchFfShaperStatus.msg\n', b'')
            else:
                data = data.replace(b'\tadd_topic("pitch_ff_shaper_status", 10);\n', b'')
            require(data == parent(path), 'child Native projection equals BASE ' + path)
            (projection / path).write_bytes(data)
        # Every existing diagnostic file and tool must also be the child's bytes.
        for path in ['src/modules/fw_att_control/FixedwingAttitudeControl.cpp',
                     'src/modules/fw_att_control/FixedwingAttitudeControl.hpp',
                     'msg/PitchOuterDiag.msg', 'msg/PitchInnerDiag.msg'] + [
                         str(p.relative_to(ROOT)) for p in (ROOT / 'Tools/pitch_native_diagnostic').iterdir()
                         if p.is_file()]:
            data = (ROOT / path).read_bytes()
            require(data == parent(path), 'frozen child diagnostic bytes ' + path)
            (projection / path).write_bytes(data)
        subprocess.run(['python3', '-B', 'Tools/pitch_native_diagnostic/verify_pitch_diag_logging_only.py',
                        '--source-only'], cwd=projection, check=True)
        print('PITCH_NATIVE_DIAGNOSTIC_SOURCE_REGRESSION=PASS')
    finally:
        subprocess.run(['git', 'worktree', 'remove', str(projection)], cwd=ROOT, check=True)


if __name__ == '__main__':
    main()

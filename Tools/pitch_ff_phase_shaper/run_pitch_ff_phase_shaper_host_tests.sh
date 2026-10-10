#!/bin/sh
set -eu
cd "$(dirname "$0")/../.."
pff_test_bin=$(mktemp -t pitch-pff-host)
trap 'rm -f "$pff_test_bin"' EXIT
c++ -std=c++17 -O2 -Wall -Wextra -Werror -I. \
    Tools/pitch_ff_phase_shaper/pitch_ff_phase_shaper_host_test.cpp -o "$pff_test_bin"
"$pff_test_bin"
pff_generated=${PFF_PX4_BUILD:-build/px4_fmu-v6c_default}
if [ ! -f "$pff_generated/uORB/topics/rate_ctrl_status.h" ]; then
    pff_generated=/Users/jiangjiashun/PX4/PX4-roll-tbr-v2-pitch-diag-20261010/build/px4_fmu-v6c_default
fi
c++ -std=c++17 -O2 -Wall -Wextra -Werror -Wno-unused-parameter -D__PX4_POSIX \
    -I. -Isrc -Isrc/lib -Isrc/lib/matrix -Isrc/include \
    -Iplatforms/common -Iplatforms/common/include -I"$pff_generated" \
    -include visibility.h Tools/pitch_ff_phase_shaper/pitch_ff_output_chain_test.cpp \
    src/lib/rate_control/rate_control.cpp -o "$pff_test_bin"
"$pff_test_bin"

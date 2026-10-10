#!/bin/sh
set -eu
cd "$(dirname "$0")/../.."
pitch_diag_test_bin=$(mktemp -t pitch-diag-host)
trap 'rm -f "$pitch_diag_test_bin"' EXIT
c++ -std=c++17 -O2 -Wall -Wextra -Werror -Wno-unused-parameter -D__PX4_POSIX \
    -I. -Isrc -Isrc/lib -Isrc/lib/matrix -Isrc/include \
    -Iplatforms/common -Iplatforms/common/include -Ibuild/px4_fmu-v6c_default \
    -include visibility.h Tools/pitch_native_diagnostic/pitch_diag_host_test.cpp \
    src/modules/fw_att_control/fw_pitch_controller.cpp \
    src/lib/rate_control/rate_control.cpp -o "$pitch_diag_test_bin"
"$pitch_diag_test_bin"

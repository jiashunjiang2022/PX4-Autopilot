#!/bin/sh
set -eu
cd "$(dirname "$0")/../.."
task_test_bin=$(mktemp -t roll-tbr-host)
trap 'rm -f "$task_test_bin"' EXIT
c++ -std=c++17 -O2 -Wall -Wextra -Werror -Wno-unused-parameter -D__PX4_POSIX \
    -I. -Isrc -Isrc/lib -Isrc/lib/matrix -Isrc/include \
    -Iplatforms/common -Iplatforms/common/include -Ibuild/px4_fmu-v6c_default \
    -include visibility.h tools/roll_tbr_validation/roll_tbr_host_test.cpp \
    src/lib/rate_control/rate_control.cpp -o "$task_test_bin"
"$task_test_bin"

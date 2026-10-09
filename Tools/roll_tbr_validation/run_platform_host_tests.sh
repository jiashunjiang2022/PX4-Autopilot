#!/bin/sh
set -eu
cd "$(dirname "$0")/../.."
test -f build/px4_fmu-v6c_default/uORB/topics/fixed_wing_longitudinal_setpoint.h
task_test_bin=$(mktemp -t roll-platform-host)
trap 'rm -f "$task_test_bin"' EXIT
c++ -std=c++17 -Wall -Wextra -Werror -D__PX4_POSIX -I. -Iplatforms/common -Iplatforms/common/include \
    -Ibuild/px4_fmu-v6c_default -Isrc/include -include visibility.h \
    tools/roll_tbr_validation/platform_host_test.cpp \
    src/modules/wing_phase/WingPhaseMath.cpp -o "$task_test_bin"
"$task_test_bin"

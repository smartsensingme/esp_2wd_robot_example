#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
project_dir=$(pwd)
mkdir -p build/native
export TEST_BUILD_DIR="$project_dir/build/native"
export ROBOT_TEST_BUILD_DIR="$TEST_BUILD_DIR"
export ROBOT_PID_DIR="$project_dir/components/esp_pid"
if [ "$(uname -s)" = Darwin ] && [ -z "${SDKROOT:-}" ]; then
    SDKROOT=$(xcrun --sdk macosx --show-sdk-path)
    export SDKROOT
fi
sh components/esp_2wd_robot/tests/run.sh
sh components/esp_drv8833/tests/run.sh
sh components/esp_angle_lut/tests/run.sh
test_binary=$(mktemp "$TEST_BUILD_DIR/app-fault-policy.XXXXXX")
trap 'rm -f "$test_binary"' EXIT
${CC:-cc} -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined \
    -Icomponents/esp_2wd_robot/tests/support -Icomponents/esp_drv8833/include \
    -Imain tests/test_fault_policy.c -o "$test_binary"
"$test_binary"
sh components/esp-mt6701/test_apps/dual_i2c/test_native.sh

${CC:-cc} -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined \
    -Icomponents/esp_2wd_robot/tests/support -Icomponents/esp_drv8833/include \
    -Imain tests/test_board_mapping.c -o "$test_binary"
"$test_binary"

${CC:-cc} -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined \
    -Imain/board tests/test_calibration_button.c -o "$test_binary"
"$test_binary"

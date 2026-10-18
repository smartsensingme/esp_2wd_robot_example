#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
mkdir -p build/native
if [ "$(uname -s)" = Darwin ] && [ -z "${SDKROOT:-}" ]; then
    SDKROOT=$(xcrun --sdk macosx --show-sdk-path)
    export SDKROOT
fi
${CC:-cc} -std=c11 -O2 -Wall -Wextra -Werror \
    -Icomponents/kalman/include -Icomponents/esp_2wd_robot/src \
    components/kalman/src/kalman_cv.c \
    components/esp_2wd_robot/src/robot_velocity_estimator.c \
    components/esp_2wd_robot/tools/native/velocity_replay.c \
    -lm -o build/native/velocity-replay
printf '%s\n' "build/native/velocity-replay"

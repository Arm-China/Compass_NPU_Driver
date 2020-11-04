#!/usr/bin/env bash

# Copyright (C) 2022 Arm Technology (China) Co. Ltd. All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

PLATFORM=$1

X1_SIMULATOR_PATH=/project/ai/scratch01/AIPU_SIMULATOR/aipu_simulator_x1

export PATH=/project/ai/scratch01/AIPU_BSP/toolchain/gcc-linaro-7.5.0-2019.12-x86_64_aarch64-linux-gnu/bin:$PATH
export C_INCLUDE_PATH=./3rdparty
export CPLUS_INCLUDE_PATH=./3rdparty

rm -rf output
mkdir output

if [ "$PLATFORM"x != "arm64"x ] && [ "$PLATFORM"x != "x86"x ]; then
    echo "$PLATFORM is not valid platform"
    echo "./build.sh PLATFORM"
    echo "platform, chose one. [arm64, x86]"
    exit 1
fi

if [ "$PLATFORM"x == "arm64"x ]; then
    MAKE_FLAG="CXX=aarch64-linux-gnu-g++ CPPFLAGS+=-DZHOUYI_V12=1 BUILD_TARGET_PLATFORM=fpga"

else
    MAKE_FLAG="CXX=g++ CPPFLAGS+=-DSIMULATION=1 CPPFLAGS+=-DZHOUYI_V12=1 BUILD_TARGET_PLATFORM=sim"
    cp $X1_SIMULATOR_PATH ./simulator/
fi

echo $MAKE_FLAG
make $MAKE_FLAG > /dev/null

if [ $? != 0 ]; then
    echo "make error"
else
    echo "make success"
fi

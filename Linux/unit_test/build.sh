#!/usr/bin/env bash

# Copyright (C) 2022-2023 Arm Technology (China) Co. Ltd. All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

PLATFORM=$1
ARCH=$2

export PATH=/project/ai/scratch01/AIPU_BSP/toolchain/gcc-linaro-7.5.0-2019.12-x86_64_aarch64-linux-gnu/bin:$PATH
export COMPASS_DRV_RTENVAR_SIM_LPATH=/project/ai/scratch01/AIPU_SIMULATOR/lib
export C_INCLUDE_PATH=./3rdparty
export CPLUS_INCLUDE_PATH=./3rdparty

rm -rf output
mkdir output

if [ "$PLATFORM"x != "arm64"x ] && [ "$PLATFORM"x != "x86"x ]; then
    echo "$PLATFORM is not valid platform"
    echo "./build.sh PLATFORM ARCH"
    echo " arg1: platform, chose one. [arm64, x86]"
    echo " arg2: arch, chose one. [X1, X2]"
    exit 1
fi

if [ $ARCH != "X1" ] && [ $ARCH != "X2" ]; then
    echo "$ARCH is not valid arch"
    echo "./build.sh PLATFORM ARCH"
    echo " arg1: platform, chose one. [arm64, x86]"
    echo " arg2: arch, chose one. [X1, X2]"
    exit 1
fi
if [ "$PLATFORM"x == "arm64"x ]; then
    if [ "$ARCH" != "X2" ]; then
        MAKE_FLAG="CXX=aarch64-linux-gnu-g++ CPPFLAGS+=-DZHOUYI_V12=1 BUILD_TARGET_PLATFORM=fpga"
    else
        MAKE_FLAG="CXX=aarch64-linux-gnu-g++ CPPFLAGS+=-DZHOUYI_V3=1 BUILD_TARGET_PLATFORM=fpga"
    fi
else
    if [ "$ARCH" != "X2" ]; then
        MAKE_FLAG="CXX=g++ CPPFLAGS+=-DSIMULATION=1 CPPFLAGS+=-DZHOUYI_V12=1 BUILD_TARGET_PLATFORM=sim"
    else
        MAKE_FLAG="CXX=g++ CPPFLAGS+=-DSIMULATION=1 CPPFLAGS+=-DZHOUYI_V3=1 BUILD_TARGET_PLATFORM=sim"
    fi
    export CONFIG_DRV_BRENVAR_X2_SIM_LPATH=${COMPASS_DRV_RTENVAR_SIM_LPATH}
    export export COMPASS_DRV_BRENVAR_X2_SIM_LNAME=aipu_simulator_x2
    export LD_LIBRARY_PATH=${COMPASS_DRV_RTENVAR_SIM_LPATH}:$LD_LIBRARY_PATH
fi

echo $MAKE_FLAG
make $MAKE_FLAG > /dev/null

if [ $? != 0 ]; then
    echo "make error"
else
    echo "make success"
fi

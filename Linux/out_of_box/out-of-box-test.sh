#!/usr/bin/env bash

# Copyright (C) 2023-2024 Arm Technology (China) Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0

# build UMD and aipu_simulation_test
LOCAL_PATH=$(cd "$(dirname "$0")";pwd)

function test_run_help() {
    echo "=====================Driver Test Run Help================================"
    echo "Test Run Options:"
    echo "-h, --help        help"
    echo "-a, --target      AIPU target (optional, by default build X2_1204MP3, both X2/X3 are compatible with Z1/Z2/Z3/X1):"
    echo "                    - X2_1204/X2_1204MP3"
    echo "                    - X3_1304/X3_1304MP2"
    echo "-s, --simulator     Z1~X1 simulator version (optional, only when you try to run Z1/Z2/Z3/X1 benchmark):"
    echo "                    - Z1"
    echo "                    - Z2"
    echo "                    - Z3"
    echo "                    - X1"
    echo "-c, --case        case name, which should be 'out_of_box/benchmarks/<case>', and should include following files:"
    echo "                    aipu.bin/input0.bin/output.bin"
    echo "========================================================================="
    echo "  example: Attention: '-s' and '-a' can only specify one of them, also ensure case is corresponding AIPU target"
    echo "   - Z1~X1: ./out-of-box-test.sh -s <Z1/Z2/Z3/X1> -c <case>"
    echo "   - X2: ./out-of-box-test.sh -a <X2_1204/X2_1204MP3> -c <case>"
    echo "   - X3: ./out-of-box-test.sh -a <X3_1304/X3_1304MP2> -c <case>"
    exit 0
}

VERSION=v3
TARGET=X2_1204MP3
SIMULATOR=x1
CASE=

ARGS=`getopt -o ha:s:c: --long help,target,simulator,case: -n 'out-of-box-test.sh' -- "$@"`
eval set -- "${ARGS}"

while [ -n "$1" ]
do
    case "$1" in
     -h|--help)
         test_run_help
         ;;
     -a|--target)
         TARGET="$2"
         shift
         ;;
     -s|--simulator)
         SIMULATOR="$2"
         shift
         ;;
     -c|--case)
         CASE="$2"
         shift
         ;;
     --)
         shift ;
         break
         ;;
     *)
         break
    esac
    shift
done

if [[ "$CASE"x == x ]]; then
    echo "please provide case under out_of_box/benchmarks"
    exit -1
fi

if [[ "$TARGET"x == "X3_1304"x || "$TARGET"x == "X3_1304MP2"x ]]; then
    VERSION=v3_1
fi

pushd $LOCAL_PATH/../../AI610-SDK-1012*/Linux-driver/

# set the Linux Driver workspace. Since in this case UMD driver only needs the simulator environment, we set it to the simulator directory
sed -i 's|CONFIG_DRV_BTENVAR_BASE_DIR=<YOUR_WORKSPACE>|CONFIG_DRV_BTENVAR_BASE_DIR=`realpath ../../AI610-SDK-1002-r1p8-eac0`|g' bash_env_setup.sh
sed -i 's|CONFIG_DRV_BTENVAR_BSP_BASE_DIR=${CONFIG_DRV_BTENVAR_BASE_DIR}/AIPU_BSP|CONFIG_DRV_BTENVAR_BSP_BASE_DIR=${CONFIG_DRV_BTENVAR_BASE_DIR}|g' bash_env_setup.sh
sed -i 's|CONFIG_DRV_RTENVAR_SIM_BASE_PATH=${CONFIG_DRV_BTENVAR_BASE_DIR}/AIPU_SIMULATOR|CONFIG_DRV_RTENVAR_SIM_BASE_PATH=${CONFIG_DRV_BTENVAR_BASE_DIR}/simulator|g' bash_env_setup.sh
source bash_env_setup.sh
popd

bash $LOCAL_PATH/build_umd.sh $VERSION

# run test
bash $LOCAL_PATH/run.sh -a $TARGET -c $CASE -s $SIMULATOR

pushd $LOCAL_PATH/../../AI610-SDK-1012*/Linux-driver/
# recover the Linux Driver env
sed -i 's|CONFIG_DRV_BTENVAR_BASE_DIR=`realpath ../../AI610-SDK-1002-r1p8-eac0`|CONFIG_DRV_BTENVAR_BASE_DIR=<YOUR_WORKSPACE>|g' bash_env_setup.sh
sed -i 's|CONFIG_DRV_BTENVAR_BSP_BASE_DIR=${CONFIG_DRV_BTENVAR_BASE_DIR}|CONFIG_DRV_BTENVAR_BSP_BASE_DIR=${CONFIG_DRV_BTENVAR_BASE_DIR}/AIPU_BSP|g' bash_env_setup.sh
sed -i 's|CONFIG_DRV_RTENVAR_SIM_BASE_PATH=${CONFIG_DRV_BTENVAR_BASE_DIR}/simulator|CONFIG_DRV_RTENVAR_SIM_BASE_PATH=${CONFIG_DRV_BTENVAR_BASE_DIR}/AIPU_SIMULATOR|g' bash_env_setup.sh
popd
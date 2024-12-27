#!/usr/bin/env bash

# Copyright (C) 2023-2024 Arm Technology (China) Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0
LOCAL_PATH=$(cd "$(dirname "$0")";pwd)

TEST_APP=aipu_simulation_test
TEST_APP_DIR=./umd
BENCHMARK_CASE_DIR=./benchmarks
OUTPUT_DUMP_TOP_DIR=./output
TARGET=""
CASE=""
SIMULATOR=""

test_run_help() {
    echo "=====================Driver Test Run Help================================"
    echo "Test Run Options:"
    echo "-h, --help        help"
    echo "-c, --case        benchmark case(s) to run (under out_of_box/benchmarks)"
    echo "-s, --simulator   Z1/Z2/Z3/X1 simulator version (optional, only when you try to run Z1/Z2/Z3/X1 benchmark):"
    echo "-a, --target      X2_1204MP3/X3_1304MP2 (ignore for aipu v1/v2)"
    echo "========================================================================="
    echo "usage: put case into 'bechmarks' folder, Attention: '-s' and '-a' can only specify one of them"
    echo "  - Z1~X1: ./run.sh -s <Z1/Z2/Z3/X1> -c <case>"
    echo "  - X2: ./run.sh -a <X2_1204/X2_1204MP3> -c <case>"
    echo "  - X3: ./run.sh -a <X3_1304/X32_1304MP2> -c <case>"
    exit 0
}

if [ $# = 0 ]; then
    test_run_help
fi

ARGS=`getopt -o hs:c:a: --long help,case,target: -n 'run.sh' -- "$@"`
eval set -- "${ARGS}"

while [ -n "$1" ]
do
    case "$1" in
     -h|--help)
         test_run_help
         ;;
     -c|--case)
         CASE="$2"
         shift
         ;;
     -s|--simulator)
         SIMULATOR="$2"
         shift
         ;;
     -a|--target)
         TARGET="$2"
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

EXECUTABLE_SIMULATOR=$COMPASS_DRV_RTENVAR_X1_SIMULATOR
if [[ "$SIMULATOR"x != x ]]; then
    SIMULATOR=`echo "$SIMULATOR" | tr '[A-Z]' '[a-z]'`
    if [[ "$SIMULATOR"x == "z1"x ]]; then
        EXECUTABLE_SIMULATOR=$COMPASS_DRV_RTENVAR_Z1_SIMULATOR
    elif [[ "$SIMULATOR"x == "z2"x ]]; then
        EXECUTABLE_SIMULATOR=$COMPASS_DRV_RTENVAR_Z2_SIMULATOR
    elif [[ "$SIMULATOR"x == "z3"x ]]; then
        EXECUTABLE_SIMULATOR=$COMPASS_DRV_RTENVAR_Z3_SIMULATOR
    fi
fi

if [ ! -d $TEST_APP_DIR ] && \
   [ ! -e $TEST_APP_DIR/$TEST_APP ]; then
   echo "[TEST RUN ERROR] Test binary dir $TEST_APP_DIR or binary $TEST_APP_DIR/$TEST_APP not exist!"
   exit 1
fi

if [ -d "${BENCHMARK_CASE_DIR}/${CASE}" ]; then
    OUTPUT_DIR="${OUTPUT_DUMP_TOP_DIR}/${CASE}"
else
    echo "[TEST RUN ERROR] Case binary dir ${BENCHMARK_CASE_DIR}/${CASE} not exist!"
    echo "[TEST RUN ERROR] Please use "-c" to specify a valid test case!"
    exit 1
fi

export LD_LIBRARY_PATH=${LOCAL_PATH}/umd:${CONFIG_DRV_BRENVAR_SIM_LPATH}:$LD_LIBRARY_PATH
mkdir -p ./${OUTPUT_DIR}

ARGS="--bin=${BENCHMARK_CASE_DIR}/${CASE}/aipu.bin \
    --idata=${BENCHMARK_CASE_DIR}/${CASE}/input0.bin \
    --check=${BENCHMARK_CASE_DIR}/${CASE}/output.bin \
    --dump_dir=${OUTPUT_DIR}"
SIM_ARGS="--sim=${EXECUTABLE_SIMULATOR} --cfg_dir=./"

if [[ "$TARGET"x != x && "$SIMULATOR" == x ]]; then
    ARGS="-a $TARGET $ARGS"
fi

echo $ARGS
echo -e "\033[7m[TEST RUN INFO] Run $TEST test with benchmark ${CASE}\033[0m"
echo "${TEST_APP_DIR}/${TEST_APP} $ARGS $SIM_ARGS"
${TEST_APP_DIR}/${TEST_APP} $ARGS $SIM_ARGS

ret=$?
if [ $ret != 0 ]; then
    SAVED_DIR=${OUTPUT_DIR}-fail-$(date "+%Y-%m-%d-%H-%M-%S")
else
    SAVED_DIR=${OUTPUT_DIR}-pass-$(date "+%Y-%m-%d-%H-%M-%S")
fi
mv $OUTPUT_DIR $SAVED_DIR
echo "[TEST RUN INFO] memory section dump files saved under: $SAVED_DIR"

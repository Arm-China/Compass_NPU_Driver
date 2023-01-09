#!/usr/bin/env bash

# Copyright (C) 2022 Arm Technology (China) Co. Ltd. All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

TEST_APP=aipu_simulation_test
TEST_APP_DIR=./umd
UMD_DIR=./umd
BENCHMARK_CASE_DIR=./benchmarks
OUTPUT_DUMP_TOP_DIR=./output
ARCH=""

#############################################################
#
# Configure correct simulator directory before running test!
#
#############################################################
SIMULATOR=../../../../AI502-SDK-1002-r0p0-eac0/simulator/bin/aipu_simulator_z2       #simulator path
SIMULATOR_SO_DIR=../../../../AI502-SDK-1002-r0p0-eac0/simulator/lib/    #simulator dynamic lib path
SIMULATOR=/project/ai/scratch01/AIPU_SIMULATOR/kun/bin/aipu_simulator_x1
SIMULATOR_SO_DIR=/project/ai/scratch01/AIPU_SIMULATOR/kun/lib

if [ ! -e $SIMULATOR -o ! -e $SIMULATOR_SO_DIR ]; then
    echo "Z1/Z2/Z3/X1: have to supply valid Simulator and simulator lib"
    echo "X2: only need to supply simulator lib"
    exit 1
fi

test_run_help() {
    echo "=====================Driver Test Run Help================================"
    echo "Test Run Options:"
    echo "-h, --help        help"
    echo "-c, --case        benchmark case(s) to run (use default searching path)"
    echo "-a, --arch        X2_1204/X2_1204MP3 (ignore for Z1/Z2/Z3/X1)"
    echo "========================================================================="
    echo "usage: put resnet benchmark in 'bechmark' folder"
    echo "  ./run.sh -c resnet or ./run.sh -a X2_1204/X2_1204MP3 -c resnet"
    exit 1
}

if [ $# = 0 ]; then
    test_run_help
fi

ARGS=`getopt -o hc:a: --long help,case,arch: -n 'run.sh' -- "$@"`
eval set -- "${ARGS}"

while [ -n "$1" ]
do
    case "$1" in
     -h|--help)
         test_run_help
         ;;
     -c|--case)
         case="$2"
         shift
         ;;
    -a|--arch)
         ARCH="$2"
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

if [ ! -d $TEST_APP_DIR ] && \
   [ ! -e $TEST_APP_DIR/$TEST_APP ]; then
   echo "[TEST RUN ERROR] Test binary dir $TEST_APP_DIR or binary $TEST_APP_DIR/$TEST_APP not exist!"
   exit 1
fi

if [ -d "${BENCHMARK_CASE_DIR}/${case}" ]; then
    OUTPUT_DIR="${OUTPUT_DUMP_TOP_DIR}/${case}"
else
    echo "[TEST RUN ERROR] Case binary dir ${BENCHMARK_CASE_DIR}/${case} not exist!"
    echo "[TEST RUN ERROR] Please use "-c" to specify a valid test case!"
    exit 1
fi

export LD_LIBRARY_PATH=${UMD_DIR}:${SIMULATOR_SO_DIR}:$LD_LIBRARY_PATH
mkdir -p ./${OUTPUT_DIR}

ARGS="--bin=${BENCHMARK_CASE_DIR}/${case}/aipu.bin \
    --idata=${BENCHMARK_CASE_DIR}/${case}/input0.bin \
    --check=${BENCHMARK_CASE_DIR}/${case}/output.bin \
    --dump_dir=${OUTPUT_DIR}"
SIM_ARGS="--sim=${SIMULATOR} --cfg_dir=./"

if [ "$ARCH"x != x ]; then
    ARGS="-a $ARCH $ARGS"
fi

echo -e "\033[7m[TEST RUN INFO] Run $TEST test with benchmark ${case}\033[0m"
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
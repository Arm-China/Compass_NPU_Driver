#!/usr/bin/env bash

# Copyright (C) 2022 Arm Technology (China) Co. Ltd. All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

BUILD_DEBUG_FLAG=release
if [ "$(uname -i)" == 'x86_64' ]; then
    TEST=simulation
else
    TEST=benchmark
fi
BUILD_TARGET_PLATFORM=no
TEST_APP=aipu_${TEST}_test
BENCHMARK_CASE_DIR=../../benchmarks/x2
OUTPUT_DUMP_TOP_DIR=./output
KMD_RELEASE_DIR=.
SIM_LOG_LEVEL=0
SIM_VERBOSE=""

test_run_help() {
    echo "==========================Driver Test Run Help============================"
    echo "Test Run Options:"
    echo "-h, --help        help"
    echo "-p, --platform    target platform (optional, only need to specify when
                              there are multiple platform dirs under
                              COMPASS_DRV_BRENVAR_ODIR_TOP)"
    echo "-t, --test        test case to run (by default simulation test)"
    echo "                    - simulation"
    echo "                    - dbg_simulation"
    echo "-c, --case        benchmark case dir which contains [aipu.bin]"
    echo "-d, --deubg       run debug version UMD"
    echo "-l, --log_level   simulator log level"
    echo "--verbose         simulator verbose"
    echo "=========================================================================="
    exit 1
}

if [ $# = 0 ]; then
    test_run_help
fi

ARGS=`getopt -o hp:t:c:dl: --long help,platform:,test:,case:,debug,log_level:,verbose -n 'run.sh' -- "$@"`
eval set -- "$ARGS"

while [ -n "$1" ]
do
    case "$1" in
     -h|--help)
         test_run_help
         ;;
     -p|--platform)
         BUILD_TARGET_PLATFORM="$2"
         shift
         ;;
     -t|--test)
         TEST="$2"
         TEST_APP=aipu_$2_test
         shift
         ;;
     -c|--case)
         CASE="$2"
         shift
         ;;
     -d|--debug)
         BUILD_DEBUG_FLAG=debug
         ;;
     -l|--log_level)
         SIM_LOG_LEVEL="$2"
         shift
         ;;
     --verbose)
         SIM_VERBOSE="--verbose"
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

if [ "$COMPASS_DRV_BRENVAR_SETUP_DONE"x != "yes"x ]; then
    echo -e "\033[31;1m[DRV ERROR]\033[0m please source env_setup.sh first before running"
    exit 1
fi

if [ "$BUILD_TARGET_PLATFORM"x = "no"x ]; then
    for dir in $(ls $COMPASS_DRV_BRENVAR_ODIR_TOP)
    do
        if [ -d $COMPASS_DRV_BRENVAR_ODIR_TOP/$dir ]; then
            BUILD_AIPU_DRV_ODIR=$COMPASS_DRV_BRENVAR_ODIR_TOP/$dir/$BUILD_DEBUG_FLAG
            break
        fi
    done
else
    BUILD_AIPU_DRV_ODIR=$COMPASS_DRV_BRENVAR_ODIR_TOP/$BUILD_TARGET_PLATFORM/$BUILD_DEBUG_FLAG
fi

if [ ! -e $BUILD_AIPU_DRV_ODIR/$TEST_APP ]; then
    echo -e "$COMPASS_DRV_BRENVAR_ERROR Test binary dir $BUILD_AIPU_DRV_ODIR or binary $BUILD_AIPU_DRV_ODIR/$TEST_APP not exist!"
    exit 2
fi

if [ -d $CASE ]; then
    BENCHMARK_CASE_DIR=$CASE
else
    BENCHMARK_CASE_DIR=$BENCHMARK_CASE_DIR/$CASE
fi

if [ -d "$BENCHMARK_CASE_DIR" ]; then
    OUTPUT_DIR="$OUTPUT_DUMP_TOP_DIR"
else
    echo -e "$COMPASS_DRV_BRENVAR_ERROR Case binary dir $BENCHMARK_CASE_DIR not exist!"
    echo -e "$COMPASS_DRV_BRENVAR_ERROR Please use "-c" to specify a valid test case!"
    exit 3
fi

export LD_LIBRARY_PATH=$BUILD_AIPU_DRV_ODIR:$LD_LIBRARY_PATH
mkdir -p $OUTPUT_DIR

if [ "$(uname -i)" != 'x86_64' ]; then
    if [ ! -e $BUILD_AIPU_DRV_ODIR/aipu.ko ]; then
        echo -e "$COMPASS_DRV_BRENVAR_ERROR .ko not found. Build KMD first before running this test!"
        exit 1
    else
        if [ ! -c /dev/aipu ]; then
            sudo insmod $BUILD_AIPU_DRV_ODIR/aipu.ko
        fi
    fi
fi

if [ ! -e "$BENCHMARK_CASE_DIR/aipu.bin" ]; then
    echo -e -e "$COMPASS_DRV_BRENVAR_ERROR no executable benchmark \033[1maipu.bin\033[0m found under $BENCHMARK_CASE_DIR"
    exit 4
fi

### find input.bin or input0.bin/input1.bin/.../inputn.bin, output.bin
if [ -e "$BENCHMARK_CASE_DIR/input$i.bin" ]; then
    if [ -e "$BENCHMARK_CASE_DIR/input.bin" ]; then
        IDATA_ARGS=$BENCHMARK_CASE_DIR/input.bin
    else
        echo -e "$COMPASS_DRV_RTENVAR_WARN no \033[32minput0.bin/input.bin\033[0m found under $BENCHMARK_CASE_DIR"
    fi
else
    i=0
    while [ -e "$BENCHMARK_CASE_DIR/input$i.bin" ]
    do
        if [ $i == 0 ]; then
            IDATA_ARGS=$BENCHMARK_CASE_DIR/input$i.bin
        else
            IDATA_ARGS=$IDATA_ARGS,$BENCHMARK_CASE_DIR/input$i.bin
        fi
        let i++
    done
fi

if [ ! -e "$BENCHMARK_CASE_DIR/output.bin" ]; then
    echo -e -e "$COMPASS_DRV_BRENVAR_ERROR no gt file \033[1output.bin\033[0m found under $BENCHMARK_CASE_DIR"
    exit 5
fi

### run test application with case binaries
ARGS="--bin=$BENCHMARK_CASE_DIR/aipu.bin --idata=$IDATA_ARGS --check=$BENCHMARK_CASE_DIR/output.bin \
    --dump_dir=${OUTPUT_DIR}"
if [ "$(uname -i)" == 'x86_64' ]; then
    ARGS="$ARGS --z1_sim=$COMPASS_DRV_RTENVAR_Z1_SIMULATOR \
        --z2_sim=$COMPASS_DRV_RTENVAR_Z2_SIMULATOR \
        --z3_sim=$COMPASS_DRV_RTENVAR_Z3_SIMULATOR \
        --x1_sim=$COMPASS_DRV_RTENVAR_X1_SIMULATOR \
        --log_level=$SIM_LOG_LEVEL $SIM_VERBOSE"
fi

echo -e -e "$COMPASS_DRV_BRENVAR_INFO run $TEST test (in $BUILD_AIPU_DRV_ODIR) with benchmark $CASE..."
$BUILD_AIPU_DRV_ODIR/$TEST_APP $ARGS

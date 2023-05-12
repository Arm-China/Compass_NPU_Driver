#!/usr/bin/env bash

# Copyright (C) 2022-2023 Arm Technology (China) Co. Ltd. All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

###
# CONFIG_DRV_*:  users outside ArmChina AI team should set those variables before using
# COMPASS_DRV_*: users outside ArmChina AI team should validate those variables depend on CONFIG values
#
# BTENVAR: Buildtime Environment Variables
# RTENVAR: Runtime Environment Variables
# BRENVAR: Buildtime & Runtime Environment Variables
###

#
# files depending hierarchies:
#
# /project/ai/scrach01
# |
# |-- AIPU_BSP
# |   |-- kernel
# |   |   |-- linux-5.11.18
# |   |
# |   |-- toolchain
# |   |   |-- gcc-linaro-7.5.0-2019.12-x86_64_aarch64-linux-gnu
# |   |
# |   `-- simulator
#         |
#         |-- bin
#         |   |-- aipu_simulator_x1
#         |   |-- aipu_simulator_x2
#         |   |-- aipu_simulator_z1
#         |   |-- aipu_simulator_z2
#         |   |-- aipu_simulator_z3
#         |
#         `-- lib
#             |-- libaipu_simulator_x1.so
#             |-- libaipu_simulator_x2.so
#             |-- libaipu_simulator_z1.so
#             |-- libaipu_simulator_z2.so
#             |-- libaipu_simulator_z3.so

# note:
# set the below environment variables according to real files hierarchies
#     CONFIG_DRV_BTENVAR_BASE_DIR
#     CONFIG_DRV_BTENVAR_BSP_BASE_DIR
#     CONFIG_DRV_RTENVAR_SIM_BASE_PATH

### configure kernel path for target platform:
###    CONFIG_DRV_BTENVAR_[platform]_[kernel major]_[kernel minor]_KPATH
export CONFIG_DRV_BTENVAR_BASE_DIR=/project/ai/scratch01
export CONFIG_DRV_BTENVAR_BSP_BASE_DIR=${CONFIG_DRV_BTENVAR_BASE_DIR}/AIPU_BSP
export CONFIG_DRV_RTENVAR_SIM_BASE_PATH=${CONFIG_DRV_BTENVAR_BSP_BASE_DIR}/simulator

export CONFIG_DRV_BRENVAR_X86_CLPATH=/arm/tools/gnu/gcc/7.3.0/rhe7-x86_64/lib64
export CONFIG_DRV_BTENVAR_CROSS_CXX_PATH=${CONFIG_DRV_BTENVAR_BSP_BASE_DIR}/toolchain/gcc-linaro-7.5.0-2019.12-x86_64_aarch64-linux-gnu/bin
export CONFIG_DRV_BTENVAR_JUNO_4_9_KPATH=${CONFIG_DRV_BTENVAR_BSP_BASE_DIR}/kernel/linux-4.9.168
export CONFIG_DRV_BTENVAR_JUNO_KPATH=${CONFIG_DRV_BTENVAR_BSP_BASE_DIR}/kernel/linux-5.11.18
export CONFIG_DRV_BTENVAR_6CG_KPATH=${CONFIG_DRV_BTENVAR_BSP_BASE_DIR}/kernel/linux-xlnx-armchina

# Android Configuration
export CONFIG_DRV_BTENVAR_ANDROID_NDK_PATH=${CONFIG_DRV_BTENVAR_BASE_DIR}/toolchain/build_env/android_ndk/android-ndk-r20b
export CONFIG_DRV_BTENVAR_ANDROID_CXX_PATH=${CONFIG_DRV_BTENVAR_ANDROID_NDK_PATH}/toolchains/llvm/prebuilt/linux-x86_64/bin
export CONFIG_DRV_BTENVAR_CROSS_CXX_ANDROID_PATH=${CONFIG_DRV_BTENVAR_BASE_DIR}/toolchain/build_env/aarch64-linux-android-4.9/bin
export CONFIG_DRV_BTENVAR_ANDROID_KPATH=${CONFIG_DRV_BTENVAR_BASE_DIR}/toolchain/linux-kernel/linux-android-4.14.59

### Toolchains
### add toolchain/kernel path of your supported platform(s)
# Target platform: android
export COMPASS_DRV_BTENVAR_ANDROID_CXX=clang++
export COMPASS_DRV_BTENVAR_ANDROID_AR=llvm-ar

# Target platform: x86
export COMPASS_DRV_BTENVAR_X86_CXX=g++
export COMPASS_DRV_BTENVAR_X86_AR=ar

# Target platform: arm64
export COMPASS_DRV_BTENVAR_CROSS_CXX=aarch64-linux-gnu-g++
export COMPASS_DRV_BTENVAR_CROSS_AR=aarch64-linux-gnu-ar
export COMPASS_DRV_BTENVAR_ARCH=arm64
export COMPASS_DRV_BTENVAR_CROSS_COMPILE=aarch64-linux-
export COMPASS_DRV_BTENVAR_CROSS_COMPILE_GNU=aarch64-linux-gnu-
export COMPASS_DRV_BTENVAR_CROSS_COMPILE_ANDROID=aarch64-linux-android-

### Simulation common part
export CONFIG_DRV_RTENVAR_SIM_PATH=${CONFIG_DRV_RTENVAR_SIM_BASE_PATH}/bin
export COMPASS_DRV_RTENVAR_SIM_LPATH=${CONFIG_DRV_RTENVAR_SIM_BASE_PATH}/lib
export LD_LIBRARY_PATH=${COMPASS_DRV_RTENVAR_SIM_LPATH}:$LD_LIBRARY_PATH
export PATH=${CONFIG_DRV_RTENVAR_SIM_PATH}:$PATH

### aipu v1/v2 Simulation
export COMPASS_DRV_RTENVAR_Z1_SIMULATOR=${CONFIG_DRV_RTENVAR_SIM_PATH}/aipu_simulator_z1
export COMPASS_DRV_RTENVAR_Z2_SIMULATOR=${CONFIG_DRV_RTENVAR_SIM_PATH}/aipu_simulator_z2
export COMPASS_DRV_RTENVAR_Z3_SIMULATOR=${CONFIG_DRV_RTENVAR_SIM_PATH}/aipu_simulator_z3
export COMPASS_DRV_RTENVAR_X1_SIMULATOR=${CONFIG_DRV_RTENVAR_SIM_PATH}/aipu_simulator_x1

### aipu v3 Simulation
export CONFIG_DRV_BRENVAR_X2_SIM_LPATH=${COMPASS_DRV_RTENVAR_SIM_LPATH}
export COMPASS_DRV_BRENVAR_X2_SIM_LNAME=aipu_simulator_x2

### You are not suggested to modify the following environment variables
# Driver src & build dir.
export COMPASS_DRV_BTENVAR_UMD_DIR=driver/umd
export COMPASS_DRV_BTENVAR_KMD_DIR=driver/kmd
export COMPASS_DRV_BTENVAR_TEST_DIR=./samples

export COMPASS_DRV_BRENVAR_ODIR_TOP=`pwd`/bin
export COMPASS_DRV_BTENVAR_UMD_BUILD_DIR=`pwd`/build/umd
export COMPASS_DRV_BTENVAR_KMD_BUILD_DIR=`pwd`/build/kmd
export COMPASS_DRV_BTENVAR_TEST_BUILD_DIR=`pwd`/build/samples

# Driver naming
export COMPASS_DRV_BTENVAR_UMD_V_MAJOR=5
export COMPASS_DRV_BTENVAR_UMD_V_MINOR=3.0
export COMPASS_DRV_BTENVAR_UMD_SO_NAME=libaipudrv.so
export COMPASS_DRV_BTENVAR_UMD_SO_NAME_MAJOR=${COMPASS_DRV_BTENVAR_UMD_SO_NAME}.${COMPASS_DRV_BTENVAR_UMD_V_MAJOR}
export COMPASS_DRV_BTENVAR_UMD_SO_NAME_FULL=${COMPASS_DRV_BTENVAR_UMD_SO_NAME_MAJOR}.${COMPASS_DRV_BTENVAR_UMD_V_MINOR}
export COMPASS_DRV_BTENVAR_UMD_A_NAME=libaipudrv.a
export COMPASS_DRV_BTENVAR_UMD_A_NAME_MAJOR=${COMPASS_DRV_BTENVAR_UMD_A_NAME}.${COMPASS_DRV_BTENVAR_UMD_V_MAJOR}
export COMPASS_DRV_BTENVAR_UMD_A_NAME_FULL=${COMPASS_DRV_BTENVAR_UMD_A_NAME_MAJOR}.${COMPASS_DRV_BTENVAR_UMD_V_MINOR}
export COMPASS_DRV_BTENVAR_KMD_VERSION=3.4.0

export COMPASS_DRV_BRENVAR_ERROR="\033[31;1m[DRV ERROR]\033[0m"
export COMPASS_DRV_BRENVAR_WARN="\033[31;1m[DRV WARN]\033[0m"
export COMPASS_DRV_BRENVAR_INFO="\033[32;1m[DRV INFO]\033[0m"

###
export COMPASS_DRV_BRENVAR_SETUP_DONE=yes

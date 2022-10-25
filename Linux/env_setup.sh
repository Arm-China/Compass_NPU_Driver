#!/usr/bin/env bash

# Copyright (C) 2022 Arm Technology (China) Co. Ltd. All rights reserved.
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

##############################################
#          1. Kernel/C Lib Paths             #
##############################################
# 1.1. Kernel source paths
setenv CONFIG_DRV_BTENVAR_JUNO_4_9_KPATH     /project/ai/scratch01/AIPU_BSP/kernel/linux-4.9.168
setenv CONFIG_DRV_BTENVAR_JUNO_KPATH         /project/ai/scratch01/AIPU_BSP/kernel/linux-5.11.18
setenv CONFIG_DRV_BTENVAR_6CG_KPATH          /project/ai/scratch01/AIPU_BSP/kernel/linux-xlnx-armchina
setenv CONFIG_DRV_BTENVAR_ANDROID_KPATH      /project/ai/scratch01/toolchain/linux-kernel/linux-android-4.14.59
# add kernel path(s) here for your target platform(s):
# setenv CONFIG_DRV_BTENVAR_[platform]_[kernel major]_[kernel minor]_KPATH XXX (kernel major/minor are optional)

# 1.2. C/C++ Lib paths
setenv CONFIG_DRV_BTENVAR_CROSS_CXX_PATH     /project/ai/scratch01/AIPU_BSP/toolchain/gcc-linaro-7.5.0-2019.12-x86_64_aarch64-linux-gnu/bin
setenv CONFIG_DRV_BTENVAR_ANDROID_NDK_PATH   /project/ai/scratch01/toolchain/build_env/android_ndk/android-ndk-r20b
setenv CONFIG_DRV_BTENVAR_ANDROID_CXX_PATH   ${CONFIG_DRV_BTENVAR_ANDROID_NDK_PATH}/toolchains/llvm/prebuilt/linux-x86_64/bin
# add C/C++ libs path(s) here for your platform(s)

# 1.3. Other paths
setenv CONFIG_DRV_RTENVAR_PY_INCD_PATH       /usr/local/python/3.5.2/include/python3.5m
setenv CONFIG_DRV_RTENVAR_NUMPY_INCD_PATH    /arm/tools/python/python/3.5.2/rhe7-x86_64/lib/python3.5/site-packages/numpy/core/include
# add other path(s) here for your platform(s)

##############################################
#          2. Toolchains                     #
##############################################
# 2.1. Target platform: android
setenv COMPASS_DRV_BTENVAR_ANDROID_CXX       clang++
setenv COMPASS_DRV_BTENVAR_ANDROID_AR        llvm-ar

# 2.2. Target platform: x86
setenv COMPASS_DRV_BTENVAR_X86_CXX           g++
setenv COMPASS_DRV_BTENVAR_X86_AR            ar

# 2.3. Target platform: arm64
setenv COMPASS_DRV_BTENVAR_CROSS_CXX         aarch64-linux-gnu-g++
setenv COMPASS_DRV_BTENVAR_CROSS_AR          aarch64-linux-gnu-ar
setenv COMPASS_DRV_BTENVAR_ARCH              arm64
setenv COMPASS_DRV_BTENVAR_CROSS_COMPILE     aarch64-linux-
setenv COMPASS_DRV_BTENVAR_CROSS_COMPILE_GNU aarch64-linux-gnu-

# Add toolchain(s) here for your supported platform(s)

##############################################
#          3. NPU Simulator Related          #
##############################################
# 3.1. Z1/Z2/Z3/X1 simulators
setenv CONFIG_DRV_RTENVAR_SIM_PATH           /project/ai/scratch01/AIPU_SIMULATOR
setenv COMPASS_DRV_RTENVAR_Z1_SIMULATOR      ${CONFIG_DRV_RTENVAR_SIM_PATH}/aipu_simulator_z1
setenv COMPASS_DRV_RTENVAR_Z2_SIMULATOR      ${CONFIG_DRV_RTENVAR_SIM_PATH}/aipu_simulator_z2
setenv COMPASS_DRV_RTENVAR_Z3_SIMULATOR      ${CONFIG_DRV_RTENVAR_SIM_PATH}/aipu_simulator_z3
setenv COMPASS_DRV_RTENVAR_X1_SIMULATOR      ${CONFIG_DRV_RTENVAR_SIM_PATH}/aipu_simulator_x1
setenv COMPASS_DRV_RTENVAR_SIM_LPATH         ${CONFIG_DRV_RTENVAR_SIM_PATH}/lib
setenv LD_LIBRARY_PATH                       ${COMPASS_DRV_RTENVAR_SIM_LPATH}:$LD_LIBRARY_PATH
setenv PATH                                  ${CONFIG_DRV_RTENVAR_SIM_PATH}/:$PATH

# 3.2. Z5 simulator
setenv CONFIG_DRV_BRENVAR_Z5_SIM_LPATH       /project/ai/zhouyi_compass/z5_common_lib/simulator/kun/lib
setenv COMPASS_DRV_BRENVAR_Z5_SIM_LNAME      aipu_simulator_x2
setenv LD_LIBRARY_PATH                       ${CONFIG_DRV_BRENVAR_Z5_SIM_LPATH}:$LD_LIBRARY_PATH

##############################################
#          4. Driver Internal                #
##############################################
### You are not suggested to modify the following environment variables.
# Driver src & build dir.
setenv COMPASS_DRV_BTENVAR_UMD_DIR           driver/umd
setenv COMPASS_DRV_BTENVAR_KMD_DIR           driver/kmd
setenv COMPASS_DRV_BTENVAR_TEST_DIR          ./samples
setenv COMPASS_DRV_BTENVAR_DEMO_DIR          ./out_of_box

setenv COMPASS_DRV_BRENVAR_ODIR_TOP          `pwd`/bin
setenv COMPASS_DRV_BTENVAR_UMD_BUILD_DIR     `pwd`/build/umd
setenv COMPASS_DRV_BTENVAR_KMD_BUILD_DIR     `pwd`/build/kmd
setenv COMPASS_DRV_BTENVAR_TEST_BUILD_DIR    `pwd`/build/samples/

# Driver naming
setenv COMPASS_DRV_BTENVAR_UMD_V_MAJOR       5
setenv COMPASS_DRV_BTENVAR_UMD_V_MINOR       1.0
setenv COMPASS_DRV_BTENVAR_UMD_SO_NAME       libaipudrv.so
setenv COMPASS_DRV_BTENVAR_UMD_SO_NAME_MAJOR ${COMPASS_DRV_BTENVAR_UMD_SO_NAME}.${COMPASS_DRV_BTENVAR_UMD_V_MAJOR}
setenv COMPASS_DRV_BTENVAR_UMD_SO_NAME_FULL  ${COMPASS_DRV_BTENVAR_UMD_SO_NAME_MAJOR}.${COMPASS_DRV_BTENVAR_UMD_V_MINOR}
setenv COMPASS_DRV_BTENVAR_UMD_A_NAME        libaipudrv.a
setenv COMPASS_DRV_BTENVAR_UMD_A_NAME_MAJOR  ${COMPASS_DRV_BTENVAR_UMD_A_NAME}.${COMPASS_DRV_BTENVAR_UMD_V_MAJOR}
setenv COMPASS_DRV_BTENVAR_UMD_A_NAME_FULL   ${COMPASS_DRV_BTENVAR_UMD_A_NAME_MAJOR}.${COMPASS_DRV_BTENVAR_UMD_V_MINOR}
setenv COMPASS_DRV_BTENVAR_KMD_VERSION       3.2.0

setenv COMPASS_DRV_BRENVAR_ERROR             "\033[31;1m[DRV ERROR]\033[0m"
setenv COMPASS_DRV_BRENVAR_WARN              "\033[31;1m[DRV WARN]\033[0m"
setenv COMPASS_DRV_BRENVAR_INFO              "\033[32;1m[DRV INFO]\033[0m"

setenv LD_LIBRARY_PATH                       ${PWD}/bin/sim/debug:${PWD}/bin/sim/release:$LD_LIBRARY_PATH

###
setenv COMPASS_DRV_BRENVAR_SETUP_DONE        yes

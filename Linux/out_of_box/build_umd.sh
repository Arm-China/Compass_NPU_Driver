#!/usr/bin/env bash

# Copyright (C) 2023-2024 Arm Technology (China) Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0
LOCAL_PATH=$(cd "$(dirname "$0")";pwd)

# make umd .so
VERSION=$1

if [ -d ./umd ]; then
	rm -fr ./umd
fi

pushd $LOCAL_PATH/../../AI610-SDK-1012*/Linux-driver/

# generate libaipudrv.so
./build_all.sh -p sim -v $VERSION
if [ $? -ne 0 ]; then
	echo -e "\033[32;1mCompile libaipudrv.so [fail] \033[0m"
	exit 1
else
	echo -e "\033[32;1mCompile libaipudrv.so [ok] \033[0m"
fi

mkdir -p $LOCAL_PATH/umd
cp bin/sim/release/libaipudrv.so.*.*.* $LOCAL_PATH/umd/libaipudrv.so
popd

pushd $LOCAL_PATH
# generate aipu_simulation_test
make
if [ $? -ne 0 ]; then
	echo -e "\033[32;1mCompile aipu_simulation_test [fail] \033[0m"
	exit 1
else
	echo -e "\033[32;1mCompile aipu_simulation_test [ok] \033[0m"
fi
popd

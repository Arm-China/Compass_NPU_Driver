#!/usr/bin/env bash

# Copyright (C) 2022 Arm Technology (China) Co. Ltd. All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

# make umd .so
if [ -d ./umd ]; then
	rm -fr ./umd
fi

cd ../
source bash_env_setup.sh
./build_all.sh -p sim -v x1
cd -
mkdir -p umd
mv ../bin/sim/release/libaipudrv.so.*.*.* ./umd/libaipudrv.so

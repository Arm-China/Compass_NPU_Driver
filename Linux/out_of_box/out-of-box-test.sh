#!/usr/bin/env bash

# Copyright (C) 2022-2023 Arm Technology (China) Co. Ltd. All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

# build UMD
bash build_umd.sh

# make app
make

# run test
bash run.sh -c resnet_50

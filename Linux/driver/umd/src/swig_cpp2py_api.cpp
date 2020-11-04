// Copyright (C) 2022 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  swig_cpp2py_api.cpp
 * @brief AIPU User Mode Driver (UMD) C++ to Python API implementation
 * @version 1.0
 */

#include "swig_cpp2py_api.hpp"

Aipu& OpenDevice()
{
    Aipu& aipu = Aipu::get_aipu();
    aipu.init_dev();
    return aipu;
}

void CloseDevice()
{
    Aipu& aipu = Aipu::get_aipu();
    aipu.deinit_dev();
}
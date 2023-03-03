// Copyright (C) 2022-2023 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0

#include <iostream>
#include <sys/stat.h>
#include <tr1/tuple>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <assert.h>
#include <string>
#include "doctest.h"
#include "standard_api.h"
#include "graph_base.h"
#include "device_base.h"
#include "memory_base.h"
#include "context.h"
#include "helper.h"
#include "parser_base.h"

using namespace aipudrv;
using namespace std;

class ContextTest
{
public:
    aipudrv::MainContext* p_ctx = nullptr;

    ContextTest()
    {
        p_ctx = new aipudrv::MainContext();
    }

    ~ContextTest()
    {
        if(p_ctx != nullptr)
        {
            delete p_ctx;
            p_ctx = NULL;
        }
    }

};


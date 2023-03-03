// Copyright (C) 2022-2023 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0

#define DOCTEST_CONFIG_IMPLEMENT
#include <cstdio>
#include <string>
#include "doctest.h"

using namespace std;

int main(int argc, char **argv) {
    doctest::Context context;
    context.applyCommandLine(argc, argv);

    int res = context.run(); // run doctest

    if (context.shouldExit())
    {
        // propagate the result of the tests
        return res;
    }

    return 0;
}

// Copyright (C) 2022 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  cmd_line_parsing.cpp
 * @brief AIPU UMD test implementation file: command line parsing
 */

#include <iostream>
#include <sstream>
#include <iomanip>
#include <getopt.h>
#include <string.h>
#include <stdlib.h>
#include "cmd_line_parsing.h"
#include "helper.h"
#include "dbg.hpp"

std::shared_ptr<SemOp> semOp_sp = nullptr;
std::mutex m_mtx;

static struct option opts[] = {
    { "bin", required_argument, NULL, 'b' },
    { "idata", required_argument, NULL, 'i' },
    { "check", required_argument, NULL, 'c' },
    { "dump_dir", required_argument, NULL, 'd' },
    { "z1_sim", optional_argument, NULL, 'z' },
    { "z2_sim", optional_argument, NULL, 'q' },
    { "z3_sim", optional_argument, NULL, 'k' },
    { "x1_sim", optional_argument, NULL, 'x' },
    { "x2_arch", required_argument, NULL, 'a' },
    { "log_level", optional_argument, NULL, 'l' },
    { "dump_opt", optional_argument, NULL, 'o' },
    { "verbose", optional_argument, NULL, 'v' },
    { NULL, 0, NULL, 0 }
};

void help(void)
{
    std::string help_info =
        "usage: ./test -z/q/k/x sim -b aipu.bin -i input0.bin,input1.bin -c output.bin -d ./output [-l 0-3] [-v]\n"
        "   -z/q/k/x: z1/2/3/X1 simulator path\n"
        "   -b: aipu.bin\n"
        "   -i: input bins\n"
        "   -c: output bin\n"
        "   -d: output data path\n"
        "   -a: x2 arch (X2_1204/X2_1204MP3)\n"
        "   -o: dump options for text/weight/in/out on board(hex form: ff)\n"
        "   -l: simulator log level(0-3)\n"
        "   -v: simulator verbose(0,1)\n";

    std::cout << help_info;
    exit(0);
}

int init_test_bench(int argc, char* argv[], cmd_opt_t* opt, const char* test_case)
{
    int ret = 0;
    extern char *optarg;
    int opt_idx = 0;
    int c = 0;
    char* temp = nullptr;
    char* dest = nullptr;
    uint32_t size = 0;
    std::stringstream dump_opt;

    if (nullptr == opt)
    {
        AIPU_ERR()("invalid null pointer!\n");
        return -1;
    }

    while (1)
    {
        c = getopt_long (argc, argv, "hs:C:b:i:c:d:a:s:z:q:k:x:o:l:v", opts, &opt_idx);
        if (-1 == c)
        {
            break;
        }

        switch (c)
        {
        case 0:
            if (opts[opt_idx].flag != 0)
            {
                break;
            }
            fprintf (stdout, "option %s", opts[opt_idx].name);
            if (optarg)
            {
                printf (" with arg %s", optarg);
            }
            printf ("\n");
            break;

        case 'b':
            strcpy(opt->bin_file_name, optarg);
            break;

        case 'i':
            strcpy(opt->inputs_file_name, optarg);
            break;

        case 'c':
            strcpy(opt->gt_file_name, optarg);
            break;

        case 'd':
            strcpy(opt->dump_dir, optarg);
            break;

        case 'a':
            opt->x2_arch_desc = optarg;
            break;

        case 'z':
            strcpy(opt->z1_simulator, optarg);
            break;

        case 'q':
            strcpy(opt->z2_simulator, optarg);
            break;

        case 'k':
            strcpy(opt->z3_simulator, optarg);
            break;

        case 'x':
            strcpy(opt->x1_simulator, optarg);
            break;

        case 'o':
            dump_opt << optarg;
            dump_opt >> std::hex >> opt->dump_opt;
            break;

        case 'l':
            opt->log_level_set = true;
            opt->log_level = atoi(optarg);
            break;

        case 'v':
            opt->verbose = true;
            break;

        case 'h':
            help();
            break;

        case '?':
            break;

        default:
            break;
        }
    }

    temp = strtok(opt->inputs_file_name, ",");
    opt->input_files.push_back(temp);
    while (temp)
    {
        temp = strtok(nullptr, ",");
        if (temp != nullptr)
        {
            opt->input_files.push_back(temp);
        }
    }

    for (uint32_t i = 0; i < opt->input_files.size(); i++)
    {
        ret = load_file_helper(opt->input_files[i].c_str(), &dest, &size);
        if (ret != 0)
        {
            goto finish;
        }
        opt->inputs.push_back(dest);
        opt->inputs_size.push_back(size);
    }

    ret = load_file_helper(opt->gt_file_name, &dest, &size);
    if (ret != 0)
    {
        goto finish;
    }
    opt->gt = dest;
    opt->gt_size = size;

    semOp_sp = std::make_shared<SemOp>();

finish:
    if (ret != 0)
    {
        deinit_test_bench(opt);
    }
    return ret;
}

int deinit_test_bench(cmd_opt_t* opt)
{
    if (opt == nullptr)
    {
        return 0;
    }

    for (uint32_t i = 0; i < opt->inputs.size(); i++)
    {
        unload_file_helper(opt->inputs[i]);
    }
    opt->input_files.clear();
    opt->inputs_size.clear();
    opt->inputs.clear();

    unload_file_helper(opt->gt);
    opt->gt_size = 0;
    return 0;
}
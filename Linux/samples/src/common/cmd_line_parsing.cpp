// Copyright (C) 2023-2025 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

/**
 * @file  cmd_line_parsing.cpp
 * @brief AIPU UMD test implementation file: command line parsing
 */

#include "cmd_line_parsing.h"

#include <getopt.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#include <iomanip>
#include <iostream>
#include <sstream>

#include "dbg.hpp"
#include "helper.h"

std::shared_ptr<SemOp> semOp_sp = nullptr;
std::mutex m_mtx;

enum CmdOption {
  OPTION_RESET = 1000,
  OPTION_WEIGHT_INDICES_GM,
  OPTION_WEIGHT_GM,
};

static struct option opts[] = {
    {"bin", required_argument, NULL, 'b'},
    {"idata", required_argument, NULL, 'i'},
    {"check", required_argument, NULL, 'c'},
    {"dump_dir", required_argument, NULL, 'd'},
    {"sim", required_argument, NULL, 's'},
    {"npu_arch_desc", required_argument, NULL, 'a'},
    {"log_level", required_argument, NULL, 'l'},
    {"verbose", required_argument, NULL, 'v'},
    {"time", required_argument, NULL, 't'},
    {"shape", required_argument, NULL, 'r'},
    {"weight_dir", required_argument, NULL, 'w'},
    {"loop_cnt", required_argument, NULL, 'n'},
    {"frame_cnt", required_argument, NULL, 'f'},
    {"graph_idx", required_argument, NULL, 'g'},
    {"perf_mode", required_argument, NULL, 'p'},
    {"reset", required_argument, NULL, OPTION_RESET},
    {"wt_indices", required_argument, NULL, OPTION_WEIGHT_INDICES_GM},
    {"put_weight_gm", no_argument, NULL, OPTION_WEIGHT_GM},
    {NULL, 0, NULL, 0}};

void help(void) {
  std::string help_info =
      "usage: ./test -s sim -b aipu.bin -i input0.bin,input1.bin -c output.bin "
      "-d ./output [-l 0-3] [-v] [-r] [-n] [-f]\n"
      "   -s, --sim <PATH>: aipu v1/v2 simulator path\n"
      "   -b, --bin <AIPU BIN>: aipu.bin\n"
      "   -i, --idata <INPUT BIN>: input bins\n"
      "   -c, --check <GT>: output bin\n"
      "   -d, --dump_dir <PATH>: dump path, once provided, samples will do a "
      "full dump\n"
      "   -a: [[deprecated]]: aipu target for >=v3, this option is deprecated "
      "now\n"
      "   -t, --time <MODE>: test flush or finish job time(flush | finish), "
      "only for basic_time_test\n"
      "   -l, --log_level <LEVEL>: simulator log level(0-3), default 1\n"
      "   -v, --verbose <NUM>: simulator verbose(0, 1), default 1\n"
      "   -r, --shape <SHAPE>: dynamic real input shape(eg: 1,480,640,3;if "
      "multi tensors, use'/' for isolation: 1,480,640,3/1,480,640,3)\n"
      "   -w, --weight_dir <PATH>: extra weight bin path,(note: weight bin "
      "name is like extra_weight_{0-9}.bin)\n"
      "   -n, --loop_cnt <LOOP>: outer loop counter, represents number of new "
      "contexts/loading graphs/jobs, only affects some samples\n"
      "   -f, --frame_cnt <FRAME>: inner frame counter, represents frame "
      "counter of each context/graph/job\n"
      "   -g, --graph_idx <INDEX>: idx of runnig graph, only for shared "
      "weight, if not provided, run all graph\n"
      "   -p, --perf_mode <MODE>: perf mode, only for "
      "simulation_test&benchmark_test, attention: \n"
      "       1.only valid when aipu.bin enables profiler\n"
      "       2.v3_2 simulator enables profile, its' results may not match\n"
      "       3.simulator 0:none,1:fast,2:eval,3:idu,4:probe,5:slow; hardawre "
      "will do profiling when mode is non-zero\n"
      "   --reset <MODE>: npu reset, 0:hw reset, 1:sw reset\n"
      "   --wt_indices <INDEX0,INDEX1...>: weight indices(eg: 2,5,6,8,10,19)\n"
      "   --put_weight_gm: put whole weight to GM\n";

  AIPU_INFO()("%s", help_info.c_str());
  exit(0);
}

int init_test_bench(int argc, char *argv[], cmd_opt_t *opt,
                    const char *test_case) {
  int ret = 0;
  int opt_idx = 0;
  int c = 0;
  char *temp = nullptr;
  char *optarg_bkup = nullptr;
  char *dest = nullptr;
  uint32_t size = 0;

  if (nullptr == opt) {
    AIPU_ERR()("invalid null pointer!\n");
    return -1;
  }

  while (1) {
    c = getopt_long(argc, argv,
                    "hs:b:i:c:d:a:s:z:q:k:o:l:t:r:w:n:m:f:g:v:p:", opts,
                    &opt_idx);
    if (-1 == c)
      break;

    if (optarg)
      optarg_bkup = strdup(optarg);

    switch (c) {
    case 0:
      if (opts[opt_idx].flag != 0)
        break;

      AIPU_INFO()("option %s", opts[opt_idx].name);
      if (optarg)
        AIPU_INFO()(" with arg %s", optarg);
      AIPU_INFO()("\n");

      break;

    case 'b':
      temp = strtok(optarg_bkup, ",");
      opt->bin_files.push_back(temp);
      while (temp) {
        temp = strtok(nullptr, ",");
        if (temp != nullptr)
          opt->bin_files.push_back(temp);
      }
      break;

    case 'i':
      temp = strtok(optarg_bkup, ",");
      opt->input_files.push_back(temp);
      while (temp) {
        temp = strtok(nullptr, ",");
        if (temp != nullptr)
          opt->input_files.push_back(temp);
      }

      for (uint32_t i = 0; i < opt->input_files.size(); i++) {
        ret = load_file_helper(opt->input_files[i].c_str(), &dest, &size);
        if (ret != 0)
          goto finish;

        opt->inputs.push_back(dest);
        opt->inputs_size.push_back(size);
      }
      break;

    case 'c':
      temp = strtok(optarg_bkup, ",");
      opt->gt_files.push_back(temp);
      while (temp) {
        temp = strtok(nullptr, ",");
        if (temp != nullptr)
          opt->gt_files.push_back(temp);
      }

      for (uint32_t i = 0; i < opt->gt_files.size(); i++) {
        ret = load_file_helper(opt->gt_files[i].c_str(), &dest, &size);
        if (ret != 0)
          goto finish;

        opt->gts.push_back(dest);
        opt->gts_size.push_back(size);
      }
      break;

    case 'd':
      strcpy(opt->dump_dir, optarg);
      struct stat info;
      if (stat(opt->dump_dir, &info) != 0) {
        if (mkdir(opt->dump_dir, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) != 0) {
          AIPU_ERR()("create directory %s failed\n", opt->dump_dir);
          goto finish;
        }
      }
      break;

    case 'a':
      opt->npu_arch_desc = optarg;
      break;

    case 's':
      strcpy(opt->simulator, optarg);
      break;

    case 'l':
      opt->log_level = atoi(optarg);
      break;

    case 'v':
      opt->verbose = atoi(optarg);
      break;

    case 't':
      if (!strncmp(optarg, "flush", 5))
        opt->flush_time = true;
      else
        opt->flush_time = false;
      break;

    case 'r':
      strcpy(opt->input_shape, optarg);
      break;

    case 'w':
      opt->extra_weight_dir = optarg;
      break;

    case 'n':
      opt->loop_cnt = atoi(optarg);
      break;

    case 'f':
      opt->frame_cnt = atoi(optarg);
      break;

    case 'g':
      opt->graph_idx = atoi(optarg);
      break;

    case 'p':
      opt->perf_mode = atoi(optarg);
      break;
    case 'm':
      opt->thread_num = atoi(optarg);
      break;

    case OPTION_RESET:
      opt->reset_type = atoi(optarg);
      break;

    case OPTION_WEIGHT_INDICES_GM:
      temp = strtok(optarg_bkup, ",");
      while (temp) {
        opt->wt_indices.push_back(atoi(temp));
        temp = strtok(nullptr, ",");
      }
      opt->wt_idxes_cnt = opt->wt_indices.size();
      break;

    case OPTION_WEIGHT_GM:
      opt->put_weight_gm = true;
      break;

    case 'h':
      help();
      break;

    case '?':
      break;

    default:
      break;
    }

    if (optarg_bkup) {
      free(optarg_bkup);
      optarg_bkup = nullptr;
    }
  }

  semOp_sp = std::make_shared<SemOp>();

finish:
  if (ret != 0)
    deinit_test_bench(opt);

  if (optarg_bkup != nullptr) {
    free(optarg_bkup);
    optarg_bkup = nullptr;
  }

  return ret;
}

int deinit_test_bench(cmd_opt_t *opt) {
  if (opt == nullptr)
    return 0;

  for (uint32_t i = 0; i < opt->inputs.size(); i++)
    unload_file_helper(opt->inputs[i]);

  opt->input_files.clear();
  opt->inputs_size.clear();
  opt->inputs.clear();

  for (uint32_t i = 0; i < opt->gts.size(); i++)
    unload_file_helper(opt->gts[i]);

  opt->gt_files.clear();
  opt->gts_size.clear();
  opt->gts.clear();
  return 0;
}
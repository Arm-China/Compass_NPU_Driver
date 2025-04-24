// Copyright (C) 2023-2024 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

/**
 * @file  cmd_line_parsing.h
 * @brief AIPU UMD test header file: command line parsing
 */
#ifndef _CMD_LINE_PARSING_H_
#define _CMD_LINE_PARSING_H_

#include <string>
#include <vector>

#define BUF_LEN 512

typedef struct cmd_opt {
  std::vector<std::string> bin_files;
  char input_shape[1024] = {'\0'};
  char dump_dir[BUF_LEN] = {'\0'};
  char simulator[BUF_LEN] = {'\0'};
  std::vector<std::string> input_files;
  std::vector<uint32_t> inputs_size;
  std::vector<char *> inputs;
  std::vector<std::string> gt_files;
  std::vector<uint32_t> gts_size;
  std::vector<char *> gts;
  std::string npu_arch_desc;
  uint32_t gt_size;
  bool log_level_set = false;
  uint32_t log_level;
  bool verbose = false;
  bool flush_time = false;
  std::string extra_weight_dir;
  uint32_t loop_cnt = 0;
  uint32_t frame_cnt = 0;
  int32_t graph_idx = -1;
} cmd_opt_t;

int init_test_bench(int argc, char *argv[], cmd_opt_t *opt,
                    const char *test_case);
int deinit_test_bench(cmd_opt_t *opt);

#endif /* _CMD_LINE_PARSING_H_ */
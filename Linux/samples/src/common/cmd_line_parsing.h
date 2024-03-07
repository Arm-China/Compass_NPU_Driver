// Copyright (C) 2023-2024 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  cmd_line_parsing.h
 * @brief AIPU UMD test header file: command line parsing
 */
#ifndef _CMD_LINE_PARSING_H_
#define _CMD_LINE_PARSING_H_

#include <vector>
#include <string>

#define BUF_LEN 512

typedef struct cmd_opt {
    char bin_file_name[4096];
    std::vector<std::string> bin_files;
    char inputs_file_name[1024];
    char gts_file_name[4096];
    char dump_dir[BUF_LEN];
    char simulator[BUF_LEN];
    std::vector<std::string> input_files;
    std::vector<uint32_t> inputs_size;
    std::vector<char*> inputs;
    std::vector<std::string> gt_files;
    std::vector<uint32_t> gts_size;
    std::vector<char*> gts;
    std::string npu_arch_desc;
    uint32_t gt_size;
    bool log_level_set = false;
    uint32_t log_level;
    uint32_t dump_opt;
    bool verbose = false;
    bool flush_time = false;
} cmd_opt_t;

int init_test_bench(int argc, char* argv[], cmd_opt_t* opt, const char* test_case);
int deinit_test_bench(cmd_opt_t* opt);

#endif /* _CMD_LINE_PARSING_H_ */
// Copyright (C) 2022-2023 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  main.cpp
 * @brief AIPU SDK Driver Out-of-box Example for aipu v1/v2/v3
 */

#include <stdio.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <getopt.h>
#include <sys/mman.h>
#include <iostream>
#include <string>
#include <cstring>
#include <errno.h>
#include <vector>
#include "standard_api.h"

using namespace std;

#define FNAME_MAX_LEN 4096

static struct option opts[] = {
    { "sim", required_argument, NULL, 's' },
    { "cfg_dir", required_argument, NULL, 'C' },
    { "bin", required_argument, NULL, 'b' },
    { "idata", required_argument, NULL, 'i' },
    { "check", required_argument, NULL, 'c' },
    { "dump_dir", required_argument, NULL, 'd' },
    { "arch", required_argument, NULL, 'a'},
    { "help", no_argument, NULL, 'h'},
    { NULL, 0, NULL, 0 }
};

static void help()
{
    string help_str =
    "-s: aipu v1/v2 simulator path\n"
    "-b: benchmark aipu.bin path\n"
    "-i: benchmark input.bin path\n"
    "-c: benchmark output.bin path\n"
    "-d: the result output dir\n"
    "-a: ARCH args for aipu v3, eg: X2_1204/X2_1204MP3\n"
    "usage1 for aipu v1/v2:\n"
    "   test -s /demo/sim/aipu_simulator_z1 -b /demo/benchmark/aipu.bin "
    "-i /demo/benchmark/input0.bin,/demo/benchmark/input1.bin -c /demo/benchmark/output.bin "
    "-d /demo/output(create folder firstly)\n"
    "usage2 for aipu v3:\n"
    "   test -a [X2_1204 | X2_1204MP3] -b /demo/benchmark/aipu.bin "
    "-i /demo/benchmark/input0.bin,/demo/benchmark/input1.bin -c /demo/benchmark/output.bin "
    "-d /demo/output(create folder firstly)\n";

    cout << help_str;
    exit(0);
}

static bool is_output_correct(volatile char* src1, char* src2, uint32_t cnt)
{
    for (uint32_t out_chr = 0; out_chr < cnt; out_chr++)
    {
        if (src1[out_chr] != src2[out_chr])
        {
            return false;
        }
    }
    return true;
}

static void* load_data_from_file(const char* fname, int* size)
{
    int fd = 0;
    struct stat finfo;
    void* start = nullptr;

    if ((nullptr == fname) || (nullptr == size))
    {
        goto finish;
    }

    if (stat(fname, &finfo) != 0)
    {
        goto finish;
    }

    fd = open(fname, O_RDONLY);
    if (fd <= 0)
    {
        fprintf(stderr, "open file failed: %s! (errno = %d)\n", fname, errno);
        goto finish;
    }

    start = mmap(nullptr, finfo.st_size, PROT_READ, MAP_PRIVATE, fd, 0);
    if (MAP_FAILED == start)
    {
        fprintf(stderr, "failed in mapping graph file: %s! (errno = %d)\n", fname, errno);
        goto finish;
    }

    /* success */
    *size = finfo.st_size;

finish:
    if (fd > 0)
    {
        close(fd);
    }
    return start;
}

int main(int argc, char* argv[])
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    int pass = 0, opt = 0, opt_idx = 0;
    extern char *optarg;
    string arch = ""; // X2_1204, X2_1204MP3
    char simulator[FNAME_MAX_LEN] = { 0 };
    char cfg_file_dir[FNAME_MAX_LEN] = { 0 };
    char bin_file_name[FNAME_MAX_LEN] = { 0 };
    char check_file_name[FNAME_MAX_LEN] = { 0 };
    char dump_dir[FNAME_MAX_LEN] = { 0 };
    vector<string> data_file_name_vec;
    aipu_create_job_cfg_t create_job_cfg = {0};

    aipu_ctx_handle_t* ctx = nullptr;
    const char* msg = nullptr;
    vector<pair<void *, int>> input_data;
    vector<aipu_tensor_desc_t> input_desc, output_desc;
    vector<char*> output_data;
    uint64_t graph_id = 0, job_id = 0;
    uint32_t input_cnt = 0, output_cnt = 0;
    void *in_data = nullptr, *check_data = nullptr;
    char *check_va = nullptr;
    volatile char* out_va = nullptr;
    int in_fsize = 0, check_fsize = 0, offset = 0, time_out = -1;
    uint32_t size = 0;
    bool chk_ret = false;

    /**
     * For compatibility and avoiding segfault issues in the future,
     * strongly suggest to memset the config struct to be zero because the structs
     * are updated time to time.
     */
    aipu_global_config_simulation_t sim_glb_config;
    memset(&sim_glb_config, 0, sizeof(sim_glb_config));
    aipu_job_config_simulation_t sim_job_config;
    memset(&sim_job_config, 0, sizeof(sim_job_config));
    aipu_job_config_dump_t mem_dump_config;
    memset(&mem_dump_config, 0, sizeof(mem_dump_config));

    while (1)
    {
        opt = getopt_long (argc, argv, "s:C:b:i:c:a:d:h", opts, &opt_idx);
        if (-1 == opt)
        {
            break;
        }

        switch (opt)
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
        case 'a':
            arch = optarg;
            break;

        case 's':
            strcpy(simulator, optarg);
            break;

        case 'C':
            strcpy(cfg_file_dir, optarg);
            break;

        case 'b':
            strcpy(bin_file_name, optarg);
            break;

        case 'i':
            data_file_name_vec.push_back(optarg);
            break;

        case 'c':
            strcpy(check_file_name, optarg);
            break;

        case 'd':
            strcpy(dump_dir, optarg);
            break;

        case 'h':
            help();
        case '?':
            break;

        default:
            break;
        }
    }

    if ((arch.empty() && (simulator[0] == 0)) || (cfg_file_dir[0] == 0)
        || (bin_file_name[0] == 0) || (check_file_name[0] == 0) || (dump_dir[0] == 0))
    {
        fprintf(stderr, "[TEST ERROR] need more options!\n");
        goto finish;
    }

    if (data_file_name_vec.size() == 0)
    {
        fprintf(stderr, "[TEST ERROR] need input bins!\n");
        goto finish;
    }

    /**
     * load input data file as input tensor
     */
    for (uint32_t i = 0; i < data_file_name_vec.size(); i++)
    {
        in_data = load_data_from_file(data_file_name_vec[i].c_str(), &in_fsize);
        if (in_data == nullptr)
        {
            fprintf(stderr, "[TEST ERROR] load_data_from_file fail\n");
            goto finish;
        }

        input_data.push_back(make_pair(in_data, in_fsize));
    }

    check_data = load_data_from_file(check_file_name, &check_fsize);
    if (check_data == nullptr)
    {
        fprintf(stderr, "[TEST ERROR] load_data_from_file\n");
        goto finish;
    }

    ret = aipu_init_context(&ctx);
    if (ret != AIPU_STATUS_SUCCESS)
    {
        aipu_get_error_message(ctx, ret, &msg);
        fprintf(stderr, "[TEST ERROR] AIPU_init_ctx: %s\n", msg);
        goto finish;
    }

    sim_glb_config.z1_simulator = simulator;
    sim_glb_config.z2_simulator = simulator;
    sim_glb_config.z3_simulator = simulator;
    sim_glb_config.x1_simulator = simulator;
    sim_job_config.data_dir     = dump_dir;
    if (!arch.empty())
        sim_glb_config.x2_arch_desc = arch.c_str();

    ret = aipu_config_global(ctx, AIPU_CONFIG_TYPE_SIMULATION, &sim_glb_config);
    if (ret != AIPU_STATUS_SUCCESS)
    {
        aipu_get_error_message(ctx, ret, &msg);
        fprintf(stderr, "[TEST ERROR] AIPU_config_simulation: %s\n", msg);
        goto deinit_ctx;
    }

    ret = aipu_load_graph(ctx, bin_file_name, &graph_id);
    if (ret != AIPU_STATUS_SUCCESS)
    {
        aipu_get_error_message(ctx, ret, &msg);
        fprintf(stderr, "[TEST ERROR] AIPU_load_graph_helper: %s\n", msg);
        goto deinit_ctx;
    }
    fprintf(stdout, "[TEST INFO] AIPU load graph successfully.\n");

    /**
     * use the below logic to get input tensor number,
     * for this case, there's only one input tensor.
     */
    ret = aipu_get_tensor_count(ctx, graph_id, AIPU_TENSOR_TYPE_INPUT, &input_cnt);
    if (ret != AIPU_STATUS_SUCCESS)
    {
        aipu_get_error_message(ctx, ret, &msg);
        fprintf(stderr, "[TEST ERROR] aipu_get_tensor_count: %s\n", msg);
        goto unload_graph;
    }
    fprintf(stdout, "[TEST INFO] aipu_get_tensor_count success: input cnt = %d\n", input_cnt);

    for (uint32_t i = 0; i < input_cnt; i++)
    {
        aipu_tensor_desc_t desc;
        ret = aipu_get_tensor_descriptor(ctx, graph_id, AIPU_TENSOR_TYPE_INPUT, i, &desc);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(ctx, ret, &msg);
            fprintf(stderr, "[TEST ERROR] aipu_get_tensor_descriptor: %s\n", msg);
            goto unload_graph;
        }
        input_desc.push_back(desc);
    }

    if (input_desc.size() != input_data.size())
    {
        fprintf(stderr, "[TEST ERROR] input data files number doesn't match with input tensor number\n");
        goto unload_graph;
    }

    ret = aipu_get_tensor_count(ctx, graph_id, AIPU_TENSOR_TYPE_OUTPUT, &output_cnt);
    if (ret != AIPU_STATUS_SUCCESS)
    {
        aipu_get_error_message(ctx, ret, &msg);
        fprintf(stderr, "[TEST ERROR] aipu_get_tensor_count: %s\n", msg);
        goto unload_graph;
    }
    fprintf(stdout, "[TEST INFO] aipu_get_tensor_count success: output cnt = %d\n", output_cnt);

    /**
     * get output tensor number and allocate the corresponding output buffer
     */
    for (uint32_t i = 0; i < output_cnt; i++)
    {
        aipu_tensor_desc_t desc;
        ret = aipu_get_tensor_descriptor(ctx, graph_id, AIPU_TENSOR_TYPE_OUTPUT, i, &desc);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(ctx, ret, &msg);
            fprintf(stderr, "[TEST ERROR] aipu_get_tensor_descriptor: %s\n", msg);
            goto unload_graph;
        }
        output_desc.push_back(desc);
    }
    fprintf(stderr, "[TEST INFO] aipu_get_tensor_descriptor done\n");

    for (uint32_t i = 0; i < output_cnt; i++)
    {
        char* output = new char[output_desc[i].size];
        output_data.push_back(output);
    }

    create_job_cfg.partition_id = 0;
    create_job_cfg.qos_level = 0;
    ret = aipu_create_job(ctx, graph_id, &job_id, &create_job_cfg);
    if (ret != AIPU_STATUS_SUCCESS)
    {
        aipu_get_error_message(ctx, ret, &msg);
        fprintf(stderr, "[TEST ERROR] aipu_create_job: %s\n", msg);
        goto unload_graph;
    }
    fprintf(stdout, "[TEST INFO] aipu_create_job success\n");

    /**
     * config simulation for running on simulator
     */
    ret = aipu_config_job(ctx, job_id, AIPU_CONFIG_TYPE_SIMULATION, &sim_job_config);
    if (ret != AIPU_STATUS_SUCCESS)
    {
        aipu_get_error_message(ctx, ret, &msg);
        fprintf(stderr, "[TEST ERROR] aipu_config_job: %s\n", msg);
        goto clean_job;
    }
    fprintf(stdout, "[TEST INFO] set job simulation config success\n");

    for (uint32_t i = 0; i < input_desc.size(); i++)
    {
        ret = aipu_load_tensor(ctx, job_id, i, input_data[i].first);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(ctx, ret, &msg);
            fprintf(stderr, "[TEST ERROR] aipu_load_tensor: %s\n", msg);
            goto clean_job;
        }
        fprintf(stdout, "[TEST INFO] load input tensor %d from %s\n", 0, data_file_name_vec[i].c_str());
    }

    /**
     * trigger AIPU to run graph
     */
    ret = aipu_finish_job(ctx, job_id, time_out);
    if (ret != AIPU_STATUS_SUCCESS)
    {
        aipu_get_error_message(ctx, ret, &msg);
        fprintf(stderr, "[TEST ERROR] aipu_finish_job: %s\n", msg);
        pass = -1;
        goto clean_job;
    }
    fprintf(stdout, "[TEST INFO] aipu_finish_job success\n");

    /* loading output check data... */
    /* this test only support single output graph */
    /* for multi-output graphs, just load as this one by one */
    for (uint32_t i = 0; i < output_cnt; i++)
    {
        ret = aipu_get_tensor(ctx, job_id, AIPU_TENSOR_TYPE_OUTPUT, i, output_data[i]);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(ctx, ret, &msg);
            fprintf(stderr, "[TEST ERROR] aipu_get_tensor: %s\n", msg);
            goto clean_job;
        }
        fprintf(stdout, "[TEST INFO] get output tensor %u success (%u/%u)\n",
            i, i+1, output_cnt);
    }

    /**
     * check the runing result, compare it to gold data one by one
     */
    for (uint32_t id = 0; id < output_desc.size(); id++)
    {
        out_va = (volatile char*)output_data[id];
        check_va = (char*)((unsigned long)check_data + offset);
        size = output_desc[id].size;

        chk_ret = is_output_correct(out_va, check_va, size);
        if (chk_ret == true)
        {
            fprintf(stderr, "[TEST INFO] Test Result Check PASS! (%u/%lu)\n", id + 1,
                output_desc.size());
        }
        else
        {
            pass = -1;
            fprintf(stderr, "[TEST ERROR] Test Result Check FAILED! (%u/%lu)\n", id + 1,
                output_desc.size());
        }
        offset += size;
    }

    /**
     * clean job, unload graph, deinit context
     */
clean_job:
    ret = aipu_clean_job(ctx, job_id);
    if (ret != AIPU_STATUS_SUCCESS)
    {
        aipu_get_error_message(ctx, ret, &msg);
        fprintf(stderr, "[TEST ERROR] AIPU_clean_job: %s\n", msg);
        goto unload_graph;
    }

unload_graph:
    ret = aipu_unload_graph(ctx, graph_id);
    if (ret != AIPU_STATUS_SUCCESS)
    {
        aipu_get_error_message(ctx, ret, &msg);
        fprintf(stderr, "[TEST ERROR] aipu_unload_graph: %s\n", msg);
        goto deinit_ctx;
    }

deinit_ctx:
    ret = aipu_deinit_context(ctx);
    if (ret != AIPU_STATUS_SUCCESS)
    {
        aipu_get_error_message(ctx, ret, &msg);
        fprintf(stderr, "[TEST ERROR] aipu_deinit_ctx: %s\n", msg);
        goto finish;
    }

finish:
    if (AIPU_STATUS_SUCCESS != ret)
    {
        pass = -1;
    }
    for (auto in_data_pair : input_data)
    {
        munmap(in_data_pair.first, in_data_pair.second);
        input_data.clear();
    }
    if (check_data)
    {
        munmap(check_data, check_fsize);
    }
    return pass;
}

// Copyright (C) 2022-2023 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  main.cpp
 * @brief share some io buffers in process context
 *
 * @note
 *        1, run same model(eg: alexnet) with only one input tensor in loop-0 and loop-1
 *        2, loop == 0: mark one input tensor buffer of graph-0 as shared
 *           after aipu_create_job()
 *        3, loop == 1: assign the buffer marked shared in loop-0 to new graph-1
 *           after aipu_load_graph()
 */

#include <stdio.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <iostream>
#include <string.h>
#include <errno.h>
#include <vector>
#include <math.h>
#include <chrono>
#include <time.h>
#include <sys/time.h>
#include "standard_api.h"
#include "common/cmd_line_parsing.h"
#include "common/helper.h"
#include "common/dbg.hpp"

using namespace std;

/**
 * 1: run on simulator
 * 0: run on HW
 */
#define ON_SIMULATOR 0
#define ON_HW  1

#define GRAPH_CNT 50
#define PIPELINE_JOB_CNT 50

#define DUMP 1

int main(int argc, char* argv[])
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    aipu_ctx_handle_t* ctx;
    const char* msg = nullptr;
    uint32_t len;
    uint64_t graph_id[GRAPH_CNT];
    uint64_t job_id[GRAPH_CNT];
    vector< vector<char*> > job_outputs;
    int input_file_idx = 0;
    uint32_t input_cnt, output_cnt;
    vector<aipu_tensor_desc_t> input_desc;
    vector<char*> input_data;
    vector< vector<aipu_tensor_desc_t> > output_desc_vec;
    vector<aipu_tensor_desc_t> output_desc;
    aipu_create_job_cfg create_job_cfg = {0};
    int run_on_platform = ON_SIMULATOR;
    aipu_job_status_t status[GRAPH_CNT] = { AIPU_JOB_STATUS_NO_STATUS };
    struct timeval pre_t1, pre_t2;
    struct timeval create_t1, create_t2;
    struct timeval flush_t1, flush_t2;
    struct timeval finish_t1, finish_t2;
    uint32_t pre_total=0;
    uint32_t create_dur=0, create_total=0;
    uint32_t flush_dur=0, flush_total=0;
    uint32_t finish_dur=0, finish_total=0;
    cmd_opt_t opt;
    int pass = 0;
    bool flush_time = false;

    #if DUMP
    uint32_t cfg_types = 0;
    #endif

    AIPU_CRIT() << "usage: ./aipu_basic_time_test -t flush|finish -b aipu.bin -i input0.bin -c output.bin -d ./\n";

    if (access("/dev/aipu", F_OK) == 0)
        run_on_platform = ON_HW;

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

    if(init_test_bench(argc, argv, &opt, "basic_time_test"))
    {
        AIPU_ERR()("invalid command line options/args\n");
        goto finish;
    }

    flush_time = opt.flush_time;
    mem_dump_config.dump_dir = opt.dump_dir;
    if (opt.log_level_set)
        sim_glb_config.log_level = opt.log_level;
    else
        sim_glb_config.log_level = 2;

    sim_glb_config.verbose = opt.verbose;
    sim_glb_config.en_eval = true;
    sim_job_config.data_dir = opt.dump_dir;

    ret = aipu_init_context(&ctx);
    if (ret != AIPU_STATUS_SUCCESS)
    {
        aipu_get_error_message(ctx, ret, &msg);
        AIPU_ERR()("aipu_init_context: %s\n", msg);
        goto finish;
    }
    AIPU_INFO()("aipu_init_context success\n");

    if (run_on_platform == ON_SIMULATOR)
    {
        ret = aipu_config_global(ctx, AIPU_CONFIG_TYPE_SIMULATION, &sim_glb_config);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(ctx, ret, &msg);
            AIPU_ERR()("aipu_config_global: %s\n", msg);
            goto deinit_ctx;
        }
        AIPU_INFO()("set global simulation config success\n");
    }

    len = opt.bin_files.size();
    for (uint32_t loop=0; loop<len; loop++)
    {
        ret = aipu_load_graph(ctx, opt.bin_files[loop].c_str(), &graph_id[loop]);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(ctx, ret, &msg);
            AIPU_ERR()("aipu_load_graph_helper: %s (%s)\n",
                msg, opt.bin_files[loop].c_str());
            goto deinit_ctx;
        }
        AIPU_INFO()("\n");
        AIPU_INFO()("load %d: %s\n", loop, opt.bin_files[loop].c_str());

        ret = aipu_get_tensor_count(ctx, graph_id[loop], AIPU_TENSOR_TYPE_INPUT, &input_cnt);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(ctx, ret, &msg);
            AIPU_ERR()("aipu_get_tensor_count: %s\n", msg);
            goto unload_graph;
        }
        // AIPU_INFO()("aipu_get_tensor_count success: input cnt = %d\n", input_cnt);

        input_desc.clear();
        for (uint32_t i = 0; i < input_cnt; i++)
        {
            aipu_tensor_desc_t desc;
            ret = aipu_get_tensor_descriptor(ctx, graph_id[loop], AIPU_TENSOR_TYPE_INPUT, i, &desc);
            if (ret != AIPU_STATUS_SUCCESS)
            {
                aipu_get_error_message(ctx, ret, &msg);
                AIPU_ERR()("aipu_get_tensor_descriptor: %s\n", msg);
                goto unload_graph;
            }
            input_desc.push_back(desc);
        }

        ret = aipu_get_tensor_count(ctx, graph_id[loop], AIPU_TENSOR_TYPE_OUTPUT, &output_cnt);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(ctx, ret, &msg);
            AIPU_ERR()("aipu_get_tensor_count: %s\n", msg);
            goto unload_graph;
        }

        output_desc.clear();
        for (uint32_t i = 0; i < output_cnt; i++)
        {
            aipu_tensor_desc_t desc;
            ret = aipu_get_tensor_descriptor(ctx, graph_id[loop], AIPU_TENSOR_TYPE_OUTPUT, i, &desc);
            if (ret != AIPU_STATUS_SUCCESS)
            {
                aipu_get_error_message(ctx, ret, &msg);
                AIPU_ERR()("aipu_get_tensor_descriptor: %s\n", msg);
                goto unload_graph;
            }
            output_desc.push_back(desc);
        }
        output_desc_vec.push_back(output_desc);

        vector<char*> outputs;
        for (uint32_t i = 0; i < output_cnt; i++)
        {
            char* output = new char[(output_desc_vec[loop])[i].size];
            outputs.push_back(output);
        }
        job_outputs.push_back(outputs);

        gettimeofday(&create_t1, NULL);
        ret = aipu_create_job(ctx, graph_id[loop], &job_id[loop], &create_job_cfg);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(ctx, ret, &msg);
            AIPU_ERR()("aipu_create_job: %s\n", msg);
            goto unload_graph;
        }
        gettimeofday(&create_t2, NULL);

        create_dur = (create_t2.tv_sec - create_t1.tv_sec) * 1000000 + (create_t2.tv_usec-create_t1.tv_usec);
        AIPU_INFO()("create grpah<%d>, job<%d>: %d us\n", loop, loop, create_dur);
        create_total += create_dur;
        AIPU_INFO()("total %d graphs, create %d jobs: %d us\n", loop+1, loop+1, create_total);

        for (uint32_t i = 0; i < input_cnt; i++)
        {
            if (input_desc[i].size > opt.inputs_size[input_file_idx])
            {
                AIPU_ERR()("input file %s len 0x%x < input tensor %u size 0x%x\n",
                    opt.input_files[i].c_str(), opt.inputs_size[input_file_idx], i, input_desc[i].size);
                goto clean_job;
            }

            ret = aipu_load_tensor(ctx, job_id[loop], i, opt.inputs[input_file_idx]);
            if (ret != AIPU_STATUS_SUCCESS)
            {
                aipu_get_error_message(ctx, ret, &msg);
                AIPU_ERR()("aipu_load_tensor: %s\n", msg);
                goto clean_job;
            }
            AIPU_INFO()("load input tensor %d from %s (%u/%u)\n",
                i, opt.input_files[input_file_idx].c_str(), i+1, input_cnt);

            input_file_idx++;
        }
    }

    AIPU_INFO()("\n");
    AIPU_INFO()("----infer begin----\n");

    if (flush_time)
    {
        gettimeofday(&pre_t1, NULL);
        for (uint32_t job = 0; job < len; job++)
        {
            #if DUMP
            cfg_types = AIPU_JOB_CONFIG_TYPE_DUMP_TEXT  |
                AIPU_JOB_CONFIG_TYPE_DUMP_WEIGHT        |
                AIPU_JOB_CONFIG_TYPE_DUMP_RODATA        |
                AIPU_JOB_CONFIG_TYPE_DUMP_DESCRIPTOR    |
                AIPU_JOB_CONFIG_TYPE_DUMP_INPUT         |
                AIPU_JOB_CONFIG_TYPE_DUMP_OUTPUT        |
                AIPU_JOB_CONFIG_TYPE_DUMP_TCB_CHAIN     |
                AIPU_JOB_CONFIG_TYPE_DUMP_EMULATION;

            ret = aipu_config_job(ctx, job_id[job], cfg_types, &mem_dump_config);
            if (ret != AIPU_STATUS_SUCCESS)
            {
                aipu_get_error_message(ctx, ret, &msg);
                AIPU_ERR()("aipu_config_job: %s\n", msg);
                goto clean_job;
            }
            AIPU_INFO()("set dump config success\n");

            if (run_on_platform == ON_SIMULATOR)
            {
                ret = aipu_config_job(ctx, job_id[job], AIPU_CONFIG_TYPE_SIMULATION, &sim_job_config);
                if (ret != AIPU_STATUS_SUCCESS)
                {
                    aipu_get_error_message(ctx, ret, &msg);
                    AIPU_ERR()("aipu_config_job: %s\n", msg);
                    goto clean_job;
                }
                AIPU_INFO()("set job simulation config success\n");
            }
            #endif

            gettimeofday(&flush_t1, NULL);
            ret = aipu_flush_job(ctx, job_id[job]);
            gettimeofday(&flush_t2, NULL);
            flush_dur = (flush_t2.tv_sec - flush_t1.tv_sec) * 1000000 + (flush_t2.tv_usec - flush_t1.tv_usec);
            AIPU_INFO()("flush job <%d>: %d us\n", job, flush_dur);
            flush_total += flush_dur;
            AIPU_INFO()("flush <%d> jobs: %d us\n\n", job + 1, flush_total);
        }

        pre_total = 0;
        for (uint32_t job = 0; job < len; job++)
        {
            while ((status[job] != AIPU_JOB_STATUS_DONE) &&
                (status[job] != AIPU_JOB_STATUS_EXCEPTION))
            {
                ret = aipu_get_job_status(ctx, job_id[job], &status[job], 2000);
                if (ret == AIPU_STATUS_ERROR_TIMEOUT)
                {
                    AIPU_INFO()("flush job timeout\n");
                    continue;
                } else if (ret != AIPU_STATUS_SUCCESS)
                {
                    aipu_get_error_message(ctx, ret, &msg);
                    AIPU_ERR()("aipu_get_job_status: %s\n", msg);
                    pass = -1;
                    goto clean_job;
                }
                // AIPU_INFO()("job %lu still running...\n", job_id[job]);
            }

            if (status[job] == AIPU_JOB_STATUS_DONE)
            {
                AIPU_INFO()("job <%d> done\n", job);
            } else {
                fprintf(stdout, "[TEST ERROR] job %lu exception\n", job_id[job]);
                goto clean_job;
            }

            status[job] = AIPU_JOB_STATUS_NO_STATUS;

            /**
             * don't check result since it can incur time intervence.
             */
            #if 0
            for (uint32_t i = 0; i < output_desc_vec[job].size(); i++)
            {
                memset(job_outputs[job][i], 0, (output_desc_vec[job])[i].size);
                ret = aipu_get_tensor(ctx, job_id[job], AIPU_TENSOR_TYPE_OUTPUT, i, job_outputs[job][i]);
                if (ret != AIPU_STATUS_SUCCESS)
                {
                    aipu_get_error_message(ctx, ret, &msg);
                    AIPU_ERR()("aipu_get_tensor: %s\n", msg);
                    goto clean_job;
                }
                AIPU_INFO()("job %d: get output tensor %u success (%u/%u)\n",
                    job, i, i+1, output_cnt);
            }
            pass = check_result_helper(job_outputs[job], output_desc_vec[job], opt.gts[job], opt.gts_size[job]);
            #endif
        }

        gettimeofday(&pre_t2, NULL);
        pre_total = (pre_t2.tv_sec-pre_t1.tv_sec) * 1000000 + (pre_t2.tv_usec-pre_t1.tv_usec);
        AIPU_INFO()("total infer %d jobs: %d us\n", len, pre_total);
        AIPU_INFO()("time/job: %d us (%.3f ms)\n", pre_total/ len, float(pre_total) / len / 1000);
        AIPU_INFO()("\n");
    } else {
        for (uint32_t job = 0; job < len; job++)
        {
            #if DUMP
            cfg_types = AIPU_JOB_CONFIG_TYPE_DUMP_TEXT  |
                AIPU_JOB_CONFIG_TYPE_DUMP_WEIGHT        |
                AIPU_JOB_CONFIG_TYPE_DUMP_RODATA        |
                AIPU_JOB_CONFIG_TYPE_DUMP_DESCRIPTOR    |
                AIPU_JOB_CONFIG_TYPE_DUMP_INPUT         |
                AIPU_JOB_CONFIG_TYPE_DUMP_OUTPUT        |
                AIPU_JOB_CONFIG_TYPE_DUMP_TCB_CHAIN     |
                AIPU_JOB_CONFIG_TYPE_DUMP_EMULATION;
            ret = aipu_config_job(ctx, job_id[job], cfg_types, &mem_dump_config);
            if (ret != AIPU_STATUS_SUCCESS)
            {
                aipu_get_error_message(ctx, ret, &msg);
                AIPU_ERR()("aipu_config_job: %s\n", msg);
                goto clean_job;
            }
            AIPU_INFO()("set dump config success\n");

            if (run_on_platform == ON_SIMULATOR)
            {
                ret = aipu_config_job(ctx, job_id[job], AIPU_CONFIG_TYPE_SIMULATION, &sim_job_config);
                if (ret != AIPU_STATUS_SUCCESS)
                {
                    aipu_get_error_message(ctx, ret, &msg);
                    AIPU_ERR()("aipu_config_job: %s\n", msg);
                    goto clean_job;
                }
                AIPU_INFO()("set job simulation config success\n");
            }
            #endif

            gettimeofday(&finish_t1, NULL);
            ret = aipu_finish_job(ctx, job_id[job], -1);
            gettimeofday(&finish_t2, NULL);
            finish_dur = (finish_t2.tv_sec - finish_t1.tv_sec) * 1000000 + (finish_t2.tv_usec - finish_t1.tv_usec);
            AIPU_INFO()("finish job <%d>: %d us\n", job, finish_dur);
            finish_total += finish_dur;
            AIPU_INFO()("finish <%d> jobs: %d us\n\n", job + 1, finish_total);

            for (uint32_t i = 0; i < output_desc_vec[job].size(); i++)
            {
                memset(job_outputs[job][i], 0, (output_desc_vec[job])[i].size);
                ret = aipu_get_tensor(ctx, job_id[job], AIPU_TENSOR_TYPE_OUTPUT, i, job_outputs[job][i]);
                if (ret != AIPU_STATUS_SUCCESS)
                {
                    aipu_get_error_message(ctx, ret, &msg);
                    AIPU_ERR()("aipu_get_tensor: %s\n", msg);
                    goto clean_job;
                }
                AIPU_INFO()("job %d: get output tensor %u success (%u/%u)\n",
                    job, i, i+1, output_cnt);
            }
            pass = check_result_helper(job_outputs[job], output_desc_vec[job], opt.gts[job], opt.gts_size[job]);
        }
    }

    AIPU_INFO()("----infer end----\n\n");

    clean_job:
        for (uint32_t job = 0; job < len; job++)
        {
            for (uint32_t i = 0; i < job_outputs[job].size(); i++)
                delete job_outputs[job][i];

            ret = aipu_clean_job(ctx, job_id[job]);
            if (ret != AIPU_STATUS_SUCCESS)
            {
                aipu_get_error_message(ctx, ret, &msg);
                AIPU_ERR()("aipu_clean_job: %s\n", msg);
                goto unload_graph;
            }
        }
        AIPU_INFO()("aipu_clean_job success\n");

    unload_graph:
        for (uint32_t gid = 0; gid < len; gid++)
        {
            ret = aipu_unload_graph(ctx, graph_id[gid]);
            if (ret != AIPU_STATUS_SUCCESS)
            {
                aipu_get_error_message(ctx, ret, &msg);
                AIPU_ERR()("aipu_unload_graph: %s\n", msg);
                goto deinit_ctx;
            }
        }
        AIPU_INFO()("aipu_unload_graph success\n");

    deinit_ctx:
        ret = aipu_deinit_context(ctx);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(ctx, ret, &msg);
            AIPU_ERR()("aipu_deinit_ctx: %s\n", msg);
            goto finish;
        }
        AIPU_INFO()("aipu_deinit_ctx success\n");

    finish:
        if (AIPU_STATUS_SUCCESS != ret)
            pass = -1;

    deinit_test_bench(&opt);
    return pass;
}
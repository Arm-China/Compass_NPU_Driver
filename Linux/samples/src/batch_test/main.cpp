// Copyright (C) 2022-2023 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  main.cpp
 * @brief AIPU UMD test application: basic test on simulator
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
#include "standard_api.h"
#include "common/cmd_line_parsing.h"
#include "common/helper.h"
#include "common/dbg.hpp"

using namespace std;

#define MAX_BATCH 4

int main(int argc, char* argv[])
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    aipu_ctx_handle_t* ctx;
    const char* msg = nullptr;
    uint32_t cluster_cnt, core_cnt;
    uint64_t graph_id;
    uint32_t input_cnt, output_cnt;
    vector<aipu_tensor_desc_t> input_desc;
    vector<char*> input_data;
    vector<aipu_tensor_desc_t> output_desc;
    vector<char*> output_data[MAX_BATCH];
    vector<char*> gt;
    cmd_opt_t opt;
    uint32_t batch_loop_cnt = 2;
    int pass = 0, loop = 0, total_loop = 2;
    aipu_create_job_cfg create_job_cfg = {0};
    char **input_buf = nullptr, **output_buf[MAX_BATCH];
    uint32_t queue_id = 0;

    AIPU_CRIT() << "usage: ./aipu_batch_test -b aipu.bin -i input0.bin -c output.bin -d ./\n";

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

    if(init_test_bench(argc, argv, &opt, "batch_test"))
    {
        AIPU_ERR()("invalid command line options/args\n");
        goto finish;
    }

    mem_dump_config.dump_dir = opt.dump_dir;
    if (opt.log_level_set)
        sim_glb_config.log_level = opt.log_level;
    else
        sim_glb_config.log_level = 0;

    sim_glb_config.verbose = opt.verbose;
    sim_glb_config.en_eval = true;

    /* works for aipu v1/v2 simulations only */
    sim_glb_config.z1_simulator = opt.z1_simulator;
    sim_glb_config.z2_simulator = opt.z2_simulator;
    sim_glb_config.z3_simulator = opt.z3_simulator;
    sim_glb_config.x1_simulator = opt.x1_simulator;
    sim_job_config.data_dir     = opt.dump_dir;

    for (loop = 0; loop < total_loop; loop++)
    {
        input_desc.clear();
        output_desc.clear();
        AIPU_INFO() << "Total loop #" << loop;
        ret = aipu_init_context(&ctx);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(ctx, ret, &msg);
            AIPU_ERR()("aipu_init_context: %s\n", msg);
            goto finish;
        }

        AIPU_INFO()("aipu_init_context success\n");
        ret = aipu_config_global(ctx, AIPU_CONFIG_TYPE_SIMULATION, &sim_glb_config);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(ctx, ret, &msg);
            AIPU_ERR()("aipu_config_global: %s\n", msg);
            goto deinit_ctx;
        }
        AIPU_INFO()("set global simulation config success\n");

        ret = aipu_load_graph(ctx, opt.bin_file_name, &graph_id);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(ctx, ret, &msg);
            AIPU_ERR()("aipu_load_graph_helper: %s (%s)\n", msg, opt.bin_file_name);
            goto deinit_ctx;
        }
        AIPU_INFO()("aipu_load_graph_helper success: %s\n", opt.bin_file_name);

        ret = aipu_get_cluster_count(ctx, 0, &cluster_cnt);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(ctx, ret, &msg);
            AIPU_ERR()("aipu_get_cluster_count: %s (%s)\n",
                msg, opt.bin_file_name);
            goto unload_graph;
        }

        ret = aipu_get_core_count(ctx, 0, 0, &core_cnt);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(ctx, ret, &msg);
            AIPU_ERR()("aipu_get_core_count: %s (%s)\n",
                msg, opt.bin_file_name);
            goto unload_graph;
        }

        ret = aipu_get_tensor_count(ctx, graph_id, AIPU_TENSOR_TYPE_INPUT, &input_cnt);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(ctx, ret, &msg);
            AIPU_ERR()("aipu_get_tensor_count: %s\n", msg);
            goto unload_graph;
        }

        ret = aipu_get_tensor_count(ctx, graph_id, AIPU_TENSOR_TYPE_OUTPUT, &output_cnt);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(ctx, ret, &msg);
            AIPU_ERR()("aipu_get_tensor_count: %s\n", msg);
            goto unload_graph;
        }

        for (uint32_t i = 0; i < output_cnt; i++)
        {
            aipu_tensor_desc_t desc;
            ret = aipu_get_tensor_descriptor(ctx, graph_id, AIPU_TENSOR_TYPE_OUTPUT, i, &desc);
            if (ret != AIPU_STATUS_SUCCESS)
            {
                aipu_get_error_message(ctx, ret, &msg);
                AIPU_ERR()("aipu_get_tensor_descriptor: %s\n", msg);
                goto unload_graph;
            }
            output_desc.push_back(desc);
        }

        input_buf = new char* [input_cnt];
        for (uint32_t i = 0; i < MAX_BATCH; i++)
            output_buf[i] = new char* [output_cnt];

        for (uint32_t i = 0; i < input_cnt; i++)
            input_buf[i] = opt.inputs[i];

        for (uint32_t k = 0; k < MAX_BATCH; k++)
        {
            for (uint32_t i = 0; i < output_cnt; i++)
            {
                char* output = new char[output_desc[i].size];
                output_data[k].push_back(output);
                output_buf[k][i] = output;
            }
        }

        ret = aipu_create_batch_queue(ctx, graph_id, &queue_id);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(ctx, ret, &msg);
            AIPU_ERR()("aipu_create_batch_queue: %s\n", msg);
            goto unload_graph;
        }

        ret = aipu_config_batch_dump(ctx, graph_id, queue_id,
            AIPU_CONFIG_TYPE_SIMULATION, &mem_dump_config);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(ctx, ret, &msg);
            AIPU_ERR()("aipu_config_batch_dump: %s\n", msg);
            goto clean_batch_queue;
        }

        for (uint32_t round = 0; round < batch_loop_cnt; round++)
        {
            AIPU_INFO() << "Batch round #" << round;
            for (uint32_t i = 0; i < MAX_BATCH; i++)
            {
                ret = aipu_add_batch(ctx, graph_id, queue_id, input_buf, output_buf[i]);
                if (ret != AIPU_STATUS_SUCCESS)
                {
                    aipu_get_error_message(ctx, ret, &msg);
                    AIPU_ERR()("aipu_add_batch: %s\n", msg);
                    goto clean_batch_queue;
                }
            }

            ret = aipu_finish_batch(ctx, graph_id, queue_id, &create_job_cfg);
            if (ret != AIPU_STATUS_SUCCESS)
            {
                aipu_get_error_message(ctx, ret, &msg);
                AIPU_ERR()("aipu_finish_batch: %s\n", msg);
                goto clean_batch_queue;
            }

            for (uint32_t i = 0; i < MAX_BATCH; i++)
                pass = check_result_helper(output_data[i], output_desc, opt.gt, opt.gt_size);
        }

    clean_batch_queue:
        ret = aipu_clean_batch_queue(ctx, graph_id, queue_id);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(ctx, ret, &msg);
            AIPU_ERR()("aipu_clean_batch_queue: %s\n", msg);
            goto unload_graph;
        }

    unload_graph:
        ret = aipu_unload_graph(ctx, graph_id);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(ctx, ret, &msg);
            AIPU_ERR()("aipu_unload_graph: %s\n", msg);
            goto deinit_ctx;
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

        if (input_buf != nullptr)
            delete []input_buf;

        for (uint32_t k = 0; k < MAX_BATCH; k++)
        {
            for (uint32_t i = 0; i < output_data[k].size(); i++)
                delete []output_data[k][i];

            output_data[k].clear();
            if (output_buf != nullptr)
                delete []output_buf[k];
        }
    }

    deinit_test_bench(&opt);
    return pass;
}

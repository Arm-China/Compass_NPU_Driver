// Copyright (C) 2022-2023 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0

/**
 * @file  main.cpp
 * @brief aipu v3 AIPU UMD test application: for emulation test
 *        After running this test, dumped files can be used for emulation tests.
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

int main(int argc, char* argv[])
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    aipu_create_job_cfg_t create_job_cfg = {0};
    aipu_ctx_handle_t* ctx;
    const char* msg = nullptr;
    uint32_t cluster_cnt, core_cnt;
    uint64_t graph_id, job_id;
    uint32_t input_cnt, output_cnt;
    vector<aipu_tensor_desc_t> input_desc;
    vector<char*> input_data;
    vector<aipu_tensor_desc_t> output_desc;
    vector<char*> output_data;
    vector<char*> gt;
    cmd_opt_t opt;
    uint32_t frame_cnt = 1;
    int pass = 0;
    bool simulation_flag = false;

    aipu_global_config_simulation_t sim_glb_config;
    memset(&sim_glb_config, 0, sizeof(sim_glb_config));
    aipu_job_config_simulation_t sim_job_config;
    memset(&sim_job_config, 0, sizeof(sim_job_config));
    aipu_job_config_dump_t mem_dump_config;
    memset(&mem_dump_config, 0, sizeof(mem_dump_config));

    if(init_test_bench(argc, argv, &opt, "emulation_test"))
    {
        AIPU_ERR()("invalid command line options/args\n");
        goto finish;
    }

    /* Global configurations for simulation */
    if (opt.log_level_set)
    {
        sim_glb_config.log_level = opt.log_level;
    }
    else
    {
#if ((defined RTDEBUG) && (RTDEBUG == 1))
        sim_glb_config.log_level = 3;
#else
        sim_glb_config.log_level = 0;
#endif
    }
    sim_glb_config.verbose = opt.verbose;

    if (access("/dev/aipu", F_OK) != 0)
    {
        sim_glb_config.z1_simulator = opt.z1_simulator;
        sim_glb_config.z2_simulator = opt.z2_simulator;
        sim_glb_config.z3_simulator = opt.z3_simulator;
        sim_glb_config.x1_simulator = opt.x1_simulator;
        simulation_flag = true;
   }

    /* Job memory dump configurations for simulation */
    mem_dump_config.dump_dir = opt.dump_dir;
    sim_job_config.data_dir = opt.dump_dir;

    ret = aipu_init_context(&ctx);
    if (ret != AIPU_STATUS_SUCCESS)
    {
        aipu_get_error_message(ctx, ret, &msg);
        AIPU_ERR()("aipu_init_context: %s\n", msg);
        goto finish;
    }
    AIPU_INFO()("aipu_init_context success\n");

    if (simulation_flag)
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

    ret = aipu_load_graph(ctx, opt.bin_file_name, &graph_id);
    if (ret != AIPU_STATUS_SUCCESS)
    {
        aipu_get_error_message(ctx, ret, &msg);
        AIPU_ERR()("aipu_load_graph_helper: %s (%s)\n",
            msg, opt.bin_file_name);
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

    for (uint32_t i = 0; i < input_cnt; i++)
    {
        aipu_tensor_desc_t desc;
        ret = aipu_get_tensor_descriptor(ctx, graph_id, AIPU_TENSOR_TYPE_INPUT, i, &desc);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(ctx, ret, &msg);
            AIPU_ERR()("aipu_get_tensor_descriptor: %s\n", msg);
            goto unload_graph;
        }
        input_desc.push_back(desc);
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

    ret = aipu_create_job(ctx, graph_id, &job_id, &create_job_cfg);
    if (ret != AIPU_STATUS_SUCCESS)
    {
        aipu_get_error_message(ctx, ret, &msg);
        AIPU_ERR()("aipu_create_job: %s\n", msg);
        goto unload_graph;
    }
    AIPU_INFO()("aipu_create_job success\n");

    ret = aipu_config_job(ctx, job_id, AIPU_JOB_CONFIG_TYPE_DUMP_EMULATION, &mem_dump_config);
    if (ret != AIPU_STATUS_SUCCESS)
    {
        aipu_get_error_message(ctx, ret, &msg);
        AIPU_ERR()("aipu_config_job: %s\n", msg);
        goto clean_job;
    }
    AIPU_INFO()("set dump config success\n");

    if (simulation_flag)
    {
        ret = aipu_config_job(ctx, job_id, AIPU_CONFIG_TYPE_SIMULATION, &sim_job_config);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(ctx, ret, &msg);
            AIPU_ERR()("aipu_config_job: %s\n", msg);
            goto clean_job;
        }
        AIPU_INFO()("set job simulation config success\n");
    } else if (opt.dump_opt > 0) {
        ret = aipu_config_job(ctx, job_id, opt.dump_opt, &sim_job_config);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(ctx, ret, &msg);
            AIPU_ERR()("aipu_config_job: %s\n", msg);
            goto clean_job;
        }
        AIPU_INFO()("set job simulation config success\n");
    }

    if (opt.inputs.size() != input_cnt)
    {
        fprintf(stdout, "[TEST WARN] input file count (%u) != input tensor count (%u)\n",
            (uint32_t)opt.inputs.size(), input_cnt);
    }

    for (uint32_t i = 0; i < output_cnt; i++)
    {
        char* output = new char[output_desc[i].size];
        output_data.push_back(output);
    }

    for (uint32_t frame = 0; frame < frame_cnt; frame++)
    {
        AIPU_INFO()("Frame #%u\n", frame);
        for (uint32_t i = 0; i < min((uint32_t)opt.inputs.size(), input_cnt); i++)
        {
            if (input_desc[i].size > opt.inputs_size[i])
            {
                AIPU_ERR()("input file %s len 0x%x < input tensor %u size 0x%x\n",
                    opt.input_files[i].c_str(), opt.inputs_size[i], i, input_desc[i].size);
                goto clean_job;
            }
            ret = aipu_load_tensor(ctx, job_id, i, opt.inputs[i]);
            if (ret != AIPU_STATUS_SUCCESS)
            {
                aipu_get_error_message(ctx, ret, &msg);
                AIPU_ERR()("aipu_load_tensor: %s\n", msg);
                goto clean_job;
            }
            AIPU_INFO()("load input tensor %d from %s (%u/%u)\n",
                i, opt.input_files[i].c_str(), i+1, input_cnt);
        }

        ret = aipu_finish_job(ctx, job_id, -1);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(ctx, ret, &msg);
            AIPU_ERR()("aipu_finish_job: %s\n", msg);
            pass = -1;
            goto clean_job;
        }
        AIPU_INFO()("aipu_finish_job success\n");

        for (uint32_t i = 0; i < output_cnt; i++)
        {
            ret = aipu_get_tensor(ctx, job_id, AIPU_TENSOR_TYPE_OUTPUT, i, output_data[i]);
            if (ret != AIPU_STATUS_SUCCESS)
            {
                aipu_get_error_message(ctx, ret, &msg);
                AIPU_ERR()("aipu_get_tensor: %s\n", msg);
                goto clean_job;
            }
            AIPU_INFO()("get output tensor %u success (%u/%u)\n",
                i, i+1, output_cnt);
        }

        pass = check_result_helper(output_data, output_desc, opt.gt, opt.gt_size);
    }

clean_job:
    ret = aipu_clean_job(ctx, job_id);
    if (ret != AIPU_STATUS_SUCCESS)
    {
        aipu_get_error_message(ctx, ret, &msg);
        AIPU_ERR()("aipu_clean_job: %s\n", msg);
        goto unload_graph;
    }
    //AIPU_INFO()("aipu_clean_job success\n");

unload_graph:
    ret = aipu_unload_graph(ctx, graph_id);
    if (ret != AIPU_STATUS_SUCCESS)
    {
        aipu_get_error_message(ctx, ret, &msg);
        AIPU_ERR()("aipu_unload_graph: %s\n", msg);
        goto deinit_ctx;
    }
    //AIPU_INFO()("aipu_unload_graph success\n");

deinit_ctx:
    ret = aipu_deinit_context(ctx);
    if (ret != AIPU_STATUS_SUCCESS)
    {
        aipu_get_error_message(ctx, ret, &msg);
        AIPU_ERR()("aipu_deinit_ctx: %s\n", msg);
        goto finish;
    }
    //AIPU_INFO()("aipu_deinit_ctx success\n");

finish:
    if (AIPU_STATUS_SUCCESS != ret)
    {
        pass = -1;
    }
    for (uint32_t i = 0; i < output_data.size(); i++)
    {
        delete[] output_data[i];
    }
    deinit_test_bench(&opt);
    return pass;
}

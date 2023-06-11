// Copyright (C) 2022-2023 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0

#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <fstream>
#include <string.h>
#include <errno.h>
#include <vector>
#include <math.h>
#include <unistd.h>
#include "standard_api.h"
#include "common/cmd_line_parsing.h"
#include "common/helper.h"
#include "common/dbg.hpp"

using namespace std;

/**
 * @brief multiple threads demo
 *
 * @note
 *        non-pipeline: aipu_mthread_test -b aipu.bin -i input.bin -c output.bin -d ./output/
 *        pipeline: aipu_mthread_test -p -b aipu.bin -i input.bin -c output.bin -d ./output/
 *
 *        support running both on HW and Simulator, note that it has to insmod aipu.ko firstly
 *        on HW platform.
 */

/**
 * Max thread number
 */
#define THREAD_NUM 2

/**
 * 1: run on simulator
 * 0: run on HW
 */
#define ON_SIMULATOR 1
#define ON_HW  0

aipu_ctx_handle_t* ctx;
cmd_opt_t opt;
int g_pass = 0;
int run_on_platform = ON_SIMULATOR;
aipu_global_config_simulation_t sim_glb_config;

void non_pipeline()
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    aipu_create_job_cfg_t create_job_cfg = {0};
    const char* msg = nullptr;
    uint64_t graph_id, job_id;
    uint32_t input_cnt, output_cnt;
    vector<aipu_tensor_desc_t> input_desc;
    vector<char*> input_data;
    vector<aipu_tensor_desc_t> output_desc;
    vector<shared_ptr<char> >output_data_vec;
    vector<char*> gt;
    uint32_t frame_cnt = 2;
    uint64_t cfg_types = 0;
    std::string path;
    aipu_job_config_simulation_t tmp_sim_job_config = {0};
    aipu_job_config_dump_t tmp_mem_dump_config = {0};
    int pass = 0;

    AIPU_DBG() << "non_pipeline()";

    path = opt.dump_dir;
    path = path + "/" + std::to_string(gettid());
    help_create_dir(path.c_str());

    ret = aipu_load_graph(ctx, opt.bin_file_name, &graph_id);
    if (ret != AIPU_STATUS_SUCCESS)
    {
        aipu_get_error_message(ctx, ret, &msg);
        AIPU_ERR()("aipu_load_graph_helper: %s (%s)\n",
            msg, opt.bin_file_name);
        goto finish;
    }
    AIPU_INFO() << "Graph id= " << std::hex << graph_id;

    ret = aipu_get_tensor_count(ctx, graph_id, AIPU_TENSOR_TYPE_INPUT, &input_cnt);
    if (ret != AIPU_STATUS_SUCCESS)
    {
        aipu_get_error_message(ctx, ret, &msg);
        AIPU_ERR()("aipu_get_tensor_count: %s\n", msg);
        goto unload_graph;
    }
    //AIPU_INFO()("aipu_get_tensor_count success: input cnt = %d\n", input_cnt);

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
    //AIPU_INFO()("aipu_get_tensor_count success: output cnt = %d\n", output_cnt);

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
    //AIPU_ERR()("aipu_get_tensor_descriptor done\n");

    ret = aipu_create_job(ctx, graph_id, &job_id, &create_job_cfg);
    if (ret != AIPU_STATUS_SUCCESS)
    {
        aipu_get_error_message(ctx, ret, &msg);
        AIPU_ERR()("aipu_create_job: %s\n", msg);
        goto unload_graph;
    }
    AIPU_INFO()("aipu_create_job success\n");

    #if ((defined RTDEBUG) && (RTDEBUG == 1))
        cfg_types = AIPU_JOB_CONFIG_TYPE_DUMP_TEXT  |
            AIPU_JOB_CONFIG_TYPE_DUMP_WEIGHT        |
            AIPU_JOB_CONFIG_TYPE_DUMP_RODATA        |
            AIPU_JOB_CONFIG_TYPE_DUMP_DESCRIPTOR    |
            AIPU_JOB_CONFIG_TYPE_DUMP_INPUT         |
            AIPU_JOB_CONFIG_TYPE_DUMP_OUTPUT        |
            AIPU_JOB_CONFIG_TYPE_DUMP_TCB_CHAIN     |
            AIPU_JOB_CONFIG_TYPE_DUMP_EMULATION;
    #else
        cfg_types = AIPU_JOB_CONFIG_TYPE_DUMP_OUTPUT;
    #endif
    tmp_mem_dump_config.dump_dir = path.c_str();
    AIPU_INFO() << tmp_mem_dump_config.dump_dir;
    ret = aipu_config_job(ctx, job_id, cfg_types, &tmp_mem_dump_config);
    if (ret != AIPU_STATUS_SUCCESS)
    {
        aipu_get_error_message(ctx, ret, &msg);
        AIPU_ERR()("aipu_config_job: %s\n", msg);
        goto clean_job;
    }
    AIPU_INFO()("set dump config success\n");

    if (run_on_platform == ON_SIMULATOR)
    {
        tmp_sim_job_config.data_dir = path.c_str();
        ret = aipu_config_job(ctx, job_id, AIPU_CONFIG_TYPE_SIMULATION, &tmp_sim_job_config);
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
        AIPU_INFO()("[TEST WARN] input file count (%u) != input tensor count (%u)\n",
            (uint32_t)opt.inputs.size(), input_cnt);
    }

    for (uint32_t i = 0; i < output_cnt; i++)
    {
        auto deleter = [](char* p) { delete [] p; };
        shared_ptr<char> sp(new char[output_desc[i].size], deleter);
        output_data_vec.push_back(sp);
    }

    /* run with with multiple frames */
    for (uint32_t frame = 0; frame < frame_cnt; frame++)
    {
        AIPU_INFO()("Frame #%u\n", frame);
        for (uint32_t i = 0; i < min((uint32_t)opt.inputs.size(), input_cnt); i++)
        {
            if (input_desc[i].size > opt.inputs_size[i])
            {
                AIPU_ERR()("input file %s len 0x%x < input tensor %u size 0x%x\n",
                    opt.input_files[i].c_str(), opt.inputs_size[i], i, input_desc[i].size);
                pass = -1;
                goto clean_job;
            }
            ret = aipu_load_tensor(ctx, job_id, i, opt.inputs[i]);
            if (ret != AIPU_STATUS_SUCCESS)
            {
                aipu_get_error_message(ctx, ret, &msg);
                AIPU_ERR()("aipu_load_tensor: %s\n", msg);
                pass = -1;
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
            ret = aipu_get_tensor(ctx, job_id, AIPU_TENSOR_TYPE_OUTPUT,
                i, output_data_vec[i].get());
            if (ret != AIPU_STATUS_SUCCESS)
            {
                aipu_get_error_message(ctx, ret, &msg);
                AIPU_ERR()("aipu_get_tensor: %s\n", msg);
                pass = -1;
                goto clean_job;
            }
            AIPU_INFO()("get output tensor %u success (%u/%u)\n",
                i, i+1, output_cnt);
        }

        pass = check_result(output_data_vec, output_desc, opt.gts[0], opt.gts_size[0]);

        // clear the stale data for next loop
        for (uint32_t i = 0; i < output_cnt; i++)
            memset(output_data_vec[i].get(), 0x00, output_desc[i].size);
    }

clean_job:
    ret = aipu_clean_job(ctx, job_id);
    if (ret != AIPU_STATUS_SUCCESS)
    {
        aipu_get_error_message(ctx, ret, &msg);
        AIPU_ERR()("aipu_clean_job: %s\n", msg);
        pass = -1;
        goto finish;
    }
    AIPU_INFO()("aipu_clean_job success\n");

unload_graph:
    ret = aipu_unload_graph(ctx, graph_id);
    if (ret != AIPU_STATUS_SUCCESS)
    {
        aipu_get_error_message(ctx, ret, &msg);
        AIPU_ERR()("aipu_unload_graph: %s\n", msg);
        pass = -1;
        goto finish;
    }
    AIPU_INFO()("aipu_unload_graph success\n");

finish:
    g_pass |= pass;
    AIPU_INFO()  << "g_pass =" << g_pass;
    return;
}

void pipeline()
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    aipu_create_job_cfg_t create_job_cfg = {0};
    const char* msg = nullptr;
    uint64_t graph_id;
    uint32_t input_cnt, output_cnt;
    vector<aipu_tensor_desc_t> input_desc;
    vector<char*> input_data;
    vector<aipu_tensor_desc_t> output_desc;
    vector<shared_ptr<char>> output_data_vec;
    vector<char*> gt;
    uint32_t pipe_cnt = 2;
    uint64_t job_id_vec[pipe_cnt] = {0};
    aipu_status_t aipu_sts = AIPU_STATUS_SUCCESS;
    aipu_job_status_t aipu_job_sts = AIPU_JOB_STATUS_NO_STATUS;
    uint32_t cfg_types = 0;
    std::string path, tmp_path;
    aipu_job_config_simulation_t tmp_sim_job_config = {0};
    aipu_job_config_dump_t tmp_mem_dump_config = {0};
    int pass = 0;

    AIPU_DBG() << "pipeline()";

    path = opt.dump_dir;
    path = path + "/" + std::to_string(gettid());
    AIPU_DBG() << path;

    ret = aipu_load_graph(ctx, opt.bin_file_name, &graph_id);
    if (ret != AIPU_STATUS_SUCCESS)
    {
        aipu_get_error_message(ctx, ret, &msg);
        AIPU_ERR()("aipu_load_graph_helper: %s (%s)\n",
            msg, opt.bin_file_name);
        goto finish;
    }
    AIPU_INFO() << "Graph id= " << std::hex << graph_id;

    ret = aipu_get_tensor_count(ctx, graph_id, AIPU_TENSOR_TYPE_INPUT, &input_cnt);
    if (ret != AIPU_STATUS_SUCCESS)
    {
        aipu_get_error_message(ctx, ret, &msg);
        AIPU_ERR()("aipu_get_tensor_count: %s\n", msg);
        goto unload_graph;
    }
    //AIPU_INFO()("aipu_get_tensor_count success: input cnt = %d\n", input_cnt);

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
    //AIPU_INFO()("aipu_get_tensor_count success: output cnt = %d\n", output_cnt);

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
    //AIPU_ERR()("aipu_get_tensor_descriptor done\n");

    for (uint32_t i = 0; i < pipe_cnt; i++)
    {
        ret = aipu_create_job(ctx, graph_id, &job_id_vec[i], &create_job_cfg);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(ctx, ret, &msg);
            AIPU_ERR()("aipu_create_job: %s\n", msg);
            goto unload_graph;
        }
        AIPU_INFO()("aipu_create_job %lx success\n", job_id_vec[i]);
    }

    if (opt.inputs.size() != input_cnt)
    {
        AIPU_INFO()("[TEST WARN] input file count (%u) != input tensor count (%u)\n",
            (uint32_t)opt.inputs.size(), input_cnt);
    }

    for (uint32_t i = 0; i < output_cnt; i++)
    {
        auto deleter = [](char* p) { delete [] p; };
        shared_ptr<char> sp(new char[output_desc[i].size], deleter);
        output_data_vec.push_back(sp);

    }

    #if ((defined RTDEBUG) && (RTDEBUG == 1))
        cfg_types = AIPU_JOB_CONFIG_TYPE_DUMP_TEXT  |
            AIPU_JOB_CONFIG_TYPE_DUMP_WEIGHT        |
            AIPU_JOB_CONFIG_TYPE_DUMP_RODATA        |
            AIPU_JOB_CONFIG_TYPE_DUMP_DESCRIPTOR    |
            AIPU_JOB_CONFIG_TYPE_DUMP_INPUT         |
            AIPU_JOB_CONFIG_TYPE_DUMP_OUTPUT        |
            AIPU_JOB_CONFIG_TYPE_DUMP_TCB_CHAIN     |
            AIPU_JOB_CONFIG_TYPE_DUMP_EMULATION;
    #else
        cfg_types = AIPU_JOB_CONFIG_TYPE_DUMP_OUTPUT;
    #endif

    /* run with with multiple frames */
    for (uint32_t frame = 0; frame < pipe_cnt; frame++)
    {
        AIPU_INFO()("Frame #%u\n", frame);

        tmp_path = path + "/" + std::to_string(frame);
        tmp_mem_dump_config.dump_dir = tmp_path.c_str();
        help_create_dir(tmp_mem_dump_config.dump_dir);
        AIPU_INFO() << tmp_mem_dump_config.dump_dir;
        ret = aipu_config_job(ctx, job_id_vec[frame], cfg_types, &tmp_mem_dump_config);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(ctx, ret, &msg);
            AIPU_ERR()("aipu_config_job: %s\n", msg);
            goto clean_job;
        }
        AIPU_INFO()("set dump config success\n");

        if (run_on_platform == ON_SIMULATOR)
        {
            tmp_sim_job_config.data_dir = tmp_path.c_str();
            ret = aipu_config_job(ctx, job_id_vec[frame], AIPU_CONFIG_TYPE_SIMULATION,
                &tmp_sim_job_config);
            if (ret != AIPU_STATUS_SUCCESS)
            {
                aipu_get_error_message(ctx, ret, &msg);
                AIPU_ERR()("aipu_config_job: %s\n", msg);
                goto clean_job;
            }
            AIPU_INFO()("set job simulation config success\n");
        }

        for (uint32_t i = 0; i < min((uint32_t)opt.inputs.size(), input_cnt); i++)
        {
            if (input_desc[i].size > opt.inputs_size[i])
            {
                AIPU_ERR()("input file %s len 0x%x < input tensor %u size 0x%x\n",
                    opt.input_files[i].c_str(), opt.inputs_size[i], i, input_desc[i].size);
                goto clean_job;
            }
            ret = aipu_load_tensor(ctx, job_id_vec[frame], i, opt.inputs[i]);
            if (ret != AIPU_STATUS_SUCCESS)
            {
                aipu_get_error_message(ctx, ret, &msg);
                AIPU_ERR()("aipu_load_tensor: %s\n", msg);
                goto clean_job;
            }
            AIPU_INFO()("load input tensor %d from %s (%u/%u)\n",
                i, opt.input_files[i].c_str(), i+1, input_cnt);
        }

        ret = aipu_flush_job(ctx, job_id_vec[frame]);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(ctx, ret, &msg);
            AIPU_ERR()("aipu_flush_job: %s\n", msg);
            pass = -1;
            goto clean_job;
        }
        AIPU_INFO()("aipu_flush_job %lx success\n", job_id_vec[frame]);
    }

    for (uint32_t frame = 0; frame < pipe_cnt; frame++)
    {
        aipu_job_sts = AIPU_JOB_STATUS_NO_STATUS;
        while ((aipu_job_sts != AIPU_JOB_STATUS_DONE) &&
            (aipu_job_sts != AIPU_JOB_STATUS_EXCEPTION))
        {
            aipu_sts = aipu_get_job_status(ctx, job_id_vec[frame], &aipu_job_sts, 5);
            if (aipu_sts == AIPU_STATUS_ERROR_TIMEOUT)
                continue;
            else if (aipu_sts != AIPU_STATUS_SUCCESS) {
                aipu_get_error_message(ctx, aipu_sts, &msg);
                AIPU_ERR()("aipu_get_job_status: %s\n", msg);
                break;
            }
            AIPU_INFO()(" job %lx still running...\n", job_id_vec[frame]);
        }

        if (aipu_sts == AIPU_STATUS_SUCCESS )
        {
            if (aipu_job_sts == AIPU_JOB_STATUS_DONE)
            {
                int result = 0;
                for (uint32_t i = 0; i < output_cnt; i++)
                {
                    ret = aipu_get_tensor(ctx, job_id_vec[frame], AIPU_TENSOR_TYPE_OUTPUT,
                        i, output_data_vec[i].get());
                    if (ret != AIPU_STATUS_SUCCESS)
                    {
                        aipu_get_error_message(ctx, ret, &msg);
                        AIPU_ERR()("aipu_get_tensor: %s\n", msg);
                        goto clean_job;
                    }
                    AIPU_INFO()("get output tensor %u success (%u/%u)\n",
                        i, i+1, output_cnt);
                }

                result = check_result(output_data_vec, output_desc, opt.gts[0], opt.gts_size[0]);
                if (result != 0)
                    pass = result;

                // clear the stale data for next loop
                for (uint32_t i = 0; i < output_cnt; i++)
                    memset(output_data_vec[i].get(), 0x00, output_desc[i].size);
            }  else if (aipu_job_sts == AIPU_JOB_STATUS_EXCEPTION) {
                AIPU_ERR()("get_job_status (%lx): status=%x [exception]\n",
                    job_id_vec[frame], aipu_sts);
                pass = -1;
                goto clean_job;
            }
        } else {
            AIPU_ERR()("get_job_status (%lx): status=%x, job_status=%x [fail]\n",
                job_id_vec[frame], aipu_sts, aipu_job_sts);
            pass = -1;
            goto clean_job;
        }
    }

clean_job:
    for (uint32_t frame = 0; frame < pipe_cnt; frame++)
    {
        ret = aipu_clean_job(ctx, job_id_vec[frame]);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(ctx, ret, &msg);
            AIPU_ERR()("aipu_clean_job: %s\n", msg);
            pass = -1;
            goto unload_graph;
        }
        AIPU_INFO()("aipu_clean_job %lx success\n", job_id_vec[frame]);
    }

unload_graph:
    ret = aipu_unload_graph(ctx, graph_id);
    if (ret != AIPU_STATUS_SUCCESS)
    {
        aipu_get_error_message(ctx, ret, &msg);
        AIPU_ERR()("aipu_unload_graph: %s\n", msg);
        pass = -1;
        goto finish;
    }
    AIPU_INFO()("aipu_unload_graph success\n");

finish:
    g_pass |= pass;
    return;
}

int main(int argc, char *argv[])
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    const char* msg = nullptr;
    bool pipeline_enable = false;
    vector<shared_ptr<thread>> thd_vec;
    void (*thread_cb)();

    run_on_platform = ON_SIMULATOR;
    if (access("/dev/aipu", F_OK) == 0)
        run_on_platform = ON_HW;

    AIPU_CRIT() << "usage: ./aipu_mthread_test [-p] -b aipu.bin -i input0.bin -c output.bin -d ./\n";

    for (int i = 0; i < argc; i++)
    {
        if (!strncmp(argv[i], "-p", 2))
            pipeline_enable = true;
    }

    memset(&sim_glb_config, 0, sizeof(sim_glb_config));

    if(init_test_bench(argc, argv, &opt, "multithread_test"))
    {
        AIPU_ERR()("invalid command line options/args\n");
        goto finish;
    }

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
    sim_glb_config.en_eval = true;
    sim_glb_config.simulator = opt.simulator;

    ret = aipu_init_context(&ctx);
    if (ret != AIPU_STATUS_SUCCESS)
    {
        aipu_get_error_message(ctx, ret, &msg);
        AIPU_ERR()("aipu_init_context: %s\n", msg);
        return -1;
    }
    AIPU_INFO()("aipu_init_context success\n");

    if (run_on_platform == ON_SIMULATOR)
    {
        ret = aipu_config_global(ctx, AIPU_CONFIG_TYPE_SIMULATION, &sim_glb_config);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(ctx, ret, &msg);
            AIPU_ERR()("aipu_config_global: %s\n", msg);
            exit(-1);
        }
        AIPU_INFO()("set global simulation config success\n");
    }

    if (pipeline_enable)
        thread_cb = pipeline;
    else
        thread_cb = non_pipeline;

    for (int i = 0; i < THREAD_NUM; i++)
        thd_vec.push_back(make_shared<thread>(thread_cb));

    for (int i = 0; i < THREAD_NUM; i++)
        thd_vec[i]->join();

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
        g_pass = -1;

    deinit_test_bench(&opt);

    return g_pass;
}
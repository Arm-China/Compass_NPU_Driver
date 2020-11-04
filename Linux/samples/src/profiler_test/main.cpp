// Copyright (C) 2022 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  main.cpp
 * @brief AIPU UMD test application: basic profiler test for arm64 platforms
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
#include <fstream>
#include <math.h>
#include "standard_api.h"
#include "common/cmd_line_parsing.h"
#include "common/helper.h"
#include "common/dbg.hpp"

using namespace std;

int dump_perfdata(aipu_ctx_handle_t *m_ctx, uint64_t graph_id, uint64_t job_id)
{
    aipu_status_t sts = AIPU_STATUS_SUCCESS;
    const char* msg = nullptr;
    aipu_tensor_desc_t desc;
    string perfdata_fname;
    uint32_t cnt;
    int ret = 0;

    sts = aipu_get_tensor_count(m_ctx, graph_id, AIPU_TENSOR_TYPE_PROFILER, &cnt);
    if (sts != AIPU_STATUS_SUCCESS) {
        aipu_get_error_message(m_ctx, sts, &msg);
        AIPU_ERR()("aipu_get_tensor_descriptor(%d): %s\n",
            AIPU_TENSOR_TYPE_PROFILER, msg);
        ret = -1;
        return ret;
    } else if (cnt == 0) {
        AIPU_CRIT() << "No profiler data\n";
        ret = -1;
        return ret;
    }

    sts = aipu_get_tensor_descriptor(m_ctx, graph_id, AIPU_TENSOR_TYPE_PROFILER, 0, &desc);
    if (sts != AIPU_STATUS_SUCCESS) {
        aipu_get_error_message(m_ctx, sts, &msg);
        AIPU_ERR()("aipu_get_tensor_descriptor(%d): %s\n",
            AIPU_TENSOR_TYPE_PROFILER, msg);
        ret = -1;
        return ret;
    }

    perfdata_fname = "./PerfData.bin";
    AIPU_INFO()("perfdata file: %s\n", perfdata_fname.c_str());

    ofstream ofs(perfdata_fname, ios::binary);
    if (!ofs.is_open()) {
        AIPU_ERR()("open: %s [fail]\n", perfdata_fname.c_str());
        ret = -1;
        return ret;
    }

    unique_ptr<char[]> buffer(new char[desc.size]);
    sts = aipu_get_tensor(m_ctx, job_id, AIPU_TENSOR_TYPE_PROFILER,
        0, buffer.get());
    if (sts != AIPU_STATUS_SUCCESS) {
        aipu_get_error_message(m_ctx, sts, &msg);
        AIPU_ERR()("get profiler tensor: %s [fail]\n", msg);
        ret = -1;
        goto finish;
    }
    AIPU_INFO()("get profiler tensor success");

    ofs.write(buffer.get(), desc.size);

finish:
    ofs.close();
    return ret;
}

int main(int argc, char* argv[])
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    aipu_ctx_handle_t* ctx;
    const char* msg = nullptr;
    uint64_t graph_id, job_id;
    uint32_t input_cnt, output_cnt;
    vector<aipu_tensor_desc_t> input_desc;
    vector<char*> input_data;
    vector<aipu_tensor_desc_t> output_desc;
    vector<char*> output_data;
    vector<char*> gt;
    cmd_opt_t opt;
    uint32_t frame_cnt = 5;
    int pass = 0;

    AIPU_CRIT() << "usage: ./aipu_profiler_test -b aipu.bin -i input0.bin -c output.bin -d ./\n";

    if(init_test_bench(argc, argv, &opt, "profiler_test"))
    {
        AIPU_ERR()("invalid command line options/args\n");
        goto finish;
    }

    ret = aipu_init_context(&ctx);
    if (ret != AIPU_STATUS_SUCCESS)
    {
        aipu_get_error_message(ctx, ret, &msg);
        AIPU_ERR()("aipu_init_context: %s\n", msg);
        goto finish;
    }
    AIPU_INFO()("aipu_init_context success\n");

    ret = aipu_load_graph(ctx, opt.bin_file_name, &graph_id);
    if (ret != AIPU_STATUS_SUCCESS)
    {
        aipu_get_error_message(ctx, ret, &msg);
        AIPU_ERR()("aipu_load_graph_helper: %s (%s)\n",
            msg, opt.bin_file_name);
        goto finish;
    }
    AIPU_INFO()("aipu_load_graph_helper success: %s\n", opt.bin_file_name);

    ret = aipu_get_tensor_count(ctx, graph_id, AIPU_TENSOR_TYPE_INPUT, &input_cnt);
    if (ret != AIPU_STATUS_SUCCESS)
    {
        aipu_get_error_message(ctx, ret, &msg);
        AIPU_ERR()("aipu_get_tensor_count: %s\n", msg);
        goto finish;
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
            goto finish;
        }
        input_desc.push_back(desc);
    }

    ret = aipu_get_tensor_count(ctx, graph_id, AIPU_TENSOR_TYPE_OUTPUT, &output_cnt);
    if (ret != AIPU_STATUS_SUCCESS)
    {
        aipu_get_error_message(ctx, ret, &msg);
        AIPU_ERR()("aipu_get_tensor_count: %s\n", msg);
        goto finish;
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
            goto finish;
        }
        output_desc.push_back(desc);
    }
    //fprintf(stderr, "[TEST INFO] aipu_get_tensor_descriptor done\n");

    ret = aipu_create_job(ctx, graph_id, &job_id);
    if (ret != AIPU_STATUS_SUCCESS)
    {
        aipu_get_error_message(ctx, ret, &msg);
        AIPU_ERR()("aipu_create_job: %s\n", msg);
        goto finish;
    }
    AIPU_INFO()("aipu_create_job success\n");

    if (opt.inputs.size() != input_cnt)
    {
        AIPU_INFO()("input file count (%u) != input tensor count (%u)\n",
            (uint32_t)opt.inputs.size(), input_cnt);
    }

    for (uint32_t i = 0; i < output_cnt; i++)
    {
        char* output = new char[output_desc[i].size];
        output_data.push_back(output);
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
                goto finish;
            }
            ret = aipu_load_tensor(ctx, job_id, i, opt.inputs[i]);
            if (ret != AIPU_STATUS_SUCCESS)
            {
                aipu_get_error_message(ctx, ret, &msg);
                AIPU_ERR()("aipu_load_tensor: %s\n", msg);
                goto finish;
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
            goto finish;
        }
        AIPU_INFO()("aipu_finish_job success\n");

        dump_perfdata(ctx, graph_id, job_id);

        for (uint32_t i = 0; i < output_cnt; i++)
        {
            ret = aipu_get_tensor(ctx, job_id, AIPU_TENSOR_TYPE_OUTPUT, i, output_data[i]);
            if (ret != AIPU_STATUS_SUCCESS)
            {
                aipu_get_error_message(ctx, ret, &msg);
                AIPU_ERR()("aipu_get_tensor: %s\n", msg);
                goto finish;
            }
            AIPU_INFO()("get output tensor %u success (%u/%u)\n",
                i, i+1, output_cnt);
        }

        pass = check_result_helper(output_data, output_desc, opt.gt, opt.gt_size);
    }

    ret = aipu_clean_job(ctx, job_id);
    if (ret != AIPU_STATUS_SUCCESS)
    {
        aipu_get_error_message(ctx, ret, &msg);
        AIPU_ERR()("aipu_clean_job: %s\n", msg);
        goto finish;
    }
    AIPU_INFO()("aipu_clean_job success\n");

    ret = aipu_unload_graph(ctx, graph_id);
    if (ret != AIPU_STATUS_SUCCESS)
    {
        aipu_get_error_message(ctx, ret, &msg);
        AIPU_ERR()("aipu_unload_graph: %s\n", msg);
        goto finish;
    }
    AIPU_INFO()("aipu_unload_graph success\n");

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
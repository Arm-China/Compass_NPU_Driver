// Copyright (C) 2022-2023 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  main.cpp
 * @brief AIPU UMD test application: test for aipu_load_graph_helper on arm64 platforms
 */

#include <stdio.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <iostream>
#include <fstream>
#include <string.h>
#include <errno.h>
#include <vector>
#include <math.h>
#include "standard_api.h"
#include "common/cmd_line_parsing.h"
#include "common/helper.h"
#include "common/dbg.hpp"

using namespace std;

void gen_resultbin(const char *gbin_buf, uint32_t fsize)
{
    std::ofstream obin;
    std::string *g_str = nullptr;

    g_str = new std::string(gbin_buf, fsize);
    std::cout << "g_str=" << g_str->length() << std::endl;

    obin.open("./test.bin", std::ofstream::out | std::ofstream::binary);
    if (!obin.is_open())
    {
        std::cout << "open file fail" << std::endl;
        return;
    }

    obin.write(gbin_buf, fsize);
    obin.flush();
    obin.close();
    delete g_str;
}

int debugger_bind_job(aipu_ctx_handle_t *m_ctx_handle, uint64_t m_job_id) {
    int error = -1;
    aipu_status_t result;
    aipu_core_info_t aipu_core_info;
    uint32_t aipu_core_count;
    uint32_t aipu_core_selected = 0;
    const char* status_msg = nullptr;

    result = aipu_get_core_count(m_ctx_handle, 0, 0, &aipu_core_count);
    if (AIPU_STATUS_SUCCESS != result) {
        aipu_get_error_message(m_ctx_handle, result, &status_msg);
        AIPU_ERR()("RT_API:aipu_get_core_count: %s.", status_msg);
        return error;
    }
    if (aipu_core_count < 1) {
        aipu_get_error_message(m_ctx_handle, result, &status_msg);
        AIPU_ERR()("RT_API:aipu_get_core_count: core_count: %d", aipu_core_count);
        return error;
    }

    for (uint32_t i = 0 ; i < aipu_core_count; ++i) {
        result = aipu_debugger_get_core_info(m_ctx_handle, i, &aipu_core_info);
        if (AIPU_STATUS_SUCCESS != result) {
            aipu_get_error_message(m_ctx_handle, result, &status_msg);
            AIPU_ERR()("RT_API:aipu_debugger_get_core_info: %s.", status_msg);
            return error;
        }
        // if (aipu_core_info.reg_base == m_core_addr) {
        //   aipu_core_selected = i;
        //   break;
        // }
    }

    result = aipu_debugger_bind_job(m_ctx_handle, aipu_core_selected, m_job_id);
    if (AIPU_STATUS_SUCCESS != result) {
        // RuntimeThreadExit();
        aipu_get_error_message(m_ctx_handle, result, &status_msg);
        AIPU_ERR()("RT_API:aipu_debugger_bind_job: %s.", status_msg);
        return error;
    }

    return 0;
}

int debugger_bind_job_x2(aipu_ctx_handle_t *m_ctx_handle, uint64_t m_job_id) {
    int error = -1;
    aipu_status_t result;
    aipu_core_info_t aipu_core_info;
    uint32_t aipu_part_count;
    uint32_t aipu_part_selected = 0;
    const char* status_msg = nullptr;

    result = aipu_get_partition_count(m_ctx_handle, &aipu_part_count);
    if (AIPU_STATUS_SUCCESS != result) {
        aipu_get_error_message(m_ctx_handle, result, &status_msg);
        AIPU_ERR()(
        "RT_API:aipu_get_partition_count: %s.", status_msg);
        return error;
    }
    if (aipu_part_count < 1) {
        aipu_get_error_message(m_ctx_handle, result, &status_msg);
        AIPU_ERR()("RT_API:aipu_get_partition_count: part_count: %d", aipu_part_count);
        return error;
    }

    for (uint32_t i = 0 ; i < aipu_part_count; ++i) {
        result = aipu_debugger_get_core_info(m_ctx_handle, i, &aipu_core_info);
        if (AIPU_STATUS_SUCCESS != result) {
            aipu_get_error_message(m_ctx_handle, result, &status_msg);
            AIPU_ERR()("RT_API:aipu_debugger_get_core_info: %s.", status_msg);
            return error;
        }
    }

    result = aipu_debugger_bind_job(m_ctx_handle, aipu_part_selected, m_job_id);
    if (AIPU_STATUS_SUCCESS != result) {
        // RuntimeThreadExit();
        aipu_get_error_message(m_ctx_handle, result, &status_msg);
        AIPU_ERR()("RT_API:aipu_debugger_bind_job: %s.", status_msg);
        return error;
    }

    return 0;
}

aipu_status_t misc_test(aipu_ctx_handle_t *ctx)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    const char* msg = nullptr;
    char *debug_buf = nullptr;

    ret = aipu_deinit_context(ctx);
    if (ret != AIPU_STATUS_SUCCESS)
    {
        aipu_get_error_message(ctx, ret, &msg);
        AIPU_ERR()("aipu_deinit_context: %s\n", msg);
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

    ret = aipu_debugger_malloc(ctx, 8, reinterpret_cast<void **>(&debug_buf));
    if (ret != AIPU_STATUS_SUCCESS)
    {
        aipu_get_error_message(ctx, ret, &msg);
        AIPU_ERR()("aipu_debugger_malloc: %s\n", msg);
        goto finish;
    }

    aipu_debugger_free(ctx, reinterpret_cast<void **>(&debug_buf));
    if (ret != AIPU_STATUS_SUCCESS)
    {
        aipu_get_error_message(ctx, ret, &msg);
        AIPU_ERR()("aipu_debugger_malloc: %s\n", msg);
        goto finish;
    }

finish:
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
    std::ifstream gbin;
    uint32_t fsize = 0;
    char *gbin_buf = nullptr;
    int retval = -1;
    aipu_create_job_cfg create_job_cfg = {0};

    if(init_test_bench(argc, argv, &opt, "debugger_test"))
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

    // misc_test(ctx);

    gbin.open(opt.bin_file_name, std::ifstream::in | std::ifstream::binary);
    if (!gbin.is_open())
    {
        return AIPU_STATUS_ERROR_OPEN_FILE_FAIL;
    }

    gbin.seekg (0, gbin.end);
    fsize = gbin.tellg();
    gbin.seekg (0, gbin.beg);

    gbin_buf = new char[fsize];
    gbin.read(gbin_buf, fsize);
    gbin.seekg (0, gbin.beg);

    // ret = aipu_load_graph(ctx, opt.bin_file_name, &graph_id);
    ret = aipu_load_graph_helper(ctx, gbin_buf, fsize, &graph_id);
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
    //AIPU_ERR()( "[TEST INFO] aipu_get_tensor_descriptor done\n");

    ret = aipu_create_job(ctx, graph_id, &job_id, &create_job_cfg);
    if (ret != AIPU_STATUS_SUCCESS)
    {
        aipu_get_error_message(ctx, ret, &msg);
        AIPU_ERR()("aipu_create_job: %s\n", msg);
        goto unload_graph;
    }
    AIPU_INFO()("aipu_create_job success\n");

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

    /* run with with multiple frames */

    for (uint32_t frame = 0; frame < frame_cnt; frame++)
    {
        AIPU_INFO()("Frame #%u\n", frame);

        retval = debugger_bind_job(ctx, job_id);
        // retval = debugger_bind_job_x2(ctx, job_id);
        printf("debugger_bind_job, ret = %d\n", retval);
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

        ret = aipu_debugger_run_job(ctx, job_id);
        // ret = aipu_finish_job(ctx, job_id, -1);
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
    AIPU_INFO()("aipu_clean_job success\n");

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
    {
        pass = -1;
    }
    for (uint32_t i = 0; i < output_data.size(); i++)
    {
        delete[] output_data[i];
    }
    deinit_test_bench(&opt);
    gbin.close();
    delete[] gbin_buf;

    return pass;
}
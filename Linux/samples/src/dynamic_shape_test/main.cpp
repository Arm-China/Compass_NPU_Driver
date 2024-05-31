// Copyright (C) 2023-2024 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  main.cpp
 * @brief AIPU UMD test application: dynamic shape test
 *
 * @note  aipu_dynamic_shape_test -b aipu.bin -i input.bin -c output.bin -d ./output/
 *                                -r input_shape(eg: -r 1,480,640,3)
 *        aipu.bin should support dynamic shape, you can get shape info and
 *        set shape by aipu_ioctl interface, refer to line 89-151, output
 *        shape info should get after finish job, refer to line 186-216.
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
#include "kmd/armchina_aipu.h"

using namespace std;

char * g_remain_char = NULL;

char * aipu_strtok( char * s,const char * ct)
{
    char *sbegin, *send;
    sbegin  = s ? s : g_remain_char;
    if (!sbegin) {
        return NULL;
    }
    sbegin += strspn(sbegin,ct);
    if (*sbegin == '\0') {
        g_remain_char = NULL;
        return (NULL);
    }
    send = strpbrk(sbegin, ct);
    if (send && *send != '\0') {
        *send++ = '\0';
    }
    g_remain_char = send;
    return (sbegin);
}

int main(int argc, char* argv[])
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    aipu_ctx_handle_t* ctx;
    uint64_t graph_id, job_id;
    const char *msg = nullptr;
    uint32_t output_cnt;
    vector<uint32_t> input_shape_vec;
    vector<vector<uint32_t>> input_shape_vec_vec;
    aipu_dynshape_num_t dynshape_num = {0};
    vector<aipu_dynshape_dim_num_t> dim_num_vec;
    aipu_create_job_cfg create_job_cfg = {0};
    aipu_dynshape_param_t dynshape_param = {0};
    aipu_job_config_dump_t mem_dump_config = {0};
    vector<aipu_tensor_desc_t> output_desc;
    vector<char*> output_data;
    cmd_opt_t opt;
    int pass = 0;
    char* temp_l = nullptr;
    char* temp_s = nullptr;

    if (init_test_bench(argc, argv, &opt, "dynamic_shape_test")) {
        AIPU_ERR()("invalid command line options/args\n");
        goto finish;
    }

    temp_l = aipu_strtok(opt.input_shape, "/");
    while (temp_l) {
        temp_s = strtok(temp_l, ",");
        input_shape_vec.push_back(stoi(temp_s));
        while (temp_s)
        {
            temp_s = strtok(nullptr, ",");
            if (temp_s != nullptr)
                input_shape_vec.push_back(stoi(temp_s));
        }
        input_shape_vec_vec.push_back(input_shape_vec);
        input_shape_vec.clear();
        temp_l = aipu_strtok(NULL, "/");
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
        goto deinit_ctx;
    }
    AIPU_INFO()("aipu_load_graph_helper success: %s\n", opt.bin_file_name);

    /**
     * AIPU_IOCTL_GET_DS_NUM: get dynamic shape input tensor num
     */
    dynshape_num.graph_id = graph_id;
    dynshape_num.ds_num = (uint32_t *)malloc(sizeof(uint32_t));
    ret = aipu_ioctl(ctx, AIPU_IOCTL_GET_DS_NUM, &dynshape_num);
    if (ret != AIPU_STATUS_SUCCESS) {
        aipu_get_error_message(ctx, ret, &msg);
        AIPU_ERR()("aipu_ioctl: %s\n", msg);
        goto unload_graph;
    }

    if (!(*dynshape_num.ds_num)) {
        AIPU_ERR()("dynamic shape tensor num = 0, please check aipu.bin\n");
        goto unload_graph;
    } else {
        AIPU_DBG()("dynamic shape num: %d\n", *(dynshape_num.ds_num));
    }

    /**
     * AIPU_IOCTL_GET_DS_DIM_NUM: get input tensor dim num
     * AIPU_IOCTL_GET_DS_INFO: get input tensor shape info
     *     max_threshhold = true return max shape info
     *     max_threshhold = false return min shape info
     * AIPU_IOCTL_SET_DS_INFO: set shape info to benchmark
     */
    dynshape_param.graph_id = graph_id;
    dynshape_param.input_shape_cnt = *dynshape_num.ds_num;
    dynshape_param.shape_items = (aipu_dynshape_item_t *)malloc(sizeof(aipu_dynshape_item_t) * (*dynshape_num.ds_num));

    for (uint32_t  id = 0 ; id < *dynshape_num.ds_num; id++) {
        aipu_dynshape_dim_num_t dynshape_dim_num = {0};
        dynshape_dim_num.graph_id = graph_id;
        dynshape_dim_num.ds_idx = id;
        dynshape_dim_num.max_threshhold = true;
        dynshape_dim_num.ds_dim_num = (uint32_t *)malloc(sizeof(uint32_t));
        ret = aipu_ioctl(ctx, AIPU_IOCTL_GET_DS_DIM_NUM, &dynshape_dim_num);
        if (ret != AIPU_STATUS_SUCCESS) {
            aipu_get_error_message(ctx, ret, &msg);
            AIPU_ERR()("aipu_ioctl: %s\n", msg);
            goto unload_graph;
        }

        aipu_dynshape_info_t dynshape_info = {0};
        dynshape_info.graph_id = graph_id;
        dynshape_info.ds_idx = id;
        dynshape_info.max_threshhold = true;
        dynshape_info.ds_data = (uint32_t *)malloc(sizeof(uint32_t) * (*dynshape_dim_num.ds_dim_num));
        AIPU_DBG()("dynamic shape dim num: %d\n", *(dynshape_dim_num.ds_dim_num));
        ret = aipu_ioctl(ctx, AIPU_IOCTL_GET_DS_INFO, &dynshape_info);
        if (ret != AIPU_STATUS_SUCCESS) {
            aipu_get_error_message(ctx, ret, &msg);
            AIPU_ERR()("aipu_ioctl: %s\n", msg);
            goto unload_graph;
        }

        dynshape_param.shape_items[id].ds_idx = id;
        dynshape_param.shape_items[id].ds_data = (uint32_t *)malloc(sizeof(uint32_t) * (*dynshape_dim_num.ds_dim_num));
        for (uint32_t dim_id = 0; dim_id < *dynshape_dim_num.ds_dim_num; dim_id++) {
            AIPU_DBG()("dynamic shape max shape: %d\n", *(dynshape_info.ds_data + dim_id));
            AIPU_DBG()("dynamic shape set shape: %d\n", input_shape_vec_vec[id][dim_id]);
            *(dynshape_param.shape_items[id].ds_data + dim_id) = input_shape_vec_vec[id][dim_id];
        }
    }

    ret = aipu_ioctl(ctx, AIPU_IOCTL_SET_DS_INFO, &dynshape_param);
    if (ret != AIPU_STATUS_SUCCESS) {
        aipu_get_error_message(ctx, ret, &msg);
        AIPU_ERR()("aipu_ioctl: %s\n", msg);
        goto unload_graph;
    }
    AIPU_INFO()("aipu set dynshape_param success\n");

    ret = aipu_create_job(ctx, graph_id, &job_id, &create_job_cfg);
    if (ret != AIPU_STATUS_SUCCESS) {
        aipu_get_error_message(ctx, ret, &msg);
        AIPU_ERR()("aipu_create_job: %s\n", msg);
        goto unload_graph;
    }
    AIPU_INFO()("aipu_create_job success\n");

    mem_dump_config.dump_dir = opt.dump_dir;
    if (opt.dump_opt > 0) {
        ret = aipu_config_job(ctx, job_id, opt.dump_opt, &mem_dump_config);
        if (ret != AIPU_STATUS_SUCCESS) {
            aipu_get_error_message(ctx, ret, &msg);
            AIPU_ERR()("aipu_config_job: %s\n", msg);
            goto clean_job;
        }
        AIPU_INFO()("set dump config success\n");
    }

    for (uint32_t id = 0; id < *dynshape_num.ds_num; id++) {
        ret = aipu_load_tensor(ctx, job_id, id, opt.inputs[id]);
        if (ret != AIPU_STATUS_SUCCESS) {
            aipu_get_error_message(ctx, ret, &msg);
            AIPU_ERR()("aipu_load_tensor: %s\n", msg);
            goto clean_job;
        }
        AIPU_INFO()("load input tensor %d from %s (%u/%u)\n",
                id, opt.input_files[id].c_str(), id+1, dynshape_num.ds_num);
    }

    ret = aipu_finish_job(ctx, job_id, -1);
    if (ret != AIPU_STATUS_SUCCESS) {
        aipu_get_error_message(ctx, ret, &msg);
        AIPU_ERR()("aipu_finish_job: %s\n", msg);
        goto clean_job;
    }
    AIPU_INFO()("aipu_finish_job success\n");

    ret = aipu_get_tensor_count(ctx, graph_id, AIPU_TENSOR_TYPE_OUTPUT, &output_cnt);
    if (ret != AIPU_STATUS_SUCCESS) {
        aipu_get_error_message(ctx, ret, &msg);
        AIPU_ERR()("aipu_get_tensor_count: %s\n", msg);
        goto clean_job;
    }
    AIPU_DBG()("aipu_get_tensor_count success: output cnt = %d\n", output_cnt);

    /**
     * after finish job, output tensor shape info has write to output tensor desc
     */
    for (uint32_t i = 0; i < output_cnt; i++) {
        aipu_tensor_desc_t desc;
        ret = aipu_get_tensor_descriptor(ctx, graph_id, AIPU_TENSOR_TYPE_OUTPUT, i, &desc);
        if (ret != AIPU_STATUS_SUCCESS) {
            aipu_get_error_message(ctx, ret, &msg);
            AIPU_ERR()("aipu_get_tensor_descriptor: %s\n", msg);
            goto clean_job;
        }
        output_desc.push_back(desc);
    }

    /* alloc output tensor space */
    for (uint32_t i = 0; i < output_desc.size(); i++) {
        char* output = new char[output_desc[i].size * 2];
        output_data.push_back(output);
    }

    for (uint32_t i = 0; i < output_desc.size(); i++) {
        ret = aipu_get_tensor(ctx, job_id, AIPU_TENSOR_TYPE_OUTPUT,
            i, output_data[i]);
        if (ret != AIPU_STATUS_SUCCESS) {
            aipu_get_error_message(ctx, ret, &msg);
            AIPU_ERR()("aipu_get_tensor: %s\n", msg);
            goto clean_job;
        }
        AIPU_DBG()("get output tensor %u success (%u/%lu)\n",
            i, i+1, output_desc.size());
    }

    pass = check_result_helper(output_data, output_desc, opt.gts[0], opt.gts_size[0]);

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
        pass = -1;

    for (uint32_t i = 0; i < output_data.size(); i++)
        delete[] output_data[i];

    deinit_test_bench(&opt);

    return pass;
}

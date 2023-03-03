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
 * aipu_multi_model_test -b aipu.bin -i input.bin -c output.bin -d ./output/
 *
 * @brief this case emulate that load two models and create one job for each
 *        model and infer the job.
 *
 * NOTE:
 *      1. load the same model twice and create one job for each loaded graph.
 *      2. load input data for each job and flush job one by one.
 *      3. get job's status one by one, and check each job's inferrence output.
 */

typedef struct {
    uint64_t graph_id;
    uint64_t job_id;

    /* input tensor info */
    uint32_t in_tensor_cnt;
    vector<aipu_tensor_desc_t> in_tensor_desc;
    vector<char *> input_data_vec;

    /* output tensor info */
    uint32_t out_tensor_cnt;
    vector<aipu_tensor_desc_t> out_tensor_desc;
    vector<shared_ptr<char>> output_data_vec;

    /* graph info from command line */
    string aipubin;
    vector<string> inputbins;
    string gtbin;
    char *gt_data;
    uint32_t gt_size;
} infer_info_t;

aipu_ctx_handle_t *ctx;
cmd_opt_t opt;
int g_pass = -1;

#define GRAPH_NUM 2
infer_info_t infer_info[GRAPH_NUM];

void pipeline()
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    const char *msg = nullptr;
    vector<aipu_tensor_desc_t> input_desc;
    vector<char*> input_data;
    vector<aipu_tensor_desc_t> output_desc;
    vector<shared_ptr<char>> output_data_vec;
    vector<char*> gt;
    aipu_status_t aipu_sts = AIPU_STATUS_SUCCESS;
    aipu_job_status_t aipu_job_sts = AIPU_JOB_STATUS_NO_STATUS;
    int pass = 0;

    AIPU_DBG() << "pipeline()";

    /**
     * NOTE:
     *      load the same graph twice, get each graph's input/output tensor
     *      descriptor and number.
     */
    for (uint32_t i = 0; i < GRAPH_NUM; i++)
    {
        uint64_t graph_id = 0;
        infer_info[i].graph_id = 0;
        infer_info[i].job_id = 0;
        infer_info[i].aipubin = opt.bin_file_name;
        infer_info[i].inputbins.assign(opt.input_files.begin(), opt.input_files.end());
        infer_info[i].gt_data = opt.gt;
        infer_info[i].gt_size = opt.gt_size;

        ret = aipu_load_graph(ctx, infer_info[i].aipubin.c_str(), &graph_id);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(ctx, ret, &msg);
            AIPU_ERR()("aipu_load_graph_helper: %s (%s)\n",
                msg, infer_info[i].aipubin.c_str());
            goto finish;
        }
        infer_info[i].graph_id = graph_id;
        AIPU_INFO() << infer_info[i].aipubin;
        AIPU_INFO() << "Graph id= " << infer_info[i].graph_id;

        ret = aipu_get_tensor_count(ctx, graph_id, AIPU_TENSOR_TYPE_INPUT,
            &infer_info[i].in_tensor_cnt);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(ctx, ret, &msg);
            AIPU_ERR()("aipu_get_tensor_count: %s\n", msg);
            goto unload_graph;
        }

        for (uint32_t j = 0; j < infer_info[i].in_tensor_cnt; j++)
        {
            aipu_tensor_desc_t desc;
            ret = aipu_get_tensor_descriptor(ctx, graph_id, AIPU_TENSOR_TYPE_INPUT,
                j, &desc);
            if (ret != AIPU_STATUS_SUCCESS)
            {
                aipu_get_error_message(ctx, ret, &msg);
                AIPU_ERR()("aipu_get_tensor_descriptor: %s\n", msg);
                goto unload_graph;
            }
            infer_info[i].in_tensor_desc.push_back(desc);

            infer_info[i].input_data_vec.push_back(opt.inputs[j]);
        }

        ret = aipu_get_tensor_count(ctx, graph_id, AIPU_TENSOR_TYPE_OUTPUT,
            &infer_info[i].out_tensor_cnt);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(ctx, ret, &msg);
            AIPU_ERR()("aipu_get_tensor_count: %s\n", msg);
            goto unload_graph;
        }

        for (uint32_t k = 0; k < infer_info[i].out_tensor_cnt; k++)
        {
            aipu_tensor_desc_t desc;
            ret = aipu_get_tensor_descriptor(ctx, graph_id, AIPU_TENSOR_TYPE_OUTPUT, k, &desc);
            if (ret != AIPU_STATUS_SUCCESS)
            {
                aipu_get_error_message(ctx, ret, &msg);
                AIPU_ERR()("aipu_get_tensor_descriptor: %s\n", msg);
                goto unload_graph;
            }
            infer_info[i].out_tensor_desc.push_back(desc);

            auto deleter = [](char* p) { delete[] p; };
            shared_ptr<char> sp(new char[infer_info[i].out_tensor_desc[k].size], deleter);
            infer_info[i].output_data_vec.push_back(sp);
        }
    }

    /**
     * NOTE:
     *      create one job for each graph.
     */
    for (uint32_t i = 0; i < GRAPH_NUM; i++)
    {
        ret = aipu_create_job(ctx, infer_info[i].graph_id, &infer_info[i].job_id);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(ctx, ret, &msg);
            AIPU_ERR()("aipu_create_job: %s\n", msg);
            goto unload_graph;
        }
        AIPU_INFO()("aipu_create_job %lx success\n", infer_info[i].job_id);
    }

    /**
     * NOTE:
     *       load the coressponding input frame data for each job and commit the jobs.
     */
    for (uint32_t i = 0; i < GRAPH_NUM; i++)
    {
        AIPU_INFO()("Graph #%u\n", i);
        for (uint32_t j = 0; j < infer_info[i].in_tensor_cnt; j++)
        {
            ret = aipu_load_tensor(ctx, infer_info[i].job_id, j, infer_info[i].input_data_vec[j]);
            if (ret != AIPU_STATUS_SUCCESS)
            {
                aipu_get_error_message(ctx, ret, &msg);
                AIPU_ERR()("aipu_load_tensor: %s\n", msg);
                goto clean_job;
            }
            AIPU_INFO()("load input tensor %d from %s (%u/%u)\n",
                j, infer_info[i].inputbins[j].c_str(), j+1, infer_info[i].in_tensor_cnt);
        }

        ret = aipu_flush_job(ctx, infer_info[i].job_id);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(ctx, ret, &msg);
            AIPU_ERR()("aipu_flush_job: %s\n", msg);
            pass = -1;
            goto clean_job;
        }
        AIPU_INFO()("aipu_flush_job %lx success\n", infer_info[i].job_id);
    }

    /**
     * NOTE:
     *      poll each job's status, if job is done normally, check its output data.
     */
    for (uint32_t i = 0; i < GRAPH_NUM; i++)
    {
        aipu_job_sts = AIPU_JOB_STATUS_NO_STATUS;
        while ((aipu_job_sts != AIPU_JOB_STATUS_DONE) &&
            (aipu_job_sts != AIPU_JOB_STATUS_EXCEPTION))
        {
            aipu_sts = aipu_get_job_status(ctx, infer_info[i].job_id, &aipu_job_sts, 5);
            if (aipu_sts == AIPU_STATUS_ERROR_TIMEOUT)
                continue;
            else if (aipu_sts != AIPU_STATUS_SUCCESS) {
                aipu_get_error_message(ctx, aipu_sts, &msg);
                AIPU_ERR()("aipu_get_job_status: %s\n", msg);
                break;
            }
            AIPU_INFO()(" job %lx still running...\n", infer_info[i].job_id);
        }

        if (aipu_sts == AIPU_STATUS_SUCCESS )
        {
            if (aipu_job_sts == AIPU_JOB_STATUS_DONE)
            {
                int result = 0;
                for (uint32_t j = 0; j < infer_info[i].out_tensor_cnt; j++)
                {
                    ret = aipu_get_tensor(ctx, infer_info[i].job_id, AIPU_TENSOR_TYPE_OUTPUT,
                        j, infer_info[i].output_data_vec[j].get());
                    if (ret != AIPU_STATUS_SUCCESS)
                    {
                        aipu_get_error_message(ctx, ret, &msg);
                        AIPU_ERR()("aipu_get_tensor: %s\n", msg);
                        goto clean_job;
                    }
                    AIPU_INFO()("get output tensor %u success (%u/%u)\n",
                        j, j+1, infer_info[i].out_tensor_cnt);
                }

                result = check_result(infer_info[i].output_data_vec,
                    infer_info[i].out_tensor_desc, infer_info[i].gt_data, infer_info[i].gt_size);
                if (result != 0)
                    pass = result;
            }  else if (aipu_job_sts == AIPU_JOB_STATUS_EXCEPTION) {
                AIPU_ERR()("get_job_status (%lx): status=%x [exception]\n",
                    infer_info[i].job_id, aipu_sts);
                pass = -1;
                goto clean_job;
            }
        } else {
            AIPU_ERR()("get_job_status (%lx): status=%x, job_status=%x [fail]\n",
                infer_info[i], aipu_sts, aipu_job_sts);
            pass = -1;
            goto clean_job;
        }
    }

clean_job:
    for (uint32_t i = 0; i < GRAPH_NUM; i++)
    {
        if (infer_info[i].job_id == 0)
            continue;

        ret = aipu_clean_job(ctx, infer_info[i].job_id);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(ctx, ret, &msg);
            AIPU_ERR()("aipu_clean_job: %s\n", msg);
            pass = -1;
            goto unload_graph;
        }
        AIPU_INFO()("aipu_clean_job %lx success\n", infer_info[i].job_id);
    }

unload_graph:
    for (uint32_t i = 0; i < GRAPH_NUM; i++)
    {
        if (infer_info[i].graph_id == 0)
            continue;

        ret = aipu_unload_graph(ctx, infer_info[i].graph_id);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(ctx, ret, &msg);
            AIPU_ERR()("aipu_unload_graph: %s\n", msg);
            pass = -1;
            goto finish;
        }
        AIPU_INFO()("aipu_unload_graph success\n");
    }

finish:
    g_pass = pass;
    return;
}

int main(int argc, char *argv[])
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    const char *msg = nullptr;

    AIPU_CRIT() << "usage: ./aipu_multi_model_test -b aipu.bin -i input0.bin -c output.bin -d ./\n";

    if(init_test_bench(argc, argv, &opt, "multi_model_test"))
    {
        AIPU_ERR()("invalid command line options/args\n");
        return -1;
    }

    ret = aipu_init_context(&ctx);
    if (ret != AIPU_STATUS_SUCCESS)
    {
        aipu_get_error_message(ctx, ret, &msg);
        AIPU_ERR()("aipu_init_context: %s\n", msg);
        return -1;
    }
    AIPU_INFO()("aipu_init_context success\n");

    pipeline();

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
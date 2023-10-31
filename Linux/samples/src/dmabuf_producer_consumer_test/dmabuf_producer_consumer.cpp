// Copyright (C) 2023-2024 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  main.cpp
 * @brief AIPU UMD test application: dmabuf test as exporter and importer
 */

/**
 * @brief get a dmabuf from producer and set it as input tensor, as well as request
 *        a dmabuf as output tensor, then infer the model, finally it transfer output
 *        dmabuf to consumer which will check the inferrence result.
 *
 * @note  it has to ensure that the input/output buffer can't be shared with
 *        other intermidiate buffers. when generating model binary with NN
 *        Compiler graph builder(aipugb), it has to append parameters
 *        '--disable_input_buffer_reuse' or '--disable_output_buffer_reuse'.
 *        please reference the detailed command in sample/README.md.
 *
 */
#include "common.h"

using namespace std;

#define DMABUF_SWITCH 1
#define OUTPUT_SWITCH 1

int main(int argc, char* argv[])
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
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
    int pass = 0, loop = 0, total_loop = 2;
    uint64_t cfg_types = 0;
    aipu_create_job_cfg create_job_cfg = {0};
    #if DMABUF_SWITCH
    aipu_shared_tensor_info_t input_share_tensor;
    int dmabuf_input_fd = -1;
    #if OUTPUT_SWITCH
    aipu_shared_tensor_info_t output_share_tensor;
    int dmabuf_output_fd = -1;
    #endif
    #endif

    AIPU_CRIT() << "usage: ./aipu_dmabuf_main_test -b aipu.bin -d ./\n";

    aipu_job_config_dump_t mem_dump_config;
    memset(&mem_dump_config, 0, sizeof(mem_dump_config));

    if(init_test_bench(argc, argv, &opt, "dmabuf_mmap_test"))
    {
        AIPU_ERR()("invalid command line options/args\n");
        goto finish;
    }

    mem_dump_config.dump_dir = opt.dump_dir;

    for (loop = 0; loop < total_loop; loop++)
    {
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

        ret = aipu_get_cluster_count(ctx, 0, &cluster_cnt);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(ctx, ret, &msg);
            AIPU_ERR()("aipu_get_cluster_count: %s (%s)\n",
                msg, opt.bin_file_name);
            goto unload_graph;
        }
        //AIPU_INFO()("aipu_get_cluster_count success: cnt = %u\n", cluster_cnt);

        ret = aipu_get_core_count(ctx, 0, 0, &core_cnt);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(ctx, ret, &msg);
            AIPU_ERR()("aipu_get_core_count: %s (%s)\n",
                msg, opt.bin_file_name);
            goto unload_graph;
        }
        //AIPU_INFO()("aipu_get_core_count success: cnt = %u\n", core_cnt);

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
        //AIPU_INFO()("aipu_get_tensor_descriptor done\n");

        #if DMABUF_SWITCH
        /**
         * @NOTE:
         * construct input share dma_buf's descriptor
         */
        AIPU_INFO()("recv producer dmabuf fd, wait ...\n");
        dmabuf_input_fd = recver(PRODUCER_SERVER_PATH);
        if (dmabuf_input_fd < 0)
        {
            AIPU_ERR() << "recver dmabuf_input_fd [fail]\n";
            goto unload_graph;
        }
        AIPU_INFO()("recv producer success\n");

        // dmabuf_dump_file("dmabuf_input.bin", dmabuf_input_fd, input_desc[0].size);

        /**
         * @dmabuf_fd: the fd of dma_buf
         * @offset_in_dmabuf: the start offset of valid data in dma_buf
         * @tensor_idx: the tensor index which the dma_buf will replace
         * @type: the replaced tensor type(input or output)
         */
        input_share_tensor.dmabuf_fd = dmabuf_input_fd;
        input_share_tensor.offset_in_dmabuf = 0;
        input_share_tensor.tensor_idx = 0;
        input_share_tensor.type = AIPU_TENSOR_TYPE_INPUT;
        input_share_tensor.shared_case_type = AIPU_SHARE_BUF_DMABUF;

        #if OUTPUT_SWITCH
        /**
         * @NOTE:
         * construct output share dma_buf's descriptor
         */
        dmabuf_output_fd = dmabuf_malloc(PAGE_ALIGN_SIZE(output_desc[0].size));
        if (dmabuf_output_fd < 0)
        {
            AIPU_ERR() << "dmabuf_malloc dmabuf for output [fail]\n";
            goto unload_graph;
        }
        AIPU_INFO()("dmabuf_malloc dmabuf for output success\n");

        output_share_tensor.dmabuf_fd = dmabuf_output_fd;
        output_share_tensor.offset_in_dmabuf = 0;
        output_share_tensor.tensor_idx = 0;
        output_share_tensor.type = AIPU_TENSOR_TYPE_OUTPUT;
        output_share_tensor.shared_case_type = AIPU_SHARE_BUF_DMABUF;
        #endif
        #endif

        ret = aipu_create_job(ctx, graph_id, &job_id, &create_job_cfg);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(ctx, ret, &msg);
            AIPU_ERR()("aipu_create_job: %s\n", msg);
            goto unload_graph;
        }
        AIPU_INFO()("aipu_create_job success\n");
        #if DMABUF_SWITCH
        /**
         * bind dma_buf to IO buffer after the job is created.
         */
        ret = aipu_specify_iobuf(ctx, job_id, &input_share_tensor);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(ctx, ret, &msg);
            AIPU_ERR()("aipu_specify_iobuf: input %s\n", msg);
            goto unload_graph;
        }
        AIPU_INFO()("aipu_specify_iobuf input success\n");

        #if OUTPUT_SWITCH
         /**
         * bind dma_buf to IO buffer after the job is created.
         */
        ret = aipu_specify_iobuf(ctx, job_id, &output_share_tensor);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(ctx, ret, &msg);
            AIPU_ERR()("aipu_specify_iobuf: output %s\n", msg);
            goto unload_graph;
        }
        AIPU_INFO()("aipu_specify_iobuf ouput success\n");
        #endif
        #endif

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
        ret = aipu_config_job(ctx, job_id, cfg_types, &mem_dump_config);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(ctx, ret, &msg);
            AIPU_ERR()("aipu_config_job: %s\n", msg);
            goto clean_job;
        }
        AIPU_INFO()("set dump config success\n");

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
            #if !DMABUF_SWITCH
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
            #endif

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

            #if OUTPUT_SWITCH
            // send output dmabuf's fd to comsumer
            if (sender(dmabuf_output_fd, CONSUMER_SERVER_PATH) < 0)
            {
                AIPU_ERR() << "send dmabuf_output_fd [fail]\n";
                goto clean_job;
            }
            AIPU_INFO()("sender success\n");
            #endif
            pass = check_result_helper(output_data, output_desc, opt.gts[0], opt.gts_size[0]);
        }

        input_desc.clear();
        output_desc.clear();

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

        #if DMABUF_SWITCH
        dmabuf_free(dmabuf_input_fd);
        #endif

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

        output_data.clear();
    }
    deinit_test_bench(&opt);
    return pass;
}

// Copyright (C) 2022-2023 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  main.cpp
 * @brief AIPU UMD test application: dmabuf mmap test
 */

/**
 * @brief request one dma_buf and filled it firstly, then
 *        specify this dma_buf as model's input tensor buffer.
 *
 */

#include <stdio.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
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

#define DEV_EXPORTER "/dev/aipu"

/**
 * the size of requested dma_buf,change it accordingly.
 */
#define DMABUF_SZ (1 << 20)

/**
 * the fd of requested dma_buf.
 */
int dmabuf_fd = 0;

/**
 * request one dma_buf from dma_buf exporter(NUP driver module),
 * record its fd to 'dmabuf_fd'.
 */
int dmabuf_malloc(uint64_t size)
{
    int ret = 0;
    int fd = 0;
    struct aipu_dma_buf_request dma_buf_req = {0};

    fd = open(DEV_EXPORTER, O_RDWR);
    if (fd < 0)
    {
        ret = -1;
        AIPU_ERR() << "open " << DEV_EXPORTER << " [fail]\n";
        goto out;
    }

    dma_buf_req.bytes = size;
    ret = ioctl(fd, AIPU_IOCTL_ALLOC_DMA_BUF, &dma_buf_req);
    if (ret < 0)
    {
        AIPU_ERR() << "ioctl " << DEV_EXPORTER << " [fail]\n";
        goto out;
    }

    dmabuf_fd = dma_buf_req.fd;

out:
    close(fd);
    return ret;
}

/**
 * free allocated dma_buf
 */
int dmabuf_free(int _fd)
{
    int ret = 0;
    int fd = 0;

    fd = open(DEV_EXPORTER, O_RDWR);
    if (fd < 0)
    {
        ret = -1;
        AIPU_ERR() << "open " << DEV_EXPORTER << " [fail]\n";
        goto out;
    }

    ret = ioctl(fd, AIPU_IOCTL_FREE_DMA_BUF, &_fd);
    if (ret < 0)
    {
        AIPU_ERR() << "ioctl " << DEV_EXPORTER << " [fail]\n";
        goto out;
    }

out:
    close(fd);
    return ret;
}

/**
 * map physical pages of requested dma_buf to user mode,
 * then fill its with input data which is taken as model's
 * input data.
 */
int dmabuf_fill(int fd, char *data, uint32_t size)
{
    int ret = 0;
    char *va = nullptr;

    va = (char *)mmap(NULL, DMABUF_SZ, PROT_READ|PROT_WRITE, MAP_SHARED, dmabuf_fd, 0);
    if (va == MAP_FAILED)
    {
        ret = -1;
        AIPU_ERR() << "mmap dmabuf [fail]\n";
        goto out;
    }

    memcpy(va, data, size);
    munmap(va, DMABUF_SZ);

out:
    return ret;
}

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
    int pass = 0, loop = 0, total_loop = 1;
    uint64_t cfg_types = 0;
    aipu_create_job_cfg create_job_cfg = {0};
    aipu_shared_tensor_info_t share_tensor;

    AIPU_CRIT() << "usage: ./aipu_simulation_test -b aipu.bin -i input0.bin -c output.bin -d ./\n";

    aipu_job_config_dump_t mem_dump_config;
    memset(&mem_dump_config, 0, sizeof(mem_dump_config));

    if(init_test_bench(argc, argv, &opt, "simulation_test"))
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

        /**
         * request one dma_buf as model's input tensor buffer,
         * and fill it with initial input data.
         */
        if (dmabuf_malloc(DMABUF_SZ) < 0)
        {
            AIPU_ERR() << "dmabuf_malloc [fail]\n";
            goto unload_graph;
        }

        if (dmabuf_fill(dmabuf_fd, opt.inputs[0], input_desc[0].size) != 0)
        {
            AIPU_ERR() << "dmabuf_fill [fail]\n";
            goto unload_graph;
        }

        /**
         * @NOTE:
         * construct share dma_buf's descriptor
         *
         * @dmabuf_fd: the fd of dma_buf
         * @offset_in_dmabuf: the start offset of valid data in dma_buf
         * @tensor_idx: the tensor index which the dma_buf will replace
         * @type: the replaced tensor type(input or output)
         */
        share_tensor.dmabuf_fd = dmabuf_fd;
        share_tensor.offset_in_dmabuf = 0;
        share_tensor.tensor_idx = 0;
        share_tensor.type = AIPU_TENSOR_TYPE_INPUT;

        ret = aipu_create_job(ctx, graph_id, &job_id, &create_job_cfg);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(ctx, ret, &msg);
            AIPU_ERR()("aipu_create_job: %s\n", msg);
            goto unload_graph;
        }
        AIPU_INFO()("aipu_create_job success\n");

        ret = aipu_specify_iobuf(ctx, job_id, &share_tensor);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(ctx, ret, &msg);
            AIPU_ERR()("aipu_specify_iobuf: %s\n", msg);
            goto unload_graph;
        }
        AIPU_INFO()("aipu_specify_iobuf success\n");

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
            #if 0
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

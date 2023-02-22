// Copyright (C) 2022 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  standard_api_impl.cpp
 * @brief AIPU User Mode Driver (UMD) Standard API implementation file
 */

#include <stdlib.h>
#include <assert.h>
#include "standard_api.h"
#include "graph_base.h"
#include "job_base.h"
#include "ctx_ref_map.h"
#include "type.h"
#include "utils/helper.h"
#include "utils/log.h"

static aipu_status_t api_get_graph(const aipu_ctx_handle_t* ctx, uint64_t graph_id, aipudrv::GraphBase** graph)
{
    aipudrv::CtxRefMap& ctx_map = aipudrv::CtxRefMap::get_ctx_map();
    aipudrv::MainContext* p_ctx = nullptr;

    assert(graph != nullptr);

    if (nullptr == ctx)
        return AIPU_STATUS_ERROR_NULL_PTR;

    p_ctx = ctx_map.get_ctx_ref(ctx->handle);
    if (nullptr == p_ctx)
        return AIPU_STATUS_ERROR_INVALID_CTX;

    *graph = p_ctx->get_graph_object(graph_id);
    if (nullptr == *graph)
        return AIPU_STATUS_ERROR_INVALID_GRAPH_ID;

    return AIPU_STATUS_SUCCESS;
}

static aipu_status_t api_get_job(const aipu_ctx_handle_t* ctx, uint64_t job_id, aipudrv::JobBase** job)
{
    aipudrv::CtxRefMap& ctx_map = aipudrv::CtxRefMap::get_ctx_map();
    aipudrv::MainContext* p_ctx = nullptr;

    assert(job != nullptr);

    if (nullptr == ctx)
        return AIPU_STATUS_ERROR_NULL_PTR;

    p_ctx = ctx_map.get_ctx_ref(ctx->handle);
    if (nullptr == p_ctx)
        return AIPU_STATUS_ERROR_INVALID_CTX;

    *job = p_ctx->get_job_object(job_id);
    if (nullptr == *job)
        return AIPU_STATUS_ERROR_INVALID_JOB_ID;

    return AIPU_STATUS_SUCCESS;
}

aipu_status_t aipu_get_error_message(const aipu_ctx_handle_t* ctx, aipu_status_t status, const char** msg)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    aipudrv::CtxRefMap& ctx_map = aipudrv::CtxRefMap::get_ctx_map();
    aipudrv::MainContext* p_ctx = nullptr;

    /**
     * special cases when aipu_ctx_handle is invalid, call static method
     * to return error message directly.
     */
    if ((nullptr == ctx) && (AIPU_STATUS_ERROR_OPEN_DEV_FAIL == status))
    {
        *msg = aipudrv::MainContext::get_static_msg(AIPU_STATUS_ERROR_OPEN_DEV_FAIL);
        ret = AIPU_STATUS_ERROR_OPEN_DEV_FAIL;
        goto finish;
    }

    if ((nullptr == ctx) || (nullptr == msg))
    {
        *msg = aipudrv::MainContext::get_static_msg(AIPU_STATUS_ERROR_NULL_PTR);
        ret = AIPU_STATUS_ERROR_NULL_PTR;
        goto finish;
    }

    p_ctx = ctx_map.get_ctx_ref(ctx->handle);
    if (nullptr == p_ctx)
    {
        *msg = aipudrv::MainContext::get_static_msg(AIPU_STATUS_ERROR_INVALID_CTX);
        ret = AIPU_STATUS_ERROR_INVALID_CTX;
    } else {
        ret = p_ctx->get_status_msg(status, msg);
    }

finish:
    return ret;
}

aipu_status_t aipu_init_context(aipu_ctx_handle_t** ctx)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    aipudrv::CtxRefMap& ctx_map = aipudrv::CtxRefMap::get_ctx_map();
    aipudrv::MainContext* p_ctx = nullptr;
    aipu_ctx_handle_t* ctx_handle = nullptr;
    uint32_t handle = 0;

    if (nullptr == ctx)
    {
        ret = AIPU_STATUS_ERROR_NULL_PTR;
        goto finish;
    }

    *ctx = nullptr;
    handle = ctx_map.create_ctx_ref();
    p_ctx = ctx_map.get_ctx_ref(handle);
    if (p_ctx == nullptr)
    {
        ret = AIPU_STATUS_ERROR_NULL_PTR;
        goto finish;
    }

    ret = p_ctx->init();
    if (AIPU_STATUS_SUCCESS != ret)
    {
        ctx_map.destroy_ctx_ref(handle);
    } else {
        /* success */
        ctx_handle = new aipu_ctx_handle_t;
        ctx_handle->handle = handle;
        *ctx = ctx_handle;
    }

finish:
    return ret;
}

aipu_status_t aipu_deinit_context(const aipu_ctx_handle_t* ctx)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    aipudrv::CtxRefMap& ctx_map = aipudrv::CtxRefMap::get_ctx_map();
    aipudrv::MainContext* p_ctx = nullptr;

    if (nullptr == ctx)
    {
        ret = AIPU_STATUS_ERROR_NULL_PTR;
        goto finish;
    }

    p_ctx = ctx_map.get_ctx_ref(ctx->handle);
    if (nullptr == p_ctx)
    {
        ret = AIPU_STATUS_ERROR_INVALID_CTX;
    } else {
        ret = p_ctx->deinit();
        if (AIPU_STATUS_SUCCESS == ret)
        {
            ctx_map.destroy_ctx_ref(ctx->handle);
            delete ctx;
        }
    }

finish:
    return ret;
}

aipu_status_t aipu_load_graph(const aipu_ctx_handle_t* ctx, const char* graph_file, uint64_t* id)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    aipudrv::CtxRefMap& ctx_map = aipudrv::CtxRefMap::get_ctx_map();
    aipudrv::MainContext* p_ctx = nullptr;

    if (nullptr == ctx)
    {
        ret = AIPU_STATUS_ERROR_NULL_PTR;
        goto finish;
    }

    p_ctx = ctx_map.get_ctx_ref(ctx->handle);
    if (nullptr == p_ctx)
        ret = AIPU_STATUS_ERROR_INVALID_CTX;
    else
        ret = p_ctx->load_graph(graph_file, id);

finish:
    return ret;
}

aipu_status_t aipu_load_graph_helper(const aipu_ctx_handle_t* ctx, const char* graph_buf, uint32_t graph_size, uint64_t* id)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    aipudrv::CtxRefMap& ctx_map = aipudrv::CtxRefMap::get_ctx_map();
    aipudrv::MainContext* p_ctx = nullptr;

    if (nullptr == ctx)
    {
        ret = AIPU_STATUS_ERROR_NULL_PTR;
        goto finish;
    }

    p_ctx = ctx_map.get_ctx_ref(ctx->handle);
    if (nullptr == p_ctx)
        ret = AIPU_STATUS_ERROR_INVALID_CTX;
    else
        ret = p_ctx->load_graph(graph_buf, graph_size, id);

finish:
    return ret;
}

aipu_status_t aipu_unload_graph(const aipu_ctx_handle_t* ctx, uint64_t graph)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    aipudrv::CtxRefMap& ctx_map = aipudrv::CtxRefMap::get_ctx_map();
    aipudrv::MainContext* p_ctx = nullptr;

    if (nullptr == ctx)
    {
        ret = AIPU_STATUS_ERROR_NULL_PTR;
        goto finish;
    }

    p_ctx = ctx_map.get_ctx_ref(ctx->handle);
    if (nullptr == p_ctx)
        ret = AIPU_STATUS_ERROR_INVALID_CTX;
    else
        ret = p_ctx->unload_graph(graph);

finish:
    return ret;
}

aipu_status_t aipu_create_job(const aipu_ctx_handle_t* ctx, uint64_t graph, uint64_t* job, aipu_create_job_cfg_t *config)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    aipudrv::CtxRefMap& ctx_map = aipudrv::CtxRefMap::get_ctx_map();
    aipudrv::MainContext* p_ctx = nullptr;

    if ((nullptr == ctx) || (nullptr == job))
    {
        ret = AIPU_STATUS_ERROR_NULL_PTR;
        goto finish;
    }

    if (!aipudrv::valid_graph_id(graph))
    {
        ret = AIPU_STATUS_ERROR_INVALID_GRAPH_ID;
        goto finish;
    }

    p_ctx = ctx_map.get_ctx_ref(ctx->handle);
    if (nullptr == p_ctx)
        ret = AIPU_STATUS_ERROR_INVALID_CTX;
    else
        ret = p_ctx->create_job(graph, job, config);

finish:
    return ret;
}

aipu_status_t aipu_finish_job(const aipu_ctx_handle_t* ctx, uint64_t job_id, int32_t time_out)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    aipudrv::JobBase* job = nullptr;
    aipu_job_status_t status;

    if (nullptr == ctx)
        return AIPU_STATUS_ERROR_NULL_PTR;

    if (!aipudrv::valid_job_id(job_id))
        return AIPU_STATUS_ERROR_INVALID_JOB_ID;

    ret = api_get_job(ctx, job_id, &job);
    if (AIPU_STATUS_SUCCESS != ret)
        return ret;

    ret = job->schedule();
   if (AIPU_STATUS_SUCCESS != ret)
        return ret;

    if (time_out <= 0)
        time_out = -1;

    ret = job->get_status_blocking(&status, time_out);
    if ((AIPU_STATUS_SUCCESS == ret) && (AIPU_JOB_STATUS_DONE != status))
        ret = AIPU_STATUS_ERROR_JOB_EXCEPTION;

    return ret;
}

aipu_status_t aipu_flush_job(const aipu_ctx_handle_t* ctx, uint64_t id)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    aipudrv::JobBase* job = nullptr;

    if (nullptr == ctx)
        return AIPU_STATUS_ERROR_NULL_PTR;

    if (!aipudrv::valid_job_id(id))
        return AIPU_STATUS_ERROR_INVALID_JOB_ID;

    ret = api_get_job(ctx, id, &job);
    if (AIPU_STATUS_SUCCESS != ret)
        return ret;

    /* callback to be implemented */
    return job->schedule();
}

aipu_status_t aipu_get_job_status(const aipu_ctx_handle_t* ctx, uint64_t id, aipu_job_status_t* status)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    aipudrv::JobBase* job = nullptr;

    if (nullptr == ctx)
        return AIPU_STATUS_ERROR_NULL_PTR;

    if (!aipudrv::valid_job_id(id))
        return AIPU_STATUS_ERROR_INVALID_JOB_ID;

    ret = api_get_job(ctx, id, &job);
    if (AIPU_STATUS_SUCCESS != ret)
        return ret;

    return job->get_status(status);
}

aipu_status_t aipu_clean_job(const aipu_ctx_handle_t* ctx, uint64_t id)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    aipudrv::GraphBase* graph = nullptr;

    if (nullptr == ctx)
        return AIPU_STATUS_ERROR_NULL_PTR;

    if (!aipudrv::valid_job_id(id))
        return AIPU_STATUS_ERROR_INVALID_JOB_ID;

    ret = api_get_graph(ctx, aipudrv::job_id2graph_id(id), &graph);
    if (AIPU_STATUS_SUCCESS != ret)
        return ret;

    return graph->destroy_job(id);
}

aipu_status_t aipu_get_tensor_count(const aipu_ctx_handle_t* ctx, uint64_t id, aipu_tensor_type_t type,
    uint32_t* cnt)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    aipudrv::GraphBase* graph = nullptr;

    if ((nullptr == ctx) || (nullptr == cnt))
        return AIPU_STATUS_ERROR_NULL_PTR;

    if (!aipudrv::valid_graph_id(id))
        return AIPU_STATUS_ERROR_INVALID_GRAPH_ID;

    ret = api_get_graph(ctx, aipudrv::get_graph_id(id), &graph);
    if (AIPU_STATUS_SUCCESS != ret)
        return ret;

    return graph->get_tensor_count(type, cnt);
}

aipu_status_t aipu_get_tensor_descriptor(const aipu_ctx_handle_t* ctx, uint64_t id, aipu_tensor_type_t type,
    uint32_t tensor, aipu_tensor_desc_t* desc)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    aipudrv::GraphBase* graph = nullptr;

    if (nullptr == ctx)
        return AIPU_STATUS_ERROR_NULL_PTR;

    if (!aipudrv::valid_graph_id(id))
        return AIPU_STATUS_ERROR_INVALID_GRAPH_ID;

    ret = api_get_graph(ctx, aipudrv::get_graph_id(id), &graph);
    if (AIPU_STATUS_SUCCESS != ret)
        return ret;

    return graph->get_tensor_descriptor(type, tensor, desc);
}

aipu_status_t aipu_load_tensor(const aipu_ctx_handle_t* ctx, uint64_t job_id, uint32_t tensor, const void* data)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    aipudrv::JobBase* job = nullptr;

    if (nullptr == ctx)
        return AIPU_STATUS_ERROR_NULL_PTR;

    if (!aipudrv::valid_job_id(job_id))
        return AIPU_STATUS_ERROR_INVALID_JOB_ID;

    ret = api_get_job(ctx, job_id, &job);
    if (AIPU_STATUS_SUCCESS != ret)
        return ret;

    return job->load_tensor(tensor, data);
}

aipu_status_t aipu_get_tensor(const aipu_ctx_handle_t* ctx, uint64_t job_id, aipu_tensor_type_t type, uint32_t tensor,
    void* data)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    aipudrv::JobBase* job = nullptr;

    if (nullptr == ctx)
        return AIPU_STATUS_ERROR_NULL_PTR;

    if (!aipudrv::valid_job_id(job_id))
        return AIPU_STATUS_ERROR_INVALID_JOB_ID;

    ret = api_get_job(ctx, job_id, &job);
    if (AIPU_STATUS_SUCCESS != ret)
        return ret;

    return job->get_tensor(type, tensor, data);
}

aipu_status_t aipu_get_partition_count(const aipu_ctx_handle_t* ctx, uint32_t* cnt)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    aipudrv::CtxRefMap& ctx_map = aipudrv::CtxRefMap::get_ctx_map();
    aipudrv::MainContext* p_ctx = nullptr;

    if (nullptr == ctx)
    {
        ret = AIPU_STATUS_ERROR_NULL_PTR;
        goto finish;
    }

    p_ctx = ctx_map.get_ctx_ref(ctx->handle);
    if (nullptr == p_ctx)
        ret = AIPU_STATUS_ERROR_INVALID_CTX;
    else
        ret = p_ctx->get_partition_count(cnt);

finish:
    return ret;
}

aipu_status_t aipu_get_cluster_count(const aipu_ctx_handle_t* ctx, uint32_t partition_id, uint32_t* cnt)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    aipudrv::CtxRefMap& ctx_map = aipudrv::CtxRefMap::get_ctx_map();
    aipudrv::MainContext* p_ctx = nullptr;

    if (nullptr == ctx)
    {
        ret = AIPU_STATUS_ERROR_NULL_PTR;
        goto finish;
    }

    p_ctx = ctx_map.get_ctx_ref(ctx->handle);
    if (nullptr == p_ctx)
        ret = AIPU_STATUS_ERROR_INVALID_CTX;
    else
        ret = p_ctx->get_cluster_count(partition_id, cnt);

finish:
    return ret;
}

aipu_status_t aipu_get_core_count(const aipu_ctx_handle_t* ctx, uint32_t partition_id, uint32_t cluster, uint32_t* cnt)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    aipudrv::CtxRefMap& ctx_map = aipudrv::CtxRefMap::get_ctx_map();
    aipudrv::MainContext* p_ctx = nullptr;

    if (nullptr == ctx)
    {
        ret = AIPU_STATUS_ERROR_NULL_PTR;
        goto finish;
    }

    p_ctx = ctx_map.get_ctx_ref(ctx->handle);
    if (nullptr == p_ctx)
        ret = AIPU_STATUS_ERROR_INVALID_CTX;
    else
        ret = p_ctx->get_core_count(partition_id, cluster, cnt);

finish:
    return ret;
}

aipu_status_t aipu_debugger_get_core_info(const aipu_ctx_handle_t* ctx, uint32_t id, aipu_core_info_t* info)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    aipudrv::CtxRefMap& ctx_map = aipudrv::CtxRefMap::get_ctx_map();
    aipudrv::MainContext* p_ctx = nullptr;

    if ((nullptr == ctx) || (nullptr == info))
    {
        ret = AIPU_STATUS_ERROR_NULL_PTR;
        goto finish;
    }

    p_ctx = ctx_map.get_ctx_ref(ctx->handle);
    if (nullptr == p_ctx)
        ret = AIPU_STATUS_ERROR_INVALID_CTX;
    else
        ret = p_ctx->get_core_info(id, info);

finish:
    return ret;
}

aipu_status_t aipu_debugger_get_job_info(const aipu_ctx_handle_t* ctx,
    uint64_t job, aipu_debugger_job_info_t* info)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    aipudrv::CtxRefMap& ctx_map = aipudrv::CtxRefMap::get_ctx_map();
    aipudrv::MainContext* p_ctx = nullptr;

    if (nullptr == ctx)
    {
        ret = AIPU_STATUS_ERROR_NULL_PTR;
        goto finish;
    }

    p_ctx = ctx_map.get_ctx_ref(ctx->handle);
    if (nullptr == p_ctx)
        ret = AIPU_STATUS_ERROR_INVALID_CTX;
    else
        ret = p_ctx->debugger_get_job_info(job, info);

finish:
    return ret;
}

aipu_status_t aipu_debugger_bind_job(const aipu_ctx_handle_t* ctx, uint32_t id, uint64_t job_id)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    aipudrv::JobBase* job = nullptr;

    if (nullptr == ctx)
        return AIPU_STATUS_ERROR_NULL_PTR;

    ret = api_get_job(ctx, job_id, &job);
    if (AIPU_STATUS_SUCCESS != ret)
        return ret;

    return job->bind_core(id);
}

aipu_status_t aipu_debugger_run_job(const aipu_ctx_handle_t* ctx, uint64_t job_id)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    aipudrv::JobBase* job = nullptr;

    if (nullptr == ctx)
        return AIPU_STATUS_ERROR_NULL_PTR;

    ret = api_get_job(ctx, job_id, &job);
    if (AIPU_STATUS_SUCCESS != ret)
        return ret;

    return job->debugger_run();
}

aipu_status_t aipu_debugger_malloc(const aipu_ctx_handle_t* ctx, uint32_t size, void** va)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    aipudrv::CtxRefMap& ctx_map = aipudrv::CtxRefMap::get_ctx_map();
    aipudrv::MainContext* p_ctx = nullptr;

    if (nullptr == ctx)
    {
        ret = AIPU_STATUS_ERROR_NULL_PTR;
        goto finish;
    }

    p_ctx = ctx_map.get_ctx_ref(ctx->handle);
    if (nullptr == p_ctx)
        ret = AIPU_STATUS_ERROR_INVALID_CTX;
    else
        ret = p_ctx->debugger_malloc(size, va);

finish:
    return ret;
}

aipu_status_t aipu_debugger_free(const aipu_ctx_handle_t* ctx, void* va)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    aipudrv::CtxRefMap& ctx_map = aipudrv::CtxRefMap::get_ctx_map();
    aipudrv::MainContext* p_ctx = nullptr;

    if (nullptr == ctx)
    {
        ret = AIPU_STATUS_ERROR_NULL_PTR;
        goto finish;
    }

    p_ctx = ctx_map.get_ctx_ref(ctx->handle);
    if (nullptr == p_ctx)
        ret = AIPU_STATUS_ERROR_INVALID_CTX;
    else
        ret = p_ctx->debugger_free(va);

finish:
    return ret;
}

aipu_status_t aipu_config_job(const aipu_ctx_handle_t* ctx, uint64_t job_id, uint64_t types, void* config)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    aipudrv::JobBase* job = nullptr;

    if (nullptr == ctx)
        return AIPU_STATUS_ERROR_NULL_PTR;

    if (!aipudrv::valid_job_id(job_id))
        return AIPU_STATUS_ERROR_INVALID_JOB_ID;

    ret = api_get_job(ctx, job_id, &job);
    if (AIPU_STATUS_SUCCESS != ret)
        return ret;

    if (types & (AIPU_JOB_CONFIG_TYPE_DUMP_TEXT |
         AIPU_JOB_CONFIG_TYPE_DUMP_WEIGHT       |
         AIPU_JOB_CONFIG_TYPE_DUMP_RODATA       |
         AIPU_JOB_CONFIG_TYPE_DUMP_DESCRIPTOR   |
         AIPU_JOB_CONFIG_TYPE_DUMP_INPUT        |
         AIPU_JOB_CONFIG_TYPE_DUMP_OUTPUT       |
         AIPU_JOB_CONFIG_TYPE_DUMP_REUSE        |
         AIPU_JOB_CONFIG_TYPE_DUMP_TCB_CHAIN    |
         AIPU_JOB_CONFIG_TYPE_DUMP_EMULATION    |
         AIPU_JOB_CONFIG_TYPE_DUMP_PROFILE))
        ret = job->config_mem_dump(types, (aipu_job_config_dump_t*)config);
    else if (types == AIPU_CONFIG_TYPE_SIMULATION)
        ret = job->config_simulation(types, (aipu_job_config_simulation_t*)config);
    else
        ret = AIPU_STATUS_ERROR_INVALID_CONFIG;

    return ret;
}

aipu_status_t aipu_config_global(const aipu_ctx_handle_t* ctx, uint64_t types, void* config)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    aipudrv::CtxRefMap& ctx_map = aipudrv::CtxRefMap::get_ctx_map();
    aipudrv::MainContext* p_ctx = nullptr;

    if (nullptr == ctx)
    {
        ret = AIPU_STATUS_ERROR_NULL_PTR;
        goto finish;
    }

    p_ctx = ctx_map.get_ctx_ref(ctx->handle);
    if (nullptr == p_ctx)
    {
        ret = AIPU_STATUS_ERROR_INVALID_CTX;
        goto finish;
    }

    if (types & AIPU_CONFIG_TYPE_HW)
    {
        ret = p_ctx->config_hw(types, (aipu_global_config_hw_t*)config);
        if (ret != AIPU_STATUS_SUCCESS)
            goto finish;

        types &= ~AIPU_CONFIG_TYPE_HW;
    } else if (types & AIPU_CONFIG_TYPE_SIMULATION) {
        ret = p_ctx->config_simulation(types, (aipu_global_config_simulation_t*)config);
        if (ret != AIPU_STATUS_SUCCESS)
            goto finish;

        types &= ~AIPU_CONFIG_TYPE_SIMULATION;
    }

    if (types & AIPU_GLOBAL_CONFIG_TYPE_DISABLE_VER_CHECK)
    {
        p_ctx->disable_version_check();
        types &= ~AIPU_GLOBAL_CONFIG_TYPE_DISABLE_VER_CHECK;
    }

    if (types & AIPU_GLOBAL_CONFIG_TYPE_ENABLE_VER_CHECK)
    {
        p_ctx->enable_version_check();
        types &= ~AIPU_GLOBAL_CONFIG_TYPE_ENABLE_VER_CHECK;
    }

    if (types)
        ret = AIPU_STATUS_ERROR_INVALID_CONFIG;

finish:
    return ret;
}

aipu_status_t aipu_import_buffers(const aipu_ctx_handle_t* ctx, uint64_t job_id, aipu_tensor_type_t type, int* fds)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    aipudrv::JobBase* job = nullptr;

    if (nullptr == ctx)
        return AIPU_STATUS_ERROR_NULL_PTR;

    ret = api_get_job(ctx, job_id, &job);
    if (AIPU_STATUS_SUCCESS != ret)
        return ret;

    return job->import_buffers(type, fds);
}

aipu_status_t aipu_export_buffers(const aipu_ctx_handle_t* ctx, uint64_t job_id, aipu_tensor_type_t type, int* fds)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    aipudrv::JobBase* job = nullptr;

    if (nullptr == ctx)
        return AIPU_STATUS_ERROR_NULL_PTR;

    ret = api_get_job(ctx, job_id, &job);
    if (AIPU_STATUS_SUCCESS != ret)
        return ret;

    return job->export_buffers(type, fds);
}

aipu_status_t aipu_get_target(const aipu_ctx_handle_t* ctx, char *target)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    aipudrv::CtxRefMap& ctx_map = aipudrv::CtxRefMap::get_ctx_map();
    aipudrv::MainContext* p_ctx = nullptr;

    if (nullptr == ctx)
    {
        ret = AIPU_STATUS_ERROR_NULL_PTR;
        goto finish;
    }

    p_ctx = ctx_map.get_ctx_ref(ctx->handle);
    if (nullptr == p_ctx)
    {
        ret = AIPU_STATUS_ERROR_INVALID_CTX;
        goto finish;
    }

    ret = p_ctx->aipu_get_target(target);

finish:
    return ret;
}

aipu_status_t aipu_get_device_status(const aipu_ctx_handle_t* ctx, device_status_t *status)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    aipudrv::CtxRefMap& ctx_map = aipudrv::CtxRefMap::get_ctx_map();
    aipudrv::MainContext* p_ctx = nullptr;

    if (nullptr == ctx)
    {
        ret = AIPU_STATUS_ERROR_NULL_PTR;
        goto finish;
    }

    p_ctx = ctx_map.get_ctx_ref(ctx->handle);
    if (nullptr == p_ctx)
    {
        ret = AIPU_STATUS_ERROR_INVALID_CTX;
        goto finish;
    }

    ret = p_ctx->aipu_get_device_status(status);
finish:
    return ret;
}

aipu_status_t aipu_config_batch_dump(const aipu_ctx_handle_t* ctx, uint64_t graph_id,
    uint32_t queue_id, uint64_t types, aipu_job_config_dump_t *config)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    aipudrv::GraphBase* graph = nullptr;

    if (nullptr == ctx)
        return AIPU_STATUS_ERROR_NULL_PTR;

    if (!aipudrv::valid_graph_id(graph_id))
        return AIPU_STATUS_ERROR_INVALID_GRAPH_ID;

    ret = api_get_graph(ctx, aipudrv::get_graph_id(graph_id), &graph);
    if (AIPU_STATUS_SUCCESS != ret)
        return ret;

    ret = graph->config_for_batch(queue_id, types, config);
    return ret;
}

aipu_status_t aipu_create_batch_queue(const aipu_ctx_handle_t* ctx, uint64_t graph_id,
    uint32_t *queue_id)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    aipudrv::GraphBase *graph = nullptr;

    if (nullptr == ctx)
        return AIPU_STATUS_ERROR_NULL_PTR;

    if (!aipudrv::valid_graph_id(graph_id))
        return AIPU_STATUS_ERROR_INVALID_GRAPH_ID;

    ret = api_get_graph(ctx, aipudrv::get_graph_id(graph_id), &graph);
    if (AIPU_STATUS_SUCCESS != ret)
       goto out;

    ret = graph->get_batch_queue_id(queue_id);

out:
    return ret;
}

aipu_status_t aipu_clean_batch_queue(const aipu_ctx_handle_t* ctx, uint64_t graph_id,
    uint32_t queue_id)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    aipudrv::GraphBase *graph = nullptr;

    if (nullptr == ctx)
        return AIPU_STATUS_ERROR_NULL_PTR;

    if (!aipudrv::valid_graph_id(graph_id))
        return AIPU_STATUS_ERROR_INVALID_GRAPH_ID;

    ret = api_get_graph(ctx, aipudrv::get_graph_id(graph_id), &graph);
    if (AIPU_STATUS_SUCCESS != ret)
       goto out;

    ret = graph->clean_batch_queue(queue_id);
out:
    return ret;
}

aipu_status_t aipu_add_batch(const aipu_ctx_handle_t* ctx, uint64_t graph_id, uint32_t queue_id,
    char *inputs[], char *outputs[])
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    aipudrv::GraphBase *graph = nullptr;
    uint32_t in_tensor_cnt = 0, out_tensor_cnt = 0;

    if (nullptr == ctx)
        return AIPU_STATUS_ERROR_NULL_PTR;

    if (!aipudrv::valid_graph_id(graph_id))
        return AIPU_STATUS_ERROR_INVALID_GRAPH_ID;

    ret = api_get_graph(ctx, aipudrv::get_graph_id(graph_id), &graph);
    if (AIPU_STATUS_SUCCESS != ret)
       goto out;

    in_tensor_cnt = graph->m_input_cnt;
    out_tensor_cnt = graph->m_output_cnt;

    if (in_tensor_cnt == 0)
    {
        ret = graph->get_tensor_count(AIPU_TENSOR_TYPE_INPUT, &in_tensor_cnt);
        if (AIPU_STATUS_SUCCESS != ret)
           goto out;

        graph->m_input_cnt = in_tensor_cnt;
    }

    if (out_tensor_cnt == 0)
    {
        ret = graph->get_tensor_count(AIPU_TENSOR_TYPE_OUTPUT, &out_tensor_cnt);
        if (AIPU_STATUS_SUCCESS != ret)
           goto out;

        graph->m_output_cnt = out_tensor_cnt;
    }

    ret = graph->add_batch(queue_id, inputs, in_tensor_cnt, outputs, out_tensor_cnt);
out:
    return ret;
}

aipu_status_t aipu_finish_batch(const aipu_ctx_handle_t* ctx, uint64_t graph_id,
    uint32_t queue_id, aipu_create_job_cfg_t *config)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    aipudrv::CtxRefMap& ctx_map = aipudrv::CtxRefMap::get_ctx_map();
    aipudrv::MainContext* p_ctx = nullptr;
    aipudrv::GraphBase* graph = nullptr;

    if (nullptr == ctx || nullptr == config)
        return AIPU_STATUS_ERROR_NULL_PTR;

    p_ctx = ctx_map.get_ctx_ref(ctx->handle);
    if (nullptr == p_ctx)
        return AIPU_STATUS_ERROR_INVALID_CTX;

    if (!aipudrv::valid_graph_id(graph_id))
        return AIPU_STATUS_ERROR_INVALID_GRAPH_ID;

    ret = api_get_graph(ctx, aipudrv::get_graph_id(graph_id), &graph);
    if (AIPU_STATUS_SUCCESS != ret)
        return ret;

    ret = p_ctx->run_batch(*graph, queue_id, config);
    return ret;
}

aipu_status_t aipu_ioctl(aipu_ctx_handle_t *ctx, uint32_t cmd, void *arg)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    aipudrv::CtxRefMap& ctx_map = aipudrv::CtxRefMap::get_ctx_map();
    aipudrv::MainContext* p_ctx = nullptr;

    if (nullptr == ctx)
        return AIPU_STATUS_ERROR_NULL_PTR;

    p_ctx = ctx_map.get_ctx_ref(ctx->handle);
    if (nullptr == p_ctx)
        return AIPU_STATUS_ERROR_INVALID_CTX;

    ret = p_ctx->ioctl_cmd(cmd, arg);
    return ret;
}
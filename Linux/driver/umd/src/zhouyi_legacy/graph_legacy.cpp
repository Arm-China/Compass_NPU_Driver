// Copyright (C) 2022 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  graph_legacy.cpp
 * @brief AIPU User Mode Driver (UMD) z1/2/3 legacy graph module implementation
 */

#include <cstring>
#include "context.h"
#include "graph_legacy.h"
#include "parser_legacy.h"
#include "job_legacy.h"
#include "utils/helper.h"
#include "utils/log.h"

aipudrv::GraphLegacy::GraphLegacy(void* ctx, GRAPH_ID id, DeviceBase* dev): Graph(ctx, id, dev)
{
    m_parser = new ParserLegacy();
}

aipudrv::GraphLegacy::~GraphLegacy()
{
    unload();
    delete m_parser;
}

aipu_status_t aipudrv::GraphLegacy::create_job(JOB_ID* id, const aipu_global_config_simulation_t* cfg,
    aipu_create_job_cfg_t *config)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    JobLegacy* job = new JobLegacy((MainContext*)m_ctx, *this, m_dev, config);

    ret = job->init(cfg);
    if (AIPU_STATUS_SUCCESS != ret)
        return ret;

    *id = add_job(job);
    return ret;
}

aipu_status_t aipudrv::GraphLegacy::get_tensor_count(aipu_tensor_type_t type, uint32_t* cnt)
{
    if (nullptr == cnt)
        return AIPU_STATUS_ERROR_NULL_PTR;

    switch (type)
    {
        case AIPU_TENSOR_TYPE_INPUT:
            *cnt = (uint32_t)m_io.inputs.size();
            break;
        case AIPU_TENSOR_TYPE_OUTPUT:
            *cnt = (uint32_t)m_io.outputs.size();
            break;
        case AIPU_TENSOR_TYPE_PRINTF:
            *cnt = (uint32_t)m_io.printf.size();
            break;
        case AIPU_TENSOR_TYPE_PROFILER:
            *cnt = (uint32_t)m_io.profiler.size();
            break;
        case AIPU_TENSOR_TYPE_INTER_DUMP:
            *cnt = (uint32_t)m_io.inter_dumps.size();
            break;
        default:
            LOG(LOG_WARN, "no tensor with type: %d\n", type);
            return AIPU_STATUS_ERROR_INVALID_TENSOR_TYPE;
    }

    return AIPU_STATUS_SUCCESS;
}

aipu_status_t aipudrv::GraphLegacy::get_tensor_descriptor(aipu_tensor_type_t type, uint32_t tensor, aipu_tensor_desc_t* desc)
{
    uint32_t cnt = 0;
    GraphIOTensorDesc io;

    get_tensor_count(type, &cnt);
    if (tensor >= cnt)
        return AIPU_STATUS_ERROR_INVALID_TENSOR_ID;

    if (nullptr == desc)
        return AIPU_STATUS_ERROR_NULL_PTR;

    switch (type)
    {
        case AIPU_TENSOR_TYPE_INPUT:
            io = m_io.inputs[tensor];
            break;
        case AIPU_TENSOR_TYPE_OUTPUT:
            io = m_io.outputs[tensor];
            break;
        case AIPU_TENSOR_TYPE_PRINTF:
            io = m_io.printf[tensor];
            break;
        case AIPU_TENSOR_TYPE_PROFILER:
            io = m_io.profiler[tensor];
            break;
        case AIPU_TENSOR_TYPE_INTER_DUMP:
            io = m_io.inter_dumps[tensor];
            break;
        default:
            return AIPU_STATUS_ERROR_INVALID_TENSOR_ID;
    }

    desc->id = tensor;
    desc->size = io.size;
    desc->scale = io.scale;
    desc->zero_point = io.zero_point;
    desc->data_type = io.data_type;

    return AIPU_STATUS_SUCCESS;
}

aipu_status_t aipudrv::GraphLegacy::assign_shared_tensor(aipu_tensor_type_t type,
    uint32_t tensor_idx, uint64_t shared_pa_addr)
{
    std::vector<struct GraphIOTensorDesc> *iobuffer_vec = nullptr;

    switch (type)
    {
        case AIPU_TENSOR_TYPE_INPUT:
            iobuffer_vec = &m_io.inputs;
            break;

        case AIPU_TENSOR_TYPE_OUTPUT:
            iobuffer_vec = &m_io.outputs;
            break;

        default:
            return AIPU_STATUS_ERROR_INVALID_OP;
    }

    if (tensor_idx >= iobuffer_vec->size())
        return AIPU_STATUS_ERROR_INVALID_TENSOR_ID;

    m_shared_tensor_map[iobuffer_vec->at(tensor_idx).ref_section_iter] = shared_pa_addr;

    return AIPU_STATUS_SUCCESS;
}
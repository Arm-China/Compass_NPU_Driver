// Copyright (C) 2022 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  graph_v3.cpp
 * @brief AIPU User Mode Driver (UMD) x2 graph module implementation
 */

#include <cstring>
#include "context.h"
#include "graph_v3.h"
#include "job_v3.h"
#include "parser_elf.h"
#include "utils/helper.h"
#include "utils/log.h"

aipudrv::GraphV3::GraphV3(void* ctx, GRAPH_ID id, DeviceBase* dev): Graph(ctx, id, dev)
{
    m_parser = new ParserELF();
}

aipudrv::GraphV3::~GraphV3()
{
    unload();
    delete m_parser;
}

void aipudrv::GraphV3::print_parse_info()
{
    LOG(LOG_DEFAULT, "=====================Graph Parse Results====================");
    LOG(LOG_DEFAULT, "Target device: z%u-%u", m_hw_version, m_hw_config);
    LOG(LOG_DEFAULT, "--Text:      size 0x%lx", m_btext.size);
    LOG(LOG_DEFAULT, "--Rodata:    size 0x%lx", m_brodata.size);
    LOG(LOG_DEFAULT, "--DCR:       size 0x%lx", m_bdesc.size);
    LOG(LOG_DEFAULT, "--Weight:    size 0x%lx", m_bweight.size);
    LOG(LOG_DEFAULT, "--Data (CC): size 0x%lx", m_bdata.size);
    LOG(LOG_DEFAULT, "--Remap:     cnt  0x%lx", m_remap.size());
    LOG(LOG_DEFAULT, "--Subgraph:  cnt  0x%lx", m_subgraphs.size());
    for (uint32_t i = 0; i < m_subgraphs.size(); i++)
    {
        LOG(LOG_DEFAULT, "[subgraph #%d]\n", m_subgraphs[i].id);
        LOG(LOG_DEFAULT, "--Text:       offset 0x%lx, size 0x%lx", m_subgraphs[i].text.offset, m_subgraphs[i].text.size);
        LOG(LOG_DEFAULT, "--Rodata:     offset 0x%lx, size 0x%lx", m_subgraphs[i].rodata.offset, m_subgraphs[i].rodata.size);
        LOG(LOG_DEFAULT, "--DCR:        offset 0x%lx, size 0x%lx", m_subgraphs[i].dcr.offset, m_subgraphs[i].dcr.size);
        LOG(LOG_DEFAULT, "--printf:     size 0x%x", m_subgraphs[i].printfifo_size);
        LOG(LOG_DEFAULT, "--profiler:   size 0x%x", m_subgraphs[i].profiler_buf_size);
        LOG(LOG_DEFAULT, "--precursors: size 0x%lx", m_subgraphs[i].precursors.size());
        LOG(LOG_DEFAULT, "--stack:      size 0x%x, align 0x%x", m_subgraphs[i].stack_size, m_subgraphs[i].stack_align_in_page);
        LOG(LOG_DEFAULT, "--static:     cnt 0x%lx", m_subgraphs[i].static_sections.size());
        for (uint32_t j = 0; j < m_subgraphs[i].static_sections.size(); j++)
        {
            LOG(LOG_DEFAULT, "----static section [%d]: size 0x%x, align 0x%x",
                j, m_subgraphs[i].static_sections[j].size, m_subgraphs[i].static_sections[j].align_in_page);
            for (uint32_t k = 0; k < m_subgraphs[i].static_sections[j].sub_sections.size(); k++)
            {
                LOG(LOG_DEFAULT, "------subsection [%d]: offset 0x%x",
                    k, m_subgraphs[i].static_sections[j].sub_sections[k].offset_in_section);
            }
        }
        LOG(LOG_DEFAULT, "--reuse:      cnt 0x%lx", m_subgraphs[i].reuse_sections.size());
        for (uint32_t j = 0; j < m_subgraphs[i].reuse_sections.size(); j++)
        {
            LOG(LOG_DEFAULT, "----reuse section [%d]: size 0x%x, align 0x%x",
                j, m_subgraphs[i].reuse_sections[j].size, m_subgraphs[i].reuse_sections[j].align_in_page);
            for (uint32_t k = 0; k < m_subgraphs[i].reuse_sections[j].sub_sections.size(); k++)
            {
                LOG(LOG_DEFAULT, "------subsection [%d]: offset 0x%x",
                    k, m_subgraphs[i].reuse_sections[j].sub_sections[k].offset_in_section);
            }
        }
    }
    LOG(LOG_DEFAULT, "============================================================");
}

aipu_status_t aipudrv::GraphV3::extract_gm_info(int sg_id)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    GMConfig *gmconfig = nullptr;
    GM_info_desc gm_info_desc = {0};

    if (m_gmconfig.size() == 0)
        goto out;

    gmconfig = (GMConfig *)&m_gmconfig[0];

    if (gmconfig->GM_buf_idx[0].fm_index != 0 || gmconfig->GM_buf_idx[1].fm_index != 0)
    {
        LOG(LOG_WARN, "beyond fm scope: gm0_fm: %d, gm1_fm: %d\n",
            gmconfig->GM_buf_idx[0].fm_index, gmconfig->GM_buf_idx[1].fm_index);
        ret = AIPU_STATUS_ERROR_INVALID_GM;
        goto out;
    }

    for (uint32_t i = 0; i < 2; i++)
    {
        if (gmconfig->GM_region_ctrl[i] == 0)
            continue;

        gm_info_desc.gm_buf_idx = gmconfig->GM_buf_idx[i];
        gm_info_desc.gm_buf_type = GM_SUB_BUF_TYPE_IGNORE; // change it according to the condition
        if (gmconfig->GM_buf_idx[i].buf_type == GM_BUF_TYPE_REUSE)
        {
            for (auto item : m_subgraphs[sg_id].io.inputs)
            {
                if (gmconfig->GM_buf_idx[i].buf_index == item.ref_section_iter)
                {
                    gm_info_desc.gm_buf_type = GM_SUB_BUF_TYPE_INPUT;

                    /* check whether one buffer is used as input and output */
                    for (auto out_item : m_subgraphs[sg_id].io.outputs)
                    {
                        if (gmconfig->GM_buf_idx[i].buf_index == out_item.ref_section_iter)
                        {
                            gm_info_desc.gm_buf_type = GM_SUB_BUF_TYPE_INOUT;
                            break;
                        }
                    }

                    goto match;
                }
            }

            for (auto item : m_subgraphs[sg_id].io.outputs)
            {
                if (gmconfig->GM_buf_idx[i].buf_index == item.ref_section_iter)
                {
                    gm_info_desc.gm_buf_type = GM_SUB_BUF_TYPE_OUTPUT;
                    goto match;
                }
            }

            /**
             * if the buffer isn't input or output, it must be an internal temp reuse buffer
             */
            if (gmconfig->GM_buf_idx[i].buf_index < m_subgraphs[sg_id].reuse_sections.size())
                gm_info_desc.gm_buf_type = GM_SUB_BUF_TYPE_TEMP;

        } else if (gmconfig->GM_buf_idx[i].buf_type == GM_BUF_TYPE_WEIGHT) {
            if (gmconfig->GM_buf_idx[i].buf_index < m_subgraphs[sg_id].static_sections.size())
            {
                gm_info_desc.gm_buf_type = GM_SUB_BUF_TYPE_IGNORE;
            }
        } else {
            LOG(LOG_ERR, "GM buffer type: [%d] invalid\n", gmconfig->GM_buf_idx[i].buf_type);
            ret = AIPU_STATUS_ERROR_INVALID_GM;
            goto out;
        }

    match:
        if (gm_info_desc.gm_buf_type != GM_SUB_BUF_TYPE_IGNORE)
            m_gm_info[gmconfig->GM_buf_idx[i].buf_type][gmconfig->GM_buf_idx[i].buf_index] = gm_info_desc;
    }

out:
    return ret;
}

aipu_status_t aipudrv::GraphV3::create_job(JOB_ID* id, const aipu_global_config_simulation_t* glb_sim_cfg,
    aipu_global_config_hw_t *hw_cfg, aipu_create_job_cfg_t *job_config)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    JobV3 *job = nullptr;
    uint32_t part_cnt = 0;

    if (nullptr == job_config)
        return AIPU_STATUS_ERROR_NULL_PTR;

    m_dev->get_partition_count(&part_cnt);
    if (job_config->partition_id > part_cnt)
        return AIPU_STATUS_ERROR_INVALID_PARTITION_ID;

    if (job_config->qos_level > AIPU_JOB_QOS_HIGH)
        return AIPU_STATUS_ERROR_INVALID_QOS;

    job = new JobV3((MainContext*)m_ctx, *this, m_dev, job_config);
    ret = job->init(glb_sim_cfg, hw_cfg);
    if (AIPU_STATUS_SUCCESS != ret)
        return ret;

    *id = add_job(job);
    return ret;
}

aipu_status_t aipudrv::GraphV3::get_tensor_count(aipu_tensor_type_t type, uint32_t* cnt)
{
    if (nullptr == cnt)
        return AIPU_STATUS_ERROR_NULL_PTR;

    switch (type)
    {
        case AIPU_TENSOR_TYPE_INPUT:
            *cnt = (uint32_t)m_subgraphs[0].io.inputs.size();
            break;
        case AIPU_TENSOR_TYPE_OUTPUT:
            *cnt = (uint32_t)m_subgraphs[0].io.outputs.size();
            break;
        case AIPU_TENSOR_TYPE_PRINTF:
            *cnt = (uint32_t)m_subgraphs[0].io.printf.size();
            break;
        case AIPU_TENSOR_TYPE_PROFILER:
            *cnt = (uint32_t)m_subgraphs[0].io.profiler.size();
            break;
        default:
            LOG(LOG_WARN, "no tensor with type: %d\n", type);
            return AIPU_STATUS_ERROR_INVALID_TENSOR_TYPE;
    }

    return AIPU_STATUS_SUCCESS;
}

aipu_status_t aipudrv::GraphV3::get_tensor_descriptor(aipu_tensor_type_t type, uint32_t tensor, aipu_tensor_desc_t* desc)
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
            io = m_subgraphs[0].io.inputs[tensor];
            break;
        case AIPU_TENSOR_TYPE_OUTPUT:
            io = m_subgraphs[0].io.outputs[tensor];
            break;
        case AIPU_TENSOR_TYPE_PRINTF:
            io = m_subgraphs[0].io.printf[tensor];
            break;
        case AIPU_TENSOR_TYPE_PROFILER:
            io = m_subgraphs[0].io.profiler[tensor];
            break;
        default:
            LOG(LOG_WARN, "no tensor with type: %d\n", type);
            return AIPU_STATUS_ERROR_INVALID_TENSOR_ID;
    }

    desc->id = tensor;
    desc->size = io.size;
    desc->scale = io.scale;
    desc->zero_point = io.zero_point;
    desc->data_type = io.data_type;

    return AIPU_STATUS_SUCCESS;
}

aipu_status_t aipudrv::GraphV3::assign_shared_tensor(aipu_tensor_type_t type,
    uint32_t tensor_idx, uint64_t shared_pa_addr)
{
    std::vector<struct GraphIOTensorDesc> *iobuffer_vec = nullptr;

    switch (type)
    {
        case AIPU_TENSOR_TYPE_INPUT:
            iobuffer_vec = &m_subgraphs[0].io.inputs;
            break;

        case AIPU_TENSOR_TYPE_OUTPUT:
            iobuffer_vec = &m_subgraphs[0].io.outputs;
            break;

        default:
            return AIPU_STATUS_ERROR_INVALID_OP;
    }

    if (tensor_idx >= iobuffer_vec->size())
        return AIPU_STATUS_ERROR_INVALID_TENSOR_ID;

    m_shared_tensor_map[iobuffer_vec->at(tensor_idx).ref_section_iter] = shared_pa_addr;

    return AIPU_STATUS_SUCCESS;
}
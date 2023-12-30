// Copyright (C) 2023-2024 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

#include <cstring>
#include "job_v4.h"
#include "../common/graph_v3x.h"
#include "parser_base.h"
#include "utils/helper.h"

#if defined(SIMULATION) && defined(ZHOUYI_V4)
#include "simulator/simulator_v4.h"
#endif

aipudrv::GM_V4::GM_V4(JobV4 &_job) : m_job(_job), m_graph(m_job.get_graph())
{
    for (uint32_t type = GM_BUF_TYPE_REUSE; type < GM_BUF_TYPE_MAX; type++)
    {
        if (m_graph.m_gm_info[GM_BUF_TYPE_REUSE].size() > 0)
        {
            m_job.m_gm_info[type].insert(m_graph.m_gm_info[type].begin(),
                m_graph.m_gm_info[type].end());
        }
    }
}

aipudrv::GM_V4::~GM_V4()
{
    for (auto buf : m_gm_free_buffer)
        m_job.m_mem->free(&buf);
}

void aipudrv::GM_V4::gm_dynamic_switch(uint32_t core_cnt)
{
    /**
     * set this env variable as true(y/Y), small model containing only
     * one subgraph can use GM_V4. but if hope multiple small models
     * parallel to run, it has to be set as false(n/N).
     */
    const char *gm_allow_small_model = getenv("UMD_GM_ASM");

    if (gm_allow_small_model != nullptr)
    {
        if (gm_allow_small_model[0] == 'y' || gm_allow_small_model[0] == 'Y')
            m_gm_asm = true;
    } else {
        if (core_cnt > 1)
            m_gm_asm = false;
        else if (core_cnt == 1)
            m_gm_asm = true;
    }
}

aipu_status_t aipudrv::GM_V4::gm_malloc(uint32_t sg_id, uint32_t idx, uint32_t buf_type,
    std::string &buf_name, BufferDesc *buf)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    uint32_t asid = 0;
    uint32_t mem_region = AIPU_BUF_REGION_DEFAULT;
    const std::vector<struct GraphSectionDesc> &section_desc = (buf_type == GM_BUF_TYPE_REUSE)
        ? m_graph.get_subgraph(sg_id).reuse_sections : m_graph.get_subgraph(sg_id).static_sections;
    uint32_t buf_size = section_desc[idx].size;

    buf_size = (buf_size + (1 << 18)) & (1 << 18); // 256KB alignment

    ret = m_job.m_mem->malloc(buf_size,
        section_desc[idx].align_in_page, &buf, buf_name.c_str(), (asid << 8) | mem_region);
    if (AIPU_STATUS_SUCCESS != ret)
        goto out;

    /* record and free weight buffer, the reuse buffer is freed in another path */
    if (buf_type == GM_BUF_TYPE_WEIGHT)
        m_gm_free_buffer.push_back(buf);

    /* ignore reuse temp buffer (non input/output) */
    if (buf_type == GM_BUF_TYPE_REUSE &&
        m_job.m_gm_info[buf_type][idx].gm_buf_type == GM_SUB_BUF_TYPE_IGNORE)
        goto out;

    get_valid_map_base(*buf);
    if (m_job.m_gm_info[buf_type][idx].gm_buf_type == GM_SUB_BUF_TYPE_TEMP)
        goto out;

    for (int _buf_typ = EM_GM_BUF_INPUT; _buf_typ < EM_GM_BUF_MAX; _buf_typ++)
    {
        m_gm_buf_map_size = buf_size;
    }

out:
    return ret;
}

bool aipudrv::GM_V4::gm_is_gm_buffer(uint32_t idx, uint32_t buf_type)
{
    bool ret = false;

    if (!m_job.m_mem->is_gm_enable())
        goto out;

    if (m_job.m_qos == AIPU_JOB_QOS_HIGH && !m_job.m_mem->is_both_gm_region_enable())
        goto out;

    if (m_job.m_gm_info[buf_type].count(idx) != 1)
        goto out;

    if (!m_gm_asm && m_job.m_sg_cnt == 1)
    {
        m_job.m_gm_info[buf_type].erase(idx);
        goto out;
    }

    ret = true;
out:
    return ret;
}

bool aipudrv::GM_V4::gm_need_remap()
{
    if (m_gm_buf_map_size != 0)
        return true;

    // is a temp buffer, non input/output
    if (m_gm_buf_base != 0)
        return true;

    return false;
}

void aipudrv::GM_V4::get_valid_map_base(BufferDesc &buf)
{
    if (m_gm_alloc_buffer.size() == 0)
        m_gm_buf_base = buf.pa;
    m_gm_alloc_buffer.push_back(&buf);
}
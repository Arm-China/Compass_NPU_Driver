// Copyright (C) 2023-2024 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

#include <cstring>
#include "job_v3_1.h"
#include "../common/graph_v3x.h"
#include "parser_base.h"
#include "utils/helper.h"

#if defined(SIMULATION) && defined(ZHOUYI_V3_1)
#include "simulator/simulator_v3_1.h"
#endif

aipudrv::GM_V3_1::GM_V3_1(JobV3_1 &_job) : m_job(_job), m_graph(m_job.get_graph())
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

aipudrv::GM_V3_1::~GM_V3_1()
{
    for (auto buf : m_gm_free_buffer)
        m_job.m_mem->free(&buf);
}

aipu_status_t aipudrv::GM_V3_1::gm_malloc(uint32_t sg_id, uint32_t idx, uint32_t buf_type,
    std::string &buf_name, BufferDesc *buf)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    uint32_t asid = 0;
    uint32_t mem_region = AIPU_BUF_REGION_DEFAULT;
    const std::vector<struct GraphSectionDesc> &section_desc = (buf_type == GM_BUF_TYPE_REUSE)
        ? m_graph.get_bss(sg_id).reuse_sections : m_graph.get_bss(sg_id).static_sections;
    uint32_t buf_size = section_desc[idx].size;

    buf_size = (buf_size + ((1 << 18) - 1)) & (~((1 << 18) - 1)); // 256KB alignment

    ret = m_job.m_mem->malloc(buf_size,
        section_desc[idx].align_in_page, &buf, buf_name.c_str(), (asid << 8) | mem_region);
    if (ret != AIPU_STATUS_SUCCESS)
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
        m_gm_buf_sync_size = buf_size;
    }

out:
    return ret;
}

bool aipudrv::GM_V3_1::gm_is_gm_buffer(uint32_t idx, uint32_t buf_type)
{
    bool ret = false;

    if (!m_job.m_mem->is_gm_enable())
        goto out;

    if (m_job.m_gm_info[buf_type].count(idx) != 1)
        goto out;

    ret = true;
out:
    return ret;
}

bool aipudrv::GM_V3_1::gm_need_remap()
{
    if (m_gm_buf_sync_size != 0)
        return true;

    // is a temp buffer, non input/output
    if (m_gm_buf_base != 0)
        return true;

    return false;
}

void aipudrv::GM_V3_1::get_valid_map_base(BufferDesc &buf)
{
    if (m_gm_alloc_buffer.size() == 0)
        m_gm_buf_base = buf.pa;
    m_gm_alloc_buffer.push_back(&buf);
}
// Copyright (C) 2022 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0

#include <cstring>
#include "job_v3.h"
#include "graph_v3.h"
#include "parser_base.h"
#include "utils/helper.h"

#if defined(SIMULATION) && defined(ZHOUYI_V3)
#include "simulator/simulator_v3.h"
#endif

aipudrv::GM_V3::GM_V3(JobV3 &_job) : m_job(_job), m_graph(m_job.get_graph())
{
    /**
     * set this env variable as true(y/Y), small model containing only
     * one subgraph can use GM. but if hope multiple small models
     * parallel to run, it has to be set as false(n/N).
     */
    const char *gm_allow_small_model = getenv("UMD_GM_ASM");

    if (gm_allow_small_model != nullptr)
    {
        if (gm_allow_small_model[0] == 'y' || gm_allow_small_model[0] == 'Y')
            m_gm_asm = true;
    }

    for (uint32_t type = GM_BUF_TYPE_REUSE; type < GM_BUF_TYPE_MAX; type++)
    {
        if (m_graph.m_gm_info[GM_BUF_TYPE_REUSE].size() > 0)
        {
            m_job.m_gm_info[type].insert(m_graph.m_gm_info[type].begin(),
                m_graph.m_gm_info[type].end());
        }
    }
}

aipudrv::GM_V3::~GM_V3()
{
    for (auto buf : m_gm_free_buffer)
        m_job.m_mem->free(buf);
}

aipu_status_t aipudrv::GM_V3::gm_malloc(uint32_t sg_id, uint32_t idx, uint32_t buf_type,
    std::string &buf_name, BufferDesc &buf)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    uint32_t asid = 0;
    ValidSyncBuffer io_region = {0};
    uint32_t mem_region = AIPU_BUF_REGION_DEFAULT;
    const std::vector<struct GraphSectionDesc> &section_desc = (buf_type == GM_BUF_TYPE_REUSE)
        ? m_graph.get_subgraph(sg_id).reuse_sections : m_graph.get_subgraph(sg_id).static_sections;
    uint32_t gm_region_sz = (m_job.m_qos == AIPU_JOB_QOS_SLOW)
        ? m_job.m_mem->get_gm_size(AIPU_JOB_QOS_SLOW) : m_job.m_mem->get_gm_size(AIPU_JOB_QOS_HIGH);
    uint32_t buf_size = section_desc[idx].size, gm_size = 0;

    /* check which GM region the buffer comes from */
    #if 0
    if (m_job.m_sg_cnt == 1)
    {
        if (m_job.m_qos == AIPU_JOB_QOS_SLOW)
            mem_region = AIPU_BUF_REGION_QOS_SLOW_GM;
        else if (m_job.m_qos == AIPU_JOB_QOS_HIGH)
            mem_region = AIPU_BUF_REGION_QOS_FAST_GM;
    }
    #endif

    if (m_job.m_qos == AIPU_JOB_QOS_HIGH)
        gm_size = m_job.m_mem->get_gm_size(1);
    else
        gm_size = m_job.m_mem->get_gm_size(0);

    if (buf_size < gm_size)
        buf_size = gm_size;

    ret = m_job.m_mem->malloc(buf_size,
        section_desc[idx].align_in_page, &buf, buf_name.c_str(), (asid << 8) | mem_region);
    if (AIPU_STATUS_SUCCESS != ret)
        goto out;

    /* record and free weight buffer, the reuse buffer is freed in another path */
    if (buf_type == GM_BUF_TYPE_WEIGHT)
        m_gm_free_buffer.push_back(&buf);

    /* ignore reuse temp buffer (non input/output) */
    if (buf_type == GM_BUF_TYPE_REUSE &&
        m_job.m_gm_info[buf_type][idx].gm_buf_type == GM_SUB_BUF_TYPE_IGNORE)
        goto out;

    get_valid_map_base(buf);
    if (m_job.m_gm_info[buf_type][idx].gm_buf_type == GM_SUB_BUF_TYPE_TEMP)
        goto out;

    get_valid_sync_region(sg_id, idx, buf_type, buf, io_region);
    for (int _buf_typ = EM_GM_BUF_INPUT; _buf_typ < EM_GM_BUF_MAX; _buf_typ++)
    {
        DEV_PA_64 sync_pa = io_region.valid_sync_buf[_buf_typ].sync_pa;
        uint32_t sync_sz = io_region.valid_sync_buf[_buf_typ].sync_size;

        if (sync_pa == 0 && sync_sz == 0)
            continue;

        if ((sync_pa < m_gm_base) || (sync_pa >= m_gm_base + gm_region_sz))
            continue;

        m_gm_sync_buf_cnt[_buf_typ]++;
        if (m_gm_buf_base_pa[_buf_typ] == 0)
        {
            m_gm_buf_base_pa[_buf_typ] = sync_pa;
            if (sync_pa + sync_sz >= m_gm_base + gm_region_sz)
                m_gm_buf_map_size[_buf_typ] = m_gm_base + gm_region_sz - sync_pa;
            else
                m_gm_buf_map_size[_buf_typ] = sync_sz;
        } else if (sync_pa + sync_sz <= m_gm_base + gm_region_sz) {
            m_gm_buf_map_size[_buf_typ] = sync_pa + sync_sz - m_gm_buf_base_pa[_buf_typ];
        } else if ((sync_pa < m_gm_base + gm_region_sz)
            && (sync_pa + sync_sz > m_gm_base + gm_region_sz)) {
            m_gm_buf_map_size[_buf_typ] = m_gm_base + gm_region_sz - m_gm_buf_base_pa[_buf_typ];
        } else {
            LOG(LOG_ERR, "GM buff invalid: pa: %lx, sz: %x, base_pa: %lx\n",
                sync_pa, sync_sz, m_gm_buf_base_pa[_buf_typ]);
            ret = AIPU_STATUS_ERROR_INVALID_GM;
        }
    }

out:
    return ret;
}

bool aipudrv::GM_V3::gm_is_gm_buffer(uint32_t idx, uint32_t buf_type)
{
    bool ret = false;

    if (!m_job.m_mem->is_gm_enable())
        goto out;

    if (m_job.m_qos == AIPU_JOB_QOS_HIGH && !m_job.m_mem->is_both_gm_region_enable())
        goto out;

    if(m_job.m_gm_info[buf_type].count(idx) != 1)
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

bool aipudrv::GM_V3::gm_need_remap()
{
    if ((m_gm_buf_map_size[EM_GM_BUF_INPUT] != 0)
        || (m_gm_buf_map_size[EM_GM_BUF_OUTPUT] != 0))
        return true;

    // is a temp buffer, non input/output
    if (m_gm_base != 0)
        return true;

    return false;
}

bool aipudrv::GM_V3::gm_need_sync_out()
{
    if (m_gm_buf_map_size[EM_GM_BUF_OUTPUT] != 0)
        return true;
    else
        return false;
}

void aipudrv::GM_V3::get_valid_map_base(BufferDesc &buf)
{
    if (m_gm_alloc_buffer.size() == 0)
        m_gm_base = buf.pa;
    m_gm_alloc_buffer.push_back(&buf);
}

void aipudrv::GM_V3::get_valid_sync_region(uint32_t sg_id, uint32_t idx, uint32_t buf_type,
    BufferDesc &buf, ValidSyncBuffer &region)
{
    if (buf_type == GM_BUF_TYPE_REUSE)
    {
        for (auto desc : m_graph.get_subgraph(sg_id).io.inputs)
        {
            /**
             * it exist several input buffers exist in one large buffer
             */
            if (desc.ref_section_iter == idx)
            {
                if (region.valid_sync_buf[EM_GM_BUF_INPUT].sync_size == 0)
                {
                    region.valid_sync_buf[EM_GM_BUF_INPUT].sync_pa = buf.pa + desc.offset_in_section;
                    region.valid_sync_buf[EM_GM_BUF_INPUT].sync_size =
                        (desc.size + AIPU_PAGE_SIZE) & (~(AIPU_PAGE_SIZE - 1));
                } else {
                    if (region.valid_sync_buf[EM_GM_BUF_INPUT].sync_pa < buf.pa + desc.offset_in_section)
                    {
                        region.valid_sync_buf[EM_GM_BUF_INPUT].sync_size = (buf.pa + desc.offset_in_section + desc.size -
                            region.valid_sync_buf[EM_GM_BUF_INPUT].sync_pa + AIPU_PAGE_SIZE) & (~(AIPU_PAGE_SIZE - 1));
                    } else {
                        DEV_PA_64 tmp_pa = region.valid_sync_buf[EM_GM_BUF_INPUT].sync_pa;
                        uint32_t tmp_sz = region.valid_sync_buf[EM_GM_BUF_INPUT].sync_size;
                        region.valid_sync_buf[EM_GM_BUF_INPUT].sync_pa = buf.pa + desc.offset_in_section;
                        region.valid_sync_buf[EM_GM_BUF_INPUT].sync_size = (tmp_pa + tmp_sz -
                            region.valid_sync_buf[EM_GM_BUF_INPUT].sync_pa + AIPU_PAGE_SIZE) & (~(AIPU_PAGE_SIZE - 1));
                    }
                }
            }
        }

        for (auto desc : m_graph.get_subgraph(sg_id).io.outputs)
        {
            /**
             * it exist several output buffers exist in one large buffer
             */
            if (desc.ref_section_iter == idx)
            {
                if (region.valid_sync_buf[EM_GM_BUF_OUTPUT].sync_size == 0)
                {
                    region.valid_sync_buf[EM_GM_BUF_OUTPUT].sync_pa = buf.pa + desc.offset_in_section;
                    region.valid_sync_buf[EM_GM_BUF_OUTPUT].sync_size =
                        (desc.size + AIPU_PAGE_SIZE) & (~(AIPU_PAGE_SIZE - 1));
                } else {
                    if (region.valid_sync_buf[EM_GM_BUF_OUTPUT].sync_pa < buf.pa + desc.offset_in_section)
                    {
                        region.valid_sync_buf[EM_GM_BUF_OUTPUT].sync_size = (buf.pa + desc.offset_in_section + desc.size -
                            region.valid_sync_buf[EM_GM_BUF_OUTPUT].sync_pa + AIPU_PAGE_SIZE) & (~(AIPU_PAGE_SIZE - 1));
                    } else {
                        DEV_PA_64 tmp_pa = region.valid_sync_buf[EM_GM_BUF_OUTPUT].sync_pa;
                        uint32_t tmp_sz = region.valid_sync_buf[EM_GM_BUF_OUTPUT].sync_size;
                        region.valid_sync_buf[EM_GM_BUF_OUTPUT].sync_pa = buf.pa + desc.offset_in_section;
                        region.valid_sync_buf[EM_GM_BUF_OUTPUT].sync_size = (tmp_pa + tmp_sz -
                            region.valid_sync_buf[EM_GM_BUF_OUTPUT].sync_pa + AIPU_PAGE_SIZE) & (~(AIPU_PAGE_SIZE - 1));
                    }
                }
            }
        }
    } else {
        region.valid_sync_buf[EM_GM_BUF_INPUT].sync_pa = buf.pa;
        region.valid_sync_buf[EM_GM_BUF_INPUT].sync_size = buf.size;
        region.valid_sync_buf[EM_GM_BUF_OUTPUT].sync_pa = 0;
        region.valid_sync_buf[EM_GM_BUF_OUTPUT].sync_size = 0;
    }
}
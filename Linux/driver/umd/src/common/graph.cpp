// Copyright (C) 2022-2023 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  graph.cpp
 * @brief AIPU User Mode Driver (UMD) graph module implementation
 */

#include <cstring>
#include "graph.h"
#include "parser_base.h"
#include "utils/helper.h"
#include "utils/log.h"

aipudrv::Graph::Graph(void* ctx, GRAPH_ID id, DeviceBase* dev): GraphBase(ctx, id, dev)
{
    m_btext.init(nullptr, 0);
    m_bcrodata.init(nullptr, 0);
    m_brodata.init(nullptr, 0);
    m_bdesc.init(nullptr, 0);
    m_bweight.init(nullptr, 0);
    m_bdata.init(nullptr, 0);
    m_text.reset();
    m_crodata.reset();
    m_weight.reset();
}

aipudrv::Graph::~Graph()
{
}

aipu_status_t aipudrv::Graph::load(std::istream& gbin, uint32_t size, bool ver_check)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;

    ret = m_parser->parse_graph(gbin, size, *this);
    if (AIPU_STATUS_SUCCESS != ret)
        goto finish;

    m_do_vcheck = ver_check;
    if (ver_check && !m_dev->has_target(m_arch, m_hw_version, m_hw_config, m_hw_revision))
        return AIPU_STATUS_ERROR_TARGET_NOT_FOUND;

    m_mem->dump_tracking_log_start();

    /* alloc and load text buffer */
    if (m_btext.size != 0)
    {
        ret = m_mem->malloc(m_btext.size, 0, &m_text, "text");
        if (AIPU_STATUS_SUCCESS != ret)
            goto finish;
        m_mem->write(m_text.pa, m_btext.va, m_btext.size);
    }

    if (m_bcrodata.size != 0)
    {
        ret = m_mem->malloc(m_bcrodata.size, 0, &m_crodata, "crodata");
        if (ret != AIPU_STATUS_SUCCESS)
            goto finish;
        m_mem->write(m_crodata.pa, m_bcrodata.va, m_bcrodata.size);
    }

    /* alloc and load weight buffer */
    // if (m_bweight.size != 0)
    // {
    //     ret = m_mem->malloc(m_bweight.size, 0, &m_weight, "weight");
    //     if (AIPU_STATUS_SUCCESS != ret)
    //         goto finish;
    //     m_mem->write(m_weight.pa, m_bweight.va, m_bweight.size);
    // }

finish:
    return ret;
}

/**
 * @brief each graph only needs to allocate one copy of weight
 *        for multiple jobs in order to reduce memory consumption.
 *
 * @note  if weight buffer is in DDR region, the whole weight data
 *        is put in one large buffer locating in ASID1.
 *        if intend to put weight buffer in SRAM or DTCM, the large
 *        weight buffer is split into more small buffers in order to
 *        put them more to specific region. the rest of buffer that
 *        can't be allocated from SRAM/DTCM is from ASID0 default.
 */
aipu_status_t aipudrv::Graph::alloc_weight_buffer(std::vector<struct GraphSectionDesc> &static_sections)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    struct GraphSectionDesc *static_section = nullptr;

    pthread_rwlock_wrlock(&m_alloc_wt_lock);
    if (m_weight.size != 0 || m_weights.size() != 0)
        goto finish;

    if (get_weight_region() == AIPU_MEM_REGION_DEFAULT)
    {
        if (m_weight.size == 0 && m_bweight.size != 0)
        {
            /**
             * allocate weight from ASID1 region defalut.if all ASIDs are configured
             * with the same base addr, it's also equal to allocate from ASID0.
             */
            ret = m_mem->malloc(m_bweight.size, 0, &m_weight, "weight",
                (1 << 8) | AIPU_MEM_REGION_DEFAULT);
            if (AIPU_STATUS_SUCCESS != ret)
                goto finish;
            m_mem->write(m_weight.pa, m_bweight.va, m_bweight.size);
        }

        for (uint32_t i = 0; i < static_sections.size(); i++)
        {
            BufferDesc buf;
            static_section = &static_sections[i];

            buf.init(m_weight.asid_base, m_weight.pa + static_section->offset,
                static_section->size, static_section->size);
            m_weights.push_back(buf);
        }
    } else {
        for (uint32_t i = 0; i < static_sections.size(); i++)
        {
            if (m_weights.size() != static_sections.size())
            {
                BufferDesc buf;
                std::string str = "static_" + std::to_string(i);
                static_section = &static_sections[i];

                buf.reset();
                ret = m_mem->malloc(static_section->size, static_section->align_in_page, &buf, str.c_str(),
                    get_weight_region());
                if (AIPU_STATUS_SUCCESS != ret)
                {
                    LOG(LOG_ERR, "alloc weight buffer %d [fail]", i);
                    goto finish;
                }

                m_mem->write(buf.pa, (char *)static_section->load_src, static_section->size);
                m_weights.push_back(buf);
            }
        }
    }

finish:
    pthread_rwlock_unlock(&m_alloc_wt_lock);
    return ret;
}

aipu_status_t aipudrv::Graph::unload()
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;

    ret = destroy_jobs();
    if (ret != AIPU_STATUS_SUCCESS)
        return ret;

    if (m_text.size != 0)
    {
        m_mem->free(&m_text);
        m_text.reset();
    }

    if (m_crodata.size != 0)
    {
        m_mem->free(&m_crodata);
        m_crodata.reset();
    }

    if (get_weight_region() == AIPU_MEM_REGION_DEFAULT)
    {
        if (m_weight.size != 0)
        {
            m_mem->free(&m_weight);
            m_weight.reset();
        }

        m_weights.clear();
    } else {
        for (uint32_t i = 0; i < m_weights.size(); i++)
            m_mem->free(&m_weights[i]);

        m_weights.clear();
    }

    m_mem->dump_tracking_log_end();
    return ret;
}
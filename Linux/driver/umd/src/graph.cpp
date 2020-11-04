// Copyright (C) 2022 Arm Technology (China) Co. Ltd. All rights reserved.
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
    if (m_bweight.size != 0)
    {
        ret = m_mem->malloc(m_bweight.size, 0, &m_weight, "weight");
        if (AIPU_STATUS_SUCCESS != ret)
            goto finish;
        m_mem->write(m_weight.pa, m_bweight.va, m_bweight.size);
    }

finish:
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

    if (m_weight.size != 0)
    {
        m_mem->free(&m_weight);
        m_weight.reset();
    }

    m_mem->dump_tracking_log_end();
    return ret;
}

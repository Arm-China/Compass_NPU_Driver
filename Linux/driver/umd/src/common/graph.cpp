// Copyright (C) 2023-2024 Arm Technology (China) Co. Ltd.
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
    m_bdata.init(nullptr, 0);
    m_bweight.clear();
}

aipudrv::Graph::~Graph()
{
}

aipu_status_t aipudrv::Graph::load(std::istream& gbin, uint32_t size,bool ver_check,
    aipu_load_graph_cfg_t *config)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;

    /**
    * decide weight allocation strategy
    */
    if (config != nullptr)
    {
        m_wt_mem_region = config->wt_mem_region;
        if (config->wt_idxes)
        {
            for (int i = 0; i < config->wt_idxes_cnt; i++)
                m_wt_idxes.insert(config->wt_idxes[i]);
        }

        if (config->extra_weight_path != nullptr)
        {
            if(access(config->extra_weight_path, F_OK) == 0)
            {
                m_extra_weight_path = config->extra_weight_path;
            } else {
                LOG(LOG_ERR, "Extra weight path: %s [non exist]\n", config->extra_weight_path);
                return AIPU_STATUS_ERROR_OPEN_FILE_FAIL;
            }
        }
    }

    ret = m_parser->parse_graph(gbin, size, *this);
    if (ret != AIPU_STATUS_SUCCESS)
        goto finish;

    m_mem->dump_tracking_log_start();
    m_do_vcheck = ver_check;
    if (ver_check && !m_dev->has_target(m_arch, m_hw_version, m_hw_config, m_hw_revision))
        return AIPU_STATUS_ERROR_TARGET_NOT_FOUND;

    /* alloc and load text buffer */
    if (m_btext.size != 0)
    {
        /**
         * expand 16 bytes more to export RO base for debugger.
         * there is no effect for text self.
         */
        ret = m_mem->malloc(m_btext.size + 16, 0, &m_text, "text");
        if (ret != AIPU_STATUS_SUCCESS)
            goto finish;
        m_mem->write(m_text->pa, m_btext.va, m_btext.size);
    }

    if (m_bcrodata.size != 0)
    {
        ret = m_mem->malloc(m_bcrodata.size, 0, &m_crodata, "crodata");
        if (ret != AIPU_STATUS_SUCCESS)
            goto finish;
        m_mem->write(m_crodata->pa, m_bcrodata.va, m_bcrodata.size);
    }

    if (m_bweight.size() > 0)
    {
        ret = alloc_weight_buffer(get_static_section_ref(0), config);
        if (ret != AIPU_STATUS_SUCCESS)
            goto finish;
    }

    // if (m_bweight.size != 0)
    // {
    //     ret = m_mem->malloc(m_bweight.size, 0, &m_weight, "weight");
    //     if (ret != AIPU_STATUS_SUCCESS)
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
#if 0
aipu_status_t aipudrv::Graph::alloc_weight_buffer(std::vector<struct GraphSectionDesc> &static_sections,
    aipu_load_graph_cfg_t *config)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    struct GraphSectionDesc *static_section = nullptr;
    int pad_sz = 0;

    /**
    * decide weight allocation strategy
    */
    if (config != nullptr)
    {
        m_wt_mem_region = config->wt_mem_region;
        if (config->wt_idxes)
        {
            for (int i = 0; i < config->wt_idxes_cnt; i++)
                m_wt_idxes.insert(config->wt_idxes[i]);
        }
    }

#ifndef SIMULATION
    if (m_hw_version == AIPU_ISA_VERSION_ZHOUYI_V3)
        pad_sz = 0x800;
#endif

    if (m_wt_mem_region == AIPU_MEM_REGION_DEFAULT)
    {
        uint32_t asid = 0;

        if (m_bweight.size != 0)
        {
            if (m_hw_version == AIPU_ISA_VERSION_ZHOUYI_V3
                || m_hw_version == AIPU_ISA_VERSION_ZHOUYI_V3_1)
                asid = 1;

            /**
             * allocate weight from ASID1 region defalut.if all ASIDs are configured
             * with the same base addr, it's also equal to allocate from ASID0.
             */
            ret = m_mem->malloc(get_const_size() + pad_sz, 0, &m_weight, "weight",
                (asid << 8) | AIPU_MEM_REGION_DEFAULT);
            if (ret != AIPU_STATUS_SUCCESS)
            {
                LOG(LOG_ERR, "alloc weight buffer [fail]");
                goto finish;
            }

            if (get_zerocpy_const_size() > 0)
            {
                ret = m_mem->malloc(get_zerocpy_const_size() + pad_sz, 0, &wb_zerocpy_const, "zerocpy_const",
                    AIPU_MEM_REGION_DEFAULT);
                if (ret != AIPU_STATUS_SUCCESS)
                {
                    LOG(LOG_ERR, "alloc zerocpy_const buffer [fail]");
                    goto finish;
                }
            }
        }

        for (uint32_t i = 0; i < static_sections.size(); i++)
        {
            BufferDesc *buf = new BufferDesc;
            static_section = &static_sections[i];
            buf->reset();

            if (static_section->type == SECTION_TYPE_ZEROCPY_CONSTANT)
            {
                m_mem->write(wb_zerocpy_const->pa + static_section->relative_addr,
                    m_bweight.va + static_section->offset_in_file, static_section->size);
                buf->init(wb_zerocpy_const->asid_base, wb_zerocpy_const->pa + static_section->relative_addr,
                    static_section->size, static_section->size);
                LOG(LOG_INFO, "zerocpy %d, pa=%lx, a_b=%lx, asid_pa=%lx, relative_addr=%x\n", i,
                    buf->pa, buf->asid_base, buf->align_asid_pa, static_section->relative_addr);
            } else {
                m_mem->write(m_weight->pa + static_section->relative_addr,
                    m_bweight.va + static_section->offset_in_file, static_section->size);
                buf->init(m_weight->asid_base, m_weight->pa + static_section->relative_addr,
                    static_section->size, static_section->size, 0, asid << 8);
            }

            m_weights.push_back(buf);
        }
    } else {
        for (uint32_t i = 0; i < static_sections.size(); i++)
        {
            if (m_weights.size() != static_sections.size())
            {
                BufferDesc *buf = nullptr;
                std::string str = "static_" + std::to_string(i);
                static_section = &static_sections[i];

                if (m_wt_idxes.count(i) == 1)
                    ret = m_mem->malloc(static_section->size + pad_sz, static_section->align_in_page, &buf, str.c_str(),
                        m_wt_mem_region);
                else
                    ret = m_mem->malloc(static_section->size + pad_sz, static_section->align_in_page, &buf, str.c_str(),
                        AIPU_MEM_REGION_DEFAULT);

                if (ret != AIPU_STATUS_SUCCESS)
                {
                    LOG(LOG_ERR, "alloc weight buffer %d [fail]", i);
                    goto finish;
                }

                m_mem->write(buf->pa, (char *)static_section->load_src, static_section->size);
                m_weights.push_back(buf);
            }
        }
    }

finish:
    return ret;
}
#else
aipu_status_t aipudrv::Graph::alloc_weight_buffer(std::vector<struct GraphSectionDesc> &static_sections,
    aipu_load_graph_cfg_t *config)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    struct GraphSectionDesc *static_section = nullptr;
    uint32_t asid = 0;
    int pad_sz = 0;

#ifndef SIMULATION
    if (m_hw_version == AIPU_ISA_VERSION_ZHOUYI_V3)
        pad_sz = 0x800;
#endif

    if (m_wt_mem_region == AIPU_MEM_REGION_DEFAULT)
    {
        for (uint32_t bss_id = 0; bss_id < get_bss_cnt(); bss_id++)
        {
            std::vector<struct GraphSectionDesc> &static_sections = get_static_section_ref(bss_id);
            struct WeightBufferInfo weightBufferInfo = {0};

            if (m_bweight.size() > 0 && m_bweight[bss_id].size != 0)
            {
                if (m_hw_version == AIPU_ISA_VERSION_ZHOUYI_V3
                    || m_hw_version == AIPU_ISA_VERSION_ZHOUYI_V3_1)
                    asid = 1;

                /**
                * allocate weight from ASID1 region defalut.if all ASIDs are configured
                * with the same base addr, it's also equal to allocate from ASID0.
                */
                ret = m_mem->malloc(get_const_size(bss_id) + pad_sz, 0,
                    &weightBufferInfo.wb_weight, "weight", (asid << 8) | AIPU_MEM_REGION_DEFAULT);
                if (ret != AIPU_STATUS_SUCCESS)
                {
                    LOG(LOG_ERR, "alloc weight buffer [fail]");
                    goto finish;
                }

                if (get_zerocpy_const_size(bss_id) > 0)
                {
                    ret = m_mem->malloc(get_zerocpy_const_size(bss_id) + pad_sz, 0,
                        &weightBufferInfo.wb_zerocpy_const, "zerocpy_const", AIPU_MEM_REGION_DEFAULT);
                    if (ret != AIPU_STATUS_SUCCESS)
                    {
                        LOG(LOG_ERR, "alloc zerocpy_const buffer [fail]");
                        goto finish;
                    }
                }
            }

            for (uint32_t i = 0; i < static_sections.size(); i++)
            {
                BufferDesc *buf = new BufferDesc;
                static_section = &static_sections[i];
                buf->reset();

                if (static_section->type == SECTION_TYPE_ZEROCPY_CONSTANT)
                {
                    m_mem->write(weightBufferInfo.wb_zerocpy_const->pa + static_section->relative_addr,
                        m_bweight[bss_id].va + static_section->offset_in_file, static_section->size);
                    buf->init(weightBufferInfo.wb_zerocpy_const->asid_base,
                        weightBufferInfo.wb_zerocpy_const->pa + static_section->relative_addr,
                        static_section->size, static_section->size);
                    LOG(LOG_INFO, "zerocpy %d, pa=%lx, a_b=%lx, asid_pa=%lx, relative_addr=%x\n", i,
                        buf->pa, buf->asid_base, buf->align_asid_pa, static_section->relative_addr);
                } else {
                    m_mem->write(weightBufferInfo.wb_weight->pa + static_section->relative_addr,
                        m_bweight[bss_id].va + static_section->offset_in_file, static_section->size);
                    buf->init(weightBufferInfo.wb_weight->asid_base,
                        weightBufferInfo.wb_weight->pa + static_section->relative_addr,
                        static_section->size, static_section->size, 0, asid << 8);
                }

                weightBufferInfo.wb_weights.push_back(buf);
                if (bss_id != 0)
                    m_weight_buffers_vec[0].wb_weights.push_back(buf);
            }

            weightBufferInfo.wb_asid_base = weightBufferInfo.wb_weight->asid_base;
            m_weight_buffers_vec.push_back(weightBufferInfo);
        }
    } else {
        if (get_bss_cnt() == 1)
        {
            for (uint32_t bss_id = 0; bss_id < get_bss_cnt(); bss_id++)
            {
                std::vector<struct GraphSectionDesc> &static_sections = get_static_section_ref(bss_id);
                struct WeightBufferInfo weightBufferInfo = {0};

                for (uint32_t i = 0; i < static_sections.size(); i++)
                {
                    if (weightBufferInfo.wb_weights.size() != static_sections.size())
                    {
                        BufferDesc *buf = nullptr;
                        std::string str = "static_" + std::to_string(i);
                        static_section = &static_sections[i];

                        if ((m_wt_idxes.count(i) == 1) && (static_section->type != SECTION_TYPE_ZEROCPY_CONSTANT)) {
                            ret = m_mem->malloc(static_section->size + pad_sz, static_section->align_in_page,
                                &buf, str.c_str(), m_wt_mem_region);
                        } else {
                            if (static_section->type == SECTION_TYPE_ZEROCPY_CONSTANT)
                                ret = m_mem->malloc(static_section->size + pad_sz, static_section->align_in_page,
                                    &buf, str.c_str(), AIPU_MEM_REGION_DEFAULT);
                            else {
                                ret = m_mem->malloc(static_section->size + pad_sz, static_section->align_in_page,
                                    &buf, str.c_str(), (asid << 8) | AIPU_MEM_REGION_DEFAULT);
                            }
                        }

                        if (ret != AIPU_STATUS_SUCCESS)
                        {
                            LOG(LOG_ERR, "alloc weight buffer %d [fail]", i);
                            goto finish;
                        }

                        m_mem->write(buf->pa, (char *)static_section->load_src, static_section->size);
                        weightBufferInfo.wb_weights.push_back(buf);
                        if (bss_id != 0)
                            m_weight_buffers_vec[0].wb_weights.push_back(buf);
                    }
                }

                m_weight_buffers_vec.push_back(weightBufferInfo);
            }
        } else {
            LOG(LOG_ALERT, "Not support: specify weight region on BSS coutnt > 0\n");
        }
    }

finish:
    return ret;
}
#endif

aipu_status_t aipudrv::Graph::unload()
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;

    ret = destroy_jobs();
    if (ret != AIPU_STATUS_SUCCESS)
        return ret;

    if (m_text && m_text->size != 0)
        m_mem->free(&m_text);

    if (m_crodata && m_crodata->size != 0)
        m_mem->free(&m_crodata);

    if (m_bweight.size() > 0)
    {
        for (uint32_t bss_id = 0; bss_id < get_bss_cnt(); bss_id++)
        {
            struct WeightBufferInfo &weightBufferInfo = m_weight_buffers_vec[bss_id];

            if (weightBufferInfo.wb_zerocpy_const != nullptr && weightBufferInfo.wb_zerocpy_const->size != 0)
                m_mem->free(&weightBufferInfo.wb_zerocpy_const);

            if (weightBufferInfo.wb_weight != nullptr)
            {
                if (weightBufferInfo.wb_weight->size != 0)
                    m_mem->free(&weightBufferInfo.wb_weight);

                if (bss_id == 0)
                {
                    for (uint32_t i = 0; i < weightBufferInfo.wb_weights.size(); i++)
                    {
                        weightBufferInfo.wb_weights[i]->reset();
                        delete weightBufferInfo.wb_weights[i];
                        weightBufferInfo.wb_weights[i] = nullptr;
                    }
                }
            } else {
                if (bss_id == 0)
                {
                    for (uint32_t i = 0; i < weightBufferInfo.wb_weights.size(); i++)
                        m_mem->free(&weightBufferInfo.wb_weights[i]);
                }
            }
            weightBufferInfo.wb_weights.clear();
        }
    }

    m_mem->dump_tracking_log_end();
    return ret;
}

int32_t aipudrv::Graph::get_dynamic_shape_dim_num(uint32_t idx, bool max_shape_dim)
{
    if (!is_dynamic_shape())
        return 0;

    if (idx >= 0 && idx < m_input_shape_constraint.size())
    {
        if (!max_shape_dim)
            return m_input_shape_constraint[idx][0].size();
        else
            return m_input_shape_constraint[idx][1].size();
    }

    return 0;
}

bool aipudrv::Graph::get_dynamic_shape_data(uint32_t idx, bool max_shape_dim, uint32_t *data)
{
    if (!is_dynamic_shape())
        return false;

    if (data == nullptr)
    {
        LOG(LOG_ERR, "data ptr is NULL\n");
        return false;
    }

    if (idx >= 0 && idx < m_input_shape_constraint.size())
    {
        if (!max_shape_dim)
        {
            for (uint32_t i = 0; i < m_input_shape_constraint[idx][0].size(); i++)
                *(data + i) = m_input_shape_constraint[idx][0][i];
        } else {
            for (uint32_t i = 0; i < m_input_shape_constraint[idx][1].size(); i++)
                *(data + i) = m_input_shape_constraint[idx][1][i];
        }
    }

    return true;
}

aipu_status_t aipudrv::Graph::set_graph_extra_weight(BinSection extra_weight)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    char *start = (char *)extra_weight.va;
    size_t extra_weight_bin_cnt = *(size_t *)start;

    start += sizeof(size_t);
    for (size_t i = 0; i < extra_weight_bin_cnt; i++)
    {
        struct ExtraWeightInfo extraWeightInfo;
        struct BinSection ew_binsection = {0};
        size_t extra_weight_bin_name_len = *(size_t *)start;
        std::string path;

        start += sizeof(size_t);
        char *plus_mark = std::strstr(start, "+");
        extraWeightInfo.extraWeight_name = std::string(start, plus_mark - start);
        extraWeightInfo.extraWeight_hash = std::string(plus_mark + 1,
            extra_weight_bin_name_len - (plus_mark + 1 - start));
        start += extra_weight_bin_name_len;

        path = m_extra_weight_path + "/" + extraWeightInfo.extraWeight_name;
        ret = umd_mmap_file_helper(path.c_str(), (void **)&ew_binsection.va, &ew_binsection.size);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            LOG(LOG_ERR, "Mmap extra weight: %s [fail]\n", path.c_str());
            m_extra_weight_info_vec.clear();
            return ret;
        }

        set_graph_weight(ew_binsection);
        m_extra_weight_info_vec.push_back(extraWeightInfo);
    }

    return ret;
}
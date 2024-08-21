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
    m_bweight.init(nullptr, 0);
    m_bdata.init(nullptr, 0);
}

aipudrv::Graph::~Graph()
{
}

aipu_status_t aipudrv::Graph::load(std::istream& gbin, uint32_t size,bool ver_check,
    aipu_load_graph_cfg_t *config)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;

    ret = m_parser->parse_graph(gbin, size, *this);
    if (AIPU_STATUS_SUCCESS != ret)
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
        if (AIPU_STATUS_SUCCESS != ret)
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

    ret = alloc_weight_buffer(get_static_section_ref(), config);
    if (ret != AIPU_STATUS_SUCCESS)
        goto finish;

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
                || m_hw_version == AIPU_ISA_VERSION_ZHOUYI_V4)
                asid = 1;

            /**
             * allocate weight from ASID1 region defalut.if all ASIDs are configured
             * with the same base addr, it's also equal to allocate from ASID0.
             */
            ret = m_mem->malloc(get_const_size() + pad_sz, 0, &m_weight, "weight",
                (asid << 8) | AIPU_MEM_REGION_DEFAULT);
            if (AIPU_STATUS_SUCCESS != ret)
            {
                LOG(LOG_ERR, "alloc weight buffer [fail]");
                goto finish;
            }

            if (get_zerocpy_const_size() > 0)
            {
                ret = m_mem->malloc(get_zerocpy_const_size() + pad_sz, 0, &m_zerocpy_const, "zerocpy_const",
                    AIPU_MEM_REGION_DEFAULT);
                if (AIPU_STATUS_SUCCESS != ret)
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
                m_mem->write(m_zerocpy_const->pa + static_section->relative_addr,
                    m_bweight.va + static_section->offset_in_file, static_section->size);
                buf->init(m_zerocpy_const->asid_base, m_zerocpy_const->pa + static_section->relative_addr,
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

                if (AIPU_STATUS_SUCCESS != ret)
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

    if (m_zerocpy_const != nullptr && m_zerocpy_const->size != 0)
        m_mem->free(&m_zerocpy_const);

    if (m_weight != nullptr)
    {
        if (m_weight->size != 0)
            m_mem->free(&m_weight);

        for (uint32_t i = 0; i < m_weights.size(); i++)
        {
            m_weights[i]->reset();
            delete m_weights[i];
        }
    } else {
        for (uint32_t i = 0; i < m_weights.size(); i++)
            m_mem->free(&m_weights[i]);
    }
    m_weights.clear();

    m_parsed_shape.clear();

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

bool aipudrv::Graph::set_dynamic_shape_data(aipu_dynshape_param_t *shape_param)
{
    if (!is_dynamic_shape())
        return false;

    if (shape_param->input_shape_cnt != get_dynamic_shape_num())
    {
        LOG(LOG_ERR, "config shape count not match input shape count\n");
        return false;
    }

    if (shape_param->shape_items == nullptr)
    {
        LOG(LOG_ERR, "config shape address is NULL\n");
        return false;
    }

    m_parsed_shape.clear();
    for (uint32_t i = 0; i < shape_param->input_shape_cnt; i++)
    {
        aipu_dynshape_item_t *shape_item = &shape_param->shape_items[i];
        uint32_t idx = shape_item->ds_idx;

        if (idx >= 0 && idx < m_input_shape_constraint.size())
        {
            uint32_t size = 1;

            for (uint32_t dim = 0; dim < m_input_shape_constraint[idx][0].size(); dim++)
            {
                size *= shape_item->ds_data[dim];
                m_parsed_shape[idx].push_back(shape_item->ds_data[dim]);
            }

            if (size < m_input_shape_threshhold[idx][0] &&
                size > m_input_shape_threshhold[idx][1])
            {
                m_parsed_shape.clear();
                LOG(LOG_ERR, "input %d: dynamic shape invalid, valid scope [%lu, %lu]\n",
                    idx, m_input_shape_threshhold[idx][0], m_input_shape_threshhold[idx][1]);
                return false;
            }

            m_config_in_tensor_size[idx] = size;
            if (get_io_tensor_type(idx) == AIPU_DATA_TYPE_U16
                || get_io_tensor_type(idx) == AIPU_DATA_TYPE_S16
                || get_io_tensor_type(idx) == AIPU_DATA_TYPE_F16
                || get_io_tensor_type(idx) == AIPU_DATA_TYPE_BF16)
                m_config_in_tensor_size[idx] <<= 1;
            else if (get_io_tensor_type(idx) == AIPU_DATA_TYPE_U32
                || get_io_tensor_type(idx) == AIPU_DATA_TYPE_S32
                || get_io_tensor_type(idx) == AIPU_DATA_TYPE_F32)
                m_config_in_tensor_size[idx] <<= 2;
        } else {
            m_parsed_shape.clear();
            m_config_in_tensor_size.clear();
            LOG(LOG_ERR, "input %d invalid index\n", idx);
            return false;
        }
    }

    if (m_parsed_shape.size() != get_dynamic_shape_num())
    {
        m_parsed_shape.clear();
        m_config_in_tensor_size.clear();
        LOG(LOG_ERR, "set shape count != input shape count\n");
        return false;
    }

    reset_dynamic_out_shape_updated_flag();
    update_dynamic_io_tensor_size(AIPU_TENSOR_TYPE_INPUT);

    return true;
}
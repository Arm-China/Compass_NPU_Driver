// Copyright (C) 2023-2024 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  dynamic_shape.cpp
 * @brief AIPU User Mode Driver (UMD) dynamic shape implementation
 */

#include <cstring>
#include <numeric>
#include "graph.h"
#include "parser_base.h"
#include "dynamic_shape.h"
#include "utils/helper.h"
#include "utils/log.h"

aipudrv::DynamicShape::DynamicShape(JobBase &_jobbase, GraphV3X &_graph, aipu_dynshape_param_t *dyn_params)
    : m_jobbase(_jobbase), m_graph(_graph)
{
    if (dyn_params != nullptr)
        m_dynamic_shape_set_done = set_dynamic_shape_data(dyn_params);
}

aipudrv::DynamicShape::~DynamicShape()
{
}

aipu_status_t aipudrv::DynamicShape::update_dynamic_io_tensor_size(aipu_tensor_type_t type)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    GraphV3X &graph_v3x = static_cast<GraphV3X &>(m_graph);

    if (type == AIPU_TENSOR_TYPE_INPUT)
    {
        for (uint32_t i = 0; i < graph_v3x.get_bss(0).io.inputs.size(); i++)
        {
            graph_v3x.get_bss(0).io.inputs[i].size = get_config_in_tensor_size(i);
        }
    } else if (type == AIPU_TENSOR_TYPE_OUTPUT) {
        for (uint32_t i = 0; i < graph_v3x.get_bss(0).io.outputs.size(); i++)
        {
            graph_v3x.get_bss(0).io.outputs[i].size = get_config_out_tensor_size(i);
        }
    } else {
        LOG(LOG_ERR, "Invalid io tensor type:%d\n", type);
        return AIPU_STATUS_ERROR_INVALID_OP;
    }

    return ret;
}

bool aipudrv::DynamicShape::set_dynamic_shape_data(aipu_dynshape_param_t *shape_param)
{
    auto clear_dynamic_tensor_info = [this]() {
        m_config_in_tensor_shape.clear();
        m_config_in_tensor_size.clear();
    };

    clear_dynamic_tensor_info();
    const auto& shape_constraint = m_graph.m_input_shape_constraint;
    for (uint32_t idx = 0; idx < shape_constraint.size(); ++idx)
    {
        aipu_dynshape_item_t *shape_item = nullptr;
        if (shape_param != nullptr)
        {
            for (uint32_t item_idx = 0; item_idx < shape_param->input_shape_cnt; ++item_idx)
            {
                if (shape_param->shape_items[item_idx].ds_idx == idx)
                {
                    shape_item = &shape_param->shape_items[item_idx];
                    break;
                }
            }
        }

        if (shape_item == nullptr)
        {
            for (uint32_t dim = 0; dim < shape_constraint.at(idx)[0].size(); ++dim)
                m_config_in_tensor_shape[idx].push_back(shape_constraint.at(idx)[0][dim]);

            uint32_t size = std::accumulate(m_config_in_tensor_shape[idx].begin(),
                                            m_config_in_tensor_shape[idx].end(),
                                            1,
                                            std::multiplies<uint32_t> {});
            m_config_in_tensor_size[idx] = size;
        }
        else
        {
            if (shape_item->ds_data == nullptr)
            {
                clear_dynamic_tensor_info();
                LOG(LOG_ERR, "provided %uth shape is nullptr\n", idx);
                return false;
            }

            uint32_t size = 1;
            uint32_t refer_rank = m_graph.m_input_shape_constraint[idx][0].size();
            for (uint32_t dim = 0; dim < refer_rank; dim++)
            {
                if (shape_item->ds_data[dim] < m_graph.m_input_shape_constraint[idx][0][dim] ||
                    shape_item->ds_data[dim] > m_graph.m_input_shape_constraint[idx][1][dim])
                {
                    clear_dynamic_tensor_info();
                    LOG(LOG_ERR, "input %d, dim %d beyond scope\n", idx, dim);
                    return false;
                }

                size *= shape_item->ds_data[dim];
                m_config_in_tensor_shape[idx].push_back(shape_item->ds_data[dim]);
            }

            if (size < m_graph.m_input_shape_threshhold[idx][0] ||
                size > m_graph.m_input_shape_threshhold[idx][1])
            {
                clear_dynamic_tensor_info();
                LOG(LOG_ERR, "input %d: dynamic shape invalid, valid scope [%lu, %lu]\n",
                    idx, m_graph.m_input_shape_threshhold[idx][0], m_graph.m_input_shape_threshhold[idx][1]);
                return false;
            }
            m_config_in_tensor_size[idx] = size;
        }
        if (m_graph.get_io_tensor_type(idx) == AIPU_DATA_TYPE_U16 ||
            m_graph.get_io_tensor_type(idx) == AIPU_DATA_TYPE_S16 ||
            m_graph.get_io_tensor_type(idx) == AIPU_DATA_TYPE_F16 ||
            m_graph.get_io_tensor_type(idx) == AIPU_DATA_TYPE_BF16)
            m_config_in_tensor_size[idx] <<= 1;
        else if (m_graph.get_io_tensor_type(idx) == AIPU_DATA_TYPE_U32 ||
            m_graph.get_io_tensor_type(idx) == AIPU_DATA_TYPE_S32 ||
            m_graph.get_io_tensor_type(idx) == AIPU_DATA_TYPE_F32)
            m_config_in_tensor_size[idx] <<= 2;
    }

    return update_dynamic_io_tensor_size(AIPU_TENSOR_TYPE_INPUT) == AIPU_STATUS_SUCCESS;
}
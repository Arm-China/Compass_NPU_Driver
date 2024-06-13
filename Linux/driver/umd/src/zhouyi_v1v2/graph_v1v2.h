// Copyright (C) 2023-2024 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  graph_v1v2.h
 * @brief AIPU User Mode Driver (UMD) aipu v1/v2 graph module header
 */

#ifndef _GRAPH_V12_H_
#define _GRAPH_V12_H_

#include <map>
#include <vector>
#include <deque>
#include <pthread.h>
#include "standard_api.h"
#include "graph.h"

namespace aipudrv
{
class GraphV12: public Graph
{
private:
    uint32_t m_entry = 0;
    uint32_t m_stack_size = 0;
    uint32_t m_stack_align_in_page = 0;
    std::vector<struct GraphParamMapLoadDesc> m_param_map;
    std::vector<struct GraphSectionDesc> m_static_sections;
    std::vector<struct GraphSectionDesc> m_reuse_sections;
    struct GraphIOTensors m_io;

public:
    virtual void print_parse_info(){};
    virtual aipu_status_t create_job(JOB_ID* id, const aipu_global_config_simulation_t* cfg,
        aipu_global_config_hw_t* hw_cfg, aipu_create_job_cfg_t *config = nullptr);
    virtual aipu_status_t get_tensor_count(aipu_tensor_type_t type, uint32_t* cnt);
    virtual aipu_status_t get_tensor_descriptor(aipu_tensor_type_t type,
        uint32_t tensor, aipu_tensor_desc_t* desc);

    virtual std::vector<struct GraphSectionDesc> &get_static_section_ref()
    {
        return m_static_sections;
    }

public:
    void set_enrty(uint32_t offset)
    {
        m_entry = offset;
    }
    void set_stack(uint32_t sg_id, uint32_t size, uint32_t align)
    {
        m_stack_size = size;
        m_stack_align_in_page = align;
    }
    void add_param(uint32_t sg_id, struct GraphParamMapLoadDesc param)
    {
        m_param_map.push_back(param);
    }
    void add_static_section(uint32_t sg_id, struct GraphSectionDesc section)
    {
        m_static_sections.push_back(section);
    }
    void add_reuse_section(uint32_t sg_id, struct GraphSectionDesc section)
    {
        m_reuse_sections.push_back(section);
    }
    void set_io_tensors(uint32_t sg_id, struct GraphIOTensors io)
    {
        m_io = io;
    }

public:
    GraphV12(void* ctx, GRAPH_ID id, DeviceBase* dev);
    virtual ~GraphV12();
    GraphV12(const GraphV12& graph) = delete;
    GraphV12& operator=(const GraphV12& graph) = delete;

    friend class JobV12;
};
}

#endif /* _GRAPH_V12_H_ */

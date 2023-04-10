// Copyright (C) 2022-2023 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  graph_v3.h
 * @brief AIPU User Mode Driver (UMD) aipu v3 graph module header
 */

#ifndef _GRAPH_V3_H_
#define _GRAPH_V3_H_

#include <map>
#include <vector>
#include <deque>
#include "graph.h"

namespace aipudrv
{

enum
{
    GM_BUF_TYPE_REUSE = 0,
    GM_BUF_TYPE_WEIGHT,
    GM_BUF_TYPE_MAX
};

/* buffer index desc for GM */
struct BssBufferIndex
{
    uint32_t fm_index;  //the idx of feature map list
    uint32_t buf_type;  //0:reuse buffer, 1: static(constant) buffer
    uint32_t buf_index; //the index of buffer
    uint32_t resver0;
};

enum
{
    GM_SUB_BUF_TYPE_IGNORE = 0,
    GM_SUB_BUF_TYPE_INPUT,
    GM_SUB_BUF_TYPE_OUTPUT,
    GM_SUB_BUF_TYPE_INOUT,
    GM_SUB_BUF_TYPE_TEMP,
    GM_SUB_BUF_TYPE_MAX
};

enum
{
    SUBG_DEPEND_NONE = 0,
    SUBG_DEPEND_IMMEDIATE = 1,
    SUBG_DEPEND_PREALL = -1,
};

struct GM_info_desc {
    uint32_t gm_buf_type; // 0: ignore, 1: input, 2: output
    BssBufferIndex gm_buf_idx;
};

/* section: .note.aipu.gmconfig */
struct GMConfig
{
    uint32_t GM_control = 0;
    uint32_t GM_region_ctrl[2] = {0};
    uint32_t reserve0 = 0;
    uint32_t reserve1 = 0;
    uint32_t reserve2 = 0;
    BssBufferIndex GM_buf_idx[2] = {0};
};

struct SegMMUConfig
{
    struct MMUAddr
    {
        uint32_t control[2];
    };

    MMUAddr seg[4] = {0};
    uint32_t SegMMU_ctl = 0;
    uint32_t SegMMU_remap = 0;
    uint32_t reserve0 = 0;
    uint32_t reserve1 = 0;
    uint32_t reserve2 = 0;
    uint32_t reserve3 = 0;
};

/* section: .note.aipu.segmmu */
struct SegMMUList
{
    uint32_t num_mmu;
    SegMMUConfig *segmmu;
};

struct BinSubGraphSection {
    char* va;
    uint64_t offset;
    uint64_t size;
    void load(char* _va, uint64_t _offset, uint64_t _size)
    {
        va = _va;
        offset = _offset;
        size = _size;
    }
};

struct Subgraph {
    uint32_t id;
    struct BinSubGraphSection text;
    struct BinSubGraphSection rodata;
    struct BinSubGraphSection dcr;
    uint32_t printfifo_size;
    uint32_t profiler_buf_size;
    uint32_t private_data_size;
    std::vector<uint32_t> precursors;
    int32_t precursor_cnt;
    uint32_t stack_size;
    uint32_t stack_align_in_page;
    std::vector<struct GraphParamMapLoadDesc> param_map;
    std::vector<struct GraphSectionDesc> static_sections;
    std::vector<struct GraphSectionDesc> reuse_sections;
    std::vector<struct GraphParamMapLoadDesc> private_buffers_map;
    std::vector<struct GraphSectionDesc> private_buffers;
    struct GraphIOTensors io;
};

class GraphV3: public Graph
{
private:
    std::vector<struct Subgraph> m_subgraphs;
    std::vector<struct GMConfig> m_gmconfig;
    BinSection m_bsegmmu;
    bool m_fake_subgraph = false;

public:
    std::map<uint32_t, GM_info_desc> m_gm_info[2];
    uint32_t m_segmmu_num = 0;

public:
    void print_parse_info();
    aipu_status_t extract_gm_info(int sg_id);
    aipu_status_t create_job(JOB_ID* id, const aipu_global_config_simulation_t* cfg,
        aipu_global_config_hw_t* hw_cfg, aipu_create_job_cfg_t *config  = nullptr);
    aipu_status_t get_tensor_count(aipu_tensor_type_t type, uint32_t* cnt);
    aipu_status_t get_tensor_descriptor(aipu_tensor_type_t type, uint32_t tensor, aipu_tensor_desc_t* desc);
    aipu_status_t assign_shared_tensor(aipu_tensor_type_t type,
    uint32_t tensor_idx, uint64_t shared_pa_addr);

public:
    void set_subgraph(struct Subgraph sg)
    {
        m_subgraphs.push_back(sg);
    }

    void set_fake_subgraph()
    {
        m_fake_subgraph = true;
    }

    uint32_t get_subgraph_cnt()
    {
        if (m_fake_subgraph)
            return 0;
        else
            return m_subgraphs.size();
    }

    const Subgraph &get_subgraph(uint32_t sg_id) const
    {
        return m_subgraphs[sg_id];
    }

    void set_gmconfig(BinSection &gm_section)
    {
        GMConfig gmconfig = {0};

        memcpy((void *)&gmconfig, gm_section.va, gm_section.size);
        m_gmconfig.push_back(gmconfig);
    }

    void set_segmmu(BinSection &segmmu_section)
    {
        m_segmmu_num = *(uint32_t *)segmmu_section.va;

        /* extract the head 4 bytes segmmu num information */
        m_bsegmmu.init(segmmu_section.va + 4, segmmu_section.size - 4);
    }

    void set_stack(uint32_t sg_id, uint32_t size, uint32_t align)
    {
        if (sg_id < (uint32_t)m_subgraphs.size())
        {
            m_subgraphs[sg_id].stack_size = size;
            m_subgraphs[sg_id].stack_align_in_page = align;
        }
    }
    void add_param(uint32_t sg_id, struct GraphParamMapLoadDesc param)
    {
        if (sg_id < (uint32_t)m_subgraphs.size())
        {
            m_subgraphs[sg_id].param_map.push_back(param);
        }
    }
    void add_static_section(uint32_t sg_id, struct GraphSectionDesc section)
    {
        if (sg_id < (uint32_t)m_subgraphs.size())
            m_subgraphs[sg_id].static_sections.push_back(section);
    }
    void add_reuse_section(uint32_t sg_id, struct GraphSectionDesc section)
    {
        if (sg_id < (uint32_t)m_subgraphs.size())
            m_subgraphs[sg_id].reuse_sections.push_back(section);
    }
    void set_io_tensors(uint32_t sg_id, struct GraphIOTensors io)
    {
        if (sg_id < (uint32_t)m_subgraphs.size())
            m_subgraphs[sg_id].io = io;
    }

public:
    GraphV3(void* ctx, GRAPH_ID id, DeviceBase* dev);
    ~GraphV3();
    GraphV3(const GraphV3& graph) = delete;
    GraphV3& operator=(const GraphV3& graph) = delete;

    friend class JobV3;
};
}

#endif /* _GRAPH_V3_H_ */
// Copyright (C) 2022 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  graph.h
 * @brief AIPU User Mode Driver (UMD) graph module header
 */

#ifndef _GRAPH_H_
#define _GRAPH_H_

#include <map>
#include <vector>
#include <deque>
#include <pthread.h>
#include "standard_api.h"
#include "graph_base.h"
#include "parser_base.h"

namespace aipudrv
{
enum GraphRemapLoadType
{
    PARAM_MAP_LOAD_TYPE_REUSE,
    PARAM_MAP_LOAD_TYPE_STATIC,
};

struct GraphSubSectionDesc {
    uint32_t offset_in_section;   /**< offset in a section where this subsection based in */
};

struct GraphSectionDesc {
    void* load_src;               /**< section data load source (if applicable) */
    uint32_t size;                /**< section data size */
    uint32_t align_in_page;       /**< section assress alignment requirement (in page) */
    uint32_t offset;
    bool support_dma_buf;
    std::vector<GraphSubSectionDesc> sub_sections; /**< sub-section(s) in this section */
    void init()                   /**< section initializer */
    {
        load_src = nullptr;
        size = 0;
        align_in_page = 1;
        offset = 0;
        support_dma_buf = false;
        sub_sections.clear();
    }
};

struct GraphIOTensors {
    std::vector<struct GraphIOTensorDesc> inputs;
    std::vector<struct GraphIOTensorDesc> outputs;
    std::vector<struct GraphIOTensorDesc> inter_dumps;
    std::vector<struct GraphIOTensorDesc> profiler;
    std::vector<struct GraphIOTensorDesc> printf;
    std::vector<struct GraphIOTensorDesc> layer_counter;
    std::vector<struct GraphIOTensorDesc> err_code;
};

struct GraphParamMapLoadDesc {
    uint32_t offset_in_map;          /**< parameter load offset in rodata parameter map */
    uint32_t load_type;              /**< data type: reuse/static */
    uint32_t buf_type;               /**< buffer type: input/output/segmmu */
    uint32_t ref_section_iter;       /**< referenced section iterator */
    uint32_t ref_sub_section_iter;
    uint32_t sub_section_offset;     /**< subsection offset in its section */
    uint32_t addr_mask;
    void init(uint32_t offset, uint32_t sec_type, uint32_t _buf_type, uint32_t sec_iter,
        uint32_t sub_sec_iter, uint32_t sub_sec_offset, uint32_t mask)
    {
        offset_in_map = offset;
        load_type = sec_type;
        buf_type = _buf_type;
        ref_section_iter = sec_iter;
        ref_sub_section_iter = sub_sec_iter;
        sub_section_offset = sub_sec_offset;
        addr_mask = mask;
    }
};

class ParserBase;

class Graph: public GraphBase
{
protected:
    ParserBase* m_parser = nullptr;
    /* section descriptions in the graph binary */
    struct BinSection m_btext;
    struct BinSection m_bcrodata;
    struct BinSection m_brodata;
    struct BinSection m_bdesc;
    struct BinSection m_bweight;
    struct BinSection m_bdata;
    std::vector<RemapEntry> m_remap;

protected:
    /* Buffers in memory for AIPU's access */
    BufferDesc m_text;
    BufferDesc m_crodata;
    BufferDesc m_weight;
    bool m_do_vcheck = true;

    /* DTCM size, KB unit */
    int m_dtcm_size = 0;

public:
    virtual void set_stack(uint32_t sg_id, uint32_t size, uint32_t align) = 0;
    virtual void add_param(uint32_t sg_id, struct GraphParamMapLoadDesc param) = 0;
    virtual void add_static_section(uint32_t sg_id, struct GraphSectionDesc section) = 0;
    virtual void add_reuse_section(uint32_t sg_id, struct GraphSectionDesc section) = 0;
    virtual void set_io_tensors(uint32_t sg_id, struct GraphIOTensors io) = 0;
    virtual void set_gmconfig(BinSection &gm_section) {}
    virtual void set_segmmu(BinSection &segmmu_section) {}
    virtual aipu_status_t extract_gm_info(int sg_id) { return AIPU_STATUS_SUCCESS; }

public:
    virtual void print_parse_info() = 0;
    virtual aipu_status_t load(std::istream& gbin, uint32_t size, bool ver_check = true);
    virtual aipu_status_t unload();
    virtual aipu_status_t create_job(JOB_ID* id, const aipu_global_config_simulation_t* cfg,
        aipu_create_job_cfg_t *config = nullptr) = 0;
    virtual aipu_status_t get_tensor_count(aipu_tensor_type_t type, uint32_t* cnt) = 0;
    virtual aipu_status_t get_tensor_descriptor(aipu_tensor_type_t type,
        uint32_t tensor, aipu_tensor_desc_t* desc) = 0;

public:
    /* Set functions */
    void set_parser(ParserBase* parser)
    {
        m_parser = parser;
    }
    void set_graph_text(const char* data, uint64_t size)
    {
        m_btext.va = data;
        m_btext.size = size;
    }
    void set_graph_crodata(const char* data, uint64_t size)
    {
        m_bcrodata.va = data;
        m_bcrodata.size = size;
    }
    void set_graph_dp(const char* data, uint64_t size)
    {
        m_bdata.va = data;
        m_bdata.size = size;
    }
    void set_graph_rodata(BinSection rodata)
    {
        m_brodata = rodata;
    }
    void set_graph_desc(BinSection desc)
    {
        m_bdesc = desc;
    }
    void set_graph_weight(BinSection weight)
    {
        m_bweight = weight;
    }
    void add_remap(RemapEntry remap)
    {
        m_remap.push_back(remap);
    }
    void set_dtcm_size(int dtcm_sz)
    {
        m_dtcm_size = dtcm_sz;
    }

    const BufferDesc &get_weight() const
    {
        return m_weight;
    }

    virtual void set_enrty(uint32_t offset){};

    /* Get functions */
    const char* get_bweight_base()
    {
        return m_bweight.va;
    }
    virtual DEV_PA_64 debugger_get_instr_base()
    {
        return m_text.pa;
    }

public:
    Graph(void* ctx, GRAPH_ID id, DeviceBase* dev);
    virtual ~Graph();
    Graph(const Graph& graph) = delete;
    Graph& operator=(const Graph& graph) = delete;

    friend class JobBase;
};
}

#endif /* _GRAPH_H_ */

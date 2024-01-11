// Copyright (C) 2023-2024 Arm Technology (China) Co. Ltd.
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
#include <mutex>
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
    uint32_t offset_in_file;
    uint32_t relative_addr;
    uint32_t type;                /**< weight const or zerocpy_const(15) */
    uint32_t slot_index;
    std::vector<GraphSubSectionDesc> sub_sections; /**< sub-section(s) in this section */
    void init()                   /**< section initializer */
    {
        load_src = nullptr;
        size = 0;
        align_in_page = 1;
        offset_in_file = 0;
        relative_addr = 0;
        type = 0;
        slot_index = 0;
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
    std::vector<struct GraphIOTensorDesc> segmmus;
    std::vector<struct GraphIOTensorDesc> outputs_shape;
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
private:
    uint32_t zerocpy_const_size = 0;
    uint32_t const_size = 0;

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

    /* dynamic shape */
    struct BinSection m_bglobalparam;

    /* entry: <min shape (N, H, W, C), max shape (N, H, W, C)> etc */
    std::map<int, std::vector<std::vector<uint32_t>>> m_input_shape_constraint;

    /* entry: <min size, max size>, size = N*H*W*C */
    std::map<int, std::vector<uint64_t>> m_input_shape_threshhold;

    /* entry: idx: <N, H, W, C> */
    std::map<int, std::vector<uint32_t>> m_parsed_shape;

    /* new set input tensor size for dynamic shape */
    std::map<int, uint32_t> m_config_in_tensor_size;

    /* new output tensor size for dynamic shape */
    std::map<int, uint32_t> m_config_out_tensor_size;
    bool m_dynamic_shape = false;
    bool m_dynamic_out_shape_updated = false;
    std::mutex m_dynamic_out_shape_updated_mtx;

protected:
    /* Buffers in memory for AIPU's access */
    BufferDesc *m_text = nullptr;
    BufferDesc *m_crodata = nullptr;

    /* weight in a whole buffer case */
    BufferDesc *m_weight = nullptr;
    BufferDesc *m_zerocpy_const = nullptr;

    /* weight in split buffer case */
    std::vector<BufferDesc*> m_weights;

    bool m_do_vcheck = true;

    /* DTCM size, KB unit */
    int m_dtcm_size = 0;

public:
    virtual int32_t get_dynamic_shape_dim_num(uint32_t idx, bool max_shape_dim);
    virtual bool get_dynamic_shape_data(uint32_t idx, bool max_shape_dim, uint32_t *data);
    virtual bool set_dynamic_shape_data(aupu_dynshape_param_t *shape_param);
    virtual aipu_status_t update_dynamic_io_tensor_size(aipu_tensor_type_t type)
    {
        return AIPU_STATUS_SUCCESS;
    }

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
        aipu_global_config_hw_t* hw_cfg, aipu_create_job_cfg_t *config = nullptr) = 0;
    virtual aipu_status_t get_tensor_count(aipu_tensor_type_t type, uint32_t* cnt) = 0;
    virtual aipu_status_t get_tensor_descriptor(aipu_tensor_type_t type,
        uint32_t tensor, aipu_tensor_desc_t* desc) = 0;
    virtual void add_const_section(uint32_t sg_id, struct GraphSectionDesc section) {};
    virtual void add_zerocpy_const_section(uint32_t sg_id, struct GraphSectionDesc section) {};
    aipu_status_t alloc_weight_buffer(std::vector<struct GraphSectionDesc> &static_sections);

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

    void set_modle_global_param(BinSection mgp_section)
    {
        m_bglobalparam = mgp_section;
        m_dynamic_shape = true;
    }

    #define GET_U32_FROM_PTR_ADV(ptr) (*(uint32_t *)ptr++)
    bool set_input_shape_constrait(BinSection &isc_section)
    {
        uint32_t *start = (uint32_t *)isc_section.va;
        uint32_t num_inputs = GET_U32_FROM_PTR_ADV(start);

        for (uint32_t i = 0; i < num_inputs; i++)
        {
            uint32_t dim = GET_U32_FROM_PTR_ADV(start);
            std::vector<uint32_t> shape_vec;

            for (uint32_t j = 0; j < dim; j++)
            {
                shape_vec.push_back(GET_U32_FROM_PTR_ADV(start));
            }

            if (shape_vec.size() > 0)
            {
                uint64_t size = 1;

                m_input_shape_constraint[i/2].push_back(shape_vec);
                for (uint32_t k = 0; k < shape_vec.size(); k++)
                {
                     size *= shape_vec[k];

                     if (size == 0)
                     {
                        LOG(LOG_ALERT, "input shape %d, dim %d is 0\n", i/2, k);
                        return false;
                     }
                }

                m_input_shape_threshhold[i/2].push_back(size);
            }
        }

        return true;
    }

    bool is_dynamic_shape()
    {
        return m_dynamic_shape;
    }

    uint32_t get_dynamic_shape_num()
    {
        if (!is_dynamic_shape())
            return 0;
        else
            return m_input_shape_constraint.size();
    }

    bool in_config_shape(uint32_t idx)
    {
        return m_parsed_shape.count(idx) == 1;
    }

    uint32_t get_config_shape_sz()
    {
        return m_parsed_shape.size();
    }

    uint32_t get_config_shape_dim_sz(uint32_t input_idx)
    {
        if (m_parsed_shape.count(input_idx))
            return m_parsed_shape[input_idx].size();
        else
            return 0;
    }

    uint32_t get_config_shape_item(uint32_t input_idx, uint32_t dim_idx)
    {
        return m_parsed_shape[input_idx][dim_idx];
    }

    uint32_t get_config_in_tensor_size(uint32_t input_idx)
    {
        return m_config_in_tensor_size[input_idx];
    }

    void set_config_out_tensor_size(uint32_t output_idx, uint32_t size)
    {
        m_config_out_tensor_size[output_idx] = size;
    }

    uint32_t get_config_out_tensor_size(uint32_t output_idx)
    {
        return m_config_out_tensor_size[output_idx];
    }

    void clear_config_out_tensor_size()
    {
        m_config_out_tensor_size.clear();;
    }

    // false: not updated yet
    bool testset_dynamic_out_shape_updated()
    {
        bool flag = false;

        m_dynamic_out_shape_updated_mtx.lock();
        flag =  m_dynamic_out_shape_updated;
        if (!m_dynamic_out_shape_updated)
            m_dynamic_out_shape_updated = true;
        m_dynamic_out_shape_updated_mtx.unlock();

        return flag;
    }

    void reset_dynamic_out_shape_updated_flag()
    {
        m_dynamic_out_shape_updated_mtx.lock();
        m_dynamic_out_shape_updated = false;
        m_dynamic_out_shape_updated_mtx.unlock();
    }

    bool is_dynamic_out_shape_updated()
    {
        return m_dynamic_out_shape_updated;
    }

    virtual void set_enrty(uint32_t offset){};

    /* Get functions */
    const char* get_bweight_base()
    {
        return m_bweight.va;
    }
    virtual DEV_PA_64 debugger_get_instr_base()
    {
        return m_text->pa;
    }

    uint32_t get_zerocpy_const_size()
    {
        return zerocpy_const_size;
    }

    uint32_t get_const_size()
    {
        return const_size;
    }

    void set_const_size(uint32_t _const_size, uint32_t _zerocpy_const_size)
    {
        /**
         * if one graph doesn't need weight, it just reserves
         * 4KB as default placehold for whole flow.
         */
        if (_const_size == 0)
            _const_size = 4096;

        const_size = _const_size;
        zerocpy_const_size = _zerocpy_const_size;
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

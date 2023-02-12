// Copyright (C) 2022 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  job_base.h
 * @brief AIPU User Mode Driver (UMD) job base class header
 */

#ifndef _JOB_BASE_H_
#define _JOB_BASE_H_

#include <vector>
#include <tuple>
#include <pthread.h>
#include "standard_api.h"
#include "context.h"
#include "graph.h"
#include "device_base.h"
#include "memory_base.h"
#include "type.h"

namespace aipudrv
{
enum JOBIOBufferType
{
    AIPU_JOB_BUFFER_VOID = 0,
    AIPU_JOB_BUFFER_INTERNAL,
    AIPU_JOB_BUFFER_DMABUF_IMPORTED,
    AIPU_JOB_BUFFER_DMABUF_EXPORTED,
};

struct JobIOBuffer
{
    uint32_t  id;
    uint32_t  size;
    JOBIOBufferType type;
    DEV_PA_64 pa; /* used when type != AIPU_JOB_BUFFER_DMABUF_IMPORTED */
    int  fd;      /* used when type == AIPU_JOB_BUFFER_DMABUF_* */
    void init(uint32_t _id, uint32_t _size, JOBIOBufferType _type, DEV_PA_64 _pa)
    {
        id   = _id;
        size = _size;
        type = _type;
        pa   = _pa;
        fd   = 0;
    }
};

typedef enum
{
    AIPU_JOB_STATUS_INIT  = 3,
    AIPU_JOB_STATUS_SCHED = 4,
    AIPU_JOB_STATUS_BIND  = 5,
} aipu_job_status_internal_t;

class JobBase
{
protected:
    MainContext*      m_ctx;
    JOB_ID            m_id;
    GraphBase&        m_graph;
    DeviceBase*       m_dev;
    MemoryBase*       m_mem;
    uint32_t          m_remap_flag = 0;

protected:
    /* shared buffers */
    BufferDesc m_rodata;
    BufferDesc m_descriptor;
    BufferDesc m_pprint;
    std::vector<struct JobIOBuffer> m_inputs;
    std::vector<struct JobIOBuffer> m_outputs;
    std::vector<struct JobIOBuffer> m_inter_dumps;
    std::vector<struct JobIOBuffer> m_profiler;
    std::vector<struct JobIOBuffer> m_printf;
    std::vector<struct JobIOBuffer> m_layer_counter;
    std::vector<struct JobIOBuffer> m_err_code;
    std::vector<struct JobIOBuffer> m_segmmus;

protected:
    bool m_dump_text = false;
    bool m_dump_weight = false;
    bool m_dump_reuse = false;
    bool m_dump_rodata = false;
    bool m_dump_dcr = false;
    bool m_dump_input = false;
    bool m_dump_output = false;
    bool m_dump_tcb = false;
    bool m_dump_emu = false;

    /* for X2 profile dump control */
    bool m_dump_profile = false;

    std::string m_dump_dir = "./";
    std::string m_dump_prefix = "temp";
    std::string m_dump_output_prefix = "temp";
    std::string m_dump_misc_prefix = "";
    bool m_support_dma_buf = false;
    int m_profile_fd = -1;

protected:
    uint32_t m_status = AIPU_JOB_STATUS_NO_STATUS;

private:
    DEV_PA_64 get_base_pa(int sec_type, BufferDesc& rodata,
        BufferDesc& descriptor, bool align_asid);
    void create_io_buffers(std::vector<struct JobIOBuffer>& bufs,
        const std::vector<GraphIOTensorDesc>& desc,
        const std::vector<BufferDesc>& reuses);

protected:
    aipu_status_t setup_rodata(
        const std::vector<struct GraphParamMapLoadDesc>& param_map,
        const std::vector<BufferDesc>& reuse_buf,
        const std::vector<BufferDesc>& static_buf,
        BufferDesc rodata, BufferDesc dcr);
    virtual Graph& get_graph()
    {
        return static_cast<Graph&>(m_graph);
    }

    virtual uint32_t get_subgraph_cnt() = 0;
    virtual const std::vector<BufferDesc> & get_reuse() = 0;
    void setup_remap(BufferDesc& rodata, BufferDesc& descriptor);
    void create_io_buffers(const struct GraphIOTensors& io,
        const std::vector<BufferDesc>& reuses);
    void dump_buffer(DEV_PA_64 pa, const char* bin_va, uint32_t size, const char* name);
    void dump_single_buffer(DEV_PA_64 pa, uint32_t size, const char* name);
    void dump_job_shared_buffers();
    void dump_job_private_buffers(BufferDesc& rodata, BufferDesc& descriptor);
    void dump_job_shared_buffers_after_run();
    void dump_job_private_buffers_after_run(BufferDesc& rodata, BufferDesc& descriptor);
    aipu_status_t validate_schedule_status();

public:
    virtual aipu_status_t init(const aipu_global_config_simulation_t* cfg) = 0;
    virtual aipu_status_t schedule() = 0;
    virtual aipu_status_t destroy() = 0;
    aipu_status_t load_tensor(uint32_t tensor, const void* data);
    aipu_status_t get_tensor(aipu_tensor_type_t type, uint32_t tensor, void* data);
    aipu_status_t mark_shared_tensor(aipu_tensor_type_t type, uint32_t tensor, uint64_t &pa_addr);
    aipu_status_t assign_shared_tensor(aipu_tensor_type_t type, uint32_t tensor, uint64_t share_pa_addr);
    virtual aipu_status_t get_status(aipu_job_status_t* status);
    virtual aipu_status_t get_status_blocking(aipu_job_status_t* status, int32_t time_out);
    aipu_status_t config_mem_dump(uint64_t types, const aipu_job_config_dump_t* config);
    virtual void dumpcfg_alljob() {}
    virtual aipu_status_t config_simulation(uint64_t types, const aipu_job_config_simulation_t* config)
    {
        return AIPU_STATUS_SUCCESS;
    };
    virtual aipu_status_t bind_core(uint32_t core_id) = 0;
    virtual aipu_status_t debugger_run()
    {
        return AIPU_STATUS_ERROR_OP_NOT_SUPPORTED;
    }
    virtual aipu_status_t import_buffers(aipu_tensor_type_t type, int* fds)
    {
        return AIPU_STATUS_ERROR_OP_NOT_SUPPORTED;
    }
    virtual aipu_status_t export_buffers(aipu_tensor_type_t type, int* fds)
    {
        return AIPU_STATUS_ERROR_OP_NOT_SUPPORTED;
    }

public:
    std::tuple<std::string, uint32_t, uint32_t> m_dump_tcb_info[2];
    /* Set functions */
    void set_id(JOB_ID id)
    {
        m_id = id;
    }
    /* Get functions */
    JOB_ID get_id()
    {
        return m_id;
    }

public:
    JobBase(MainContext* ctx, GraphBase& graph, DeviceBase* dev);
    virtual ~JobBase();
    JobBase(const JobBase& job) = delete;
    JobBase& operator=(const JobBase& job) = delete;
};
}

#endif /* _JOB_BASE_H_ */
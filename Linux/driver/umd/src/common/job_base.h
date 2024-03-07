// Copyright (C) 2023-2024 Arm Technology (China) Co. Ltd.
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
    DEV_PA_64 align_asid_pa; /* alignd buffer address relative to ASID base */

    /**
     * if this iobuffer comes from dma_buf, needs below information
     */
    int  dmabuf_fd;
    uint32_t dmabuf_size;
    uint32_t offset_in_dmabuf;

    /**
     * if this buffer is managed by user self, it can't dump its data since UMD can't
     * get its mapped address.
     */
    bool dump_ignore_flag;

    void init(uint32_t _id, uint32_t _size, JOBIOBufferType _type, DEV_PA_64 _pa,
        DEV_PA_64 _align_asid_pa = 0, int _dmabuf_fd = -1, uint32_t _dmabuf_size = 0,
        uint32_t _offset_in_dmabuf = 0, bool _dump_ignore_flag = false)
    {
        id   = _id;
        size = _size;
        type = _type;
        pa   = _pa;
        align_asid_pa = _align_asid_pa;
        dmabuf_fd   = _dmabuf_fd;
        dmabuf_size = _dmabuf_size;
        offset_in_dmabuf = _offset_in_dmabuf;
        dump_ignore_flag = _dump_ignore_flag;
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
    BufferDesc *m_rodata = nullptr;
    BufferDesc *m_descriptor = nullptr;
    BufferDesc *m_pprint = nullptr;
    std::vector<struct JobIOBuffer> m_inputs;
    std::vector<struct JobIOBuffer> m_outputs;
    std::vector<struct JobIOBuffer> m_inter_dumps;
    std::vector<struct JobIOBuffer> m_profiler;
    std::vector<struct JobIOBuffer> m_printf;
    std::vector<struct JobIOBuffer> m_layer_counter;
    std::vector<struct JobIOBuffer> m_err_code;
    std::vector<struct JobIOBuffer> m_segmmus;
    std::vector<struct JobIOBuffer> m_outputs_shape;

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

    /* for aipu v3 profile dump control */
    bool m_dump_profile = false;

    std::string m_dump_dir = "./";
    std::string m_dump_prefix = "temp";
    std::string m_dump_output_prefix = "temp";
    std::string m_dump_misc_prefix = "";
    int m_profile_fd = -1;

protected:
    uint32_t m_status = AIPU_JOB_STATUS_NO_STATUS;

    /* call back function for handling job self */
    aipu_job_callback_func_t m_callback_func = nullptr;

    /**
     * set 'true' if successfully alloc a large buffer
     * to gather more scatter buffers
     */
    bool m_optimized_reuse_alloc = false;

protected:
    const aipu_global_config_simulation_t *m_cfg = nullptr;
    const aipu_global_config_hw_t *m_hw_cfg = nullptr;

protected:
    std::ofstream m_ro_entry_dump;
    std::string m_ro_entry_name = "./ro_entry.txt";

private:
    DEV_PA_64 get_base_pa(int sec_type, BufferDesc& rodata,
        BufferDesc* descriptor, bool align_asid);
    void create_io_buffers(std::vector<struct JobIOBuffer>& bufs,
        const std::vector<GraphIOTensorDesc>& desc,
        const std::vector<BufferDesc*>& reuses);

protected:
    aipu_status_t setup_rodata(
        const std::vector<struct GraphParamMapLoadDesc>& param_map,
        const std::vector<BufferDesc*>& reuse_buf,
        const std::vector<BufferDesc*>& static_buf,
        BufferDesc &rodata, BufferDesc *dcr,
        std::set<uint32_t> *dma_buf_idx = nullptr);
    virtual Graph& get_graph()
    {
        return static_cast<Graph&>(m_graph);
    }

    virtual uint32_t get_subgraph_cnt() = 0;
    virtual const std::vector<BufferDesc*> & get_reuse() = 0;
    void setup_remap(BufferDesc& rodata, BufferDesc* descriptor);
    void create_io_buffers(const struct GraphIOTensors& io,
        const std::vector<BufferDesc*>& reuses);
    void update_io_buffers(const struct GraphIOTensors& io,
        const std::vector<BufferDesc*>& reuses);
    void dump_buffer(DEV_PA_64 pa, const char* bin_va, uint32_t size, const char* name);
    void dump_single_buffer(DEV_PA_64 pa, uint32_t size, const char* name);
    void dump_share_buffer(struct JobIOBuffer &iobuf, const char* name, bool keep_name = false);
    int readwrite_dma_buf(struct JobIOBuffer &iobuf, void *data, bool read = true);
    void dump_job_shared_buffers();
    void dump_job_private_buffers(BufferDesc& rodata, BufferDesc* descriptor);
    void dump_job_shared_buffers_after_run();
    void dump_job_private_buffers_after_run(BufferDesc& rodata, BufferDesc* descriptor);
    aipu_status_t validate_schedule_status();

public:
    virtual aipu_status_t init(const aipu_global_config_simulation_t* cfg,
       const aipu_global_config_hw_t* hw_cfg) = 0;
    virtual aipu_status_t schedule() = 0;
    virtual aipu_status_t destroy() = 0;
    aipu_status_t load_tensor(uint32_t tensor, const void* data);
    aipu_status_t get_tensor(aipu_tensor_type_t type, uint32_t tensor, void* data);
    virtual aipu_status_t get_status(aipu_job_status_t* status);
    virtual aipu_status_t get_status_blocking(aipu_job_status_t* status, int32_t time_out);
    aipu_status_t config_mem_dump(uint64_t types, const aipu_job_config_dump_t* config);
    virtual void dumpcfg_alljob() {}
    virtual aipu_status_t specify_io_buffer(aipu_shared_tensor_info_t &tensor_info)
    {
        return AIPU_STATUS_SUCCESS;
    }

    virtual aipu_status_t config_simulation(uint64_t types, const aipu_job_config_simulation_t* config)
    {
        return AIPU_STATUS_SUCCESS;
    };
    virtual aipu_status_t bind_core(uint32_t core_id) = 0;
    virtual aipu_status_t debugger_run()
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

    Graph& get_base_graph()
    {
        return static_cast<Graph&>(m_graph);
    }

    void set_job_cb(aipu_job_callback_func_t _cb_wrap)
    {
        m_callback_func = _cb_wrap;
    }

    aipu_job_callback_func_t get_job_cb()
    {
        return m_callback_func;
    }

public:
    JobBase(MainContext* ctx, GraphBase& graph, DeviceBase* dev);
    virtual ~JobBase();
    JobBase(const JobBase& job) = delete;
    JobBase& operator=(const JobBase& job) = delete;
};
}

#endif /* _JOB_BASE_H_ */
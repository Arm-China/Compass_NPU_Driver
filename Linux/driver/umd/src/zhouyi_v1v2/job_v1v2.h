// Copyright (C) 2022-2023 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  job_v1v2.h
 * @brief AIPU User Mode Driver (UMD) aipu v1/v2 job class header
 */

#ifndef _JOB_V12_H_
#define _JOB_V12_H_

#include <vector>
#include <pthread.h>
#include "standard_api.h"
#include "graph_v1v2.h"
#include "job_base.h"
#include "type.h"

namespace aipudrv
{
class JobV12: public JobBase
{
private:
    DEV_PA_64 m_spc = 0;
    DEV_PA_64 m_intr_pc = 0;
    BufferDesc m_stack;
    std::vector<BufferDesc> m_reuses;
    std::vector<BufferDesc> m_weights; /* Do NOT free me in this class */
    std::string m_sim;
    uint32_t m_log_level = 0;
    bool m_en_eval = false;
    std::string m_data_dir;
    std::string m_log_path;
    bool m_is_defer_run = false;
    bool m_do_trigger = false;
    uint32_t m_bind_core_id = 0;
    uint32_t m_fm_mem_region = AIPU_BUF_REGION_DEFAULT;

public:
    /**
     * record buffer index, will not free these special buffer as it is
     * allocated externally.
     */
    std::set<uint32_t>   m_dma_buf_idx;

private:
    GraphV12& get_graph()
    {
        return static_cast<GraphV12&>(m_graph);
    }

    virtual uint32_t get_subgraph_cnt()
    {
        return 1;
    }

    const std::vector<BufferDesc> & get_reuse() override
    {
        return static_cast< std::vector<BufferDesc>& >(m_reuses);
    }
    aipu_status_t free_job_buffers();
    aipu_status_t setup_rodata_v12(std::set<uint32_t> *dma_buf_idx = nullptr);

public:
    virtual aipu_status_t init(const aipu_global_config_simulation_t* cfg,
        const aipu_global_config_hw_t* hw_cfg);
    virtual aipu_status_t schedule();
    virtual aipu_status_t destroy();
    virtual aipu_status_t specify_io_buffer(uint32_t type, uint32_t index,
        uint64_t offset, int fd = -1, bool update_ro = true);
    aipu_status_t config_simulation(uint64_t types, const aipu_job_config_simulation_t* config);
    aipu_status_t bind_core(uint32_t core_id);
    aipu_status_t debugger_run();

public:
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
    JobV12(MainContext* ctx, GraphBase& graph, DeviceBase* dev,
        aipu_create_job_cfg_t *config = nullptr);
    virtual ~JobV12();
    JobV12(const JobV12& job) = delete;
    JobV12& operator=(const JobV12& job) = delete;
};
}

#endif /* _JOB_V12_H_ */
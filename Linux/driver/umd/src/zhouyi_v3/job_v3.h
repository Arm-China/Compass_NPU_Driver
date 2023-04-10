// Copyright (C) 2022-2023 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  job_v3.h
 * @brief AIPU User Mode Driver (UMD) job class header
 */

#ifndef _JOB_V3_H_
#define _JOB_V3_H_

#include <vector>
#include <memory>
#include <pthread.h>
#include "graph_v3.h"
#include "kmd/tcb.h"
#include "job_base.h"
#include "gm.h"

namespace aipudrv
{
struct TCB
{
    DEV_PA_64 pa;
    void init(DEV_PA_64 _pa)
    {
        pa = _pa;
    }
};

struct Task
{
    TCB        tcb;
    BufferDesc stack;
    BufferDesc private_data;
};

struct SubGraphTask
{
    uint32_t id;
    std::vector<BufferDesc> reuses;
    std::vector<BufferDesc> reuse_priv_buffers;
    std::vector<BufferDesc> weights;
    std::vector<Task>       tasks;
    void reset(uint32_t _id)
    {
        id = _id;
        reuses.clear();
        weights.clear();
        reuse_priv_buffers.clear();
        tasks.clear();
    }
};

struct dumpcfg_input_desc
{
    std::string file;
    DEV_PA_64 base;
};

struct dumpcfg_output_desc
{
    std::string file;
    DEV_PA_64 base;
    uint32_t size;
};

struct dumpcfg_host_desc
{
    uint32_t part_id;
    uint32_t hi_addr;
    uint32_t lo_addr;
};

class JobV3: public JobBase
{
private:
    uint32_t    m_tot_tcb_cnt = 0;
    uint32_t    m_sg_cnt = 0;
    uint32_t    m_task_per_sg = 0;
    uint32_t    m_partition_id = 0;
    uint32_t    m_grid_id = 0;
    uint32_t    m_core_cnt = 0;
    uint32_t    m_qos = 0;
    uint32_t    m_fm_mem_region = AIPU_MEM_REGION_DEFAULT;
    bool        m_is_defer_run = false;
    bool        m_do_trigger = false;

private:
    BufferDesc m_tcbs;
    TCB m_init_tcb;
    std::unique_ptr<char []> m_backup_tcb;
    bool m_backup_tcb_used = false;
    std::vector<SubGraphTask> m_sg_job;
    std::map<uint32_t, GM_info_desc> m_gm_info[2];
    uint32_t m_segmmu_num = 0;
    uint32_t m_segmmu_tcb_num = 3;
    std::vector<SegMMUConfig> m_segmmu_sec;
    GM_V3 *m_gm = nullptr;

    std::string m_dumpcfg_header;
    dumpcfg_host_desc m_dumpcfg_host;
    std::vector<dumpcfg_input_desc> m_dumpcfg_input;
    std::vector<dumpcfg_output_desc> m_dumpcfg_output;
    std::string m_dumpcfg_meta;

    /**
     * record stack/dp buffers for each allocated subgraph, try to
     * share them to subsequent subgraphs.
     */
    std::vector<SubGraphTask *> m_sgt_allocated;

public:
    GraphV3& get_graph()
    {
        return static_cast<GraphV3&>(m_graph);
    }

    virtual uint32_t get_subgraph_cnt()
    {
        return get_graph().get_subgraph_cnt();
    }

    const std::vector<BufferDesc> & get_reuse() override
    {
        return static_cast< std::vector<BufferDesc>& >(m_sg_job[0].reuses);
    }

private:
    aipu_status_t setup_rodata_sg(uint32_t sg_id, const std::vector<struct GraphParamMapLoadDesc>& param_map,
        std::vector<BufferDesc>& reuse_buf, std::vector<BufferDesc>& static_buf);
    aipu_status_t setup_tcb_task(uint32_t sg_id, uint32_t grid_id, uint32_t core_id, uint32_t task_id);
    aipu_status_t setup_tcb_sg(uint32_t sg_id, uint32_t grid_id, uint32_t core_id);
    void          set_job_params(uint32_t sg_cnt, uint32_t task_per_sg, uint32_t remap, uint32_t core_cnt);
    aipu_status_t alloc_load_job_buffers();
    aipu_status_t free_job_buffers();
    aipu_status_t alloc_subgraph_buffers();
    aipu_status_t init_per_task_data();
    aipu_status_t setup_tcbs();
    void setup_gm_sync_from_ddr(tcb_t *tcb);
    void setup_gm_sync_to_ddr(tcb_t *tcb);
    aipu_status_t setup_segmmu(SubGraphTask &sg);
    void free_sg_buffers(const SubGraphTask& sg);
    aipu_status_t dump_for_emulation();

public:
    aipu_status_t init(const aipu_global_config_simulation_t* cfg,
        const aipu_global_config_hw_t* hw_cfg);
    aipu_status_t schedule();
    aipu_status_t destroy();
    aipu_status_t bind_core(uint32_t core_id);
    aipu_status_t debugger_run();

    #if (defined(SIMULATION) && defined(ZHOUYI_V3))
    virtual void dumpcfg_alljob();
    #endif

    void dump_specific_buffers();
    aipu_status_t import_buffers(aipu_tensor_type_t type, int* fds);
    aipu_status_t export_buffers(aipu_tensor_type_t type, int* fds);

public:
    /* for simulation, record which cmdpool this job is committed to */
    uint32_t m_bind_cmdpool_id;

    /* Set functions */

    /* Get functions */
    JOB_ID get_id()
    {
        return m_id;
    }

    uint32_t get_part_id()
    {
        return m_partition_id;
    }

    uint32_t get_qos()
    {
        return m_qos;
    }

public:
    JobV3(MainContext* ctx, GraphBase& graph, DeviceBase* dev, aipu_create_job_cfg_t *config = nullptr);
    ~JobV3();
    JobV3(const JobV3& job) = delete;
    JobV3& operator=(const JobV3& job) = delete;

    friend class GM_V3;
};
}

#endif /* _JOB_V3_H_ */
// Copyright (C) 2022 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  context.h
 * @brief AIPU User Mode Driver (UMD) context module header
 */

#ifndef _CONTEXT_H_
#define _CONTEXT_H_

#include <map>
#include <cstring>
#include <fstream>
#include <pthread.h>
#include "standard_api.h"
#include "graph_base.h"
#include "device_base.h"
#include "memory_base.h"

namespace aipudrv
{
typedef std::map<GRAPH_ID, GraphBase*> GraphTable;

class MainContext
{
private:
    DeviceBase* m_dev = nullptr;
    MemoryBase* m_dram = nullptr;
    GraphTable  m_graphs;
    pthread_rwlock_t m_glock;
    bool m_do_vcheck = true;
    std::map<void*, BufferDesc> m_dbg_buffers;

private:
    static std::map<uint32_t, std::string> umd_status_string;
    aipu_status_t m_last_err = AIPU_STATUS_SUCCESS;
    char          m_last_err_msg[2048];

private:
    aipu_global_config_simulation_t m_sim_cfg;
    aipu_global_config_hw_t m_hw_cfg;

private:
    uint64_t create_unique_graph_id_inner() const;
    aipu_status_t create_graph_object(std::istream& gbin, uint32_t size, uint64_t id, GraphBase** gobj);
    aipu_status_t destroy_graph_object(GraphBase** gobj);

private:
    bool is_deinit_ok();

public:
    static const char* get_static_msg(aipu_status_t err);

public:
    /* x2 core APIs */
    aipu_status_t init();
    void force_deinit();
    aipu_status_t deinit();
    GraphBase*    get_graph_object(GRAPH_ID id);
    JobBase*      get_job_object(JOB_ID id);
    aipu_status_t get_status_msg(aipu_status_t status, const char** msg);
    aipu_status_t load_graph(const char* graph_file, GRAPH_ID* id);
    aipu_status_t load_graph(const char* graph_buf, uint32_t graph_size, GRAPH_ID* id);
    aipu_status_t unload_graph(GRAPH_ID id);
    aipu_status_t get_simulation_instance(void** simulator, void** memory);
    aipu_status_t create_job(GRAPH_ID graph, JOB_ID* id, aipu_create_job_cfg_t *config);
    aipu_status_t get_partition_count(uint32_t* cnt);
    aipu_status_t get_cluster_count(uint32_t partition_id, uint32_t* cnt);
    aipu_status_t get_core_count(uint32_t partition_id, uint32_t cluster, uint32_t* cnt);
    aipu_status_t get_core_info(uint32_t core_id, aipu_core_info_t* info);
    aipu_status_t debugger_get_job_info(JOB_ID job, aipu_debugger_job_info_t* info);
    aipu_status_t config_simulation(uint64_t types, aipu_global_config_simulation_t* config);
    aipu_status_t config_hw(uint64_t types, aipu_global_config_hw_t* config);
    aipu_status_t aipu_get_target(char *target);
    aipu_status_t aipu_get_device_status(device_status_t *status);
    aipu_status_t run_batch(GraphBase &graph, uint32_t queue_id, aipu_create_job_cfg_t *config);
    aipu_status_t get_status(JobBase *job, aipu_job_status_t *status);
    aipu_status_t ioctl_cmd(uint32_t cmd, void *arg);

    void disable_version_check()
    {
        m_do_vcheck = false;
    }
    void enable_version_check()
    {
        m_do_vcheck = true;
    }
    aipu_status_t debugger_malloc(uint32_t size, void** va);
    aipu_status_t debugger_free(void* va);

    aipu_status_t log_err_msg(aipu_status_t err, const char* msg = nullptr)
    {
        m_last_err = err;
        if (msg != nullptr)
            strcpy(m_last_err_msg, msg);

        return m_last_err;
    }
    aipu_status_t log_rt_err_msg(uint32_t err, const char* msg = nullptr)
    {
        m_last_err = (aipu_status_t)(uint16_t)err;
        if (msg != nullptr)
            strcpy(m_last_err_msg, msg);
        else
            snprintf(m_last_err_msg, sizeof(m_last_err_msg), "%s (error code: 0x%x)",
                get_static_msg(m_last_err), err);

        return m_last_err;
    }

public:
    DeviceBase* get_dev()
    {
        return m_dev;
    };

    GraphTable &get_graphtable()
    {
        return m_graphs;
    }

public:
    MainContext(const MainContext& ctx) = delete;
    MainContext& operator=(const MainContext& ctx) = delete;
    ~MainContext();
    MainContext();
};
}

struct ctx_handle {
    uint32_t handle;
};

#endif /* _CONTEXT_H_ */

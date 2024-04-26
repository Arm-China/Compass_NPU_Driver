// Copyright (C) 2023-2024 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  simulator_v4.h
 * @brief AIPU User Mode Driver (UMD) zhouyi aipu v4 simulator module header
 */

#ifndef _SIMULATOR_V4_H_
#define _SIMULATOR_V4_H_

#include <map>
#include <set>
#include <queue>
#include <mutex>
#include <atomic>
#include <sstream>
#include <pthread.h>
#include "standard_api.h"
#include "device_base.h"
#include "umemory.h"
#include "simulator/aipu.h"
#include "simulator/config.h"
#include "kmd/tcb.h"
#include "utils/debug.h"
#include "job_v4.h"

namespace aipudrv
{

#define TSM_BUILD_INFO 0x14
#define TSM_STATUS     0x18
#define TSM_CMD_POOL0_CONFIG 0x800
#define TSM_CMD_POOL0_STATUS 0x804

#define CLUSTER_PRESENT 0x1000
#define CLUSTER_ENABLE  0x1000

#define MAX_PART_CNT 1
#define MAX_CLUSTER_CNT 4

enum {
    POOL_PCP = 0,
    POOL_SCP,
    POOL_MAX,
};

class SimulatorV4 : public DeviceBase
{
private:
    pthread_rwlock_t m_lock;
    std::mutex m_poll_mtex;
    sim_aipu::config_t m_config;
    sim_aipu::Aipu *m_aipu = nullptr;
    uint32_t m_code = 0;
    uint32_t m_log_level;
    std::string m_log_filepath;
    bool m_verbose;
    bool m_enable_avx;
    bool m_en_eval;
    uint32_t m_gm_size = 0;
    std::string m_plugin_filename;
    std::string m_json_filename;
    std::string m_arch_desc;
    std::vector<uint32_t> m_cluster_in_part[MAX_PART_CNT];
    uint32_t m_max_cmdpool_cnt = 0;
    std::vector<BufferDesc*> m_reserve_mem;

    /**
     * @cmdpool_id: the next cmdpool index in one partition
     * @m_cmdpool_in_part: the cmdpool number belong to one partition
     */
    struct cmdpool_info
    {
        uint32_t cmdpool_id;
        std::vector<uint32_t> m_cmdpool_in_part;
    };
    std::map<uint32_t, cmdpool_info> m_part_cmdpool;

    /* 1. buffer all jobs in this queue */
    typedef struct {
        void *job;
        struct JobDesc jobdesc;
    } job_queue_elem_t;
    std::queue< job_queue_elem_t > m_buffer_queue;

    /* 2. move jobs from buffer queue to this queue */
    std::map< uint16_t, void * > m_commit_map;

    /* 3. move jobs from commit queue to this queue when cmdpool done ready */
    std::set< void * > m_done_set;

    /* 4. Simulator puts all done jobs to this queue */
    static std::set< uint16_t > m_sim_done_grid_set;
    static std::mutex m_sim_done_grid_mtx;

    volatile bool m_cmdpool_busy = false;

    std::atomic<uint16_t> m_grid_id = {0};

    #define MAX_GROUP_ID (1 << 15)
    bool m_group_id_bitmap[MAX_GROUP_ID] = {false};
    std::mutex m_group_id_mtx;

    uint32_t m_partition_mode = POOL_PCP;
    std::map<uint32_t, std::map<uint32_t, uint32_t>> m_cmdpool_id =
        {
            {
                0, {
                    {0, 0},
                    {1, 1},
                    {2, 2},
                    {3, 3},
                },
            },
            {
                1, {
                    {0, 4},
                    {1, 5},
                    {2, 6},
                    {3, 7},
                }
            },
        };

private:
    bool is_cmdpool_full(int qos, int part_id, int partition_mode,
        int cluster_idx, uint32_t reg_val);
    uint32_t get_cmdpool_id(uint32_t cluster_id, uint32_t part_id)
    {
        return m_cmdpool_id[cluster_id][part_id];
    }

    void parse_cluster_info()
    {
        uint32_t reg_val = 0;
        uint32_t present_cluster_cnt = 0;
        aipu_partition_cap part_cap;

        for (uint32_t i = 0; i < MAX_CLUSTER_CNT; i++)
        {
            m_aipu->read_register(CLUSTER0_CONFIG + 0x20 * i, reg_val);
            if (reg_val & CLUSTER_PRESENT)
                present_cluster_cnt++;
        }

        /* currently the aipu v3a simulation only for one partition, one cluster in partition */
        memset((void *)&part_cap, 0x0, sizeof(part_cap));
        for (uint32_t cluster_idx = 0; cluster_idx < present_cluster_cnt; cluster_idx++)
        {
            m_aipu->read_register(CLUSTER0_CTRL + 0x20 * cluster_idx, reg_val);
            if (!(reg_val & CLUSTER_ENABLE))
                continue;

            /* specify cluster to support core partition */
            reg_val |= m_partition_mode << 13;
            m_aipu->write_register(CLUSTER0_CTRL + 0x20 * cluster_idx, reg_val);

            part_cap.id = part_cap.cluster_cnt;
            part_cap.arch = AIPU_ARCH_ZHOUYI;
            part_cap.version = AIPU_ISA_VERSION_ZHOUYI_V3;
            part_cap.config = 1304;
            part_cap.clusters[part_cap.cluster_cnt].core_cnt = (reg_val >> 8) & 0xF;
            part_cap.clusters[part_cap.cluster_cnt].tec_cnt = reg_val & 0xF;
            part_cap.cluster_cnt++;

            m_partition_cnt++;
            m_part_caps.push_back(part_cap);
        }

        if (m_partition_cnt > 0)
        {
            m_cluster_cnt = m_part_caps[0].cluster_cnt;
            m_core_cnt = m_part_caps[0].clusters[0].core_cnt;
        }
    }

public:
    virtual int get_grid_id(uint16_t &grid_id);
    virtual int get_start_group_id(int group_cnt, uint16_t &start_group_id);

    int put_start_group_id(uint16_t start_group_id, int group_cnt)
    {
        int ret = 0;

        m_group_id_mtx.lock();
        for (uint16_t i = start_group_id; i < start_group_id + group_cnt; i++)
            m_group_id_bitmap[i] = false;
        m_group_id_mtx.unlock();

        return ret;
    }

public:
    UMemory *get_umemory(void)
    {
        return static_cast<UMemory*>(m_dram);
    }

public:
    bool has_target(uint32_t arch, uint32_t version, uint32_t config, uint32_t rev);
    aipu_status_t parse_config(uint32_t config, uint32_t &code);
    aipu_status_t schedule(const JobDesc& job);
    aipu_status_t fill_commit_queue();
    aipu_ll_status_t poll_status(uint32_t max_cnt, int32_t time_out,
        bool of_this_thread, void *jobbase = nullptr);
    static void sim_cb_handler(uint32_t event, uint64_t value, void *context);

    aipu_status_t get_simulation_instance(void** simulator, void** memory)
    {
        if (m_aipu != nullptr)
        {
            *(sim_aipu::Aipu**)simulator = m_aipu;
            *(sim_aipu::IMemEngine**)memory = static_cast<UMemory*>(m_dram);
            return AIPU_STATUS_SUCCESS;
        }
        return AIPU_STATUS_ERROR_INVALID_OP;
    }

    const char *get_config_code()
    {
        std::map<uint32_t, const char *> npu_sim_codetoname =
        {
            {3, "tbd1"},
            {4, "tbd2"}
        };

        if (npu_sim_codetoname.count(m_config.code) == 1)
            return npu_sim_codetoname[m_config.code];
        else
            return npu_sim_codetoname[sim_aipu::config_t::tbd1];
    }

public:
    virtual void enable_profiling(bool en)
    {
        m_aipu->enable_profiling(en);
    }

public:
    static SimulatorV4* get_v4_simulator(const aipu_global_config_simulation_t* cfg)
    {
        static SimulatorV4 sim_instance(cfg);
        sim_instance.inc_ref_cnt();
        return &sim_instance;
    }
    aipu_status_t get_cluster_id(uint32_t part_id, std::vector<uint32_t> &cluster_in_part)
    {
        if (part_id > sizeof(m_cluster_in_part))
            return AIPU_STATUS_ERROR_INVALID_PARTITION_ID;

        cluster_in_part = m_cluster_in_part[part_id];
        return AIPU_STATUS_SUCCESS;
    }

    virtual ~SimulatorV4();
    SimulatorV4(const SimulatorV4& sim) = delete;
    SimulatorV4& operator=(const SimulatorV4& sim) = delete;

private:
    SimulatorV4(const aipu_global_config_simulation_t* cfg);
};

inline sim_aipu::config_t sim_create_config(int code, uint32_t log_level = 0,
    const std::string &log_path = "./sim.log", bool verbose = false,
    bool enable_avx = false, bool en_eval = 0, uint32_t gm_size = 4 * MB_SIZE,
    const std::string &plugin_filename = "", const std::string &json_filename = "")
{
    sim_aipu::config_t config = {0};

    config.code = code;
    config.enable_calloc = false;
    config.max_pkg_num = -1;
    config.enable_avx = enable_avx;
    config.en_eval = en_eval;
    config.log.filepath = log_path;
    config.log.level = log_level;
    config.log.verbose = verbose;
    config.gm_size = gm_size;
    config.plugin_filename = plugin_filename;
    config.json_filename = json_filename;
    config.freq_mhz = 1000;
    config.ddr_latency_rd = 0;
    config.ddr_latency_wr = 0;
    config.ddr_bw = 512;

    LOG(LOG_DEBUG, "\nconfig.code = %d\n"
        "config.enable_calloc = %d\n"
        "config.max_pkg_num = %ld\n"
        "config.enable_avx = %d\n"
        "config.en_eval = %d\n"
        "config.log.filepath = %s\n"
        "config.log.level = %d\n"
        "config.log.verbose = %d\n"
        "config.gm_size = 0x%x\n"
        "config.plugin_filename = %s\n"
        "config.json_filename = %s\n",
        config.code, config.enable_calloc, config.max_pkg_num, config.enable_avx,
        config.en_eval, config.log.filepath.c_str(), config.log.level,
        config.log.verbose, config.gm_size, config.plugin_filename.c_str(), config.json_filename.c_str());

    return config;
}

}

#endif /* _SIMULATOR_V4_H_ */

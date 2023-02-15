// Copyright (C) 2022 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  simulator_v3.cpp
 * @brief AIPU User Mode Driver (UMD) zhouyi x2 simulator module implementation
 */

#include <cstring>
#include <unistd.h>
#include <assert.h>
#include "simulator_v3.h"

aipudrv::SimulatorV3* aipudrv::SimulatorV3::m_sim = nullptr;

aipudrv::SimulatorV3::SimulatorV3(const aipu_global_config_simulation_t* cfg)
{
    BufferDesc *desc = new BufferDesc;
    m_dev_type = DEV_TYPE_SIMULATOR_V3;
    m_dram = UMemory::get_memory();
    if (nullptr == cfg)
    {
        m_log_level = RTDEBUG_SIMULATOR_LOG_LEVEL;
        m_verbose = false;
        m_enable_avx = false;
        m_en_eval = false;
        m_gm_size = 4 * MB_SIZE;
    } else {
        m_log_level = cfg->log_level;
        m_verbose = cfg->verbose;
        m_enable_avx = cfg->enable_avx;
        m_en_eval = cfg->en_eval;
        m_gm_size = cfg->gm_size;

        if (cfg->log_file_path != nullptr)
            m_log_filepath = cfg->log_file_path;
        if (cfg->x2_arch_desc != nullptr)
            m_arch_desc = cfg->x2_arch_desc;
    }
    pthread_rwlock_init(&m_lock, NULL);

    /* reserve 1KB for debug */
    m_dram->reserve_mem(0xC1000000, 1024, desc, "rsv");
    m_reserve_mem.push_back(desc);
}

aipudrv::SimulatorV3::~SimulatorV3()
{
    if (m_aipu != nullptr)
    {
        cmd_pool_destroy();

        delete m_aipu;
        m_aipu = nullptr;
    }

    for (auto buf : m_reserve_mem)
    {
        m_dram->free(buf, "rsv");
        delete buf;
    }

    pthread_rwlock_destroy(&m_lock);
    m_dram = nullptr;
    m_sim = nullptr;
}

aipu_status_t aipudrv::SimulatorV3::parse_config(uint32_t config, uint32_t &sim_code)
{
    std::string key = "null";
    typedef struct {
        uint32_t config;
        uint32_t core;
        uint32_t cluster;
        uint32_t sim_code;
    } arch_item_t;
    std::map<std::string, arch_item_t> npu_arch_map =
    {
        {"X2_1204", { 1204, 1, 1, sim_aipu::config_t::X2_1204}},
        {"X2_1204MP3", { 1204, 3, 1, sim_aipu::config_t::X2_1204MP3}}
    };

    if (!m_arch_desc.empty() && npu_arch_map.count(m_arch_desc) > 0)
    {
        sim_code = npu_arch_map[m_arch_desc].sim_code;
        key = m_arch_desc;
    } else {
        if (config == 1204) {
            key = "X2_1204";
        } else {
            LOG(LOG_ERR, "Only support: X2_1204/X2_1204MP3\n");
            return AIPU_STATUS_ERROR_TARGET_NOT_FOUND;
        }

        if (npu_arch_map.count(key) > 0)
        {
            sim_code = npu_arch_map[key].sim_code;
        } else {
            LOG(LOG_ERR, "No KUN target: %d\n", config);
            return AIPU_STATUS_ERROR_TARGET_NOT_FOUND;
        }
    }

    if (npu_arch_map[key].config != 1204)
    {
        LOG(LOG_ERR, "Only support: X2_1204/X2_1204MP3\n");
        return AIPU_STATUS_ERROR_TARGET_NOT_FOUND;
    }

    return AIPU_STATUS_SUCCESS;
}

bool aipudrv::SimulatorV3::has_target(uint32_t arch, uint32_t version, uint32_t config, uint32_t rev)
{
    aipu_partition_cap aipu_cap = {0};
    uint32_t reg_val = 0, sim_code = 0;
    bool ret = false;

    if ((arch != AIPU_ARCH_ZHOUYI) || (version != AIPU_ISA_VERSION_ZHOUYI_X2) || (rev != 0))
        return false;

    pthread_rwlock_wrlock(&m_lock);
    if (m_aipu != nullptr)
    {
        ret = true;
        goto unlock;
    }

    if (parse_config(config, sim_code))
    {
        ret = false;
        goto unlock;
    }

    m_config = sim_create_config(sim_code, m_log_level, m_log_filepath,
        m_verbose, m_enable_avx, m_en_eval, m_gm_size);
    m_aipu = new sim_aipu::Aipu(m_config, static_cast<UMemory&>(*m_dram));
    if (m_aipu == nullptr)
    {
        ret = false;
        goto unlock;
    }

    if (sim_code == sim_aipu::config_t::X2_1204 || sim_code == sim_aipu::config_t::X2_1204MP3)
        m_dram->gm_init(m_config.gm_size);

    m_code = sim_code;
    m_aipu->read_register(TSM_BUILD_INFO, reg_val);
    m_max_partition_cnt = ((reg_val >> 24) & 0xf) + 1;
    m_max_cmdpool_cnt = (reg_val >> 16) & 0xf;

    if (m_max_partition_cnt > MAX_PART_CNT)
    {
        LOG(LOG_ERR, "Invalid config: max_part %d\n", m_max_partition_cnt);
        ret = false;
        goto unlock;
    }

    if (set_cluster_to_partition() != AIPU_STATUS_SUCCESS)
    {
        ret = false;
        goto unlock;
    }

    /* assign cmdpool id to one partition */
    if (set_cmdpool_to_partition(m_partition_cnt, m_max_cmdpool_cnt) != AIPU_STATUS_SUCCESS)
    {
        ret = false;
        goto unlock;
    }

    aipu_cap.arch = arch;
    aipu_cap.version = version;
    aipu_cap.config = config;
    m_part_caps.push_back(aipu_cap);

    ret = true;

unlock:
    pthread_rwlock_unlock(&m_lock);
    return ret;
}

aipu_status_t aipudrv::SimulatorV3::schedule(const JobDesc& jobdesc)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    JobV3 *job = static_cast<JobV3 *>(jobdesc.jobbase);
    uint32_t part_id = job->get_part_id();
    uint32_t cmd_pool_id = 0, value = 0;
    uint32_t qos = job->get_qos();

    if (part_id > m_partition_cnt)
    {
        ret = AIPU_STATUS_ERROR_INVALID_PARTITION_ID;
        goto out;
    }

    assert(m_aipu != nullptr);

    pthread_rwlock_wrlock(&m_lock);
    if (job->m_bind_cmdpool_id == 0xffffffff)
        cmd_pool_id= job->m_bind_cmdpool_id = get_cmdpool_id(part_id);
    else
        cmd_pool_id= job->m_bind_cmdpool_id;

    if (!cmd_pool_created(cmd_pool_id))
    {
        cmd_pool_add_job(cmd_pool_id, job, jobdesc);
        m_aipu->write_register(TSM_CMD_SCHED_ADDR_HI, get_high_32(jobdesc.tcb_head));
        m_aipu->write_register(TSM_CMD_SCHED_ADDR_LO, get_low_32(jobdesc.tcb_head));

        /* specify cmdpool number & QoS */
        value = (cmd_pool_id << 16) | (qos << 8);
        m_aipu->write_register(TSM_CMD_SCHED_CTRL, value | CREATE_CMD_POOL);

        /* bind cmdpool to a partition */
        m_aipu->read_register(TSM_CMD_POOL0_CONFIG + cmd_pool_id * 0x40, value);
        value &= ~0xf;
        value |= part_id & 0xf;
        m_aipu->write_register(TSM_CMD_POOL0_CONFIG + cmd_pool_id * 0x40, value);

        LOG(LOG_INFO, "triggering simulator...");
        m_aipu->write_register(TSM_CMD_SCHED_CTRL, DISPATCH_CMD_POOL);
    } else {
        cmd_pool_append_job(cmd_pool_id, job, jobdesc);
    }
    pthread_rwlock_unlock(&m_lock);

out:
    return ret;
}

aipu_ll_status_t aipudrv::SimulatorV3::get_status(std::vector<aipu_job_status_desc>& jobs_status,
    uint32_t max_cnt, void *jobbase)
{
    uint32_t value = 0;
    aipu_job_status_desc desc;
    JobV3 *job = static_cast<JobV3 *>(jobbase);
    uint32_t cmd_pool_id = job->m_bind_cmdpool_id;
    uint32_t cmd_pool_status_reg = CMD_POOL0_STATUS + 0x40 * cmd_pool_id;

    if (job->get_subgraph_cnt() == 0)
        return AIPU_LL_STATUS_SUCCESS;

    /**
     * dump a combination runtime.cfg for all jobs in one running period
     */
    static bool dumpcfg_flag = false;
    if (!dumpcfg_flag)
    {
        dumpcfg_flag = true;
        job->dumpcfg_alljob();
    }

    job->dump_specific_buffers();
    if (cmd_pool_job_is_in(cmd_pool_id, job))
    {
        /**
         * note:
         * just poll once for all tcb subchains in a common cmdpool.
         * as simulator will handle all tcb subchains then set `IDLE`
         * bit of corresponding cmdpool status
         */
        if (m_cmdpools[cmd_pool_id]->destroy_done() == false)
        {
            m_cmdpools[cmd_pool_id]->set_destroy_flag();
            while (m_aipu->read_register(cmd_pool_status_reg, value) > 0)
            {
                if (value & CMD_POOL0_IDLE)
                {
                    m_aipu->write_register(TSM_CMD_SCHED_CTRL, DESTROY_CMD_POOL);
                    LOG(LOG_INFO, "simulation done.");
                    break;
                }
                sleep(1);
                LOG(LOG_INFO, "wait for simulation execution...");
            }
        }

        desc.state = AIPU_JOB_STATE_DONE;
        jobs_status.push_back(desc);
        cmd_pool_erase_job(cmd_pool_id, job);
    }

    return AIPU_LL_STATUS_SUCCESS;
}

aipu_ll_status_t aipudrv::SimulatorV3::poll_status(std::vector<aipu_job_status_desc>& jobs_status,
    uint32_t max_cnt, int32_t time_out, bool of_this_thread, void *jobbase)
{
    uint32_t value = 0;
    aipu_job_status_desc desc;
    JobV3 *job = static_cast<JobV3 *>(jobbase);
    uint32_t cmd_pool_id = job->m_bind_cmdpool_id;
    uint32_t cmd_pool_status_reg = CMD_POOL0_STATUS + 0x40 * cmd_pool_id;

    if (job->get_subgraph_cnt() != 0)
    {
        while (m_aipu->read_register(cmd_pool_status_reg, value) > 0)
        {
            if (value & CMD_POOL0_IDLE)
            {
                m_aipu->write_register(TSM_CMD_SCHED_CTRL, DESTROY_CMD_POOL);
                LOG(LOG_INFO, "simulation done.");
                break;
            }
            sleep(1);
            LOG(LOG_INFO, "wait for simulation execution...");
        }
        cmd_pool_erase_job(cmd_pool_id, job);
    }

    desc.state = AIPU_JOB_STATE_DONE;
    jobs_status.push_back(desc);

    return AIPU_LL_STATUS_SUCCESS;
}

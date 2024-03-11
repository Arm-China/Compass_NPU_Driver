// Copyright (C) 2023-2024 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  simulator_v4.cpp
 * @brief AIPU User Mode Driver (UMD) zhouyi aipu v4 simulator module implementation
 */

#include <cstring>
#include <unistd.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include "simulator_v4.h"
#include "helper.h"

/**
 * The bellow are used for syncing between UMD and Simulator
 * when some grid job is done. Simulator will directly call
 * one callback to notify UMD and convey the done grid's ID to UMD.
 */
std::set< uint16_t > aipudrv::SimulatorV4::m_sim_done_grid_set = {};
std::mutex aipudrv::SimulatorV4::m_sim_done_grid_mtx;
std::mutex simv4_mtx;
std::condition_variable simv4_cv;
bool simv4_has_grid_done = false;

bool has_some_grid_done()
{
    return simv4_has_grid_done;
}

aipudrv::SimulatorV4::SimulatorV4(const aipu_global_config_simulation_t* cfg)
{
    m_dev_type = DEV_TYPE_SIMULATOR_V4;
    m_dram = UMemory::get_memory();
    if (nullptr == cfg)
    {
        m_log_level = RTDEBUG_SIMULATOR_LOG_LEVEL;
        m_verbose = false;
        m_enable_avx = false;
        m_en_eval = false;
        m_gm_size = 8 * MB_SIZE;
    } else {
        m_log_level = cfg->log_level;
        m_verbose = cfg->verbose;
        m_enable_avx = cfg->enable_avx;
        m_en_eval = cfg->en_eval;
        m_gm_size = cfg->gm_size;

        if (cfg->plugin_name != nullptr)
            m_plugin_filename = cfg->plugin_name;
        if (cfg->json_filename != nullptr)
            m_json_filename = cfg->json_filename;
        if (cfg->log_file_path != nullptr)
            m_log_filepath = cfg->log_file_path;
        if (cfg->npu_arch_desc != nullptr)
            m_arch_desc = cfg->npu_arch_desc;
    }
    pthread_rwlock_init(&m_lock, NULL);
}

aipudrv::SimulatorV4::~SimulatorV4()
{
    if (m_aipu != nullptr)
    {
        delete m_aipu;
        m_aipu = nullptr;
    }

    pthread_rwlock_destroy(&m_lock);
    m_dram = nullptr;
}

int aipudrv::SimulatorV4::get_grid_id(uint16_t &grid_id)
{
    grid_id = m_grid_id++;
    return 0;
}

int aipudrv::SimulatorV4::get_start_group_id(int group_cnt, uint16_t &start_group_id)
{
    int ret = 0;

    m_group_id_mtx.lock();
    for (int i = 0; i < MAX_GROUP_ID; i++)
    {
        if (m_group_id_bitmap[i] == false)
        {
            int j = 0;

            if (i + group_cnt >= MAX_GROUP_ID)
            {
                ret = -1;
                LOG(LOG_ERR, "Group ID bit map overflow\n");
                goto out;
            }

            for (j = i; j < i + group_cnt; j++)
            {
                if (m_group_id_bitmap[j] == true)
                {
                    ret = -1;
                    break;
                }
            }

            start_group_id = i;
            ret = 0;

            for (j = i; j < i + group_cnt; j++)
                m_group_id_bitmap[j] = true;

            break;
        }
    }

out:
    m_group_id_mtx.unlock();
    return ret;
}

aipu_status_t aipudrv::SimulatorV4::parse_config(uint32_t config, uint32_t &sim_code)
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
        {"tbd1", { 1304, 1, 1, 3}},
        {"tbd2", { 1304, 4, 1, 4}}
    };

    if (!m_arch_desc.empty() && npu_arch_map.count(m_arch_desc) > 0)
    {
        sim_code = npu_arch_map[m_arch_desc].sim_code;
        key = m_arch_desc;
    } else {
        if (config == 1304) {
            key = "tbd1";
            LOG(LOG_ALERT, "Not support requested sim target: %s, switch to : %s\n",
                m_arch_desc.c_str(), key.c_str());
        } else {
            LOG(LOG_ERR, "Only support: tbd1/tbd2\n");
            return AIPU_STATUS_ERROR_TARGET_NOT_FOUND;
        }

        if (npu_arch_map.count(key) > 0)
        {
            sim_code = npu_arch_map[key].sim_code;
        } else {
            LOG(LOG_ERR, "No LIN target: %d\n", config);
            return AIPU_STATUS_ERROR_TARGET_NOT_FOUND;
        }
    }

    if (npu_arch_map[key].config != 1304)
    {
        LOG(LOG_ERR, "Only support: tbd1/tbd2\n");
        return AIPU_STATUS_ERROR_TARGET_NOT_FOUND;
    }

    return AIPU_STATUS_SUCCESS;
}

bool aipudrv::SimulatorV4::has_target(uint32_t arch, uint32_t version, uint32_t config, uint32_t rev)
{
    uint32_t reg_val = 0, sim_code = 0;
    BufferDesc *rev_buf = nullptr;
    bool ret = false;
    char *umd_asid_base = getenv("UMD_ASID_BASE");
    char *umd_part_mode = getenv("UMD_PART_MODE");
    uint64_t umd_asid_base_pa = 0;
    char *ptr = nullptr;

    if ((arch != AIPU_ARCH_ZHOUYI) || (version != AIPU_ISA_VERSION_ZHOUYI_V4) || (rev != 0))
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

    if (sim_code == sim_aipu::config_t::tbd1 || sim_code == sim_aipu::config_t::tbd2)
        m_dram->gm_init(m_config.gm_size);

    if (umd_asid_base != nullptr)
    {
        umd_asid_base_pa = strtoul(umd_asid_base, &ptr, 10);
        if (umd_asid_base_pa > get_umemory()->get_memregion_base(ASID_REGION_0, MEM_REGION_DDR))
        {
            umd_asid_base_pa = get_umemory()->get_memregion_base(ASID_REGION_0, MEM_REGION_DDR);
            LOG(LOG_WARN, "\nreq ASID base > sim DDR base, use DDR base as ASID base: 0x%lx\n",
                umd_asid_base_pa);
        }
    } else {
        umd_asid_base_pa = get_umemory()->get_memregion_base(ASID_REGION_0, MEM_REGION_DDR);
    }

    m_dram->set_asid_base(0, umd_asid_base_pa);
    if (SHARE_ONE_ASID == 1)
        m_dram->set_asid_base(1, umd_asid_base_pa);
    else
        m_dram->set_asid_base(1, get_umemory()->get_memregion_base(ASID_REGION_1, MEM_REGION_DDR));
    m_dram->set_asid_base(2, umd_asid_base_pa);
    m_dram->set_asid_base(3, umd_asid_base_pa);

    /* reserve 4KB for debug */
    m_dram->reserve_mem(0xC1000000, AIPU_PAGE_SIZE, &rev_buf, "rsv");
    m_reserve_mem.push_back(rev_buf);

    m_code = sim_code;
    m_aipu->read_register(TSM_BUILD_INFO, reg_val);
    m_max_cmdpool_cnt = ((reg_val >> 16) & 0xf) + 1;

    if (umd_part_mode != nullptr)
    {
        m_partition_mode = umd_part_mode[0] - '0';
        if (m_partition_mode >= POOL_MAX)
            m_partition_mode = POOL_SCP;
    }

    m_aipu->set_event_handler((sim_aipu::event_handler_t)(SimulatorV4::sim_cb_handler), nullptr);
    parse_cluster_info();
    ret = true;

unlock:
    pthread_rwlock_unlock(&m_lock);
    return ret;
}

bool aipudrv::SimulatorV4::is_cmdpool_full(int qos, int part_id, int partition_mode,
    int cluster_idx, uint32_t reg_val)
{
    uint32_t cmdpool_full_flag = 0;

    if (qos == AIPU_JOB_QOS_SLOW)
    {
        if ((partition_mode == POOL_SCP) && (part_id == 1))
            cmdpool_full_flag = (1 << (cluster_idx + 4)) & reg_val;
        else
            cmdpool_full_flag = (1 << cluster_idx) & reg_val;
    } else {
        if ((partition_mode == POOL_SCP) && (part_id == 1))
            cmdpool_full_flag = (1 << (cluster_idx + 12)) & reg_val;
        else
            cmdpool_full_flag = (1 << (cluster_idx + 8)) & reg_val;
    }

    return !!cmdpool_full_flag;
}

aipu_status_t aipudrv::SimulatorV4::schedule(const JobDesc& jobdesc)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    JobV4 *job = static_cast<JobV4 *>(jobdesc.jobbase);
    uint16_t grid_id = job->get_grid_id();
    uint32_t part_id = job->get_part_id();
    uint32_t cmd_pool_id = 0, value = 0, cluster_idx = 0;
    uint32_t qos = job->get_qos();
    job_queue_elem_t job_queue_item;
    JobDesc job_desc{0};

    if (part_id > POOL_SCP)
        part_id = POOL_PCP;

    if (m_aipu == nullptr)
        return AIPU_STATUS_ERROR_NULL_PTR;

    pthread_rwlock_wrlock(&m_lock);
    if (job->m_bind_cmdpool_id == 0xffffffff)
        cmd_pool_id = job->m_bind_cmdpool_id = get_cmdpool_id(cluster_idx, part_id);
    else
        cmd_pool_id = job->m_bind_cmdpool_id;

    job_queue_item.job = jobdesc.jobbase;
    job_queue_item.jobdesc = jobdesc;
    m_buffer_queue.push(job_queue_item);

    if (!m_cmdpool_busy)
    {
        uint32_t reg_val = 0;

        m_aipu->read_register(TSM_STATUS, reg_val);

        if (!is_cmdpool_full(qos, part_id, m_partition_mode,
            cluster_idx, reg_val))
        {
            m_cmdpool_busy = true;
            job_queue_item = m_buffer_queue.front();
            m_buffer_queue.pop();
            job = (JobV4 *)job_queue_item.job;
            job_desc = job_queue_item.jobdesc;
            m_commit_map[grid_id] = job_desc.jobbase;

            m_aipu->write_register(TSM_CMD_SCHED_ADDR_HI, get_high_32(job_desc.tcb_head));
            m_aipu->write_register(TSM_CMD_SCHED_ADDR_LO, get_low_32(job_desc.tcb_head));
            m_aipu->write_register(TSM_CMD_TCB_NUMBER, job_desc.tcb_number);

            /* specify cmdpool number & QoS */
            value = (part_id << 19) | (cmd_pool_id << 16) | (qos << 8);
            m_aipu->write_register(TSM_CMD_SCHED_CTRL, value | CREATE_CMD_POOL);

            LOG(LOG_INFO, "triggering simulator...%lx", job->get_id());
            m_aipu->write_register(TSM_CMD_SCHED_CTRL, DISPATCH_CMD_POOL);
        } else {
            LOG(LOG_ALERT, "CMD POOL %d, QOS %d [full]", cmd_pool_id, qos);
        }
    }
    pthread_rwlock_unlock(&m_lock);

    return ret;
}

aipu_status_t aipudrv::SimulatorV4::fill_commit_queue()
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    uint32_t max_limit = 1, max = 0;
    job_queue_elem_t job_queue_item {0};

    LOG(LOG_INFO, "Enter %s...", __FUNCTION__);

    if (m_buffer_queue.size() >= max_limit)
        max = max_limit;
    else
        max = m_buffer_queue.size();

    if (m_commit_map.size() >= 16)
        return ret;

    for(uint32_t i = 0; i < max; i++)
    {
        job_queue_item = m_buffer_queue.front();
        JobBase *jobbase = (JobBase *)job_queue_item.job;
        JobDesc jobdesc = job_queue_item.jobdesc;
        JobV4 *job = static_cast<JobV4 *>(jobbase);
        uint16_t grid_id = job->get_grid_id();
        uint32_t part_id = job->get_part_id();
        uint32_t cmd_pool_id = 0, value = 0;
        uint32_t qos = job->get_qos();
        uint32_t reg_val = 0;
        uint32_t cluster_idx = 0;

        if (part_id > POOL_SCP)
            part_id = POOL_PCP;

        if (m_aipu == nullptr)
            return AIPU_STATUS_ERROR_NULL_PTR;

        if (job->m_bind_cmdpool_id == 0xffffffff)
            cmd_pool_id = job->m_bind_cmdpool_id = get_cmdpool_id(0, part_id);
        else
            cmd_pool_id = job->m_bind_cmdpool_id;

        m_aipu->read_register(TSM_STATUS, reg_val);

        if (!is_cmdpool_full(qos, part_id, m_partition_mode,
            cluster_idx, reg_val))
        {
            m_aipu->write_register(TSM_CMD_SCHED_ADDR_HI, get_high_32(jobdesc.tcb_head));
            m_aipu->write_register(TSM_CMD_SCHED_ADDR_LO, get_low_32(jobdesc.tcb_head));
            m_aipu->write_register(TSM_CMD_TCB_NUMBER, get_low_32(jobdesc.tcb_number));

            /* specify cmdpool number & QoS */
            value = (part_id << 19) | (cmd_pool_id << 16) | (qos << 8);
            // m_aipu->write_register(TSM_CMD_SCHED_CTRL, value | CREATE_CMD_POOL);

            LOG(LOG_INFO, "triggering simulator...%lx", job->get_id());
            m_aipu->write_register(TSM_CMD_SCHED_CTRL, value | DISPATCH_CMD_POOL);

            m_buffer_queue.pop();
            m_commit_map[grid_id] = jobbase;
        } else {
            LOG(LOG_ALERT, "CMD POOL %d, QOS %d [full]", cmd_pool_id, qos);
            break;
        }
    }

    LOG(LOG_INFO, "Exit %s...", __FUNCTION__);
    return ret;
}

aipu_ll_status_t aipudrv::SimulatorV4::poll_status(uint32_t max_cnt, int32_t time_out,
    bool of_this_thread, void *jobbase)
{
    JobV4 *job = static_cast<JobV4 *>(jobbase);
    uint16_t grid_id = job->get_grid_id();

    LOG(LOG_INFO, "Enter %s...", __FUNCTION__);

    if (job->get_subgraph_cnt() == 0)
    {
        job->update_job_status(AIPU_JOB_STATE_DONE);
        return AIPU_LL_STATUS_SUCCESS;
    }

    while (1)
    {
        pthread_rwlock_wrlock(&m_lock);
        if (m_done_set.count(jobbase))
        {
            m_done_set.erase(jobbase);
            job->update_job_status(AIPU_JOB_STATE_DONE);
            pthread_rwlock_unlock(&m_lock);
            break;
        }
        pthread_rwlock_unlock(&m_lock);

        m_poll_mtex.lock();
        if (m_commit_map.count(grid_id))
        {
            if (m_sim_done_grid_set.count(grid_id) == 0)
            {
                LOG(LOG_INFO, "wait, sim doing...\n");
                std::unique_lock<std::mutex> lck(simv4_mtx);
                simv4_cv.wait(lck, has_some_grid_done);
                simv4_has_grid_done = false;
                LOG(LOG_INFO, "wakeup, sim done...\n");
            }

            pthread_rwlock_wrlock(&m_lock);
            std::vector <uint16_t> sim_done_grid_vec;
            for (const auto done_gridid : m_sim_done_grid_set)
            {
                if (m_commit_map.count(done_gridid))
                {
                    m_done_set.insert(m_commit_map[done_gridid]);
                    m_commit_map.erase(done_gridid);
                    sim_done_grid_vec.push_back(done_gridid);
                }
            }

            // clear done grid record from sim_done_grid set
            m_sim_done_grid_mtx.lock();
            for (auto sim_done_grid_id : sim_done_grid_vec)
                m_sim_done_grid_set.erase(sim_done_grid_id);
            m_sim_done_grid_mtx.unlock();
            LOG(LOG_INFO, "batch job done...\n");

            if (m_buffer_queue.size() > 0)
            {
                fill_commit_queue();

                /**
                 * dump a combination runtime.cfg for all jobs in one running period,
                 * it doesn't allow to be used in multiple threads scenario.
                 */
                // job->dumpcfg_alljob();
            }
            pthread_rwlock_unlock(&m_lock);
        }
        m_poll_mtex.unlock();
    }
    LOG(LOG_INFO, "Exit %s...", __FUNCTION__);

    return AIPU_LL_STATUS_SUCCESS;
}

void aipudrv::SimulatorV4::sim_cb_handler(uint32_t event, uint64_t value, void *context)
{
    LOG(LOG_INFO, "Enter sim_cb_handler...\n");

    if (event == sim_aipu::AIPU_EV_GRID_END)
    {
        m_sim_done_grid_mtx.lock();
        m_sim_done_grid_set.insert(value);
        std::unique_lock<std::mutex> lck(simv4_mtx);
        simv4_has_grid_done = true;
        simv4_cv.notify_one();
        m_sim_done_grid_mtx.unlock();
    } else {
        LOG(LOG_ALERT, "sim_cn_handler has no event: %d\n", event);
    }
}
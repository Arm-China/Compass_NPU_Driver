// Copyright (C) 2023-2025 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

/**
 * @file  simulator_v3.cpp
 * @brief AIPU User Mode Driver (UMD) zhouyi aipu v3 simulator module
 * implementation
 */

#include "simulator_v3.h"

#include <unistd.h>

#include <cstring>

#include "utils/helper.h"

namespace aipudrv {
SimulatorV3::SimulatorV3() {
  m_dev_type = DEV_TYPE_SIMULATOR_V3;
  m_hw_config = 1204;
  m_dram = UMemory::get_memory();
  m_dram->set_dev(this);
  m_dram->set_isa_version(AIPU_ISA_VERSION_ZHOUYI_V3);

  m_log_level = RTDEBUG_SIMULATOR_LOG_LEVEL;
  m_verbose = false;
  m_enable_avx = false;
  m_en_eval = false;
  m_gm_size = 4 * MB_SIZE;

  pthread_rwlock_init(&m_lock, NULL);
}

SimulatorV3::~SimulatorV3() {
  if (m_aipu != nullptr) {
    cmd_pool_destroy();

    delete m_aipu;
    m_aipu = nullptr;
  }

  pthread_rwlock_destroy(&m_lock);
  m_dram = nullptr;
}

void SimulatorV3::set_cfg(const aipu_global_config_simulation_t *cfg) {
  if (cfg == nullptr)
    return;

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

aipu_status_t SimulatorV3::parse_config(uint32_t config, uint32_t &sim_code) {
  std::string key;
  if (m_arch_desc.empty()) {
    for (auto &it : m_npu_arch_map) {
      if (it.second.config == config) {
        key = it.first;
        break;
      }
    }

    if (!key.empty()) {
      LOG(LOG_ALERT, "Not provide npu arch, and config set to %u", config);
    } else {
      LOG(LOG_ERR,
          "Not provide npu arch, and config: %u does not support in v3",
          config);
      return AIPU_STATUS_ERROR_TARGET_NOT_FOUND;
    }
  } else if (m_npu_arch_map.count(m_arch_desc) > 0) {
    key = m_arch_desc;
  } else {
    LOG(LOG_ERR, "%s does not support in v3", m_arch_desc.c_str());
    return AIPU_STATUS_ERROR_TARGET_NOT_FOUND;
  }

  sim_code = m_npu_arch_map.at(key).sim_code;
  return AIPU_STATUS_SUCCESS;
}

bool SimulatorV3::has_target(uint32_t arch, uint32_t version, uint32_t config,
                             uint32_t rev) {
  return arch == AIPU_ARCH_ZHOUYI && version == AIPU_ISA_VERSION_ZHOUYI_V3 &&
         rev == 0;
}

aipu_status_t SimulatorV3::init() {
  aipu_partition_cap aipu_cap = {0};
  uint32_t reg_val = 0, sim_code = 0;
  BufferDesc *rev_buf = nullptr;
  aipu_status_t ret = AIPU_STATUS_SUCCESS;

  pthread_rwlock_wrlock(&m_lock);
  if (m_aipu != nullptr)
    goto unlock;

  ret = parse_config(m_hw_config, sim_code);
  if (ret != AIPU_STATUS_SUCCESS)
    goto unlock;

  m_config = sim_create_config(sim_code, m_log_level, m_log_filepath, m_verbose,
                               m_enable_avx, m_en_eval, m_gm_size,
                               m_plugin_filename, m_json_filename);
  m_aipu = new sim_aipu::Aipu(m_config, static_cast<UMemory &>(*m_dram));
  if (m_aipu == nullptr) {
    ret = AIPU_STATUS_ERROR_DEV_ABNORMAL;
    goto unlock;
  }

  m_dram->gm_init(m_config.gm_size);

  /* reserve 4KB for debug */
  m_dram->reserve_mem(0xC1000000, AIPU_PAGE_SIZE, &rev_buf, "rsv",
                      MEM_REGION_RSV);
  m_reserve_mem.push_back(rev_buf);

  m_code = sim_code;
  m_aipu->read_register(TSM_BUILD_INFO, reg_val);
  m_max_partition_cnt = ((reg_val >> 24) & 0xf) + 1;
  m_max_cmdpool_cnt = (reg_val >> 16) & 0xf;

  if (m_max_partition_cnt > MAX_PART_CNT) {
    LOG(LOG_ERR, "Invalid config: max_part %d", m_max_partition_cnt);
    ret = AIPU_STATUS_ERROR_DEV_ABNORMAL;
    goto unlock;
  }

  if (set_cluster_to_partition() != AIPU_STATUS_SUCCESS) {
    ret = AIPU_STATUS_ERROR_DEV_ABNORMAL;
    goto unlock;
  }

  /* assign cmdpool id to one partition */
  if (set_cmdpool_to_partition(m_partition_cnt, m_max_cmdpool_cnt) !=
      AIPU_STATUS_SUCCESS) {
    ret = AIPU_STATUS_ERROR_DEV_ABNORMAL;
    goto unlock;
  }

  aipu_cap.arch = AIPU_ARCH_ZHOUYI;
  aipu_cap.version = AIPU_ISA_VERSION_ZHOUYI_V3;
  aipu_cap.config = m_hw_config;
  m_part_caps.push_back(aipu_cap);

unlock:
  pthread_rwlock_unlock(&m_lock);
  return ret;
}

#if 0
aipu_status_t SimulatorV3::schedule(const JobDesc& jobdesc)
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

    if (m_aipu == nullptr)
        return AIPU_STATUS_ERROR_NULL_PTR;

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
#else
aipu_status_t SimulatorV3::schedule(const JobDesc &jobdesc) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  JobV3 *job = static_cast<JobV3 *>(jobdesc.jobbase);
  uint32_t part_id = job->get_part_id();
  uint32_t cmd_pool_id = 0, value = 0;
  uint32_t qos = job->get_qos();
  job_queue_elem_t job_queue_item;
  JobDesc job_desc{0};

  if (part_id > m_partition_cnt) {
    ret = AIPU_STATUS_ERROR_INVALID_PARTITION_ID;
    goto out;
  }

  if (m_aipu == nullptr)
    return AIPU_STATUS_ERROR_NULL_PTR;

  pthread_rwlock_wrlock(&m_lock);
  if (job->m_bind_cmdpool_id == 0xffffffff)
    cmd_pool_id = job->m_bind_cmdpool_id = get_cmdpool_id(part_id);
  else
    cmd_pool_id = job->m_bind_cmdpool_id;

  job_queue_item.job = jobdesc.jobbase;
  job_queue_item.jobdesc = jobdesc;
  m_buffer_queue.push(job_queue_item);

  if (!m_cmdpool_busy) {
    m_cmdpool_busy = true;
    job_queue_item = m_buffer_queue.front();
    m_buffer_queue.pop();
    job = (JobV3 *)job_queue_item.job;
    job_desc = job_queue_item.jobdesc;
    m_commit_queue.insert(job_desc.jobbase);

    if (!cmd_pool_created(cmd_pool_id)) {
      cmd_pool_add_job(cmd_pool_id, job, job_desc);
      m_aipu->write_register(TSM_CMD_SCHED_ADDR_HI,
                             get_high_32(job_desc.tcb_head));
      m_aipu->write_register(TSM_CMD_SCHED_ADDR_LO,
                             get_low_32(job_desc.tcb_head));

      /* specify cmdpool number & QoS */
      value = (cmd_pool_id << 16) | (qos << 8);
      m_aipu->write_register(TSM_CMD_SCHED_CTRL, value | CREATE_CMD_POOL);

      /* bind cmdpool to a partition */
      m_aipu->read_register(TSM_CMD_POOL0_CONFIG + cmd_pool_id * 0x40, value);
      value &= ~0xf;
      value |= part_id & 0xf;
      m_aipu->write_register(TSM_CMD_POOL0_CONFIG + cmd_pool_id * 0x40, value);

      LOG(LOG_INFO, "triggering simulator...%lx", job->get_id());
      m_aipu->write_register(TSM_CMD_SCHED_CTRL, DISPATCH_CMD_POOL);
    } else {
      cmd_pool_append_job(cmd_pool_id, job, job_desc);
      LOG(LOG_INFO, "append job...%lx", job->get_id());
    }
  }
  pthread_rwlock_unlock(&m_lock);

out:
  return ret;
}

aipu_status_t SimulatorV3::fill_commit_queue() {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  uint32_t max_limit = 3, max = 0;
  job_queue_elem_t job_queue_item{0};

  LOG(LOG_INFO, "Enter %s...", __FUNCTION__);

  if (m_buffer_queue.size() >= max_limit)
    max = max_limit;
  else
    max = m_buffer_queue.size();

  for (uint32_t i = 0; i < max; i++) {
    job_queue_item = m_buffer_queue.front();
    m_buffer_queue.pop();
    JobBase *jobbase = (JobBase *)job_queue_item.job;
    JobDesc jobdesc = job_queue_item.jobdesc;
    JobV3 *job = static_cast<JobV3 *>(jobbase);
    uint32_t part_id = job->get_part_id();
    uint32_t cmd_pool_id = 0, value = 0;
    uint32_t qos = job->get_qos();

    if (part_id > m_partition_cnt) {
      ret = AIPU_STATUS_ERROR_INVALID_PARTITION_ID;
      goto out;
    }

    if (m_aipu == nullptr)
      return AIPU_STATUS_ERROR_NULL_PTR;

    if (job->m_bind_cmdpool_id == 0xffffffff)
      cmd_pool_id = job->m_bind_cmdpool_id = get_cmdpool_id(part_id);
    else
      cmd_pool_id = job->m_bind_cmdpool_id;

    m_commit_queue.insert(jobbase);

    if (!cmd_pool_created(cmd_pool_id)) {
      m_cmdpool_busy = true;
      cmd_pool_add_job(cmd_pool_id, job, jobdesc);
      m_aipu->write_register(TSM_CMD_SCHED_ADDR_HI,
                             get_high_32(jobdesc.tcb_head));
      m_aipu->write_register(TSM_CMD_SCHED_ADDR_LO,
                             get_low_32(jobdesc.tcb_head));

      /* specify cmdpool number & QoS */
      value = (cmd_pool_id << 16) | (qos << 8);
      m_aipu->write_register(TSM_CMD_SCHED_CTRL, value | CREATE_CMD_POOL);

      /* bind cmdpool to a partition */
      m_aipu->read_register(TSM_CMD_POOL0_CONFIG + cmd_pool_id * 0x40, value);
      value &= ~0xf;
      value |= part_id & 0xf;
      m_aipu->write_register(TSM_CMD_POOL0_CONFIG + cmd_pool_id * 0x40, value);

      LOG(LOG_INFO, "triggering simulator...%lx", job->get_id());
      m_aipu->write_register(TSM_CMD_SCHED_CTRL, DISPATCH_CMD_POOL);
    } else {
      cmd_pool_append_job(cmd_pool_id, job, jobdesc);
      LOG(LOG_INFO, "append job...%lx", job->get_id());
    }
  }

out:
  LOG(LOG_INFO, "Exit %s...", __FUNCTION__);
  return ret;
}
#endif

aipu_ll_status_t SimulatorV3::poll_status(uint32_t max_cnt, int32_t time_out,
                                          bool of_this_thread, void *jobbase) {
  uint32_t value = 0;
  JobV3 *job = static_cast<JobV3 *>(jobbase);
  uint32_t cmd_pool_id = job->m_bind_cmdpool_id;
  uint32_t cmd_pool_status_reg = CMD_POOL0_STATUS + 0x40 * cmd_pool_id;

  LOG(LOG_INFO, "Enter %s...", __FUNCTION__);

  if (job->get_subgraph_cnt() == 0) {
    job->update_job_status(AIPU_JOB_STATE_DONE);
    return AIPU_LL_STATUS_SUCCESS;
  }

  while (1) {
    pthread_rwlock_wrlock(&m_lock);
    if (m_done_queue.count(jobbase)) {
      cmd_pool_erase_job(cmd_pool_id, job);
      m_done_queue.erase(jobbase);
      job->update_job_status(AIPU_JOB_STATE_DONE);
      pthread_rwlock_unlock(&m_lock);
      break;
    }
    pthread_rwlock_unlock(&m_lock);

    m_poll_mtex.lock();
    if (m_commit_queue.count(jobbase)) {
      while (m_aipu->read_register(cmd_pool_status_reg, value) > 0) {
        LOG(LOG_INFO, "wait for simulation execution, cmdpool sts=%x", value);
        if (value & CMD_POOL0_IDLE) {
          m_aipu->write_register(TSM_CMD_SCHED_CTRL, DESTROY_CMD_POOL);
          LOG(LOG_INFO, "simulation done.");
          break;
        }
        sleep(1);
      }

      pthread_rwlock_wrlock(&m_lock);
      for (auto iter = m_commit_queue.begin(); iter != m_commit_queue.end();
           iter++) {
        m_done_queue.insert(*iter);
      }
      m_commit_queue.clear();
      cmd_pool_destroy();
      m_cmdpool_busy = false;
      LOG(LOG_INFO, "cmd_pool_destroy...");

      if (m_buffer_queue.size() > 0) {
        fill_commit_queue();

        /**
         * dump a combination runtime.cfg for all jobs in one running period,
         * it doesn't allow to be used in multiple threads scenario.
         */
        job->dumpcfg_alljob();
      }
      pthread_rwlock_unlock(&m_lock);
    }
    m_poll_mtex.unlock();
  }
  LOG(LOG_INFO, "Exit %s...", __FUNCTION__);

  return AIPU_LL_STATUS_SUCCESS;
}
} // namespace aipudrv

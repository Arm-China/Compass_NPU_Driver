// Copyright (C) 2023-2025 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

/**
 * @file  simulator_v3_2.cpp
 * @brief AIPU User Mode Driver (UMD) zhouyi aipu v3_2 simulator module
 * implementation
 */

#include "simulator_v3_2.h"

#include <unistd.h>

#include <condition_variable>
#include <cstring>
#include <mutex>
#include <thread>

#include "utils/helper.h"
#include "zhouyi_v3x/zhouyi_v3_2/job_v3_2.h"

namespace aipudrv {
/**
 * The bellow are used for syncing between UMD and Simulator
 * when some grid job is done. Simulator will directly call
 * one callback to notify UMD and convey the done grid's ID to UMD.
 */
std::set<uint16_t> SimulatorV3_2::m_sim_done_grid_set = {};
std::mutex SimulatorV3_2::m_sim_done_grid_mtx;
std::mutex simv3_2_mtx;
std::condition_variable simv3_2_cv;
bool simv3_2_has_grid_done = false;

bool has_some_grid_done() { return simv3_2_has_grid_done; }

SimulatorV3_2::SimulatorV3_2() {
  m_dev_type = DEV_TYPE_SIMULATOR_V3_2;
  m_hw_config = 1304;
  m_dram = UMemory::get_memory();
  m_dram->set_dev(this);
  m_dram->set_isa_version(AIPU_ISA_VERSION_ZHOUYI_V3_2);

  pthread_rwlock_init(&m_lock, NULL);
}

SimulatorV3_2::~SimulatorV3_2() {
  if (m_aipu != nullptr) {
    delete m_aipu;
    m_aipu = nullptr;
  }

  pthread_rwlock_destroy(&m_lock);
  m_dram = nullptr;
}

void SimulatorV3_2::set_cfg(const aipu_global_config_simulation_t *cfg) {
  if (cfg == nullptr)
    return;

  m_log_level = cfg->log_level;
  m_verbose = cfg->verbose;
  m_enable_avx = cfg->enable_avx;
  m_gm_size = cfg->gm_size;

  if (cfg->plugin_name != nullptr)
    m_plugin_filename = cfg->plugin_name;
  if (cfg->json_filename != nullptr)
    m_json_filename = cfg->json_filename;
  if (cfg->log_file_path != nullptr)
    m_log_filepath = cfg->log_file_path;
  if (cfg->npu_arch_desc != nullptr)
    m_arch_desc = cfg->npu_arch_desc;

  m_en_fast_perf = cfg->en_fast_perf;
  m_freq_mhz = cfg->freq_mhz;
  m_ddr_latency_rd = cfg->ddr_latency_rd;
  m_ddr_latency_wr = cfg->ddr_latency_wr;
  m_ddr_bw = cfg->ddr_bw;

  if (cfg->perf_report != nullptr)
    m_perf_report = cfg->perf_report;
}

int SimulatorV3_2::get_grid_id(uint16_t &grid_id) {
  grid_id = m_grid_id++;
  return 0;
}

int SimulatorV3_2::get_start_group_id(int group_cnt, uint16_t &start_group_id) {
  int ret = 0;

  if (group_cnt == 0)
    return ret;

  m_group_id_mtx.lock();
  for (uint32_t i = 0; i < MAX_GROUP_ID; i++) {
    if (m_group_id_bitmap[i] == false) {
      uint32_t j = 0;
      if (i + group_cnt >= MAX_GROUP_ID) {
        ret = -1;
        LOG(LOG_ERR, "Group ID bit map overflow");
        goto out;
      }

      for (j = i; j < i + group_cnt; j++) {
        if (m_group_id_bitmap[j] == true) {
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

/* if dev->init() is behind graph parser, we can use config to check some
 * parameters */
aipu_status_t SimulatorV3_2::parse_config(uint32_t config, uint32_t &sim_code) {
  std::string target = m_arch_desc;
  if (target.empty()) {
    for (auto &it : m_npu_arch_map) {
      if (it.second.config == config) {
        target = it.first;
        break;
      }
    }

    if (target.empty()) {
      LOG(LOG_ERR,
          "Not provide npu arch, and config: %u does not support in v2",
          config);
      return AIPU_STATUS_ERROR_TARGET_NOT_FOUND;
    }
  }

  uint32_t core_num = 1;
  size_t pos = 0;
  std::string bare_target = target;
  if ((pos = target.find("MP", pos)) != std::string::npos) {
    auto mp = target.substr(pos + 2, target.size() - pos - 2);
    core_num = std::stoi(mp);
    if (core_num == 0)
      core_num = 1;
    if (pos > 0 && target[pos - 1] == '_')
      pos = pos - 1;
    bare_target = target.substr(0, pos);
  }

  if (m_npu_arch_map.count(bare_target) != 0) {
    auto &item = m_npu_arch_map.at(bare_target);
    sim_code = (item.tec_num & 0xF) + ((item.aiff_num & 0xF) << 4) +
               ((core_num & 0xF) << 8) + ((item.cluster_num & 0xF) << 12) +
               (1 << 31);
  } else {
    LOG(LOG_ERR, "%s does not support in v3_2", m_arch_desc.c_str());
    return AIPU_STATUS_ERROR_TARGET_NOT_FOUND;
  }

  return AIPU_STATUS_SUCCESS;
}

bool SimulatorV3_2::has_target(uint32_t arch, uint32_t version, uint32_t config,
                               uint32_t rev) {
  return arch == AIPU_ARCH_ZHOUYI && version == AIPU_ISA_VERSION_ZHOUYI_V3_2;
}

aipu_status_t SimulatorV3_2::init() {
  uint32_t reg_val = 0, sim_code = 0;
  BufferDesc *rev_buf = nullptr;
  aipu_status_t ret = AIPU_STATUS_SUCCESS;

  pthread_rwlock_wrlock(&m_lock);
  if (m_aipu != nullptr) {
    goto unlock;
  }

  ret = parse_config(m_hw_config, sim_code);
  if (ret != AIPU_STATUS_SUCCESS)
    goto unlock;

  sim_create_config(sim_code, m_config);
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
  m_max_cmdpool_cnt = ((reg_val >> 16) & 0xf) + 1;

  m_aipu->set_event_handler((event_handler_t)(SimulatorV3_2::sim_cb_handler),
                            nullptr);
  parse_cluster_info();

unlock:
  pthread_rwlock_unlock(&m_lock);
  return ret;
}

bool SimulatorV3_2::is_cmdpool_full(int qos, int part_id, int partition_mode,
                                    int cluster_idx, uint32_t reg_val) {
  uint32_t cmdpool_full_flag = 0;

  if (qos == AIPU_JOB_QOS_SLOW) {
    cmdpool_full_flag = (1 << cluster_idx) & reg_val;
  } else {
    cmdpool_full_flag = (1 << (cluster_idx + 8)) & reg_val;
  }

  return !!cmdpool_full_flag;
}

aipu_status_t SimulatorV3_2::schedule(const JobDesc &jobdesc) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  JobV3_2 *job = static_cast<JobV3_2 *>(jobdesc.jobbase);
  uint16_t grid_id = job->get_grid_id();
  uint32_t part_id = job->get_part_id();
  uint32_t cmd_pool_id = 0, value = 0, cluster_idx = 0;
  uint32_t qos = job->get_qos();
  job_queue_elem_t job_queue_item;
  JobDesc job_desc{0};

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

  if (!m_cant_add_job_flag) {
    uint32_t reg_val = 0;

    m_aipu->read_register(TSM_STATUS, reg_val);

    if (!is_cmdpool_full(qos, part_id, m_partition_mode, cluster_idx,
                         reg_val)) {
      m_cant_add_job_flag = true;
      job_queue_item = m_buffer_queue.front();
      m_buffer_queue.pop();
      job = (JobV3_2 *)job_queue_item.job;
      job_desc = job_queue_item.jobdesc;
      m_commit_map[grid_id] = job_desc.jobbase;

      m_aipu->write_register(TSM_CMD_SCHED_ADDR_HI,
                             get_high_32(job_desc.tcb_head));
      m_aipu->write_register(TSM_CMD_SCHED_ADDR_LO,
                             get_low_32(job_desc.tcb_head));
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

aipu_status_t SimulatorV3_2::fill_commit_queue() {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  uint32_t max_limit = 1, max = 0;
  job_queue_elem_t job_queue_item{0};

  LOG(LOG_INFO, "Enter %s...", __FUNCTION__);

  if (m_buffer_queue.size() >= max_limit)
    max = max_limit;
  else
    max = m_buffer_queue.size();

  if (m_commit_map.size() >= 16)
    return ret;

  for (uint32_t i = 0; i < max; i++) {
    job_queue_item = m_buffer_queue.front();
    JobBase *jobbase = (JobBase *)job_queue_item.job;
    JobDesc jobdesc = job_queue_item.jobdesc;
    JobV3_2 *job = static_cast<JobV3_2 *>(jobbase);
    uint16_t grid_id = job->get_grid_id();
    uint32_t part_id = job->get_part_id();
    uint32_t cmd_pool_id = 0, value = 0;
    uint32_t qos = job->get_qos();
    uint32_t reg_val = 0;
    uint32_t cluster_idx = 0;

    if (m_aipu == nullptr)
      return AIPU_STATUS_ERROR_NULL_PTR;

    if (job->m_bind_cmdpool_id == 0xffffffff)
      cmd_pool_id = job->m_bind_cmdpool_id = get_cmdpool_id(0, part_id);
    else
      cmd_pool_id = job->m_bind_cmdpool_id;

    m_aipu->read_register(TSM_STATUS, reg_val);

    if (!is_cmdpool_full(qos, part_id, m_partition_mode, cluster_idx,
                         reg_val)) {
      m_aipu->write_register(TSM_CMD_SCHED_ADDR_HI,
                             get_high_32(jobdesc.tcb_head));
      m_aipu->write_register(TSM_CMD_SCHED_ADDR_LO,
                             get_low_32(jobdesc.tcb_head));
      m_aipu->write_register(TSM_CMD_TCB_NUMBER,
                             get_low_32(jobdesc.tcb_number));

      /* specify cmdpool number & QoS */
      value = (part_id << 19) | (cmd_pool_id << 16) | (qos << 8);
      // m_aipu->write_register(TSM_CMD_SCHED_CTRL, value | CREATE_CMD_POOL);

      LOG(LOG_INFO, "triggering simulator...%lx", job->get_id());
      m_aipu->write_register(TSM_CMD_SCHED_CTRL, value | DISPATCH_CMD_POOL);

      m_buffer_queue.pop();
      m_commit_map[grid_id] = jobbase;
      m_cant_add_job_flag = true;
    } else {
      LOG(LOG_ALERT, "CMD POOL %d, QOS %d [full]", cmd_pool_id, qos);
      break;
    }
  }

  LOG(LOG_INFO, "Exit %s...", __FUNCTION__);
  return ret;
}

aipu_ll_status_t SimulatorV3_2::poll_status(uint32_t max_cnt, int32_t time_out,
                                            bool of_this_thread,
                                            void *jobbase) {
  JobV3_2 *job = static_cast<JobV3_2 *>(jobbase);
  uint16_t grid_id = job->get_grid_id();

  LOG(LOG_INFO, "Enter %s...", __FUNCTION__);

  if (job->get_subgraph_cnt() == 0) {
    job->update_job_status(AIPU_JOB_STATE_DONE);
    return AIPU_LL_STATUS_SUCCESS;
  }

  while (1) {
    pthread_rwlock_wrlock(&m_lock);
    if (m_done_set.count(jobbase)) {
      m_done_set.erase(jobbase);
      job->update_job_status(AIPU_JOB_STATE_DONE);
      pthread_rwlock_unlock(&m_lock);
      break;
    }
    pthread_rwlock_unlock(&m_lock);

    m_poll_mtex.lock();
    if (m_commit_map.count(grid_id)) {
      if (m_sim_done_grid_set.count(grid_id) == 0) {
        LOG(LOG_INFO, "wait, sim doing...");
        std::unique_lock<std::mutex> lck(simv3_2_mtx);
        simv3_2_cv.wait(lck, has_some_grid_done);
        simv3_2_has_grid_done = false;
        LOG(LOG_INFO, "wakeup, sim done...");
      }

      pthread_rwlock_wrlock(&m_lock);
      std::vector<uint16_t> sim_done_grid_vec;
      for (const auto done_gridid : m_sim_done_grid_set) {
        if (m_commit_map.count(done_gridid)) {
          m_done_set.insert(m_commit_map[done_gridid]);
          m_commit_map.erase(done_gridid);
          m_cant_add_job_flag = false;
          sim_done_grid_vec.push_back(done_gridid);
        }
      }

      // clear done grid record from sim_done_grid set
      m_sim_done_grid_mtx.lock();
      for (auto sim_done_grid_id : sim_done_grid_vec)
        m_sim_done_grid_set.erase(sim_done_grid_id);
      m_sim_done_grid_mtx.unlock();
      LOG(LOG_INFO, "batch job done...");

      if (m_buffer_queue.size() > 0) {
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

void SimulatorV3_2::sim_cb_handler(uint32_t event, uint64_t value,
                                   void *context) {
  LOG(LOG_INFO, "Enter sim_cb_handler...");

  if (event == AIPU_EV_GRID_END) {
    m_sim_done_grid_mtx.lock();
    m_sim_done_grid_set.insert(value);
    std::unique_lock<std::mutex> lck(simv3_2_mtx);
    simv3_2_has_grid_done = true;
    simv3_2_cv.notify_one();
    m_sim_done_grid_mtx.unlock();
  } else {
    LOG(LOG_ALERT, "sim_cn_handler has no event: %d", event);
  }
}
} // namespace aipudrv

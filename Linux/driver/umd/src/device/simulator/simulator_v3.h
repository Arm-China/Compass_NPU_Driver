// Copyright (C) 2023-2025 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

/**
 * @file  simulator_v3.h
 * @brief AIPU User Mode Driver (UMD) zhouyi aipu v3 simulator module header
 */

#ifndef _SIMULATOR_V3_H_
#define _SIMULATOR_V3_H_

#include <map>
#include <mutex>
#include <queue>
#include <set>
#include <sstream>

#include "device/simulator/umemory.h"
#include "device_base.h"
#include "kmd/tcb.h"
#include "standard_api.h"
#include "utils/debug.h"
#include "wrapper.h"
#include "zhouyi_v3x/zhouyi_v3/job_v3.h"

namespace aipudrv {
class CmdPool {
private:
  MemoryBase *m_dram = nullptr;
  DEV_PA_64 m_tcb_head = 0;
  DEV_PA_64 m_tcb_tail = 0;
  bool set_destroy_done = false;

public:
  void update_tcb(DEV_PA_64 head, DEV_PA_64 tail) {
    tcb_v3::tcb_t prev;
    m_dram->read(m_tcb_tail, &prev, sizeof(prev));
    prev.next = get_low_32(head);
    m_dram->write(m_tcb_tail, &prev, sizeof(prev));
    m_tcb_tail = tail;
  }

  void set_destroy_flag() {
    if (!set_destroy_done) {
      tcb_v3::tcb_t prev;
      m_dram->read(m_tcb_tail, &prev, sizeof(prev));
      prev.flag |= tcb_ctl::FLAG_END_TYPE_END_WITH_DESTROY;
      m_dram->write(m_tcb_tail, &prev, sizeof(prev));
      set_destroy_done = !set_destroy_done;
    }
  }

  bool destroy_done() { return set_destroy_done; }

  void reset_destroy_done() { set_destroy_done = false; }

public:
  CmdPool() = default;
  CmdPool(MemoryBase *mem, DEV_PA_64 head, DEV_PA_64 tail) {
    m_dram = mem;
    m_tcb_head = head;
    m_tcb_tail = tail;
  }
};

/* DONOT use simulator in multiple thread, because many objects are lack of
 * protection */
class SimulatorV3 : public DeviceBase {
private:
  static constexpr uint32_t MAX_PART_CNT = 1;
  static constexpr uint32_t MAX_CLUSTER_CNT = 1;

  bool m_initialized = false;
  std::string m_target;
  const aipu_global_config_simulation_t *m_cfg =
      nullptr; /* keep entity valid */
  std::unique_ptr<SimWrapper> m_wrapper = nullptr;

  pthread_rwlock_t m_lock;
  std::mutex m_poll_mtex;

  std::vector<uint32_t> m_cluster_in_part[MAX_PART_CNT];
  uint32_t m_max_partition_cnt = 0;
  uint32_t m_max_cmdpool_cnt = 0;
  std::vector<BufferDesc *> m_reserve_mem;

  /**
   * @cmdpool_id: the next cmdpool index in one partition
   * @m_cmdpool_in_part: the cmdpool number belong to one partition
   */
  struct CmdpoolInfo {
    uint32_t cmdpool_id;
    std::vector<uint32_t> m_cmdpool_in_part;
  };
  std::map<uint32_t, CmdpoolInfo> m_part_cmdpool;

  /**
   * each cmdpool has a job queue for caching committed Jobs.
   * the corresponding Job is removed after its status is polled.
   */
  std::map<uint32_t, std::map<JobBase *, JobDesc>> m_commit_jobs;

  /**
   * the cmdpool abstraction for managing TCB chains. each cmdpool has an object
   * in this abstraction.
   */
  std::map<uint32_t, CmdPool *> m_cmdpools;

  /**
   * indicate which cmdpool has tcbchain
   * 1: exist tcbchain
   * 0: non-exist tcbchain
   */
  uint32_t m_cmdpool_bitmap = 0;

  /* 1. buffer all jobs in this queue */
  typedef struct {
    void *job;
    JobDesc jobdesc;
  } job_queue_elem_t;
  std::queue<job_queue_elem_t> m_buffer_queue;

  /* 2. move jobs from buffer queue to this queue */
  std::set<void *> m_commit_queue;

  /* 3. move jobs from commit queue to this queue when cmdpool done ready */
  std::set<void *> m_done_queue;

  volatile bool m_cmdpool_busy = false;

private:
  SimulatorV3();

  aipu_ll_status_t fill_commit_queue();
  bool cmd_pool_created(uint32_t cmdpool_id) {
    return m_commit_jobs.count(cmdpool_id) == 1;
  }

  /**
   * init a cmdpool and add the first TCB chain
   */
  void cmd_pool_add_job(uint32_t cmdpool_id, JobBase *job,
                        const JobDesc &job_desc) {
    m_commit_jobs[cmdpool_id][job] = job_desc;
    m_cmdpools[cmdpool_id] =
        new CmdPool(m_dram, job_desc.tcb_head, job_desc.tcb_tail);
    m_cmdpool_bitmap |= (1 << cmdpool_id);
  }

  /**
   * append a TCB chain with a cmdpool
   */
  void cmd_pool_append_job(uint32_t cmdpool_id, JobBase *job,
                           const JobDesc &job_desc) {
    uint32_t value = 0;
    uint32_t qos = static_cast<JobV3 *>(job)->get_qos();

    if (m_cmdpools[cmdpool_id] == nullptr) {
      LOG(LOG_ERR, "CMD POOL %u is null", cmdpool_id);
      return;
    }

    m_commit_jobs[cmdpool_id][job] = job_desc;
    m_cmdpools[cmdpool_id]->update_tcb(job_desc.tcb_head, job_desc.tcb_tail);

    /* specify cmdpool number & QoS */
    m_wrapper->read_register(reg_addr::TSM_CMD_SCHED_CTRL, value);
    value &= ~((0xf << 16) | (0x3 << 8));
    value |= (cmdpool_id << 16) | (qos << 8) | reg_ctl::DISPATCH_CMD_POOL;
    m_wrapper->write_register(reg_addr::TSM_CMD_SCHED_CTRL, value);
  }

  /**
   * remove one Job and its TCB chain
   */
  void cmd_pool_erase_job(uint32_t cmdpool_id, JobBase *job) {
    if (m_commit_jobs.count(cmdpool_id) > 0) {
      if (m_commit_jobs[cmdpool_id].count(job) > 0) {
        /* erase one done job from emulated command pool */
        m_commit_jobs[cmdpool_id].erase(job);

        /* cleanup the command queue */
        if (m_commit_jobs[cmdpool_id].size() == 0) {
          m_commit_jobs.erase(cmdpool_id);
        }
      }
    }
  }

  /**
   * cleanup JOB queue and cmdpool queue
   */
  void cmd_pool_destroy(void) {
    if (!m_commit_jobs.empty()) {
      for (uint32_t i = 0; i < m_commit_jobs.size(); i++)
        m_wrapper->write_register(reg_addr::TSM_CMD_SCHED_CTRL,
                                  reg_ctl::DESTROY_CMD_POOL);

      for (auto item : m_commit_jobs)
        m_commit_jobs[item.first].clear();

      m_commit_jobs.clear();
    }

    for (auto item : m_cmdpools) {
      delete item.second;
      item.second = nullptr;
    }

    m_cmdpools.clear();
    m_cmdpool_bitmap = 0;
  }

  bool cmd_pool_job_is_in(uint32_t cmdpool_id, JobBase *job) {
    return (m_commit_jobs.count(cmdpool_id) == 1) &&
           (m_commit_jobs[cmdpool_id].count(job) == 1);
  }

  uint32_t get_cmdpool_id(uint32_t part_id) {
    uint32_t index = 0, cmdpool_id = 0;

    index = m_part_cmdpool[part_id].cmdpool_id++;
    cmdpool_id = m_part_cmdpool[part_id].m_cmdpool_in_part[index];
    if (m_part_cmdpool[part_id].cmdpool_id ==
        m_part_cmdpool[part_id].m_cmdpool_in_part.size())
      m_part_cmdpool[part_id].cmdpool_id = 0;

    return cmdpool_id;
  }

  aipu_ll_status_t set_cluster_to_partition() {
    uint32_t reg_val = 0, cluster_idx = 0;
    uint32_t present_cluster_cnt = 0, part_idx = 0;
    uint32_t cluster_num_in_part[MAX_PART_CNT] = {0};
    aipu_partition_cap part_cap;

    for (uint32_t i = 0; i < MAX_CLUSTER_CNT; i++) {
      m_wrapper->read_register(reg_addr::CLUSTER0_CONFIG + 0x20 * i, reg_val);
      if (reg_val & reg_ctl::CLUSTER_PRESENT)
        present_cluster_cnt++;
    }

    /* alloc cluster number to each partition */
    for (uint32_t i = 0; i < present_cluster_cnt; i++) {
      cluster_num_in_part[part_idx++] += 1;
      if (part_idx == m_max_partition_cnt)
        part_idx = 0;
    }

    /* currently the aipu v3 simulation only for one partition, one cluster in
     * partition */
    for (uint32_t part = 0; part < m_max_partition_cnt; part++) {
      uint32_t cluster_cnt = cluster_num_in_part[part];

      if (cluster_cnt == 0)
        continue;

      memset((void *)&part_cap, 0x0, sizeof(part_cap));
      for (uint32_t idx = 0; idx < cluster_cnt; idx++) {
        m_cluster_in_part[part].push_back(cluster_idx);

        /* specify cluster to someone partition */
        m_wrapper->read_register(reg_addr::CLUSTER0_CTRL + 0x20 * cluster_idx,
                                 reg_val);
        m_wrapper->write_register(reg_addr::CLUSTER0_CTRL + 0x20 * cluster_idx,
                                  reg_val | (part << 16));
        m_wrapper->read_register(reg_addr::CLUSTER0_CONFIG + 0x20 * cluster_idx,
                                 reg_val);

        part_cap.id = part;
        part_cap.arch = AIPU_ARCH_ZHOUYI;
        part_cap.version = AIPU_ISA_VERSION_ZHOUYI_V3;
        part_cap.clusters[part_cap.cluster_cnt].core_cnt = (reg_val >> 8) & 0xF;
        part_cap.clusters[part_cap.cluster_cnt].tec_cnt = reg_val & 0xF;
        part_cap.cluster_cnt++;
        cluster_idx++;
      }

      if (part_cap.cluster_cnt > 0) {
        m_partition_cnt++;
        m_part_caps.push_back(part_cap);
      }
    }

    if (m_partition_cnt > 0) {
      m_cluster_cnt = m_part_caps[0].cluster_cnt;
      m_core_cnt = m_part_caps[0].clusters[0].core_cnt;
    }

    return AIPU_LL_STATUS_SUCCESS;
  }

  aipu_ll_status_t set_cmdpool_to_partition(uint32_t partition_cnt,
                                            uint32_t cmdpool_cnt) {
    uint32_t cluster_cnt = 0;

    if (partition_cnt > MAX_PART_CNT || cmdpool_cnt > 8) {
      LOG(LOG_ERR, "Invalid config: part %d, cmdpool %d", m_max_partition_cnt,
          m_max_cmdpool_cnt);
      return AIPU_LL_STATUS_ERROR_INVALID_CONFIG;
    }

    for (uint32_t i = 0; i < partition_cnt; i++)
      cluster_cnt += m_cluster_in_part[i].size();

    if (partition_cnt > cmdpool_cnt) {
      LOG(LOG_ERR, "Invalid config: part %d, cmdpool %d", partition_cnt,
          cmdpool_cnt);
      return AIPU_LL_STATUS_ERROR_INVALID_CONFIG;
    } else {
      uint32_t part_idx = 0;
      for (uint32_t cmdpool_idx = 0; cmdpool_idx < cmdpool_cnt; cmdpool_idx++) {
        m_part_cmdpool[part_idx].cmdpool_id = 0;
        m_part_cmdpool[part_idx++].m_cmdpool_in_part.push_back(cmdpool_idx);
        if (part_idx == partition_cnt)
          part_idx = 0;
      }
    }

    return AIPU_LL_STATUS_SUCCESS;
  }

public:
  virtual ~SimulatorV3();
  SimulatorV3(const SimulatorV3 &sim) = delete;
  SimulatorV3 &operator=(const SimulatorV3 &sim) = delete;

  aipu_ll_status_t init() override;
  bool has_target(uint32_t arch, uint32_t version, uint32_t config,
                  uint32_t rev) override;
  aipu_ll_status_t schedule(const JobDesc &job) override;

  aipu_ll_status_t poll_status(uint32_t max_cnt, int32_t time_out,
                               bool of_this_thread, void *jobbase = nullptr);

  static SimulatorV3 *get_v3_simulator() {
    static SimulatorV3 sim_instance;
    return &sim_instance;
  }

  aipu_ll_status_t get_simulation_instance(void **simulator,
                                           void **memory) override {
    if (m_wrapper != nullptr) {
      *(sim_aipu_t *)simulator = m_wrapper->sim();
      *(mem_interface_t **)memory = m_wrapper->mem();
      return AIPU_LL_STATUS_SUCCESS;
    }
    return AIPU_LL_STATUS_ERROR_INVALID_OP;
  }

  void set_target(const std::string &target) override { m_target = target; }

  void set_cfg(const aipu_global_config_simulation_t *cfg) override {
    m_cfg = cfg;
  }

  const char *get_config_code() const override {
    return m_wrapper->code_to_string().c_str();
  }

  void enable_profiling(bool en) override { m_wrapper->enable_profiling(en); }

  bool get_profile_en() const override {
    return m_wrapper->config().perf.mode == PERF_MODE_EVAL;
  }

  aipu_ll_status_t
  get_cluster_id(uint32_t part_id,
                 std::vector<uint32_t> &cluster_in_part) override {
    if (part_id > sizeof(m_cluster_in_part))
      return AIPU_LL_STATUS_ERROR_INVALID_PARTITION_ID;

    cluster_in_part = m_cluster_in_part[part_id];
    return AIPU_LL_STATUS_SUCCESS;
  }

  /**
   * the below 3 APIs help to dump tcbchains
   */
  void set_destroy_to_all_tcbchain() {
    for (auto cmdpool : m_cmdpools) {
      cmdpool.second->set_destroy_flag();
      cmdpool.second->reset_destroy_done();
    }
  }

  uint32_t get_cmdpool_bitmap() { return m_cmdpool_bitmap; }

  void clear_cmdpool_bitmap(uint32_t cmdpool_id) {
    if (m_cmdpool_bitmap & (1 << cmdpool_id))
      m_cmdpool_bitmap &= ~(1 << cmdpool_id);
  }
};

} // namespace aipudrv

#endif /* _SIMULATOR_V3_H_ */

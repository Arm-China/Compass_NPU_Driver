// Copyright (C) 2023-2025 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

/**
 * @file  simulator_v2.h
 * @brief AIPU User Mode Driver (UMD) zhouyi aipu v3_2 simulator module header
 */

#ifndef _SIMULATOR_V3_2_H_
#define _SIMULATOR_V3_2_H_

#include <string.h>

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <set>
#include <sstream>

#include "device/simulator/umemory.h"
#include "device_base.h"
#include "kmd/tcb.h"
#include "simulator/config.h"
#include "standard_api.h"
#include "utils/debug.h"
#include "wrapper.h"

namespace aipudrv {
enum {
  POOL_PCP = 0,
  POOL_MAX,
};

/* DONOT use simulator in multiple thread, because many objects are lack of
 * protection */
class SimulatorV3_2 : public DeviceBase {
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
  uint32_t m_max_cmdpool_cnt = 0;
  std::vector<BufferDesc *> m_reserve_mem;
  static constexpr uint32_t MAX_GROUP_ID = (1 << 15);

  /**
   * @cmdpool_id: the next cmdpool index in one partition
   * @m_cmdpool_in_part: the cmdpool number belong to one partition
   */
  struct CmdpoolInfo {
    uint32_t cmdpool_id;
    std::vector<uint32_t> m_cmdpool_in_part;
  };
  std::map<uint32_t, CmdpoolInfo> m_part_cmdpool;

  /* 1. buffer all jobs in this queue */
  typedef struct {
    void *job;
    JobDesc jobdesc;
  } job_queue_elem_t;
  std::queue<job_queue_elem_t> m_buffer_queue;

  /* 2. move jobs from buffer queue to this queue */
  std::map<uint16_t, void *> m_commit_map;

  /* 3. move jobs from commit queue to this queue when cmdpool done ready */
  std::set<void *> m_done_set;

  /* 4. Simulator puts all done jobs to this queue */
  static std::set<uint16_t> m_sim_done_grid_set;
  static std::mutex m_sim_done_grid_mtx;

  volatile bool m_cant_add_job_flag = false;

  std::atomic<uint16_t> m_grid_id = {1};

  bool m_group_id_bitmap[MAX_GROUP_ID] = {false};
  std::mutex m_group_id_mtx;

  uint32_t m_partition_mode = POOL_PCP;
  std::map<uint32_t, std::map<uint32_t, uint32_t>> m_cmdpool_id = {
      {
          0,
          {
              {0, 0},
              {1, 1},
              {2, 2},
              {3, 3},
          },
      },
      {1,
       {
           {0, 4},
           {1, 5},
           {2, 6},
           {3, 7},
       }},
  };

private:
  SimulatorV3_2();

  bool is_cmdpool_full(int qos, int part_id, int partition_mode,
                       int cluster_idx, uint32_t reg_val);
  aipu_ll_status_t fill_commit_queue();

  uint32_t get_cmdpool_id(uint32_t cluster_id, uint32_t part_id) {
    return m_cmdpool_id[cluster_id][part_id];
  }

  void parse_cluster_info() {
    uint32_t reg_val = 0;
    uint32_t present_cluster_cnt = 0;
    aipu_partition_cap part_cap;

    for (uint32_t i = 0; i < MAX_CLUSTER_CNT; i++) {
      m_wrapper->read_register(reg_addr::CLUSTER0_CONFIG + 0x20 * i, reg_val);
      if (reg_val & reg_ctl::CLUSTER_PRESENT)
        present_cluster_cnt++;
    }

    /* currently the aipu v3a simulation only for one partition, one cluster in
     * partition */
    memset((void *)&part_cap, 0x0, sizeof(part_cap));
    for (uint32_t cluster_idx = 0; cluster_idx < present_cluster_cnt;
         cluster_idx++) {
      m_wrapper->read_register(reg_addr::CLUSTER0_CTRL + 0x20 * cluster_idx,
                               reg_val);
      if (!(reg_val & reg_ctl::CLUSTER_ENABLE))
        continue;

      /* specify cluster to support core partition */
      reg_val |= m_partition_mode << 13;
      m_wrapper->write_register(reg_addr::CLUSTER0_CTRL + 0x20 * cluster_idx,
                                reg_val);

      part_cap.id = part_cap.cluster_cnt;
      part_cap.arch = AIPU_ARCH_ZHOUYI;
      part_cap.version = m_target.find("X3P") != std::string::npos
                             ? AIPU_ISA_VERSION_ZHOUYI_V3_2_0
                             : AIPU_ISA_VERSION_ZHOUYI_V3_2_1;
      part_cap.config = target_to_config(m_target);
      part_cap.clusters[part_cap.cluster_cnt].core_cnt = (reg_val >> 8) & 0xF;
      part_cap.clusters[part_cap.cluster_cnt].tec_cnt = reg_val & 0xF;
      part_cap.cluster_cnt++;

      m_partition_cnt++;
      m_part_caps.push_back(part_cap);
    }

    if (m_partition_cnt > 0) {
      m_cluster_cnt = m_part_caps[0].cluster_cnt;
      m_core_cnt = m_part_caps[0].clusters[0].core_cnt;
    }
  }

public:
  virtual ~SimulatorV3_2();
  SimulatorV3_2(const SimulatorV3_2 &sim) = delete;
  SimulatorV3_2 &operator=(const SimulatorV3_2 &sim) = delete;

  aipu_ll_status_t init() override;
  bool has_target(uint32_t arch, uint32_t version, uint32_t config,
                  uint32_t rev) override;
  aipu_ll_status_t schedule(const JobDesc &job) override;
  aipu_ll_status_t poll_status(uint32_t max_cnt, int32_t time_out,
                               bool of_this_thread, void *jobbase = nullptr);
  static void sim_cb_handler(uint32_t event, uint64_t value, void *context);

  int get_grid_id(uint16_t &grid_id) override;
  int get_start_group_id(int group_cnt, uint16_t &start_group_id) override;

  static SimulatorV3_2 *get_v3_2_simulator() {
    static SimulatorV3_2 sim_instance;
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

  int put_start_group_id(uint16_t start_group_id, int group_cnt) override {
    int ret = 0;

    m_group_id_mtx.lock();
    for (uint16_t i = start_group_id; i < start_group_id + group_cnt; i++)
      m_group_id_bitmap[i] = false;
    m_group_id_mtx.unlock();

    return ret;
  }

  const char *get_config_code() const override {
    return m_wrapper->code_to_string().c_str();
  }

  aipu_ll_status_t
  get_cluster_id(uint32_t part_id,
                 std::vector<uint32_t> &cluster_in_part) override {
    if (part_id > sizeof(m_cluster_in_part))
      return AIPU_LL_STATUS_ERROR_INVALID_PARTITION_ID;

    cluster_in_part = m_cluster_in_part[part_id];
    return AIPU_LL_STATUS_SUCCESS;
  }

  void enable_profiling(bool en) override { m_wrapper->enable_profiling(en); }

  void dump_profiling() override { m_wrapper->dump_profiling(); }

  bool get_profile_en() const override {
    return m_wrapper->config().perf.mode == PERF_MODE_FAST;
  }
};
} // namespace aipudrv

#endif /* _SIMULATOR_V3_2_H_ */

// Copyright (C) 2023-2025 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

/**
 * @file  job_v3x.h
 * @brief AIPU User Mode Driver (UMD) job class header
 */

#ifndef _JOB_V3X_H_
#define _JOB_V3X_H_

#include <memory>
#include <set>
#include <vector>

#include "graph_base.h"
#include "job_base.h"
#include "kmd/tcb.h"
#include "zhouyi_v3x/common/coredump.h"
#include "zhouyi_v3x/common/dynamic_shape.h"
#include "zhouyi_v3x/common/gm_v3x.h"
#include "zhouyi_v3x/common/graph_v3x.h"

namespace aipudrv {
struct TCB {
  DEV_PA_64 pa;
  void init(DEV_PA_64 _pa) { pa = _pa; }
};

struct Task {
  TCB tcb;
  BufferDesc *stack = nullptr;
  BufferDesc *private_data = nullptr;
};

struct SubGraphTask {
  uint32_t id;
  uint32_t bss_id;
  std::vector<BufferDesc *> reuse_priv_buffers;
  std::vector<Task> tasks;

  /**
   * record buffer index, will not free these special buffer as it is
   * allocated externally.
   */
  void reset(uint32_t _id, uint32_t _bss_id) {
    id = _id;
    bss_id = _bss_id;
    reuse_priv_buffers.clear();
    tasks.clear();
  }
};

struct DumpcfgInputDesc {
  std::string file;
  DEV_PA_64 base;
};

struct DumpcfgOutputDesc {
  std::string file;
  DEV_PA_64 base;
  uint32_t size;
};

struct dumpcfgHostDesc {
  uint32_t part_id;
  uint32_t hi_addr;
  uint32_t lo_addr;
};

class JobV3X : public JobBase {
protected:
  uint32_t m_tot_tcb_cnt = 0;
  uint32_t m_sg_cnt = 0;
  uint32_t m_core_cnt = 0;
  uint32_t m_task_per_sg = 0;
  uint32_t m_partition_id = 0;
  uint16_t m_grid_id = 0;
  uint32_t m_core_id = 0;
  uint32_t m_cluster_id = 0;
  uint32_t m_qos = 0;
  bool m_bind_enable = false;
  bool m_is_defer_run = false;
  bool m_do_trigger = false;
  bool m_dyn_data_load = false;
  std::vector<WeightBufferInfo> m_weight;
  std::map<FMSection, BufferDesc *> m_secbuf_desc;
  /* for feature map specified memory */
  uint32_t m_sfm_mem_region = AIPU_MEM_REGION_DEFAULT;
  std::set<uint32_t> m_sfm_idxes;
  aipu_dynshape_param_t *m_dynshape_param = nullptr;

protected:
  TCB m_init_tcb;
  std::unique_ptr<char[]> m_backup_tcb;
  bool m_backup_tcb_used = false;
  std::vector<SubGraphTask> m_sg_job;
  uint32_t m_segmmu_num = 0;
  std::vector<SegMMUConfig> m_segmmu_sec;
  DynamicShape *m_dyn_shape = nullptr;
  Coredump *m_coredump = nullptr;
  GM_V3X *m_gm = nullptr;

  std::vector<BufferDesc *> m_reuses_desc; /* includes all bss reuses */
  std::set<uint32_t> m_dmabuf_idxs;

  /**
   * record stack/dp buffers for each allocated subgraph, try to
   * share them to subsequent subgraphs.
   */
  std::vector<SubGraphTask *> m_sgt_allocated;

  uint32_t m_reserved_iova_size = 0;

  BufferDesc *m_tcbs = nullptr;
  BufferDesc *m_global_param = nullptr;
  BufferDesc *m_top_priv_buf = nullptr;

  /* dump all jobs */
  std::string m_dumpcfg_header;
  dumpcfgHostDesc m_dumpcfg_host;
  std::vector<DumpcfgInputDesc> m_dumpcfg_input;
  std::vector<DumpcfgOutputDesc> m_dumpcfg_output;
  std::string m_dumpcfg_meta;
  std::tuple<std::string, uint64_t, uint32_t> m_dump_tcb_info[2];

public:
  /* for simulation, record which cmdpool this job is committed to */
  uint32_t m_bind_cmdpool_id;

public:
  GraphV3X &graph() { return static_cast<GraphV3X &>(m_graph); }

  MemoryBase *mem() { return m_mem; }

  Coredump *get_coredump() { return m_coredump; }

  uint32_t get_subgraph_cnt() override { return graph().get_subgraph_cnt(); }

  const std::vector<BufferDesc *> &get_reuse() override {
    return m_reuses_desc;
  }

private:
  aipu_status_t parse_dynamic_out_shape() override;

  aipu_status_t setup_reuse_buffer(const BufferDesc *desc);
  aipu_status_t setup_gm_buffer(const BufferDesc *desc);
  aipu_status_t setup_dyn_shape_buffer(const BufferDesc *desc);
  aipu_status_t setup_sg_priv_buffer(const BufferDesc *desc);
  aipu_status_t setup_sg_common_buffer(const BufferDesc *stack,
                                       const BufferDesc *dp);
  aipu_status_t setup_job_buffers();

protected:
  virtual void set_job_params(uint32_t sg_cnt, uint32_t task_per_sg,
                              uint32_t remap, uint32_t core_cnt) = 0;
  virtual aipu_status_t setup_task_tcb(uint32_t sg_id, uint32_t grid_id,
                                       uint32_t core_id, uint32_t task_id,
                                       bool new_grid = false) = 0;
  virtual aipu_status_t setup_tcb_group(uint32_t sg_id, uint32_t grid_id,
                                        uint32_t core_id, bool new_grid) = 0;
  virtual aipu_status_t setup_tcb_chain() = 0;
  virtual aipu_status_t setup_segmmu(SubGraphTask &sg_task) = 0;
  virtual uint32_t get_tcb_head_cnt(uint32_t sg_idx, uint32_t head_cnt) = 0;
  virtual DEV_PA_64 get_first_task_tcb_pa() = 0;

  virtual aipu_status_t alloc_job_buffers() = 0;
  virtual aipu_status_t free_job_buffers() override = 0;
  void free_sg_buffers(SubGraphTask &sg_task);

  aipu_status_t
  config_dynamic_params(const aipu_dynshape_param_t *params) override;
  aipu_status_t
  setup_rodata_sg(const std::vector<GraphParamMapLoadDesc> &param_map,
                  std::vector<BufferDesc *> &reuse_buf,
                  std::vector<BufferDesc *> &static_buf,
                  std::set<uint32_t> *dma_buf_idx = nullptr);
  aipu_status_t
  specify_io_buffer(aipu_shared_tensor_info_t &tensor_info) override;
  virtual aipu_status_t
  specify_io(aipu_shared_tensor_info_t &tensor_info,
             const std::vector<JobIOBuffer> *iobuffer_vec) = 0;

  virtual aipu_status_t init_grid_id(uint16_t &grid_id) {
    if (graph().get_bss_cnt() > 1) {
      /* placeholder first grid id to avoid next bss duplicate */
      if (m_dev->get_grid_id(grid_id) < 0)
        return AIPU_STATUS_ERROR_ALLOC_GRIP_ID;
    }
    return AIPU_STATUS_SUCCESS;
  }
  virtual aipu_status_t init_group_id(uint32_t sg_cnt) {
    return AIPU_STATUS_SUCCESS;
  }
  virtual uint16_t get_last_group_id() const { return 0; }
  virtual aipu_status_t do_coredump() override {
    return m_coredump != nullptr ? m_coredump->do_coredump()
                                 : AIPU_STATUS_ERROR_INVALID_COREDUMP;
  }

#if defined(SIMULATION)
  virtual void dumpcfg_alljob() = 0;
#endif

public:
  aipu_status_t init(const aipu_global_config_simulation_t *cfg,
                     const aipu_global_config_hw_t *hw_cfg) override;
  aipu_status_t schedule() override;
  aipu_status_t destroy() override;
  aipu_status_t bind_core(uint32_t core_id) override;
  aipu_status_t debugger_run() override;

  /* dump */
  virtual aipu_status_t dump_for_emulation();
  virtual aipu_status_t dump_emu_metadata(const std::string &metafile) = 0;
  void dump_extension_buffers(bool before) override;
  void dump_graphjson(const std::string &fullpath) override;

public:
  uint16_t get_grid_id() { return m_grid_id; }

  uint32_t get_part_id() { return m_partition_id; }

  uint32_t get_qos() { return m_qos; }

  DEV_PA_64 get_tcb_head_pa() { return m_init_tcb.pa; }

protected:
  JobV3X(MainContext *ctx, GraphBase &graph, DeviceBase *dev,
         aipu_create_job_cfg_t *config = nullptr);
  ~JobV3X();
  JobV3X(const JobV3X &job) = delete;
  JobV3X &operator=(const JobV3X &job) = delete;

  friend class GM_V3X;
  friend class GM_V3;
  friend class GM_V3_2;
  friend class Coredump;
};
} // namespace aipudrv

#endif /* _JOB_V3X_H_ */
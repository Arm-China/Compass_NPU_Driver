// Copyright (C) 2023-2025 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

/**
 * @file  job_v3_2.h
 * @brief AIPU User Mode Driver (UMD) v3_2 job class header
 */

#ifndef _JOB_V3_2_H_
#define _JOB_V3_2_H_

#include <memory>

#include "graph_base.h"
#include "kmd/tcb.h"
#include "zhouyi_v3x/common/job_v3x.h"
#include "zhouyi_v3x/zhouyi_v3_2/gm_v3_2.h"

namespace aipudrv {
class JobV3_2 : public JobV3X {
private:
  uint16_t m_start_group_id = 0;
  uint16_t m_group_id_idx = 0;
  BufferDesc *m_top_job_buf = nullptr;
  std::map<FMSection, BufferDesc> m_secbuf_desc;
  /* put_weights_gm:weight+zcy, other:zcy */
  std::vector<WeightBufferInfo> m_weight;
  uint64_t m_exec_id = 0;

private:
  void set_job_params(uint32_t sg_cnt, uint32_t task_per_sg, uint32_t remap,
                      uint32_t core_cnt) override;
  aipu_status_t setup_task_tcb(uint32_t sg_id, uint32_t grid_id,
                               uint32_t core_id, uint32_t task_id,
                               bool is_new_grid = false) override;
  aipu_status_t setup_tcb_group(uint32_t sg_id, uint32_t grid_id,
                                uint32_t core_id,
                                bool is_new_grid = false) override;

  aipu_status_t setup_tcb_chain() override;
  void get_tcb_head_cnt(uint32_t sg_idx, uint32_t &head_cnt) override;
  aipu_status_t free_job_buffers() override;
  void free_sg_buffers(SubGraphTask &sg_task) override;
  void setup_gm_sync_to_ddr(tcb_t &tcb) override{};
  aipu_status_t setup_segmmu(SubGraphTask &sg_task) override;
  aipu_status_t dump_for_emulation() override;
  aipu_status_t init_group_id(uint32_t sg_cnt) override;
  aipu_status_t alloc_load_job_buffers() override;
  DEV_PA_64 get_first_task_tcb_pa() override {
    return m_init_tcb.pa + 2 * sizeof(tcb_t);
  }
  DEV_PA_64 get_asid0_base() override { return m_top_job_buf->asid_base; }

  aipu_status_t config_tcb_smmu(tcb_t &tcb);
  aipu_status_t config_tcb_deps(tcb_t &tcb, uint32_t sg_id);
  aipu_status_t setup_gm_buffer();
  aipu_status_t setup_dyn_shape_buffer();
  aipu_status_t setup_sg_priv_buffer();
  aipu_status_t setup_reuse_buffer();
  aipu_status_t setup_sg_common_buffer();
  aipu_status_t load_job_buffers();
  aipu_status_t
  specify_io_buffer(aipu_shared_tensor_info_t &tensor_info) override;

public:
#if defined(SIMULATION)
  void dumpcfg_alljob() override;
#endif

public:
  JobV3_2(MainContext *ctx, GraphBase &graph, DeviceBase *dev,
          aipu_create_job_cfg_t *config = nullptr);
  ~JobV3_2();
  JobV3_2(const JobV3_2 &job) = delete;
  JobV3_2 &operator=(const JobV3_2 &job) = delete;

  friend class GM_V3_2;
};

} // namespace aipudrv

#endif /* _JOB_V3_2_H_ */
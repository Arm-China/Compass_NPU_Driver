// Copyright (C) 2023-2025 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

/**
 * @file  job_v3.h
 * @brief AIPU User Mode Driver (UMD) job class header
 */

#ifndef _JOB_V3_H_
#define _JOB_V3_H_
#include <memory>

#include "kmd/tcb.h"
#include "zhouyi_v3x/common/job_v3x.h"
#include "zhouyi_v3x/zhouyi_v3/gm_v3.h"

namespace aipudrv {
class JobV3 : public JobV3X {
private:
  uint32_t m_segmmu_tcb_num = 0;
  bool m_top_reuse = true;

private:
  void set_job_params(uint32_t sg_cnt, uint32_t task_per_sg, uint32_t remap,
                      uint32_t core_cnt) override;
  aipu_status_t alloc_job_buffers() override;
  aipu_status_t free_job_buffers() override;
  aipu_status_t setup_task_tcb(uint32_t sg_id, uint32_t grid_id,
                               uint32_t core_id, uint32_t task_id,
                               bool new_grid = false) override;
  aipu_status_t setup_tcb_group(uint32_t sg_id, uint32_t grid_id,
                                uint32_t core_id, bool new_grid) override;
  aipu_status_t setup_tcb_chain() override;
  aipu_status_t config_tcb_smmu(DEV_PA_64 init_tcb_pa);
  aipu_status_t setup_segmmu(SubGraphTask &sg_task) override;
  aipu_status_t
  specify_io(aipu_shared_tensor_info_t &tensor_info,
             const std::vector<JobIOBuffer> *iobuffer_vec) override;
  aipu_status_t alloc_reuse_buffer(); /* reserved */
  DEV_PA_64 get_first_task_tcb_pa() override {
    return m_init_tcb.pa + (1 + m_segmmu_tcb_num) * tcb_ctl::TCB_LEN;
  }

  uint32_t get_tcb_head_cnt(uint32_t sg_idx, uint32_t head_cnt) override {
    static uint32_t cur_bss_idx = graph().get_subgraph(0).bss_idx;

    if ((sg_idx == 0) ||
        (cur_bss_idx != graph().get_subgraph(sg_idx).bss_idx)) {
      cur_bss_idx = graph().get_subgraph(sg_idx).bss_idx;
      head_cnt += 1;
      head_cnt += m_segmmu_tcb_num;
    }
    return head_cnt;
  }

  /* dump */
  aipu_status_t dump_emu_metadata(const std::string &metafile) override;

public:
#if defined(SIMULATION)
  void dumpcfg_alljob() override;
#endif

public:
  JobV3(MainContext *ctx, GraphBase &graph, DeviceBase *dev,
        aipu_create_job_cfg_t *config = nullptr);
  ~JobV3();
  JobV3(const JobV3 &job) = delete;
  JobV3 &operator=(const JobV3 &job) = delete;

  friend class GM_V3;
};
} // namespace aipudrv

#endif /* _JOB_V3_H_ */
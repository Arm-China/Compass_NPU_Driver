// Copyright (C) 2023-2025 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

#include "gm_v3_2.h"

#include <vector>

#include "utils/helper.h"
#include "zhouyi_v3x/common/coredump.h"
#include "zhouyi_v3x/common/graph_v3x.h"
#include "zhouyi_v3x/zhouyi_v3_2/job_v3_2.h"
#if defined(SIMULATION)
#include "device/simulator/simulator_v3_2.h"
#endif

namespace aipudrv {
using namespace tcb_v3_2;

GM_V3_2::GM_V3_2(JobV3_2 &job) : GM_V3X(job, job.graph(), job.mem()) {}

GM_V3_2::~GM_V3_2() {}

void GM_V3_2::setup_gm_sync_from_ddr(tcb_t &tcb) {
  if (!m_mem->is_gm_enable()) {
    LOG(LOG_INFO, "gm is disable");
    return;
  }

  if (m_mem->get_gm_size() == 0) {
    LOG(LOG_DEBUG, "gm size is 0");
    return;
  }
  JobV3_2 &job = reinterpret_cast<JobV3_2 &>(m_job);
  if (m_graph.m_wt_in_gm_storage_flag != GM_WT_FLAG_WEIGHT_ONLY &&
      m_graph.m_wt_in_gm_storage_flag != GM_WT_FLAG_WEIGHT_INDEX_ONLY) {
    if (job.m_secbuf_desc.count(FMSection::GM) == 0 ||
        job.m_secbuf_desc.at(FMSection::GM)->size == 0) {
      LOG(LOG_INFO, "gm buffer size is 0");
      return;
    }
  }

  const auto &gm_info = m_graph.get_gmsec_info();

  uint32_t remap_mode = 0; /* time priority */
  uint32_t remap_size = (aligned(gm_info.remap_size, 1 << 18) >> 18) - 1;
  tcb.grid.gm_ctrl = (remap_size & 0xFF) << 8 | (remap_mode & 0x1) << 1 |
                     tcb_ctl::GM_CTRL_REMAP_EN;

  if (gm_info.sync_size != 0) {
    uint32_t sync_size = (aligned(gm_info.sync_size, 1 << 18) >> 18);
    tcb.grid.gm_sync = tcb_ctl::GM_REGION_CTRL_SYNC_TO_GM | (sync_size & 0xFFF);
  }
  std::vector<WeightBufferInfo> weights = m_graph.get_weight_buffer_info();

  if (m_graph.m_wt_in_gm_storage_flag == GM_WT_FLAG_WEIGHT_ONLY) {
    BufferDesc *gm_desc = weights[0].wb_weight;
    tcb.grid.gm_addr_low = get_low_32(gm_desc->pa);
    tcb.grid.gm_addr_high = get_high_32(gm_desc->pa);
  } else if (m_graph.m_wt_in_gm_storage_flag == GM_WT_FLAG_WEIGHT_INDEX_ONLY) {
    tcb.grid.gm_addr_low = get_low_32(weights[0].wb_weight_index_gm_pa);
    tcb.grid.gm_addr_high = get_high_32(weights[0].wb_weight_index_gm_pa);
  } else if (m_graph.m_wt_in_gm_storage_flag == GM_WT_FLAG_NO_WEIGHT ||
             m_graph.m_wt_in_gm_storage_flag == GM_WT_FLAG_WEIGHT_FM ||
             m_graph.m_wt_in_gm_storage_flag == GM_WT_FLAG_WEIGHT_INDEX_FM) {
    const auto &gm_desc = job.m_secbuf_desc.at(FMSection::GM);
    tcb.grid.gm_addr_low = get_low_32(gm_desc->pa);
    tcb.grid.gm_addr_high = get_high_32(gm_desc->pa);
  }
}

} // namespace aipudrv
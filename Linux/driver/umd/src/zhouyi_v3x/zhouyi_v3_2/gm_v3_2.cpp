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
GM_V3_2::GM_V3_2(JobV3_2 &job) : GM_V3X(job, job.graph()) { m_mem = job.m_mem; }

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
  if (job.m_secbuf_desc.count(FMSection::GM) == 0 ||
      job.m_secbuf_desc.at(FMSection::GM).size == 0) {
    LOG(LOG_INFO, "gm buffer size is 0");
    return;
  }

  const auto &gm_desc = job.m_secbuf_desc.at(FMSection::GM);
  const auto &gm_info = m_graph.get_gmsec_info();

  uint32_t remap_mode = 0; /* time priority */
  uint32_t remap_size = (aligned(gm_info.remap_size, 1 << 18) >> 18) - 1;
  tcb.grid.gm_ctrl =
      (remap_size & 0xFF) << 8 | (remap_mode & 0x1) << 1 | GM_CTRL_REMAP_EN;

  if (gm_info.sync_size != 0) {
    uint32_t sync_size = (aligned(gm_info.sync_size, 1 << 18) >> 18);
    tcb.grid.gm_sync = GM_SYNC_DDR_TO_GM | (sync_size & 0xFFF);
  }

  tcb.grid.gm_addr_low = get_low_32(gm_desc.pa);
  tcb.grid.gm_addr_high = get_high_32(gm_desc.pa);
}

} // namespace aipudrv
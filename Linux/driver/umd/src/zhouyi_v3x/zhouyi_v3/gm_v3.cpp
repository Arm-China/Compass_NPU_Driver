// Copyright (C) 2023-2025 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

#include "gm_v3.h"

#include <cstring>

#include "utils/helper.h"
#include "zhouyi_v3x/common/coredump.h"
#include "zhouyi_v3x/common/graph_v3x.h"
#include "zhouyi_v3x/zhouyi_v3/job_v3.h"
#if defined(SIMULATION) && defined(ZHOUYI_V3)
#include "device/simulator/simulator_v3.h"
#endif

namespace aipudrv {
using namespace tcb_v3;

GM_V3::GM_V3(JobV3 &job) : GM_V3X(job, job.graph(), job.mem()) {}

GM_V3::~GM_V3() {}

void GM_V3::gm_dynamic_switch(uint32_t core_cnt) {
  /**
   * set this env variable as true(y/Y), small model containing only
   * one subgraph can use GM. but if hope multiple small models
   * parallel to run, it has to be set as false(n/N).
   */
  const char *gm_allow_small_model = getenv("UMD_GM_ASM");

  if (gm_allow_small_model != nullptr) {
    if (gm_allow_small_model[0] == 'y' || gm_allow_small_model[0] == 'Y')
      m_gm_asm = true;
  } else {
    if (core_cnt > 1)
      m_gm_asm = false;
    else if (core_cnt == 1)
      m_gm_asm = true;
  }
}

aipu_status_t GM_V3::gm_malloc(uint32_t bss_id, uint32_t idx, uint32_t buf_type,
                               std::string &buf_name, BufferDesc *buf) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;

  const std::vector<GraphSectionDesc> &section_desc =
      (buf_type == GM_BUF_TYPE_REUSE) ? m_graph.get_bss(bss_id).reuse_sections
                                      : m_graph.get_bss(bss_id).static_sections;
  uint32_t buf_size = section_desc[idx].size, gm_size = 0;

  uint32_t gm_id = 0;
  if (m_job.m_qos == AIPU_JOB_QOS_HIGH)
    gm_id = 1;
  gm_size = m_mem->get_gm_size(gm_id);

  if (buf_size < gm_size)
    buf_size = gm_size;

  int pad_sz = m_graph.get_alloc_pad_size();
  uint32_t mem_region = AIPU_BUF_REGION_DEFAULT;
  ret = m_mem->malloc(buf_size + pad_sz, section_desc[idx].align_in_page, &buf,
                      buf_name.c_str(), AIPU_ASID0 | mem_region);
  if (ret != AIPU_STATUS_SUCCESS)
    return ret;

  return AIPU_STATUS_SUCCESS;
}

aipu_status_t GM_V3::setup_buffer(uint32_t bss_id, uint32_t idx,
                                  uint32_t buf_type, BufferDesc *buf) {
  ValidSyncBuffer io_region = {0};

  uint32_t gm_region_sz = (m_job.m_qos == AIPU_JOB_QOS_SLOW)
                              ? m_mem->get_gm_size(AIPU_JOB_QOS_SLOW)
                              : m_mem->get_gm_size(AIPU_JOB_QOS_HIGH);

  /* record and free weight buffer, the reuse buffer is freed in another path */
  if (buf_type == GM_BUF_TYPE_WEIGHT) {
    LOG(LOG_WARN, "v3 hasn't support put weight into gm at runtime");
    return AIPU_STATUS_SUCCESS;
  }

  /* ignore reuse temp buffer (non input/output) */
  if (buf_type == GM_BUF_TYPE_REUSE &&
      m_graph.m_gm_config_desc[buf_type][idx].sub_buf_type ==
          GM_SUB_BUF_TYPE_IGNORE)
    return AIPU_STATUS_SUCCESS;

  set_valid_map_base(*buf);
  if (m_graph.m_gm_config_desc[buf_type][idx].sub_buf_type ==
      GM_SUB_BUF_TYPE_TEMP)
    return AIPU_STATUS_SUCCESS;

  set_valid_sync_region(bss_id, idx, buf_type, *buf, io_region);
  for (int _buf_typ = EM_GM_BUF_INPUT; _buf_typ < EM_GM_BUF_MAX; _buf_typ++) {
    DEV_PA_64 sync_pa = io_region.valid_sync_buf[_buf_typ].sync_pa;
    uint32_t sync_sz = io_region.valid_sync_buf[_buf_typ].sync_size;

    if (sync_pa == 0 && sync_sz == 0)
      continue;

    if ((sync_pa < m_gm_map_base) || (sync_pa >= m_gm_map_base + gm_region_sz))
      continue;

    m_gm_sync_buf_cnt[_buf_typ]++;
    if (m_gm_sync_buf_base[_buf_typ] == 0) {
      m_gm_sync_buf_base[_buf_typ] = sync_pa;
      if (sync_pa + sync_sz >= m_gm_map_base + gm_region_sz)
        m_gm_sync_buf_size[_buf_typ] = m_gm_map_base + gm_region_sz - sync_pa;
      else
        m_gm_sync_buf_size[_buf_typ] = sync_sz;
    } else if (sync_pa + sync_sz <= m_gm_map_base + gm_region_sz) {
      m_gm_sync_buf_size[_buf_typ] =
          sync_pa + sync_sz - m_gm_sync_buf_base[_buf_typ];
    } else if ((sync_pa < m_gm_map_base + gm_region_sz) &&
               (sync_pa + sync_sz > m_gm_map_base + gm_region_sz)) {
      m_gm_sync_buf_size[_buf_typ] =
          m_gm_map_base + gm_region_sz - m_gm_sync_buf_base[_buf_typ];
    } else {
      LOG(LOG_ERR, "GM buff invalid: pa: %lx, sz: %x, base_pa: %lx\n", sync_pa,
          sync_sz, m_gm_sync_buf_base[_buf_typ]);
      return AIPU_STATUS_ERROR_INVALID_GM;
    }
  }

  return AIPU_STATUS_SUCCESS;
}

bool GM_V3::is_gm_buffer(uint32_t idx, uint32_t buf_type) {
  bool ret = false;

  if (!m_mem->is_gm_enable())
    goto out;

  if (m_job.m_qos == AIPU_JOB_QOS_HIGH && !m_mem->is_both_gm_region_enable())
    goto out;

  if (m_graph.m_gm_config_desc[buf_type].count(idx) != 1)
    goto out;

  if (!m_gm_asm && m_job.m_sg_cnt == 1) {
    m_graph.m_gm_config_desc[buf_type].erase(idx);
    goto out;
  }

  ret = true;
out:
  return ret;
}

bool GM_V3::gm_need_remap() {
  if ((m_gm_sync_buf_size[EM_GM_BUF_INPUT] != 0) ||
      (m_gm_sync_buf_size[EM_GM_BUF_OUTPUT] != 0))
    return true;

  if (m_gm_map_base != 0)
    return true;

  /* is a temp buffer, non input/output */
  return false;
}

bool GM_V3::gm_need_sync_out() {
  if (m_gm_sync_buf_size[EM_GM_BUF_OUTPUT] != 0)
    return true;
  else
    return false;
}

void GM_V3::set_valid_map_base(BufferDesc &buf) {
  if (m_gm_alloc_buffer.size() == 0)
    m_gm_map_base = buf.pa;
  m_gm_alloc_buffer.push_back(&buf);
}

void GM_V3::set_valid_sync_region(uint32_t bss_id, uint32_t idx,
                                  uint32_t buf_type, BufferDesc &buf,
                                  ValidSyncBuffer &region) {
  if (buf_type == GM_BUF_TYPE_REUSE) {
    for (auto desc : m_graph.get_bss(bss_id).io.inputs) {
      /**
       * it exist several input buffers exist in one large buffer
       */
      if (desc.ref_section_iter == idx) {
        if (region.valid_sync_buf[EM_GM_BUF_INPUT].sync_size == 0) {
          region.valid_sync_buf[EM_GM_BUF_INPUT].sync_pa =
              buf.pa + desc.offset_in_section;
          region.valid_sync_buf[EM_GM_BUF_INPUT].sync_size =
              (desc.size + AIPU_PAGE_SIZE) & (~(AIPU_PAGE_SIZE - 1));
        } else {
          if (region.valid_sync_buf[EM_GM_BUF_INPUT].sync_pa <
              buf.pa + desc.offset_in_section) {
            region.valid_sync_buf[EM_GM_BUF_INPUT].sync_size =
                (buf.pa + desc.offset_in_section + desc.size -
                 region.valid_sync_buf[EM_GM_BUF_INPUT].sync_pa +
                 AIPU_PAGE_SIZE) &
                (~(AIPU_PAGE_SIZE - 1));
          } else {
            DEV_PA_64 tmp_pa = region.valid_sync_buf[EM_GM_BUF_INPUT].sync_pa;
            uint32_t tmp_sz = region.valid_sync_buf[EM_GM_BUF_INPUT].sync_size;
            region.valid_sync_buf[EM_GM_BUF_INPUT].sync_pa =
                buf.pa + desc.offset_in_section;
            region.valid_sync_buf[EM_GM_BUF_INPUT].sync_size =
                (tmp_pa + tmp_sz -
                 region.valid_sync_buf[EM_GM_BUF_INPUT].sync_pa +
                 AIPU_PAGE_SIZE) &
                (~(AIPU_PAGE_SIZE - 1));
          }
        }
      }
    }

#if 0
        for (auto desc : m_graph.get_subgraph(bss_id).io.outputs)
        {
            /**
             * it exist several output buffers exist in one large buffer
             */
            if (desc.ref_section_iter == idx)
            {
                if (region.valid_sync_buf[EM_GM_BUF_OUTPUT].sync_size == 0)
                {
                    region.valid_sync_buf[EM_GM_BUF_OUTPUT].sync_pa = buf.pa + desc.offset_in_section;
                    region.valid_sync_buf[EM_GM_BUF_OUTPUT].sync_size =
                        (desc.size + AIPU_PAGE_SIZE) & (~(AIPU_PAGE_SIZE - 1));
                } else {
                    if (region.valid_sync_buf[EM_GM_BUF_OUTPUT].sync_pa < buf.pa + desc.offset_in_section)
                    {
                        region.valid_sync_buf[EM_GM_BUF_OUTPUT].sync_size = (buf.pa + desc.offset_in_section + desc.size -
                            region.valid_sync_buf[EM_GM_BUF_OUTPUT].sync_pa + AIPU_PAGE_SIZE) & (~(AIPU_PAGE_SIZE - 1));
                    } else {
                        DEV_PA_64 tmp_pa = region.valid_sync_buf[EM_GM_BUF_OUTPUT].sync_pa;
                        uint32_t tmp_sz = region.valid_sync_buf[EM_GM_BUF_OUTPUT].sync_size;
                        region.valid_sync_buf[EM_GM_BUF_OUTPUT].sync_pa = buf.pa + desc.offset_in_section;
                        region.valid_sync_buf[EM_GM_BUF_OUTPUT].sync_size = (tmp_pa + tmp_sz -
                            region.valid_sync_buf[EM_GM_BUF_OUTPUT].sync_pa + AIPU_PAGE_SIZE) & (~(AIPU_PAGE_SIZE - 1));
                    }
                }
            }
        }
#endif
  } else {
    region.valid_sync_buf[EM_GM_BUF_INPUT].sync_pa = buf.pa;
    region.valid_sync_buf[EM_GM_BUF_INPUT].sync_size = buf.size;
    region.valid_sync_buf[EM_GM_BUF_OUTPUT].sync_pa = 0;
    region.valid_sync_buf[EM_GM_BUF_OUTPUT].sync_size = 0;
  }
}

void GM_V3::setup_gm_sync_from_ddr(tcb_t &tcb) {
  uint32_t gm_region_idx = 0;

  if (!m_mem->is_gm_enable())
    return;

  if (!gm_need_remap())
    return;

  if (!m_mem->is_both_gm_region_enable() && m_job.m_qos == AIPU_JOB_QOS_HIGH)
    return;

  if (m_job.m_qos == AIPU_JOB_QOS_SLOW)
    gm_region_idx = 0;
  else if (m_job.m_qos == AIPU_JOB_QOS_HIGH)
    gm_region_idx = 1;

  if (m_mem->is_both_gm_region_enable())
    tcb.grid.gm_ctrl = tcb_ctl::GM_CTRL_REMAP_BOTH_REGION_EN;
  else
    tcb.grid.gm_ctrl = tcb_ctl::GM_CTRL_REMAP_REGION0_EN;

  tcb.grid.gm_rgnx_addr[0].v64 = 0;
  tcb.grid.gm_rgnx_addr[1].v64 = 0;
  tcb.grid.gm_rgnx_addr[gm_region_idx].v64 = m_gm_map_base;

  tcb.grid.gm_rgnx_ctrl[0] = tcb_ctl::GM_REGION_CTRL_IGNORE_CFG;
  tcb.grid.gm_rgnx_ctrl[1] = tcb_ctl::GM_REGION_CTRL_IGNORE_CFG;
  if (m_gm_map_base != 0)
    tcb.grid.gm_rgnx_ctrl[gm_region_idx] = 0;

  if (m_gm_sync_buf_size[EM_GM_BUF_INPUT] != 0) {
    tcb.grid.gm_rgnx_ctrl[gm_region_idx] = tcb_ctl::GM_REGION_CTRL_SYNC_TO_GM;
    DEV_PA_64 offset = m_gm_sync_buf_base[EM_GM_BUF_INPUT] - m_gm_map_base;
    tcb.grid.gm_rgnx_ctrl[gm_region_idx] |=
        get_low_32(offset >> 12) & 0xffff000;
    tcb.grid.gm_rgnx_ctrl[gm_region_idx] |=
        (m_gm_sync_buf_size[EM_GM_BUF_INPUT] >> 12) & 0xfff;
  }
}

} // namespace aipudrv
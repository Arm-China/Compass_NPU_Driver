// Copyright (C) 2023-2024 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

/**
 * @file  job_v3.cpp
 * @brief AIPU User Mode Driver (UMD) aipu v3 job module implementation
 */

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <cstring>
#include <mutex>

// clang-format off
#include "job_v3.h"
// clang-format on

#include "utils/helper.h"
#include "zhouyi_v3x/common/dynamic_shape.h"
#include "zhouyi_v3x/common/graph_v3x.h"

#if defined(SIMULATION)
#include "device/simulator/simulator_v3.h"
#endif

namespace aipudrv {
JobV3::JobV3(MainContext *ctx, GraphBase &graph, DeviceBase *dev,
             aipu_create_job_cfg_t *config)
    : JobV3X(ctx, graph, dev, config) {
  m_gm = new GM_V3(*this);

  for (uint32_t i = 0; i < get_graph().get_weight_buffer_info().size(); i++) {
    if (m_mem->get_asid_base(0) !=
        get_graph().get_weight_buffer_info()[i].wb_asid_base) {
      m_same_asid = false;
      break;
    }
  }
  if (m_same_asid && m_mem->get_asid_base(0) != m_mem->get_asid_base(1))
    m_same_asid = false;
}

JobV3::~JobV3() {
  if (m_gm != nullptr) {
    delete m_gm;
    m_gm = nullptr;
  }

  if (m_dyn_shape != nullptr) {
    delete m_dyn_shape;
    m_dyn_shape = nullptr;
  }
}

void JobV3::set_job_params(uint32_t sg_cnt, uint32_t task_per_sg,
                           uint32_t remap, uint32_t core_cnt) {
  m_sg_cnt = sg_cnt;
  m_task_per_sg = task_per_sg;
  m_remap_flag = remap;

  /**
   * single BSS and tcb chain format:
   *   1 init-tcb + n task-tcb
   *
   * - the placehold task tcb is needed for appending new tcb chain.
   * - if using SegMMU, m_tot_tcb_cnt will be extended accordingly.
   *
   * multiple BSS and tcb chain format:
   *   BSS0: [1 init-tcb + n task-tcb/no grid done interrupt] +
   *   BSS1: [1 init-tcb + n task-tcb/no grid done interrupt] +
   *   + ... +
   *   BSSx: [1 init-tcb + n task-tcb/grid done interrupt]
   *
   * - for LLM model which uses multiple BSS to describe subgraphs set, the
   * subgraphs refer to one BSS will share a common ASID for their weight data.
   * so each BSS may describe a set of subgraphs' weight buffers which locate in
   * seperate ASID region. finally subgraphs(tcb groups) belong to one BSS
   * descripter will start with an init-tcb with specific ASID_1 base address.
   */
  m_segmmu_tcb_num = core_cnt;
  if ((m_segmmu_num != 0) || !m_same_asid)
    m_segmmu_tcb_skip = (m_segmmu_tcb_num + 1) / 2;

  m_tot_tcb_cnt = m_sg_cnt * m_task_per_sg + get_graph().get_bss_cnt();
  if (m_segmmu_num > 0 || !m_same_asid)
    m_tot_tcb_cnt += get_graph().get_bss_cnt() * ((m_segmmu_tcb_num + 1) / 2);

  m_backup_tcb.reset(new char[m_tot_tcb_cnt * sizeof(tcb_t)]);

  /**
   * case: only disable GM for multiple core NPU when multiple small models
   *       parallel to run on separate core.
   */
  m_gm->gm_dynamic_switch(core_cnt);
}

void JobV3::setup_gm_sync_from_ddr(tcb_t &tcb) {
  GM_V3 *gm_v3 = reinterpret_cast<GM_V3 *>(m_gm);
  uint32_t gm_region_idx = 0;

  if (!m_mem->is_gm_enable())
    return;

  if (!gm_v3->gm_need_remap())
    return;

  if (!m_mem->is_both_gm_region_enable() && m_qos == AIPU_JOB_QOS_HIGH)
    return;

  if (m_qos == AIPU_JOB_QOS_SLOW)
    gm_region_idx = 0;
  else if (m_qos == AIPU_JOB_QOS_HIGH)
    gm_region_idx = 1;

  if (m_mem->is_both_gm_region_enable())
    tcb.gm_ctl = GM_CTRL_REMAP_BOTH_REGION_EN;
  else
    tcb.gm_ctl = GM_CTRL_REMAP_REGION0_EN;

  tcb.gm_rgnx_addr[0].v64 = 0;
  tcb.gm_rgnx_addr[1].v64 = 0;
  tcb.gm_rgnx_addr[gm_region_idx].v64 = gm_v3->m_gm_map_base;

  tcb.gm_rgnx_ctrl[0] = GM_REGION_CTRL_IGNORE_CFG;
  tcb.gm_rgnx_ctrl[1] = GM_REGION_CTRL_IGNORE_CFG;
  if (gm_v3->m_gm_map_base != 0)
    tcb.gm_rgnx_ctrl[gm_region_idx] = 0;

  if (gm_v3->m_gm_sync_buf_size[EM_GM_BUF_INPUT] != 0) {
    tcb.gm_rgnx_ctrl[gm_region_idx] = GM_REGION_CTRL_SYNC_TO_GM;
    DEV_PA_64 offset =
        gm_v3->m_gm_sync_buf_base[EM_GM_BUF_INPUT] - gm_v3->m_gm_map_base;
    tcb.gm_rgnx_ctrl[gm_region_idx] |= get_low_32(offset >> 12) & 0xffff000;
    tcb.gm_rgnx_ctrl[gm_region_idx] |=
        (gm_v3->m_gm_sync_buf_size[EM_GM_BUF_INPUT] >> 12) & 0xfff;
  }
}

void JobV3::setup_gm_sync_to_ddr(tcb_t &tcb) {
  uint32_t gm_region_idx = 0;
  tcb_t pre_tcb = {0};
  GM_V3 *gm_v3 = reinterpret_cast<GM_V3 *>(m_gm);

  if (!m_mem->is_gm_enable())
    return;

  if (!gm_v3->gm_need_sync_out())
    return;

  if (m_qos == AIPU_JOB_QOS_SLOW)
    gm_region_idx = 0;
  else if (m_qos == AIPU_JOB_QOS_HIGH) {
    if (m_mem->is_both_gm_region_enable())
      gm_region_idx = 1;
    else
      return;
  }

  /* modify the last task tcb and link to GM sync tcb */
  m_mem->read(m_init_tcb.pa + sizeof(tcb_t) * (m_tot_tcb_cnt - 2), &pre_tcb,
              sizeof(tcb_t));
  pre_tcb.next =
      get_low_32(m_init_tcb.pa + sizeof(tcb_t) * (m_tot_tcb_cnt - 1));
  pre_tcb.flag &= ~(TCB_FLAG_END_TYPE_GROUP_END | TCB_FLAG_END_TYPE_GRID_END |
                    TCB_FLAG_END_TYPE_END_WITH_DESTROY);
  m_mem->write(m_init_tcb.pa + sizeof(tcb_t) * (m_tot_tcb_cnt - 2),
               (const char *)&pre_tcb, sizeof(tcb_t));

  /* config GM sync tcb: GM->DDR */
  tcb.next = 0;
  tcb.flag = TCB_FLAG_DEP_TYPE_PRE_ALL | TCB_FLAG_END_TYPE_GROUP_END |
             TCB_FLAG_END_TYPE_GRID_END;
  tcb.igrid_id = pre_tcb.gridid;
  tcb.gm_rgnx_addr[0].v64 = 0;
  tcb.gm_rgnx_addr[1].v64 = 0;
  tcb.gm_rgnx_addr[gm_region_idx].v64 = gm_v3->m_gm_map_base;
  tcb.gm_rgnx_ctrl[0] = GM_REGION_CTRL_IGNORE_CFG;
  tcb.gm_rgnx_ctrl[1] = GM_REGION_CTRL_IGNORE_CFG;
  tcb.gm_rgnx_ctrl[gm_region_idx] = GM_REGION_CTRL_SYNC_TO_DDR;
  tcb.gm_rgnx_ctrl[gm_region_idx] |=
      get_low_32(gm_v3->m_gm_sync_buf_base[EM_GM_BUF_OUTPUT] -
                 gm_v3->m_gm_map_base) &
      0xffff000;
  tcb.gm_rgnx_ctrl[gm_region_idx] |=
      (gm_v3->m_gm_sync_buf_size[EM_GM_BUF_OUTPUT] >> 12) & 0xfff;
  tcb.gm_ctl = GM_CTRL_TSM_IGNORE_CFG;

  m_mem->write(m_init_tcb.pa + sizeof(tcb_t) * (m_tot_tcb_cnt - 1),
               (const char *)&tcb, sizeof(tcb_t));
}

#define SEGMMU_MEM_CTRL_EN (1 << 0)
#define SEGMMU_REMAP_EN (1 << 4)
#define SEGMMU_REMAP_SHARE_EN (1 << 5)
#define SEGMMU_IN_ASID_WR (1 << 0)
#define SEGMMU_IN_ASID_RD (1 << 1)
aipu_status_t JobV3::setup_segmmu(SubGraphTask &sg_task) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  SegMMUConfig *segmmu = nullptr;
  union segmmu_id {
    uint32_t id;
    struct {
      uint32_t segmmu_ctrl_idx : 8;
      uint32_t segmmu_idx : 8;
      uint32_t core_id_mask : 16;
    };
  };

  if (m_segmmu_num == 0)
    goto out;

  segmmu = (SegMMUConfig *)get_graph().m_bsegmmu.va;
  for (uint32_t i = 0; i < m_core_cnt; i++) {
    if (m_segmmu_num != 1)
      segmmu++;

    segmmu->SegMMU_ctl = SEGMMU_REMAP_SHARE_EN | SEGMMU_MEM_CTRL_EN;
    segmmu->SegMMU_remap = 0;

    m_segmmu_sec.push_back(*segmmu);
  }

  for (auto &iobuf : m_segmmus) {
    segmmu_id s_id = {.id = iobuf.id};

    if ((s_id.core_id_mask & ((1 << m_core_cnt) - 1)) == 0) {
      LOG(LOG_ERR,
          "Segmmu core idx invalid, (core_id, seg_idx, ctrl_idx): (%x, %d, "
          "%d)\n",
          s_id.core_id_mask, s_id.segmmu_idx, s_id.segmmu_ctrl_idx);
      goto out;
    }

    for (uint32_t core_idx = 0; core_idx < m_core_cnt; core_idx++) {
      if (!(s_id.core_id_mask & (1 << core_idx)))
        continue;

      if (s_id.segmmu_idx >= 0 && s_id.segmmu_idx < 4) {
        if (s_id.segmmu_ctrl_idx >= 0 && s_id.segmmu_ctrl_idx <= 1) {
          uint32_t ctrl = m_segmmu_sec[core_idx]
                              .seg[s_id.segmmu_idx]
                              .control[s_id.segmmu_ctrl_idx];
          ctrl &= 0x3fff;
          ctrl |= (iobuf.pa & (~0x3fff));
          m_segmmu_sec[core_idx]
              .seg[s_id.segmmu_idx]
              .control[s_id.segmmu_ctrl_idx] = ctrl;
        } else {
          LOG(LOG_ERR,
              "Segmmu ctrl idx invalid, (core_id, seg_idx, ctrl_idx): (%x, %d, "
              "%d)\n",
              core_idx, s_id.segmmu_idx, s_id.segmmu_ctrl_idx);
          goto out;
        }
      } else {
        LOG(LOG_ERR,
            "Segmmu seg idx invalid, (core_id, seg_idx, ctrl_idx): (%x, %d, "
            "%d)\n",
            core_idx, s_id.segmmu_idx, s_id.segmmu_ctrl_idx);
        goto out;
      }
    }
  }

out:
  return ret;
}

void JobV3::get_tcb_head_cnt(uint32_t sg_idx, uint32_t &head_cnt) {
  static uint32_t cur_bss_idx = get_graph().get_subgraph(0).bss_idx;

  if ((sg_idx == 0) ||
      (cur_bss_idx != get_graph().get_subgraph(sg_idx).bss_idx)) {
    cur_bss_idx = get_graph().get_subgraph(sg_idx).bss_idx;
    head_cnt += 1;
    head_cnt += m_segmmu_tcb_skip;
  }
}

aipu_status_t JobV3::setup_task_tcb(uint32_t sg_id, uint32_t grid_id,
                                    uint32_t core_id, uint32_t task_id,
                                    bool is_new_grid) {
  GraphV3X &graph = get_graph();
  Task &task = m_sg_job[sg_id].tasks[task_id];
  tcb_t tcb;
  TCB *next_tcb = nullptr;

  if (task_id != (m_task_per_sg - 1))
    next_tcb = &m_sg_job[sg_id].tasks[task_id + 1].tcb;
  else if (sg_id != (m_sg_cnt - 1))
    next_tcb = &m_sg_job[sg_id + 1].tasks[0].tcb;
  else
    next_tcb = nullptr;

  memset(&tcb, 0, sizeof(tcb_t));
  tcb.flag = TCB_FLAG_TASK_TYPE_TASK;
  tcb.interrupt = EN_INTERRUPT_ALL_TYPE;

  if (task_id == (m_task_per_sg - 1)) {
    tcb.flag |= TCB_FLAG_END_TYPE_GROUP_END;
    if (!next_tcb && m_sg_cnt == 1)
      tcb.interrupt |= EN_INTERRUPT_CORE;
  }

  if (next_tcb != nullptr) {
    if ((get_graph().get_bss_cnt() > 1) && is_new_grid &&
        (task_id == (m_task_per_sg - 1)))
      tcb.next =
          get_low_32(next_tcb->pa - (1 + m_segmmu_tcb_skip) * sizeof(tcb_t));
    else
      tcb.next = get_low_32(next_tcb->pa);
  } else {
    tcb.next = 0;
    tcb.flag |= TCB_FLAG_END_TYPE_GRID_END;
    tcb.interrupt |= EN_INTERRUPT_CLUSTER;

#ifndef SIMULATION
    if (m_sg_cnt == 1)
      tcb.flag &= ~TCB_FLAG_END_TYPE_GRID_END;
#endif
  }

  /* It is assumed that subgraphs are topology sorted. */
  if (task_id == 0) {
    switch (graph.get_subgraph(sg_id).precursor_cnt) {
    case SUBG_DEPEND_NONE:
      tcb.flag |= TCB_FLAG_DEP_TYPE_NONE;
      break;
    case SUBG_DEPEND_IMMEDIATE:
      tcb.flag |= TCB_FLAG_DEP_TYPE_IMMEDIATE;
      break;
    case SUBG_DEPEND_PREALL:
      tcb.flag |= TCB_FLAG_DEP_TYPE_PRE_ALL;
      break;
    default:
      LOG(LOG_ERR, "subgraph %u, precursor_cnt=%d\n", sg_id,
          graph.get_subgraph(sg_id).precursor_cnt);
      return AIPU_STATUS_ERROR_INVALID_GBIN;
    }

    /* TODO: multiple bss 1core parallel */
    /* to parallel model which runs only on single core, unset depend-all flag
     */
    if (m_sg_cnt == 1)
      tcb.flag &= ~TCB_FLAG_DEP_TYPE_PRE_ALL;
  }

  tcb.spc = get_low_32(graph.m_text->align_asid_pa +
                       graph.get_subgraph(sg_id).text.offset);
  tcb.gridid = (uint16_t)grid_id;
  tcb.groupid = (uint16_t)core_id;
  tcb.taskid = (uint16_t)task_id;
  tcb.grid_dim_x = 1;
  tcb.grid_dim_y = 1;
  tcb.grid_dim_z = 1;
  tcb.group_dim_x = m_task_per_sg;
  tcb.group_dim_y = 1;
  tcb.group_dim_z = 1;
  tcb.group_id_x = 1;
  tcb.group_id_y = 0;
  tcb.group_id_z = 0;
  tcb.task_id_x = (uint16_t)task_id;
  tcb.task_id_y = 0;
  tcb.task_id_z = 0;
  tcb.tcbp = get_low_32(task.tcb.pa - m_tcbs->asid_base);
  tcb.sp = get_low_32(task.stack->align_asid_pa);
  tcb.pp = get_low_32(m_rodata->align_asid_pa +
                      graph.get_subgraph(sg_id).rodata.offset);
  tcb.dp = get_low_32(task.private_data->align_asid_pa);

  /* const rodata */
  if (graph.m_crodata != nullptr && graph.m_crodata->size > 0)
    tcb.cp = get_low_32(graph.m_crodata->align_asid_pa);

  /* update profile buffer offset according to subgraph index */
  if (m_profiler.size() > 0) {
    tcb.pprofiler = get_low_32(m_profiler[0].align_asid_pa +
                               graph.get_subgraph(sg_id).profiler_buf_size);
    tcb.pprofiler_start = m_profiler[0].align_asid_pa;
  }

  tcb.pcoredump = 0;
  if (m_coredump && m_coredump->is_initialized()) {
    auto &tec_buffer = m_coredump->get_tecs_buf();
    uint32_t buf_idx = core_id * m_task_per_sg + task_id;
    tcb.pcoredump = tec_buffer.at(buf_idx).first | 0x29A;
  }

  if (graph.get_subgraph(sg_id).printfifo_size > 0) {
    uint32_t pa =
        m_pprint->align_asid_pa + AIPU_PAGE_SIZE * sg_id + 1024 * task_id;
    tcb.pprint = get_low_32(pa);
    if (m_hw_cfg->enable_tec_done_irq)
      tcb.interrupt |= EN_INTERRUPT_TEC;
  }

  if (get_graph().is_dynamic_shape() && m_dyn_shape->is_set_dyn_shape_true() &&
      m_dyn_shape->get_config_shape_sz() > 0)
    tcb.global_param = get_low_32(m_model_global_param->align_asid_pa);

  /* flush TCB to AIPU mem */
  m_mem->write(task.tcb.pa, (const char *)&tcb, sizeof(tcb_t));

  return AIPU_STATUS_SUCCESS;
}

aipu_status_t JobV3::setup_tcb_group(uint32_t sg_id, uint32_t grid_id,
                                     uint32_t core_id, bool is_new_grid) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;

  /* setup task TCBs */
  for (uint32_t t = 0; t < m_task_per_sg; t++) {
    ret = setup_task_tcb(sg_id, grid_id, core_id, t, is_new_grid);
    if (ret != AIPU_STATUS_SUCCESS)
      return ret;
  }

  return ret;
}

aipu_status_t JobV3::config_tcb_smmu(DEV_PA_64 init_tcb_pa) {
  tcb_t tcb;

  if (m_segmmu_num > 0) {
    for (uint32_t i = 0; i < m_segmmu_tcb_num; i++) {
      SegMMUConfig &segmmu = m_segmmu_sec[i];

      if (i % 2 == 0) {
        memset(&tcb, 0, sizeof(tcb_t));
        tcb.smmu.ctrl = segmmu.SegMMU_ctl;
        tcb.smmu.remap = segmmu.SegMMU_remap;

        for (int j = 0; j < 4; j++) {
          tcb.smmu.segs[j].ctrl0 = segmmu.seg[j].control[0];
          tcb.smmu.segs[j].ctrl1 = segmmu.seg[j].control[1];
        }

        /* handle the last one segmmu config */
        if (i == m_segmmu_tcb_num - 1) {
          m_mem->write(init_tcb_pa + (1 + i / 2) * sizeof(tcb_t),
                       (const char *)&tcb, sizeof(tcb_t));
          break;
        }
      } else {
        tcb.next_core_smmu.ctrl = segmmu.SegMMU_ctl;
        tcb.next_core_smmu.remap = segmmu.SegMMU_remap;

        for (int j = 0; j < 4; j++) {
          tcb.next_core_smmu.segs[j].ctrl0 = segmmu.seg[j].control[0];
          tcb.next_core_smmu.segs[j].ctrl1 = segmmu.seg[j].control[1];
        }

        m_mem->write(init_tcb_pa + (1 + i / 2) * sizeof(tcb_t),
                     (const char *)&tcb, sizeof(tcb_t));
      }
    }
  } else if ((m_segmmu_num == 0) && !m_same_asid) {
    for (uint32_t i = 0; i < m_segmmu_tcb_num; i++) {
      if (i % 2 == 0) {
        memset(&tcb, 0, sizeof(tcb_t));
        tcb.smmu.ctrl =
            SEGMMU_REMAP_SHARE_EN | SEGMMU_REMAP_EN | SEGMMU_MEM_CTRL_EN;

        /* handle the last one segmmu config */
        if (i == m_segmmu_tcb_num - 1) {
          m_mem->write(init_tcb_pa + (1 + i / 2) * sizeof(tcb_t),
                       (const char *)&tcb, sizeof(tcb_t));
          break;
        }
      } else {
        tcb.next_core_smmu.ctrl =
            SEGMMU_REMAP_SHARE_EN | SEGMMU_REMAP_EN | SEGMMU_MEM_CTRL_EN;
        m_mem->write(init_tcb_pa + (1 + i / 2) * sizeof(tcb_t),
                     (const char *)&tcb, sizeof(tcb_t));
      }
    }
  }

  return AIPU_STATUS_SUCCESS;
}

aipu_status_t JobV3::setup_tcb_chain() {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  tcb_t tcb;
  uint32_t core_id = 0;
  uint32_t cur_bss_idx = get_graph().get_subgraph(0).bss_idx;
  DEV_PA_64 next_init_tcb_pa = m_init_tcb.pa;
  uint32_t init_tcb_cnt = 0;
  bool is_new_grid = false;
  uint32_t tmp_segmmu_tcb_skip = 0;
  uint16_t cur_grid_id = m_grid_id;

  for (uint32_t i = 0; i < get_graph().get_subgraph_cnt(); i++) {
    /**
     * currently a model may contain multiple subgraphs, those subgraphs
     * maybe refer to distinct BSS which describe weight buffers that locate
     * in sepearate ASID region. so subgraphs refering to one BSS construct
     * a grid or a sub-tcb chain with its own init tcb.
     *
     */
    if ((i == 0) || (cur_bss_idx != get_graph().get_subgraph(i).bss_idx)) {
      cur_bss_idx = get_graph().get_subgraph(i).bss_idx;
      init_tcb_cnt += 1;
      tmp_segmmu_tcb_skip += m_segmmu_tcb_skip;
      is_new_grid = true;
      cur_grid_id = m_grid_id;

      /* 1. setup init TCB */
      memset(&tcb, 0, sizeof(tcb_t));

      if (m_segmmu_tcb_skip == 1)
        tcb.flag = (1 << 16) | TCB_FLAG_TASK_TYPE_INIT;
      else if (m_segmmu_tcb_skip == 2)
        tcb.flag = (3 << 16) | TCB_FLAG_TASK_TYPE_INIT;
      else
        tcb.flag = TCB_FLAG_TASK_TYPE_INIT;

      tcb.next = get_low_32(m_sg_job[i].tasks[0].tcb.pa);
      tcb.igrid_id = (cur_grid_id << 16);
      if (m_dev->get_grid_id(m_grid_id) < 0) {
        ret = AIPU_STATUS_ERROR_ALLOC_GRIP_ID;
        return ret;
      }

      /* 1.1 config GM if need */
      if (i == 0)
        setup_gm_sync_from_ddr(tcb);

      /**
       * 1.2 config ASID for each GRID
       * #reserved[8:11], dtcm_en[7], rd_en[6], wr_en[5], size[0:4]
       * the ASID ctrl bits locate at low 12 bits in lo address.
       */

      /**
       * ASID0: feature map buffer region
       * the whole graph share one copy of reuse buffer for feature map.
       */
      tcb.asids[0].v32.lo =
          get_low_32(m_mem->get_asid_base(0) | ASID_RD | ASID_WR);
      tcb.asids[0].v32.hi = get_high_32(m_mem->get_asid_base(0));

      /**
       * ASID1: weight buffer region
       * if LLM model contains multiple BSSs, each BSS will locate in private
       * ASID1 region. so here, set ASID1 base register from weight buffer's
       * asid_base(pa).
       */
      if (get_graph().get_weight_buffer_info().size() > cur_bss_idx) {
        DEV_PA_64 asid1_base =
            get_graph().get_weight_buffer_info()[cur_bss_idx].wb_asid_base;
        tcb.asids[1].v32.lo = get_low_32(asid1_base | ASID_RD | ASID_WR);
        tcb.asids[1].v32.hi = get_high_32(asid1_base);
      } else {
        tcb.asids[1].v32.lo =
            get_low_32(m_mem->get_asid_base(1) | ASID_RD | ASID_WR);
        tcb.asids[1].v32.hi = get_high_32(m_mem->get_asid_base(1));
      }

      /**
       * ASID2 & ASID3: default 0, not use
       */
      for (uint32_t j = 2; j < 3; j++) {
        tcb.asids[j].v32.lo = 0;
        tcb.asids[j].v32.hi = 0;
      }
      m_mem->write(next_init_tcb_pa, (const char *)&tcb, sizeof(tcb_t));

      /* 1.3 config SegMMU if need */
      config_tcb_smmu(next_init_tcb_pa);
    }

    /* 2. setup TCB group */
    ret = setup_tcb_group(get_graph().get_subgraph(i).id, cur_grid_id, core_id,
                          is_new_grid);
    if (ret != AIPU_STATUS_SUCCESS)
      return ret;

    if (++core_id >= m_core_cnt)
      core_id = 0;

    is_new_grid = false;
    next_init_tcb_pa = m_init_tcb.pa + (init_tcb_cnt + tmp_segmmu_tcb_skip +
                                        (i + 1) * m_task_per_sg) *
                                           sizeof(tcb_t);
  }

  /**
   * store aligned TEXT and RO base at tail of text buffer for debugger
   */
  m_mem->write(get_graph().m_text->pa + get_graph().m_btext.size,
               &get_graph().m_text->align_asid_pa, 4);
  m_mem->write(get_graph().m_text->pa + get_graph().m_btext.size + 4,
               &m_rodata->align_asid_pa, 4);

  // setup_gm_sync_to_ddr(tcb);
  m_status = AIPU_JOB_STATUS_INIT;

  return AIPU_STATUS_SUCCESS;
}

aipu_status_t JobV3::dump_for_emulation() {
  if (m_dump_emu == false)
    return AIPU_STATUS_SUCCESS;

  constexpr uint32_t INIT_NUM = 3;
  DEV_PA_64 dump_pa = 0;
  uint32_t dump_size = 0;
  char dump_name[4096] = {0};
  int emu_input_cnt =
      INIT_NUM + m_inputs.size() + (m_descriptor != nullptr ? 1 : 0);
  int emu_output_cnt = m_outputs.size();
  int file_id = -1;
  tcb_t tcb = {0};
  bool default_output_prefix = true;
  std::string runtime_cfg = m_dump_dir + "/runtime.cfg";
  std::string metadata_txt = m_dump_dir + "/metadata.txt";
  std::map<uint32_t, std::string> gm_info = {
      {512 << 10, "512K"}, {1 << 20, "1M"},   {2 << 20, "2M"},
      {4 << 20, "4M"},     {8 << 20, "8M"},   {16 << 20, "16M"},
      {32 << 20, "32M"},   {64 << 20, "64M"},
  };

  FileWrapper ofs(runtime_cfg, std::ios_base::in | std::ios_base::out |
                                   std::ios_base::trunc);
  FileWrapper ofsmt(metadata_txt, std::ios_base::in | std::ios_base::out |
                                      std::ios_base::trunc);
  if (!ofs.is_open() || !ofsmt.is_open())
    return AIPU_STATUS_ERROR_OPEN_FILE_FAIL;

  ofs << "[COMMON]\n";

  /* runtime.cfg: config */
  ofs << "#configuration 1:X2_1204 2:X2_1204MP3\n";
  if (m_dev->get_config_code() != nullptr)
    ofs << "CONFIG=" << m_dev->get_config_code() << "\n";

  /* runtime.cfg: enable_avx */
  ofs << "#if ENABLE_AVX is true then using the intel SIMD instructions to "
         "speedup.\n";
  if (m_cfg->enable_avx)
    ofs << "ENABLE_AVX=true\n";
  else
    ofs << "ENABLE_AVX=false\n";

  /* runtime.cfg: log file path */
  ofs << "#Where log output to store is.\n";
  ofs << "LOG_FILEPATH=" << m_cfg->log_file_path << "\n";

  /* runtime.cfg: log_level */
  ofs << "#which level is your selected: 0:ERROR, 1: WARN, 2: INFO, 3: DEBUG\n";
  ofs << "LOG_LEVEL=" << m_cfg->log_level << "\n";

  /* runtime.cfg: verbose */
  ofs << "#if LOG_VERBOSE is true then print log to console. otherwise no\n";
  if (m_cfg->verbose)
    ofs << "LOG_VERBOSE=true\n";
  else
    ofs << "LOG_VERBOSE=false\n";

  /* runtime.cfg: enable_calloc */
  ofs << "#if ENABLE_CALLOC is true the allocation memory is set to zero.\n";
  if (m_cfg->enable_calloc)
    ofs << "ENABLE_CALLOC=true\n";
  else
    ofs << "ENABLE_CALLOC=false\n";

  /* runtime.cfg: gm_size */
  ofs << "#GM support: 512KiB,1MiB,2MiB,4MiB,8MiB,16MiB,32MiB,64MiB.\n";
  if (gm_info.count(m_cfg->gm_size) == 1)
    ofs << "GM_SIZE=" << gm_info[m_cfg->gm_size] << "\n";

  if (m_cfg->plugin_name != nullptr) {
    ofs << "#PLUGIN_FILENAME\n";
    ofs << "PLUGIN_FILENAME=" << m_cfg->plugin_name << "\n";
  }

  if (m_cfg->json_filename != nullptr) {
    ofs << "#JSON_FILENAME\n";
    ofs << "JSON_FILENAME=" << m_cfg->json_filename << "\n";
  }

  /* runtime.cfg: en_eval */
  ofs << "\n[PROFILE]\n";
  if (m_cfg->en_eval)
    ofs << "EN_EVAL=1\n";
  else
    ofs << "EN_EVAL=0\n";

  if (m_profiler.size() == 1) {
    ofs << "PROFILE_BUF_ADDR=0x" << std::hex << m_profiler[0].pa << "\n";
    ofs << "PROFILE_BUF_SIZE=0x" << std::hex << m_profiler[0].size << "\n";
  }
  ofs << "\n";

  ofs.dump_to_string(m_dumpcfg_header);

  /* runtime.cfg: [INPUT] */
  for (uint32_t bss_id = 0;
       bss_id < get_graph().get_weight_buffer_info().size(); bss_id++) {
    if (get_graph().get_weight_buffer_info()[bss_id].wb_weight != nullptr &&
        get_graph().get_weight_buffer_info()[bss_id].wb_weight->size > 0) {
      emu_input_cnt += 1;
      if (get_graph().get_weight_buffer_info()[bss_id].wb_zerocpy_const !=
              nullptr &&
          get_graph().get_weight_buffer_info()[bss_id].wb_zerocpy_const->size !=
              0)
        emu_input_cnt += 1;
    } else
      emu_input_cnt +=
          get_graph().get_weight_buffer_info()[bss_id].wb_weights.size();
  }

  ofs << "[INPUT]\n";
  ofs << "COUNT=" << emu_input_cnt << "\n";

  /* dump temp.text */
  dump_pa = get_graph().m_text->pa;
  dump_size = get_graph().m_btext.size;
  if (dump_size != 0) {
    snprintf(dump_name, 128, "%s/%s.text", m_dump_dir.c_str(),
             m_dump_prefix.c_str());
    m_mem->dump_file(dump_pa, dump_name, dump_size);

    ofs << "FILE" << std::dec << ++file_id << "=" << m_dump_prefix << ".text\n";
    ofs << "BASE" << file_id << "=0x" << std::hex << dump_pa << "\n";
    m_dumpcfg_input.push_back({dump_name, dump_pa});
  }

  /* dump temp.weight */
  for (uint32_t bss_id = 0;
       bss_id < get_graph().get_weight_buffer_info().size(); bss_id++) {
    if (get_graph().get_weight_buffer_info()[bss_id].wb_weight != nullptr &&
        get_graph().get_weight_buffer_info()[bss_id].wb_weight->req_size > 0) {
      dump_pa = get_graph().get_weight_buffer_info()[bss_id].wb_weight->pa;
      dump_size =
          get_graph().get_weight_buffer_info()[bss_id].wb_weight->req_size;
      if (dump_size != 0) {
        snprintf(dump_name, 128, "%s/%s.weight%d", m_dump_dir.c_str(),
                 m_dump_prefix.c_str(), bss_id);
        m_mem->dump_file(dump_pa, dump_name, dump_size);

        ofs << "FILE" << std::dec << ++file_id << "=" << m_dump_prefix
            << ".weight" << bss_id << "\n";
        ofs << "BASE" << file_id << "=0x" << std::hex << dump_pa << "\n";
        m_dumpcfg_input.push_back({dump_name, dump_pa});

        if (get_graph().get_weight_buffer_info()[bss_id].wb_zerocpy_const !=
                nullptr &&
            get_graph()
                    .get_weight_buffer_info()[bss_id]
                    .wb_zerocpy_const->size > 0) {
          dump_pa =
              get_graph().get_weight_buffer_info()[bss_id].wb_zerocpy_const->pa;
          dump_size = get_graph()
                          .get_weight_buffer_info()[bss_id]
                          .wb_zerocpy_const->req_size;
          snprintf(dump_name, 128, "%s/%s.zerocpy_const%d", m_dump_dir.c_str(),
                   m_dump_prefix.c_str(), bss_id);
          m_mem->dump_file(dump_pa, dump_name, dump_size);

          ofs << "FILE" << std::dec << ++file_id << "=" << m_dump_prefix
              << ".zerocpy_const" << bss_id << "\n";
          ofs << "BASE" << file_id << "=0x" << std::hex << dump_pa << "\n";
          m_dumpcfg_input.push_back({dump_name, dump_pa});
        }
      }
    } else {
      for (uint32_t i = 0;
           i < get_graph().get_weight_buffer_info()[bss_id].wb_weights.size();
           i++) {
        dumpcfg_input_desc input_desc;
        std::string name;

        dump_pa =
            get_graph().get_weight_buffer_info()[bss_id].wb_weights[i]->pa;
        dump_size =
            get_graph().get_weight_buffer_info()[bss_id].wb_weights[i]->size;
        name = m_dump_dir + "/" + m_dump_prefix + ".weight" + std::to_string(i);
        m_mem->dump_file(dump_pa, name.c_str(), dump_size);

        ofs << "FILE" << std::dec << ++file_id << "=" << m_dump_prefix
            << ".weight\n";
        ofs << "BASE" << file_id << "=0x" << std::hex << dump_pa << "\n";
        input_desc.file = name;
        input_desc.base = dump_pa;
        m_dumpcfg_input.push_back(input_desc);
      }
    }
  }

  /* dump temp.rodata */
  dump_pa = m_rodata->pa;
  dump_size = m_rodata->size;
  snprintf(dump_name, 128, "%s/%s.ro", m_dump_dir.c_str(),
           m_dump_prefix.c_str());
  m_mem->dump_file(dump_pa, dump_name, dump_size);
  ofs << "FILE" << std::dec << ++file_id << "=" << m_dump_prefix << ".ro\n";
  ofs << "BASE" << file_id << "=0x" << std::hex << dump_pa << "\n";
  m_dumpcfg_input.push_back({dump_name, dump_pa});

  /* dump temp.dcr */
  if (m_descriptor != nullptr && m_descriptor->size != 0) {
    dump_pa = m_descriptor->pa;
    dump_size = m_descriptor->size;
    snprintf(dump_name, 128, "%s/%s.dcr", m_dump_dir.c_str(),
             m_dump_prefix.c_str());
    m_mem->dump_file(dump_pa, dump_name, dump_size);

    ofs << "FILE" << std::dec << ++file_id << "=" << m_dump_prefix << ".dcr\n";
    ofs << "BASE" << file_id << "=0x" << std::hex << dump_pa << "\n";
    m_dumpcfg_input.push_back({dump_name, dump_pa});
  }

  /* dump temp.tcb */
  dump_pa = m_init_tcb.pa;
  dump_size = m_tot_tcb_cnt * sizeof(tcb_t);
  snprintf(dump_name, 128, "%s/%s.tcb", m_dump_dir.c_str(),
           m_dump_prefix.c_str());
  m_dump_tcb_info[0] =
      std::make_tuple(m_dump_dir + "/init.tcb", dump_pa,
                      (1 + (m_segmmu_tcb_num + 1) / 2) * sizeof(tcb_t));
  m_dump_tcb_info[1] = std::make_tuple(
      m_dump_dir + "/task.tcb", dump_pa + std::get<2>(m_dump_tcb_info[0]),
      dump_size - std::get<2>(m_dump_tcb_info[0]));
  m_mem->dump_file(std::get<1>(m_dump_tcb_info[0]),
                   std::get<0>(m_dump_tcb_info[0]).c_str(),
                   std::get<2>(m_dump_tcb_info[0]));
  m_mem->dump_file(std::get<1>(m_dump_tcb_info[1]),
                   std::get<0>(m_dump_tcb_info[1]).c_str(),
                   std::get<2>(m_dump_tcb_info[1]));
  m_mem->dump_file(dump_pa, dump_name, dump_size);

  ofs << "FILE" << std::dec << ++file_id << "=" << m_dump_prefix << ".tcb\n";
  ofs << "BASE" << file_id << "=0x" << std::hex << dump_pa << "\n";

  /* dump temp.input[n] */
  for (uint32_t i = 0; i < m_inputs.size(); i++) {
    if (m_inputs[i].dump_ignore_flag)
      continue;

    dump_pa = m_inputs[i].pa;
    dump_size = m_inputs[i].size;
    snprintf(dump_name, 128, "%s/%s.input%u", m_dump_dir.c_str(),
             m_dump_prefix.c_str(), i);

    if (m_inputs[i].dmabuf_fd < 0)
      m_mem->dump_file(dump_pa, dump_name, dump_size);
    else
      dump_share_buffer(m_inputs[i], dump_name, true);

    ofs << "FILE" << std::dec << ++file_id << "=" << m_dump_prefix << ".input"
        << i << "\n";
    ofs << "BASE" << file_id << "=0x" << std::hex << dump_pa << "\n";
    m_dumpcfg_input.push_back({dump_name, dump_pa});
  }
  ofs << "\n";

  ofs << "[HOST]\n";
  ofs << "TCBP_HI=0x" << std::hex << get_high_32(m_init_tcb.pa) << "\n";
  ofs << "TCBP_LO=0x" << get_low_32(m_init_tcb.pa) << "\n";
  m_dumpcfg_host = {m_partition_id, get_high_32(m_init_tcb.pa),
                    get_low_32(m_init_tcb.pa)};
  ofs << "\n";

  /* runtime.cfg: [OUTPUT] */
  ofs << "[OUTPUT]\n";
  ofs << "COUNT=" << std::dec << emu_output_cnt << "\n";

  /* dump output.bin[n] */
  if (strncmp(m_dump_output_prefix.c_str(), "temp", 4))
    default_output_prefix = false;

  for (uint32_t i = 0; i < m_outputs.size(); i++) {
    if (m_outputs[i].dump_ignore_flag)
      continue;

    dump_pa = m_outputs[i].pa;
    dump_size = m_outputs[i].size;

    if (default_output_prefix) {
      ofs << "FILE" << std::dec << i << "=" << m_dump_output_prefix << ".output"
          << i << "\n";
      snprintf(dump_name, 128, "%s/%s.output%u", m_dump_dir.c_str(),
               m_dump_prefix.c_str(), i);
      m_dumpcfg_output.push_back({dump_name, dump_pa, dump_size});
    } else {
      if (i == 0) {
        ofs << "FILE" << std::dec << i << "=" << m_dump_output_prefix << "\n";
      } else {
        ofs << "FILE" << std::dec << i << "=" << m_dump_output_prefix << i
            << "\n";
      }
    }

    ofs << "BASE" << std::dec << i << "=0x" << std::hex << dump_pa << "\n";
    ofs << "SIZE" << std::dec << i << "=0x" << std::hex << dump_size << "\n";
  }

  /* close runtime.cfg */
  ofs.close();

  /* dump metadata.txt */
  ofsmt << "Total TCBs Count: " << std::dec << m_tot_tcb_cnt << "\n";
  for (uint32_t i = 0; i < m_tot_tcb_cnt; i++) {
    m_mem->read(m_init_tcb.pa + i * sizeof(tcb_t), &tcb, sizeof(tcb_t));

    if (TCB_FLAG_TASK_TYPE(tcb.flag) == TCB_FLAG_TASK_TYPE_INIT) {
      /* init tcb + segmmu tcb */
      if (tcb.next != 0) {
        ofsmt << "\n***INIT TCB " << std::dec << i << " ***\n";
        ofsmt << "flag: 0x" << std::hex << tcb.flag << "\n";
        ofsmt << "next: 0x" << tcb.next << "\n";

        ofsmt << "GM_CTRL: 0x" << tcb.gm_ctl << "\n";
        ofsmt << "grid_id: " << std::dec << (tcb.igrid_id >> 16) << "\n";
        ofsmt << "GM0_CTRL: 0x" << std::hex << tcb.gm_rgnx_ctrl[0] << "\n";
        ofsmt << "GM1_CTRL: 0x" << tcb.gm_rgnx_ctrl[1] << "\n";
        ofsmt << "GM0_LO: 0x" << tcb.gm_rgnx_addr[0].v32.lo << "\n";
        ofsmt << "GM0_HI: 0x" << tcb.gm_rgnx_addr[0].v32.hi << "\n";
        ofsmt << "GM1_LO: 0x" << tcb.gm_rgnx_addr[1].v32.lo << "\n";
        ofsmt << "GM1_HI: 0x" << tcb.gm_rgnx_addr[1].v32.hi << "\n";
        for (int j = 0; j < 4; j++) {
          ofsmt << "ASID" << std::dec << j << "_LO: 0x" << std::hex
                << tcb.asids[j].v32.lo << "\n";
          ofsmt << "ASID" << std::dec << j << "_HI: 0x" << std::hex
                << tcb.asids[j].v32.hi << "\n";
        }
      } else {
        ofsmt << "\n***INIT SEGMMU TCB " << std::dec << i << " ***\n";
        ofsmt << "SMMU.CTRL: 0x" << std::hex << tcb.smmu.ctrl << "\n";
        ofsmt << "SMMU.REMAP: 0x" << tcb.smmu.remap << "\n";
        for (int j = 0; j < 4; j++) {
          ofsmt << "SMMU.SEGS" << std::dec << j << ".CTRL0: 0x" << std::hex
                << tcb.smmu.segs[j].ctrl0 << "\n";
          ofsmt << "SMMU.SEGS" << std::dec << j << ".CTRL1: 0x" << std::hex
                << tcb.smmu.segs[j].ctrl1 << "\n";
        }

        ofsmt << "NEXT_SMMU.CTRL: 0x" << std::hex << tcb.next_core_smmu.ctrl
              << "\n";
        ofsmt << "NEXT_SMMU.REMAP: 0x" << tcb.next_core_smmu.remap << "\n";
        for (int j = 0; j < 4; j++) {
          ofsmt << "NEXT_SMMU.SEGS" << std::dec << j << ".CTRL0: 0x" << std::hex
                << tcb.next_core_smmu.segs[j].ctrl0 << "\n";
          ofsmt << "NEXT_SMMU.SEGS" << std::dec << j << ".CTRL1: 0x" << std::hex
                << tcb.next_core_smmu.segs[j].ctrl1 << "\n";
        }
      }
    } else if (TCB_FLAG_TASK_TYPE(tcb.flag) == TCB_FLAG_TASK_TYPE_TASK) {
      ofsmt << "\n***TASK TCB " << std::dec << i << "***\n";
      ofsmt << "flag: 0x" << std::hex << tcb.flag << "\n";

      ofsmt << "next: 0x" << tcb.next << "\n";
      ofsmt << "loop_count: " << std::dec << tcb.loop_count << "\n";

      ofsmt << "start_pc: 0x" << std::hex << tcb.spc << "\n";
      ofsmt << "interrupt: 0x" << std::hex << tcb.interrupt << "\n";

      ofsmt << "group_id: " << std::dec << tcb.groupid << "\n";
      ofsmt << "grid_id: " << tcb.gridid << "\n";
      ofsmt << "task_id: " << tcb.taskid << "\n";

      ofsmt << "grid_dim_x: " << tcb.grid_dim_x << "\n";
      ofsmt << "grid_dim_y: " << tcb.grid_dim_y << "\n";
      ofsmt << "grid_dim_z: " << tcb.grid_dim_z << "\n";

      ofsmt << "group_dim_x: " << tcb.group_dim_x << "\n";
      ofsmt << "group_dim_y: " << tcb.group_dim_y << "\n";
      ofsmt << "group_dim_z: " << tcb.group_dim_z << "\n";

      ofsmt << "group_id_x: " << tcb.group_id_x << "\n";
      ofsmt << "group_id_y: " << tcb.group_id_y << "\n";
      ofsmt << "group_id_z: " << tcb.group_id_z << "\n";

      ofsmt << "task_id_x: " << tcb.task_id_x << "\n";
      ofsmt << "task_id_y: " << tcb.task_id_y << "\n";
      ofsmt << "task_id_z: " << tcb.task_id_z << "\n";

      ofsmt << "sp: 0x" << std::hex << tcb.sp << "\n";
      ofsmt << "pp: 0x" << tcb.pp << "\n";
      ofsmt << "dp: 0x" << tcb.dp << "\n";
      ofsmt << "cp: 0x" << tcb.cp << "\n";
      ofsmt << "pprint: 0x" << tcb.pprint << "\n";
      ofsmt << "pprofiler: 0x" << tcb.pprofiler << "\n";
      ofsmt << "fmdp: 0x" << tcb.fmdp << "\n";
      ofsmt << "tap: 0x" << tcb.tap << "\n";
      ofsmt << "dap: 0x" << tcb.dap << "\n";
      ofsmt << "pap: 0x" << tcb.pap << "\n";
      ofsmt << "idp: 0x" << tcb.idp << "\n";
      ofsmt << "dsize: 0x" << tcb.dsize << "\n";
    } else {
      LOG(LOG_ERR, "invalid TCB task type, flag: 0x%x\n", tcb.flag);
    }
  }

  ofsmt << "\n***IO Tensors***\n";
  for (uint32_t i = 0; i < m_inputs.size(); i++) {
    dump_pa = m_inputs[i].pa;
    dump_size = m_inputs[i].size;

    ofsmt << "input" << std::dec << i << "_addr: 0x" << std::hex << dump_pa
          << "\n";
    ofsmt << "input" << std::dec << i << "_size: 0x" << std::hex << dump_size
          << "\n";
  }

  for (uint32_t i = 0; i < m_outputs.size(); i++) {
    dump_pa = m_outputs[i].pa;
    dump_size = m_outputs[i].size;

    ofsmt << "output" << std::dec << i << "_addr: 0x" << std::hex << dump_pa
          << "\n";
    ofsmt << "output" << std::dec << i << "_size: 0x" << std::hex << dump_size
          << "\n";
  }

  ofsmt.dump_to_string(m_dumpcfg_meta);
  /* close metadata.txt */
  ofsmt.close();
  return AIPU_STATUS_SUCCESS;
}

#if defined(SIMULATION)
void JobV3::dumpcfg_alljob() {
  JobV3 *job = nullptr;
  std::vector<uint32_t> cluster_id[4];
  uint32_t count = 0, cmdpool_mask = 0;
  MainContext *ctx = static_cast<MainContext *>(get_graph().m_ctx);
  GraphTable &graphs = ctx->get_graphtable();
  GraphV3X *graph = nullptr;
  std::ostringstream oss;
  SimulatorV3 *sim = static_cast<SimulatorV3 *>(m_dev);
  static bool dump_done = false;
  static std::mutex mtex;

  {
    std::lock_guard<std::mutex> _lock(mtex);
    if (dump_done)
      return;
    dump_done = true;
  }

  if (!m_dump_emu)
    return;

  FileWrapper ofs("./runtime.cfg", std::ios::out);
  FileWrapper ofsmt("./metadata.txt", std::ios::out);
  if (!ofs.is_open() || !ofsmt.is_open())
    return;

  /**
   * set destroy flag for each tcbchain in all cmdpool, the simulator
   * can know when it's reasonable to set end/idle status for cmdpool.
   */
  sim->set_destroy_to_all_tcbchain();

  /* runtime.cfg: [COMMON] */
  ofs << m_dumpcfg_header << "\n";

  /* runtime.cfg: [INPUT] */
  count = 0;
  auto graph_iter = graphs.begin();
  for (auto g : graphs) {
    graph_iter++;
    graph = static_cast<GraphV3X *>(g.second);
    auto job_iter = graph->m_jobs.begin();
    for (auto item : graph->m_jobs) {
      job_iter++;
      job = static_cast<JobV3 *>(item.second);
      for (uint32_t i = 0; i < job->m_dumpcfg_input.size(); i++) {
        oss << "FILE" << std::dec << count << "="
            << job->m_dumpcfg_input.at(i).file << "\n";
        oss << "BASE" << count << "=0x" << std::hex
            << job->m_dumpcfg_input.at(i).base << "\n";
        count++;
      }

      /* dump finally updated init/task tcb for each job */
      for (int i = 0; i < 2; i++) {
        if (i == 0) {
          m_mem->dump_file(std::get<1>(job->m_dump_tcb_info[i]),
                           std::get<0>(job->m_dump_tcb_info[i]).c_str(),
                           std::get<2>(job->m_dump_tcb_info[i]));
        } else {
          if ((graph_iter != graphs.end()) ||
              (graph_iter == graphs.end() && job_iter != graph->m_jobs.end())) {
            void *addr = nullptr;
            uint64_t size = 0;
            uint32_t offset = (m_tot_tcb_cnt - 1) * sizeof(tcb_t) + 4;

            /**
             * copy the 'next' field from job's last task tcb to task.tcb file's
             * corresponding location.this will connect current job's tcb chain
             * to next job's tcb chain.
             */
            umd_mmap_file_helper(std::get<0>(job->m_dump_tcb_info[i]).c_str(),
                                 &addr, &size);
            m_mem->read(job->get_tcb_head_pa() + offset,
                        (char *)addr + size - 2 * sizeof(tcb_t) + 4, 4);
          }
        }
        oss << "FILE" << std::dec << count << "="
            << std::get<0>(job->m_dump_tcb_info[i]) << "\n";
        oss << "BASE" << count << "=0x" << std::hex
            << std::get<1>(job->m_dump_tcb_info[i]) << "\n";
        count++;
      }
    }
  }
  ofs << "[INPUT]\n";
  ofs << "COUNT=" << count << "\n";
  ofs << oss.str();
  ofs << "\n";

  /* runtime.cfg: [HOST] */
  oss.str("");
  count = 0;
  cmdpool_mask = sim->get_cmdpool_bitmap();
  for (auto g : graphs) {
    graph = static_cast<GraphV3X *>(g.second);
    for (auto item : graph->m_jobs) {
      job = static_cast<JobV3 *>(item.second);
      if (cmdpool_mask & (1 << job->m_bind_cmdpool_id)) {
        oss << "SET_PARTITION" << std::dec << count << "="
            << job->m_dumpcfg_host.part_id << "\n";
        oss << "TCBP_HI" << std::dec << count << "=0x" << std::hex
            << job->m_dumpcfg_host.hi_addr << "\n";
        oss << "TCBP_LO" << std::dec << count << "=0x" << std::hex
            << job->m_dumpcfg_host.lo_addr << "\n";
        count++;
        cmdpool_mask &= ~(1 << job->m_bind_cmdpool_id);
        sim->clear_cmdpool_bitmap(job->m_bind_cmdpool_id);
      }
    }
  }
  ofs << "[HOST]\n";
  ofs << "COUNT=" << count << "\n";
  ofs << oss.str();
  ofs << "\n";

  /* runtime.cfg: [ALLOCATE_PARTITION] */
  count = 0;
  ofs << "[ALLOCATE_PARTITION]\n";
  for (int i = 0; i < 4; i++) {
    m_dev->get_cluster_id(i, cluster_id[i]);
    count += cluster_id[i].size();
  }
  ofs << "COUNT=" << count << "\n";
  for (int part_id = 0; part_id < 4; part_id++) {
    for (auto cluster_id : cluster_id[part_id])
      ofs << "CLUSTER" << std::dec << cluster_id << "=" << part_id << "\n";
  }
  ofs << "\n";

  /* runtime.cfg: [OUTPUT] */
  oss.str("");
  count = 0;
  for (auto g : graphs) {
    graph = static_cast<GraphV3X *>(g.second);
    for (auto item : graph->m_jobs) {
      job = static_cast<JobV3 *>(item.second);
      for (uint32_t i = 0; i < job->m_dumpcfg_output.size(); i++) {
        oss << "FILE" << std::dec << count << "="
            << job->m_dumpcfg_output.at(i).file << "\n";
        oss << "BASE" << std::dec << count << "=0x" << std::hex
            << job->m_dumpcfg_output.at(i).base << "\n";
        oss << "SIZE" << std::dec << count << "=0x" << std::hex
            << job->m_dumpcfg_output.at(i).size << "\n";
        count++;
      }
    }
  }
  ofs << "[OUTPUT]\n";
  ofs << "COUNT=" << std::dec << count << "\n";
  ofs << oss.str();
  ofs << "\n";

  /* clost runtime.cfg */
  ofs.close();

  /* gen metadata.txt */
  for (auto g : graphs) {
    graph = static_cast<GraphV3X *>(g.second);
    for (auto item : graph->m_jobs) {
      job = static_cast<JobV3 *>(item.second);
      ofsmt << job->m_dumpcfg_meta;
      ofsmt << "\n";
    }
    ofsmt << "\n";
  }
  ofsmt.close();
}
#endif

} // namespace aipudrv
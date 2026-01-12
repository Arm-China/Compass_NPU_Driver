// Copyright (C) 2023-2025 Arm Technology (China) Co. Ltd.
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
using namespace tcb_v3;

JobV3::JobV3(MainContext *ctx, GraphBase &graph, DeviceBase *dev,
             aipu_create_job_cfg_t *config)
    : JobV3X(ctx, graph, dev, config) {
  m_gm = new GM_V3(*this);
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
  m_segmmu_tcb_num = (core_cnt + 1) / 2;

  m_tot_tcb_cnt = m_sg_cnt * m_task_per_sg + graph().get_bss_cnt() +
                  graph().get_bss_cnt() * m_segmmu_tcb_num;
  m_backup_tcb.reset(new char[m_tot_tcb_cnt * sizeof(tcb_t)]);

  /**
   * case: only disable GM for multiple core NPU when multiple small models
   *       parallel to run on separate core.
   */
  m_gm->gm_dynamic_switch(core_cnt);
}

/**
 * 1.<v3_2 reuse gm buffer mallocs independently, and >=v3_2 reuse gm buffer is
 * piece of gm buffer 2.other reuse sections mallocs independently
 */
aipu_status_t JobV3::alloc_reuse_buffer() {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  uint32_t pad_size = graph().get_alloc_pad_size();

  /* allocate reuse buffers, all subgraphs share one copy of reuse buffers */
  for (uint32_t k = 0; k < graph().get_bss(0).reuse_sections.size(); k++) {
    const GraphSectionDesc &section_desc = graph().get_bss(0).reuse_sections[k];
    BufferDesc *bufferDesc = nullptr;

    if (section_desc.size != 0) {
      std::string buf_name = "reuse_" + std::to_string(k);

      if (m_gm->is_gm_buffer(k, GM_BUF_TYPE_REUSE)) {
        bufferDesc = new BufferDesc;
        if (graph().get_isa() == ISAv5) {
          buf_name = "gm_" + buf_name;
          ret = m_gm->gm_malloc(0, k, GM_BUF_TYPE_REUSE, buf_name, bufferDesc);
        }
      } else {
        if ((m_sfm_idxes.count(k) == 1) ||
            (m_sfm_mem_region != AIPU_MEM_REGION_DEFAULT))
          ret = m_mem->malloc(section_desc.size + pad_size,
                              section_desc.align_in_page, &bufferDesc,
                              buf_name.c_str(), m_sfm_mem_region);
        else
          ret = m_mem->malloc(section_desc.size + pad_size,
                              section_desc.align_in_page, &bufferDesc,
                              buf_name.c_str(), AIPU_MEM_REGION_DEFAULT);
      }

      if (ret != AIPU_STATUS_SUCCESS) {
        LOG(LOG_ERR, "alloc reuse buffer %d [fail]", k);
        goto out;
      }

      if (m_dump_reuse)
        m_mem->mem_bzero(bufferDesc->pa, bufferDesc->size);

      m_reuses_desc.push_back(bufferDesc);
    } else {
      LOG(LOG_WARN, "reuse %d: size == 0", k);
    }
  }

  if (get_subgraph_cnt() > 0 && graph().get_subgraph(0).printfifo_size > 0) {
    std::string buf_name = "printf";
    ret = m_mem->malloc(get_subgraph_cnt() * AIPU_PAGE_SIZE, 0, &m_pprint,
                        buf_name.c_str());
    if (ret != AIPU_STATUS_SUCCESS)
      goto out;
  }

out:
  return ret;
}

/**
 * destructor will release buffers
 */
aipu_status_t JobV3::alloc_job_buffers() {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;

  m_secbuf_desc[FMSection::Text] = graph().m_text;
  m_secbuf_desc[FMSection::Crodata] = graph().m_crodata;

  const auto &fm_info = graph().get_fmsec_info();
  for (const auto &info : fm_info.info) {
    if (info.second.size == 0)
      continue;

    /* v3 jobs share text/crodata/zerocopy sections */
    if (info.first == FMSection::Text || info.first == FMSection::Crodata ||
        info.first == FMSection::ZcyConst ||
        info.first == FMSection::ReservedIOVA)
      continue;

    std::string name = graph().get_section_name(info.first);
    uint32_t size = info.first == FMSection::TotalReuse
                        ? info.second.size + 0x800
                        : info.second.size;
    ret = m_mem->malloc(size, 0, &m_secbuf_desc[info.first], name.c_str());
    if (ret != AIPU_STATUS_SUCCESS)
      return ret;

    /* free the orginal buffer starting from addr 0x0 */
    if (info.first == FMSection::TcbChain) {
      if (m_secbuf_desc.at(FMSection::TcbChain)->align_asid_pa == 0 ||
          m_secbuf_desc.at(FMSection::TcbChain)->pa == 0) {
        m_mem->free(&m_secbuf_desc.at(FMSection::TcbChain));

        BufferDesc *tcb_rsv = nullptr;
        ret = m_mem->malloc(0x1000, 0, &tcb_rsv, "tcb_rsv");
        if (ret != AIPU_STATUS_SUCCESS)
          return ret;

        ret = m_mem->malloc(size, 0, &m_secbuf_desc[FMSection::TcbChain],
                            name.c_str());
        if (ret != AIPU_STATUS_SUCCESS) {
          m_mem->free(&tcb_rsv);
          return ret;
        }

        m_mem->free(&tcb_rsv);
      }
    }
  }
  return AIPU_STATUS_SUCCESS;
}

aipu_status_t JobV3::free_job_buffers() {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;

  for (auto &desc : m_secbuf_desc) {
    if (desc.first == FMSection::Text || desc.first == FMSection::Crodata ||
        desc.first == FMSection::ZcyConst ||
        desc.first == FMSection::ReservedIOVA)
      continue;

    if (desc.first == FMSection::Dcr && graph().m_put_desc_gm &&
        m_mem->is_gm_enable()) {
      m_mem->free_bufferdesc(&m_descriptor);
      continue;
    } else if (desc.first == FMSection::TotalPriv && graph().m_put_ws_gm &&
               m_mem->is_gm_enable()) {
      m_mem->free_bufferdesc(&m_top_priv_buf);
      continue;
    }

    if (desc.second && desc.second->size != 0)
      m_mem->free(&desc.second, graph().get_section_name(desc.first).c_str());
  }

  for (uint32_t i = 0; i < m_sg_job.size(); i++) {
    free_sg_buffers(m_sg_job[i]);
    m_sg_job[i].reset(i, -1);
  }

  if (m_top_reuse) /* default */
  {
    for (uint32_t i = 0; i < m_reuses_desc.size(); i++) {
      if (m_sfm_idxes.count(i) == 0 ||
          graph().is_gm_buffer(i, GM_BUF_TYPE_REUSE))
        m_mem->free_bufferdesc(&m_reuses_desc[i]);
      else
        m_mem->free(&m_reuses_desc[i]);
    }
  } else /* scattered reuse buffer */
  {
    for (uint32_t i = 0; i < m_reuses_desc.size(); i++) {
      if (m_dmabuf_idxs.count(i) == 1) {
        m_mem->free_bufferdesc(&m_reuses_desc[i]);
        continue;
      }

      m_mem->free(&m_reuses_desc[i]);
    }
  }

  for (uint32_t i = 0; i < m_weight.size(); ++i) {
    if (graph().m_put_weight_gm) {
      if (m_weight[i].wb_weight && m_weight[i].wb_weight->size != 0)
        m_mem->free_bufferdesc(&m_weight[i].wb_weight);

      if (m_weight[i].wb_zerocpy_const &&
          m_weight[i].wb_zerocpy_const->size != 0)
        m_mem->free_bufferdesc(&m_weight[i].wb_zerocpy_const);
    }
  }

  if (m_coredump)
    m_coredump->free_coredump();

  m_init_tcb.init(0);

  m_reuses_desc.clear();

  m_sg_job.clear();

  m_inputs.clear();
  m_outputs.clear();
  m_inter_dumps.clear();
  m_profiler.clear();
  m_printf.clear();
  m_layer_counter.clear();
  return ret;
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

  /* all cores share 1 segmmu configuration if only 1 segmmu info, otherwise,
   * one-to-one correspondence */
  segmmu = (SegMMUConfig *)graph().m_bsegmmu.va;
  for (uint32_t i = 0; i < m_core_cnt; i++) {
    segmmu->SegMMU_ctl = SEGMMU_REMAP_SHARE_EN | SEGMMU_MEM_CTRL_EN;
    segmmu->SegMMU_remap = 0;
    m_segmmu_sec.push_back(*segmmu);

    if (m_segmmu_num != 1) {
      if (m_segmmu_num != m_core_cnt) {
        LOG(LOG_ERR,
            "if segmmu num(%u) is not 1, should be equal to core num(%u)!",
            m_segmmu_num, m_core_cnt);
        goto out;
      }
      segmmu++;
    }
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

aipu_status_t JobV3::setup_task_tcb(uint32_t sg_id, uint32_t grid_id,
                                    uint32_t core_id, uint32_t task_id,
                                    bool new_grid) {
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
  tcb.flag = tcb_ctl::FLAG_TASK_TYPE_TASK_V3;
  tcb.task.interrupt_en = tcb_ctl::EN_INTERRUPT_ALL_TYPE_V3;

  if (task_id == (m_task_per_sg - 1)) {
    tcb.flag |= tcb_ctl::FLAG_END_TYPE_GROUP_END;
    if (!next_tcb && m_sg_cnt == 1)
      tcb.task.interrupt_en |= tcb_ctl::EN_INTERRUPT_CORE;
  }

  if (next_tcb != nullptr) {
    if ((graph().get_bss_cnt() > 1) && new_grid &&
        (task_id == (m_task_per_sg - 1)))
      tcb.next =
          get_low_32(next_tcb->pa - (1 + m_segmmu_tcb_num) * sizeof(tcb_t));
    else
      tcb.next = get_low_32(next_tcb->pa);
  } else {
    tcb.next = 0;
    tcb.flag |= tcb_ctl::FLAG_END_TYPE_GRID_END;
    tcb.task.interrupt_en |= tcb_ctl::EN_INTERRUPT_CLUSTER;

#ifndef SIMULATION
    if (m_sg_cnt == 1)
      tcb.flag &= ~tcb_ctl::FLAG_END_TYPE_GRID_END;
#endif
  }

  /* It is assumed that subgraphs are topology sorted. */
  if (task_id == 0) {
    switch (graph().get_subgraph(sg_id).precursor_cnt) {
    case SUBG_DEPEND_NONE:
      tcb.flag |= tcb_ctl::FLAG_DEP_TYPE_NONE;
      break;
    case SUBG_DEPEND_IMMEDIATE:
      tcb.flag |= tcb_ctl::FLAG_DEP_TYPE_IMMEDIATE;
      break;
    case SUBG_DEPEND_PREALL:
      tcb.flag |= tcb_ctl::FLAG_DEP_TYPE_PRE_ALL;
      break;
    default:
      LOG(LOG_ERR, "subgraph %u, precursor_cnt=%d\n", sg_id,
          graph().get_subgraph(sg_id).precursor_cnt);
      return AIPU_STATUS_ERROR_INVALID_GBIN;
    }

    /* TODO: multiple bss 1core parallel */
    /* to parallel model which runs only on single core, unset depend-all flag
     */
    if (m_sg_cnt == 1)
      tcb.flag &= ~tcb_ctl::FLAG_DEP_TYPE_PRE_ALL;
  }

  tcb.task.spc = get_low_32(graph().m_text->align_asid_pa +
                            graph().get_subgraph(sg_id).text.offset);
  tcb.task.grid_id = (uint16_t)grid_id;
  tcb.task.group_id = (uint16_t)core_id;
  tcb.task.task_id = (uint16_t)task_id;
  tcb.task.grid_dim_x = 1;
  tcb.task.grid_dim_y = 1;
  tcb.task.grid_dim_z = 1;
  tcb.task.group_dim_x = m_task_per_sg;
  tcb.task.group_dim_y = 1;
  tcb.task.group_dim_z = 1;
  tcb.task.group_id_x = 1;
  tcb.task.group_id_y = 0;
  tcb.task.group_id_z = 0;
  tcb.task.task_id_x = (uint16_t)task_id;
  tcb.task.task_id_y = 0;
  tcb.task.task_id_z = 0;
  tcb.task.tcbp = get_low_32(task.tcb.pa - m_tcbs->asid_base);
  tcb.task.sp = get_low_32(task.stack->align_asid_pa);
  tcb.task.pp = get_low_32(m_rodata->align_asid_pa +
                           graph().get_subgraph(sg_id).rodata.offset);
  tcb.task.dp = get_low_32(task.private_data->align_asid_pa);

  /* const rodata */
  if (graph().m_crodata != nullptr && graph().m_crodata->size > 0)
    tcb.task.cp = get_low_32(graph().m_crodata->align_asid_pa);

  /* update profile buffer offset according to subgraph index */
  if (m_profiler.size() > 0) {
    tcb.task.pprofiler =
        get_low_32(m_profiler[0].align_asid_pa +
                   graph().get_subgraph(sg_id).profiler_buf_size);
    tcb.task.rsvd2[2] = m_profiler[0].align_asid_pa;
  }

  tcb.task.rsvd2[1] = 0;
  if (m_coredump && m_coredump->is_initialized()) {
    auto &tec_buffer = m_coredump->get_tecs_buf();
    uint32_t buf_idx = core_id * m_task_per_sg + task_id;
    tcb.task.rsvd2[1] = tec_buffer.at(buf_idx).first | 0x29A;
  }

  if (graph().get_subgraph(sg_id).printfifo_size > 0) {
    uint32_t pa =
        m_pprint->align_asid_pa + AIPU_PAGE_SIZE * sg_id + 1024 * task_id;
    tcb.task.pprint = get_low_32(pa);
    if (m_hw_cfg->enable_tec_done_irq)
      tcb.task.interrupt_en |= tcb_ctl::EN_INTERRUPT_TEC;
  }

  if (graph().is_dynamic_shape())
    tcb.task.global_param = get_low_32(m_global_param->align_asid_pa);

  /* flush TCB to AIPU mem */
  m_mem->write(task.tcb.pa, (const char *)&tcb, sizeof(tcb_t));

  return AIPU_STATUS_SUCCESS;
}

aipu_status_t JobV3::setup_tcb_group(uint32_t sg_id, uint32_t grid_id,
                                     uint32_t core_id, bool new_grid) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;

  /* setup task TCBs */
  for (uint32_t t = 0; t < m_task_per_sg; t++) {
    ret = setup_task_tcb(sg_id, grid_id, core_id, t, new_grid);
    if (ret != AIPU_STATUS_SUCCESS)
      return ret;
  }

  return ret;
}

aipu_status_t JobV3::config_tcb_smmu(DEV_PA_64 init_tcb_pa) {
  tcb_t tcb;

  if (m_segmmu_num > 0) {
    for (uint32_t i = 0; i < m_core_cnt; i++) {
      SegMMUConfig &segmmu = m_segmmu_sec[i];

      if (i % 2 == 0) {
        memset(&tcb, 0, sizeof(tcb_t));
        tcb.group.smmu.ctrl = segmmu.SegMMU_ctl;
        tcb.group.smmu.remap = segmmu.SegMMU_remap;

        for (int j = 0; j < 4; j++) {
          tcb.group.smmu.segs[j].ctrl0 = segmmu.seg[j].control[0];
          tcb.group.smmu.segs[j].ctrl1 = segmmu.seg[j].control[1];
        }

        /* handle the last one segmmu config */
        if (i == m_core_cnt - 1) {
          m_mem->write(init_tcb_pa + (1 + i / 2) * sizeof(tcb_t),
                       (const char *)&tcb, sizeof(tcb_t));
          break;
        }
      } else {
        tcb.group.next_core_smmu.ctrl = segmmu.SegMMU_ctl;
        tcb.group.next_core_smmu.remap = segmmu.SegMMU_remap;

        for (int j = 0; j < 4; j++) {
          tcb.group.next_core_smmu.segs[j].ctrl0 = segmmu.seg[j].control[0];
          tcb.group.next_core_smmu.segs[j].ctrl1 = segmmu.seg[j].control[1];
        }

        m_mem->write(init_tcb_pa + (1 + i / 2) * sizeof(tcb_t),
                     (const char *)&tcb, sizeof(tcb_t));
      }
    }
  } else {
    for (uint32_t i = 0; i < m_core_cnt; i++) {
      if (i % 2 == 0) {
        memset(&tcb, 0, sizeof(tcb_t));
        tcb.group.smmu.ctrl =
            SEGMMU_REMAP_SHARE_EN | SEGMMU_REMAP_EN | SEGMMU_MEM_CTRL_EN;

        /* handle the last one segmmu config */
        if (i == m_core_cnt - 1) {
          m_mem->write(init_tcb_pa + (1 + i / 2) * sizeof(tcb_t),
                       (const char *)&tcb, sizeof(tcb_t));
          break;
        }
      } else {
        tcb.group.next_core_smmu.ctrl =
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
  uint32_t cur_bss_idx = graph().get_subgraph(0).bss_idx;
  DEV_PA_64 next_init_tcb_pa = m_init_tcb.pa;
  uint32_t init_tcb_cnt = 0;
  bool new_grid = false;
  uint32_t segmmu_tcb_nums = 0;
  uint16_t cur_grid_id = m_grid_id;

  for (uint32_t i = 0; i < graph().get_subgraph_cnt(); i++) {
    /**
     * currently a model may contain multiple subgraphs, those subgraphs
     * maybe refer to distinct BSS which describe weight buffers that locate
     * in sepearate ASID region. so subgraphs refering to one BSS construct
     * a grid or a sub-tcb chain with its own init tcb.
     *
     */
    new_grid = false;
    if ((i + 1 < graph().get_subgraph_cnt()) &&
        graph().get_subgraph(i).bss_idx != graph().get_subgraph(i + 1).bss_idx)
      new_grid = true;

    if ((i == 0) || (cur_bss_idx != graph().get_subgraph(i).bss_idx)) {
      cur_bss_idx = graph().get_subgraph(i).bss_idx;
      init_tcb_cnt += 1;
      segmmu_tcb_nums += m_segmmu_tcb_num;
      cur_grid_id = m_grid_id;

      /* 1. setup init TCB */
      memset(&tcb, 0, sizeof(tcb_t));

      if (m_segmmu_tcb_num == 1)
        tcb.flag = (1 << 16) | tcb_ctl::FLAG_TASK_TYPE_INIT;
      else if (m_segmmu_tcb_num == 2)
        tcb.flag = (3 << 16) | tcb_ctl::FLAG_TASK_TYPE_INIT;
      else
        tcb.flag = tcb_ctl::FLAG_TASK_TYPE_INIT;

      tcb.next = get_low_32(m_sg_job[i].tasks[0].tcb.pa);
      tcb.grid.grid_id = (cur_grid_id << 16);
      if (m_dev->get_grid_id(m_grid_id) < 0) {
        ret = AIPU_STATUS_ERROR_ALLOC_GRIP_ID;
        return ret;
      }

      /* 1.1 config GM if need */
      if (i == 0)
        reinterpret_cast<GM_V3 *>(m_gm)->setup_gm_sync_from_ddr(tcb);

      /**
       * 1.2 config ASID for each GRID
       * #reserved[8:11], dtcm_en[7], rd_en[6], wr_en[5], size[0:4]
       * the ASID ctrl bits locate at low 12 bits in lo address.
       */

      /**
       * ASID0: feature map buffer region
       * the whole graph share one copy of reuse buffer for feature map.
       */
      tcb.grid.asids[0].v32.lo = get_low_32(
          m_mem->get_asid_base(0) | tcb_ctl::ASID_RD | tcb_ctl::ASID_WR);
      tcb.grid.asids[0].v32.hi = get_high_32(m_mem->get_asid_base(0));

      /**
       * ASID1: weight buffer region
       * if LLM model contains multiple BSSs, each BSS will locate in private
       * ASID1 region. so here, set ASID1 base register from weight buffer's
       * asid_base(pa).
       */
      DEV_PA_64 asid1_base = 0;
      if (m_weight.size() > cur_bss_idx)
        asid1_base = m_weight[cur_bss_idx].wb_asid_base;
      tcb.grid.asids[1].v32.lo =
          get_low_32(asid1_base | tcb_ctl::ASID_RD | tcb_ctl::ASID_WR);
      tcb.grid.asids[1].v32.hi = get_high_32(asid1_base);

      /**
       * ASID2 & ASID3: default 0, not use
       */
      for (uint32_t j = 2; j < 3; j++) {
        tcb.grid.asids[j].v32.lo = 0;
        tcb.grid.asids[j].v32.hi = 0;
      }
      m_mem->write(next_init_tcb_pa, (const char *)&tcb, sizeof(tcb_t));

      /* 1.3 config SegMMU if need */
      config_tcb_smmu(next_init_tcb_pa);
    }

    /* 2. setup TCB group */
    ret = setup_tcb_group(graph().get_subgraph(i).id, cur_grid_id, core_id,
                          new_grid);
    if (ret != AIPU_STATUS_SUCCESS)
      return ret;

    if (++core_id >= m_core_cnt)
      core_id = 0;

    next_init_tcb_pa = m_init_tcb.pa + (init_tcb_cnt + segmmu_tcb_nums +
                                        (i + 1) * m_task_per_sg) *
                                           sizeof(tcb_t);
  }

  /**
   * store aligned TEXT and RO base at tail of text buffer for debugger
   */
  m_mem->write(m_text->pa + graph().m_btext.size, &m_text->align_asid_pa, 4);
  m_mem->write(m_text->pa + graph().m_btext.size + 4, &m_rodata->align_asid_pa,
               4);

  m_status = AIPU_JOB_STATUS_INIT;

  return AIPU_STATUS_SUCCESS;
}

aipu_status_t JobV3::specify_io(aipu_shared_tensor_info_t &tensor_info,
                                const std::vector<JobIOBuffer> *iobuffer_vec) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  uint64_t buffer_pa = tensor_info.pa;
  int fd = tensor_info.dmabuf_fd;
  uint32_t index = tensor_info.tensor_idx;
  uint64_t offset = tensor_info.offset_in_dmabuf;

  aipu_dma_buf dma_buf{fd, 0, 0};
  uint32_t reuse_index = (*iobuffer_vec)[index].ref_section_iter;
  BufferDesc *bufferDesc = m_reuses_desc[reuse_index];

  switch (tensor_info.shared_case_type) {
  case AIPU_SHARE_BUF_IN_ONE_PROCESS:
    bufferDesc->init(m_mem->get_asid_base(0), buffer_pa, bufferDesc->size,
                     bufferDesc->req_size);
    update_io_buffers(graph().get_bss(0).io, m_reuses_desc);
    break;
  case AIPU_SHARE_BUF_CUSTOMED:
    bufferDesc->init(m_mem->get_asid_base(0), buffer_pa, bufferDesc->size,
                     bufferDesc->req_size);
    (*iobuffer_vec)[index].set_dump_ignore_flag(true);
    break;
  case AIPU_SHARE_BUF_DMABUF:
    ret = convert_ll_status(
        m_dev->ioctl_cmd(AIPU_IOCTL_GET_DMABUF_INFO, &dma_buf));
    if (ret != AIPU_STATUS_SUCCESS)
      return ret;

    buffer_pa = dma_buf.pa + offset;
    bufferDesc->init(m_mem->get_asid_base(0), buffer_pa, bufferDesc->size,
                     bufferDesc->req_size);
    (*iobuffer_vec)[index].set_dmabuf_info(fd, dma_buf.bytes, offset);
    break;
  case AIPU_SHARE_BUF_ATTACH_DMABUF:
    LOG(LOG_ERR, "v3 doesn't support dmabuf attach");
    return AIPU_STATUS_ERROR_INVALID_CONFIG;
  default:
    return AIPU_STATUS_ERROR_INVALID_OP;
  }

  LOG(LOG_DEBUG, "specify_io_buffer: pa=%lx, size=%lx, share_case_type=%d",
      buffer_pa, bufferDesc->size, tensor_info.shared_case_type);

  return AIPU_STATUS_SUCCESS;
}

aipu_status_t JobV3::dump_emu_metadata(const std::string &metafile) {
  FileWrapper ofsmt(metafile, std::ios_base::in | std::ios_base::out |
                                  std::ios_base::trunc);
  if (!ofsmt.is_open())
    return AIPU_STATUS_ERROR_OPEN_FILE_FAIL;

  tcb_t tcb = {0};
  ofsmt << "Total TCBs Count: " << std::dec << m_tot_tcb_cnt << "\n";
  for (uint32_t i = 0; i < m_tot_tcb_cnt; i++) {
    m_mem->read(m_init_tcb.pa + i * sizeof(tcb_t), &tcb, sizeof(tcb_t));

    if (tcb_task_type(tcb.flag) == tcb_ctl::FLAG_TASK_TYPE_INIT) {
      /* init tcb + segmmu tcb */
      if (tcb.next != 0) {
        ofsmt << "\n***INIT TCB " << std::dec << i << " ***\n";
        ofsmt << "flag: 0x" << std::hex << tcb.flag << "\n";
        ofsmt << "next: 0x" << tcb.next << "\n";

        ofsmt << "GM_CTRL: 0x" << tcb.grid.gm_ctrl << "\n";
        ofsmt << "grid_id: " << std::dec << (tcb.grid.grid_id >> 16) << "\n";
        ofsmt << "GM0_CTRL: 0x" << std::hex << tcb.grid.gm_rgnx_ctrl[0] << "\n";
        ofsmt << "GM1_CTRL: 0x" << tcb.grid.gm_rgnx_ctrl[1] << "\n";
        ofsmt << "GM0_LO: 0x" << tcb.grid.gm_rgnx_addr[0].v32.lo << "\n";
        ofsmt << "GM0_HI: 0x" << tcb.grid.gm_rgnx_addr[0].v32.hi << "\n";
        ofsmt << "GM1_LO: 0x" << tcb.grid.gm_rgnx_addr[1].v32.lo << "\n";
        ofsmt << "GM1_HI: 0x" << tcb.grid.gm_rgnx_addr[1].v32.hi << "\n";
        for (int j = 0; j < 4; j++) {
          ofsmt << "ASID" << std::dec << j << "_LO: 0x" << std::hex
                << tcb.grid.asids[j].v32.lo << "\n";
          ofsmt << "ASID" << std::dec << j << "_HI: 0x" << std::hex
                << tcb.grid.asids[j].v32.hi << "\n";
        }
      } else {
        ofsmt << "\n***INIT SEGMMU TCB " << std::dec << i << " ***\n";
        ofsmt << "SMMU.CTRL: 0x" << std::hex << tcb.group.smmu.ctrl << "\n";
        ofsmt << "SMMU.REMAP: 0x" << tcb.group.smmu.remap << "\n";
        for (int j = 0; j < 4; j++) {
          ofsmt << "SMMU.SEGS" << std::dec << j << ".CTRL0: 0x" << std::hex
                << tcb.group.smmu.segs[j].ctrl0 << "\n";
          ofsmt << "SMMU.SEGS" << std::dec << j << ".CTRL1: 0x" << std::hex
                << tcb.group.smmu.segs[j].ctrl1 << "\n";
        }

        ofsmt << "NEXT_SMMU.CTRL: 0x" << std::hex
              << tcb.group.next_core_smmu.ctrl << "\n";
        ofsmt << "NEXT_SMMU.REMAP: 0x" << tcb.group.next_core_smmu.remap
              << "\n";
        for (int j = 0; j < 4; j++) {
          ofsmt << "NEXT_SMMU.SEGS" << std::dec << j << ".CTRL0: 0x" << std::hex
                << tcb.group.next_core_smmu.segs[j].ctrl0 << "\n";
          ofsmt << "NEXT_SMMU.SEGS" << std::dec << j << ".CTRL1: 0x" << std::hex
                << tcb.group.next_core_smmu.segs[j].ctrl1 << "\n";
        }
      }
    } else if (tcb_task_type(tcb.flag) == tcb_ctl::FLAG_TASK_TYPE_TASK_V3) {
      ofsmt << "\n***TASK TCB " << std::dec << i << "***\n";
      ofsmt << "flag: 0x" << std::hex << tcb.flag << "\n";

      ofsmt << "next: 0x" << tcb.next << "\n";
      ofsmt << "loop_count: " << std::dec << tcb.task.loop_count << "\n";

      ofsmt << "start_pc: 0x" << std::hex << tcb.task.spc << "\n";
      ofsmt << "interrupt: 0x" << std::hex << tcb.task.interrupt_en << "\n";

      ofsmt << "group_id: " << std::dec << tcb.task.group_id << "\n";
      ofsmt << "grid_id: " << tcb.task.grid_id << "\n";
      ofsmt << "task_id: " << tcb.task.task_id << "\n";

      ofsmt << "grid_dim_x: " << tcb.task.grid_dim_x << "\n";
      ofsmt << "grid_dim_y: " << tcb.task.grid_dim_y << "\n";
      ofsmt << "grid_dim_z: " << tcb.task.grid_dim_z << "\n";

      ofsmt << "group_dim_x: " << tcb.task.group_dim_x << "\n";
      ofsmt << "group_dim_y: " << tcb.task.group_dim_y << "\n";
      ofsmt << "group_dim_z: " << tcb.task.group_dim_z << "\n";

      ofsmt << "group_id_x: " << tcb.task.group_id_x << "\n";
      ofsmt << "group_id_y: " << tcb.task.group_id_y << "\n";
      ofsmt << "group_id_z: " << tcb.task.group_id_z << "\n";

      ofsmt << "task_id_x: " << tcb.task.task_id_x << "\n";
      ofsmt << "task_id_y: " << tcb.task.task_id_y << "\n";
      ofsmt << "task_id_z: " << tcb.task.task_id_z << "\n";

      ofsmt << "sp: 0x" << std::hex << tcb.task.sp << "\n";
      ofsmt << "pp: 0x" << tcb.task.pp << "\n";
      ofsmt << "dp: 0x" << tcb.task.dp << "\n";
      ofsmt << "cp: 0x" << tcb.task.cp << "\n";
      ofsmt << "pprint: 0x" << tcb.task.pprint << "\n";
      ofsmt << "pprofiler: 0x" << tcb.task.pprofiler << "\n";
      ofsmt << "fmdp: 0x" << tcb.task.fmdp << "\n";
      ofsmt << "tap: 0x" << tcb.task.tap << "\n";
      ofsmt << "dap: 0x" << tcb.task.dap << "\n";
      ofsmt << "pap: 0x" << tcb.task.pap << "\n";
      ofsmt << "idp: 0x" << tcb.task.idp << "\n";
      ofsmt << "dsize: 0x" << tcb.task.dsize << "\n";
    } else {
      LOG(LOG_ERR, "invalid TCB task type, flag: 0x%x\n", tcb.flag);
    }
  }

  DEV_PA_64 dump_pa = 0;
  uint32_t dump_size = 0;
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
  MainContext *ctx = static_cast<MainContext *>(graph().m_ctx);
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
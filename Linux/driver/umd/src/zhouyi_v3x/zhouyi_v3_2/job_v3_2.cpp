// Copyright (C) 2023-2025 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

/**
 * @file  job_v3_2.cpp
 * @brief AIPU User Mode Driver (UMD) aipu v3_2 job module implementation
 */

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <cstring>
#include <mutex>
#include <vector>

// clang-format off
#include "job_v3_2.h"
// clang-format on

#include "utils/helper.h"
#include "zhouyi_v3x/common/graph_v3x.h"
#include "zhouyi_v3x/zhouyi_v3_2/gm_v3_2.h"

#if defined(SIMULATION)
#include "device/simulator/simulator_v3_2.h"
#endif

namespace aipudrv {
using namespace tcb_v3_2;

JobV3_2::JobV3_2(MainContext *ctx, GraphBase &graph, DeviceBase *dev,
                 aipu_create_job_cfg_t *config)
    : JobV3X(ctx, graph, dev, config) {
  m_gm = new GM_V3_2(*this);
  m_optimized_reuse_alloc = true;
}

JobV3_2::~JobV3_2() {
  if (m_dyn_shape != nullptr) {
    delete m_gm;
    m_gm = nullptr;
  }

  if (m_dyn_shape != nullptr) {
    delete m_dyn_shape;
    m_dyn_shape = nullptr;
  }
}

void JobV3_2::set_job_params(uint32_t sg_cnt, uint32_t task_per_sg,
                             uint32_t remap, uint32_t core_cnt) {
  m_sg_cnt = sg_cnt;
  m_task_per_sg = task_per_sg;
  m_remap_flag = remap;

  /**
   * tcb chain format
   * global group init tcb:
   *     1 grid init-tcb + 1 group init-tcb + n task-tcb grp
   *
   * local group init tcb:
   *     1 grid init-tcb + 1 group init-tcb + 1 task-tcb grp +...+ 1 group
   * init-tcb + 1 task-tcb grp
   */
  m_tot_tcb_cnt = 1 + m_sg_cnt * (m_task_per_sg + 1);

  m_backup_tcb.reset(new char[m_tot_tcb_cnt * sizeof(tcb_t)]);
}

#define SEGMMU_MEM_CTRL_EN (1 << 0)
#define SEGMMU_REMAP_EN (1 << 4)
#define SEGMMU_REMAP_SHARE_EN (1 << 5)
#define SEGMMU_IN_ASID_WR (1 << 0)
#define SEGMMU_IN_ASID_RD (1 << 1)
aipu_status_t JobV3_2::setup_segmmu(SubGraphTask &sg_task) {
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
          "Segmmu core idx invalid, (core_id, seg_idx, ctrl_idx): (%x, %d, %d)",
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
              "%d)",
              core_idx, s_id.segmmu_idx, s_id.segmmu_ctrl_idx);
          goto out;
        }
      } else {
        LOG(LOG_ERR,
            "Segmmu seg idx invalid, (core_id, seg_idx, ctrl_idx): (%x, %d, "
            "%d)",
            core_idx, s_id.segmmu_idx, s_id.segmmu_ctrl_idx);
        goto out;
      }
    }
  }

out:
  return ret;
}

aipu_status_t JobV3_2::init_grid_id(uint16_t &grid_id) {
  return m_dev->get_grid_id(grid_id) < 0 ? AIPU_STATUS_ERROR_ALLOC_GRIP_ID
                                         : AIPU_STATUS_SUCCESS;
}

aipu_status_t JobV3_2::init_group_id(uint32_t sg_cnt) {
  if (m_dev->get_start_group_id(m_sg_cnt, m_start_group_id) < 0)
    return AIPU_STATUS_ERROR_ALLOC_GROUP_ID;

  m_group_id_idx = m_start_group_id;
  return AIPU_STATUS_SUCCESS;
}

aipu_status_t JobV3_2::alloc_job_buffers() {
  const auto &fm_info = graph().get_fmsec_info();
  uint32_t iova_size = fm_info.info.at(FMSection::ReservedIOVA).size;
  if (iova_size > m_reserved_iova_size)
    m_reserved_iova_size = iova_size;

  uint32_t align_page = graph().get_asid_align_page();
  uint32_t asid_cfg = (0 << 8) | AIPU_MEM_REGION_DEFAULT;
  aipu_status_t ret = m_mem->malloc(fm_info.size, align_page, &m_top_job_buf,
                                    "job_top", asid_cfg, m_reserved_iova_size);
  if (ret != AIPU_STATUS_SUCCESS)
    return ret;

  m_exec_id = m_top_job_buf->exec_id;
  for (const auto &info : fm_info.info) {
    BufferDesc *desc = new BufferDesc;
    if (info.first == FMSection::ReservedIOVA) {
      DEV_PA_64 pa =
          m_top_job_buf->pa + m_top_job_buf->size; /* pa from jobbuf end */
      /* different job maybe have different iova size */
      desc->init(m_top_job_buf->asid_base, pa, m_reserved_iova_size,
                 m_reserved_iova_size);
    } else if (info.first == FMSection::TcbChain) {
      DEV_PA_64 pa = m_top_job_buf->pa + info.second.offset;
      if (graph().get_isa() == ISAType::ISAv6 && graph().get_revision() < 2) {
        uint32_t tail_id = ((pa + info.second.size) >> 12) & 0x3FF;
        uint32_t offset = ((m_grid_id & 0x3FF) - tail_id + 0x400) & 0x3FF;
        pa += offset * AIPU_PAGE_SIZE;
      }
      desc->init(m_top_job_buf->asid_base, pa, info.second.size,
                 info.second.size);
    } else {
      DEV_PA_64 pa = m_top_job_buf->pa + info.second.offset;
      desc->init(m_top_job_buf->asid_base, pa, info.second.size,
                 info.second.size);
    }
    m_secbuf_desc[info.first] = desc;
    std::string name = graph().get_section_name(info.first).c_str();
    m_mem->add_tracking(desc->pa, desc->size, MemOperationSub, name.c_str(),
                        false, 0);
  }
  return AIPU_STATUS_SUCCESS;
}

aipu_status_t JobV3_2::free_job_buffers() {
  for (auto &desc : m_secbuf_desc)
    m_mem->free_bufferdesc(&desc.second);

  for (uint32_t i = 0; i < m_weight.size(); ++i) {
    if (m_weight[i].wb_zerocpy_const != nullptr &&
        m_weight[i].wb_zerocpy_const->size != 0)
      m_mem->free_bufferdesc(&m_weight[i].wb_zerocpy_const);

    if (graph().m_put_weight_gm) {
      if (m_weight[i].wb_weight != nullptr && m_weight[i].wb_weight->size != 0)
        m_mem->free_bufferdesc(&m_weight[i].wb_weight);
    }
  }

  /* weight desc in graph, zcy desc in job */
  for (uint32_t bss_id = 0; bss_id < graph().get_bss_cnt(); ++bss_id) {
    const std::vector<GraphSectionDesc> &static_sections =
        graph().get_static_section_ref(bss_id);
    for (uint32_t i = 0; i < static_sections.size(); ++i) {
      if (static_sections[i].type == SECTION_TYPE_ZEROCPY_CONSTANT) {
        uint32_t offset = i;
        for (uint32_t bss = 0; bss < bss_id; ++bss)
          offset += graph().get_static_section_ref(bss).size();
        if (m_weight.size() > 0 && offset < m_weight[0].wb_weights.size() &&
            m_weight[0].wb_weights[offset] != nullptr)
          m_mem->free_bufferdesc(&m_weight[0].wb_weights[offset]);
      }
    }
  }

  for (uint32_t i = 0; i < m_reuses_desc.size(); i++) {
    if (m_sfm_idxes.count(i) == 0 || graph().is_gm_buffer(i, GM_BUF_TYPE_REUSE))
      m_mem->free_bufferdesc(&m_reuses_desc[i]);
    else
      m_mem->free(&m_reuses_desc[i]);
  }
  m_reuses_desc.clear();

  if (m_top_job_buf && m_top_job_buf->size != 0)
    m_mem->free(&m_top_job_buf, "job_top");

  m_init_tcb.init(0);

  for (uint32_t i = 0; i < m_sg_job.size(); i++) {
    free_sg_buffers(m_sg_job[i]);
    m_sg_job[i].reset(0, 0);
  }

  m_dev->put_start_group_id(m_start_group_id, m_sg_cnt);

  m_sg_job.clear();

  m_inputs.clear();
  m_outputs.clear();
  m_inter_dumps.clear();
  m_profiler.clear();
  m_printf.clear();
  m_layer_counter.clear();
  return AIPU_STATUS_SUCCESS;
}

aipu_status_t JobV3_2::config_tcb_smmu(tcb_t &tcb) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;

  if (m_segmmu_num > 0) {
    if (m_segmmu_num == 1) {
      SegMMUConfig &segmmu = m_segmmu_sec[0];

      tcb.group.segmmu_ctrl = segmmu.SegMMU_ctl;
      tcb.group.segmmu_remap_ctrl0 = segmmu.SegMMU_remap;
      tcb.group.segmmu_remap_ctrl1 = segmmu.SegMMU_remap;

      for (int j = 0; j < 4; j++) {
        tcb.group.segmmu_seg_ctrl[2 * j] = segmmu.seg[j].control[0];
        tcb.group.segmmu_seg_ctrl[2 * j + 1] = segmmu.seg[j].control[1];
      }
    }
  }

  return ret;
}

aipu_status_t JobV3_2::config_tcb_deps(tcb_t &tcb, uint32_t sg_id) {
  switch (graph().get_subgraph(sg_id).precursor_cnt) {
  case SUBG_DEPEND_NONE:
    tcb.flag |= tcb_ctl::FLAG_DEP_TYPE_NONE;
    break;

  case 1 ... 4: {
    uint16_t dep_group_id = 0;

    tcb.flag |= tcb_ctl::FLAG_DEP_TYPE_GROUP;
    for (int32_t i = 0; i < graph().get_subgraph(sg_id).precursor_cnt; i++) {
      if (graph().get_subgraph(sg_id).precursors[i] > 0x7fff) {
        LOG(LOG_ERR, "Depend group id(%d) is invalid",
            graph().get_subgraph(sg_id).precursors[i]);
        return AIPU_STATUS_ERROR_INVALID_GBIN;
      }

      dep_group_id =
          graph().get_subgraph(sg_id).precursors[i] + m_start_group_id;
      dep_group_id &= 0x7FFF; // 15 bits group id field
      tcb.group_deps[i] = tcb_ctl::EN_GROUP_DEPEND | dep_group_id;
    }
  } break;

  case SUBG_DEPEND_PREALL:
    tcb.flag |= tcb_ctl::FLAG_DEP_TYPE_PRE_ALL;
    break;

  default:
    LOG(LOG_ERR, "subgraph %u, precursor_cnt=%d", sg_id,
        graph().get_subgraph(sg_id).precursor_cnt);
    return AIPU_STATUS_ERROR_INVALID_GBIN;
  }

  /* only depends last group instead of all cores; if first group, depend none
   */
  if (m_sg_cnt == graph().get_bss_cnt() &&
      tcb_dep_type(tcb.flag) == tcb_ctl::FLAG_DEP_TYPE_PRE_ALL) {
    tcb.flag &= ~tcb_ctl::FLAG_DEP_TYPE_PRE_ALL;
    if (sg_id != 0) {
      tcb_t prev_task0;
      Task &task = m_sg_job[sg_id - 1].tasks[0];
      m_mem->read(task.tcb.pa, &prev_task0, sizeof(tcb_t));
      tcb.flag |= tcb_ctl::FLAG_DEP_TYPE_GROUP;
      tcb.group_deps[0] = tcb_ctl::EN_GROUP_DEPEND | prev_task0.task.group_id;
    }
  }
  return AIPU_STATUS_SUCCESS;
}

aipu_status_t JobV3_2::setup_task_tcb(uint32_t sg_id, uint32_t grid_id,
                                      uint32_t core_id, uint32_t task_id,
                                      bool) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  Task &task = m_sg_job[sg_id].tasks[task_id];
  tcb_t tcb;

  memset(&tcb, 0, sizeof(tcb_t));
  tcb.task.interrupt_en = tcb_ctl::EN_INTERRUPT_ALL_TYPE_V32;
  tcb.flag = tcb_ctl::FLAG_TASK_TYPE_TASK_V32;

  if (task_id == (m_task_per_sg - 1))
    tcb.flag |= tcb_ctl::FLAG_END_TYPE_GROUP_END;

  /* v3_2 last tcb must have a grid end, although we don't need a cluster
   * interrupt */
  if ((sg_id == (m_sg_cnt - 1)) && (task_id == (m_task_per_sg - 1)))
    tcb.flag |= tcb_ctl::FLAG_END_TYPE_GRID_END;

  /**
   * depend_all: set init group tcb and first tcb of tcb group
   * depend_groups: set init group and all tasks of tcb group
   */
  ret = config_tcb_deps(tcb, sg_id);
  if (ret != AIPU_STATUS_SUCCESS)
    return ret;
  if (task_id != 0 && tcb_dep_type(tcb.flag) == tcb_ctl::FLAG_DEP_TYPE_PRE_ALL)
    tcb.flag &= ~tcb_ctl::FLAG_DEP_TYPE_PRE_ALL;

  tcb.task.spc = get_low_32(m_text->align_asid_pa +
                            graph().get_subgraph(sg_id).text.offset);
  tcb.task.group_id = (uint16_t)m_group_id_idx;
  tcb.task.grid_id = (uint16_t)grid_id;
  tcb.task.task_id = (uint16_t)task_id;
  tcb.task.warmup_len = graph().get_subgraph(sg_id).warmup_len;
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
  if (m_crodata != nullptr && m_crodata->size > 0)
    tcb.task.cp = get_low_32(m_crodata->align_asid_pa);

  /* update profile buffer offset according to subgraph index */
  if (m_profiler.size() > 0)
    tcb.task.pprofiler =
        get_low_32(m_profiler[0].align_asid_pa +
                   graph().get_subgraph(sg_id).profiler_buf_size);

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
  }

  if (graph().is_dynamic_shape())
    tcb.task.global_param = get_low_32(m_global_param->align_asid_pa);

  /* flush TCB to AIPU mem */
  m_mem->write(task.tcb.pa, (const char *)&tcb, sizeof(tcb_t));

  return AIPU_STATUS_SUCCESS;
}

aipu_status_t JobV3_2::setup_tcb_group(uint32_t sg_id, uint32_t grid_id,
                                       uint32_t core_id, bool) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;

  /* setup task TCBs */
  for (uint32_t t = 0; t < m_task_per_sg; t++) {
    ret = setup_task_tcb(sg_id, grid_id, core_id, t);
    if (ret != AIPU_STATUS_SUCCESS)
      return ret;
  }

  /* increase group index for each group */
  m_group_id_idx++;

  return ret;
}

aipu_status_t JobV3_2::setup_tcb_chain() {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  tcb_t tcb;
  uint32_t core_id = 0;

  /* Grid init TCB */
  memset(&tcb, 0, sizeof(tcb_t));
  tcb.flag = tcb_ctl::FLAG_TASK_TYPE_GRID_INIT | tcb_ctl::FLAG_L2D_FLUSH;
  tcb.grid.group_num = m_sg_cnt;
  tcb.grid.grid_id = m_grid_id;
  tcb.grid.group_id = m_group_id_idx;
  tcb.grid.interrupt_en = tcb_ctl::EN_INTERRUPT_GRID_ALL;

  reinterpret_cast<GM_V3_2 *>(m_gm)->setup_gm_sync_from_ddr(tcb);
  m_mem->write(m_init_tcb.pa, (const char *)&tcb, sizeof(tcb_t));

  for (uint32_t i = 0; i < graph().get_subgraph_cnt(); i++) {
    /* Group init TCB */
    memset(&tcb, 0, sizeof(tcb_t));
    tcb.flag = tcb_ctl::FLAG_TASK_TYPE_GROUP_INIT | tcb_ctl::FLAG_GRID_INIT;
    tcb.group.grid_id = m_grid_id;
    tcb.group.group_id = m_group_id_idx;

    // SegMMU
    // config_tcb_smmu(tcb);

    ret = config_tcb_deps(tcb, graph().get_subgraph(i).id);
    if (ret != AIPU_STATUS_SUCCESS)
      return ret;

    /**
     * ASID0: feature map buffer region
     * the whole graph share one copy of reuse buffer for feature map.
     */
    tcb.group.asids[0] = get_low_32(m_top_job_buf->asid_base |
                                    tcb_ctl::ASID_RD | tcb_ctl::ASID_WR);
    tcb.group.asids[1] = get_high_32(m_top_job_buf->asid_base);

    /**
     * ASID1: weight buffer region
     * if LLM model contains multiple BSSs, each BSS will locate in private
     * ASID1 region. so here, set ASID1 base register from weight buffer's
     * asid_base(pa).
     */
    DEV_PA_64 asid1_base = 0;
    uint32_t bss_idx = graph().get_subgraph(i).bss_idx;
    if (m_weight.size() > bss_idx)
      asid1_base = m_weight[bss_idx].wb_asid_base;
    tcb.group.asids[2] =
        get_low_32(asid1_base | tcb_ctl::ASID_RD | tcb_ctl::ASID_WR);
    tcb.group.asids[3] = get_high_32(asid1_base);

    for (uint32_t j = 2; j < 4; j++) {
      tcb.group.asids[2 * j] = 0;
      tcb.group.asids[2 * j + 1] = 0;
    }
    m_mem->write(m_init_tcb.pa + sizeof(tcb_t) +
                     (m_task_per_sg + 1) * i * sizeof(tcb_t),
                 (const char *)&tcb, sizeof(tcb_t));

    /* Task TCB */
    ret = setup_tcb_group(graph().get_subgraph(i).id, m_grid_id, core_id);
    if (ret != AIPU_STATUS_SUCCESS)
      return ret;

    if (++core_id >= m_core_cnt)
      core_id = 0;
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

aipu_status_t
JobV3_2::specify_io(aipu_shared_tensor_info_t &tensor_info,
                    const std::vector<JobIOBuffer> *iobuffer_vec) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  uint64_t buffer_pa = 0;
  int fd = tensor_info.dmabuf_fd;
  uint32_t index = tensor_info.tensor_idx;

  uint32_t reuse_index = (*iobuffer_vec)[index].ref_section_iter;
  BufferDesc *bufferDesc = m_reuses_desc[reuse_index];
  aipu_dma_buf dma_buf{fd, 0, 0};

  switch (tensor_info.shared_case_type) {
  case AIPU_SHARE_BUF_IN_ONE_PROCESS:
  case AIPU_SHARE_BUF_CUSTOMED: {
    LOG(LOG_ERR, "v3_2 only supports dmabuf, please use dmabuf instead");
    return AIPU_STATUS_ERROR_INVALID_CONFIG;
  } break;

  case AIPU_SHARE_BUF_DMABUF: {
    ret = convert_ll_status(
        m_dev->ioctl_cmd(AIPU_IOCTL_GET_DMABUF_INFO, &dma_buf));
    if (ret != AIPU_STATUS_SUCCESS)
      return ret;

    if (dma_buf.bytes > m_reserved_iova_size) {
      LOG(LOG_ERR,
          "dmabuf size > reserved iova size (0x%llx,0x%x), you can specify "
          "larger reserve size in 'aipu_create_job_cfg_t'",
          dma_buf.bytes, m_reserved_iova_size);
      return AIPU_STATUS_ERROR_INVALID_SIZE;
    }

    aipu_rebind_buf_desc rebind_desc = {0};
    rebind_desc.pa_unmap = dma_buf.pa;
    rebind_desc.exec_id = m_exec_id;
    ret = convert_ll_status(
        m_dev->ioctl_cmd(AIPU_IOCTL_REBIND_DMA_BUF, &rebind_desc));
    if (ret != AIPU_STATUS_SUCCESS)
      return ret;

    buffer_pa = rebind_desc.pa;
    dma_buf.pa = rebind_desc.pa;
    bufferDesc->init(get_asid0_base(), buffer_pa, bufferDesc->size,
                     bufferDesc->req_size);
    (*iobuffer_vec)[index].set_dmabuf_info(fd, dma_buf.bytes, 0);
  } break;

  case AIPU_SHARE_BUF_ATTACH_DMABUF: {
    dma_buf.exec_id = m_exec_id;
    ret =
        convert_ll_status(m_dev->ioctl_cmd(AIPU_IOCTL_ATTACH_DMABUF, &dma_buf));
    if (ret != AIPU_STATUS_SUCCESS)
      return ret;

    buffer_pa = dma_buf.pa;
    bufferDesc->init(get_asid0_base(), buffer_pa, bufferDesc->size,
                     bufferDesc->req_size);
    (*iobuffer_vec)[index].set_dmabuf_info(fd, dma_buf.bytes, 0);
  } break;

  default:
    return AIPU_STATUS_ERROR_INVALID_OP;
  }

  LOG(LOG_DEBUG, "specify_io: pa=%lx, size=%lx, share_case_type=%d", buffer_pa,
      bufferDesc->size, tensor_info.shared_case_type);
  return AIPU_STATUS_SUCCESS;
}

aipu_status_t JobV3_2::dump_emu_metadata(const std::string &metafile) {
  FileWrapper ofsmt(metafile, std::ios_base::in | std::ios_base::out |
                                  std::ios_base::trunc);
  if (!ofsmt.is_open())
    return AIPU_STATUS_ERROR_OPEN_FILE_FAIL;

  tcb_t tcb = {0};
  /* dump metadata.txt */
  ofsmt << "Total TCBs Count: " << std::dec << m_tot_tcb_cnt << "\n";

  /* Grid/Group init TCB and Task TCB */
  for (uint32_t i = 0; i < m_tot_tcb_cnt; i++) {
    m_mem->read(m_init_tcb.pa + sizeof(tcb_t) * i, &tcb, sizeof(tcb_t));

    if (tcb_task_type(tcb.flag) == tcb_ctl::FLAG_TASK_TYPE_GRID_INIT) {
      ofsmt << "\n***GRID INIT TCB " << std::dec << i << " ***\n";

      ofsmt << "flag: 0x" << std::hex << tcb.flag << "\n";
      ofsmt << "group_num: " << std::dec << tcb.grid.group_num << "\n";
      ofsmt << "grid_interrupt_en: 0x" << std::hex << tcb.grid.interrupt_en
            << "\n";
      ofsmt << "grid_groupid: " << std::dec << tcb.grid.group_id << "\n";
      ofsmt << "grid_gridid: " << tcb.grid.grid_id << "\n";
      ofsmt << "gm_ctrl: 0x" << std::hex << tcb.grid.gm_ctrl << "\n";
      ofsmt << "gm_sync: 0x" << tcb.grid.gm_sync << "\n";
      ofsmt << "gm_addr_low: 0x" << tcb.grid.gm_addr_low << "\n";
      ofsmt << "gm_addr_high: 0x" << tcb.grid.gm_addr_high << "\n";
    } else if (tcb_task_type(tcb.flag) == tcb_ctl::FLAG_TASK_TYPE_GROUP_INIT) {
      ofsmt << "\n***GROUP INIT TCB " << std::dec << i << " ***\n";

      ofsmt << "flag: 0x" << std::hex << tcb.flag << "\n";
      ofsmt << "segmmu_ctrl: 0x" << tcb.group.segmmu_ctrl << "\n";
      ofsmt << "segmmu_remap_ctrl0: 0x" << tcb.group.segmmu_remap_ctrl0 << "\n";
      ofsmt << "segmmu_remap_ctrl1: 0x" << tcb.group.segmmu_remap_ctrl1 << "\n";
      ofsmt << "group_interrupt_en: " << std::hex << tcb.group.interrupt_en
            << "\n";
      ofsmt << "group_groupid: " << std::dec << tcb.group.group_id << "\n";
      ofsmt << "group_gridid: " << tcb.group.grid_id << "\n";

      for (int j = 0; j < 8; j++) {
        ofsmt << "segmmu_seg" << std::dec << j << "_ctrl0: 0x" << std::hex
              << tcb.group.segmmu_seg_ctrl[2 * j] << "\n";
        ofsmt << "segmmu_seg" << std::dec << j << "_ctrl1: 0x" << std::hex
              << tcb.group.segmmu_seg_ctrl[2 * j + 1] << "\n";
      }

      for (int j = 0; j < 4; j++) {
        ofsmt << "ASID" << std::dec << j << "_LO: 0x" << std::hex
              << tcb.group.asids[2 * j] << "\n";
        ofsmt << "ASID" << std::dec << j << "_HI: 0x" << std::hex
              << tcb.group.asids[2 * j + 1] << "\n";
      }
    } else if (tcb_task_type(tcb.flag) == tcb_ctl::FLAG_TASK_TYPE_TASK_V32) {
      ofsmt << "\n***TASK TCB " << std::dec << i << " ***\n";

      ofsmt << "flag: 0x" << std::hex << tcb.flag << "\n";
      ofsmt << "start_pc: 0x" << std::hex << tcb.task.spc << "\n";
      ofsmt << "interrupt_en: 0x" << tcb.task.interrupt_en << "\n";

      ofsmt << "group_id: " << std::dec << tcb.task.group_id << "\n";
      ofsmt << "grid_id: " << tcb.task.grid_id << "\n";
      ofsmt << "task_id: " << tcb.task.task_id << "\n";
      ofsmt << "warm_len: " << tcb.task.warmup_len << "\n";

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
      ofsmt << "dsize: 0x" << tcb.task.dsize << "\n";
      ofsmt << "tcbp: 0x" << tcb.task.tcbp << "\n";

      ofsmt << "group_deps[0]: " << tcb.group_deps[0] << "\n";
      ofsmt << "group_deps[1]: " << tcb.group_deps[1] << "\n";
      ofsmt << "group_deps[2]: " << tcb.group_deps[2] << "\n";
      ofsmt << "group_deps[3]: " << tcb.group_deps[3] << "\n";
    } else {
      LOG(LOG_ERR, "invalid TCB type\n");
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
void JobV3_2::dumpcfg_alljob() {
  JobV3_2 *job = nullptr;
  std::vector<uint32_t> cluster_id[4];
  uint32_t count = 0, cmdpool_mask = 0;
  MainContext *ctx = static_cast<MainContext *>(graph().m_ctx);
  GraphTable &graphs = ctx->get_graphtable();
  GraphV3X *graph = nullptr;
  std::ostringstream oss;
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
      job = static_cast<JobV3_2 *>(item.second);
      for (uint32_t i = 0; i < job->m_dumpcfg_input.size(); i++) {
        oss << "FILE" << std::dec << count << "="
            << job->m_dumpcfg_input.at(i).file << "\n";
        oss << "BASE" << count << "=0x" << std::hex
            << job->m_dumpcfg_input.at(i).base << "\n";
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
  cmdpool_mask = 1;
  for (auto g : graphs) {
    graph = static_cast<GraphV3X *>(g.second);
    for (auto item : graph->m_jobs) {
      job = static_cast<JobV3_2 *>(item.second);
      if (cmdpool_mask & (1 << job->m_bind_cmdpool_id)) {
        oss << "SET_PARTITION" << std::dec << count << "="
            << job->m_dumpcfg_host.part_id << "\n";
        oss << "TCBP_HI" << std::dec << count << "=0x" << std::hex
            << job->m_dumpcfg_host.hi_addr << "\n";
        oss << "TCBP_LO" << std::dec << count << "=0x" << std::hex
            << job->m_dumpcfg_host.lo_addr << "\n";
        count++;
        cmdpool_mask &= ~(1 << job->m_bind_cmdpool_id);
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
      job = static_cast<JobV3_2 *>(item.second);
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
      job = static_cast<JobV3_2 *>(item.second);
      ofsmt << job->m_dumpcfg_meta;
      ofsmt << "\n";
    }
    ofsmt << "\n";
  }
  ofsmt.close();
}
#endif
} // namespace aipudrv
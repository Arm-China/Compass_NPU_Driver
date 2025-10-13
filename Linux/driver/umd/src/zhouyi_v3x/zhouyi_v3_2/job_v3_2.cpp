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

void JobV3_2::get_tcb_head_cnt(uint32_t sg_idx, uint32_t &head_cnt) {
  head_cnt = 2 + sg_idx;
}

aipu_status_t JobV3_2::init_group_id(uint32_t sg_cnt) {
  if (m_dev->get_start_group_id(m_sg_cnt, m_start_group_id) < 0) {
    return AIPU_STATUS_ERROR_ALLOC_GROUP_ID;
  }
  m_group_id_idx = m_start_group_id;
  return AIPU_STATUS_SUCCESS;
}

aipu_status_t JobV3_2::setup_gm_buffer() {
  BufferDesc &desc = m_secbuf_desc[FMSection::GM];
  const auto &gm_info = graph().get_gmsec_info();

  if (graph().m_put_weight_gm) {
    m_weight.clear();
    m_weight.resize(graph().get_bss_cnt());
    DEV_PA_64 pa_base =
        desc.pa + gm_info.info.at(GMBufType::GM_BUF_TYPE_WEIGHT).offset;
    for (uint32_t i = 0; i < graph().get_bss_cnt(); ++i) {
      uint32_t weight_size = graph().get_const_size(i);
      BufferDesc *weight = new BufferDesc;
      weight->init(desc.asid_base, pa_base, weight_size, weight_size);
      m_weight[i].wb_weight = weight;
      pa_base += ALIGN_PAGE(weight_size);

      uint32_t zcy_size = graph().get_zerocpy_const_size(i);
      BufferDesc *zcy = new BufferDesc;
      zcy->init(desc.asid_base, pa_base, zcy_size, zcy_size);
      m_weight[i].wb_zerocpy_const = zcy;
      pa_base += ALIGN_PAGE(zcy_size);

      m_weight[i].wb_asid_base = desc.asid_base;
    }
    graph().setup_weight_buffer(m_weight, true /* setup_zcy */);
  }

  BufferDesc sec_desc;
  const auto &fm_info = graph().get_fmsec_info();
  uint32_t gm_offset = fm_info.info.at(FMSection::GM).offset;
  if (graph().m_put_desc_gm) {
    const auto &info =
        graph().get_gmsec_info().info.at(GMBufType::GM_BUF_TYPE_DESCRIPTOR);
    sec_desc.init(desc.asid_base, desc.pa + info.offset, info.size, info.size);
    m_secbuf_desc[FMSection::Dcr] = sec_desc;
  }

  if (gm_info.info.count(GMBufType::GM_BUF_TYPE_REUSE) != 0) {
    uint32_t idx =
        graph().m_gm_config_desc[GMBufType::GM_BUF_TYPE_REUSE].begin()->first;
    const auto &info = gm_info.info.at(GMBufType::GM_BUF_TYPE_REUSE);
    if (idx < m_reuses_desc.size()) {
      m_reuses_desc[idx]->asid_base = desc.asid_base;
      m_reuses_desc[idx]->pa = desc.pa + info.offset;
      m_reuses_desc[idx]->align_asid_pa =
          desc.pa + info.offset - desc.asid_base;
      m_reuses_desc[idx]->size = info.size;
      m_reuses_desc[idx]->req_size = info.size;
    }
  }

  if (graph().m_put_ws_gm) {
    const auto &info =
        graph().get_gmsec_info().info.at(GMBufType::GM_BUF_TYPE_WORKSPACE);
    sec_desc.init(m_top_job_buf->pa,
                  m_top_job_buf->pa + gm_offset + info.offset, info.size,
                  info.size);
    m_secbuf_desc[FMSection::TotalPriv] = sec_desc;
  }

  if (get_coredump() != nullptr) {
    aipu_status_t ret = get_coredump()->set_gm_info(0, &desc);
    if (ret != AIPU_STATUS_SUCCESS) {
      LOG(LOG_ERR, "set coredump gm information fail");
      return ret;
    }
  }

  return AIPU_STATUS_SUCCESS;
}

aipu_status_t JobV3_2::setup_dyn_shape_buffer() {
  DSModelGlobalParam *param = (DSModelGlobalParam *)graph().m_bglobalparam.va;
  uint32_t input_shape_offset = param->input_shape_offset;

  DEV_PA_64 pa = m_secbuf_desc[FMSection::ModelParam].pa;
  m_mem->write(pa, graph().m_bglobalparam.va, sizeof(DSModelGlobalParam));

  for (uint32_t input_idx = 0; input_idx < m_dyn_shape->get_config_shape_sz();
       input_idx++) {
    if (!m_dyn_shape->in_config_shape(input_idx)) {
      LOG(LOG_ERR, "input shape %d is not configured", input_idx);
      return AIPU_STATUS_ERROR_NOT_CONFIG_SHAPE;
    }

    for (uint32_t dim_idx = 0;
         dim_idx < m_dyn_shape->get_config_shape_dim_sz(input_idx); dim_idx++) {
      uint32_t shape_item =
          m_dyn_shape->get_config_shape_item(input_idx, dim_idx);

      m_mem->write(pa + input_shape_offset, &shape_item, sizeof(uint32_t));
      input_shape_offset += sizeof(uint32_t);
    }
  }
  m_dyn_data_load = true;
  return AIPU_STATUS_SUCCESS;
}

aipu_status_t JobV3_2::setup_sg_priv_buffer() {
  uint32_t priv_offset = 0;
  SubGraphTask sg_task = {0};
  BufferDesc &desc = m_secbuf_desc[FMSection::TotalPriv];
  m_top_priv_buf = &desc;

  for (uint32_t sg_idx = 0; sg_idx < m_sg_cnt; sg_idx++) {
    const auto &sg = graph().get_subgraph(sg_idx);
    sg_task.reset(sg_idx, sg.bss_idx);

    if (sg.precursor_cnt == SUBG_DEPEND_PREALL)
      priv_offset = 0;

    /* each subgraph has private buffer core-accessed */
    for (uint32_t k = 0; k < sg.private_buffers.size(); k++) {
      const GraphSectionDesc &section_desc = sg.private_buffers[k];
      BufferDesc *bufferDesc = nullptr;

      if (section_desc.size == 0) {
        LOG(LOG_WARN, "sg: %u, private buffer: %d size == 0", sg_idx, k);
        continue;
      }

      bufferDesc = new BufferDesc;
      bufferDesc->reset();
      bufferDesc->init(desc.asid_base, desc.pa + priv_offset,
                       ALIGN_PAGE(section_desc.size), section_desc.size);
      priv_offset += ALIGN_PAGE(section_desc.size);

      if (m_dump_reuse)
        m_mem->mem_bzero(bufferDesc->pa, bufferDesc->size);

      sg_task.reuse_priv_buffers.push_back(bufferDesc);
    }
    m_sg_job.push_back(sg_task);
  }
  return AIPU_STATUS_SUCCESS;
}

aipu_status_t JobV3_2::setup_reuse_buffer() {
  uint32_t offset = 0;
  BufferDesc &desc = m_secbuf_desc[FMSection::TotalReuse];

  for (uint32_t k = 0; k < graph().get_bss(0).reuse_sections.size(); k++) {
    const GraphSectionDesc &section_desc = graph().get_bss(0).reuse_sections[k];
    BufferDesc *bufferDesc = nullptr;

    if (section_desc.size == 0) {
      LOG(LOG_WARN, "bss: %u reuse %u: size == 0", 0, k);
      continue;
    }

    std::string buf_name = "reuse_" + std::to_string(k);
    if (m_sfm_idxes.count(k) == 0 ||
        graph().is_gm_buffer(k, GM_BUF_TYPE_REUSE)) {
      bufferDesc = new BufferDesc;
      bufferDesc->reset();
      if (!graph().is_gm_buffer(k, GM_BUF_TYPE_REUSE)) {
        bufferDesc->init(desc.asid_base, desc.pa + offset,
                         ALIGN_PAGE(section_desc.size), section_desc.size);
        offset += ALIGN_PAGE(section_desc.size);
      }
    } else {
      aipu_status_t ret =
          m_mem->malloc(section_desc.size + graph().get_alloc_pad_size(),
                        section_desc.align_in_page, &bufferDesc,
                        buf_name.c_str(), m_sfm_mem_region);
      if (ret != AIPU_STATUS_SUCCESS) {
        LOG(LOG_ERR, "malloc from memory type %u fail", m_sfm_mem_region);
        return AIPU_STATUS_ERROR_BUF_ALLOC_FAIL;
      }

      if (bufferDesc->pa < m_top_job_buf->pa ||
          bufferDesc->pa + bufferDesc->size - m_top_job_buf->pa > 0xE0000000) {
        LOG(LOG_ERR,
            "please make sure sram address is in asid 3G/3.5G range, now sram "
            "pa 0x%lx, base pa 0x%lx",
            bufferDesc->pa, m_top_job_buf->pa);
        return AIPU_STATUS_ERROR_OP_NOT_SUPPORTED;
      }
      bufferDesc->asid_base = m_top_job_buf->asid_base;
      bufferDesc->align_asid_pa = bufferDesc->pa - bufferDesc->asid_base;
    }

    if (m_dump_reuse)
      m_mem->mem_bzero(bufferDesc->pa, bufferDesc->size);

    m_reuses_desc.push_back(bufferDesc);
  }

  return AIPU_STATUS_SUCCESS;
}

aipu_status_t JobV3_2::setup_sg_common_buffer() {
  uint32_t reuse_sg_id = 0;
  uint32_t head_cnt = 0;
  bool dep_all_flag = false;

  BufferDesc &stack_desc = m_secbuf_desc[FMSection::Stack];
  BufferDesc &dp_desc = m_secbuf_desc[FMSection::Dpdata];
  DEV_PA_64 stack_pa = stack_desc.pa;
  DEV_PA_64 dp_pa = dp_desc.pa;
  for (uint32_t sg_id = 0; sg_id < m_sg_cnt; ++sg_id) {
    SubGraphTask &sg_task = m_sg_job[sg_id];
    get_tcb_head_cnt(sg_id, head_cnt);

    if (graph().get_subgraph(sg_id).precursor_cnt == SUBG_DEPEND_PREALL) {
      reuse_sg_id = 0;
      dep_all_flag = true;
    }

    if (dep_all_flag && reuse_sg_id < m_sgt_allocated.size()) {
      for (uint32_t task_id = 0; task_id < m_task_per_sg; ++task_id) {
        Task task;
        memset((void *)&task, 0, sizeof(task));

        task = m_sgt_allocated[reuse_sg_id]->tasks[task_id];
        task.tcb.init(m_init_tcb.pa +
                      (head_cnt + sg_id * m_task_per_sg + task_id) *
                          sizeof(tcb_t));
        sg_task.tasks.push_back(task);
      }

      reuse_sg_id++;
      continue;
    } else {
      dep_all_flag = false;
    }

    /* init per-task data structs */
    for (uint32_t task_id = 0; task_id < m_task_per_sg; ++task_id) {
      Task task;
      memset((void *)&task, 0, sizeof(task));

      /* 1. init task tcb */
      task.tcb.init(m_init_tcb.pa +
                    (head_cnt + sg_id * m_task_per_sg + task_id) *
                        sizeof(tcb_t));

      /* 2. task stack */
      task.stack = new BufferDesc;
      uint32_t aligned_size = AIPU_ALIGN_BYTES(
          graph().get_bss(0).stack_size,
          graph().get_bss(0).stack_align_in_page * AIPU_PAGE_SIZE);
      task.stack->init(stack_desc.asid_base, stack_pa,
                       graph().get_bss(0).stack_size,
                       graph().get_bss(0).stack_size);
      stack_pa += aligned_size;

      /* 3. task dp */
      uint32_t dp_size = graph().get_subgraph(sg_id).private_data_size;
      if (dp_size != 0) {
        task.private_data = new BufferDesc;
        task.private_data->init(dp_desc.asid_base, dp_pa, dp_size, dp_size);
        m_mem->mem_bzero(task.private_data->pa, task.private_data->size);
        dp_pa += ALIGN_PAGE(dp_size);
      }
      sg_task.tasks.push_back(task);
    }
    m_sgt_allocated.push_back(&sg_task);
  }

  return AIPU_STATUS_SUCCESS;
}

aipu_status_t JobV3_2::alloc_load_job_buffers() {
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
    BufferDesc desc;
    if (info.first == FMSection::ReservedIOVA) {
      DEV_PA_64 pa =
          m_top_job_buf->pa + m_top_job_buf->size; /* pa from jobbuf end */
      /* different job maybe have different iova size */
      desc.init(m_top_job_buf->pa, pa, m_reserved_iova_size,
                m_reserved_iova_size);
    } else if (info.first == FMSection::TcbChain) {
      DEV_PA_64 pa = m_top_job_buf->pa + info.second.offset;
      uint32_t tail_id = ((pa + info.second.size) >> 12) & 0x3FF;
      uint32_t offset = ((m_grid_id & 0x3FF) - tail_id + 0x400) & 0x3FF;
      pa += offset * AIPU_PAGE_SIZE;
      desc.init(m_top_job_buf->pa, pa, info.second.size, info.second.size);
    } else {
      DEV_PA_64 pa = m_top_job_buf->pa + info.second.offset;
      desc.init(m_top_job_buf->pa, pa, info.second.size, info.second.size);
    }
    m_secbuf_desc[info.first] = desc;
    std::string name = graph().get_section_name(info.first).c_str();
    m_mem->add_tracking(desc.pa, desc.size, MemOperationSub, name.c_str(),
                        false, 0);
  }

  /* 1. load and assign buffers */
  ret = load_job_buffers();
  if (ret != AIPU_STATUS_SUCCESS)
    goto finish;

  /* 2. get IO buffer address, all subgraphs share the same copy of reuse
   * buffers */
  create_io_buffers(graph().get_bss(0).io, m_reuses_desc);
  if (m_sg_cnt == 0)
    goto finish;

  /* 3. setup rodata & dcr, update entry for all subgraphs in global RO/DCR
   * section */
  ret = setup_rodata_sg(graph().get_bss(0).param_map, m_reuses_desc,
                        m_weight[0].wb_weights);
  if (ret != AIPU_STATUS_SUCCESS)
    goto finish;

  /* update subgraph private buffers PA in RO/DCR section */
  for (uint32_t sg_idx = 0; sg_idx < m_sg_cnt; sg_idx++) {
    std::vector<BufferDesc *> invalid_buf;
    const auto &param_map = graph().get_subgraph(sg_idx).private_buffers_map;
    ret = setup_rodata_sg(param_map, m_sg_job[sg_idx].reuse_priv_buffers,
                          invalid_buf);
    if (ret != AIPU_STATUS_SUCCESS)
      goto finish;
  }

  /* 4. setup remap */
  setup_remap(m_secbuf_desc[FMSection::Rodata], &m_secbuf_desc[FMSection::Dcr]);

  /* 5. parse SegMMU config */
  ret = setup_segmmu(m_sg_job[0]);
  if (ret != AIPU_STATUS_SUCCESS)
    goto finish;

finish:
  if (ret) {
    for (uint32_t i = 0; i < m_sg_job.size(); i++)
      free_sg_buffers(m_sg_job[i]);

    free_job_buffers();
  }
  return ret;
}

aipu_status_t JobV3_2::load_job_buffers() {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;

  m_weight = graph().get_weight_buffer_info();

  if (m_secbuf_desc[FMSection::Text].size != 0) {
    m_mem->write(m_secbuf_desc[FMSection::Text].pa, graph().m_btext.va,
                 graph().m_btext.size);
    m_text = &m_secbuf_desc[FMSection::Text];
  }

  if (m_secbuf_desc[FMSection::Crodata].size != 0) {
    m_mem->write(m_secbuf_desc[FMSection::Crodata].pa, graph().m_bcrodata.va,
                 graph().m_bcrodata.size);
    m_crodata = &m_secbuf_desc[FMSection::Crodata];
  }

  ret = setup_reuse_buffer();
  if (ret != AIPU_STATUS_SUCCESS)
    return ret;

  if (m_secbuf_desc[FMSection::GM].size != 0)
    setup_gm_buffer();

  if (!graph().m_put_weight_gm &&
      m_secbuf_desc[FMSection::ZcyConst].size != 0) {
    auto &desc = m_secbuf_desc[FMSection::ZcyConst];
    DEV_PA_64 pa_base = desc.pa;
    for (uint32_t i = 0; i < graph().get_bss_cnt(); ++i) {
      uint32_t zcy_size = graph().get_zerocpy_const_size(i);
      BufferDesc *zcy = new BufferDesc;
      zcy->init(desc.asid_base, pa_base, zcy_size, zcy_size);
      m_weight[i].wb_zerocpy_const = zcy;
      pa_base += ALIGN_PAGE(zcy_size);
    }
    ret = graph().setup_zcy_buffer(m_weight);
    if (ret != AIPU_STATUS_SUCCESS)
      return ret;
  }

  if (m_secbuf_desc[FMSection::ModelParam].size != 0) {
    ret = setup_dyn_shape_buffer();
    if (ret != AIPU_STATUS_SUCCESS)
      return ret;
    m_model_global_param = &m_secbuf_desc[FMSection::ModelParam];
  }

  if (m_secbuf_desc[FMSection::Rodata].size != 0) {
    m_mem->write(m_secbuf_desc[FMSection::Rodata].pa, graph().m_brodata.va,
                 graph().m_brodata.size);
    m_rodata = &m_secbuf_desc[FMSection::Rodata];
  }

  if (m_secbuf_desc[FMSection::Dcr].size != 0) {
    m_mem->write(m_secbuf_desc[FMSection::Dcr].pa, graph().m_bdesc.va,
                 graph().m_bdesc.size);
    m_descriptor = &m_secbuf_desc[FMSection::Dcr];
  }

  if (m_secbuf_desc[FMSection::Printf].size != 0) {
    m_pprint = &m_secbuf_desc[FMSection::Printf];
  }

  m_mem->zeroize(m_secbuf_desc[FMSection::TcbChain].pa,
                 m_tot_tcb_cnt * sizeof(tcb_t));
  m_init_tcb.init(m_secbuf_desc[FMSection::TcbChain].pa);
  m_tcbs = &m_secbuf_desc[FMSection::TcbChain];

  ret = setup_sg_priv_buffer();
  if (ret != AIPU_STATUS_SUCCESS)
    return ret;

  if (m_secbuf_desc[FMSection::Coredump].size != 0)
    m_coredump->setup_coredump_buffer(m_secbuf_desc[FMSection::Coredump]);

  ret = setup_sg_common_buffer();
  if (ret != AIPU_STATUS_SUCCESS)
    return ret;

  return AIPU_STATUS_SUCCESS;
}

void JobV3_2::free_sg_buffers(SubGraphTask &sg_task) {
  for (uint32_t i = 0; i < sg_task.reuse_priv_buffers.size(); i++)
    m_mem->free_bufferdesc(&sg_task.reuse_priv_buffers[i]);
  sg_task.reuse_priv_buffers.clear();

  for (uint32_t i = 0; i < m_sgt_allocated.size(); i++) {
    for (uint32_t j = 0; j < m_task_per_sg; j++) {
      Task *task;
      task = &m_sgt_allocated[i]->tasks[j];
      m_mem->free_bufferdesc(&task->stack);
      m_mem->free_bufferdesc(&task->private_data);
    }
  }
  m_sgt_allocated.clear();
}

aipu_status_t JobV3_2::free_job_buffers() {
  for (uint32_t bss_id = 0; bss_id < graph().get_bss_cnt(); ++bss_id) {
    if (bss_id >= m_weight.size())
      break;

    if (m_weight[bss_id].wb_zerocpy_const != nullptr &&
        m_weight[bss_id].wb_zerocpy_const->size != 0)
      m_mem->free_bufferdesc(&m_weight[bss_id].wb_zerocpy_const);

    if (graph().m_put_weight_gm) {
      if (m_weight[bss_id].wb_weight != nullptr &&
          m_weight[bss_id].wb_weight->size != 0)
        m_mem->free_bufferdesc(&m_weight[bss_id].wb_weight);
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
    tcb.flag |= TCB_FLAG_DEP_TYPE_NONE;
    break;

  case 1 ... 4: {
    uint16_t dep_group_id = 0;

    tcb.flag |= TCB_FLAG_DEP_TYPE_GROUP;
    for (int32_t i = 0; i < graph().get_subgraph(sg_id).precursor_cnt; i++) {
      if (graph().get_subgraph(sg_id).precursors[i] > 0x7fff) {
        LOG(LOG_ERR, "Depend group id(%d) is invalid",
            graph().get_subgraph(sg_id).precursors[i]);
        return AIPU_STATUS_ERROR_INVALID_GBIN;
      }

      dep_group_id =
          graph().get_subgraph(sg_id).precursors[i] + m_start_group_id;
      dep_group_id &= 0x7FFF; // 15 bits group id field
      tcb.group_deps[i] = EN_GROUP_DEPEND | dep_group_id;
    }
  } break;

  case SUBG_DEPEND_PREALL:
    tcb.flag |= TCB_FLAG_DEP_TYPE_PRE_ALL;
    break;

  default:
    LOG(LOG_ERR, "subgraph %u, precursor_cnt=%d", sg_id,
        graph().get_subgraph(sg_id).precursor_cnt);
    return AIPU_STATUS_ERROR_INVALID_GBIN;
  }

  if (m_sg_cnt == graph().get_bss_cnt() &&
      TCB_FLAG_DEP_TYPE(tcb.flag) == TCB_FLAG_DEP_TYPE_PRE_ALL) {
    /* only depends last sg instead of all cores, if first sg, depend none */
    tcb.flag &= ~TCB_FLAG_DEP_TYPE_PRE_ALL;
    if (sg_id != 0) {
      tcb_t last_sg;
      Task &task = m_sg_job[sg_id - 1].tasks[0];
      m_mem->read(task.tcb.pa, &last_sg, sizeof(tcb_t));
      tcb.flag |= TCB_FLAG_DEP_TYPE_GROUP;
      tcb.group_deps[0] = EN_GROUP_DEPEND | last_sg.task.group_id;
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
  tcb.task.interrupt_en = EN_INTERRUPT_TEC_SIGNAL | EN_INTERRUPT_TEC_EXCEPTION |
                          EN_INTERRUPT_TEC_FAULT;
  tcb.flag = TCB_FLAG_TASK_TYPE_TASK;

  if (task_id == (m_task_per_sg - 1))
    tcb.flag |= TCB_FLAG_END_TYPE_GROUP_END;

  /* v3_2 last tcb must have a grid end, although we don't need a cluster
   * interrupt */
  if ((sg_id == (m_sg_cnt - 1)) && (task_id == (m_task_per_sg - 1)))
    tcb.flag |= TCB_FLAG_END_TYPE_GRID_END;

  /**
   * depend_all: set init group tcb and first tcb of tcb group
   * depend_groups: set init group and all tasks of tcb group
   */
  ret = config_tcb_deps(tcb, sg_id);
  if (ret != AIPU_STATUS_SUCCESS)
    return ret;
  if (task_id != 0 && TCB_FLAG_DEP_TYPE(tcb.flag) == TCB_FLAG_DEP_TYPE_PRE_ALL)
    tcb.flag &= ~TCB_FLAG_DEP_TYPE_PRE_ALL;

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
    tcb.task.interrupt_en |= EN_INTERRUPT_TEC_SIGNAL;
  }

  if (graph().is_dynamic_shape())
    tcb.task.global_param = get_low_32(m_model_global_param->align_asid_pa);

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
  tcb.flag = TCB_FLAG_TASK_TYPE_GRID_INIT | TCB_FLAG_L2D_FLUSH;
  tcb.grid.group_num = m_sg_cnt;
  tcb.grid.grid_id = m_grid_id;
  tcb.grid.group_id = m_group_id_idx;
  tcb.grid.interrupt_en = EN_INTERRUPT_GRID_ALL;
  if (m_sg_cnt == graph().get_bss_cnt())
    tcb.grid.interrupt_en &= ~EN_INTERRUPT_GRID_DONE;

  m_gm->setup_gm_sync_from_ddr(tcb);
  m_mem->write(m_init_tcb.pa, (const char *)&tcb, sizeof(tcb_t));

  for (uint32_t i = 0; i < graph().get_subgraph_cnt(); i++) {
    /* Group init TCB */
    memset(&tcb, 0, sizeof(tcb_t));
    tcb.flag = TCB_FLAG_TASK_TYPE_GROUP_INIT | TCB_FLAG_GRID_INIT;
    tcb.group.grid_id = m_grid_id;
    tcb.group.group_id = m_group_id_idx;
    if (m_sg_cnt == graph().get_bss_cnt())
      tcb.group.interrupt_en = EN_INTERRUPT_GROUP_DONE;

    // SegMMU
    // config_tcb_smmu(tcb);

    ret = config_tcb_deps(tcb, graph().get_subgraph(i).id);
    if (ret != AIPU_STATUS_SUCCESS)
      return ret;

    /**
     * ASID0: feature map buffer region
     * the whole graph share one copy of reuse buffer for feature map.
     */
    tcb.group.asids[0] =
        get_low_32(m_top_job_buf->asid_base | ZY_ASID_RD | ZY_ASID_WR);
    tcb.group.asids[1] = get_high_32(m_top_job_buf->asid_base);

    /**
     * ASID1: weight buffer region
     * if LLM model contains multiple BSSs, each BSS will locate in private
     * ASID1 region. so here, set ASID1 base register from weight buffer's
     * asid_base(pa).
     */
    DEV_PA_64 asid1_base = 0;
    uint32_t bss_idx = graph().get_subgraph(i).bss_idx;
    if (graph().get_weight_buffer_info().size() > bss_idx)
      asid1_base = graph().get_weight_buffer_info()[bss_idx].wb_asid_base;
    tcb.group.asids[2] = get_low_32(asid1_base | ZY_ASID_RD | ZY_ASID_WR);
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
JobV3_2::specify_io_buffer(aipu_shared_tensor_info_t &tensor_info) {
  /* shared buffer shouldn't reuse with any other reuse buffers */
  const char *str = "free_input";
  const std::vector<struct JobIOBuffer> *iobuffer_vec = nullptr;
  uint32_t type = tensor_info.type;
  uint32_t index = tensor_info.tensor_idx;
  switch (type) {
  case AIPU_TENSOR_TYPE_INPUT:
    iobuffer_vec = &m_inputs;
    if (!graph().get_disable_input_reuse()) {
      LOG(LOG_WARN, "if use shared input buffer, you'd better disable input "
                    "reuse at aipu compiler side");
    }
    break;

  case AIPU_TENSOR_TYPE_OUTPUT:
    iobuffer_vec = &m_outputs;
    if (!graph().get_disable_output_reuse()) {
      LOG(LOG_WARN, "if use shared output buffer, you'd better disable output "
                    "reuse at aipu compiler side");
    }
    str = "free_output";
    break;

  default:
    LOG(LOG_ERR, "tensor type: %d, index: %d [not exist]", type, index);
    return AIPU_STATUS_ERROR_INVALID_TENSOR_ID;
  }

  if (index >= iobuffer_vec->size())
    return AIPU_STATUS_ERROR_INVALID_TENSOR_ID;

  uint32_t reuse_index = (*iobuffer_vec)[index].ref_section_iter;
  if (type == AIPU_TENSOR_TYPE_INPUT) {
    for (uint32_t i = 0; i < graph().get_bss(0).io.outputs.size(); i++) {
      auto &idtensor_desc = graph().get_bss(0).io.outputs[i];
      if (idtensor_desc.ref_section_iter == reuse_index)
        return AIPU_STATUS_ERROR_DMABUF_SHARED_IO;
    }
  } else {
    for (uint32_t i = 0; i < graph().get_bss(0).io.inputs.size(); i++) {
      auto &idtensor_desc = graph().get_bss(0).io.inputs[i];
      if (idtensor_desc.ref_section_iter == reuse_index)
        return AIPU_STATUS_ERROR_DMABUF_SHARED_IO;
    }
  }

  /* free io buffer allocated internally,replace it with new buffer */
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  BufferDesc *bufferDesc = m_reuses_desc[reuse_index];
  m_dmabuf_idxs.insert(reuse_index);
  if (!m_optimized_reuse_alloc) {
    ret = m_mem->free_phybuffer(bufferDesc, str);
    if (ret != AIPU_STATUS_SUCCESS)
      return ret;
  }

  int share_case_type = tensor_info.shared_case_type;
  uint64_t buffer_pa = tensor_info.pa;
  int fd = tensor_info.dmabuf_fd;
  struct aipu_dma_buf dma_buf {
    fd, 0, 0
  };
  switch (share_case_type) {
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

  LOG(LOG_DEBUG, "specify_io_buffer: pa=%lx, size=%lx, share_case_type=%d",
      buffer_pa, bufferDesc->size, share_case_type);

  ret = setup_rodata_sg(graph().get_bss(0).param_map, m_reuses_desc,
                        graph().get_weight_buffer_info()[0].wb_weights,
                        &m_dmabuf_idxs);
  if (ret != AIPU_STATUS_SUCCESS)
    return ret;

  return AIPU_STATUS_SUCCESS;
}

aipu_status_t JobV3_2::dump_for_emulation() {
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
      {0, "0M"},         {512 << 10, "512K"}, {1 << 20, "1M"},
      {2 << 20, "2M"},   {4 << 20, "4M"},     {8 << 20, "8M"},
      {16 << 20, "16M"}, {32 << 20, "32M"},   {64 << 20, "64M"},
  };

  if (m_dump_emu == false)
    return AIPU_STATUS_SUCCESS;

  FileWrapper ofs(runtime_cfg, std::ios_base::in | std::ios_base::out |
                                   std::ios_base::trunc);
  FileWrapper ofsmt(metadata_txt, std::ios_base::in | std::ios_base::out |
                                      std::ios_base::trunc);
  if (!ofs.is_open() || !ofsmt.is_open())
    return AIPU_STATUS_ERROR_OPEN_FILE_FAIL;

  ofs << "[COMMON]\n";

  /* runtime.cfg: config */
  ofs << "#configuration K:cluster,C:core,A:aiff,T:tec \n";
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
  ofs << "#GM_V3_2 support typical: "
         "0KB,512KiB,1MiB,2MiB,4MiB,8MiB,16MiB,32MiB,64MiB.\n";
  if (gm_info.count(m_cfg->gm_size) == 1)
    ofs << "GM_SIZE=" << gm_info[m_cfg->gm_size] << "\n";

  if (m_cfg->plugin_name != nullptr) {
    ofs << "#PLUGIN_FILENAME\n";
    ofs << "PLUGIN_FILENAME=" << m_cfg->plugin_name << "\n";
  }

  /* runtime.cfg: en_eval */
  if (m_dev->get_profile_en()) {
    ofs << "\n[PROFILE]\n";
    ofs << "#perf mode: 0:None,1:fast mode,2:eval,3:idu,4:probe\n";
    ofs << "MODE=1\n";
    ofs << "SYS_FREQ_MHZ=" << m_cfg->freq_mhz << "\n";
    ofs << "DDR_RD_LATENCY=" << m_cfg->ddr_latency_rd << "\n";
    ofs << "DDR_WR_LATENCY=" << m_cfg->ddr_latency_wr << "\n";
    ofs << "DDR_BW_BITS=" << m_cfg->ddr_bw << "\n";

    if (m_cfg->perf_report != nullptr)
      ofs << "REPORT_FILENAME=" << m_cfg->perf_report << "\n";

    if (m_profiler.size() == 1) {
      ofs << "PROFILE_BUF_ADDR=0x" << std::hex << m_profiler[0].pa << "\n";
      ofs << "PROFILE_BUF_SIZE=0x" << std::hex << m_profiler[0].size << "\n";
    }

    if (m_cfg->json_filename != nullptr) {
      ofs << "#GRPAH_FILENAME\n";
      ofs << "GRPAH_FILENAME=" << m_cfg->json_filename << "\n";
    }
  }
  ofs << "\n";

  ofs.dump_to_string(m_dumpcfg_header);

  /* runtime.cfg: [INPUT] */
  for (uint32_t bss_id = 0; bss_id < graph().get_weight_buffer_info().size();
       bss_id++) {
    if (graph().get_weight_buffer_info()[bss_id].wb_weight != nullptr &&
        graph().get_weight_buffer_info()[bss_id].wb_weight->size > 0) {
      emu_input_cnt += 1;
      if (graph().get_weight_buffer_info()[bss_id].wb_zerocpy_const !=
              nullptr &&
          graph().get_weight_buffer_info()[bss_id].wb_zerocpy_const->size != 0)
        emu_input_cnt += 1;
    } else
      emu_input_cnt +=
          graph().get_weight_buffer_info()[bss_id].wb_weights.size();
  }

  ofs << "[INPUT]\n";
  ofs << "COUNT=" << emu_input_cnt << "\n";

  /* dump temp.text */
  dump_pa = m_text->pa;
  dump_size = m_text->req_size - 16;
  if (dump_size != 0) {
    snprintf(dump_name, 128, "%s/%s.text", m_dump_dir.c_str(),
             m_dump_prefix.c_str());
    m_mem->dump_file(dump_pa, dump_name, dump_size);

    ofs << "FILE" << std::dec << ++file_id << "=" << m_dump_prefix << ".text\n";
    ofs << "BASE" << file_id << "=0x" << std::hex << dump_pa << "\n";
    m_dumpcfg_input.push_back({dump_name, dump_pa});
  }

  /* dump temp.weight */
  for (uint32_t bss_id = 0; bss_id < graph().get_weight_buffer_info().size();
       bss_id++) {
    if (graph().get_weight_buffer_info()[bss_id].wb_weight != nullptr &&
        graph().get_weight_buffer_info()[bss_id].wb_weight->req_size > 0) {
      dump_pa = graph().get_weight_buffer_info()[bss_id].wb_weight->pa;
      dump_size = graph().get_weight_buffer_info()[bss_id].wb_weight->req_size;
      if (dump_size != 0) {
        snprintf(dump_name, 128, "%s/%s.weight", m_dump_dir.c_str(),
                 m_dump_prefix.c_str());
        m_mem->dump_file(dump_pa, dump_name, dump_size);

        ofs << "FILE" << std::dec << ++file_id << "=" << m_dump_prefix
            << ".weight\n";
        ofs << "BASE" << file_id << "=0x" << std::hex << dump_pa << "\n";
        m_dumpcfg_input.push_back({dump_name, dump_pa});

        if (graph().get_weight_buffer_info()[bss_id].wb_zerocpy_const !=
                nullptr &&
            graph().get_weight_buffer_info()[bss_id].wb_zerocpy_const->size >
                0) {
          dump_pa =
              graph().get_weight_buffer_info()[bss_id].wb_zerocpy_const->pa;
          dump_size = graph()
                          .get_weight_buffer_info()[bss_id]
                          .wb_zerocpy_const->req_size;
          snprintf(dump_name, 128, "%s/%s.zerocpy_const", m_dump_dir.c_str(),
                   m_dump_prefix.c_str());
          m_mem->dump_file(dump_pa, dump_name, dump_size);

          ofs << "FILE" << std::dec << ++file_id << "=" << m_dump_prefix
              << ".zerocpy_const\n";
          ofs << "BASE" << file_id << "=0x" << std::hex << dump_pa << "\n";
          m_dumpcfg_input.push_back({dump_name, dump_pa});
        }
      }
    } else {
      for (uint32_t i = 0;
           i < graph().get_weight_buffer_info()[bss_id].wb_weights.size();
           i++) {
        dumpcfg_input_desc input_desc;
        std::string name;

        dump_pa = graph().get_weight_buffer_info()[bss_id].wb_weights[i]->pa;
        dump_size =
            graph().get_weight_buffer_info()[bss_id].wb_weights[i]->size;
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
  dump_size = m_rodata->req_size;
  snprintf(dump_name, 128, "%s/%s.ro", m_dump_dir.c_str(),
           m_dump_prefix.c_str());
  m_mem->dump_file(dump_pa, dump_name, dump_size);
  ofs << "FILE" << std::dec << ++file_id << "=" << m_dump_prefix << ".ro\n";
  ofs << "BASE" << file_id << "=0x" << std::hex << dump_pa << "\n";
  m_dumpcfg_input.push_back({dump_name, dump_pa});

  /* dump temp.dcr */
  if (m_descriptor != nullptr && m_descriptor->size != 0) {
    dump_pa = m_descriptor->pa;
    dump_size = m_descriptor->req_size;
    snprintf(dump_name, 128, "%s/%s.dcr", m_dump_dir.c_str(),
             m_dump_prefix.c_str());
    m_mem->dump_file(dump_pa, dump_name, dump_size);

    ofs << "FILE" << std::dec << ++file_id << "=" << m_dump_prefix << ".dcr\n";
    ofs << "BASE" << file_id << "=0x" << std::hex << dump_pa << "\n";
    m_dumpcfg_input.push_back({dump_name, dump_pa});
  }

  /* dump temp.tcb */
  dump_pa = m_tcbs->pa;
  dump_size = m_tot_tcb_cnt * sizeof(tcb_t);
  snprintf(dump_name, 128, "%s/%s.tcb", m_dump_dir.c_str(),
           m_dump_prefix.c_str());
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
  ofs << "TCB_NUM=0x" << std::hex << m_tot_tcb_cnt << "\n";
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

  /* Grid/Group init TCB and Task TCB */
  for (uint32_t i = 0; i < m_tot_tcb_cnt; i++) {
    m_mem->read(m_init_tcb.pa + sizeof(tcb_t) * i, &tcb, sizeof(tcb_t));

    if (TCB_FLAG_TASK_TYPE(tcb.flag) == TCB_FLAG_TASK_TYPE_GRID_INIT) {
      ofsmt << "\n***GRID INIT TCB " << std::dec << i << " ***\n";

      ofsmt << "flag: 0x" << std::hex << tcb.flag << "\n";
      ofsmt << "group_num: " << std::dec << tcb.grid.group_num << "\n";
      ofsmt << "grid_intrrupt_en: 0x" << std::hex << tcb.grid.interrupt_en
            << "\n";
      ofsmt << "grid_groupid: " << std::dec << tcb.grid.group_id << "\n";
      ofsmt << "grid_gridid: " << tcb.grid.grid_id << "\n";
      ofsmt << "gm_ctrl: 0x" << std::hex << tcb.grid.gm_ctrl << "\n";
      ofsmt << "gm_sync: 0x" << tcb.grid.gm_sync << "\n";
      ofsmt << "gm_addr_low: 0x" << tcb.grid.gm_addr_low << "\n";
      ofsmt << "gm_addr_high: 0x" << tcb.grid.gm_addr_high << "\n";
    } else if (TCB_FLAG_TASK_TYPE(tcb.flag) == TCB_FLAG_TASK_TYPE_GROUP_INIT) {
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
    } else if (TCB_FLAG_TASK_TYPE(tcb.flag) == TCB_FLAG_TASK_TYPE_TASK) {
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
// Copyright (C) 2023-2025 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

/**
 * @file  job_v3x.cpp
 * @brief AIPU User Mode Driver (UMD) aipu v3x job module implementation
 */

#include "job_v3x.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <cstring>
#include <mutex>

#include "context.h"
#include "utils/helper.h"

namespace aipudrv {
JobV3X::JobV3X(MainContext *ctx, GraphBase &graph, DeviceBase *dev,
               aipu_create_job_cfg_t *config)
    : JobBase(ctx, graph, dev) {
  if (config != nullptr) {
    m_partition_id = config->partition_id;
    m_qos = config->qos_level;
    m_sfm_mem_region = config->fm_mem_region;
    m_reserved_iova_size = config->reserved_iova_size;
    m_dev->get_core_count(0, 0, &m_core_cnt);

    if (config->dbg_dispatch) /* will be deprecated in the future */
    {
      m_bind_enable = true;
      m_core_id = config->dbg_core_id;
      m_cluster_id = 0;

      if (m_core_id >= m_core_cnt) {
        LOG(LOG_WARN, "bind core id %u, but core cnt %u, set core id as 0",
            m_core_id, m_core_cnt);
        m_core_id = 0;
      }

      if (this->graph().get_isa() > ISAv5) {
        m_core_id = 1 << m_core_id;
      }
    }

    if (config->bind_enable) {
      m_bind_enable = true;
      m_core_id = config->bind_core_ids;
      m_cluster_id = 0; /* current only support cluster0 */

      if (m_core_id == 0 || m_core_id >= pow(2, m_core_cnt)) {
        LOG(LOG_WARN, "wrong bind core id 0x%x, set to core0", m_core_id);
        m_core_id = 0x01;
      }

      /* x2 only support one core binding */
      if (this->graph().get_isa() == ISAv5) {
        uint32_t val = 0;
        while ((m_core_id & 1) == 0) {
          m_core_id >>= 1;
          val++;
        }
        m_core_id = val;
        if (m_core_id >= m_core_cnt) {
          LOG(LOG_WARN, "bind core id %u, but core cnt %u, set core id as 0",
              m_core_id, m_core_cnt);
          m_core_id = 0;
        }
      }
    }

    if (config->fm_idxes) {
      for (int i = 0; i < config->fm_idxes_cnt; i++)
        m_sfm_idxes.insert(config->fm_idxes[i]);
    }
  }

  m_init_tcb.init(0);
  m_sgt_allocated.clear();

#if defined(SIMULATION)
  /**
   * initialize an invalid cmdpool id, set it on scheduling to sim
   */
  m_bind_cmdpool_id = 0xffffffff;
#endif

  m_segmmu_num = this->graph().m_segmmu_num;

  /* also support set dynamic params through aipu_config_job() */
  if (this->graph().is_dynamic_shape()) {
    if (config != nullptr) {
      if (config->dynshape == nullptr)
        m_dyn_shape =
            new DynamicShape(*this, this->graph(), config->dynshape_params);
      else
        m_dyn_shape = new DynamicShape(*this, this->graph(), config->dynshape);
    } else {
      LOG(LOG_WARN,
          "graph is dynamic, you'd better provide input shape information");
    }
  }

#ifndef SIMULATION
  if (this->graph().m_coredump_en) {
    m_coredump = new Coredump(this, dev);
    if (m_coredump->init() != AIPU_STATUS_SUCCESS) {
      LOG(LOG_WARN, "coredump initializes failed!");
    }
  }
#endif
}

JobV3X::~JobV3X() {
  if (m_gm != nullptr) {
    delete m_gm;
    m_gm = nullptr;
  }

  if (m_dyn_shape != nullptr) {
    delete m_dyn_shape;
    m_dyn_shape = nullptr;
  }

  if (m_coredump != nullptr) {
    delete m_coredump;
    m_coredump = nullptr;
  }
}

aipu_status_t
JobV3X::setup_rodata_sg(const std::vector<GraphParamMapLoadDesc> &param_map,
                        std::vector<BufferDesc *> &reuse_buf,
                        std::vector<BufferDesc *> &static_buf,
                        std::set<uint32_t> *dma_buf_idx) {
  BufferDesc rodata;
  rodata.init(0, m_rodata->pa, m_rodata->size, m_rodata->req_size);
  if (m_descriptor != nullptr) {
    BufferDesc dcr;
    dcr.init(0, m_descriptor->pa, m_descriptor->size, m_descriptor->req_size);
    return setup_rodata(param_map, reuse_buf, static_buf, rodata, &dcr,
                        dma_buf_idx);
  }
  return setup_rodata(param_map, reuse_buf, static_buf, rodata, nullptr,
                      dma_buf_idx);
}

aipu_status_t
JobV3X::config_dynamic_params(const aipu_dynshape_param_t *params) {
  if (params == nullptr) {
    LOG(LOG_ERR, "arguments have nullptr");
    return AIPU_STATUS_ERROR_NULL_PTR;
  }

  if (!graph().is_dynamic_shape()) {
    LOG(LOG_ERR, "job 0x%lx is not belong to dynamic graph", m_id);
    return AIPU_STATUS_ERROR_OP_NOT_SUPPORTED;
  }

  aipu_status_t ret = m_dyn_shape->set_dynamic_shape_data(params);
  if (ret != AIPU_STATUS_SUCCESS)
    return ret;

  update_single_io_buffers(graph().get_bss(0).io.inputs, m_inputs,
                           m_reuses_desc);

  return setup_dyn_shape_buffer(m_global_param);
}

aipu_status_t JobV3X::setup_reuse_buffer(const BufferDesc *desc) {
  if (desc == nullptr)
    return AIPU_STATUS_ERROR_NULL_PTR;

  m_optimized_reuse_alloc = true;

  uint32_t offset = 0;
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
        bufferDesc->init(desc->asid_base, desc->pa + offset,
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

      if (bufferDesc->pa < desc->asid_base ||
          bufferDesc->pa + bufferDesc->size - desc->asid_base > 0xE0000000) {
        LOG(LOG_ERR,
            "please make sure sram address is in asid 3G/3.5G range, now sram "
            "pa 0x%lx, base pa 0x%lx",
            bufferDesc->pa, desc->asid_base);
        return AIPU_STATUS_ERROR_OP_NOT_SUPPORTED;
      }
      bufferDesc->asid_base = desc->asid_base;
      bufferDesc->align_asid_pa = bufferDesc->pa - desc->asid_base;
    }

    if (m_dump_reuse)
      m_mem->mem_bzero(bufferDesc->pa, bufferDesc->size);

    m_reuses_desc.push_back(bufferDesc);
  }

  return AIPU_STATUS_SUCCESS;
}

aipu_status_t JobV3X::setup_gm_buffer(const BufferDesc *desc) {
  if (desc == nullptr)
    return AIPU_STATUS_ERROR_NULL_PTR;

  const auto &gm_info = graph().get_gmsec_info();

  if (graph().m_put_weight_gm) {
    m_weight.clear();
    m_weight.resize(graph().get_bss_cnt());
    DEV_PA_64 pa_base =
        desc->pa + gm_info.info.at(GMBufType::GM_BUF_TYPE_WEIGHT).offset;
    for (uint32_t i = 0; i < graph().get_bss_cnt(); ++i) {
      uint32_t weight_size = graph().get_const_size(i);
      BufferDesc *weight = new BufferDesc;
      weight->init(desc->asid_base, pa_base, weight_size, weight_size);
      m_weight[i].wb_weight = weight;
      pa_base += ALIGN_PAGE(weight_size);

      uint32_t zcy_size = graph().get_zerocpy_const_size(i);
      BufferDesc *zcy = new BufferDesc;
      zcy->init(desc->asid_base, pa_base, zcy_size, zcy_size);
      m_weight[i].wb_zerocpy_const = zcy;
      pa_base += ALIGN_PAGE(zcy_size);

      m_weight[i].wb_asid_base = desc->asid_base;
    }
    graph().setup_weight_buffer(m_weight, true /* setup_zcy */);
  }

  if (graph().m_put_desc_gm) {
    BufferDesc *sec_desc = new BufferDesc;
    const auto &info =
        graph().get_gmsec_info().info.at(GMBufType::GM_BUF_TYPE_DESCRIPTOR);
    sec_desc->init(desc->asid_base, desc->pa + info.offset, info.size,
                   info.size);
    m_secbuf_desc[FMSection::Dcr] = sec_desc;
  }

  if (gm_info.info.count(GMBufType::GM_BUF_TYPE_REUSE) != 0) {
    uint32_t idx =
        graph().m_gm_config_desc[GMBufType::GM_BUF_TYPE_REUSE].begin()->first;
    const auto &info = gm_info.info.at(GMBufType::GM_BUF_TYPE_REUSE);
    if (idx < m_reuses_desc.size()) {
      m_reuses_desc[idx]->asid_base = desc->asid_base;
      m_reuses_desc[idx]->pa = desc->pa + info.offset;
      m_reuses_desc[idx]->align_asid_pa =
          desc->pa + info.offset - desc->asid_base;
      m_reuses_desc[idx]->size = info.size;
      m_reuses_desc[idx]->req_size = info.size;
    }

    BufferDesc sec_desc;
    sec_desc.init(desc->asid_base, desc->pa + info.offset, info.size,
                  info.size);
    aipu_status_t ret =
        m_gm->setup_buffer(0, idx, GMBufType::GM_BUF_TYPE_REUSE, &sec_desc);
    if (ret != AIPU_STATUS_SUCCESS)
      return ret;
  }

  if (graph().m_put_ws_gm) {
    BufferDesc *sec_desc = new BufferDesc;
    const auto &info =
        graph().get_gmsec_info().info.at(GMBufType::GM_BUF_TYPE_WORKSPACE);
    sec_desc->init(desc->asid_base, desc->pa + info.offset, info.size,
                   info.size);
    m_secbuf_desc[FMSection::TotalPriv] = sec_desc;
  }

  if (get_coredump() != nullptr) {
    uint32_t gm_id = 0;
    if (graph().get_isa() == ISAv5 && m_qos == AIPU_JOB_QOS_HIGH)
      gm_id = 1;

    aipu_status_t ret = get_coredump()->set_gm_info(gm_id, desc);
    if (ret != AIPU_STATUS_SUCCESS) {
      LOG(LOG_ERR, "set coredump gm information fail");
      return ret;
    }
  }

  return AIPU_STATUS_SUCCESS;
}

aipu_status_t JobV3X::setup_dyn_shape_buffer(const BufferDesc *desc) {
  if (desc == nullptr)
    return AIPU_STATUS_ERROR_NULL_PTR;

  DSModelGlobalParam *param = (DSModelGlobalParam *)graph().m_bglobalparam.va;
  uint32_t input_shape_offset = param->input_shape_offset;

  DEV_PA_64 pa = desc->pa;
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

aipu_status_t JobV3X::setup_sg_priv_buffer(const BufferDesc *desc) {
  if (desc == nullptr)
    return AIPU_STATUS_ERROR_NULL_PTR;

  uint32_t priv_offset = 0;

  for (uint32_t sg_idx = 0; sg_idx < m_sg_cnt; sg_idx++) {
    const auto &sg = graph().get_subgraph(sg_idx);

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
      bufferDesc->init(desc->asid_base, desc->pa + priv_offset,
                       ALIGN_PAGE(section_desc.size), section_desc.size);
      priv_offset += ALIGN_PAGE(section_desc.size);

      if (m_dump_reuse)
        m_mem->mem_bzero(bufferDesc->pa, bufferDesc->size);

      m_sg_job[sg_idx].reuse_priv_buffers.push_back(bufferDesc);
    }
  }
  return AIPU_STATUS_SUCCESS;
}

aipu_status_t JobV3X::setup_sg_common_buffer(const BufferDesc *stack_desc,
                                             const BufferDesc *dp_desc) {
  uint32_t reuse_sg_id = 0;
  uint32_t head_cnt = 0;
  bool dep_all_flag = false;

  DEV_PA_64 stack_pa = stack_desc != nullptr ? stack_desc->pa : 0;
  DEV_PA_64 dp_pa = dp_desc != nullptr ? dp_desc->pa : 0;
  for (uint32_t sg_id = 0; sg_id < m_sg_cnt; ++sg_id) {
    SubGraphTask &sg_task = m_sg_job[sg_id];
    head_cnt = get_tcb_head_cnt(sg_id, head_cnt);

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
                          tcb_ctl::TCB_LEN);
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
                        tcb_ctl::TCB_LEN);

      /* 2. task stack */
      task.stack = new BufferDesc;
      uint32_t aligned_size = AIPU_ALIGN_BYTES(
          graph().get_bss(0).stack_size,
          graph().get_bss(0).stack_align_in_page * AIPU_PAGE_SIZE);
      task.stack->init(stack_desc->asid_base, stack_pa,
                       graph().get_bss(0).stack_size,
                       graph().get_bss(0).stack_size);
      stack_pa += aligned_size;

      /* 3. task dp */
      uint32_t dp_size = graph().get_subgraph(sg_id).private_data_size;
      if (dp_size != 0) {
        task.private_data = new BufferDesc;
        task.private_data->init(dp_desc->asid_base, dp_pa, dp_size, dp_size);
        m_mem->mem_bzero(task.private_data->pa, task.private_data->size);
        dp_pa += ALIGN_PAGE(dp_size);
      }
      sg_task.tasks.push_back(task);
    }
    m_sgt_allocated.push_back(&sg_task);
  }

  return AIPU_STATUS_SUCCESS;
}

aipu_status_t JobV3X::setup_job_buffers() {
  /* TODO: split const&zct to avid copy */
  m_weight = graph().get_weight_buffer_info();

  /* share graph's text/crodata */
  m_text = graph().m_text;
  if (m_text == nullptr && graph().m_btext.size != 0) {
    m_mem->write(m_secbuf_desc[FMSection::Text]->pa, graph().m_btext.va,
                 graph().m_btext.size);
    m_text = m_secbuf_desc[FMSection::Text];
  }

  m_crodata = graph().m_crodata;
  if (m_crodata == nullptr && graph().m_bcrodata.size != 0) {
    m_mem->write(m_secbuf_desc[FMSection::Crodata]->pa, graph().m_bcrodata.va,
                 graph().m_bcrodata.size);
    m_crodata = m_secbuf_desc[FMSection::Crodata];
  }

  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  ret = setup_reuse_buffer(m_secbuf_desc[FMSection::TotalReuse]);
  if (ret != AIPU_STATUS_SUCCESS)
    return ret;

  if (m_secbuf_desc.count(FMSection::GM) != 0 &&
      m_secbuf_desc[FMSection::GM]->size != 0) {
    ret = setup_gm_buffer(m_secbuf_desc[FMSection::GM]);
    if (ret != AIPU_STATUS_SUCCESS)
      return ret;
  }

  /* v3 has no 'ZcyConst' section */
  if (!graph().m_put_weight_gm &&
      m_secbuf_desc.count(FMSection::ZcyConst) != 0 &&
      m_secbuf_desc[FMSection::ZcyConst]->size != 0) {
    auto &desc = m_secbuf_desc[FMSection::ZcyConst];
    DEV_PA_64 pa_base = desc->pa;
    for (uint32_t i = 0; i < graph().get_bss_cnt(); ++i) {
      uint32_t zcy_size = graph().get_zerocpy_const_size(i);
      BufferDesc *zcy = new BufferDesc;
      zcy->init(desc->asid_base, pa_base, zcy_size, zcy_size);
      m_weight[i].wb_zerocpy_const = zcy;
      pa_base += ALIGN_PAGE(zcy_size);
    }
    ret = graph().setup_zcy_buffer(m_weight);
    if (ret != AIPU_STATUS_SUCCESS)
      return ret;
  }

  if (m_secbuf_desc.count(FMSection::ModelParam) != 0 &&
      m_secbuf_desc[FMSection::ModelParam]->size != 0) {
    ret = setup_dyn_shape_buffer(m_secbuf_desc[FMSection::ModelParam]);
    if (ret != AIPU_STATUS_SUCCESS)
      return ret;
    m_global_param = m_secbuf_desc[FMSection::ModelParam];
  }

  if (m_secbuf_desc.count(FMSection::Rodata) != 0 &&
      m_secbuf_desc[FMSection::Rodata]->size != 0) {
    m_mem->write(m_secbuf_desc[FMSection::Rodata]->pa, graph().m_brodata.va,
                 graph().m_brodata.size);
    m_rodata = m_secbuf_desc[FMSection::Rodata];
  }

  if (m_secbuf_desc.count(FMSection::Dcr) != 0 &&
      m_secbuf_desc[FMSection::Dcr]->size != 0) {
    m_mem->write(m_secbuf_desc[FMSection::Dcr]->pa, graph().m_bdesc.va,
                 graph().m_bdesc.size);
    m_descriptor = m_secbuf_desc[FMSection::Dcr];
  }

  if (m_secbuf_desc.count(FMSection::Printf) != 0 &&
      m_secbuf_desc[FMSection::Printf]->size != 0) {
    m_pprint = m_secbuf_desc[FMSection::Printf];
  }

  m_tcbs = m_secbuf_desc[FMSection::TcbChain];
  m_mem->zeroize(m_tcbs->pa, m_tcbs->size);
  m_init_tcb.init(m_tcbs->pa);

  SubGraphTask sg_task = {0};
  for (uint32_t sg_idx = 0; sg_idx < m_sg_cnt; sg_idx++) {
    const auto &sg = graph().get_subgraph(sg_idx);
    sg_task.reset(sg_idx, sg.bss_idx);
    m_sg_job.push_back(sg_task);
  }

  if (m_secbuf_desc.count(FMSection::TotalPriv) != 0 &&
      m_secbuf_desc[FMSection::TotalPriv]->size != 0) {
    ret = setup_sg_priv_buffer(m_secbuf_desc[FMSection::TotalPriv]);
    if (ret != AIPU_STATUS_SUCCESS)
      return ret;
    m_top_priv_buf = m_secbuf_desc[FMSection::TotalPriv];
  }

  if (m_secbuf_desc.count(FMSection::Stack) != 0 ||
      m_secbuf_desc.count(FMSection::Dpdata) != 0) {
    ret = setup_sg_common_buffer(m_secbuf_desc[FMSection::Stack],
                                 m_secbuf_desc[FMSection::Dpdata]);
    if (ret != AIPU_STATUS_SUCCESS)
      return ret;
  }

  if (m_secbuf_desc.count(FMSection::Coredump) != 0 &&
      m_secbuf_desc[FMSection::Coredump]->size != 0)
    m_coredump->setup_coredump_buffer(*m_secbuf_desc[FMSection::Coredump]);

  return AIPU_STATUS_SUCCESS;
}

aipu_status_t
JobV3X::specify_io_buffer(aipu_shared_tensor_info_t &tensor_info) {
  /* shared buffer shouldn't reuse with any other reuse buffers */
  const char *str = "free_input";
  const std::vector<JobIOBuffer> *iobuffer_vec = nullptr;
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

  ret = specify_io(tensor_info, iobuffer_vec);
  if (ret != AIPU_STATUS_SUCCESS)
    return ret;

  std::vector<BufferDesc *> empty_buf;
  ret = setup_rodata_sg(
      graph().get_bss(0).param_map, m_reuses_desc,
      m_weight.size() > 0 ? m_weight[0].wb_weights : empty_buf, &m_dmabuf_idxs);
  if (ret != AIPU_STATUS_SUCCESS)
    return ret;

  return AIPU_STATUS_SUCCESS;
}

void JobV3X::free_sg_buffers(SubGraphTask &sg_task) {
  for (uint32_t i = 0; i < sg_task.reuse_priv_buffers.size(); i++)
    m_mem->free_bufferdesc(&sg_task.reuse_priv_buffers[i]);
  sg_task.reuse_priv_buffers.clear();

  for (uint32_t i = 0; i < m_sgt_allocated.size(); i++) {
    for (uint32_t j = 0; j < m_task_per_sg; j++) {
      Task *task;
      task = &m_sgt_allocated[i]->tasks[j];
      if (task->stack)
        m_mem->free_bufferdesc(&task->stack);
      if (task->private_data)
        m_mem->free_bufferdesc(&task->private_data);
    }
  }
  m_sgt_allocated.clear();
}

aipu_status_t JobV3X::init(const aipu_global_config_simulation_t *cfg,
                           const aipu_global_config_hw_t *hw_cfg) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;

  m_cfg = cfg;
  m_hw_cfg = hw_cfg;

  m_dev->get_core_count(m_partition_id, 0, &m_core_cnt);
  set_job_params(graph().get_subgraph_cnt(),
                 m_dev->tec_cnt_per_core(m_partition_id),
                 graph().get_remap_flag(), m_core_cnt);

  init_grid_id(m_grid_id);

  init_group_id(m_sg_cnt);

  ret = alloc_job_buffers();
  if (ret != AIPU_STATUS_SUCCESS)
    return ret;

  ret = setup_job_buffers();
  if (ret != AIPU_STATUS_SUCCESS)
    return ret;

  /* get IO buffer address, all subgraphs share the same copy of reuse buffers
   */
  create_io_buffers(graph().get_bss(0).io, m_reuses_desc);

  /* setup rodata & dcr, update entry for all subgraphs in global RO/DCR section
   */
  std::vector<BufferDesc *> empty_buf;
  ret =
      setup_rodata_sg(graph().get_bss(0).param_map, m_reuses_desc,
                      m_weight.size() > 0 ? m_weight[0].wb_weights : empty_buf);
  if (ret != AIPU_STATUS_SUCCESS)
    return ret;

  /* update subgraph private buffers PA in RO/DCR section */
  for (uint32_t sg_idx = 0; sg_idx < m_sg_cnt; sg_idx++) {
    std::vector<BufferDesc *> invalid_buf;
    const auto &param_map = graph().get_subgraph(sg_idx).private_buffers_map;
    ret = setup_rodata_sg(param_map, m_sg_job[sg_idx].reuse_priv_buffers,
                          invalid_buf);
    if (ret != AIPU_STATUS_SUCCESS)
      return ret;
  }

  /* 4. setup remap */
  setup_remap(*m_secbuf_desc[FMSection::Rodata], m_secbuf_desc[FMSection::Dcr]);

  /* 5. parse SegMMU config */
  ret = setup_segmmu(m_sg_job[0]);
  if (ret != AIPU_STATUS_SUCCESS)
    return ret;

  /**
   * no need to create TCBs if there's no subgraphs,
   * just directly return.
   */
  if (m_sg_cnt == 0) {
    m_status = AIPU_JOB_STATUS_INIT;
    return ret;
  }

  ret = setup_tcb_chain();
  if (ret != AIPU_STATUS_SUCCESS)
    return ret;

  if (m_backup_tcb != nullptr)
    m_mem->read(m_init_tcb.pa, m_backup_tcb.get(),
                m_tot_tcb_cnt * tcb_ctl::TCB_LEN);

  return AIPU_STATUS_SUCCESS;
}

aipu_status_t JobV3X::schedule() {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  JobDesc desc;

  ret = validate_schedule_status();
  if (ret != AIPU_STATUS_SUCCESS) {
    LOG(LOG_ERR, "Job state %d is invalid", m_status);
    return ret;
  }

  if (get_subgraph_cnt() == 0) {
    LOG(LOG_WARN, "Subgraph counter is 0, and it will not dispatch");
    m_status = AIPU_JOB_STATUS_DONE;
    return AIPU_STATUS_SUCCESS;
  }

  if (m_text->size == 0) {
    LOG(LOG_WARN, "Graph text size is 0, and it will not dispatch");
    m_status = AIPU_JOB_STATUS_DONE;
    return AIPU_STATUS_SUCCESS;
  }

  if (graph().is_dynamic_shape() && !m_dyn_data_load) {
    LOG(LOG_ERR, "graph is dynamic, but you have not set dynamic parameters");
    return AIPU_STATUS_ERROR_OP_NOT_SUPPORTED;
  }

  /* back tcbchain */
  if (m_backup_tcb != nullptr && m_backup_tcb_used == true)
    m_mem->write(m_init_tcb.pa, m_backup_tcb.get(),
                 m_tot_tcb_cnt * tcb_ctl::TCB_LEN);
  m_backup_tcb_used = true;

  dump_buffers(true /*before*/);
  ret = dump_for_emulation();
  if (ret != AIPU_STATUS_SUCCESS)
    return ret;

  memset(&desc.kdesc, 0, sizeof(desc.kdesc));

  /* for simulation */
  desc.kdesc.job_id = m_id;
  desc.kdesc.version_compatible = !graph().m_do_vcheck;
  desc.kdesc.aipu_config = graph().get_config();
  desc.jobbase = this;
  desc.tcb_number = m_tot_tcb_cnt;
  desc.tcb_head = m_init_tcb.pa;
  desc.tcb_tail =
      m_init_tcb.pa +
      (m_tot_tcb_cnt - 1) * tcb_ctl::TCB_LEN; /* last task tcb pa for sim */

  /* for HW */
  desc.kdesc.exec_flag = (m_qos == AIPU_JOB_QOS_HIGH)
                             ? AIPU_JOB_EXEC_FLAG_QOS_FAST
                             : AIPU_JOB_EXEC_FLAG_QOS_SLOW;
  desc.kdesc.core_id = 0;
  if (m_bind_enable) {
    desc.kdesc.exec_flag |= AIPU_JOB_EXEC_FLAG_BIND_DISPATCH;
    desc.kdesc.core_id = m_core_id;
  }
  desc.kdesc.enable_poll_opt = !m_hw_cfg->poll_in_commit_thread;
  desc.kdesc.profile_fd = m_profile_fd;
  if (m_profiler.size() > 0) {
    desc.kdesc.profile_pa = m_profiler[0].pa;
    desc.kdesc.profile_sz = m_profiler[0].size;
  }
  desc.kdesc.aipu_version =
      graph().bin_to_dev_isa(); /* maintain a m_dev_isa? */
  desc.kdesc.partition_id = m_partition_id;
  desc.kdesc.exec_flag |= (m_segmmu_num > 0) ? AIPU_JOB_EXEC_FLAG_SEG_MMU : 0;
  desc.kdesc.exec_flag |= (m_sg_cnt == 1) ? AIPU_JOB_EXEC_FLAG_SINGLE_GROUP
                                          : AIPU_JOB_EXEC_FLAG_MULTI_GROUP;
  desc.kdesc.asid0_base = get_asid0_base();

  desc.kdesc.head_tcb_pa = m_init_tcb.pa;
  desc.kdesc.tail_tcb_pa =
      m_init_tcb.pa + (m_tot_tcb_cnt - 1) * tcb_ctl::TCB_LEN;
  desc.kdesc.first_task_tcb_pa = get_first_task_tcb_pa();
  desc.kdesc.last_task_tcb_pa =
      m_init_tcb.pa + (m_tot_tcb_cnt - 1) * tcb_ctl::TCB_LEN;
  desc.kdesc.group_id = get_last_group_id();

  /* for debugger */
  desc.kdesc.is_defer_run = m_is_defer_run;
  desc.kdesc.do_trigger = m_do_trigger;
  desc.kdesc.is_coredump_en = false;
  if (m_coredump != nullptr && m_coredump->is_initialized())
    desc.kdesc.is_coredump_en = true;

  ret = convert_ll_status(m_dev->schedule(desc));
  if (ret != AIPU_STATUS_SUCCESS)
    return ret;

  if ((m_is_defer_run == true) && (m_do_trigger == false))
    m_status = AIPU_JOB_STATUS_BIND;
  else
    m_status = AIPU_JOB_STATUS_SCHED;

  return ret;
}

aipu_status_t JobV3X::destroy() { return free_job_buffers(); }

void JobV3X::dump_extension_buffers(bool before) {
  DEV_PA_64 dump_pa;
  uint32_t dump_size;

  if (m_dump_tcb) {
    dump_pa = m_tcbs->pa;
    dump_size = m_tot_tcb_cnt * tcb_ctl::TCB_LEN;
    if (dump_size != 0)
      dump_buffer(dump_pa, dump_size,
                  (before ? "TCB_BeforeRun" : "TCB_AfterRun"), nullptr);
  }

#ifndef SIMULATION
  /* it will pass 'm_profile_fd' to KMD for real-time profiling */
  if (m_dump_profile && m_profiler.size() > 0) {
    std::string profile_file_name =
        m_dump_dir + "/" + m_dump_misc_prefix + "_PerfData.bin";
    m_profile_fd = open(profile_file_name.c_str(), O_RDWR | O_CREAT, 0644);
    if (m_profile_fd < 0)
      LOG(LOG_ALERT, "open: %s [fail], ret: %d", profile_file_name.c_str(),
          m_profile_fd);
    else
      chmod(profile_file_name.c_str(), 0644);

    convert_ll_status(m_dev->ioctl_cmd(AIPU_IOCTL_ENABLE_TICKCOUNTER, nullptr));
  }
#endif
}

void JobV3X::dump_graphjson(const std::string &fullpath) {
  if (graph().m_bgraphjson.size == 0) {
    std::string json_data(graph().m_bgraphjson.va, graph().m_bgraphjson.size);
    std::ofstream json_file(fullpath, std::ios::out | std::ios::trunc);
    json_file.write(json_data.c_str(), json_data.size());
    json_file.close();
  }
}

aipu_status_t JobV3X::bind_core(uint32_t partition_id) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  uint32_t partition_cnt = 0;

  ret = convert_ll_status(m_dev->get_partition_count(&partition_cnt));
  if (ret != AIPU_STATUS_SUCCESS)
    return ret;

  if (partition_id >= partition_cnt)
    return AIPU_STATUS_ERROR_INVALID_PARTITION_ID;

  ret = validate_schedule_status();
  if (ret != AIPU_STATUS_SUCCESS)
    return ret;

  m_is_defer_run = true;
  m_do_trigger = false;
  m_partition_id = partition_id;
  ret = schedule();

  return ret;
}

aipu_status_t JobV3X::debugger_run() {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  aipu_job_status_t status = AIPU_JOB_STATUS_NO_STATUS;

  if (m_status != AIPU_JOB_STATUS_BIND)
    return AIPU_STATUS_ERROR_INVALID_OP;

  m_is_defer_run = true;
  m_do_trigger = true;
  ret = schedule();
  if (ret != AIPU_STATUS_SUCCESS)
    return ret;

  ret = get_status_blocking(&status, -1);
  if ((AIPU_STATUS_SUCCESS == ret) && (AIPU_JOB_STATUS_DONE != status))
    ret = AIPU_STATUS_ERROR_JOB_EXCEPTION;

  return ret;
}

aipu_status_t JobV3X::parse_dynamic_out_shape() {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  uint64_t size = 0;
  uint32_t data[96] = {0};

  if (graph().is_dynamic_shape() && m_dyn_shape->is_set_dyn_shape_true() &&
      (m_dyn_shape->get_config_shape_sz() == m_inputs.size())) {
    if (m_outputs_shape.size() != m_outputs.size()) {
      ret = AIPU_STATUS_ERROR_UNMATCH_OUT_SHAPE;
      LOG(LOG_ERR, "DS out tensor cnt != Original out tensor cnt");
      goto out;
    }

    for (uint32_t i = 0; i < m_outputs_shape.size(); i++) {
      m_mem->read(m_outputs_shape[i].pa, (char *)data, m_outputs_shape[i].size);
      size = 1;
      for (uint32_t j = 0; j < m_outputs_shape[i].size / 4; j++) {
        LOG(LOG_INFO, "output idx: %u, shape idx: %u, dim: %u", i, j, data[j]);
        size *= data[j];
      }

      if (size == 0) {
        m_dyn_shape->clear_config_out_tensor_size();
        ret = AIPU_STATUS_ERROR_ZERO_TENSOR_SIZE;
        LOG(LOG_ERR, "Invalid dynamic out shape %d: size (0)", i);
        goto out;
      }

      if (m_outputs[i].type == AIPU_DATA_TYPE_U16 ||
          m_outputs[i].type == AIPU_DATA_TYPE_S16 ||
          m_outputs[i].type == AIPU_DATA_TYPE_F16 ||
          m_outputs[i].type == AIPU_DATA_TYPE_BF16)
        size <<= 1;
      else if (m_outputs[i].type == AIPU_DATA_TYPE_U32 ||
               m_outputs[i].type == AIPU_DATA_TYPE_S32 ||
               m_outputs[i].type == AIPU_DATA_TYPE_F32)
        size <<= 2;

      m_dyn_shape->set_config_out_tensor_size(i, size);
    }

    m_dyn_shape->update_dynamic_io_tensor_size(AIPU_TENSOR_TYPE_OUTPUT);
    update_single_io_buffers(graph().get_bss(0).io.outputs, m_outputs,
                             m_reuses_desc);
  }

out:
  return ret;
}

aipu_status_t JobV3X::dump_for_emulation() {
  if (m_dump_emu == false)
    return AIPU_STATUS_SUCCESS;

  /* dump runtime.cfg */
  std::string runtime_cfg = m_dump_dir + "/runtime.cfg";
  static const std::map<uint32_t, std::string> gm_info = {
      {0, "0K"},         {512 << 10, "512K"}, {1 << 20, "1M"},
      {2 << 20, "2M"},   {4 << 20, "4M"},     {8 << 20, "8M"},
      {16 << 20, "16M"}, {32 << 20, "32M"},   {64 << 20, "64M"},
  };

  FileWrapper ofs(runtime_cfg, std::ios_base::in | std::ios_base::out |
                                   std::ios_base::trunc);
  if (!ofs.is_open())
    return AIPU_STATUS_ERROR_OPEN_FILE_FAIL;

  ofs << "[COMMON]\n";

  /* runtime.cfg: config */
  ofs << "#configuration X3P/S K:cluster,C:core,A:aiff,T:tec \n";
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
  ofs << "#GM support: 0KB, 512KiB,1MiB,2MiB,4MiB,8MiB,16MiB,32MiB,64MiB.\n";
  if (gm_info.count(m_cfg->gm_size) == 1)
    ofs << "GM_SIZE=" << gm_info.at(m_cfg->gm_size) << "\n";

  if (m_cfg->plugin_name != nullptr) {
    ofs << "#PLUGIN_FILENAME\n";
    ofs << "PLUGIN_FILENAME=" << m_cfg->plugin_name << "\n";
  }

  /* runtime.cfg: profile */
  if (m_dev->get_profile_en()) {
    ofs << "\n[PROFILE]\n";
    ofs << "#perf mode: 0:None,1:fast mode,2:eval,3:idu,4:probe\n";
    if (graph().get_isa() == ISAv5)
      ofs << "MODE=2\n";
    else {
      ofs << "MODE=1\n";
      ofs << "SYS_FREQ_MHZ=" << m_cfg->freq_mhz << "\n";
      ofs << "DDR_RD_LATENCY=" << m_cfg->ddr_latency_rd << "\n";
      ofs << "DDR_WR_LATENCY=" << m_cfg->ddr_latency_wr << "\n";
      ofs << "DDR_BW_BITS=" << m_cfg->ddr_bw << "\n";

      if (m_cfg->perf_report != nullptr)
        ofs << "REPORT_FILENAME=" << m_cfg->perf_report << "\n";
    }

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

  /* 3: text,ro,tcb */
  int emu_input_cnt = 3 + m_inputs.size() + (m_descriptor != nullptr ? 1 : 0);
  int emu_output_cnt = m_outputs.size();
  /* runtime.cfg: [INPUT] */
  for (uint32_t bss_id = 0; bss_id < m_weight.size(); bss_id++) {
    if (m_weight[bss_id].wb_weight != nullptr &&
        m_weight[bss_id].wb_weight->size > 0) {
      emu_input_cnt += 1;
      if (m_weight[bss_id].wb_zerocpy_const != nullptr &&
          m_weight[bss_id].wb_zerocpy_const->size != 0)
        emu_input_cnt += 1;
    } else
      emu_input_cnt += m_weight[bss_id].wb_weights.size();
  }

  ofs << "[INPUT]\n";
  ofs << "COUNT=" << emu_input_cnt << "\n";

  int file_id = -1;
  DEV_PA_64 dump_pa = 0;
  uint32_t dump_size = 0;
  char dump_name[256] = {0};

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

  /* dump crodata */
  if (m_crodata != nullptr && m_crodata->req_size != 0) {
    dump_pa = m_crodata->pa;
    dump_size = m_crodata->req_size;
    snprintf(dump_name, 128, "%s/%s.crodata", m_dump_dir.c_str(),
             m_dump_prefix.c_str());
    m_mem->dump_file(dump_pa, dump_name, dump_size);

    ofs << "FILE" << std::dec << ++file_id << "=" << m_dump_prefix
        << ".crodata\n";
    ofs << "BASE" << file_id << "=0x" << std::hex << dump_pa << "\n";
    m_dumpcfg_input.push_back({dump_name, dump_pa});
  }

  /* dump temp.weight */
  for (uint32_t bss_id = 0; bss_id < m_weight.size(); bss_id++) {
    if (m_weight[bss_id].wb_weight != nullptr &&
        m_weight[bss_id].wb_weight->req_size > 0) {
      dump_pa = m_weight[bss_id].wb_weight->pa;
      dump_size = m_weight[bss_id].wb_weight->req_size;

      snprintf(dump_name, 128, "%s/%s.weight%d", m_dump_dir.c_str(),
               m_dump_prefix.c_str(), bss_id);
      m_mem->dump_file(dump_pa, dump_name, dump_size);

      ofs << "FILE" << std::dec << ++file_id << "=" << m_dump_prefix
          << ".weight" << bss_id << "\n";
      ofs << "BASE" << file_id << "=0x" << std::hex << dump_pa << "\n";
      m_dumpcfg_input.push_back({dump_name, dump_pa});

      if (m_weight[bss_id].wb_zerocpy_const != nullptr &&
          m_weight[bss_id].wb_zerocpy_const->size > 0) {
        dump_pa = m_weight[bss_id].wb_zerocpy_const->pa;
        dump_size = m_weight[bss_id].wb_zerocpy_const->req_size;
        snprintf(dump_name, 128, "%s/%s.zerocpy_const%d", m_dump_dir.c_str(),
                 m_dump_prefix.c_str(), bss_id);
        m_mem->dump_file(dump_pa, dump_name, dump_size);

        ofs << "FILE" << std::dec << ++file_id << "=" << m_dump_prefix
            << ".zerocpy_const" << bss_id << "\n";
        ofs << "BASE" << file_id << "=0x" << std::hex << dump_pa << "\n";
        m_dumpcfg_input.push_back({dump_name, dump_pa});
      }
    } else {
      for (uint32_t i = 0; i < m_weight[bss_id].wb_weights.size(); i++) {
        DumpcfgInputDesc input_desc;
        std::string name;

        dump_pa = m_weight[bss_id].wb_weights[i]->pa;
        dump_size = m_weight[bss_id].wb_weights[i]->size;
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
  if (m_descriptor != nullptr && m_descriptor->req_size != 0) {
    dump_pa = m_descriptor->pa;
    dump_size = m_descriptor->req_size;
    snprintf(dump_name, 128, "%s/%s.dcr", m_dump_dir.c_str(),
             m_dump_prefix.c_str());
    m_mem->dump_file(dump_pa, dump_name, dump_size);

    ofs << "FILE" << std::dec << ++file_id << "=" << m_dump_prefix << ".dcr\n";
    ofs << "BASE" << file_id << "=0x" << std::hex << dump_pa << "\n";
    m_dumpcfg_input.push_back({dump_name, dump_pa});
  }

  /* dump global_model_param */
  if (m_global_param != nullptr && m_global_param->req_size) {
    dump_pa = m_global_param->pa;
    dump_size = m_global_param->req_size;
    snprintf(dump_name, 128, "%s/%s.global_param", m_dump_dir.c_str(),
             m_dump_prefix.c_str());
    m_mem->dump_file(dump_pa, dump_name, dump_size);

    ofs << "FILE" << std::dec << ++file_id << "=" << m_dump_prefix
        << ".global_param\n";
    ofs << "BASE" << file_id << "=0x" << std::hex << dump_pa << "\n";
    m_dumpcfg_input.push_back({dump_name, dump_pa});
  }

  /* dump temp.tcb */
  dump_pa = m_tcbs->pa;
  dump_size = m_tot_tcb_cnt * tcb_ctl::TCB_LEN;
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

    dump_buffer(m_inputs[i], dump_name, true);

    ofs << "FILE" << std::dec << ++file_id << "=" << m_dump_prefix << ".input"
        << i << "\n";
    ofs << "BASE" << file_id << "=0x" << std::hex << dump_pa << "\n";
    m_dumpcfg_input.push_back({dump_name, dump_pa});
  }
  ofs << "\n";

  ofs << "[HOST]\n";
  ofs << "TCBP_HI=0x" << std::hex << get_high_32(m_init_tcb.pa) << "\n";
  ofs << "TCBP_LO=0x" << get_low_32(m_init_tcb.pa) << "\n";
  if (graph().get_isa() != ISAv5)
    ofs << "TCB_NUM=0x" << std::hex << m_tot_tcb_cnt << "\n";
  m_dumpcfg_host = {m_partition_id, get_high_32(m_init_tcb.pa),
                    get_low_32(m_init_tcb.pa)};
  ofs << "\n";

  /* runtime.cfg: [OUTPUT] */
  ofs << "[OUTPUT]\n";
  ofs << "COUNT=" << std::dec << emu_output_cnt << "\n";

  /* dump output.bin[n] */
  bool default_output_prefix = true;
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
      if (i == 0)
        ofs << "FILE" << std::dec << i << "=" << m_dump_output_prefix << "\n";
      else
        ofs << "FILE" << std::dec << i << "=" << m_dump_output_prefix << i
            << "\n";
    }

    ofs << "BASE" << std::dec << i << "=0x" << std::hex << dump_pa << "\n";
    ofs << "SIZE" << std::dec << i << "=0x" << std::hex << dump_size << "\n";
  }

  /* close runtime.cfg */
  ofs.close();

  /* dump metadata.txt */
  std::string metadata_txt = m_dump_dir + "/metadata.txt";
  dump_emu_metadata(metadata_txt);

  return AIPU_STATUS_SUCCESS;
}
} // namespace aipudrv